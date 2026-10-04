/*
 * Talkman CCI slave scanner.
 *
 * WOA does not publish rear/front/OIS 7-bit addresses: ACPI CAMP/CAMS
 * have no I2C SerialBus, ARM64 sensor .sys files are 8 KB stubs, ARM32
 * SMIApp takes i2c_addr at runtime, DCC 104541* has no slave-addr field.
 * Power WOA-known rails/clocks/GPIOs, then read SMIA 0x0000 and Sony 0x0016.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/uidgid.h>
#include <linux/pinctrl/consumer.h>
#include "msm_cci.h"
#include "msm_camera_i2c.h"
#include "msm_camera_io_util.h"

#define DRV_NAME		"talkman-cci-scan"
/* Android AID_SHELL (not a CCI slave-id). debugfs scan: 0660 root:shell. */
#define TALKMAN_CCI_SCAN_GID	2000
#define SMIA_MODEL_ID_REG	0x0000
#define SONY_CHIP_ID_REG	0x0016
#define SCAN_BUF_SZ		4096
#define SID_MIN			0x08
#define SID_MAX			0x77

struct talkman_cci_scan {
	struct device *dev;
	struct regulator *vana;
	struct regulator *vaf;
	struct regulator *vdig;
	struct regulator *vio;
	struct regulator *vana_front;
	struct clk *mclk0_src;
	struct clk *mclk0;
	struct clk *mclk2_src;
	struct clk *mclk2;
	u32 mclk_rate;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_scan;
	int front_rst_gpio;		/* CAM_FRONT_RES_N GPIO 104, -1 if absent */
	struct dentry *dbg;
	struct mutex lock;
	char *buf;
	size_t buf_len;
	/*
	 * "power" debugfs: keep one camera powered with CCI initialised so
	 * "i2c" can do raw reads/writes (BU24210 @ 0x7c probing, front
	 * bring-up) without re-powering per transaction.
	 */
	int held;			/* 0 off, 1 rear, 2 front */
	int held_inits;
	struct msm_camera_cci_client *held_cci;
	struct msm_camera_i2c_client held_client;
};

static int scan_append(struct talkman_cci_scan *s, const char *fmt, ...)
{
	va_list ap;
	int n;

	if (s->buf_len >= SCAN_BUF_SZ - 1)
		return -ENOSPC;
	va_start(ap, fmt);
	n = vsnprintf(s->buf + s->buf_len, SCAN_BUF_SZ - s->buf_len, fmt, ap);
	va_end(ap);
	if (n < 0)
		return n;
	if (s->buf_len + n >= SCAN_BUF_SZ)
		s->buf_len = SCAN_BUF_SZ - 1;
	else
		s->buf_len += n;
	return 0;
}

static int scan_power_rail(struct regulator *r, bool on)
{
	int rc;

	if (IS_ERR_OR_NULL(r))
		return 0;
	if (on) {
		rc = regulator_enable(r);
		if (!rc)
			pr_info("%s: enable volt=%d is_en=%d\n", __func__,
				regulator_get_voltage(r),
				regulator_is_enabled(r));
		return rc;
	}
	return regulator_disable(r);
}

static int scan_clk_on(struct clk *src, struct clk *clk, u32 rate)
{
	int rc;

	if (IS_ERR_OR_NULL(src) || IS_ERR_OR_NULL(clk))
		return -ENODEV;
	if (rate) {
		rc = clk_set_rate(src, rate);
		if (rc)
			return rc;
	}
	rc = clk_prepare_enable(src);
	if (rc)
		return rc;
	rc = clk_prepare_enable(clk);
	if (rc) {
		clk_disable_unprepare(src);
		return rc;
	}
	pr_info("%s: mclk req=%u src_get=%lu clk_get=%lu\n",
		__func__, rate, clk_get_rate(src), clk_get_rate(clk));
	msm_cam_dump_mclk0_pad("cci_scan_mclk", 1);
	return 0;
}

static void scan_clk_off(struct clk *src, struct clk *clk)
{
	if (!IS_ERR_OR_NULL(clk))
		clk_disable_unprepare(clk);
	if (!IS_ERR_OR_NULL(src))
		clk_disable_unprepare(src);
}

static int scan_pins(struct talkman_cci_scan *s, bool scan)
{
	struct pinctrl_state *st;

	if (IS_ERR_OR_NULL(s->pinctrl))
		return 0;
	st = scan ? s->pins_scan : s->pins_default;
	if (IS_ERR_OR_NULL(st))
		return 0;
	return pinctrl_select_state(s->pinctrl, st);
}

static int scan_power_rear(struct talkman_cci_scan *s, bool on)
{
	int rc;

	if (!on) {
		scan_clk_off(s->mclk0_src, s->mclk0);
		scan_power_rail(s->vio, false);
		scan_power_rail(s->vaf, false);
		scan_power_rail(s->vana, false);
		scan_power_rail(s->vdig, false);
		return 0;
	}

	rc = scan_power_rail(s->vdig, true);
	if (rc)
		return rc;
	rc = scan_power_rail(s->vana, true);
	if (rc)
		goto err_vdig;
	rc = scan_power_rail(s->vaf, true);
	if (rc)
		goto err_vana;
	rc = scan_power_rail(s->vio, true);
	if (rc)
		goto err_vaf;
	usleep_range(1000, 1500);
	rc = scan_clk_on(s->mclk0_src, s->mclk0, s->mclk_rate);
	if (rc)
		goto err_vio;
	msleep(25);
	return 0;

err_vio:
	scan_power_rail(s->vio, false);
err_vaf:
	scan_power_rail(s->vaf, false);
err_vana:
	scan_power_rail(s->vana, false);
err_vdig:
	scan_power_rail(s->vdig, false);
	return rc;
}

static int scan_power_front(struct talkman_cci_scan *s, bool on)
{
	int rc;

	if (!on) {
		if (gpio_is_valid(s->front_rst_gpio))
			gpio_set_value(s->front_rst_gpio, 0);
		scan_clk_off(s->mclk2_src, s->mclk2);
		scan_power_rail(s->vio, false);
		scan_power_rail(s->vana_front, false);
		return 0;
	}

	rc = scan_power_rail(s->vana_front, true);
	if (rc)
		return rc;
	rc = scan_power_rail(s->vio, true);
	if (rc)
		goto err_vana;
	usleep_range(1000, 1500);
	rc = scan_clk_on(s->mclk2_src, s->mclk2, s->mclk_rate);
	if (rc)
		goto err_vio;
	/*
	 * CAM_FRONT_RES_N (schematic GPIO 104) is active-low. A SMIA sensor
	 * held in reset never ACKs, so release it after rails + MCLK and
	 * give the sensor its power-on settle before the SID sweep.
	 */
	if (gpio_is_valid(s->front_rst_gpio)) {
		gpio_set_value(s->front_rst_gpio, 1);
		pr_info("%s: front reset GPIO %d released\n", DRV_NAME,
			s->front_rst_gpio);
	}
	msleep(10);
	return 0;

err_vio:
	scan_power_rail(s->vio, false);
err_vana:
	scan_power_rail(s->vana_front, false);
	return rc;
}

static int scan_one_sid(struct talkman_cci_scan *s,
			struct msm_camera_i2c_client *client,
			u16 sid, const char *bus)
{
	uint16_t model = 0, sony = 0;
	int rc;

	client->cci_client->sid = sid;
	rc = msm_camera_cci_i2c_read(client, SMIA_MODEL_ID_REG, &model,
				     MSM_CAMERA_I2C_WORD_DATA);
	if (rc)
		return rc;
	sony = 0;
	msm_camera_cci_i2c_read(client, SONY_CHIP_ID_REG, &sony,
				MSM_CAMERA_I2C_WORD_DATA);
	pr_info("%s: ACK %s sid=0x%02x wr=0x%02x SMIA 0x0000=0x%04x Sony 0x0016=0x%04x%s\n",
		DRV_NAME, bus, sid, sid << 1, model, sony,
		(model == 0x0230 || sony == 0x0230) ? " (IMX230)" :
		(model == 0x0278 || sony == 0x0278) ? " (IMX278)" : "");
	scan_append(s, "ACK %s sid=0x%02x wr=0x%02x model=0x%04x chip=0x%04x\n",
		    bus, sid, sid << 1, model, sony);
	return 0;
}

static int scan_master(struct talkman_cci_scan *s,
		       struct msm_camera_i2c_client *client,
		       enum cci_i2c_master_t master, const char *bus)
{
	u16 sid;
	int found = 0;

	client->cci_client->cci_i2c_master = master;
	for (sid = SID_MIN; sid <= SID_MAX; sid++) {
		if (!scan_one_sid(s, client, sid, bus))
			found++;
	}
	if (!found) {
		pr_info("%s: no ACK on %s (sid 0x%02x-0x%02x)\n",
			DRV_NAME, bus, SID_MIN, SID_MAX);
		scan_append(s, "no ACK on %s\n", bus);
	}
	return found;
}

/*
 * Allocate a CCI client and INIT both masters. Returns the number of
 * successful INITs (each needs a matching MSM_CCI_RELEASE) or -errno.
 */
static int scan_cci_init(struct talkman_cci_scan *s,
			 struct msm_camera_i2c_client *client,
			 struct msm_camera_cci_client **out)
{
	struct msm_camera_cci_client *cci_client;
	struct v4l2_subdev *sd;
	int rc, inited = 0;

	sd = msm_cci_get_subdev();
	if (!sd) {
		pr_err("%s: CCI subdev not ready\n", DRV_NAME);
		scan_append(s, "CCI subdev not ready\n");
		return -ENODEV;
	}

	cci_client = kzalloc(sizeof(*cci_client), GFP_KERNEL);
	if (!cci_client)
		return -ENOMEM;

	cci_client->cci_subdev = sd;
	cci_client->cci_i2c_master = MASTER_1;
	cci_client->sid = 0;
	cci_client->retries = 0;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;
	memset(client, 0, sizeof(*client));
	client->cci_client = cci_client;
	client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	/*
	 * MSM_CCI_INIT programs SCL/SDA timing only for cci_i2c_master,
	 * and only after the global reset on the first INIT. Rear I2C is
	 * schematic CCI1 (GPIO 19/20), so INIT MASTER_1 first. A
	 * MASTER_0-only INIT left MASTER_1 unprogrammed: each SID then
	 * hit CCI_TIMEOUT ("MASTER_1 error 0x40000000") instead of a NACK.
	 * Second INIT (ref_count++) sets MASTER_0 clk params.
	 */
	cci_client->cci_i2c_master = MASTER_1;
	rc = msm_sensor_cci_i2c_util(client, MSM_CCI_INIT);
	if (rc) {
		pr_err("%s: CCI INIT master1 failed rc=%d\n", DRV_NAME, rc);
		scan_append(s, "CCI INIT master1 failed %d\n", rc);
		kfree(cci_client);
		client->cci_client = NULL;
		return rc;
	}
	inited++;

	cci_client->cci_i2c_master = MASTER_0;
	rc = msm_sensor_cci_i2c_util(client, MSM_CCI_INIT);
	if (rc)
		pr_err("%s: CCI INIT master0 rc=%d\n", DRV_NAME, rc);
	else
		inited++;

	*out = cci_client;
	return inited;
}

static void scan_cci_release(struct msm_camera_i2c_client *client,
			     struct msm_camera_cci_client *cci_client,
			     int inited)
{
	while (inited-- > 0)
		msm_sensor_cci_i2c_util(client, MSM_CCI_RELEASE);
	kfree(cci_client);
	client->cci_client = NULL;
}

static int talkman_cci_do_scan(struct talkman_cci_scan *s)
{
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_camera_i2c_client client;
	int rc, found = 0, inited;

	s->buf_len = 0;
	s->buf[0] = '\0';

	if (s->held) {
		scan_append(s, "power is held (%d); echo off > power first\n",
			    s->held);
		return -EBUSY;
	}

	inited = scan_cci_init(s, &client, &cci_client);
	if (inited < 0)
		return inited;

	scan_pins(s, true);

	rc = scan_power_rear(s, true);
	if (rc) {
		pr_err("%s: rear power-up rc=%d\n", DRV_NAME, rc);
		scan_append(s, "rear power-up failed %d\n", rc);
	} else {
		scan_append(s,
			    "rear CSI0 mclk0 24MHz GPIO13 CCI1 GPIO19/20 L25/L29/L23:\n");
		found += scan_master(s, &client, MASTER_0, "cci0-master0-rear");
		found += scan_master(s, &client, MASTER_1, "cci0-master1-rear");
		scan_power_rear(s, false);
	}

	rc = scan_power_front(s, true);
	if (rc) {
		pr_err("%s: front power-up rc=%d\n", DRV_NAME, rc);
		scan_append(s, "front power-up failed %d\n", rc);
	} else {
		scan_append(s, "front CSI2 mclk2 GPIO15 L17:\n");
		found += scan_master(s, &client, MASTER_0,
				     "cci0-master0-front");
		found += scan_master(s, &client, MASTER_1,
				     "cci0-master1-front");
		scan_power_front(s, false);
	}

	scan_pins(s, false);
	scan_cci_release(&client, cci_client, inited);
	scan_append(s, "done, %d ACK(s)\n", found);
	pr_info("%s: finished, %d ACK(s). Read debugfs for the table.\n",
		DRV_NAME, found);
	return 0;
}

static ssize_t scan_read(struct file *file, char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	struct talkman_cci_scan *s = file->private_data;
	ssize_t rc;

	mutex_lock(&s->lock);
	rc = simple_read_from_buffer(ubuf, count, ppos, s->buf, s->buf_len);
	mutex_unlock(&s->lock);
	return rc;
}

static ssize_t scan_write(struct file *file, const char __user *ubuf,
			  size_t count, loff_t *ppos)
{
	struct talkman_cci_scan *s = file->private_data;

	mutex_lock(&s->lock);
	talkman_cci_do_scan(s);
	mutex_unlock(&s->lock);
	return count;
}

static const struct file_operations scan_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = scan_read,
	.write = scan_write,
	.llseek = default_llseek,
};

/* ---- held power + raw CCI access ---------------------------------- */

static void scan_hold_off(struct talkman_cci_scan *s)
{
	if (!s->held)
		return;
	if (s->held == 1)
		scan_power_rear(s, false);
	else
		scan_power_front(s, false);
	scan_pins(s, false);
	scan_cci_release(&s->held_client, s->held_cci, s->held_inits);
	s->held_cci = NULL;
	s->held_inits = 0;
	s->held = 0;
	pr_info("%s: power off\n", DRV_NAME);
	scan_append(s, "power off\n");
}

static int scan_hold_on(struct talkman_cci_scan *s, int which)
{
	int rc;

	if (s->held)
		scan_hold_off(s);

	rc = scan_cci_init(s, &s->held_client, &s->held_cci);
	if (rc < 0)
		return rc;
	s->held_inits = rc;
	scan_pins(s, true);
	rc = which == 1 ? scan_power_rear(s, true) : scan_power_front(s, true);
	if (rc) {
		pr_err("%s: %s power-up rc=%d\n", DRV_NAME,
		       which == 1 ? "rear" : "front", rc);
		scan_append(s, "%s power-up failed %d\n",
			    which == 1 ? "rear" : "front", rc);
		scan_pins(s, false);
		scan_cci_release(&s->held_client, s->held_cci, s->held_inits);
		s->held_cci = NULL;
		s->held_inits = 0;
		return rc;
	}
	s->held = which;
	pr_info("%s: power held: %s\n", DRV_NAME,
		which == 1 ? "rear (L25/L29/L23 VAF, mclk0)" :
			     "front (L17, mclk2, GPIO104 released)");
	scan_append(s, "power held: %s\n", which == 1 ? "rear" : "front");
	return 0;
}

static ssize_t power_write(struct file *file, const char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct talkman_cci_scan *s = file->private_data;
	char cmd[16];
	int rc = 0;

	if (count >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, ubuf, count))
		return -EFAULT;
	cmd[count] = '\0';
	strim(cmd);

	mutex_lock(&s->lock);
	s->buf_len = 0;
	s->buf[0] = '\0';
	if (!strcmp(cmd, "rear") || !strcmp(cmd, "1"))
		rc = scan_hold_on(s, 1);
	else if (!strcmp(cmd, "front") || !strcmp(cmd, "2"))
		rc = scan_hold_on(s, 2);
	else if (!strcmp(cmd, "off") || !strcmp(cmd, "0"))
		scan_hold_off(s);
	else
		rc = -EINVAL;
	mutex_unlock(&s->lock);
	return rc ? rc : count;
}

static const struct file_operations power_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = scan_read,
	.write = power_write,
	.llseek = default_llseek,
};

/*
 * i2c: "r <master> <sid7> <reg> [alen] [dlen]"
 *      "w <master> <sid7> <reg> <val> [alen] [dlen]"
 * master 0/1, sid7 is the 7-bit address (BU24210 write 0x7c -> 0x3e),
 * alen/dlen 1 or 2 bytes (default 2/1). Needs "power" held first.
 * Result is appended to the shared buffer and printed to dmesg.
 */
static ssize_t i2c_write(struct file *file, const char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	struct talkman_cci_scan *s = file->private_data;
	char cmd[96], op;
	unsigned int master, sid, reg, val = 0, alen = 2, dlen = 1;
	uint16_t data = 0;
	int n, rc;

	if (count >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, ubuf, count))
		return -EFAULT;
	cmd[count] = '\0';

	/* "recover <master>": clock a stuck slave off the bus via TLMM.
	 * CCI0 is GPIO 17/18, CCI1 (rear IMX230 + BU24210) is GPIO 19/20.
	 * Only while no power is held, so the CCI block is released. */
	if (sscanf(cmd, "recover %u", &master) == 1) {
		int lvl;

		if (master > 1)
			return -EINVAL;
		mutex_lock(&s->lock);
		s->buf_len = 0;
		s->buf[0] = '\0';
		if (s->held) {
			scan_append(s, "recover: echo off > power first\n");
			mutex_unlock(&s->lock);
			return -EBUSY;
		}
		lvl = msm_cam_talkman_i2c_bus_recover(master ? 19 : 17,
						      master ? 20 : 18);
		scan_append(s, "recover m%u: SDA after=%d (1 = released)\n",
			    master, lvl);
		mutex_unlock(&s->lock);
		return lvl < 0 ? lvl : count;
	}

	if (sscanf(cmd, "%c", &op) != 1)
		return -EINVAL;
	if (op == 'r') {
		n = sscanf(cmd, "%c %i %i %i %i %i",
			   &op, &master, &sid, &reg, &alen, &dlen);
		if (n < 4)
			return -EINVAL;
	} else if (op == 'w') {
		n = sscanf(cmd, "%c %i %i %i %i %i %i",
			   &op, &master, &sid, &reg, &val, &alen, &dlen);
		if (n < 5)
			return -EINVAL;
	} else {
		return -EINVAL;
	}
	if (master > 1 || sid > 0x7f ||
	    (alen != 1 && alen != 2) || (dlen != 1 && dlen != 2))
		return -EINVAL;

	mutex_lock(&s->lock);
	if (!s->held || !s->held_client.cci_client) {
		scan_append(s, "power not held; echo rear > power first\n");
		mutex_unlock(&s->lock);
		return -ENXIO;
	}
	s->held_cci->cci_i2c_master = master ? MASTER_1 : MASTER_0;
	s->held_cci->sid = sid;
	s->held_client.addr_type = alen == 1 ? MSM_CAMERA_I2C_BYTE_ADDR :
					       MSM_CAMERA_I2C_WORD_ADDR;
	if (op == 'r') {
		rc = msm_camera_cci_i2c_read(&s->held_client, reg, &data,
					     dlen == 1 ? MSM_CAMERA_I2C_BYTE_DATA :
							 MSM_CAMERA_I2C_WORD_DATA);
		pr_info("%s: i2c r m%u sid=0x%02x reg=0x%04x -> 0x%0*x rc=%d\n",
			DRV_NAME, master, sid, reg, dlen * 2, data, rc);
		scan_append(s, "r m%u sid=0x%02x reg=0x%04x -> 0x%0*x rc=%d\n",
			    master, sid, reg, dlen * 2, data, rc);
	} else {
		rc = msm_camera_cci_i2c_write(&s->held_client, reg, val,
					      dlen == 1 ? MSM_CAMERA_I2C_BYTE_DATA :
							  MSM_CAMERA_I2C_WORD_DATA);
		pr_info("%s: i2c w m%u sid=0x%02x reg=0x%04x <- 0x%0*x rc=%d\n",
			DRV_NAME, master, sid, reg, dlen * 2, val, rc);
		scan_append(s, "w m%u sid=0x%02x reg=0x%04x <- 0x%0*x rc=%d\n",
			    master, sid, reg, dlen * 2, val, rc);
	}
	mutex_unlock(&s->lock);
	return rc ? rc : count;
}

static const struct file_operations i2c_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = scan_read,
	.write = i2c_write,
	.llseek = default_llseek,
};

/* ---- BU24210 OIS/AF companion (CCI1 write 0x7c) lab commands --------
 *
 * Protocol recovered from the WOA rear sensor driver
 * (qccamrearsensor_primarySMIApp8992.sys, out/qa-ois-re/BU24210-PROTOCOL.md),
 * not from the LG BU24205/35 GPL drivers, whose 0x60xx/0xF0xx map reads
 * 0xFF on this chip. Everything is slave 0x7c (7-bit 0x3e), 16-bit
 * register, 8-bit data:
 *   0x0580  DTI capability (bit3: poll status before every page)
 *   0x0581  DTI control   (0x03 = start, 0x04 = end)
 *   0x0582  DTI status    (bit1 ready, bit2 corrupted, bit3 improper use)
 *   0x0583  page number   (0..127, 64 bytes each)
 *   0x0584.. 64-byte data window (consecutive registers)
 *   0x0524  != 0 when the firmware has come up
 *   0x0520/21/23/25, 0x0550..0x0559  mode/init registers (values from DCC)
 * The .kar in /vendor/firmware is the DCC DTI element: 22-byte header
 * (+4 dev, +5 reg bits, +6 ctrl, +8 data, +10 n_elem, +20 fw size) then
 * the image, written whole, page 0 first.
 * AF is not a 0x7c register: the driver reads the sensor's SMIA++ actuator
 * descriptor (0x1B42 device, 0x1B44 focus register) and writes
 * pos>>8, pos, 0x02, 0x01 to reg..reg+3 on that device.
 */
#define BU24210_SID		0x3e
#define BU24210_MASTER		MASTER_1
#define IMX230_SID		0x10
#define BU24210_KAR_HDR		22
#define DTI_CAP			0x0580
#define DTI_CTRL		0x0581
#define DTI_STATUS		0x0582
#define DTI_PAGE		0x0583
#define DTI_DATA		0x0584
#define DTI_PAGE_SIZE		64
#define OIS_MODE		0x0520
#define OIS_READY		0x0524
#define SMIA_ACT_CAP		0x1B04
#define SMIA_ACT_TYPE		0x1B40
#define SMIA_AF_DEV		0x1B42
#define SMIA_FOCUS_REG		0x1B44
#define SMIA_DTI_CTRL		0x0A00
#define SMIA_DTI_STATUS		0x0A01
#define SMIA_DTI_PAGE		0x0A02
#define SMIA_DTI_DATA		0x0A04

/* Manual AF proxy override (afdev). 0 = use the sensor descriptor. */
static unsigned int ois_af_sid7, ois_af_reg;

static struct msm_camera_i2c_client *lab_client(struct talkman_cci_scan *s,
						unsigned int sid7,
						enum msm_camera_i2c_reg_addr_type at)
{
	if (s->held != 1 || !s->held_client.cci_client)
		return NULL;
	s->held_cci->cci_i2c_master = BU24210_MASTER;
	s->held_cci->sid = sid7;
	s->held_client.addr_type = at;
	return &s->held_client;
}

static struct msm_camera_i2c_client *ois_client(struct talkman_cci_scan *s)
{
	return lab_client(s, BU24210_SID, MSM_CAMERA_I2C_WORD_ADDR);
}

static struct msm_camera_i2c_client *sensor_client(struct talkman_cci_scan *s)
{
	return lab_client(s, IMX230_SID, MSM_CAMERA_I2C_WORD_ADDR);
}

static int lab_w8(struct msm_camera_i2c_client *c, u16 reg, u8 val)
{
	return msm_camera_cci_i2c_write(c, reg, val, MSM_CAMERA_I2C_BYTE_DATA);
}

static int lab_r8(struct msm_camera_i2c_client *c, u16 reg, u8 *val)
{
	uint16_t d = 0;
	int rc = msm_camera_cci_i2c_read(c, reg, &d, MSM_CAMERA_I2C_BYTE_DATA);

	*val = d & 0xff;
	return rc;
}

static int lab_r16(struct msm_camera_i2c_client *c, u16 reg, u16 *val)
{
	uint16_t d = 0;
	int rc = msm_camera_cci_i2c_read(c, reg, &d, MSM_CAMERA_I2C_WORD_DATA);

	*val = d;
	return rc;
}

/* cam_drv_SMIApp_DTI_poll_till_ready: 0x0582 bit1, 200 x 1 ms */
static int dti_poll_ready(struct talkman_cci_scan *s,
			  struct msm_camera_i2c_client *c, const char *what)
{
	u8 st = 0;
	int i, rc = 0;

	for (i = 0; i < 200; i++) {
		rc = lab_r8(c, DTI_STATUS, &st);
		if (rc)
			break;
		if (st & 0x0c) {
			scan_append(s, "dti %s: status 0x%02x (%s)\n", what, st,
				    st & 0x08 ? "improper interface usage" :
						"data corrupted");
			return -EIO;
		}
		if (st & 0x02)
			return 0;
		usleep_range(1000, 1100);
	}
	scan_append(s, "dti %s: timeout status=0x%02x rc=%d\n", what, st, rc);
	return rc ? rc : -ETIMEDOUT;
}

/* cam_drv_SMIApp_OIS_poll_till_ready: 0x0524 != 0, 100 x 1 ms */
static int ois_poll_ready(struct talkman_cci_scan *s,
			  struct msm_camera_i2c_client *c, const char *what)
{
	u8 st = 0;
	int i, rc = 0;

	for (i = 0; i < 100; i++) {
		rc = lab_r8(c, OIS_READY, &st);
		if (rc || st)
			break;
		usleep_range(1000, 1100);
	}
	pr_info("%s: ois %s: 0x0524=0x%02x after %d reads rc=%d\n",
		DRV_NAME, what, st, i + 1, rc);
	if (rc)
		return rc;
	return st ? 0 : -ETIMEDOUT;
}

/* DCC register blocks (10454103.dcc, rev17_2 Karma). reg 0 = poll 0x0524. */
struct ois_regval { u16 reg; u8 val; };
#define POLL { 0, 0 }
static const struct ois_regval ois_blk_init[] = {
	{ 0x0520, 0x01 }, POLL, { 0x0523, 0x00 }, POLL, { 0x0520, 0x01 }, POLL,
	{ 0x0525, 0xC0 }, POLL, { 0x0521, 0x05 }, POLL, { 0x0559, 0x00 },
	{ 0x0550, 0x50 }, POLL, { 0x0551, 0x41 }, POLL, { 0x0552, 0x98 }, POLL,
	{ 0x0553, 0xC3 }, POLL, { 0x0554, 0x3F }, POLL, { 0x0555, 0xF8 }, POLL,
	{ 0x0556, 0x3F }, POLL, { 0x0557, 0xF8 }, POLL,
	{ 0x0550, 0x50 }, POLL, { 0x0551, 0x51 }, POLL, { 0x0552, 0x98 }, POLL,
	{ 0x0553, 0xC3 }, POLL, { 0x0554, 0x3F }, POLL, { 0x0555, 0xF8 }, POLL,
	{ 0x0556, 0x3F }, POLL, { 0x0557, 0xF8 }, POLL,
};
static const struct ois_regval ois_blk_on_vf[] = {
	{ 0x0520, 0x01 }, POLL, { 0x0525, 0xF9 }, POLL, { 0x0521, 0x06 },
	{ 0x0559, 0x00 }, { 0x0520, 0x02 }, POLL,
};
static const struct ois_regval ois_blk_off[] = {
	{ 0x0520, 0x01 }, POLL, { 0x0525, 0xC0 }, POLL, { 0x0521, 0x05 }, POLL,
	{ 0x0559, 0x00 },
	{ 0x0550, 0x50 }, POLL, { 0x0551, 0x41 }, POLL, { 0x0552, 0x98 }, POLL,
	{ 0x0553, 0xC3 }, POLL, { 0x0554, 0x3F }, POLL, { 0x0555, 0xF8 }, POLL,
	{ 0x0556, 0x3F }, POLL, { 0x0557, 0xF8 }, POLL,
};
static const struct ois_regval ois_blk_deinit[] = {
	{ 0x0520, 0x01 }, POLL, { 0x0523, 0x04 }, POLL, { 0x052C, 0x0B },
	{ 0x052D, 0x00 }, POLL, { 0x0520, 0x00 },
};

static int ois_apply(struct talkman_cci_scan *s, const struct ois_regval *b,
		     size_t n, const char *what)
{
	struct msm_camera_i2c_client *c = ois_client(s);
	size_t i;
	int rc;

	if (!c)
		return -ENXIO;
	for (i = 0; i < n; i++) {
		if (!b[i].reg)
			rc = ois_poll_ready(s, c, what);
		else
			rc = lab_w8(c, b[i].reg, b[i].val);
		if (rc) {
			scan_append(s, "ois %s: step %zu (0x%04x=0x%02x) rc=%d\n",
				    what, i, b[i].reg, b[i].val, rc);
			return rc;
		}
	}
	scan_append(s, "ois %s: %zu steps ok\n", what, n);
	return 0;
}

/* Register windows that distinguish a running firmware from the boot ROM. */
static void ois_snapshot(struct talkman_cci_scan *s,
			 struct msm_camera_i2c_client *c, const char *what)
{
	static const u16 win[] = { 0x0000, 0x0520, 0x0580 };
	u8 buf[8];
	int i, j, rc;

	for (i = 0; i < ARRAY_SIZE(win); i++) {
		memset(buf, 0, sizeof(buf));
		rc = msm_camera_cci_i2c_read_seq(c, win[i], buf, sizeof(buf));
		scan_append(s, "%s 0x%04x:", what, win[i]);
		for (j = 0; j < sizeof(buf); j++)
			scan_append(s, " %02x", buf[j]);
		scan_append(s, " rc=%d\n", rc);
		if (rc)
			return;
	}
}

/* cam_drv_SMIApp_write_dti_data + handle_ois_fw_loading, synchronous. */
/* One DTI page as a single I2C transaction on the CCI1 pads (GPIO 19/20),
 * the way the Windows driver's platform layer sends it. */
static int dti_page_bitbang(const u8 *img, u32 n)
{
	u8 frame[2 + DTI_PAGE_SIZE];

	frame[0] = DTI_DATA >> 8;
	frame[1] = DTI_DATA & 0xff;
	memcpy(frame + 2, img, n);
	return msm_cam_talkman_i2c_bitbang_write(19, 20, BU24210_SID, frame,
						 2 + n);
}

static int ois_download(struct talkman_cci_scan *s, const char *name,
			bool stop_after_dti, bool use_cci)
{
	struct msm_camera_i2c_client *c = ois_client(s);
	const struct firmware *fw;
	const u8 *img;
	u32 total, fw_size, off, page = 0;
	u8 cap = 0, st = 0, rdy = 0;
	int rc;

	if (!c) {
		scan_append(s, "ois: echo rear > power first\n");
		return -ENXIO;
	}
	rc = request_firmware(&fw, name, s->dev);
	if (rc) {
		scan_append(s, "ois: request_firmware(%s) rc=%d\n", name, rc);
		return rc;
	}
	if (fw->size < BU24210_KAR_HDR + 64 || fw->data[0] != 0x02 ||
	    fw->data[3] != 0) {
		scan_append(s, "ois: %s is not a v0 DTI .kar (size %zu, type 0x%02x)\n",
			    name, fw->size, fw->data[0]);
		rc = -EINVAL;
		goto out;
	}
	total = fw->data[1] | (fw->data[2] << 8);
	fw_size = fw->data[20] | (fw->data[21] << 8);
	if (total != fw->size || fw_size + BU24210_KAR_HDR != total ||
	    fw->data[4] != 0x7c ||
	    (fw->data[6] | fw->data[7] << 8) != DTI_CTRL ||
	    (fw->data[8] | fw->data[9] << 8) != DTI_DATA ||
	    (fw->data[16] | fw->data[17] << 8) != 0) {
		scan_append(s, "ois: %s header dev=%02x ctrl=%02x%02x data=%02x%02x total=%u fw=%u file=%zu unexpected\n",
			    name, fw->data[4], fw->data[7], fw->data[6],
			    fw->data[9], fw->data[8], total, fw_size, fw->size);
		rc = -EINVAL;
		goto out;
	}
	img = fw->data + BU24210_KAR_HDR;
	page = fw->data[14] | (fw->data[15] << 8);
	scan_append(s, "ois: %s fw=%u bytes (%u pages from page %u)\n",
		    name, fw_size, DIV_ROUND_UP(fw_size, DTI_PAGE_SIZE), page);

	rc = lab_r8(c, DTI_CAP, &cap);
	lab_r8(c, DTI_STATUS, &st);
	lab_r8(c, OIS_READY, &rdy);
	scan_append(s, "ois: pre cap=0x%02x status=0x%02x 0x0524=0x%02x rc=%d\n",
		    cap, st, rdy, rc);
	if (rc)
		goto out;
	ois_snapshot(s, c, "ois: pre");

	if (page == 0) {
		rc = lab_w8(c, DTI_CTRL, 0x03);
		if (rc) {
			scan_append(s, "ois: 0x0581=0x03 start rc=%d\n", rc);
			goto out;
		}
	}
	for (off = 0; off < fw_size; off += DTI_PAGE_SIZE, page++) {
		u32 n = min_t(u32, DTI_PAGE_SIZE, fw_size - off);

		if ((cap & 0x08) && page != 0) {
			rc = dti_poll_ready(s, c, "page");
			if (rc)
				goto out;
		}
		rc = lab_w8(c, DTI_PAGE, page);
		if (!rc && use_cci)
			rc = msm_camera_cci_i2c_write_seq(c, DTI_DATA,
							  (u8 *)img + off, n);
		else if (!rc)
			rc = dti_page_bitbang(img + off, n);
		if (rc) {
			scan_append(s, "ois: page %u (%u B) rc=%d\n", page, n, rc);
			goto out;
		}
	}
	scan_append(s, "ois: %u bytes in %u pages written (%s)\n", fw_size, page,
		    use_cci ? "cci 10-byte chunks" : "bit-bang 64-byte bursts");
	rc = dti_poll_ready(s, c, "end");
	if (rc)
		goto out;
	rc = lab_w8(c, DTI_CTRL, 0x04);
	if (rc) {
		scan_append(s, "ois: 0x0581=0x04 end rc=%d\n", rc);
		goto out;
	}
	{
		/* Trace 0x0524 right after the DTI close: a firmware start
		 * should show as a transition (or a short NAK window). */
		u8 tr[24];
		int i, first_err = -1;

		for (i = 0; i < ARRAY_SIZE(tr); i++) {
			rc = lab_r8(c, OIS_READY, &tr[i]);
			if (rc) {
				tr[i] = 0xee;
				if (first_err < 0)
					first_err = i;
			}
			usleep_range(2000, 2200);
		}
		scan_append(s, "ois: 0x0524 trace (2 ms):");
		for (i = 0; i < ARRAY_SIZE(tr); i++)
			scan_append(s, " %02x", tr[i]);
		scan_append(s, "%s\n", first_err >= 0 ? " (ee = NAK)" : "");
		rc = 0;
	}
	ois_snapshot(s, c, "ois: post-dti");
	if (stop_after_dti) {
		scan_append(s, "ois: stopped after DTI close (no 0x0520 write)\n");
		goto out;
	}
	rc = ois_poll_ready(s, c, "fw-up");
	lab_r8(c, OIS_READY, &rdy);
	scan_append(s, "ois: after download 0x0524=0x%02x rc=%d\n", rdy, rc);
	if (rc)
		goto out;
	rc = lab_w8(c, OIS_MODE, 0x01);
	if (!rc)
		rc = ois_poll_ready(s, c, "mode1");
	scan_append(s, "ois: 0x0520=0x01 rc=%d\n", rc);
	if (rc)
		goto out;
	rc = ois_apply(s, ois_blk_init, ARRAY_SIZE(ois_blk_init), "init");
out:
	release_firmware(fw);
	return rc;
}

/* cam_drv_SMIApp_cache_nvm: 64-byte module NVM page via the sensor DTI. */
static int sensor_nvm(struct talkman_cci_scan *s, unsigned int page)
{
	struct msm_camera_i2c_client *c = sensor_client(s);
	u8 buf[DTI_PAGE_SIZE], st = 0;
	int i, rc;

	if (!c)
		return -ENXIO;
	if (page > 255)
		return -EINVAL;
	rc = lab_w8(c, SMIA_DTI_PAGE, page);
	if (!rc)
		rc = lab_w8(c, SMIA_DTI_CTRL, 0x01);
	if (rc) {
		scan_append(s, "nvm: page select rc=%d\n", rc);
		return rc;
	}
	for (i = 0; i < 50; i++) {
		rc = lab_r8(c, SMIA_DTI_STATUS, &st);
		if (rc || (st & 0x01))
			break;
		usleep_range(1000, 1100);
	}
	if (rc || !(st & 0x01)) {
		scan_append(s, "nvm: page %u status=0x%02x rc=%d\n", page, st, rc);
		return rc ? rc : -ETIMEDOUT;
	}
	rc = msm_camera_cci_i2c_read_seq(c, SMIA_DTI_DATA, buf, sizeof(buf));
	scan_append(s, "nvm page %u rc=%d:", page, rc);
	for (i = 0; i < DTI_PAGE_SIZE; i++)
		scan_append(s, "%s%02x", (i % 16) ? " " : "\n  ", buf[i]);
	scan_append(s, "\n");
	lab_w8(c, SMIA_DTI_CTRL, 0x00);
	return rc;
}

/* cam_drv_read_af_vital_info / read_actuator_capabilities. */
static int sensor_afinfo(struct talkman_cci_scan *s, unsigned int *sid7,
			 unsigned int *reg, bool *word_addr)
{
	struct msm_camera_i2c_client *c = sensor_client(s);
	u8 cap = 0, dev = 0;
	u16 type = 0, focus = 0;
	int rc;

	if (!c)
		return -ENXIO;
	rc = lab_r8(c, SMIA_ACT_CAP, &cap);
	rc |= lab_r16(c, SMIA_ACT_TYPE, &type);
	rc |= lab_r8(c, SMIA_AF_DEV, &dev);
	rc |= lab_r16(c, SMIA_FOCUS_REG, &focus);
	scan_append(s, "afinfo: cap=0x%02x type=0x%04x dev=0x%02x focus=0x%04x rc=%d%s\n",
		    cap, type, dev, focus, rc,
		    cap & 0x04 ? "" : " (cap bit2 clear: WOA would disable lens)");
	if (ois_af_sid7) {
		*sid7 = ois_af_sid7;
		*reg = ois_af_reg;
		*word_addr = true;
		scan_append(s, "afinfo: manual override sid7=0x%02x reg=0x%04x\n",
			    *sid7, *reg);
		return rc;
	}
	if (rc)
		return rc;
	if (!dev) {
		scan_append(s, "afinfo: descriptor empty; use 'afdev <sid7> <reg>' (Mersu DCC: 0x10 0x0D80)\n");
		return -ENODEV;
	}
	*sid7 = dev >> 1;
	*reg = focus;
	*word_addr = (dev == 0x20 || dev == 0x7c);
	return 0;
}

/* cam_drv_lens_move generic branch: reg=pos>>8, reg+1=pos, reg+2=2, reg+3=1. */
static int ois_af(struct talkman_cci_scan *s, unsigned int pos)
{
	struct msm_camera_i2c_client *c;
	unsigned int sid7 = 0, reg = 0;
	bool word = true;
	u8 st = 0xff;
	int i, rc;

	if (pos > 0xffff)
		return -EINVAL;
	rc = sensor_afinfo(s, &sid7, &reg, &word);
	if (rc)
		return rc;
	c = lab_client(s, sid7, word ? MSM_CAMERA_I2C_WORD_ADDR :
					MSM_CAMERA_I2C_BYTE_ADDR);
	if (!c)
		return -ENXIO;
	rc = lab_w8(c, reg, pos >> 8);
	if (!rc)
		rc = lab_w8(c, reg + 1, pos & 0xff);
	if (!rc)
		rc = lab_w8(c, reg + 2, 0x02);
	if (!rc)
		rc = lab_w8(c, reg + 3, 0x01);
	if (rc) {
		scan_append(s, "af: write to sid7=0x%02x reg=0x%04x rc=%d\n",
			    sid7, reg, rc);
		return rc;
	}
	for (i = 0; i < 30; i++) {
		rc = lab_r8(c, reg + 3, &st);
		if (rc || !(st & 0x01))
			break;
		usleep_range(5000, 5100);
	}
	pr_info("%s: af sid7=0x%02x reg=0x%04x pos=%u busy=0x%02x reads=%d rc=%d\n",
		DRV_NAME, sid7, reg, pos, st, i + 1, rc);
	scan_append(s, "af: sid7=0x%02x reg=0x%04x pos=%u busy=0x%02x reads=%d rc=%d\n",
		    sid7, reg, pos, st, i + 1, rc);
	return rc;
}

static int ois_dump(struct talkman_cci_scan *s, unsigned int reg,
		    unsigned int n)
{
	struct msm_camera_i2c_client *c = ois_client(s);
	u8 buf[16];
	int rc;
	unsigned int i;

	if (!c)
		return -ENXIO;
	if (!n || n > sizeof(buf))
		return -EINVAL;
	rc = msm_camera_cci_i2c_read_seq(c, reg, buf, n);
	scan_append(s, "ois: 0x%04x:", reg);
	for (i = 0; i < n; i++)
		scan_append(s, " %02x", buf[i]);
	scan_append(s, " rc=%d\n", rc);
	return rc;
}

/*
 * ois: "reset" | "nvm <page>" | "afinfo" | "afdev <sid7> <reg>" |
 *      "dl <kar> [stop][cci]" (default: bit-banged 64-byte pages, run init) |
 *      "init" | "on" | "off" | "deinit" | "af <0..65535>" | "rd <reg> [n]"
 * Rear power must be held. Snap must be closed.
 */
static ssize_t ois_write(struct file *file, const char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	struct talkman_cci_scan *s = file->private_data;
	char cmd[128], name[96], arg[16];
	unsigned int a = 0, b = 1;
	bool word;
	int rc;

	if (count >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, ubuf, count))
		return -EFAULT;
	cmd[count] = '\0';
	strim(cmd);

	mutex_lock(&s->lock);
	s->buf_len = 0;
	s->buf[0] = '\0';
	if (sscanf(cmd, "dl %95s %15s", name, arg) == 2)
		rc = ois_download(s, name, strstr(arg, "stop") != NULL,
				  strstr(arg, "cci") != NULL);
	else if (sscanf(cmd, "dl %95s", name) == 1)
		rc = ois_download(s, name, false, false);
	else if (!strcmp(cmd, "reset")) {
		/* Cold-start the module: rails off long enough for the BU24210
		 * to lose its RAM, then the normal rear hold. Only works once
		 * L23/L25/L29 are no longer regulator-always-on. */
		if (s->held != 1) {
			rc = -ENXIO;
		} else {
			scan_hold_off(s);
			msleep(2000);
			rc = scan_hold_on(s, 1);
		}
	} else if (sscanf(cmd, "nvm %u", &a) == 1)
		rc = sensor_nvm(s, a);
	else if (!strcmp(cmd, "afinfo"))
		rc = sensor_afinfo(s, &a, &b, &word);
	else if (sscanf(cmd, "afdev %i %i", &a, &b) == 2) {
		ois_af_sid7 = a & 0x7f;
		ois_af_reg = b & 0xffff;
		scan_append(s, "afdev: sid7=0x%02x reg=0x%04x (0 0 clears)\n",
			    ois_af_sid7, ois_af_reg);
		rc = 0;
	} else if (!strcmp(cmd, "init"))
		rc = ois_apply(s, ois_blk_init, ARRAY_SIZE(ois_blk_init), "init");
	else if (!strcmp(cmd, "on"))
		rc = ois_apply(s, ois_blk_on_vf, ARRAY_SIZE(ois_blk_on_vf), "on");
	else if (!strcmp(cmd, "off"))
		rc = ois_apply(s, ois_blk_off, ARRAY_SIZE(ois_blk_off), "off");
	else if (!strcmp(cmd, "deinit"))
		rc = ois_apply(s, ois_blk_deinit, ARRAY_SIZE(ois_blk_deinit), "deinit");
	else if (sscanf(cmd, "af %u", &a) == 1)
		rc = ois_af(s, a);
	else if (sscanf(cmd, "rd %i %u", &a, &b) >= 1)
		rc = ois_dump(s, a, b);
	else
		rc = -EINVAL;
	mutex_unlock(&s->lock);
	return rc ? rc : count;
}

static const struct file_operations ois_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = scan_read,
	.write = ois_write,
	.llseek = default_llseek,
};

static void talkman_cci_scan_own(struct dentry *d, umode_t mode)
{
	struct inode *inode;
	kgid_t shell = KGIDT_INIT(TALKMAN_CCI_SCAN_GID);

	if (!d || !d->d_inode)
		return;
	inode = d->d_inode;
	if (uid_eq(inode->i_uid, GLOBAL_ROOT_UID) &&
	    gid_eq(inode->i_gid, shell) &&
	    ((inode->i_mode & 0777) == (mode & 0777)))
		return;
	inode->i_uid = GLOBAL_ROOT_UID;
	inode->i_gid = shell;
	inode->i_mode = (inode->i_mode & ~0777) | (mode & 0777);
}

static int talkman_cci_scan_probe(struct platform_device *pdev)
{
	struct talkman_cci_scan *s;
	struct device_node *np = pdev->dev.of_node;

	s = devm_kzalloc(&pdev->dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->buf = devm_kzalloc(&pdev->dev, SCAN_BUF_SZ, GFP_KERNEL);
	if (!s->buf)
		return -ENOMEM;
	s->dev = &pdev->dev;
	mutex_init(&s->lock);

	s->vana = devm_regulator_get(&pdev->dev, "cam_vana");
	s->vaf = devm_regulator_get(&pdev->dev, "cam_vaf");
	s->vdig = devm_regulator_get(&pdev->dev, "cam_vdig");
	s->vio = devm_regulator_get(&pdev->dev, "cam_vio");
	s->vana_front = devm_regulator_get(&pdev->dev, "cam_vana_front");
	if (IS_ERR(s->vana) || IS_ERR(s->vaf) || IS_ERR(s->vdig) ||
	    IS_ERR(s->vio) || IS_ERR(s->vana_front)) {
		dev_err(&pdev->dev, "camera regulators not ready\n");
		return -EPROBE_DEFER;
	}

	s->mclk0_src = devm_clk_get(&pdev->dev, "mclk0_src");
	s->mclk0 = devm_clk_get(&pdev->dev, "mclk0");
	s->mclk2_src = devm_clk_get(&pdev->dev, "mclk2_src");
	s->mclk2 = devm_clk_get(&pdev->dev, "mclk2");
	if (IS_ERR(s->mclk0_src) || IS_ERR(s->mclk0) ||
	    IS_ERR(s->mclk2_src) || IS_ERR(s->mclk2)) {
		dev_err(&pdev->dev, "mclk clocks not ready\n");
		return -EPROBE_DEFER;
	}
	if (of_property_read_u32(np, "qcom,mclk-rate", &s->mclk_rate))
		s->mclk_rate = 24000000;

	s->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (!IS_ERR(s->pinctrl)) {
		s->pins_default = pinctrl_lookup_state(s->pinctrl,
						       PINCTRL_STATE_DEFAULT);
		s->pins_scan = pinctrl_lookup_state(s->pinctrl, "scan");
	}

	s->front_rst_gpio = of_get_named_gpio(np, "mmo,front-reset-gpio", 0);
	if (gpio_is_valid(s->front_rst_gpio)) {
		if (devm_gpio_request_one(&pdev->dev, s->front_rst_gpio,
					  GPIOF_OUT_INIT_LOW,
					  "CAM_FRONT_RES_N")) {
			dev_warn(&pdev->dev, "front reset GPIO %d busy\n",
				 s->front_rst_gpio);
			s->front_rst_gpio = -1;
		}
	}

	s->dbg = debugfs_create_dir(DRV_NAME, NULL);
	if (s->dbg) {
		struct dentry *d;

		talkman_cci_scan_own(s->dbg, 0755);
		d = debugfs_create_file("scan", 0660, s->dbg, s, &scan_fops);
		talkman_cci_scan_own(d, 0660);
		d = debugfs_create_file("power", 0660, s->dbg, s, &power_fops);
		talkman_cci_scan_own(d, 0660);
		d = debugfs_create_file("i2c", 0660, s->dbg, s, &i2c_fops);
		talkman_cci_scan_own(d, 0660);
		d = debugfs_create_file("ois", 0660, s->dbg, s, &ois_fops);
		talkman_cci_scan_own(d, 0660);
	}

	platform_set_drvdata(pdev, s);
	scan_append(s,
		    "idle. echo 1 > scan | echo rear|front|off > power | "
		    "echo 'r <m> <sid7> <reg> [alen] [dlen]' > i2c\n");
	dev_info(&pdev->dev,
		 "CCI scanner ready (mclk %u Hz). echo 1 > /sys/kernel/debug/%s/scan\n",
		 s->mclk_rate, DRV_NAME);
	return 0;
}

static int talkman_cci_scan_remove(struct platform_device *pdev)
{
	struct talkman_cci_scan *s = platform_get_drvdata(pdev);

	mutex_lock(&s->lock);
	scan_hold_off(s);
	mutex_unlock(&s->lock);
	debugfs_remove_recursive(s->dbg);
	return 0;
}

static const struct of_device_id talkman_cci_scan_match[] = {
	{ .compatible = "mmo,talkman-cci-scan" },
	{}
};
MODULE_DEVICE_TABLE(of, talkman_cci_scan_match);

static struct platform_driver talkman_cci_scan_driver = {
	.probe = talkman_cci_scan_probe,
	.remove = talkman_cci_scan_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = talkman_cci_scan_match,
	},
};

module_platform_driver(talkman_cci_scan_driver);

MODULE_DESCRIPTION("Talkman CCI I2C scanner for unknown camera slaves");
MODULE_LICENSE("GPL v2");
