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
 * Register sequence is the Rohm BU242xx family flow from the GPL LG
 * drivers (lgit_ois_rohm.c BU24205, lgit_imx258_claf_rohm_ois.c BU24235):
 *   0xF010 <- 0x00           start download
 *   0x0000..                 program RAM (burst)
 *   0xF008 (32 bit)          checksum of the download
 *   0xF006 <- 0x00           download complete
 *   0x6024 == 0x01           ready poll
 *   0x6020 <- 0x01           servo on / lens centering
 *   0x60D6 <- 0x01           VCM (AF) init
 *   0x60DA <- u16            AF target
 *   0x609E <- 0x01           standby
 * The .kar in /vendor/firmware is the WOA DCC blob: 22-byte DTI wrapper
 * (type 0x02, u16le total, pad, "7C 10 81 05 84 05 01 00", 8 zero bytes,
 * u16le fw_size) then the Cortex-M0 image. Everything is verified on the
 * device through the checksum and the status register; nothing here is
 * bound to the HAL.
 */
#define BU24210_SID		0x3e
#define BU24210_MASTER		MASTER_1
#define BU24210_KAR_HDR		22
#define BU24210_REG_START_DL	0xF010
#define BU24210_REG_CHECKSUM	0xF008
#define BU24210_REG_COMPLETE_DL	0xF006
#define BU24210_REG_STATUS	0x6024
#define BU24210_REG_CTRL	0x6020
#define BU24210_REG_VCM_INIT	0x60D6
#define BU24210_REG_VCM_TARGET	0x60DA
#define BU24210_REG_STANDBY	0x609E
#define BU24210_DL_CHUNK	32

static struct msm_camera_i2c_client *ois_client(struct talkman_cci_scan *s)
{
	if (s->held != 1 || !s->held_client.cci_client)
		return NULL;
	s->held_cci->cci_i2c_master = BU24210_MASTER;
	s->held_cci->sid = BU24210_SID;
	s->held_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	return &s->held_client;
}

static int ois_w8(struct msm_camera_i2c_client *c, u16 reg, u8 val)
{
	return msm_camera_cci_i2c_write(c, reg, val, MSM_CAMERA_I2C_BYTE_DATA);
}

static int ois_r8(struct msm_camera_i2c_client *c, u16 reg, u8 *val)
{
	uint16_t d = 0;
	int rc = msm_camera_cci_i2c_read(c, reg, &d, MSM_CAMERA_I2C_BYTE_DATA);

	*val = d & 0xff;
	return rc;
}

/* LG poll_ready: 1 ms, then up to @limit reads of 0x6024 5 ms apart. */
static int ois_poll_ready(struct talkman_cci_scan *s,
			  struct msm_camera_i2c_client *c, int limit,
			  const char *what)
{
	u8 st = 0;
	int i, rc = 0;

	usleep_range(1000, 1100);
	for (i = 0; i < limit; i++) {
		rc = ois_r8(c, BU24210_REG_STATUS, &st);
		if (rc)
			break;
		if (st == 0x01)
			break;
		usleep_range(5000, 5100);
	}
	pr_info("%s: ois %s: 0x6024=0x%02x after %d reads rc=%d\n",
		DRV_NAME, what, st, i + 1, rc);
	scan_append(s, "%s: 0x6024=0x%02x reads=%d rc=%d\n",
		    what, st, i + 1, rc);
	if (rc)
		return rc;
	return st == 0x01 ? 0 : -ETIMEDOUT;
}

static int ois_download(struct talkman_cci_scan *s, const char *name)
{
	struct msm_camera_i2c_client *c = ois_client(s);
	const struct firmware *fw;
	const u8 *fw_data;
	u32 fw_size, total, sum = 0, off, chk_be, chk_le;
	u8 chk[4] = {0, 0, 0, 0}, st = 0;
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
	if (fw->size < BU24210_KAR_HDR + 16 || fw->data[0] != 0x02) {
		scan_append(s, "ois: %s is not a DTI .kar (size %zu, type 0x%02x)\n",
			    name, fw->size, fw->data[0]);
		rc = -EINVAL;
		goto out;
	}
	total = fw->data[1] | (fw->data[2] << 8);
	fw_size = fw->data[20] | (fw->data[21] << 8);
	if (total != fw->size || fw_size + BU24210_KAR_HDR != total) {
		scan_append(s, "ois: %s header total=%u fw=%u file=%zu mismatch\n",
			    name, total, fw_size, fw->size);
		rc = -EINVAL;
		goto out;
	}
	fw_data = fw->data + BU24210_KAR_HDR;
	for (off = 0; off < fw_size; off++)
		sum += fw_data[off];
	scan_append(s, "ois: %s fw=%u bytes wrapper=%02x%02x %02x%02x %02x%02x %02x byte-sum=0x%08x\n",
		    name, fw_size, fw->data[4], fw->data[5], fw->data[6],
		    fw->data[7], fw->data[8], fw->data[9], fw->data[10], sum);

	ois_r8(c, BU24210_REG_STATUS, &st);
	scan_append(s, "ois: pre 0x6024=0x%02x\n", st);

	rc = ois_w8(c, BU24210_REG_START_DL, 0x00);
	if (rc) {
		scan_append(s, "ois: F010 start rc=%d\n", rc);
		goto out;
	}
	for (off = 0; off < fw_size; off += BU24210_DL_CHUNK) {
		u32 n = min_t(u32, BU24210_DL_CHUNK, fw_size - off);

		rc = msm_camera_cci_i2c_write_seq(c, off, (u8 *)fw_data + off,
						  n);
		if (rc) {
			scan_append(s, "ois: burst at 0x%04x (%u B) rc=%d\n",
				    off, n, rc);
			goto out;
		}
	}
	scan_append(s, "ois: %u bytes written to 0x0000..0x%04x\n",
		    fw_size, fw_size - 1);

	rc = msm_camera_cci_i2c_read_seq(c, BU24210_REG_CHECKSUM, chk, 4);
	chk_be = (chk[0] << 24) | (chk[1] << 16) | (chk[2] << 8) | chk[3];
	chk_le = (chk[3] << 24) | (chk[2] << 16) | (chk[1] << 8) | chk[0];
	pr_info("%s: ois F008=%02x %02x %02x %02x (be 0x%08x le 0x%08x) sum 0x%08x rc=%d\n",
		DRV_NAME, chk[0], chk[1], chk[2], chk[3], chk_be, chk_le, sum, rc);
	scan_append(s, "ois: F008=%02x %02x %02x %02x be=0x%08x le=0x%08x %s rc=%d\n",
		    chk[0], chk[1], chk[2], chk[3], chk_be, chk_le,
		    (chk_be == sum || chk_le == sum) ? "MATCH" : "no-match",
		    rc);

	rc = ois_w8(c, BU24210_REG_COMPLETE_DL, 0x00);
	if (rc) {
		scan_append(s, "ois: F006 complete rc=%d\n", rc);
		goto out;
	}
	rc = ois_poll_ready(s, c, 15, "post-download");
out:
	release_firmware(fw);
	return rc;
}

static int ois_servo(struct talkman_cci_scan *s)
{
	struct msm_camera_i2c_client *c = ois_client(s);
	u8 v = 0;
	int rc;

	if (!c)
		return -ENXIO;
	ois_r8(c, BU24210_REG_CTRL, &v);
	scan_append(s, "ois: pre 0x6020=0x%02x\n", v);
	rc = ois_w8(c, BU24210_REG_CTRL, 0x01);
	if (rc)
		return rc;
	rc = ois_poll_ready(s, c, 20, "servo-on");
	if (rc)
		return rc;
	rc = ois_w8(c, BU24210_REG_VCM_INIT, 0x01);
	scan_append(s, "ois: 60D6 vcm-init rc=%d\n", rc);
	if (rc)
		return rc;
	return ois_poll_ready(s, c, 15, "vcm-init");
}

static int ois_af(struct talkman_cci_scan *s, unsigned int target)
{
	struct msm_camera_i2c_client *c = ois_client(s);
	int rc;

	if (!c)
		return -ENXIO;
	if (target > 1023)
		return -EINVAL;
	rc = msm_camera_cci_i2c_write(c, BU24210_REG_VCM_TARGET, target,
				      MSM_CAMERA_I2C_WORD_DATA);
	pr_info("%s: ois af 60DA <- %u rc=%d\n", DRV_NAME, target, rc);
	scan_append(s, "ois: 60DA <- %u rc=%d\n", target, rc);
	return rc;
}

static int ois_standby(struct talkman_cci_scan *s)
{
	struct msm_camera_i2c_client *c = ois_client(s);
	int rc;

	if (!c)
		return -ENXIO;
	rc = ois_w8(c, BU24210_REG_CTRL, 0x01);
	ois_poll_ready(s, c, 15, "centering");
	rc = ois_w8(c, BU24210_REG_STANDBY, 0x01);
	scan_append(s, "ois: 609E standby rc=%d\n", rc);
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
 * ois: "dl <kar>" | "servo" | "af <0..1023>" | "standby" | "rd <reg> [n]"
 * Rear power must be held. Snap must be closed.
 */
static ssize_t ois_write(struct file *file, const char __user *ubuf,
			 size_t count, loff_t *ppos)
{
	struct talkman_cci_scan *s = file->private_data;
	char cmd[128], name[96];
	unsigned int a = 0, b = 1;
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
	if (sscanf(cmd, "dl %95s", name) == 1)
		rc = ois_download(s, name);
	else if (!strcmp(cmd, "servo"))
		rc = ois_servo(s);
	else if (sscanf(cmd, "af %u", &a) == 1)
		rc = ois_af(s, a);
	else if (!strcmp(cmd, "standby"))
		rc = ois_standby(s);
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
