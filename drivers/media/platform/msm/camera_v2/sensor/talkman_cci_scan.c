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
	struct dentry *dbg;
	struct mutex lock;
	char *buf;
	size_t buf_len;
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
	msleep(3);
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

static int talkman_cci_do_scan(struct talkman_cci_scan *s)
{
	struct msm_camera_cci_client *cci_client;
	struct msm_camera_i2c_client client;
	struct v4l2_subdev *sd;
	int rc, found = 0, inited = 0;

	s->buf_len = 0;
	s->buf[0] = '\0';

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
	memset(&client, 0, sizeof(client));
	client.cci_client = cci_client;
	client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	/*
	 * MSM_CCI_INIT programs SCL/SDA timing only for cci_i2c_master,
	 * and only after the global reset on the first INIT. Rear I2C is
	 * schematic CCI1 (GPIO 19/20), so INIT MASTER_1 first. A
	 * MASTER_0-only INIT left MASTER_1 unprogrammed: each SID then
	 * hit CCI_TIMEOUT ("MASTER_1 error 0x40000000") instead of a NACK.
	 * Second INIT (ref_count++) sets MASTER_0 clk params.
	 */
	cci_client->cci_i2c_master = MASTER_1;
	rc = msm_sensor_cci_i2c_util(&client, MSM_CCI_INIT);
	if (rc) {
		pr_err("%s: CCI INIT master1 failed rc=%d\n", DRV_NAME, rc);
		scan_append(s, "CCI INIT master1 failed %d\n", rc);
		kfree(cci_client);
		return rc;
	}
	inited++;

	cci_client->cci_i2c_master = MASTER_0;
	rc = msm_sensor_cci_i2c_util(&client, MSM_CCI_INIT);
	if (rc)
		pr_err("%s: CCI INIT master0 rc=%d\n", DRV_NAME, rc);
	else
		inited++;

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
	while (inited--)
		msm_sensor_cci_i2c_util(&client, MSM_CCI_RELEASE);
	kfree(cci_client);
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

	s->dbg = debugfs_create_dir(DRV_NAME, NULL);
	if (s->dbg) {
		struct dentry *scan;

		talkman_cci_scan_own(s->dbg, 0755);
		scan = debugfs_create_file("scan", 0660, s->dbg, s, &scan_fops);
		talkman_cci_scan_own(scan, 0660);
	}

	platform_set_drvdata(pdev, s);
	scan_append(s, "idle. echo 1 > /sys/kernel/debug/%s/scan\n", DRV_NAME);
	dev_info(&pdev->dev,
		 "CCI scanner ready (mclk %u Hz). echo 1 > /sys/kernel/debug/%s/scan\n",
		 s->mclk_rate, DRV_NAME);
	return 0;
}

static int talkman_cci_scan_remove(struct platform_device *pdev)
{
	struct talkman_cci_scan *s = platform_get_drvdata(pdev);

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
