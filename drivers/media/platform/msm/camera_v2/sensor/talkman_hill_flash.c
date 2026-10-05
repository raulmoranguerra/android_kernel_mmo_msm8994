/*
 * Lumia 950 (talkman) rear flash: "Hill", the three-LED flash driver on
 * CCI0 at 7-bit address 0x30 (ID register 0x00 = 0x16, revision 0x02).
 *
 * Register use as programmed by the Windows Phone WhiteLED flash driver
 * (qccamflash8992.sys, hill.c), checked on the device:
 *   0x02-0x04  LED 1-3 current: flash in 50 mA steps; torch 0xc0 (the
 *              Windows assist light, its maximum level)
 *   0x05       0x19 (Windows flash mode 0)
 *   0x06       boost/current-limit band, 0 up to 1750 mA total
 *   0x07       flash timeout, 4 ms steps, max 0x1f (about 124 ms)
 *   0x08       mode: 0x03 standby, 0x06 torch, 0xe7 flash (fires on write)
 *   0x09       status/faults
 * The chip answers on CCI with no camera rail powered.
 *
 * Two LED class devices:
 *   led:flash_torch  torch (Quick Settings tile, flash-mode torch). It
 *                    replaces the GPIO 12 gpio-leds torch; GPIO 12 is the
 *                    chip's TORCH_EN pin and is held low here.
 *   led:flash_rear   photo strobe. Brightness is mA per LED (50 mA steps,
 *                    max mmo,flash-ma). While lit, flash mode is re-armed
 *                    every 60 ms, before the chip's 124 ms timeout, and the
 *                    strobe switches itself off after mmo,flash-max-ms.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include "msm_cci.h"
#include "msm_camera_i2c.h"

#define DRV_NAME		"talkman-hill-flash"
#define HILL_SID		0x30
#define HILL_ID			0x16

#define HILL_REG_ID		0x00
#define HILL_REG_LED1		0x02
#define HILL_REG_CFG		0x05
#define HILL_REG_BAND		0x06
#define HILL_REG_TIMEOUT	0x07
#define HILL_REG_MODE		0x08
#define HILL_REG_STATUS		0x09

#define HILL_MODE_STANDBY	0x03
#define HILL_MODE_TORCH		0x06
#define HILL_MODE_FLASH		0xe7

#define HILL_TORCH_CODE		0xc0
#define HILL_FLASH_STEP_MA	50
#define HILL_TIMEOUT_MAX	0x1f
#define HILL_REARM_MS		60

struct hill {
	struct device *dev;
	struct mutex lock;
	struct led_classdev torch;
	struct led_classdev flash;
	struct work_struct torch_work;
	struct work_struct flash_work;
	struct delayed_work rearm;
	enum led_brightness torch_b;
	enum led_brightness flash_b;
	bool strobing;
	unsigned long deadline;
	u32 flash_ma;
	u32 flash_max_ms;
	int torch_en_gpio;
	/* CCI held for the length of a strobe */
	struct msm_camera_i2c_client client;
	struct msm_camera_cci_client *cci;
	int inited;
};

/*
 * INIT both CCI masters as talkman_cci_scan does (master 1 first: the
 * first INIT does the global reset and only programs the timing of the
 * master it names). Each successful INIT needs a matching RELEASE.
 */
static int hill_cci_get(struct hill *h)
{
	struct v4l2_subdev *sd;
	int rc;

	if (h->cci)
		return 0;
	sd = msm_cci_get_subdev();
	if (!sd)
		return -EPROBE_DEFER;
	h->cci = kzalloc(sizeof(*h->cci), GFP_KERNEL);
	if (!h->cci)
		return -ENOMEM;
	h->cci->cci_subdev = sd;
	h->cci->retries = 3;
	h->cci->id_map = 0;
	h->cci->i2c_freq_mode = I2C_STANDARD_MODE;
	memset(&h->client, 0, sizeof(h->client));
	h->client.cci_client = h->cci;
	h->client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	h->inited = 0;

	h->cci->cci_i2c_master = MASTER_1;
	rc = msm_sensor_cci_i2c_util(&h->client, MSM_CCI_INIT);
	if (rc)
		goto fail;
	h->inited++;
	h->cci->cci_i2c_master = MASTER_0;
	rc = msm_sensor_cci_i2c_util(&h->client, MSM_CCI_INIT);
	if (rc)
		goto fail;
	h->inited++;
	h->cci->sid = HILL_SID;
	return 0;
fail:
	dev_err(h->dev, "CCI INIT failed rc=%d\n", rc);
	while (h->inited-- > 0)
		msm_sensor_cci_i2c_util(&h->client, MSM_CCI_RELEASE);
	kfree(h->cci);
	h->cci = NULL;
	return rc;
}

static void hill_cci_put(struct hill *h)
{
	if (!h->cci)
		return;
	while (h->inited-- > 0)
		msm_sensor_cci_i2c_util(&h->client, MSM_CCI_RELEASE);
	kfree(h->cci);
	h->cci = NULL;
}

static int hill_write(struct hill *h, u8 reg, u8 val)
{
	int rc = msm_camera_cci_i2c_write(&h->client, reg, val,
					  MSM_CAMERA_I2C_BYTE_DATA);

	if (rc < 0)
		dev_err(h->dev, "write 0x%02x <- 0x%02x failed %d\n",
			reg, val, rc);
	return rc;
}

static int hill_set_leds(struct hill *h, u8 code)
{
	int i, rc = 0;

	for (i = 0; i < 3 && !rc; i++)
		rc = hill_write(h, HILL_REG_LED1 + i, code);
	return rc;
}

/* Windows assist-light sequence: currents, config, then torch mode. */
static int hill_torch_on(struct hill *h)
{
	int rc;

	rc = hill_write(h, HILL_REG_MODE, HILL_MODE_STANDBY);
	if (!rc)
		rc = hill_set_leds(h, HILL_TORCH_CODE);
	if (!rc)
		rc = hill_write(h, HILL_REG_CFG, 0x19);
	if (!rc)
		rc = hill_write(h, HILL_REG_BAND, 0x00);
	if (!rc)
		rc = hill_write(h, HILL_REG_TIMEOUT, 0x00);
	if (!rc)
		rc = hill_write(h, HILL_REG_MODE, HILL_MODE_TORCH);
	return rc;
}

static void hill_torch_work(struct work_struct *work)
{
	struct hill *h = container_of(work, struct hill, torch_work);

	mutex_lock(&h->lock);
	if (h->strobing)
		goto out;	/* the strobe restores the torch when it ends */
	if (hill_cci_get(h))
		goto out;
	if (h->torch_b)
		hill_torch_on(h);
	else
		hill_write(h, HILL_REG_MODE, HILL_MODE_STANDBY);
	hill_cci_put(h);
out:
	mutex_unlock(&h->lock);
}

static void hill_strobe_stop(struct hill *h)
{
	if (!h->strobing)
		return;
	h->strobing = false;
	if (h->torch_b)
		hill_torch_on(h);
	else
		hill_write(h, HILL_REG_MODE, HILL_MODE_STANDBY);
	hill_cci_put(h);
	h->flash_b = LED_OFF;
	h->flash.brightness = LED_OFF;
}

static void hill_flash_work(struct work_struct *work)
{
	struct hill *h = container_of(work, struct hill, flash_work);
	u8 code;
	int rc;

	mutex_lock(&h->lock);
	if (!h->flash_b) {
		cancel_delayed_work(&h->rearm);
		hill_strobe_stop(h);
		goto out;
	}
	if (h->strobing)
		goto out;	/* already lit; the current is set at start */
	code = clamp_t(u32, h->flash_b / HILL_FLASH_STEP_MA, 1,
		       h->flash_ma / HILL_FLASH_STEP_MA);
	if (hill_cci_get(h))
		goto out;
	rc = hill_write(h, HILL_REG_MODE, HILL_MODE_STANDBY);
	if (!rc)
		rc = hill_set_leds(h, code);
	if (!rc)
		rc = hill_write(h, HILL_REG_CFG, 0x19);
	if (!rc)
		rc = hill_write(h, HILL_REG_BAND, 0x00);
	if (!rc)
		rc = hill_write(h, HILL_REG_TIMEOUT, HILL_TIMEOUT_MAX);
	if (!rc)
		rc = hill_write(h, HILL_REG_MODE, HILL_MODE_FLASH);
	if (rc) {
		hill_write(h, HILL_REG_MODE, HILL_MODE_STANDBY);
		hill_cci_put(h);
		goto out;
	}
	h->strobing = true;
	h->deadline = jiffies + msecs_to_jiffies(h->flash_max_ms);
	dev_info(h->dev, "strobe %u mA x3 (max %u ms)\n",
		 code * HILL_FLASH_STEP_MA, h->flash_max_ms);
	schedule_delayed_work(&h->rearm, msecs_to_jiffies(HILL_REARM_MS));
out:
	mutex_unlock(&h->lock);
}

static void hill_rearm_work(struct work_struct *work)
{
	struct hill *h = container_of(to_delayed_work(work), struct hill,
				      rearm);

	mutex_lock(&h->lock);
	if (!h->strobing)
		goto out;
	if (time_after(jiffies, h->deadline)) {
		dev_info(h->dev, "strobe stopped at %u ms limit\n",
			 h->flash_max_ms);
		hill_strobe_stop(h);
		goto out;
	}
	if (hill_write(h, HILL_REG_MODE, HILL_MODE_FLASH)) {
		hill_strobe_stop(h);
		goto out;
	}
	schedule_delayed_work(&h->rearm, msecs_to_jiffies(HILL_REARM_MS));
out:
	mutex_unlock(&h->lock);
}

static void hill_torch_set(struct led_classdev *cdev, enum led_brightness b)
{
	struct hill *h = container_of(cdev, struct hill, torch);

	h->torch_b = b;
	schedule_work(&h->torch_work);
}

static void hill_flash_set(struct led_classdev *cdev, enum led_brightness b)
{
	struct hill *h = container_of(cdev, struct hill, flash);

	h->flash_b = b;
	schedule_work(&h->flash_work);
}

static int talkman_hill_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct hill *h;
	int rc;

	h = devm_kzalloc(&pdev->dev, sizeof(*h), GFP_KERNEL);
	if (!h)
		return -ENOMEM;
	h->dev = &pdev->dev;
	mutex_init(&h->lock);
	INIT_WORK(&h->torch_work, hill_torch_work);
	INIT_WORK(&h->flash_work, hill_flash_work);
	INIT_DELAYED_WORK(&h->rearm, hill_rearm_work);

	h->flash_ma = 300;
	of_property_read_u32(np, "mmo,flash-ma", &h->flash_ma);
	h->flash_ma = clamp_t(u32, h->flash_ma, HILL_FLASH_STEP_MA, 500);
	h->flash_max_ms = 1500;
	of_property_read_u32(np, "mmo,flash-max-ms", &h->flash_max_ms);

	/* TORCH_EN would light the LEDs regardless of the mode register */
	h->torch_en_gpio = of_get_named_gpio(np, "mmo,torch-en-gpio", 0);
	if (gpio_is_valid(h->torch_en_gpio) &&
	    devm_gpio_request_one(&pdev->dev, h->torch_en_gpio,
				  GPIOF_OUT_INIT_LOW, "HILL_TORCH_EN"))
		dev_warn(&pdev->dev, "TORCH_EN GPIO %d busy\n",
			 h->torch_en_gpio);

	h->torch.name = "led:flash_torch";
	h->torch.max_brightness = LED_FULL;
	h->torch.brightness_set = hill_torch_set;
	h->flash.name = "led:flash_rear";
	h->flash.max_brightness = h->flash_ma;
	h->flash.brightness_set = hill_flash_set;

	rc = led_classdev_register(&pdev->dev, &h->torch);
	if (rc)
		return rc;
	rc = led_classdev_register(&pdev->dev, &h->flash);
	if (rc) {
		led_classdev_unregister(&h->torch);
		return rc;
	}
	platform_set_drvdata(pdev, h);
	dev_info(&pdev->dev, "rear flash on CCI0 0x%02x, strobe %u mA x3, %u ms max\n",
		 HILL_SID, h->flash_ma, h->flash_max_ms);
	return 0;
}

static int talkman_hill_remove(struct platform_device *pdev)
{
	struct hill *h = platform_get_drvdata(pdev);

	led_classdev_unregister(&h->flash);
	led_classdev_unregister(&h->torch);
	cancel_work_sync(&h->torch_work);
	cancel_work_sync(&h->flash_work);
	cancel_delayed_work_sync(&h->rearm);
	mutex_lock(&h->lock);
	hill_strobe_stop(h);
	mutex_unlock(&h->lock);
	return 0;
}

static const struct of_device_id talkman_hill_match[] = {
	{ .compatible = "mmo,talkman-hill-flash" },
	{ }
};

static struct platform_driver talkman_hill_driver = {
	.probe = talkman_hill_probe,
	.remove = talkman_hill_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = talkman_hill_match,
	},
};

/* after msm_cci (module_init) so the CCI subdev exists when first used */
static int __init talkman_hill_init(void)
{
	return platform_driver_register(&talkman_hill_driver);
}
late_initcall(talkman_hill_init);

MODULE_DESCRIPTION("Lumia 950 rear flash (Hill) on CCI");
MODULE_LICENSE("GPL v2");
