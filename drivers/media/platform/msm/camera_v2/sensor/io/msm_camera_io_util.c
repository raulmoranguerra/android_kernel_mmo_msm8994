/* Copyright (c) 2011-2014, 2019 The Linux Foundataion. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/string.h>
#include <linux/bitops.h>
#include <asm/div64.h>
#include <soc/qcom/camera2.h>
#include <linux/msm-bus.h>
#include "msm_camera_io_util.h"

#define BUFF_SIZE_128 128

/* MSM8992 MMSS + TLMM. GPIO 13 is CAM_MCLK0, not XSHUTDN. */
#define TALKMAN_MMSS_PHYS		0xfd8c0000
#define TALKMAN_MMSS_SIZE		0x5200
#define TALKMAN_TLMM_PHYS		0xfd510000
#define TALKMAN_TLMM_SIZE		0x4000
#define TALKMAN_MMPLL4_MODE		0x0090
#define TALKMAN_MCLK0_CMD_RCGR		0x3360
#define TALKMAN_MCLK0_CFG_RCGR		0x3364
#define TALKMAN_MCLK0_M			0x3368
#define TALKMAN_MCLK0_N			0x336c
#define TALKMAN_MCLK0_D			0x3370
#define TALKMAN_CAMSS_MCLK0_CBCR	0x3384
/* CAF F_MM(24000000, mmpll4_out_main, 10, 1, 4): 960 MHz / 10 * M/N 1/4.
 * CFG MODE is bits [13:12]==2 (dual-edge), not bit 12. 0x2313 with MND
 * bypassed is 96 MHz (HID only) - that is the #26 dump lie. */
#define TALKMAN_MCLK0_CFG_24MHZ		0x2313
#define TALKMAN_MCLK0_M_24MHZ		0x1
#define TALKMAN_MCLK0_N_24MHZ		0xfc	/* ~(4 - 1) */
#define TALKMAN_MCLK0_D_24MHZ		0xfb	/* ~(4) */
#define TALKMAN_MND_DUAL_EDGE		0x2
#define TALKMAN_GPIO13			13
#define TALKMAN_GPIO13_CFG		(0x1000 + 0x10 * TALKMAN_GPIO13)
#define TALKMAN_GPIO13_INOUT		(TALKMAN_GPIO13_CFG + 4)
#define TALKMAN_GP_FUNC_SHFT		2
#define TALKMAN_GP_FUNC_MASK		0xf
#define TALKMAN_GP_DRV_SHFT		6
#define TALKMAN_GP_PULL_MASK		0x3
#define TALKMAN_GP_OE_BIT		9
#define TALKMAN_CAM_MCLK_FUNC		1

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

static void __iomem *talkman_mmss;
static void __iomem *talkman_tlmm;

static u32 talkman_mclk0_parent_hz(u32 src_sel)
{
	switch (src_sel) {
	case 0:
		return 19200000; /* XO */
	case 3:
		return 960000000; /* MMPLL4 out_main (MCLK tables) */
	case 5:
		return 600000000; /* GPLL0 */
	default:
		return 0;
	}
}

static void talkman_mclk0_read(u32 *cmd, u32 *cfg, u32 *mreg, u32 *nreg,
	u32 *dreg, u32 *cbcr, u32 *pll)
{
	*pll = readl_relaxed(talkman_mmss + TALKMAN_MMPLL4_MODE);
	*cmd = readl_relaxed(talkman_mmss + TALKMAN_MCLK0_CMD_RCGR);
	*cfg = readl_relaxed(talkman_mmss + TALKMAN_MCLK0_CFG_RCGR);
	*mreg = readl_relaxed(talkman_mmss + TALKMAN_MCLK0_M);
	*nreg = readl_relaxed(talkman_mmss + TALKMAN_MCLK0_N);
	*dreg = readl_relaxed(talkman_mmss + TALKMAN_MCLK0_D);
	*cbcr = readl_relaxed(talkman_mmss + TALKMAN_CAMSS_MCLK0_CBCR);
}

static u64 talkman_mclk0_hz(u32 cmd, u32 cfg, u32 mreg, u32 nreg, u32 cbcr,
	u32 *src_sel, u32 *hid, u32 *mnd, u32 *m, u32 *n)
{
	u32 src_div, parent, mnd_on;
	u64 hz;

	src_div = cfg & 0x1f;
	*hid = (src_div + 1) / 2;
	if (!*hid)
		*hid = 1;
	*src_sel = (cfg >> 8) & 0x7;
	*mnd = (cfg >> 12) & 0x3;
	*m = mreg & 0xff;
	*n = ((~nreg) & 0xff) + *m;
	mnd_on = (*mnd == TALKMAN_MND_DUAL_EDGE);
	parent = talkman_mclk0_parent_hz(*src_sel);
	hz = parent;
	if (mnd_on && *m && *n) {
		hz *= *m;
		do_div(hz, *n);
	}
	do_div(hz, *hid);
	if ((cmd & BIT(31)) || (cbcr & BIT(31)))
		hz = 0;
	return hz;
}

/* Re-program MMPLL4 / 10 * 1/4 and pulse CMD UPDATE. clk_set_rate can
 * write this while ROOT_OFF; MND counters need a second update after
 * camss_mclk0 CBCR enables the root. */
static void talkman_mclk0_force_24mhz(const char *why)
{
	u32 cmd, i;

	writel_relaxed(TALKMAN_MCLK0_M_24MHZ, talkman_mmss + TALKMAN_MCLK0_M);
	writel_relaxed(TALKMAN_MCLK0_N_24MHZ, talkman_mmss + TALKMAN_MCLK0_N);
	writel_relaxed(TALKMAN_MCLK0_D_24MHZ, talkman_mmss + TALKMAN_MCLK0_D);
	writel_relaxed(TALKMAN_MCLK0_CFG_24MHZ,
		talkman_mmss + TALKMAN_MCLK0_CFG_RCGR);
	wmb();
	cmd = readl_relaxed(talkman_mmss + TALKMAN_MCLK0_CMD_RCGR);
	cmd |= BIT(0);
	writel_relaxed(cmd, talkman_mmss + TALKMAN_MCLK0_CMD_RCGR);
	wmb();
	for (i = 0; i < 500; i++) {
		if (!(readl_relaxed(talkman_mmss + TALKMAN_MCLK0_CMD_RCGR)
		      & BIT(0)))
			break;
		udelay(1);
	}
	pr_info("%s: %s forced 24MHz RCG cfg=0x%x M=0x%x N=0x%x D=0x%x update_loops=%u (MMPLL4/10 MND 1/4, not HID-only 96e6)\n",
		__func__, why ? why : "?", TALKMAN_MCLK0_CFG_24MHZ,
		TALKMAN_MCLK0_M_24MHZ, TALKMAN_MCLK0_N_24MHZ,
		TALKMAN_MCLK0_D_24MHZ, i);
}

void msm_cam_dump_mclk0_pad(const char *why, int fix)
{
	u32 cmd, cfg, mreg, nreg, dreg, cbcr, pll, gcfg, gin;
	u32 hid, src_sel, mnd, m, n, oe, func, pull, drv;
	u32 i, samples, stuck;
	u64 hz;
	static const char *pull_nm[] = { "np", "pd", "kz", "pu" };

	if (!talkman_mmss)
		talkman_mmss = ioremap(TALKMAN_MMSS_PHYS, TALKMAN_MMSS_SIZE);
	if (!talkman_tlmm)
		talkman_tlmm = ioremap(TALKMAN_TLMM_PHYS, TALKMAN_TLMM_SIZE);
	if (!talkman_mmss || !talkman_tlmm) {
		pr_err("%s: %s ioremap mmss=%pK tlmm=%pK failed\n",
			__func__, why ? why : "?", talkman_mmss, talkman_tlmm);
		return;
	}

	talkman_mclk0_read(&cmd, &cfg, &mreg, &nreg, &dreg, &cbcr, &pll);
	gcfg = readl_relaxed(talkman_tlmm + TALKMAN_GPIO13_CFG);
	gin = readl_relaxed(talkman_tlmm + TALKMAN_GPIO13_INOUT);

	func = (gcfg >> TALKMAN_GP_FUNC_SHFT) & TALKMAN_GP_FUNC_MASK;
	oe = (gcfg >> TALKMAN_GP_OE_BIT) & 1;
	pull = gcfg & TALKMAN_GP_PULL_MASK;
	drv = ((gcfg >> TALKMAN_GP_DRV_SHFT) & 0x7) + 1;
	drv <<= 1;

	if (fix && (func != TALKMAN_CAM_MCLK_FUNC || !oe)) {
		u32 ncfg = gcfg;

		ncfg &= ~(TALKMAN_GP_FUNC_MASK << TALKMAN_GP_FUNC_SHFT);
		ncfg |= (TALKMAN_CAM_MCLK_FUNC << TALKMAN_GP_FUNC_SHFT);
		ncfg |= BIT(TALKMAN_GP_OE_BIT);
		writel_relaxed(ncfg, talkman_tlmm + TALKMAN_GPIO13_CFG);
		wmb();
		gcfg = readl_relaxed(talkman_tlmm + TALKMAN_GPIO13_CFG);
		func = (gcfg >> TALKMAN_GP_FUNC_SHFT) & TALKMAN_GP_FUNC_MASK;
		oe = (gcfg >> TALKMAN_GP_OE_BIT) & 1;
		pr_info("%s: %s GPIO13 forced func=%u OE=%u (CAM_MCLK, not XSHUTDN/91/92)\n",
			__func__, why ? why : "?", func, oe);
	}

	if (fix)
		talkman_mclk0_force_24mhz(why);

	talkman_mclk0_read(&cmd, &cfg, &mreg, &nreg, &dreg, &cbcr, &pll);
	gcfg = readl_relaxed(talkman_tlmm + TALKMAN_GPIO13_CFG);
	gin = readl_relaxed(talkman_tlmm + TALKMAN_GPIO13_INOUT);
	func = (gcfg >> TALKMAN_GP_FUNC_SHFT) & TALKMAN_GP_FUNC_MASK;
	oe = (gcfg >> TALKMAN_GP_OE_BIT) & 1;
	pull = gcfg & TALKMAN_GP_PULL_MASK;
	drv = ((gcfg >> TALKMAN_GP_DRV_SHFT) & 0x7) + 1;
	drv <<= 1;
	hz = talkman_mclk0_hz(cmd, cfg, mreg, nreg, cbcr,
		&src_sel, &hid, &mnd, &m, &n);

	samples = 0;
	for (i = 0; i < 16; i++)
		samples |= (readl_relaxed(talkman_tlmm + TALKMAN_GPIO13_INOUT)
			    & BIT(0)) << i;
	stuck = (samples == 0 || samples == 0xffff);

	pr_info("%s: %s MCLK0 hw_hz=%llu cmd=0x%x cfg=0x%x M=0x%x N=0x%x D=0x%x cbcr=0x%x pll4=0x%x src=%u hid=%u mnd=%u m=%u n=%u ROOT_OFF=%u CLK_OFF=%u CLK_EN=%u\n",
		__func__, why ? why : "?", (unsigned long long)hz, cmd, cfg, mreg, nreg, dreg, cbcr,
		pll, src_sel, hid, mnd, m, (mnd == TALKMAN_MND_DUAL_EDGE) ? n : 0,
		!!(cmd & BIT(31)), !!(cbcr & BIT(31)), cbcr & 1);
	pr_info("%s: %s GPIO13 cfg=0x%x inout=0x%x func=%u %s OE=%u pull=%s drv=%umA in_samples=0x%04x %s (MCLK0, not XSHUTDN)\n",
		__func__, why ? why : "?", gcfg, gin, func,
		func == TALKMAN_CAM_MCLK_FUNC ? "CAM_MCLK" : "GPIO",
		oe, pull_nm[pull], drv, samples,
		stuck ? "STUCK" : "toggling");
}
EXPORT_SYMBOL(msm_cam_dump_mclk0_pad);

/*
 * I2C bus recovery for a CCI master whose slave holds SDA low (seen on
 * talkman 2026-09-02: a firmware-less BU24210 at 0x7c wedged CCI1 after a
 * read of 0x00F8; every later transaction, including the IMX230 probe,
 * hit CCI_TIMEOUT until the phone was powered off). The CCI block has no
 * recovery path, so drive the pads directly through TLMM: switch both
 * pins to GPIO, clock SCL 9 times with SDA released, emit a STOP, restore
 * the original pad configuration. Returns SDA level after recovery
 * (1 = released) or -errno.
 */
#define TALKMAN_GP_CFG(n)	(0x1000 + 0x10 * (n))
#define TALKMAN_GP_IN_BIT	0
#define TALKMAN_GP_OUT_BIT	1

static void talkman_pad_drive_low(u32 gpio, bool low)
{
	u32 cfg = readl_relaxed(talkman_tlmm + TALKMAN_GP_CFG(gpio));

	/* open-drain emulation: output-low, or input (external pull-up) */
	if (low) {
		writel_relaxed(0, talkman_tlmm + TALKMAN_GP_CFG(gpio) + 4);
		cfg |= BIT(TALKMAN_GP_OE_BIT);
	} else {
		cfg &= ~BIT(TALKMAN_GP_OE_BIT);
	}
	writel_relaxed(cfg, talkman_tlmm + TALKMAN_GP_CFG(gpio));
	wmb();
}

static int talkman_pad_in(u32 gpio)
{
	return (readl_relaxed(talkman_tlmm + TALKMAN_GP_CFG(gpio) + 4)
		>> TALKMAN_GP_IN_BIT) & 1;
}

int msm_cam_talkman_i2c_bus_recover(u32 sda_gpio, u32 scl_gpio)
{
	u32 sda_cfg, scl_cfg, gpio_cfg;
	int sda_before, scl_before, sda_after, i;

	if (!talkman_tlmm)
		talkman_tlmm = ioremap(TALKMAN_TLMM_PHYS, TALKMAN_TLMM_SIZE);
	if (!talkman_tlmm)
		return -ENOMEM;
	if (sda_gpio > 145 || scl_gpio > 145)
		return -EINVAL;

	sda_cfg = readl_relaxed(talkman_tlmm + TALKMAN_GP_CFG(sda_gpio));
	scl_cfg = readl_relaxed(talkman_tlmm + TALKMAN_GP_CFG(scl_gpio));

	/* GPIO function, keep pull/drive bits, start as inputs */
	gpio_cfg = sda_cfg & ~(TALKMAN_GP_FUNC_MASK << TALKMAN_GP_FUNC_SHFT);
	gpio_cfg &= ~BIT(TALKMAN_GP_OE_BIT);
	writel_relaxed(gpio_cfg, talkman_tlmm + TALKMAN_GP_CFG(sda_gpio));
	gpio_cfg = scl_cfg & ~(TALKMAN_GP_FUNC_MASK << TALKMAN_GP_FUNC_SHFT);
	gpio_cfg &= ~BIT(TALKMAN_GP_OE_BIT);
	writel_relaxed(gpio_cfg, talkman_tlmm + TALKMAN_GP_CFG(scl_gpio));
	wmb();
	udelay(5);
	sda_before = talkman_pad_in(sda_gpio);
	scl_before = talkman_pad_in(scl_gpio);

	/* 9 clocks at ~100 kHz with SDA released; a stuck slave finishes
	 * its byte and releases SDA on the ACK clock. */
	for (i = 0; i < 9; i++) {
		talkman_pad_drive_low(scl_gpio, true);
		udelay(5);
		talkman_pad_drive_low(scl_gpio, false);
		udelay(5);
		if (talkman_pad_in(sda_gpio))
			break;
	}
	/* STOP: SDA low while SCL low, SCL high, then SDA high */
	talkman_pad_drive_low(scl_gpio, true);
	udelay(5);
	talkman_pad_drive_low(sda_gpio, true);
	udelay(5);
	talkman_pad_drive_low(scl_gpio, false);
	udelay(5);
	talkman_pad_drive_low(sda_gpio, false);
	udelay(5);
	sda_after = talkman_pad_in(sda_gpio);

	writel_relaxed(sda_cfg, talkman_tlmm + TALKMAN_GP_CFG(sda_gpio));
	writel_relaxed(scl_cfg, talkman_tlmm + TALKMAN_GP_CFG(scl_gpio));
	wmb();

	pr_info("%s: SDA%u/SCL%u before=%d/%d clocks=%d after SDA=%d cfg restored 0x%x/0x%x\n",
		__func__, sda_gpio, scl_gpio, sda_before, scl_before, i + 1,
		sda_after, sda_cfg, scl_cfg);
	return sda_after;
}
EXPORT_SYMBOL(msm_cam_talkman_i2c_bus_recover);

/*
 * Bit-banged I2C write on CCI pads muxed to GPIO, for slaves that need
 * one long transaction (BU24210 DTI: [S 7C 05 84 d0..d63 P]). The CCI
 * v1.1 block on MSM8992 caps a write command at 10 data bytes and issues
 * each command as its own START..STOP, re-addressing the register, which
 * a page-buffer style slave does not accept as one stream. ~100 kHz,
 * clock stretching honoured up to 1 ms. Returns 0, -EIO on NAK, -ETIMEDOUT.
 * The CCI hardware must be idle on this master while we hold the pads.
 */
#define BB_HALF_US	5

static int bb_scl_high(u32 scl)
{
	int i;

	talkman_pad_drive_low(scl, false);
	for (i = 0; i < 200; i++) {
		if (talkman_pad_in(scl))
			return 0;
		udelay(5);
	}
	return -ETIMEDOUT;
}

static int bb_write_byte(u32 sda, u32 scl, u8 b)
{
	int i, rc, ack;

	for (i = 7; i >= 0; i--) {
		talkman_pad_drive_low(sda, !((b >> i) & 1));
		udelay(BB_HALF_US);
		rc = bb_scl_high(scl);
		if (rc)
			return rc;
		udelay(BB_HALF_US);
		talkman_pad_drive_low(scl, true);
	}
	talkman_pad_drive_low(sda, false);
	udelay(BB_HALF_US);
	rc = bb_scl_high(scl);
	if (rc)
		return rc;
	ack = !talkman_pad_in(sda);
	udelay(BB_HALF_US);
	talkman_pad_drive_low(scl, true);
	udelay(BB_HALF_US);
	return ack ? 0 : -EIO;
}

int msm_cam_talkman_i2c_bitbang_write(u32 sda_gpio, u32 scl_gpio,
				      u8 addr7, const u8 *data, u32 len)
{
	u32 sda_cfg, scl_cfg, gpio_cfg, i;
	int rc = 0;

	if (!talkman_tlmm)
		talkman_tlmm = ioremap(TALKMAN_TLMM_PHYS, TALKMAN_TLMM_SIZE);
	if (!talkman_tlmm)
		return -ENOMEM;
	if (sda_gpio > 145 || scl_gpio > 145)
		return -EINVAL;

	sda_cfg = readl_relaxed(talkman_tlmm + TALKMAN_GP_CFG(sda_gpio));
	scl_cfg = readl_relaxed(talkman_tlmm + TALKMAN_GP_CFG(scl_gpio));
	gpio_cfg = sda_cfg & ~(TALKMAN_GP_FUNC_MASK << TALKMAN_GP_FUNC_SHFT);
	gpio_cfg &= ~BIT(TALKMAN_GP_OE_BIT);
	writel_relaxed(gpio_cfg, talkman_tlmm + TALKMAN_GP_CFG(sda_gpio));
	gpio_cfg = scl_cfg & ~(TALKMAN_GP_FUNC_MASK << TALKMAN_GP_FUNC_SHFT);
	gpio_cfg &= ~BIT(TALKMAN_GP_OE_BIT);
	writel_relaxed(gpio_cfg, talkman_tlmm + TALKMAN_GP_CFG(scl_gpio));
	wmb();
	udelay(BB_HALF_US);

	if (!talkman_pad_in(sda_gpio) || !talkman_pad_in(scl_gpio)) {
		rc = -EBUSY;
		goto restore;
	}
	/* START */
	talkman_pad_drive_low(sda_gpio, true);
	udelay(BB_HALF_US);
	talkman_pad_drive_low(scl_gpio, true);
	udelay(BB_HALF_US);

	rc = bb_write_byte(sda_gpio, scl_gpio, addr7 << 1);
	for (i = 0; !rc && i < len; i++)
		rc = bb_write_byte(sda_gpio, scl_gpio, data[i]);

	/* STOP (also after a NAK, so the slave is left idle) */
	talkman_pad_drive_low(sda_gpio, true);
	udelay(BB_HALF_US);
	bb_scl_high(scl_gpio);
	udelay(BB_HALF_US);
	talkman_pad_drive_low(sda_gpio, false);
	udelay(BB_HALF_US);
	if (rc)
		pr_info("%s: SDA%u/SCL%u addr 0x%02x len %u rc=%d at byte %u\n",
			__func__, sda_gpio, scl_gpio, addr7, len, rc, i);
restore:
	writel_relaxed(sda_cfg, talkman_tlmm + TALKMAN_GP_CFG(sda_gpio));
	writel_relaxed(scl_cfg, talkman_tlmm + TALKMAN_GP_CFG(scl_gpio));
	wmb();
	return rc;
}
EXPORT_SYMBOL(msm_cam_talkman_i2c_bitbang_write);

void msm_camera_io_w(u32 data, void __iomem *addr)
{
	CDBG("%s: 0x%pK %08x\n", __func__,  (addr), (data));
	writel_relaxed((data), (addr));
}

void msm_camera_io_w_mb(u32 data, void __iomem *addr)
{
	CDBG("%s: 0x%pK %08x\n", __func__,  (addr), (data));
	wmb();
	writel_relaxed((data), (addr));
	wmb();
}

u32 msm_camera_io_r(void __iomem *addr)
{
	uint32_t data = readl_relaxed(addr);
	CDBG("%s: 0x%pK %08x\n", __func__,  (addr), (data));
	return data;
}

u32 msm_camera_io_r_mb(void __iomem *addr)
{
	uint32_t data;
	rmb();
	data = readl_relaxed(addr);
	rmb();
	CDBG("%s: 0x%pK %08x\n", __func__,  (addr), (data));
	return data;
}

void msm_camera_io_memcpy_toio(void __iomem *dest_addr,
	void __iomem *src_addr, u32 len)
{
	int i;
	u32 *d = (u32 *) dest_addr;
	u32 *s = (u32 *) src_addr;

	for (i = 0; i < len; i++)
		writel_relaxed(*s++, d++);
}

void msm_camera_io_dump(void __iomem *addr, int size)
{
	char line_str[BUFF_SIZE_128], *p_str;
	int i;
	u32 *p = (u32 *) addr;
	u32 data;
	CDBG("%s: %pK %d\n", __func__, addr, size);
	line_str[0] = '\0';
	p_str = line_str;
	for (i = 0; i < size/4; i++) {
		if (i % 4 == 0) {
			snprintf(p_str, 12, "0x%pK: ",  p);
			p_str += 10;
		}
		data = readl_relaxed(p++);
		snprintf(p_str, 12, "%d ", data);
		p_str += 9;
		if ((i + 1) % 4 == 0) {
			CDBG("%s\n", line_str);
			line_str[0] = '\0';
			p_str = line_str;
		}
	}
	if (line_str[0] != '\0')
		CDBG("%s\n", line_str);
}

void msm_camera_io_memcpy(void __iomem *dest_addr,
	void __iomem *src_addr, u32 len)
{
	CDBG("%s: %pK %pK %d\n", __func__, dest_addr, src_addr, len);
	msm_camera_io_memcpy_toio(dest_addr, src_addr, len / 4);
	msm_camera_io_dump(dest_addr, len);
}

void msm_camera_io_memcpy_mb(void __iomem *dest_addr,
	void __iomem *src_addr, u32 len)
{
	int i;
	u32 *d = (u32 *) dest_addr;
	u32 *s = (u32 *) src_addr;

	for (i = 0; i < (len / 4); i++)
		msm_camera_io_w_mb(*s++, d++);
}

int msm_cam_clk_sel_src(struct device *dev, struct msm_cam_clk_info *clk_info,
		struct msm_cam_clk_info *clk_src_info, int num_clk)
{
	int i;
	int rc = 0;
	struct clk *mux_clk = NULL;
	struct clk *src_clk = NULL;

	for (i = 0; i < num_clk; i++) {
		if (clk_src_info[i].clk_name) {
			mux_clk = clk_get(dev, clk_info[i].clk_name);
			if (IS_ERR(mux_clk)) {
				pr_err("%s get failed\n",
					 clk_info[i].clk_name);
				continue;
			}
			src_clk = clk_get(dev, clk_src_info[i].clk_name);
			if (IS_ERR(src_clk)) {
				pr_err("%s get failed\n",
					clk_src_info[i].clk_name);
				continue;
			}
			clk_set_parent(mux_clk, src_clk);
		}
	}
	return rc;
}

int msm_cam_clk_enable(struct device *dev, struct msm_cam_clk_info *clk_info,
		struct clk **clk_ptr, int num_clk, int enable)
{
	int i;
	int rc = 0;
	long clk_rate;
	if (enable) {
		for (i = 0; i < num_clk; i++) {
			CDBG("%s enable %s\n", __func__, clk_info[i].clk_name);
			clk_ptr[i] = clk_get(dev, clk_info[i].clk_name);
			if (IS_ERR(clk_ptr[i])) {
				pr_err("%s get failed\n", clk_info[i].clk_name);
				rc = PTR_ERR(clk_ptr[i]);
				goto cam_clk_get_err;
			}
			if (clk_info[i].clk_rate > 0) {
				clk_rate = clk_round_rate(clk_ptr[i],
					clk_info[i].clk_rate);
				if (clk_rate < 0) {
					pr_err("%s round failed\n",
						   clk_info[i].clk_name);
					goto cam_clk_set_err;
				}
				rc = clk_set_rate(clk_ptr[i],
					clk_rate);
				if (rc < 0) {
					pr_err("%s set failed\n",
						clk_info[i].clk_name);
					goto cam_clk_set_err;
				}

			} else if (clk_info[i].clk_rate == INIT_RATE) {
				clk_rate = clk_get_rate(clk_ptr[i]);
				if (clk_rate == 0) {
					clk_rate =
						  clk_round_rate(clk_ptr[i], 0);
					if (clk_rate < 0) {
						pr_err("%s round rate failed\n",
							  clk_info[i].clk_name);
						goto cam_clk_set_err;
					}
					rc = clk_set_rate(clk_ptr[i],
								clk_rate);
					if (rc < 0) {
						pr_err("%s set rate failed\n",
							  clk_info[i].clk_name);
						goto cam_clk_set_err;
					}
				}
			}
			rc = clk_prepare(clk_ptr[i]);
			if (rc < 0) {
				pr_err("%s prepare failed\n",
					   clk_info[i].clk_name);
				goto cam_clk_prepare_err;
			}

			rc = clk_enable(clk_ptr[i]);
			if (rc < 0) {
				pr_err("%s enable failed\n",
					   clk_info[i].clk_name);
				goto cam_clk_enable_err;
			}
			if (clk_info[i].clk_rate > 0 ||
			    (clk_info[i].clk_name &&
			     (!strcmp(clk_info[i].clk_name, "cam_src_clk") ||
			      !strcmp(clk_info[i].clk_name, "cam_clk") ||
			      !strcmp(clk_info[i].clk_name, "csi_src_clk"))))
				pr_info("%s: %s req=%ld get_rate=%lu\n",
					__func__, clk_info[i].clk_name,
					clk_info[i].clk_rate,
					clk_get_rate(clk_ptr[i]));
			if (clk_info[i].clk_name &&
			    !strcmp(clk_info[i].clk_name, "cam_src_clk"))
				msm_cam_dump_mclk0_pad("cam_src_clk", 1);
			if (clk_info[i].clk_name &&
			    !strcmp(clk_info[i].clk_name, "cam_clk"))
				msm_cam_dump_mclk0_pad("cam_clk", 1);
			if (clk_info[i].delay > 20) {
				msleep(clk_info[i].delay);
			} else if (clk_info[i].delay) {
				usleep_range(clk_info[i].delay * 1000,
					(clk_info[i].delay * 1000) + 1000);
			}
		}
	} else {
		for (i = num_clk - 1; i >= 0; i--) {
			if (!IS_ERR_OR_NULL(clk_ptr[i])) {
				CDBG("%s disable %s\n", __func__,
					clk_info[i].clk_name);
				clk_disable(clk_ptr[i]);
				clk_unprepare(clk_ptr[i]);
				clk_put(clk_ptr[i]);
				clk_ptr[i] = NULL;
			}
		}
	}
	return rc;


cam_clk_enable_err:
	clk_unprepare(clk_ptr[i]);
cam_clk_prepare_err:
cam_clk_set_err:
	clk_put(clk_ptr[i]);
cam_clk_get_err:
	for (i--; i >= 0; i--) {
		if (!IS_ERR_OR_NULL(clk_ptr[i])) {
			clk_disable(clk_ptr[i]);
			clk_unprepare(clk_ptr[i]);
			clk_put(clk_ptr[i]);
			clk_ptr[i] = NULL;
		}
	}
	return rc;
}

int msm_camera_config_vreg(struct device *dev, struct camera_vreg_t *cam_vreg,
		int num_vreg, enum msm_camera_vreg_name_t *vreg_seq,
		int num_vreg_seq, struct regulator **reg_ptr, int config)
{
	int i = 0, j = 0;
	int rc = 0;
	struct camera_vreg_t *curr_vreg;

	if (num_vreg_seq > num_vreg) {
		pr_err("%s:%d vreg sequence invalid\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (cam_vreg == NULL) {
		pr_err("%s:%d cam_vreg sequence invalid\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!num_vreg_seq)
		num_vreg_seq = num_vreg;

	if (config) {
		for (i = 0; i < num_vreg_seq; i++) {
			if (vreg_seq) {
				j = vreg_seq[i];
				if (j >= num_vreg)
					continue;
			} else
				j = i;
			curr_vreg = &cam_vreg[j];
			reg_ptr[j] = regulator_get(dev,
				curr_vreg->reg_name);
			if (IS_ERR(reg_ptr[j])) {
				pr_err("%s: %s get failed\n",
					 __func__,
					 curr_vreg->reg_name);
				reg_ptr[j] = NULL;
				goto vreg_get_fail;
			}
			if (regulator_count_voltages(reg_ptr[j]) > 0) {
				rc = regulator_set_voltage(
					reg_ptr[j],
					curr_vreg->min_voltage,
					curr_vreg->max_voltage);
				if (rc < 0) {
					pr_err("%s: %s set voltage failed\n",
						__func__,
						curr_vreg->reg_name);
					goto vreg_set_voltage_fail;
				}
				if (curr_vreg->op_mode >= 0) {
					rc = regulator_set_optimum_mode(
						reg_ptr[j],
						curr_vreg->op_mode);
					if (rc < 0) {
						pr_err(
						"%s:%s set optimum mode fail\n",
						__func__,
						curr_vreg->reg_name);
						goto vreg_set_opt_mode_fail;
					}
				}
			}
		}
	} else {
		for (i = num_vreg_seq-1; i >= 0; i--) {
			if (vreg_seq) {
				j = vreg_seq[i];
				if (j >= num_vreg)
					continue;
			} else
				j = i;
			curr_vreg = &cam_vreg[j];
			if (reg_ptr[j]) {
				if (regulator_count_voltages(reg_ptr[j]) > 0) {
					if (curr_vreg->op_mode >= 0) {
						regulator_set_optimum_mode(
							reg_ptr[j], 0);
					}
					regulator_set_voltage(
						reg_ptr[j], 0, curr_vreg->
						max_voltage);
				}
				regulator_put(reg_ptr[j]);
				reg_ptr[j] = NULL;
			}
		}
	}
	return 0;

vreg_unconfig:
if (regulator_count_voltages(reg_ptr[j]) > 0)
	regulator_set_optimum_mode(reg_ptr[j], 0);

vreg_set_opt_mode_fail:
if (regulator_count_voltages(reg_ptr[j]) > 0)
	regulator_set_voltage(reg_ptr[j], 0,
		curr_vreg->max_voltage);

vreg_set_voltage_fail:
	regulator_put(reg_ptr[j]);
	reg_ptr[j] = NULL;

vreg_get_fail:
	for (i--; i >= 0; i--) {
		if (vreg_seq) {
			j = vreg_seq[i];
			if (j >= num_vreg)
				continue;
		} else
			j = i;
		curr_vreg = &cam_vreg[j];
		goto vreg_unconfig;
	}
	return -ENODEV;
}

int msm_camera_enable_vreg(struct device *dev, struct camera_vreg_t *cam_vreg,
		int num_vreg, enum msm_camera_vreg_name_t *vreg_seq,
		int num_vreg_seq, struct regulator **reg_ptr, int enable)
{
	int i = 0, j = 0, rc = 0;

	if (num_vreg_seq > num_vreg) {
		pr_err("%s:%d vreg sequence invalid\n", __func__, __LINE__);
		return -EINVAL;
	}
	if (!num_vreg_seq)
		num_vreg_seq = num_vreg;

	if (enable) {
		for (i = 0; i < num_vreg_seq; i++) {
			if (vreg_seq) {
				j = vreg_seq[i];
				if (j >= num_vreg)
					continue;
			} else
				j = i;
			if (IS_ERR(reg_ptr[j])) {
				pr_err("%s: %s null regulator\n",
					__func__, cam_vreg[j].reg_name);
				goto disable_vreg;
			}
			rc = regulator_enable(reg_ptr[j]);
			if (rc < 0) {
				pr_err("%s: %s enable failed\n",
					__func__, cam_vreg[j].reg_name);
				goto disable_vreg;
			}
			if (cam_vreg[j].delay > 20)
				msleep(cam_vreg[j].delay);
			else if (cam_vreg[j].delay)
				usleep_range(cam_vreg[j].delay * 1000,
					(cam_vreg[j].delay * 1000) + 1000);
		}
	} else {
		for (i = num_vreg_seq-1; i >= 0; i--) {
			if (vreg_seq) {
				j = vreg_seq[i];
				if (j >= num_vreg)
					continue;
			} else
				j = i;
			regulator_disable(reg_ptr[j]);
			if (cam_vreg[j].delay > 20)
				msleep(cam_vreg[j].delay);
			else if (cam_vreg[j].delay)
				usleep_range(cam_vreg[j].delay * 1000,
					(cam_vreg[j].delay * 1000) + 1000);
		}
	}
	return rc;
disable_vreg:
	for (i--; i >= 0; i--) {
		if (vreg_seq) {
			j = vreg_seq[i];
			if (j >= num_vreg)
				continue;
		} else
			j = i;
		regulator_disable(reg_ptr[j]);
		if (cam_vreg[j].delay > 20)
			msleep(cam_vreg[j].delay);
		else if (cam_vreg[j].delay)
			usleep_range(cam_vreg[j].delay * 1000,
				(cam_vreg[j].delay * 1000) + 1000);
	}
	return rc;
}

void msm_camera_bus_scale_cfg(uint32_t bus_perf_client,
		enum msm_bus_perf_setting perf_setting)
{
	int rc = 0;
	if (!bus_perf_client) {
		pr_err("%s: Bus Client NOT Registered!!!\n", __func__);
		return;
	}

	switch (perf_setting) {
	case S_EXIT:
		rc = msm_bus_scale_client_update_request(bus_perf_client, 1);
		msm_bus_scale_unregister_client(bus_perf_client);
		break;
	case S_PREVIEW:
		rc = msm_bus_scale_client_update_request(bus_perf_client, 1);
		break;
	case S_VIDEO:
		rc = msm_bus_scale_client_update_request(bus_perf_client, 2);
		break;
	case S_CAPTURE:
		rc = msm_bus_scale_client_update_request(bus_perf_client, 3);
		break;
	case S_ZSL:
		rc = msm_bus_scale_client_update_request(bus_perf_client, 4);
		break;
	case S_LIVESHOT:
		rc = msm_bus_scale_client_update_request(bus_perf_client, 5);
		break;
	case S_DEFAULT:
		break;
	default:
		pr_err("%s: INVALID CASE\n", __func__);
	}
}

int msm_camera_set_gpio_table(struct msm_gpio_set_tbl *gpio_tbl,
	uint8_t gpio_tbl_size, int gpio_en)
{
	int rc = 0, i;

	if (gpio_en) {
		for (i = 0; i < gpio_tbl_size; i++) {
			gpio_set_value_cansleep(gpio_tbl[i].gpio,
				gpio_tbl[i].flags);
			usleep_range(gpio_tbl[i].delay,
				gpio_tbl[i].delay + 1000);
		}
	} else {
		for (i = gpio_tbl_size - 1; i >= 0; i--) {
			if (gpio_tbl[i].flags)
				gpio_set_value_cansleep(gpio_tbl[i].gpio,
					GPIOF_OUT_INIT_LOW);
		}
	}
	return rc;
}

int msm_camera_config_single_vreg(struct device *dev,
	struct camera_vreg_t *cam_vreg, struct regulator **reg_ptr, int config)
{
	int rc = 0;
	const char *vreg_name = NULL;

	if (!dev || !cam_vreg || !reg_ptr) {
		pr_err("%s: get failed NULL parameter\n", __func__);
		goto vreg_get_fail;
	}
	if (cam_vreg->type == VREG_TYPE_CUSTOM) {
		if (cam_vreg->custom_vreg_name == NULL) {
			pr_err("%s : can't find sub reg name",
				__func__);
			goto vreg_get_fail;
		}
		vreg_name = cam_vreg->custom_vreg_name;
	} else {
		if (cam_vreg->reg_name == NULL) {
			pr_err("%s : can't find reg name", __func__);
			goto vreg_get_fail;
		}
		vreg_name = cam_vreg->reg_name;
	}

	if (config) {
		CDBG("%s enable %s\n", __func__, vreg_name);
		*reg_ptr = regulator_get(dev, vreg_name);
		if (IS_ERR(*reg_ptr)) {
			pr_err("%s: %s get failed\n", __func__, vreg_name);
			*reg_ptr = NULL;
			goto vreg_get_fail;
		}
		if (regulator_count_voltages(*reg_ptr) > 0) {
			CDBG("%s: voltage min=%d, max=%d\n",
				__func__, cam_vreg->min_voltage,
				cam_vreg->max_voltage);
			rc = regulator_set_voltage(
				*reg_ptr, cam_vreg->min_voltage,
				cam_vreg->max_voltage);
			if (rc < 0) {
				pr_err("%s: %s set voltage failed\n",
					__func__, vreg_name);
				goto vreg_set_voltage_fail;
			}
			if (cam_vreg->op_mode >= 0) {
				rc = regulator_set_optimum_mode(*reg_ptr,
					cam_vreg->op_mode);
				if (rc < 0) {
					pr_err(
					"%s: %s set optimum mode failed\n",
					__func__, vreg_name);
					goto vreg_set_opt_mode_fail;
				}
			}
		}
		rc = regulator_enable(*reg_ptr);
		if (rc < 0) {
			pr_err("%s: %s regulator_enable failed\n", __func__,
				vreg_name);
			goto vreg_unconfig;
		}
		pr_info("%s: %s enable ok volt=%d is_en=%d min=%d max=%d op=%d\n",
			__func__, vreg_name,
			regulator_count_voltages(*reg_ptr) > 0 ?
				regulator_get_voltage(*reg_ptr) : -1,
			regulator_is_enabled(*reg_ptr),
			cam_vreg->min_voltage, cam_vreg->max_voltage,
			cam_vreg->op_mode);
	} else {
		CDBG("%s disable %s\n", __func__, vreg_name);
		if (*reg_ptr) {
			CDBG("%s disable %s\n", __func__, vreg_name);
			regulator_disable(*reg_ptr);
			if (regulator_count_voltages(*reg_ptr) > 0) {
				if (cam_vreg->op_mode >= 0)
					regulator_set_optimum_mode(*reg_ptr, 0);
				regulator_set_voltage(
					*reg_ptr, 0, cam_vreg->max_voltage);
			}
			regulator_put(*reg_ptr);
			*reg_ptr = NULL;
		} else {
			pr_err("%s can't disable %s\n", __func__, vreg_name);
		}
	}
	return 0;

vreg_unconfig:
if (regulator_count_voltages(*reg_ptr) > 0)
	regulator_set_optimum_mode(*reg_ptr, 0);

vreg_set_opt_mode_fail:
if (regulator_count_voltages(*reg_ptr) > 0)
	regulator_set_voltage(*reg_ptr, 0, cam_vreg->max_voltage);

vreg_set_voltage_fail:
	regulator_put(*reg_ptr);
	*reg_ptr = NULL;

vreg_get_fail:
	return -ENODEV;
}

int msm_camera_request_gpio_table(struct gpio *gpio_tbl, uint8_t size,
	int gpio_en)
{
	int rc = 0, i = 0, err = 0;

	if (!gpio_tbl || !size) {
		pr_err("%s:%d invalid gpio_tbl %pK / size %d\n", __func__,
			__LINE__, gpio_tbl, size);
		return -EINVAL;
	}
	for (i = 0; i < size; i++) {
		pr_info("%s: %s gpio %d flags=0x%lx %s\n", __func__,
			gpio_en ? "request" : "free",
			gpio_tbl[i].gpio, gpio_tbl[i].flags,
			gpio_tbl[i].label ? gpio_tbl[i].label : "");
	}
	if (gpio_en) {
		for (i = 0; i < size; i++) {
			err = gpio_request_one(gpio_tbl[i].gpio,
				gpio_tbl[i].flags, gpio_tbl[i].label);
			if (err) {
				/*
				* After GPIO request fails, contine to
				* apply new gpios, outout a error message
				* for driver bringup debug
				*/
				pr_err("%s:%d gpio %d:%s request fails\n",
					__func__, __LINE__,
					gpio_tbl[i].gpio, gpio_tbl[i].label);
			}
		}
		msm_cam_dump_mclk0_pad("gpio_request", 0);
	} else {
		gpio_free_array(gpio_tbl, size);
	}
	return rc;
}
