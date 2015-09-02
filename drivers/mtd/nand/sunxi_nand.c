/*
 * Copyright (C) 2013 Boris BREZILLON <b.brezillon.dev@gmail.com>
 *
 * Derived from:
 *	https://github.com/yuq/sunxi-nfc-mtd
 *	Copyright (C) 2013 Qiang Yu <yuq825@gmail.com>
 *
 *	https://github.com/hno/Allwinner-Info
 *	Copyright (C) 2013 Henrik Nordström <Henrik Nordström>
 *
 *	Copyright (C) 2013 Dmitriy B. <rzk333@gmail.com>
 *	Copyright (C) 2013 Sergey Lapin <slapin@ossfans.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mtd.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#define NFC_REG_CTL		0x0000
#define NFC_REG_ST		0x0004
#define NFC_REG_INT		0x0008
#define NFC_REG_TIMING_CTL	0x000C
#define NFC_REG_TIMING_CFG	0x0010
#define NFC_REG_ADDR_LOW	0x0014
#define NFC_REG_ADDR_HIGH	0x0018
#define NFC_REG_SECTOR_NUM	0x001C
#define NFC_REG_CNT		0x0020
#define NFC_REG_CMD		0x0024
#define NFC_REG_RCMD_SET	0x0028
#define NFC_REG_WCMD_SET	0x002C
#define NFC_REG_IO_DATA		0x0030
#define NFC_REG_ECC_CTL		0x0034
#define NFC_REG_ECC_ST		0x0038
#define NFC_REG_DEBUG		0x003C
#define NFC_REG_ECC_ERR_CNT(x)	((0x0040 + (x)) & ~0x3)
#define NFC_REG_USER_DATA(x)	(0x0050 + ((x) * 4))
#define NFC_REG_SPARE_AREA	0x00A0
#define NFC_REG_PAT_ID		0x00A4
#define NFC_RAM0_BASE		0x0400
#define NFC_RAM1_BASE		0x0800

/* define bit use in NFC_CTL */
#define NFC_EN			BIT(0)
#define NFC_RESET		BIT(1)
#define NFC_BUS_WIDTH_MSK	BIT(2)
#define NFC_BUS_WIDTH_8		(0 << 2)
#define NFC_BUS_WIDTH_16	(1 << 2)
#define NFC_RB_SEL_MSK		BIT(3)
#define NFC_RB_SEL(x)		((x) << 3)
#define NFC_CE_SEL_MSK		GENMASK(26, 24)
#define NFC_CE_SEL(x)		((x) << 24)
#define NFC_CE_CTL		BIT(6)
#define NFC_PAGE_SHIFT_MSK	GENMASK(11, 8)
#define NFC_PAGE_SHIFT(x)	(((x) < 10 ? 0 : (x) - 10) << 8)
#define NFC_SAM			BIT(12)
#define NFC_RAM_METHOD		BIT(14)
#define NFC_DEBUG_CTL		BIT(31)

/* define bit use in NFC_ST */
#define NFC_RB_B2R		BIT(0)
#define NFC_CMD_INT_FLAG	BIT(1)
#define NFC_DMA_INT_FLAG	BIT(2)
#define NFC_CMD_FIFO_STATUS	BIT(3)
#define NFC_STA			BIT(4)
#define NFC_NATCH_INT_FLAG	BIT(5)
#define NFC_RB_STATE(x)		BIT(x + 8)

/* define bit use in NFC_INT */
#define NFC_B2R_INT_ENABLE	BIT(0)
#define NFC_CMD_INT_ENABLE	BIT(1)
#define NFC_DMA_INT_ENABLE	BIT(2)
#define NFC_INT_MASK		(NFC_B2R_INT_ENABLE | \
				 NFC_CMD_INT_ENABLE | \
				 NFC_DMA_INT_ENABLE)

/* define bit use in NFC_TIMING_CTL */
#define NFC_TIMING_CTL_EDO	BIT(8)

/* define NFC_TIMING_CFG register layout */
#define NFC_TIMING_CFG(tWB, tADL, tWHR, tRHW, tCAD)		\
	(((tWB) & 0x3) | (((tADL) & 0x3) << 2) |		\
	(((tWHR) & 0x3) << 4) | (((tRHW) & 0x3) << 6) |		\
	(((tCAD) & 0x7) << 8))

/* define bit use in NFC_CMD */
#define NFC_CMD_LOW_BYTE_MSK	GENMASK(7, 0)
#define NFC_CMD_HIGH_BYTE_MSK	GENMASK(15, 8)
#define NFC_CMD(x)		(x)
#define NFC_ADR_NUM_MSK		GENMASK(18, 16)
#define NFC_ADR_NUM(x)		(((x) - 1) << 16)
#define NFC_SEND_ADR		BIT(19)
#define NFC_ACCESS_DIR		BIT(20)
#define NFC_DATA_TRANS		BIT(21)
#define NFC_SEND_CMD1		BIT(22)
#define NFC_WAIT_FLAG		BIT(23)
#define NFC_SEND_CMD2		BIT(24)
#define NFC_SEQ			BIT(25)
#define NFC_DATA_SWAP_METHOD	BIT(26)
#define NFC_ROW_AUTO_INC	BIT(27)
#define NFC_SEND_CMD3		BIT(28)
#define NFC_SEND_CMD4		BIT(29)
#define NFC_CMD_TYPE_MSK	GENMASK(31, 30)
#define NFC_NORMAL_OP		(0 << 30)
#define NFC_ECC_OP		(1 << 30)
#define NFC_PAGE_OP		(2 << 30)

/* define bit use in NFC_RCMD_SET */
#define NFC_READ_CMD_MSK	GENMASK(7, 0)
#define NFC_RND_READ_CMD0_MSK	GENMASK(15, 8)
#define NFC_RND_READ_CMD1_MSK	GENMASK(23, 16)

/* define bit use in NFC_WCMD_SET */
#define NFC_PROGRAM_CMD_MSK	GENMASK(7, 0)
#define NFC_RND_WRITE_CMD_MSK	GENMASK(15, 8)
#define NFC_READ_CMD0_MSK	GENMASK(23, 16)
#define NFC_READ_CMD1_MSK	GENMASK(31, 24)

/* define bit use in NFC_ECC_CTL */
#define NFC_ECC_EN		BIT(0)
#define NFC_ECC_PIPELINE	BIT(3)
#define NFC_ECC_EXCEPTION	BIT(4)
#define NFC_ECC_BLOCK_SIZE_MSK	BIT(5)
#define NFC_RANDOM_EN		BIT(9)
#define NFC_RANDOM_DIRECTION	BIT(10)
#define NFC_ECC_MODE_MSK	GENMASK(15, 12)
#define NFC_ECC_MODE(x)		((x) << 12)
#define NFC_RANDOM_SEED_MSK	GENMASK(30, 16)
#define NFC_RANDOM_SEED(x)	((x) << 16)

/* define bit use in NFC_ECC_ST */
#define NFC_ECC_ERR(x)		BIT(x)
#define NFC_ECC_PAT_FOUND(x)	BIT(x + 16)
#define NFC_ECC_ERR_CNT(b, x)	(((x) >> ((b) * 8)) & 0xff)

/* NFC_USER_DATA helper macros */
#define NFC_BUF_TO_USER_DATA(buf)	((buf)[0] | ((buf)[1] << 8) | \
					((buf)[2] << 16) | ((buf)[3] << 24))
#define NFC_USER_DATA_TO_BUF(buf, val)	\
	{				\
		(buf)[0] = val;		\
		(buf)[1] = val >> 8;	\
		(buf)[2] = val >> 16;	\
		(buf)[3] = val >> 24;	\
	}

#define NFC_DEFAULT_TIMEOUT_MS	1000

#define NFC_SRAM_SIZE		1024

#define NFC_MAX_CS		7

/*
 * Ready/Busy detection type: describes the Ready/Busy detection modes
 *
 * @RB_NONE:	no external detection available, rely on STATUS command
 *		and software timeouts
 * @RB_NATIVE:	use sunxi NAND controller Ready/Busy support. The Ready/Busy
 *		pin of the NAND flash chip must be connected to one of the
 *		native NAND R/B pins (those which can be muxed to the NAND
 *		Controller)
 * @RB_GPIO:	use a simple GPIO to handle Ready/Busy status. The Ready/Busy
 *		pin of the NAND flash chip must be connected to a GPIO capable
 *		pin.
 */
enum sunxi_nand_rb_type {
	RB_NONE,
	RB_NATIVE,
	RB_GPIO,
};

/*
 * Ready/Busy structure: stores information related to Ready/Busy detection
 *
 * @type:	the Ready/Busy detection mode
 * @info:	information related to the R/B detection mode. Either a gpio
 *		id or a native R/B id (those supported by the NAND controller).
 */
struct sunxi_nand_rb {
	enum sunxi_nand_rb_type type;
	union {
		int gpio;
		int nativeid;
	} info;
};

/*
 * Chip Select structure: stores information related to NAND Chip Select
 *
 * @cs:		the NAND CS id used to communicate with a NAND Chip
 * @rb:		the Ready/Busy description
 */
struct sunxi_nand_chip_sel {
	u8 cs;
	struct sunxi_nand_rb rb;
};

/*
 * sunxi HW ECC infos: stores information related to HW ECC support
 *
 * @mode:	the sunxi ECC mode field deduced from ECC requirements
 * @layout:	the OOB layout depending on the ECC requirements and the
 *		selected ECC mode
 */
struct sunxi_nand_hw_ecc {
	int mode;
	struct nand_ecclayout layout;
};

/*
 * NAND chip structure: stores NAND chip device related information
 *
 * @node:		used to store NAND chips into a list
 * @nand:		base NAND chip structure
 * @mtd:		base MTD structure
 * @randomize:		randomize data
 * @clk_rate:		clk_rate required for this NAND chip
 * @timing_cfg		TIMING_CFG register value for this NAND chip
 * @selected:		current active CS
 * @nsels:		number of CS lines required by the NAND chip
 * @sels:		array of CS lines descriptions
 */
struct sunxi_nand_chip {
	struct list_head node;
	struct nand_chip nand;
	struct mtd_info mtd;
	bool randomize;
	unsigned long clk_rate;
	u32 timing_cfg;
	u32 timing_ctl;
	int selected;
	int nsels;
	struct sunxi_nand_chip_sel sels[0];
};

static inline struct sunxi_nand_chip *to_sunxi_nand(struct nand_chip *nand)
{
	return container_of(nand, struct sunxi_nand_chip, nand);
}

/*
 * NAND Controller structure: stores sunxi NAND controller information
 *
 * @controller:		base controller structure
 * @dev:		parent device (used to print error messages)
 * @regs:		NAND controller registers
 * @ahb_clk:		NAND Controller AHB clock
 * @mod_clk:		NAND Controller mod clock
 * @assigned_cs:	bitmask describing already assigned CS lines
 * @clk_rate:		NAND controller current clock rate
 * @chips:		a list containing all the NAND chips attached to
 *			this NAND controller
 * @complete:		a completion object used to wait for NAND
 *			controller events
 */
struct sunxi_nfc {
	struct nand_hw_control controller;
	struct device *dev;
	void __iomem *regs;
	struct clk *ahb_clk;
	struct clk *mod_clk;
	unsigned long assigned_cs;
	unsigned long clk_rate;
	struct list_head chips;
	struct completion complete;
};

static inline struct sunxi_nfc *to_sunxi_nfc(struct nand_hw_control *ctrl)
{
	return container_of(ctrl, struct sunxi_nfc, controller);
}

static irqreturn_t sunxi_nfc_interrupt(int irq, void *dev_id)
{
	struct sunxi_nfc *nfc = dev_id;
	u32 st = readl(nfc->regs + NFC_REG_ST);
	u32 ien = readl(nfc->regs + NFC_REG_INT);

	if (!(ien & st))
		return IRQ_NONE;

	if ((ien & st) == ien)
		complete(&nfc->complete);

	writel(st & NFC_INT_MASK, nfc->regs + NFC_REG_ST);
	writel(~st & ien & NFC_INT_MASK, nfc->regs + NFC_REG_INT);

	return IRQ_HANDLED;
}

static int sunxi_nfc_wait_int(struct sunxi_nfc *nfc, u32 flags,
			      unsigned int timeout_ms)
{
	init_completion(&nfc->complete);

	writel(flags, nfc->regs + NFC_REG_INT);

	if (!timeout_ms)
		timeout_ms = NFC_DEFAULT_TIMEOUT_MS;

	if (!wait_for_completion_timeout(&nfc->complete,
					 msecs_to_jiffies(timeout_ms))) {
		dev_err(nfc->dev, "wait interrupt timedout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int sunxi_nfc_wait_cmd_fifo_empty(struct sunxi_nfc *nfc)
{
	unsigned long timeout = jiffies +
				msecs_to_jiffies(NFC_DEFAULT_TIMEOUT_MS);

	do {
		if (!(readl(nfc->regs + NFC_REG_ST) & NFC_CMD_FIFO_STATUS))
			return 0;
	} while (time_before(jiffies, timeout));

	dev_err(nfc->dev, "wait for empty cmd FIFO timedout\n");
	return -ETIMEDOUT;
}

static int sunxi_nfc_rst(struct sunxi_nfc *nfc)
{
	unsigned long timeout = jiffies +
				msecs_to_jiffies(NFC_DEFAULT_TIMEOUT_MS);

	writel(0, nfc->regs + NFC_REG_ECC_CTL);
	writel(NFC_RESET, nfc->regs + NFC_REG_CTL);

	do {
		if (!(readl(nfc->regs + NFC_REG_CTL) & NFC_RESET))
			return 0;
	} while (time_before(jiffies, timeout));

	dev_err(nfc->dev, "wait for NAND controller reset timedout\n");
	return -ETIMEDOUT;
}

static int sunxi_nfc_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	struct sunxi_nand_rb *rb;
	unsigned long timeo = (sunxi_nand->nand.state == FL_ERASING ? 400 : 20);
	int ret;

	if (sunxi_nand->selected < 0)
		return 0;

	rb = &sunxi_nand->sels[sunxi_nand->selected].rb;

	switch (rb->type) {
	case RB_NATIVE:
		ret = !!(readl(nfc->regs + NFC_REG_ST) &
			 NFC_RB_STATE(rb->info.nativeid));
		if (ret)
			break;

		sunxi_nfc_wait_int(nfc, NFC_RB_B2R, timeo);
		ret = !!(readl(nfc->regs + NFC_REG_ST) &
			 NFC_RB_STATE(rb->info.nativeid));
		break;
	case RB_GPIO:
		ret = gpio_get_value(rb->info.gpio);
		break;
	case RB_NONE:
	default:
		ret = 0;
		dev_err(nfc->dev, "cannot check R/B NAND status!\n");
		break;
	}

	return ret;
}

static void sunxi_nfc_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	struct sunxi_nand_chip_sel *sel;
	u32 ctl;

	if (chip > 0 && chip >= sunxi_nand->nsels)
		return;

	if (chip == sunxi_nand->selected)
		return;

	ctl = readl(nfc->regs + NFC_REG_CTL) &
	      ~(NFC_PAGE_SHIFT_MSK | NFC_CE_SEL_MSK | NFC_RB_SEL_MSK | NFC_EN);

	if (chip >= 0) {
		sel = &sunxi_nand->sels[chip];

		ctl |= NFC_CE_SEL(sel->cs) | NFC_EN |
		       NFC_PAGE_SHIFT(nand->page_shift - 10);
		if (sel->rb.type == RB_NONE) {
			nand->dev_ready = NULL;
		} else {
			nand->dev_ready = sunxi_nfc_dev_ready;
			if (sel->rb.type == RB_NATIVE)
				ctl |= NFC_RB_SEL(sel->rb.info.nativeid);
		}

		writel(mtd->writesize, nfc->regs + NFC_REG_SPARE_AREA);

		if (nfc->clk_rate != sunxi_nand->clk_rate) {
			clk_set_rate(nfc->mod_clk, sunxi_nand->clk_rate);
			nfc->clk_rate = sunxi_nand->clk_rate;
		}
	}

	writel(sunxi_nand->timing_ctl, nfc->regs + NFC_REG_TIMING_CTL);
	writel(sunxi_nand->timing_cfg, nfc->regs + NFC_REG_TIMING_CFG);
	writel(ctl, nfc->regs + NFC_REG_CTL);

	sunxi_nand->selected = chip;
}

static void sunxi_nfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	int ret;
	int cnt;
	int offs = 0;
	u32 tmp;

	while (len > offs) {
		cnt = min(len - offs, NFC_SRAM_SIZE);

		ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
		if (ret)
			break;

		writel(cnt, nfc->regs + NFC_REG_CNT);
		tmp = NFC_DATA_TRANS | NFC_DATA_SWAP_METHOD;
		writel(tmp, nfc->regs + NFC_REG_CMD);

		ret = sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
		if (ret)
			break;

		if (buf)
			memcpy_fromio(buf + offs, nfc->regs + NFC_RAM0_BASE,
				      cnt);
		offs += cnt;
	}
}

static void sunxi_nfc_write_buf(struct mtd_info *mtd, const uint8_t *buf,
				int len)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	int ret;
	int cnt;
	int offs = 0;
	u32 tmp;

	while (len > offs) {
		cnt = min(len - offs, NFC_SRAM_SIZE);

		ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
		if (ret)
			break;

		writel(cnt, nfc->regs + NFC_REG_CNT);
		memcpy_toio(nfc->regs + NFC_RAM0_BASE, buf + offs, cnt);
		tmp = NFC_DATA_TRANS | NFC_DATA_SWAP_METHOD |
		      NFC_ACCESS_DIR;
		writel(tmp, nfc->regs + NFC_REG_CMD);

		ret = sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
		if (ret)
			break;

		offs += cnt;
	}
}

static uint8_t sunxi_nfc_read_byte(struct mtd_info *mtd)
{
	uint8_t ret;

	sunxi_nfc_read_buf(mtd, &ret, 1);

	return ret;
}

static void sunxi_nfc_cmd_ctrl(struct mtd_info *mtd, int dat,
			       unsigned int ctrl)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	int ret;
	u32 tmp;

	ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
	if (ret)
		return;

	if (ctrl & NAND_CTRL_CHANGE) {
		tmp = readl(nfc->regs + NFC_REG_CTL);
		if (ctrl & NAND_NCE)
			tmp |= NFC_CE_CTL;
		else
			tmp &= ~NFC_CE_CTL;
		writel(tmp, nfc->regs + NFC_REG_CTL);
	}

	if (dat == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE) {
		writel(NFC_SEND_CMD1 | dat, nfc->regs + NFC_REG_CMD);
	} else {
		writel(dat, nfc->regs + NFC_REG_ADDR_LOW);
		writel(NFC_SEND_ADR, nfc->regs + NFC_REG_CMD);
	}

	sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
}

static const u16 sunxi_nfc_randomizer_page_seeds[] = {
	0x2b75, 0x0bd0, 0x5ca3, 0x62d1, 0x1c93, 0x07e9, 0x2162, 0x3a72,
	0x0d67, 0x67f9, 0x1be7, 0x077d, 0x032f, 0x0dac, 0x2716, 0x2436,
	0x7922, 0x1510, 0x3860, 0x5287, 0x480f, 0x4252, 0x1789, 0x5a2d,
	0x2a49, 0x5e10, 0x437f, 0x4b4e, 0x2f45, 0x216e, 0x5cb7, 0x7130,
	0x2a3f, 0x60e4, 0x4dc9, 0x0ef0, 0x0f52, 0x1bb9, 0x6211, 0x7a56,
	0x226d, 0x4ea7, 0x6f36, 0x3692, 0x38bf, 0x0c62, 0x05eb, 0x4c55,
	0x60f4, 0x728c, 0x3b6f, 0x2037, 0x7f69, 0x0936, 0x651a, 0x4ceb,
	0x6218, 0x79f3, 0x383f, 0x18d9, 0x4f05, 0x5c82, 0x2912, 0x6f17,
	0x6856, 0x5938, 0x1007, 0x61ab, 0x3e7f, 0x57c2, 0x542f, 0x4f62,
	0x7454, 0x2eac, 0x7739, 0x42d4, 0x2f90, 0x435a, 0x2e52, 0x2064,
	0x637c, 0x66ad, 0x2c90, 0x0bad, 0x759c, 0x0029, 0x0986, 0x7126,
	0x1ca7, 0x1605, 0x386a, 0x27f5, 0x1380, 0x6d75, 0x24c3, 0x0f8e,
	0x2b7a, 0x1418, 0x1fd1, 0x7dc1, 0x2d8e, 0x43af, 0x2267, 0x7da3,
	0x4e3d, 0x1338, 0x50db, 0x454d, 0x764d, 0x40a3, 0x42e6, 0x262b,
	0x2d2e, 0x1aea, 0x2e17, 0x173d, 0x3a6e, 0x71bf, 0x25f9, 0x0a5d,
	0x7c57, 0x0fbe, 0x46ce, 0x4939, 0x6b17, 0x37bb, 0x3e91, 0x76db,
};

static const u16 sunxi_nfc_randomizer_ecc512_seeds[] = {
	0x227b, 0x5baa, 0x15ef, 0x5b13, 0x75b8, 0x04f7, 0x4b5e, 0x4db8,
	0x4f31, 0x669a, 0x20af, 0x5b91, 0x1d6d, 0x286a, 0x5e98, 0x07f1,
	0x44b2, 0x0288, 0x3996, 0x7a98, 0x76cc, 0x705a, 0x21c9, 0x5be5,
	0x23b8, 0x7133, 0x5514, 0x6f0b, 0x7528, 0x5e5b, 0x2ae1, 0x37b6,
	0x0d19, 0x1525, 0x2f87, 0x0025, 0x0107, 0x7012, 0x0b4f, 0x12e1,
	0x4e29, 0x6a5e, 0x744a, 0x0032, 0x61c3, 0x1463, 0x3cd4, 0x5636,
	0x5928, 0x57bf, 0x3ce1, 0x6c35, 0x4167, 0x58cc, 0x7e7f, 0x4ecd,
	0x05c9, 0x3063, 0x01ab, 0x194f, 0x6b7c, 0x6575, 0x29f5, 0x04d0,
	0x1e53, 0x4d18, 0x2f12, 0x21c4, 0x3379, 0x12ba, 0x660a, 0x6150,
	0x2f0e, 0x7339, 0x3bd6, 0x5ab0, 0x72fa, 0x568d, 0x5bf6, 0x410d,
	0x1e35, 0x2971, 0x3389, 0x420c, 0x5985, 0x169c, 0x6cb3, 0x3139,
	0x52ac, 0x1475, 0x6611, 0x47df, 0x2c0b, 0x14dd, 0x1839, 0x0853,
	0x667f, 0x245f, 0x3d7e, 0x5eb1, 0x13d1, 0x4945, 0x11ae, 0x4f1e,
	0x19bc, 0x7e72, 0x1206, 0x3a64, 0x6c27, 0x1d33, 0x3726, 0x37db,
	0x6ba3, 0x1dfb, 0x7041, 0x66b5, 0x14b0, 0x13da, 0x5378, 0x0696,
	0x7887, 0x5c44, 0x4a7e, 0x2888, 0x0794, 0x567f, 0x57bb, 0x0ac0,
};

static const u16 sunxi_nfc_randomizer_ecc1024_seeds[] = {
	0x48a6, 0x595e, 0x42c7, 0x63a7, 0x329c, 0x250d, 0x2306, 0x2324,
	0x4c68, 0x5f64, 0x2656, 0x3a4e, 0x1da6, 0x6b01, 0x775d, 0x2efc,
	0x3afa, 0x07cc, 0x378e, 0x6e39, 0x64cf, 0x00ce, 0x042b, 0x2d6e,
	0x092a, 0x66b7, 0x45cb, 0x5dd8, 0x1ef9, 0x7600, 0x3dcc, 0x2bfa,
	0x0796, 0x4d1c, 0x1cf5, 0x0399, 0x62d3, 0x56fc, 0x33cb, 0x2c74,
	0x26b2, 0x79f7, 0x4f87, 0x6d96, 0x6fec, 0x777b, 0x7dae, 0x2523,
	0x0115, 0x0b1c, 0x323a, 0x6438, 0x7ebd, 0x7a81, 0x0739, 0x1df7,
	0x1d4f, 0x6e1f, 0x0fa4, 0x3f79, 0x18bd, 0x1255, 0x43b1, 0x1f15,
	0x2446, 0x5b56, 0x2b13, 0x79ae, 0x3ce6, 0x674a, 0x7301, 0x1288,
	0x721b, 0x3892, 0x0618, 0x0a05, 0x791e, 0x265b, 0x3062, 0x0914,
	0x46ea, 0x50da, 0x38ad, 0x20e7, 0x04e2, 0x3696, 0x4ed2, 0x0d70,
	0x5585, 0x71f4, 0x080b, 0x2e22, 0x28eb, 0x6f80, 0x5109, 0x2bb0,
	0x0ca1, 0x2159, 0x1a8a, 0x61c1, 0x38b2, 0x59ae, 0x1937, 0x1076,
	0x6a33, 0x7abc, 0x4630, 0x4b37, 0x11b4, 0x4d1b, 0x079f, 0x7e51,
	0x40e8, 0x7b41, 0x7bc4, 0x037a, 0x3a2b, 0x0fb5, 0x7a06, 0x6401,
	0x6792, 0x7fab, 0x7bcd, 0x1ac6, 0x1d51, 0x1b91, 0x781f, 0x5776,
};

static u16 sunxi_nfc_randomizer_step(u16 state, int count)
{
	state &= 0x7fff;
	while (count--)
		state = ((state >> 1) |
			 ((((state >> 0) ^ (state >> 1)) & 1) << 14)) & 0x7fff;

	return state;
}

static u16 sunxi_nfc_randomizer_state(struct mtd_info *mtd, int page, bool ecc)
{
	const u16 *seeds = sunxi_nfc_randomizer_page_seeds;
	int mod = mtd->erasesize / mtd->writesize;

	if (mod > ARRAY_SIZE(sunxi_nfc_randomizer_page_seeds))
		mod = ARRAY_SIZE(sunxi_nfc_randomizer_page_seeds);

	if (ecc) {
		if (mtd->ecc_step_size == 512)
			seeds = sunxi_nfc_randomizer_ecc512_seeds;
		else
			seeds = sunxi_nfc_randomizer_ecc1024_seeds;
	}

	return seeds[page % mod];
}

static void sunxi_nfc_randomizer_config(struct mtd_info *mtd,
					int page, bool ecc)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nfc *nfc = to_sunxi_nfc(nand->controller);
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	u32 ecc_ctl = readl(nfc->regs + NFC_REG_ECC_CTL);
	u16 state;

	if (!sunxi_nand->randomize)
		return;

	ecc_ctl = readl(nfc->regs + NFC_REG_ECC_CTL);
	state = sunxi_nfc_randomizer_state(mtd, page, ecc);
	ecc_ctl = readl(nfc->regs + NFC_REG_ECC_CTL) & ~NFC_RANDOM_SEED_MSK;
	writel(ecc_ctl | NFC_RANDOM_SEED(state), nfc->regs + NFC_REG_ECC_CTL);
}

static void sunxi_nfc_randomizer_enable(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nfc *nfc = to_sunxi_nfc(nand->controller);
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);

	if (!sunxi_nand->randomize)
		return;

	writel(readl(nfc->regs + NFC_REG_ECC_CTL) | NFC_RANDOM_EN,
	       nfc->regs + NFC_REG_ECC_CTL);
}

static void sunxi_nfc_randomizer_disable(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nfc *nfc = to_sunxi_nfc(nand->controller);
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);

	if (!sunxi_nand->randomize)
		return;

	writel(readl(nfc->regs + NFC_REG_ECC_CTL) & ~NFC_RANDOM_EN,
	       nfc->regs + NFC_REG_ECC_CTL);
}

static void sunxi_nfc_randomize_bbm(struct mtd_info *mtd, int page, u8 *bbm)
{
	u16 state = sunxi_nfc_randomizer_state(mtd, page, true);

	state = sunxi_nfc_randomizer_step(state, 15);
	bbm[0] ^= state;
	state = sunxi_nfc_randomizer_step(state, 8);
	bbm[1] ^= state;
}

static void sunxi_nfc_randomizer_write_buf(struct mtd_info *mtd,
					   const uint8_t *buf, int len,
					   bool ecc, int page)
{
	sunxi_nfc_randomizer_config(mtd, page, ecc);
	sunxi_nfc_randomizer_enable(mtd);
	sunxi_nfc_write_buf(mtd, buf, len);
	sunxi_nfc_randomizer_disable(mtd);
}

static void sunxi_nfc_randomizer_read_buf(struct mtd_info *mtd, uint8_t *buf,
					  int len, bool ecc, int page)
{
	sunxi_nfc_randomizer_config(mtd, page, ecc);
	sunxi_nfc_randomizer_enable(mtd);
	sunxi_nfc_read_buf(mtd, buf, len);
	sunxi_nfc_randomizer_disable(mtd);
}

static void sunxi_nfc_hw_ecc_enable(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nfc *nfc = to_sunxi_nfc(nand->controller);
	struct sunxi_nand_hw_ecc *data = nand->ecc.priv;
	u32 ecc_ctl;

	ecc_ctl = readl(nfc->regs + NFC_REG_ECC_CTL);
	ecc_ctl &= ~(NFC_ECC_MODE_MSK | NFC_ECC_PIPELINE |
		     NFC_ECC_BLOCK_SIZE_MSK);
	ecc_ctl |= NFC_ECC_EN | NFC_ECC_MODE(data->mode) | NFC_ECC_EXCEPTION;

	writel(ecc_ctl, nfc->regs + NFC_REG_ECC_CTL);
}

static void sunxi_nfc_hw_ecc_disable(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nfc *nfc = to_sunxi_nfc(nand->controller);

	writel(readl(nfc->regs + NFC_REG_ECC_CTL) & ~NFC_ECC_EN,
	       nfc->regs + NFC_REG_ECC_CTL);
}

static int sunxi_nfc_hw_ecc_read_chunk(struct mtd_info *mtd,
				       u8 *data, int data_off,
				       u8 *oob, int oob_off,
				       int *cur_off,
				       unsigned int *max_bitflips,
				       bool bbm, int page)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(nand->controller);
	struct nand_ecc_ctrl *ecc = &nand->ecc;
	int raw_mode = 0;
	u32 status;
	int ret;

	if (*cur_off != data_off)
		nand->cmdfunc(mtd, NAND_CMD_RNDOUT, data_off, -1);

	sunxi_nfc_randomizer_read_buf(mtd, data, ecc->size, false, page);

	if (data_off + ecc->bytes != oob_off)
		nand->cmdfunc(mtd, NAND_CMD_RNDOUT, oob_off, -1);

	ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
	if (ret)
		return ret;

	sunxi_nfc_randomizer_enable(mtd);
	writel(NFC_DATA_TRANS | NFC_DATA_SWAP_METHOD | NFC_ECC_OP,
	       nfc->regs + NFC_REG_CMD);
	sunxi_nfc_randomizer_disable(mtd);

	ret = sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
	if (ret)
		return ret;

	*cur_off = oob_off + ecc->bytes + 4;

	status = readl(nfc->regs + NFC_REG_ECC_ST);
	if (status & NFC_ECC_PAT_FOUND(0)) {
		u8 pattern = 0xff;

		if (unlikely(!(readl(nfc->regs + NFC_REG_PAT_ID) & 0x1)))
			pattern = 0x0;

		memset(data, pattern, ecc->size);
		memset(oob, pattern, ecc->bytes + 4);

		return 1;
	}

	ret = NFC_ECC_ERR_CNT(0, readl(nfc->regs + NFC_REG_ECC_ERR_CNT(0)));

	memcpy_fromio(data, nfc->regs + NFC_RAM0_BASE, ecc->size);

	nand->cmdfunc(mtd, NAND_CMD_RNDOUT, oob_off, -1);
	sunxi_nfc_randomizer_read_buf(mtd, oob, ecc->bytes + 4, true, page);

	if (status & NFC_ECC_ERR(0)) {
		/*
		 * Re-read the data with the randomizer disabled to identify
		 * bitflips in erased pages.
		 */
		if (sunxi_nand->randomize) {
			nand->cmdfunc(mtd, NAND_CMD_RNDOUT, data_off, -1);
			nand->read_buf(mtd, data, ecc->size);
			nand->cmdfunc(mtd, NAND_CMD_RNDOUT, oob_off, -1);
			nand->read_buf(mtd, oob, ecc->bytes + 4);
		}

		ret = nand_check_erased_ecc_chunk(data,	ecc->size,
						  oob, ecc->bytes + 4,
						  NULL, 0, ecc->strength);
		if (ret >= 0)
			raw_mode = 1;
	} else {
		/*
		 * The engine protects 4 bytes of OOB data per chunk.
		 * Retrieve the corrected OOB bytes.
		 */
		NFC_USER_DATA_TO_BUF(oob,
				     readl(nfc->regs + NFC_REG_USER_DATA(0)));

		/* De-randomize the Bad Block Marker. */
		if (bbm && sunxi_nand->randomize)
			sunxi_nfc_randomize_bbm(mtd, page, oob);
	}

	if (ret < 0) {
		mtd->ecc_stats.failed++;
	} else {
		mtd->ecc_stats.corrected += ret;
		*max_bitflips = max_t(unsigned int, *max_bitflips, ret);
	}

	*cur_off = oob_off + ecc->bytes + 4;

	return raw_mode;
}

static void sunxi_nfc_hw_ecc_read_extra_oob(struct mtd_info *mtd,
					    u8 *oob, int *cur_off,
					    bool randomize, int page)
{
	struct nand_chip *nand = mtd->priv;
	struct nand_ecc_ctrl *ecc = &nand->ecc;
	int offset = ((ecc->bytes + 4) * ecc->steps);
	int len = mtd->oobsize - offset;

	if (len <= 0)
		return;

	if (*cur_off != offset)
		nand->cmdfunc(mtd, NAND_CMD_RNDOUT,
			      offset + mtd->writesize, -1);

	if (!randomize)
		sunxi_nfc_read_buf(mtd, oob + offset, len);
	else
		sunxi_nfc_randomizer_read_buf(mtd, oob + offset, len,
					      false, page);

	*cur_off = mtd->oobsize + mtd->writesize;
}

static int sunxi_nfc_hw_ecc_write_chunk(struct mtd_info *mtd,
					const u8 *data, int data_off,
					const u8 *oob, int oob_off,
					int *cur_off, bool bbm,
					int page)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(nand->controller);
	struct nand_ecc_ctrl *ecc = &nand->ecc;
	int ret;

	if (data_off != *cur_off)
		nand->cmdfunc(mtd, NAND_CMD_RNDIN, data_off, -1);

	sunxi_nfc_randomizer_write_buf(mtd, data, ecc->size, false, page);

	/* Fill OOB data in */
	if (sunxi_nand->randomize && bbm) {
		u8 user_data[4];

		memcpy(user_data, oob, 4);
		sunxi_nfc_randomize_bbm(mtd, page, user_data);
		writel(NFC_BUF_TO_USER_DATA(user_data),
		       nfc->regs + NFC_REG_USER_DATA(0));
	} else {
		writel(NFC_BUF_TO_USER_DATA(oob),
		       nfc->regs + NFC_REG_USER_DATA(0));
	}

	if (data_off + ecc->bytes != oob_off)
		nand->cmdfunc(mtd, NAND_CMD_RNDIN, oob_off, -1);

	ret = sunxi_nfc_wait_cmd_fifo_empty(nfc);
	if (ret)
		return ret;

	sunxi_nfc_randomizer_enable(mtd);
	writel(NFC_DATA_TRANS | NFC_DATA_SWAP_METHOD |
	       NFC_ACCESS_DIR | NFC_ECC_OP,
	       nfc->regs + NFC_REG_CMD);
	sunxi_nfc_randomizer_disable(mtd);

	ret = sunxi_nfc_wait_int(nfc, NFC_CMD_INT_FLAG, 0);
	if (ret)
		return ret;

	*cur_off = oob_off + ecc->bytes + 4;

	return 0;
}

static void sunxi_nfc_hw_ecc_write_extra_oob(struct mtd_info *mtd,
					     u8 *oob, int *cur_off,
					     int page)
{
	struct nand_chip *nand = mtd->priv;
	struct nand_ecc_ctrl *ecc = &nand->ecc;
	int offset = ((ecc->bytes + 4) * ecc->steps);
	int len = mtd->oobsize - offset;

	if (len <= 0)
		return;

	if (*cur_off != offset)
		nand->cmdfunc(mtd, NAND_CMD_RNDIN,
			      offset + mtd->writesize, -1);

	sunxi_nfc_randomizer_write_buf(mtd, oob + offset, len, false, page);

	*cur_off = mtd->oobsize + mtd->writesize;
}

static int sunxi_nfc_hw_ecc_read_page(struct mtd_info *mtd,
				      struct nand_chip *chip, uint8_t *buf,
				      int oob_required, int page)
{
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	unsigned int max_bitflips = 0;
	int ret, i, cur_off = 0;
	bool raw_mode = false;

	sunxi_nfc_hw_ecc_enable(mtd);

	for (i = 0; i < ecc->steps; i++) {
		int data_off = i * ecc->size;
		int oob_off = i * (ecc->bytes + 4);
		u8 *data = buf + data_off;
		u8 *oob = chip->oob_poi + oob_off;

		ret = sunxi_nfc_hw_ecc_read_chunk(mtd, data, data_off, oob,
						  oob_off + mtd->writesize,
						  &cur_off, &max_bitflips,
						  !i, page);
		if (ret < 0)
			return ret;
		else if (ret)
			raw_mode = true;
	}

	if (oob_required)
		sunxi_nfc_hw_ecc_read_extra_oob(mtd, chip->oob_poi, &cur_off,
						!raw_mode, page);

	sunxi_nfc_hw_ecc_disable(mtd);

	return max_bitflips;
}

static int sunxi_nfc_hw_ecc_write_page(struct mtd_info *mtd,
				       struct nand_chip *chip,
				       const uint8_t *buf, int oob_required,
				       int page)
{
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(chip);
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	int ret, i, cur_off = 0;

	sunxi_nfc_hw_ecc_enable(mtd);

	for (i = 0; i < ecc->steps; i++) {
		int data_off = i * ecc->size;
		int oob_off = i * (ecc->bytes + 4);
		const u8 *data = buf + data_off;
		const u8 *oob = chip->oob_poi + oob_off;

		ret = sunxi_nfc_hw_ecc_write_chunk(mtd, data, data_off, oob,
						   oob_off + mtd->writesize,
						   &cur_off, !i, page);
		if (ret)
			return ret;
	}

	if (oob_required || sunxi_nand->randomize)
		sunxi_nfc_hw_ecc_write_extra_oob(mtd, chip->oob_poi,
						 &cur_off, page);

	sunxi_nfc_hw_ecc_disable(mtd);

	return 0;
}

static int sunxi_nfc_hw_syndrome_ecc_read_page(struct mtd_info *mtd,
					       struct nand_chip *chip,
					       uint8_t *buf, int oob_required,
					       int page)
{
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	unsigned int max_bitflips = 0;
	int ret, i, cur_off = 0;
	bool raw_mode = false;

	sunxi_nfc_hw_ecc_enable(mtd);

	for (i = 0; i < ecc->steps; i++) {
		int data_off = i * (ecc->size + ecc->bytes + 4);
		int oob_off = data_off + ecc->size;
		u8 *data = buf + (i * ecc->size);
		u8 *oob = chip->oob_poi + (i * (ecc->bytes + 4));

		ret = sunxi_nfc_hw_ecc_read_chunk(mtd, data, data_off, oob,
						  oob_off, &cur_off,
						  &max_bitflips, !i, page);
		if (ret < 0)
			return ret;
		else if (ret)
			raw_mode = true;
	}

	if (oob_required)
		sunxi_nfc_hw_ecc_read_extra_oob(mtd, chip->oob_poi, &cur_off,
						!raw_mode, page);

	sunxi_nfc_hw_ecc_disable(mtd);

	return max_bitflips;
}

static int sunxi_nfc_hw_syndrome_ecc_write_page(struct mtd_info *mtd,
						struct nand_chip *chip,
						const uint8_t *buf,
						int oob_required, int page)
{
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(chip);
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	int ret, i, cur_off = 0;

	sunxi_nfc_hw_ecc_enable(mtd);

	for (i = 0; i < ecc->steps; i++) {
		int data_off = i * (ecc->size + ecc->bytes + 4);
		int oob_off = data_off + ecc->size;
		const u8 *data = buf + (i * ecc->size);
		const u8 *oob = chip->oob_poi + (i * (ecc->bytes + 4));

		ret = sunxi_nfc_hw_ecc_write_chunk(mtd, data, data_off,
						   oob, oob_off, &cur_off,
						   false, page);
		if (ret)
			return ret;
	}

	if (oob_required || sunxi_nand->randomize)
		sunxi_nfc_hw_ecc_write_extra_oob(mtd, chip->oob_poi,
						 &cur_off, page);

	sunxi_nfc_hw_ecc_disable(mtd);

	return 0;
}

static int sunxi_nfc_hw_common_ecc_read_oob(struct mtd_info *mtd,
					    struct nand_chip *chip,
					    int page)
{
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);

	chip->pagebuf = -1;

	return chip->ecc.read_page(mtd, chip, chip->buffers->databuf, 1, page);
}

static int sunxi_nfc_hw_common_ecc_write_oob(struct mtd_info *mtd,
					     struct nand_chip *chip,
					     int page)
{
	int ret, status;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0, page);

	chip->pagebuf = -1;

	memset(chip->buffers->databuf, 0xff, mtd->writesize);
	ret = chip->ecc.write_page(mtd, chip, chip->buffers->databuf, 1, page);
	if (ret)
		return ret;

	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static const s32 tWB_lut[] = {6, 12, 16, 20};
static const s32 tRHW_lut[] = {4, 8, 12, 20};

static int _sunxi_nand_lookup_timing(const s32 *lut, int lut_size, u32 duration,
		u32 clk_period)
{
	u32 clk_cycles = DIV_ROUND_UP(duration, clk_period);
	int i;

	for (i = 0; i < lut_size; i++) {
		if (clk_cycles <= lut[i])
			return i;
	}

	/* Doesn't fit */
	return -EINVAL;
}

#define sunxi_nand_lookup_timing(l, p, c) \
			_sunxi_nand_lookup_timing(l, ARRAY_SIZE(l), p, c)

static int sunxi_nand_chip_set_timings(struct sunxi_nand_chip *chip,
				       const struct nand_sdr_timings *timings)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(chip->nand.controller);
	u32 min_clk_period = 0;
	s32 tWB, tADL, tWHR, tRHW, tCAD;

	/* T1 <=> tCLS */
	if (timings->tCLS_min > min_clk_period)
		min_clk_period = timings->tCLS_min;

	/* T2 <=> tCLH */
	if (timings->tCLH_min > min_clk_period)
		min_clk_period = timings->tCLH_min;

	/* T3 <=> tCS */
	if (timings->tCS_min > min_clk_period)
		min_clk_period = timings->tCS_min;

	/* T4 <=> tCH */
	if (timings->tCH_min > min_clk_period)
		min_clk_period = timings->tCH_min;

	/* T5 <=> tWP */
	if (timings->tWP_min > min_clk_period)
		min_clk_period = timings->tWP_min;

	/* T6 <=> tWH */
	if (timings->tWH_min > min_clk_period)
		min_clk_period = timings->tWH_min;

	/* T7 <=> tALS */
	if (timings->tALS_min > min_clk_period)
		min_clk_period = timings->tALS_min;

	/* T8 <=> tDS */
	if (timings->tDS_min > min_clk_period)
		min_clk_period = timings->tDS_min;

	/* T9 <=> tDH */
	if (timings->tDH_min > min_clk_period)
		min_clk_period = timings->tDH_min;

	/* T10 <=> tRR */
	if (timings->tRR_min > (min_clk_period * 3))
		min_clk_period = DIV_ROUND_UP(timings->tRR_min, 3);

	/* T11 <=> tALH */
	if (timings->tALH_min > min_clk_period)
		min_clk_period = timings->tALH_min;

	/* T12 <=> tRP */
	if (timings->tRP_min > min_clk_period)
		min_clk_period = timings->tRP_min;

	/* T13 <=> tREH */
	if (timings->tREH_min > min_clk_period)
		min_clk_period = timings->tREH_min;

	/* T14 <=> tRC */
	if (timings->tRC_min > (min_clk_period * 2))
		min_clk_period = DIV_ROUND_UP(timings->tRC_min, 2);

	/* T15 <=> tWC */
	if (timings->tWC_min > (min_clk_period * 2))
		min_clk_period = DIV_ROUND_UP(timings->tWC_min, 2);

	/* T16 - T19 + tCAD */
	tWB  = sunxi_nand_lookup_timing(tWB_lut, timings->tWB_max,
					min_clk_period);
	if (tWB < 0) {
		dev_err(nfc->dev, "unsupported tWB\n");
		return tWB;
	}

	tADL = DIV_ROUND_UP(timings->tADL_min, min_clk_period) >> 3;
	if (tADL > 3) {
		dev_err(nfc->dev, "unsupported tADL\n");
		return -EINVAL;
	}

	tWHR = DIV_ROUND_UP(timings->tWHR_min, min_clk_period) >> 3;
	if (tWHR > 3) {
		dev_err(nfc->dev, "unsupported tWHR\n");
		return -EINVAL;
	}

	tRHW = sunxi_nand_lookup_timing(tRHW_lut, timings->tRHW_min,
					min_clk_period);
	if (tRHW < 0) {
		dev_err(nfc->dev, "unsupported tRHW\n");
		return tRHW;
	}

	/*
	 * TODO: according to ONFI specs this value only applies for DDR NAND,
	 * but Allwinner seems to set this to 0x7. Mimic them for now.
	 */
	tCAD = 0x7;

	/* TODO: A83 has some more bits for CDQSS, CS, CLHZ, CCS, WC */
	chip->timing_cfg = NFC_TIMING_CFG(tWB, tADL, tWHR, tRHW, tCAD);

	/*
	 * ONFI specification 3.1, paragraph 4.15.2 dictates that EDO data
	 * output cycle timings shall be used if the host drives tRC less than
	 * 30 ns.
	 */
	chip->timing_ctl = (timings->tRC_min < 30000) ? NFC_TIMING_CTL_EDO : 0;

	/* Convert min_clk_period from picoseconds to nanoseconds */
	min_clk_period = DIV_ROUND_UP(min_clk_period, 1000);

	/*
	 * Convert min_clk_period into a clk frequency, then get the
	 * appropriate rate for the NAND controller IP given this formula
	 * (specified in the datasheet):
	 * nand clk_rate = 2 * min_clk_rate
	 */
	chip->clk_rate = (2 * NSEC_PER_SEC) / min_clk_period;

	return 0;
}

static int sunxi_nand_chip_init_timings(struct sunxi_nand_chip *chip,
					struct device_node *np)
{
	const struct nand_sdr_timings *timings;
	int ret;
	int mode;

	mode = onfi_get_async_timing_mode(&chip->nand);
	if (mode == ONFI_TIMING_MODE_UNKNOWN) {
		mode = chip->nand.onfi_timing_mode_default;
	} else {
		uint8_t feature[ONFI_SUBFEATURE_PARAM_LEN] = {};

		mode = fls(mode) - 1;
		if (mode < 0)
			mode = 0;

		feature[0] = mode;
		ret = chip->nand.onfi_set_features(&chip->mtd, &chip->nand,
						ONFI_FEATURE_ADDR_TIMING_MODE,
						feature);
		if (ret)
			return ret;
	}

	timings = onfi_async_timing_mode_to_sdr_timings(mode);
	if (IS_ERR(timings))
		return PTR_ERR(timings);

	return sunxi_nand_chip_set_timings(chip, timings);
}

static int sunxi_nand_hw_common_ecc_ctrl_init(struct mtd_info *mtd,
					      struct nand_ecc_ctrl *ecc,
					      struct device_node *np)
{
	static const u8 strengths[] = { 16, 24, 28, 32, 40, 48, 56, 60, 64 };
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	struct sunxi_nand_hw_ecc *data;
	struct nand_ecclayout *layout;
	int nsectors;
	int ret;
	int i;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* Add ECC info retrieval from DT */
	for (i = 0; i < ARRAY_SIZE(strengths); i++) {
		if (ecc->strength <= strengths[i])
			break;
	}

	if (i >= ARRAY_SIZE(strengths)) {
		dev_err(nfc->dev, "unsupported strength\n");
		ret = -ENOTSUPP;
		goto err;
	}

	data->mode = i;

	/* HW ECC always request ECC bytes for 1024 bytes blocks */
	ecc->bytes = DIV_ROUND_UP(ecc->strength * fls(8 * 1024), 8);

	/* HW ECC always work with even numbers of ECC bytes */
	ecc->bytes = ALIGN(ecc->bytes, 2);

	layout = &data->layout;
	nsectors = mtd->writesize / ecc->size;

	if (mtd->oobsize < ((ecc->bytes + 4) * nsectors)) {
		ret = -EINVAL;
		goto err;
	}

	layout->eccbytes = (ecc->bytes * nsectors);

	ecc->read_oob = sunxi_nfc_hw_common_ecc_read_oob;
	ecc->write_oob = sunxi_nfc_hw_common_ecc_write_oob;
	ecc->layout = layout;
	ecc->priv = data;

	return 0;

err:
	kfree(data);

	return ret;
}

static void sunxi_nand_hw_common_ecc_ctrl_cleanup(struct nand_ecc_ctrl *ecc)
{
	kfree(ecc->priv);
}

static int sunxi_nand_hw_ecc_ctrl_init(struct mtd_info *mtd,
				       struct nand_ecc_ctrl *ecc,
				       struct device_node *np)
{
	struct nand_ecclayout *layout;
	int nsectors;
	int i, j;
	int ret;

	ret = sunxi_nand_hw_common_ecc_ctrl_init(mtd, ecc, np);
	if (ret)
		return ret;

	ecc->read_page = sunxi_nfc_hw_ecc_read_page;
	ecc->write_page = sunxi_nfc_hw_ecc_write_page;
	ecc->read_oob_raw = nand_read_oob_std;
	ecc->write_oob_raw = nand_write_oob_std;
	layout = ecc->layout;
	nsectors = mtd->writesize / ecc->size;

	for (i = 0; i < nsectors; i++) {
		if (i) {
			layout->oobfree[i].offset =
				layout->oobfree[i - 1].offset +
				layout->oobfree[i - 1].length +
				ecc->bytes;
			layout->oobfree[i].length = 4;
		} else {
			/*
			 * The first 2 bytes are used for BB markers, hence we
			 * only have 2 bytes available in the first user data
			 * section.
			 */
			layout->oobfree[i].length = 2;
			layout->oobfree[i].offset = 2;
		}

		for (j = 0; j < ecc->bytes; j++)
			layout->eccpos[(ecc->bytes * i) + j] =
					layout->oobfree[i].offset +
					layout->oobfree[i].length + j;
	}

	if (mtd->oobsize > (ecc->bytes + 4) * nsectors) {
		layout->oobfree[nsectors].offset =
				layout->oobfree[nsectors - 1].offset +
				layout->oobfree[nsectors - 1].length +
				ecc->bytes;
		layout->oobfree[nsectors].length = mtd->oobsize -
				((ecc->bytes + 4) * nsectors);
	}

	return 0;
}

static int sunxi_nand_hw_syndrome_ecc_ctrl_init(struct mtd_info *mtd,
						struct nand_ecc_ctrl *ecc,
						struct device_node *np)
{
	struct nand_ecclayout *layout;
	int nsectors;
	int i;
	int ret;

	ret = sunxi_nand_hw_common_ecc_ctrl_init(mtd, ecc, np);
	if (ret)
		return ret;

	ecc->prepad = 4;
	ecc->read_page = sunxi_nfc_hw_syndrome_ecc_read_page;
	ecc->write_page = sunxi_nfc_hw_syndrome_ecc_write_page;
	ecc->read_oob_raw = nand_read_oob_syndrome;
	ecc->write_oob_raw = nand_write_oob_syndrome;

	layout = ecc->layout;
	nsectors = mtd->writesize / ecc->size;

	for (i = 0; i < (ecc->bytes * nsectors); i++)
		layout->eccpos[i] = i;

	layout->oobfree[0].length = mtd->oobsize - i;
	layout->oobfree[0].offset = i;

	return 0;
}

static void sunxi_nand_ecc_cleanup(struct nand_ecc_ctrl *ecc)
{
	switch (ecc->mode) {
	case NAND_ECC_HW:
	case NAND_ECC_HW_SYNDROME:
		sunxi_nand_hw_common_ecc_ctrl_cleanup(ecc);
		break;
	case NAND_ECC_NONE:
		kfree(ecc->layout);
	default:
		break;
	}
}

static int sunxi_nand_ecc_init(struct mtd_info *mtd, struct nand_ecc_ctrl *ecc,
			       struct device_node *np)
{
	struct nand_chip *nand = mtd->priv;
	int strength;
	int blk_size;
	int ret;

	blk_size = of_get_nand_ecc_step_size(np);
	strength = of_get_nand_ecc_strength(np);
	if (blk_size > 0 && strength > 0) {
		ecc->size = blk_size;
		ecc->strength = strength;
	} else {
		ecc->size = nand->ecc_step_ds;
		ecc->strength = nand->ecc_strength_ds;
	}

	if (!ecc->size || !ecc->strength)
		return -EINVAL;

	ecc->mode = NAND_ECC_HW;

	ret = of_get_nand_ecc_mode(np);
	if (ret >= 0)
		ecc->mode = ret;

	switch (ecc->mode) {
	case NAND_ECC_SOFT_BCH:
		break;
	case NAND_ECC_HW:
		ret = sunxi_nand_hw_ecc_ctrl_init(mtd, ecc, np);
		if (ret)
			return ret;
		break;
	case NAND_ECC_HW_SYNDROME:
		ret = sunxi_nand_hw_syndrome_ecc_ctrl_init(mtd, ecc, np);
		if (ret)
			return ret;
		break;
	case NAND_ECC_NONE:
		ecc->layout = kzalloc(sizeof(*ecc->layout), GFP_KERNEL);
		if (!ecc->layout)
			return -ENOMEM;
		ecc->layout->oobfree[0].length = mtd->oobsize;
	case NAND_ECC_SOFT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sunxi_nand_chip_init(struct device *dev, struct sunxi_nfc *nfc,
				struct device_node *np)
{
	const struct nand_sdr_timings *timings;
	struct sunxi_nand_chip *chip;
	struct mtd_part_parser_data ppdata;
	struct mtd_info *mtd;
	struct nand_chip *nand;
	int nsels;
	int ret;
	int i;
	u32 tmp;

	if (!of_get_property(np, "reg", &nsels))
		return -EINVAL;

	nsels /= sizeof(u32);
	if (!nsels) {
		dev_err(dev, "invalid reg property size\n");
		return -EINVAL;
	}

	chip = devm_kzalloc(dev,
			    sizeof(*chip) +
			    (nsels * sizeof(struct sunxi_nand_chip_sel)),
			    GFP_KERNEL);
	if (!chip) {
		dev_err(dev, "could not allocate chip\n");
		return -ENOMEM;
	}

	chip->nsels = nsels;
	chip->selected = -1;

	for (i = 0; i < nsels; i++) {
		ret = of_property_read_u32_index(np, "reg", i, &tmp);
		if (ret) {
			dev_err(dev, "could not retrieve reg property: %d\n",
				ret);
			return ret;
		}

		if (tmp > NFC_MAX_CS) {
			dev_err(dev,
				"invalid reg value: %u (max CS = 7)\n",
				tmp);
			return -EINVAL;
		}

		if (test_and_set_bit(tmp, &nfc->assigned_cs)) {
			dev_err(dev, "CS %d already assigned\n", tmp);
			return -EINVAL;
		}

		chip->sels[i].cs = tmp;

		if (!of_property_read_u32_index(np, "allwinner,rb", i, &tmp) &&
		    tmp < 2) {
			chip->sels[i].rb.type = RB_NATIVE;
			chip->sels[i].rb.info.nativeid = tmp;
		} else {
			ret = of_get_named_gpio(np, "rb-gpios", i);
			if (ret >= 0) {
				tmp = ret;
				chip->sels[i].rb.type = RB_GPIO;
				chip->sels[i].rb.info.gpio = tmp;
				ret = devm_gpio_request(dev, tmp, "nand-rb");
				if (ret)
					return ret;

				ret = gpio_direction_input(tmp);
				if (ret)
					return ret;
			} else {
				chip->sels[i].rb.type = RB_NONE;
			}
		}

		chip->randomize =
			of_property_read_bool(np, "allwinner,randomize");
	}

	timings = onfi_async_timing_mode_to_sdr_timings(0);
	if (IS_ERR(timings)) {
		ret = PTR_ERR(timings);
		dev_err(dev,
			"could not retrieve timings for ONFI mode 0: %d\n",
			ret);
		return ret;
	}

	ret = sunxi_nand_chip_set_timings(chip, timings);
	if (ret) {
		dev_err(dev, "could not configure chip timings: %d\n", ret);
		return ret;
	}

	nand = &chip->nand;
	/* Default tR value specified in the ONFI spec (chapter 4.15.1) */
	nand->chip_delay = 200;
	nand->controller = &nfc->controller;
	nand->select_chip = sunxi_nfc_select_chip;
	nand->cmd_ctrl = sunxi_nfc_cmd_ctrl;
	nand->read_buf = sunxi_nfc_read_buf;
	nand->write_buf = sunxi_nfc_write_buf;
	nand->read_byte = sunxi_nfc_read_byte;

	if (of_get_nand_on_flash_bbt(np))
		nand->bbt_options |= NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;

	mtd = &chip->mtd;
	mtd->dev.parent = dev;
	mtd->priv = nand;
	mtd->owner = THIS_MODULE;

	ret = nand_scan_ident(mtd, nsels, NULL);
	if (ret)
		return ret;

	if (chip->randomize) {
		nand->options |= NAND_NO_SUBPAGE_WRITE;
		nand->bbt_options |= NAND_BBT_SCANRAWMODE;
	}

	ret = sunxi_nand_chip_init_timings(chip, np);
	if (ret) {
		dev_err(dev, "could not configure chip timings: %d\n", ret);
		return ret;
	}

	ret = sunxi_nand_ecc_init(mtd, &nand->ecc, np);
	if (ret) {
		dev_err(dev, "ECC init failed: %d\n", ret);
		return ret;
	}

	ret = nand_scan_tail(mtd);
	if (ret) {
		dev_err(dev, "nand_scan_tail failed: %d\n", ret);
		return ret;
	}

	ppdata.of_node = np;
	ret = mtd_device_parse_register(mtd, NULL, &ppdata, NULL, 0);
	if (ret) {
		dev_err(dev, "failed to register mtd device: %d\n", ret);
		nand_release(mtd);
		return ret;
	}

	list_add_tail(&chip->node, &nfc->chips);

	return 0;
}

static int sunxi_nand_chips_init(struct device *dev, struct sunxi_nfc *nfc)
{
	struct device_node *np = dev->of_node;
	struct device_node *nand_np;
	int nchips = of_get_child_count(np);
	int ret;

	if (nchips > 8) {
		dev_err(dev, "too many NAND chips: %d (max = 8)\n", nchips);
		return -EINVAL;
	}

	for_each_child_of_node(np, nand_np) {
		ret = sunxi_nand_chip_init(dev, nfc, nand_np);
		if (ret)
			return ret;
	}

	return 0;
}

static void sunxi_nand_chips_cleanup(struct sunxi_nfc *nfc)
{
	struct sunxi_nand_chip *chip;

	while (!list_empty(&nfc->chips)) {
		chip = list_first_entry(&nfc->chips, struct sunxi_nand_chip,
					node);
		nand_release(&chip->mtd);
		sunxi_nand_ecc_cleanup(&chip->nand.ecc);
		list_del(&chip->node);
	}
}

static int sunxi_nfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *r;
	struct sunxi_nfc *nfc;
	int irq;
	int ret;

	nfc = devm_kzalloc(dev, sizeof(*nfc), GFP_KERNEL);
	if (!nfc)
		return -ENOMEM;

	nfc->dev = dev;
	spin_lock_init(&nfc->controller.lock);
	init_waitqueue_head(&nfc->controller.wq);
	INIT_LIST_HEAD(&nfc->chips);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nfc->regs = devm_ioremap_resource(dev, r);
	if (IS_ERR(nfc->regs))
		return PTR_ERR(nfc->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to retrieve irq\n");
		return irq;
	}

	nfc->ahb_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(nfc->ahb_clk)) {
		dev_err(dev, "failed to retrieve ahb clk\n");
		return PTR_ERR(nfc->ahb_clk);
	}

	ret = clk_prepare_enable(nfc->ahb_clk);
	if (ret)
		return ret;

	nfc->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR(nfc->mod_clk)) {
		dev_err(dev, "failed to retrieve mod clk\n");
		ret = PTR_ERR(nfc->mod_clk);
		goto out_ahb_clk_unprepare;
	}

	ret = clk_prepare_enable(nfc->mod_clk);
	if (ret)
		goto out_ahb_clk_unprepare;

	ret = sunxi_nfc_rst(nfc);
	if (ret)
		goto out_mod_clk_unprepare;

	writel(0, nfc->regs + NFC_REG_INT);
	ret = devm_request_irq(dev, irq, sunxi_nfc_interrupt,
			       0, "sunxi-nand", nfc);
	if (ret)
		goto out_mod_clk_unprepare;

	platform_set_drvdata(pdev, nfc);

	ret = sunxi_nand_chips_init(dev, nfc);
	if (ret) {
		dev_err(dev, "failed to init nand chips\n");
		goto out_mod_clk_unprepare;
	}

	return 0;

out_mod_clk_unprepare:
	clk_disable_unprepare(nfc->mod_clk);
out_ahb_clk_unprepare:
	clk_disable_unprepare(nfc->ahb_clk);

	return ret;
}

static int sunxi_nfc_remove(struct platform_device *pdev)
{
	struct sunxi_nfc *nfc = platform_get_drvdata(pdev);

	sunxi_nand_chips_cleanup(nfc);

	return 0;
}

static const struct of_device_id sunxi_nfc_ids[] = {
	{ .compatible = "allwinner,sun4i-a10-nand" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_nfc_ids);

static struct platform_driver sunxi_nfc_driver = {
	.driver = {
		.name = "sunxi_nand",
		.of_match_table = sunxi_nfc_ids,
	},
	.probe = sunxi_nfc_probe,
	.remove = sunxi_nfc_remove,
};
module_platform_driver(sunxi_nfc_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Boris BREZILLON");
MODULE_DESCRIPTION("Allwinner NAND Flash Controller driver");
MODULE_ALIAS("platform:sunxi_nand");
