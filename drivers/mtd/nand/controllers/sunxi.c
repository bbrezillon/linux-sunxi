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
#include <linux/mtd/nand2.h>
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
#define NFC_REG_ECC_CNT0	0x0040
#define NFC_REG_ECC_CNT1	0x0044
#define NFC_REG_ECC_CNT2	0x0048
#define NFC_REG_ECC_CNT3	0x004c
#define NFC_REG_USER_DATA_BASE	0x0050
#define NFC_REG_SPARE_AREA	0x00A0
#define NFC_RAM0_BASE		0x0400
#define NFC_RAM1_BASE		0x0800

/* define bit use in NFC_CTL */
#define NFC_EN			BIT(0)
#define NFC_RESET		BIT(1)
#define NFC_BUS_WIDYH		BIT(2)
#define NFC_RB_SEL		BIT(3)
#define NFC_CE_SEL		GENMASK(26, 24)
#define NFC_CE_CTL		BIT(6)
#define NFC_CE_CTL1		BIT(7)
#define NFC_PAGE_SIZE		GENMASK(11, 8)
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
#define NFC_RB_STATE0		BIT(8)
#define NFC_RB_STATE1		BIT(9)
#define NFC_RB_STATE2		BIT(10)
#define NFC_RB_STATE3		BIT(11)

/* define bit use in NFC_INT */
#define NFC_B2R_INT_ENABLE	BIT(0)
#define NFC_CMD_INT_ENABLE	BIT(1)
#define NFC_DMA_INT_ENABLE	BIT(2)
#define NFC_INT_MASK		(NFC_B2R_INT_ENABLE | \
				 NFC_CMD_INT_ENABLE | \
				 NFC_DMA_INT_ENABLE)

/* define bit use in NFC_CMD */
#define NFC_CMD1_MSK		GENMASK(7, 0)
#define NFC_CMD2_MSK		GENMASK(15, 8)
#define NFC_NADDR_MSK		GENMASK(18, 16)
#define NFC_NADDR(x)		((x) << 16)
#define NFC_SEND_ADR		BIT(19)
#define NFC_ACCESS_DIR		BIT(20)
#define NFC_DATA_TRANS		BIT(21)
#define NFC_SEND_CMD1		BIT(22)
#define NFC_CMD1(x)		(NFC_SEND_CMD1 | (x))
#define NFC_WAIT_FLAG		BIT(23)
#define NFC_SEND_CMD2		BIT(24)
#define NFC_CMD2(x)		(NFC_SEND_CMD2 | ((x) << 8))
#define NFC_SEQ			BIT(25)
#define NFC_DATA_SWAP_METHOD	BIT(26)
#define NFC_ROW_AUTO_INC	BIT(27)
#define NFC_SEND_CMD3		BIT(28)
#define NFC_SEND_CMD4		BIT(29)
#define NFC_CMD_TYPE		GENMASK(31, 30)

/* define bit use in NFC_RCMD_SET */
#define NFC_READ_CMD		GENMASK(7, 0)
#define NFC_RANDOM_READ_CMD0	GENMASK(15, 8)
#define NFC_RANDOM_READ_CMD1	GENMASK(23, 16)

/* define bit use in NFC_WCMD_SET */
#define NFC_PROGRAM_CMD		GENMASK(7, 0)
#define NFC_RANDOM_WRITE_CMD	GENMASK(15, 8)
#define NFC_READ_CMD0		GENMASK(23, 16)
#define NFC_READ_CMD1		GENMASK(31, 24)

/* define bit use in NFC_ECC_CTL */
#define NFC_ECC_EN		BIT(0)
#define NFC_ECC_PIPELINE	BIT(3)
#define NFC_ECC_EXCEPTION	BIT(4)
#define NFC_ECC_BLOCK_SIZE	BIT(5)
#define NFC_RANDOM_EN		BIT(9)
#define NFC_RANDOM_DIRECTION	BIT(10)
#define NFC_ECC_MODE_SHIFT	12
#define NFC_ECC_MODE		GENMASK(15, 12)
#define NFC_RANDOM_SEED		GENMASK(30, 16)

#define NFC_DEFAULT_TIMEOUT_US	1000 * 1000

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
 * @clk_rate:		clk_rate required for this NAND chip
 * @selected:		current active CS
 * @nsels:		number of CS lines required by the NAND chip
 * @sels:		array of CS lines descriptions
 */
struct sunxi_nand_device {
	unsigned long clk_rate;
	int selected;
	int nsels;
	struct sunxi_nand_chip_sel sels[0];
};

/*
 * NAND Controller structure: stores sunxi NAND controller information
 *
 * @regs:		NAND controller registers
 * @ahb_clk:		NAND Controller AHB clock
 * @mod_clk:		NAND Controller mod clock
 * @assigned_cs:	bitmask describing already assigned CS lines
 * @clk_rate:		NAND controller current clock rate
 * @complete:		a completion object used to wait for NAND
 *			controller events
 */
struct sunxi_nfc {
	void __iomem *regs;
	struct clk *ahb_clk;
	struct clk *mod_clk;
	unsigned long assigned_cs;
	unsigned long clk_rate;
	struct completion complete;
};

static irqreturn_t sunxi_nfc_interrupt(int irq, void *dev_id)
{
	struct nand_controller *controller = dev_id;
	struct sunxi_nfc *nfc = nand_controller_get_devdata(controller);
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

static int sunxi_nfc_wait_int(struct nand_controller *controller, u32 flags,
			      unsigned int timeout_us)
{
	struct sunxi_nfc *nfc = nand_controller_get_devdata(controller);

	init_completion(&nfc->complete);

	writel(flags, nfc->regs + NFC_REG_INT);

	if (!timeout_us)
		timeout_us = NFC_DEFAULT_TIMEOUT_US;

	if (!wait_for_completion_timeout(&nfc->complete,
					 msecs_to_jiffies(timeout_us))) {
		dev_err(&controller->dev, "wait interrupt timedout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int sunxi_nfc_wait_cmd_fifo_empty(struct nand_controller *controller)
{
	struct sunxi_nfc *nfc = nand_controller_get_devdata(controller);
	unsigned long timeout = jiffies +
				msecs_to_jiffies(NFC_DEFAULT_TIMEOUT_US);

	do {
		if (!(readl(nfc->regs + NFC_REG_ST) & NFC_CMD_FIFO_STATUS))
			return 0;
	} while (time_before(jiffies, timeout));

	dev_err(&controller->dev, "wait for empty cmd FIFO timedout\n");
	return -ETIMEDOUT;
}

static int sunxi_nfc_rst(struct nand_controller *controller)
{
	struct sunxi_nfc *nfc = nand_controller_get_devdata(controller);
	unsigned long timeout = jiffies +
				msecs_to_jiffies(NFC_DEFAULT_TIMEOUT_US);

	writel(0, nfc->regs + NFC_REG_ECC_CTL);
	writel(NFC_RESET, nfc->regs + NFC_REG_CTL);

	do {
		if (!(readl(nfc->regs + NFC_REG_CTL) & NFC_RESET))
			return 0;
	} while (time_before(jiffies, timeout));

	dev_err(&controller->dev, "wait for NAND controller reset timedout\n");
	return -ETIMEDOUT;
}

static int sunxi_nfc_wait_ready(struct nand_operation *op)
{
	struct sunxi_nand_device *sunxi_nand = nand_get_ctrldata(op->dev);
	struct sunxi_nfc *nfc = nand_controller_get_devdata(op->dev->controller);
	struct sunxi_nand_rb *rb;

	if (sunxi_nand->selected < 0)
		return -EINVAL;

	rb = &sunxi_nand->sels[sunxi_nand->selected].rb;

	switch (rb->type) {
	case RB_NATIVE:
		if (readl(nfc->regs + NFC_REG_ST) &
		    (NFC_RB_STATE0 << rb->info.nativeid))
			return 0;

		sunxi_nfc_wait_int(op->dev->controller, NFC_RB_B2R, op->timeout);
		if (readl(nfc->regs + NFC_REG_ST) &
		    (NFC_RB_STATE0 << rb->info.nativeid))
			return 0;
		break;
	case RB_GPIO:
		if (gpio_get_value(rb->info.gpio))
			return 0;

		break;
	case RB_NONE:
		/* TODO: fallback to software implementation */
	default:
		dev_err(&op->dev->controller->dev,
			"cannot check R/B NAND status!\n");
		return -ENOTSUPP;
	}

	return -ETIMEDOUT;
}

static void sunxi_nfc_select(struct nand_device *nand, int chip)
{
	struct sunxi_nand_device *sunxi_nand = nand_get_ctrldata(nand);
	struct sunxi_nfc *nfc = nand_controller_get_devdata(nand->controller);
	struct sunxi_nand_chip_sel *sel;
	u32 ctl;

	if (chip > 0 && chip >= sunxi_nand->nsels)
		return;

	if (chip == sunxi_nand->selected)
		return;

	ctl = readl(nfc->regs + NFC_REG_CTL) &
	      ~(NFC_CE_SEL | NFC_RB_SEL | NFC_EN);

	if (chip >= 0) {
		sel = &sunxi_nand->sels[chip];

		ctl |= (sel->cs << 24) | NFC_EN |
		       (((nand->mtd.writesize / 1024) & 0xf) << 8);
		if (sel->rb.type == RB_NATIVE)
			ctl |= (sel->rb.info.nativeid << 3);

		writel(nand->mtd.writesize, nfc->regs + NFC_REG_SPARE_AREA);

		if (nfc->clk_rate != sunxi_nand->clk_rate) {
			clk_set_rate(nfc->mod_clk, sunxi_nand->clk_rate);
			nfc->clk_rate = sunxi_nand->clk_rate;
		}
	}

	writel(ctl, nfc->regs + NFC_REG_CTL);

	sunxi_nand->selected = chip;
}

static int sunxi_nfc_launch_op(struct nand_operation *op)
{
	struct sunxi_nfc *nfc = nand_controller_get_devdata(op->dev->controller);
	u32 cmd = 0;
	int ret, i;

	sunxi_nfc_select(op->dev, op->chip);
	if (op->ncmds)
		cmd |= NFC_CMD1(op->cmds[0]);

	if (op->naddrs) {
		u32 addr[2] = { 0, 0 };

		cmd |= NFC_NADDR(op->naddrs - 1);
		for (i = 0; i < op->naddrs; i++)
			addr[i / 4] |= op->addrs[i] << (8 * (i % 4));
		writel(addr[0], nfc->regs + NFC_REG_ADDR_LOW);
		writel(addr[1], nfc->regs + NFC_REG_ADDR_HIGH);
	}

	if (op->ncmds > 1 && op->dir == NAND_OP_OUTPUT) {
		cmd |= NFC_SEND_CMD2;
		if (cmd & NFC_ACCESS_DIR)
			writel(op->cmds[1], nfc->regs + NFC_REG_WCMD_SET);
		else
			writel(op->cmds[1], nfc->regs + NFC_REG_RCMD_SET);
	}

	ret = sunxi_nfc_wait_cmd_fifo_empty(op->dev->controller);
	if (ret)
		return ret;

	if (op->dir == NAND_OP_INPUT && op->waitready)
		sunxi_nfc_wait_ready(op);

	/*
	 * TODO: maybe we should optimize this for read commands by passing
	 * to the previous cmd request.
	 */
	if (op->dir == NAND_OP_OUTPUT)
		cmd = NFC_DATA_TRANS | NFC_ACCESS_DIR;
	else if (op->dir == NAND_OP_INPUT)
		cmd = NFC_DATA_TRANS;

	/* TODO: write into SRAM buffer */

	ret = sunxi_nfc_wait_cmd_fifo_empty(op->dev->controller);
	if (ret)
		return ret;

	sunxi_nfc_select(op->dev, -1);

	return 0;
}

static int sunxi_nfc_apply_timings(struct nand_device *nand)
{
	struct nand_sdr_timings *timings = &nand->hw_interface.timings.sdr;
	struct sunxi_nand_device *sunxi_nand = nand_get_ctrldata(nand);
	u32 min_clk_period = 0;

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


	/* Convert min_clk_period from picoseconds to nanoseconds */
	min_clk_period = DIV_ROUND_UP(min_clk_period, 1000);

	/*
	 * Convert min_clk_period into a clk frequency, then get the
	 * appropriate rate for the NAND controller IP given this formula
	 * (specified in the datasheet):
	 * nand clk_rate = 2 * min_clk_rate
	 */
	sunxi_nand->clk_rate = (2 * NSEC_PER_SEC) / min_clk_period;

	/* TODO: configure T16-T19 */

	return 0;
}

static int sunxi_nfc_init(struct nand_device *nand)
{
	struct sunxi_nfc *nfc = nand_controller_get_devdata(nand->controller);
	struct device *dev = &nand->controller->dev;
	struct device_node *np = nand->dev.of_node;
	struct sunxi_nand_device *sunxi_nand;
	int ret, i;
	u32 tmp;

	sunxi_nand = devm_kzalloc(&nand->controller->dev,
				  sizeof(*sunxi_nand) +
				  (nand->hw_interface.cs.num *
				   sizeof(struct sunxi_nand_chip_sel)),
				  GFP_KERNEL);
	if (!sunxi_nand) {
		dev_err(dev, "could not allocate chip\n");
		return -ENOMEM;
	}

	sunxi_nand->selected = -1;

	for (i = 0; i < nand->hw_interface.cs.num; i++) {
		if (nand->hw_interface.cs.ids[i] > NFC_MAX_CS) {
			dev_err(dev,
				"invalid reg value: %u (max CS = 7)\n",
				tmp);
			return -EINVAL;
		}

		if (test_and_set_bit(nand->hw_interface.cs.ids[i],
				     &nfc->assigned_cs)) {
			dev_err(dev, "CS %d already assigned\n", tmp);
			return -EINVAL;
		}

		if (!of_property_read_u32_index(np, "allwinner,rb", i, &tmp) &&
		    tmp < 2) {
			sunxi_nand->sels[i].rb.type = RB_NATIVE;
			sunxi_nand->sels[i].rb.info.nativeid = tmp;
		} else {
			ret = of_get_named_gpio(np, "rb-gpios", i);
			if (ret >= 0) {
				tmp = ret;
				sunxi_nand->sels[i].rb.type = RB_GPIO;
				sunxi_nand->sels[i].rb.info.gpio = tmp;
				ret = devm_gpio_request(&nand->dev, tmp,
							"nand-rb");
				if (ret)
					return ret;

				ret = gpio_direction_input(tmp);
				if (ret)
					return ret;
			} else {
				sunxi_nand->sels[i].rb.type = RB_NONE;
			}
		}
	}

	return 0;
}

static const struct nand_controller_ops sunxi_nfc_ops = {
	.launch_op = sunxi_nfc_launch_op,
	.initialize = sunxi_nfc_init,
	.apply_timings = sunxi_nfc_apply_timings,
};

static int sunxi_nfc_probe(struct platform_device *pdev)
{
	struct nand_controller *controller;
	struct device *dev = &pdev->dev;
	struct resource *r;
	struct sunxi_nfc *nfc;
	int irq;
	int ret;

	controller = nand_controller_alloc(&pdev->dev, sizeof(*nfc));
	if (!controller)
		return -ENOMEM;

	nfc = nand_controller_get_devdata(controller);
	if (!nfc)
		return -ENOMEM;

	controller->ops = &sunxi_nfc_ops;

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

	ret = sunxi_nfc_rst(controller);
	if (ret)
		goto out_mod_clk_unprepare;

	writel(0, nfc->regs + NFC_REG_INT);
	ret = devm_request_irq(dev, irq, sunxi_nfc_interrupt,
			       0, "sunxi-nand", nfc);
	if (ret)
		goto out_mod_clk_unprepare;

	platform_set_drvdata(pdev, controller);

	/*
	 * TODO: replace these magic values with proper flags as soon as we
	 * know what they are encoding.
	 */
	writel(0x100, nfc->regs + NFC_REG_TIMING_CTL);
	writel(0x7ff, nfc->regs + NFC_REG_TIMING_CFG);

	ret = nand_controller_register(controller);
	if (ret) {
		dev_err(dev, "failed to register sunxi NAND controller\n");
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
	struct nand_controller *controller = platform_get_drvdata(pdev);

	nand_controller_unregister(controller);

	return 0;
}

static const struct of_device_id sunxi_nfc_ids[] = {
	{ .compatible = "allwinner,sun4i-a10-nand" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_nfc_ids);

static struct platform_driver sunxi_nfc_driver = {
	.driver = {
		.name = "sunxi-nand-controller",
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
