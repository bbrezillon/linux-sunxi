/*
 *  Copyright © 2016 - Boris Brezillon <boris.brezillon@free-electrons.com>
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MTD_NAND_H
#define __LINUX_MTD_NAND_H

#include <linux/mtd/mtd.h>

struct nand_memory_organization {
	int pagesize;
	int eraseblocksize;
	size_t planesize;
	int nplanes;
	size_t diesize;
	int ndies;
	size_t chipsize;
};

struct nand_device {
	struct mtd_info mtd;
	struct nand_memory_organization memorg;
};

static inline struct nand_device *mtd_to_nand(struct mtd_info *mtd)
{
	return container_of(mtd, struct nand_device, mtd);
}

static inline struct mtd_info *nand_to_mtd(struct nand_device *nand)
{
	return &nand->mtd;
}

static inline const struct nand_memory_organization*
nand_get_mem_organization(const struct nand_device *nand)
{
	return &nand->memorg;
}

static inline int nand_register(struct nand_device *nand)
{
	return mtd_device_register(&nand->mtd);
}

static inline void nand_unregister(struct nand_device *nand)
{
	mtd_device_unregister(&nand->mtd);
}

#endif /* __LINUX_MTD_NAND_H */
