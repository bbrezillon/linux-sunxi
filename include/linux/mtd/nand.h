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
#include <linux/mtd/bbm.h>

struct nand_memory_organization {
	int pagesize;
	int oobsize;
	int eraseblocksize;
	size_t planesize;
	int nplanes;
	u64 diesize;
	int ndies;
};

/**
 * struct nand_bbt - bad block table structure
 * @mtd: pointer to MTD device structure
 * @bbt_options: bad block specific options. All options used here must come
 * 		 from nand_bbt.h.
 * @bbt_ops: struct nand_bbt_ops pointer.
 * @info: struct nand_chip_layout_info pointer.
 * @td: bad block table descriptor for flash lookup.
 * @md: bad block table mirror descriptor
 * @bbt: in memory BBT
 */
struct nand_bbt {
	unsigned int options;
	/*
	 * Discourage new custom usages here; suggest usage of the
	 * relevant NAND_BBT_* options instead
	 */
	struct nand_bbt_descr *td;
	struct nand_bbt_descr *md;
	u8 *bbt;
};

struct nand_device;

struct nand_ops {
	int (*erase)(struct nand_device *nand, struct erase_info *einfo);
};

struct nand_device {
	struct mtd_info mtd;
	struct nand_memory_organization memorg;
	struct nand_bbt bbt;

	const struct nand_ops *ops;
};

static inline struct nand_device *mtd_to_nand(struct mtd_info *mtd)
{
	return container_of(mtd, struct nand_device, mtd);
}

static inline struct mtd_info *nand_to_mtd(struct nand_device *nand)
{
	return &nand->mtd;
}

static inline loff_t nand_page_to_offs(struct nand_device *nand, int page)
{
	return (loff_t)nand->memorg.pagesize * page;
}

static inline int nand_offs_to_page(struct nand_device *nand, loff_t offs)
{
	u64 page = offs;

	do_div(page, nand->memorg.pagesize);

	return page;
}

static inline int nand_len_to_pages(struct nand_device *nand, size_t len)
{
	return DIV_ROUND_UP(len, nand->memorg.pagesize);
}

static inline size_t nand_pages_to_len(struct nand_device *nand, int npages)
{
	return (size_t)npages * nand->memorg.pagesize;
}

static inline size_t nand_page_size(struct nand_device *nand)
{
	return nand->memorg.eraseblocksize;
}

static inline int nand_per_page_oobsize(struct nand_device *nand)
{
	return nand->memorg.oobsize;
}

static inline size_t nand_eraseblock_size(struct nand_device *nand)
{
	return nand->memorg.eraseblocksize;
}

static inline loff_t nand_eraseblock_to_offs(struct nand_device *nand,
					     int block)
{
	return (loff_t)nand->memorg.pagesize * block;
}

static inline int nand_offs_to_eraseblock(struct nand_device *nand, loff_t offs)
{
	u64 block = offs;

	do_div(block, nand->memorg.eraseblocksize);

	return block;
}

static inline int nand_len_to_eraseblocks(struct nand_device *nand, size_t len)
{
	return DIV_ROUND_UP(len, nand->memorg.eraseblocksize);
}

static inline size_t nand_eraseblocks_to_len(struct nand_device *nand, int nblocks)
{
	return (size_t)nblocks * nand->memorg.eraseblocksize;
}

static inline int nand_per_eraseblock_oobsize(struct nand_device *nand)
{
	int pagesperblock = nand->memorg.eraseblocksize /
			    nand->memorg.pagesize;

	return nand->memorg.oobsize * pagesperblock;
}

static inline int nand_eraseblock_to_page(struct nand_device *nand, int block)
{
	int pagesperblock = nand->memorg.eraseblocksize /
			    nand->memorg.pagesize;

	return block * pagesperblock;
}

static inline int nand_page_to_eraseblock(struct nand_device *nand, int page)
{
	int pagesperblock = nand->memorg.eraseblocksize /
			    nand->memorg.pagesize;

	return page / pagesperblock;
}

static inline int nand_eraseblocks_per_die(struct nand_device *nand)
{
	u64 nblocks = nand->memorg.diesize;

	do_div(nblocks, nand->memorg.eraseblocksize);

	return nblocks;
}

static inline u64 nand_diesize(struct nand_device *nand)
{
	return nand->memorg.diesize;
}

static inline int nand_dies(struct nand_device *nand)
{
	return nand->memorg.ndies;
}

static inline int nand_eraseblocks(struct nand_device *nand)
{
	u64 nblocks = nand->memorg.ndies * nand->memorg.diesize;

	do_div(nblocks, nand->memorg.eraseblocksize);

	return nblocks;
}

static inline const struct nand_memory_organization*
nand_get_mem_organization(const struct nand_device *nand)
{
	return &nand->memorg;
}

static inline int nand_register(struct nand_device *nand)
{
	return mtd_device_register(&nand->mtd, NULL, 0);
}

static inline void nand_unregister(struct nand_device *nand)
{
	mtd_device_unregister(&nand->mtd);
}

static inline int nand_read(struct nand_device *nand, loff_t offs,
			    struct mtd_oob_ops *ops)
{
	return mtd_read_oob(&nand->mtd, offs, ops);
}

static inline int nand_write(struct nand_device *nand, loff_t offs,
			     struct mtd_oob_ops *ops)
{
	return mtd_write_oob(&nand->mtd, offs, ops);
}

static inline int nand_erase(struct nand_device *nand,
			     struct erase_info *einfo,
			     bool force)
{
	if (!force)
		return mtd_erase(&nand->mtd, einfo);

	return nand->ops->erase(nand, einfo);
}

#endif /* __LINUX_MTD_NAND_H */
