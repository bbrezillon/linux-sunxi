/*
 * Copyright (c) Free Electrons, 2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Author: Boris Brezillon <boris.brezillon@free-electrons.com>
 */

#include <linux/crc32.h>
#include "eba.h"

/**
 * struct ubi_eba_sleb_entry - structure encoding a single LEB -> PEB
 *			       association
 * @pnum: the physical eraseblock number attached to the LEB
 *
 * This structure is encoding a LEB -> PEB association. Note that the LEB
 * number is not stored here, because it is the index used to access the
 * entries table.
 */
struct ubi_eba_sleb_entry {
	int pnum;
};

/**
 * struct ubi_eba_sleb_table - sinle LEB to PEB association table
 * @base: the base EBA table
 * @entries: the EBA entries
 */
struct ubi_eba_sleb_table {
	struct ubi_eba_table base;
	struct ubi_eba_sleb_entry *entries;
};

static struct ubi_eba_sleb_table *to_sleb_table(struct ubi_eba_table *tbl)
{
	return container_of(tbl, struct ubi_eba_sleb_table, base);
}

static void ubi_eba_sleb_get_ldesc(struct ubi_volume *vol, int lnum,
				     struct ubi_eba_leb_desc *ldesc)
{
	struct ubi_eba_sleb_table *tbl = to_sleb_table(vol->eba_tbl);

	ldesc->lnum = lnum;
	ldesc->lpos = UBI_EBA_NA;
	ldesc->pnum = tbl->entries[lnum].pnum;
}

static struct ubi_peb_desc *
ubi_eba_sleb_invalidate_entry(struct ubi_volume *vol, int lnum)
{
	struct ubi_eba_sleb_table *tbl = to_sleb_table(vol->eba_tbl);
	struct ubi_device *ubi = vol->ubi;
	struct ubi_peb_desc *pdesc;
	int pnum = tbl->entries[lnum].pnum;

	/* This logical eraseblock is already unmapped */
	if (pnum < 0)
		return NULL;

	pdesc = ubi_alloc_pdesc(ubi, GFP_NOFS);
	if (!pdesc)
		return ERR_PTR(-ENOMEM);

	pdesc->pnum = pnum;
	pdesc->vol_id = vol->vol_id;
	pdesc->lnums[0] = lnum;

	tbl->entries[lnum].pnum = UBI_LEB_UNMAPPED;

	return pdesc;
}

static struct ubi_peb_desc *
ubi_eba_sleb_update_entry(struct ubi_volume *vol,
			    const struct ubi_eba_leb_desc *ldesc)
{
	struct ubi_eba_sleb_table *tbl = to_sleb_table(vol->eba_tbl);
	struct ubi_device *ubi = vol->ubi;
	struct ubi_peb_desc *pdesc = NULL;
	int pnum = tbl->entries[ldesc->lnum].pnum;

	if (pnum >= 0) {
		/* Retrieve the existing mapping. */
		pdesc = ubi_alloc_pdesc(ubi, GFP_NOFS);
		if (!pdesc)
			return ERR_PTR(-ENOMEM);

		pdesc->pnum = pnum;
		pdesc->vol_id = vol->vol_id;
		pdesc->lnums[0] = ldesc->lnum;
	}

	tbl->entries[ldesc->lnum].pnum = ldesc->pnum;

	return pdesc;
}

static void ubi_eba_sleb_destroy_table(struct ubi_eba_table *base)
{
	struct ubi_eba_sleb_table *tbl = to_sleb_table(base);

	kfree(tbl->entries);
	kfree(tbl);
}

static void ubi_eba_sleb_copy_table(struct ubi_volume *vol,
				      struct ubi_eba_table *dst_base,
				      int nentries)
{
	struct ubi_eba_sleb_table *src, *dst;
	int i;

	src = to_sleb_table(vol->eba_tbl);
	dst = to_sleb_table(dst_base);

	for (i = 0; i < nentries; i++)
		dst->entries[i].pnum = src->entries[i].pnum;
}

static int ubi_eba_sleb_init_entry(struct ubi_volume *vol,
				     struct ubi_ainf_leb *aleb)
{
	struct ubi_eba_sleb_table *tbl = to_sleb_table(vol->eba_tbl);

	ubi_assert(!aleb->peb->consolidated);

	tbl->entries[aleb->lnum].pnum = aleb->peb->sleb.pnum;

	return 0;
}

static struct ubi_eba_table *
ubi_eba_sleb_create_table(struct ubi_volume *vol, int nentries)
{
	struct ubi_eba_sleb_table *tbl;
	int err = -ENOMEM;
	int i;

	tbl = kzalloc(sizeof(*tbl), GFP_KERNEL);
	if (!tbl)
		return ERR_PTR(-ENOMEM);

	tbl->entries = kmalloc_array(nentries, sizeof(*tbl->entries),
				     GFP_KERNEL);
	if (!tbl->entries)
		goto err;

	for (i = 0; i < nentries; i++)
		tbl->entries[i].pnum = UBI_LEB_UNMAPPED;


	return &tbl->base;

err:
	kfree(tbl->entries);
	kfree(tbl);

	return ERR_PTR(err);
}

static int ubi_eba_sleb_copy_peb(struct ubi_volume *vol, int from, int to,
				 struct ubi_vid_io_buf *vidb)
{
	int err, vol_id, lnum, data_size, aldata_size;
	struct ubi_vid_hdr *vid_hdr = ubi_get_vid_hdr(vidb);
	struct ubi_device *ubi = vol->ubi;
	struct ubi_eba_leb_desc ldesc;
	enum ubi_io_mode io_mode;
	uint32_t crc;

	vol_id = be32_to_cpu(vid_hdr->vol_id);
	lnum = be32_to_cpu(vid_hdr->lnum);

	/*
	 * We do not want anybody to write to this logical eraseblock while we
	 * are moving it, so lock it.
	 *
	 * Note, we are using non-waiting locking here, because we cannot sleep
	 * on the LEB, since it may cause deadlocks. Indeed, imagine a task is
	 * unmapping the LEB which is mapped to the PEB we are going to move
	 * (@from). This task locks the LEB and goes sleep in the
	 * 'ubi_wl_put_peb()' function on the @ubi->move_mutex. In turn, we are
	 * holding @ubi->move_mutex and go sleep on the LEB lock. So, if the
	 * LEB is already locked, we just do not move it and return
	 * %MOVE_RETRY. Note, we do not return %MOVE_CANCEL_RACE here because
	 * we do not know the reasons of the contention - it may be just a
	 * normal I/O on this LEB, so we want to re-try.
	 */
	err = ubi_eba_leb_write_trylock(vol, lnum);
	if (err) {
		dbg_wl("contention on LEB %d:%d, cancel", vol_id, lnum);
		return MOVE_RETRY;
	}

	/*
	 * The LEB might have been put meanwhile, and the task which put it is
	 * probably waiting on @ubi->move_mutex. No need to continue the work,
	 * cancel it.
	 */
	ubi_eba_get_ldesc(vol, lnum, &ldesc);
	if (ldesc.pnum != from) {
		dbg_wl("LEB %d:%d is no longer mapped to PEB %d, mapped to PEB %d, cancel",
		       vol_id, lnum, from, ldesc.pnum);
		err = MOVE_CANCEL_RACE;
		goto out_unlock_leb;
	}

	if (vid_hdr->vol_type == UBI_VID_STATIC) {
		data_size = be32_to_cpu(vid_hdr->data_size);
		aldata_size = ALIGN(data_size, ubi->min_io_size);
	} else {
		data_size = aldata_size =
			    vol->leb_size - be32_to_cpu(vid_hdr->data_pad);
	}

	io_mode = ubi_vol_mode_to_io_mode(vol->vol_mode);

	/*
	 * OK, now the LEB is locked and we can safely start moving it. Since
	 * this function utilizes the @ubi->peb_buf buffer which is shared
	 * with some other functions - we lock the buffer by taking the
	 * @ubi->buf_mutex.
	 */
	mutex_lock(&ubi->buf_mutex);
	dbg_wl("read %d bytes of data", aldata_size);
	err = ubi_io_read_data(ubi, ubi->peb_buf, from, 0, aldata_size,
			       io_mode);
	if (err && err != UBI_IO_BITFLIPS) {
		ubi_warn(ubi, "error %d while reading data from PEB %d",
			 err, from);
		err = MOVE_SOURCE_RD_ERR;
		goto out_unlock_buf;
	}

	/*
	 * Now we have got to calculate how much data we have to copy. In
	 * case of a static volume it is fairly easy - the VID header contains
	 * the data size. In case of a dynamic volume it is more difficult - we
	 * have to read the contents, cut 0xFF bytes from the end and copy only
	 * the first part. We must do this to avoid writing 0xFF bytes as it
	 * may have some side-effects. And not only this. It is important not
	 * to include those 0xFFs to CRC because later the they may be filled
	 * by data.
	 */
	if (vid_hdr->vol_type == UBI_VID_DYNAMIC)
		aldata_size = data_size =
			ubi_calc_data_len(ubi, ubi->peb_buf, data_size);

	cond_resched();
	crc = crc32(UBI_CRC32_INIT, ubi->peb_buf, data_size);
	cond_resched();

	/*
	 * It may turn out to be that the whole @from physical eraseblock
	 * contains only 0xFF bytes. Then we have to only write the VID header
	 * and do not write any data. This also means we should not set
	 * @vid_hdr->copy_flag, @vid_hdr->data_size, and @vid_hdr->data_crc.
	 */
	if (data_size > 0) {
		vid_hdr->copy_flag = 1;
		vid_hdr->data_size = cpu_to_be32(data_size);
		vid_hdr->data_crc = cpu_to_be32(crc);
	}
	vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));

	err = ubi_io_write_vid_hdr(ubi, to, vidb);
	if (err) {
		if (err == -EIO)
			err = MOVE_TARGET_WR_ERR;
		goto out_unlock_buf;
	}

	cond_resched();

	/* Read the VID header back and check if it was written correctly */
	err = ubi_io_read_vid_hdr(ubi, to, vidb, 1);
	if (err) {
		if (err != UBI_IO_BITFLIPS) {
			ubi_warn(ubi, "error %d while reading VID header back from PEB %d",
				 err, to);
			if (is_error_sane(err))
				err = MOVE_TARGET_RD_ERR;
		} else
			err = MOVE_TARGET_BITFLIPS;
		goto out_unlock_buf;
	}

	if (data_size > 0) {
		err = ubi_io_write_data(ubi, ubi->peb_buf, to, 0, aldata_size,
					io_mode);
		if (err) {
			if (err == -EIO)
				err = MOVE_TARGET_WR_ERR;
			goto out_unlock_buf;
		}

		cond_resched();
	}

	ubi_eba_get_ldesc(vol, lnum, &ldesc);
	ubi_assert(ldesc.pnum == from);
	down_read(&ubi->eba_sem);
	ldesc.pnum = to;
	ubi_eba_sleb_update_entry(vol, &ldesc);
	up_read(&ubi->eba_sem);

out_unlock_buf:
	mutex_unlock(&ubi->buf_mutex);
out_unlock_leb:
	ubi_eba_leb_write_unlock(vol, lnum);
	return err;
}

const struct ubi_eba_table_ops ubi_eba_sleb_ops = {
	.create = ubi_eba_sleb_create_table,
	.destroy = ubi_eba_sleb_destroy_table,
	.copy = ubi_eba_sleb_copy_table,
	.get_ldesc = ubi_eba_sleb_get_ldesc,
	.invalidate_entry = ubi_eba_sleb_invalidate_entry,
	.update_entry = ubi_eba_sleb_update_entry,
	.init_entry = ubi_eba_sleb_init_entry,
	.copy_peb = ubi_eba_sleb_copy_peb,
};
