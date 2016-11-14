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


void ubi_conso_cond_cancel(struct ubi_volume *vol, int lnum)
{
	struct ubi_consolidation *conso = vol->conso;
	int i;

	ubi_assert(conso);

	mutex_lock(&conso->lock);
	if (conso->dst.cpeb) {
		for (i = 0; i <= conso->dst.ldesc.lpos; i++) {
			if (conso->dst.cpeb->lnums[i] == lnum) {
				conso->cancel = true;
				break;
			}
		}
	}
	mutex_unlock(&conso->lock);
}


static int select_leb_for_conso(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;
	int err;

	ubi_assert(conso->dst.cpeb);
	ubi_assert(conso->dst.ldesc.lpos >= 0 &&
		   conso->dst.ldesc.lpos < vol->ubi->max_lebs_per_peb);

	err = ubi_eba_select_leb_for_conso(vol, conso->dst.cpeb, &conso->src.ldesc);
	if (err)
		return err;

	conso->src.loffset = 0;
	conso->dst.ldesc.lnum = conso->src.ldesc.lnum;
	conso->dst.cpeb->lnums[conso->dst.ldesc.lpos] = conso->src.ldesc.lnum;

	return err;
}

static void reset_conso(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;

	conso->cancel = false;
	conso->src.ldesc.lnum = UBI_UNKNOWN;
	conso->dst.cpeb = NULL;
}

static void cancel_conso(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;
	struct ubi_consolidated_peb *cpeb = conso->dst.cpeb;
	struct ubi_peb_desc *pdesc;

	reset_conso(vol);

	if (!cpeb)
		return;

	pr_info("%s:%i PEB %d\n", __func__, __LINE__, cpeb->pnum);

	pdesc = ubi_alloc_pdesc(vol->ubi, GFP_NOFS);
	if (pdesc) {
		pdesc->pnum = cpeb->pnum;
		pdesc->vol_id = UBI_UNKNOWN;
		pdesc->lnums[0] = UBI_UNKNOWN;

		ubi_wl_put_peb(vol->ubi, pdesc, 0);
	}

	kfree(cpeb);
}

static int start_conso(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;
	struct ubi_consolidated_peb *cpeb;
	struct ubi_device *ubi = vol->ubi;
	struct ubi_peb_desc *pdesc;
	struct ubi_vid_io_buf vidb;
	struct ubi_vid_hdr *vidh;
	int err, i;

	cpeb = kzalloc(sizeof(*cpeb) +
		       (ubi->max_lebs_per_peb * sizeof(*cpeb->lnums)),
		       GFP_KERNEL);
	if (!cpeb)
		return -ENOMEM;

	for (i = 0; i < ubi->max_lebs_per_peb; i++)
		cpeb->lnums[i] = UBI_LEB_UNMAPPED;

	cpeb->pnum = ubi_wl_get_peb(vol->ubi);

	/*
	 * We can release ->eba_sem directly, since we won't update the
	 * EBA table now.
	 */
	up_read(&ubi->eba_sem);

	if (cpeb->pnum < 0) {
		err = cpeb->pnum;
		goto err_free_cpeb;
	}

	/* Write the dummy VID header. */
	ubi_init_vid_buf(ubi, &vidb, conso->buf);
	vidh = ubi_get_vid_hdr(&vidb);
	vidh->lpos = UBI_VID_LPOS_CONSOLIDATED;
	vidh->vol_mode = UBI_VID_MODE_MLC_SAFE;
	vidh->vol_type = UBI_VID_DYNAMIC;
	err = ubi_io_write_vid_hdr(ubi, cpeb->pnum, &vidb);
	if (err)
		goto err_put_peb;

	mutex_lock(&conso->lock);

	conso->dst.cpeb = cpeb;
	conso->dst.ldesc.pnum = cpeb->pnum;
	conso->dst.ldesc.lpos = 0;

	err = select_leb_for_conso(vol);
	if (err)
		goto err_unlock;

	mutex_unlock(&conso->lock);

	return 0;

err_unlock:
	conso->dst.cpeb = NULL;
	mutex_unlock(&conso->lock);

err_put_peb:
	pdesc = ubi_alloc_pdesc(vol->ubi, GFP_NOFS);
	if (pdesc) {
		pdesc->pnum = cpeb->pnum;
		pdesc->vol_id = UBI_UNKNOWN;
		pdesc->lnums[0] = UBI_UNKNOWN;

		ubi_wl_put_peb(ubi, pdesc, 0);
	}

err_free_cpeb:
	kfree(cpeb);

	return err;
}

static int continue_conso(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;
	struct ubi_device *ubi = vol->ubi;
	int err = 0;

	ubi_assert(conso->dst.cpeb);

	if (conso->src.loffset == vol->leb_size) {
		conso->dst.ldesc.lpos++;
		err = select_leb_for_conso(vol);
		if (err)
			return err;
	}

	/*
	 * We only try to take the lock. If it fails this means someone
	 * is modifying the LEB, which means we should cancel the
	 * consolidation.
	 */
	err = ubi_eba_leb_read_trylock(vol, conso->src.ldesc.lnum);
	if (err > 0)
		return -EBUSY;
	else if (err < 0)
		return err;

	/*
	 * Only copy one page here.
	 * TODO: support an 'aggressive' mode where we run consolidation
	 * until we're able to free the consolidated PEBs.
	 */
	err = ubi_eba_read_leb_data(vol, &conso->src.ldesc, conso->buf,
				    conso->src.loffset, ubi->min_io_size);

	/*
	 * We can safely release the lock here: we'll check if the LEB is
	 * still valid before writing the VID headers. Which means  we can
	 * cancel the consolidation if one of the LEBs we're consolidating is
	 * invalidated.
	 */
	ubi_eba_leb_read_unlock(vol, conso->src.ldesc.lnum);

	if (err && !mtd_is_bitflip(err))
		return err;

	/* Write data to the consolidated PEB. */
	err = ubi_eba_write_leb_data(vol, &conso->dst.ldesc, conso->buf,
				     conso->src.loffset, ubi->min_io_size);
	if (err)
		return err;

	conso->src.loffset += ubi->min_io_size;

	return -EAGAIN;
}

static int finish_conso(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;
	struct ubi_device *ubi = vol->ubi;
	struct ubi_consolidated_peb *cpeb = conso->dst.cpeb;
	int err = 0, nhdrs, locked, i, offset;
	struct ubi_peb_desc *pdesc;
	struct ubi_vid_io_buf vidb;
	struct ubi_vid_hdr *vidh;
	struct ubi_peb_desc **pdescs;

	nhdrs = ubi->max_lebs_per_peb;
	pdescs = kzalloc(sizeof(*pdesc) * nhdrs, GFP_KERNEL);
	if (!pdescs)
		return -ENOMEM;

	/* Try to lock all consolidated LEBs in write mode. */
	for (locked = 0; locked < nhdrs; locked++) {
		err = ubi_eba_leb_write_trylock(vol, cpeb->lnums[locked]);
		if (err > 0)
			err = -EAGAIN;

		if (err < 0)
			goto err_unlock;
	}

	/*
	 * We locked all the LEBs in write mode, now make sure nobody tried to
	 * cancel the consolidation in the meantime.
	 */
	if (conso->cancel) {
		err = -EBUSY;
		goto err_unlock;
	}

	/* Pad with zeros. */
	memset(conso->buf, 0, ubi->min_io_size);
	for (offset = (ubi->max_lebs_per_peb * vol->leb_size) + ubi->leb_start;
	     offset < ubi->peb_size - ubi->min_io_size;
	     offset += ubi->min_io_size) {
		err = ubi_io_write(ubi, conso->buf, cpeb->pnum, offset,
				   ubi->min_io_size, UBI_IO_MODE_NORMAL);
		if (err)
			goto err_unlock;
	}

	ubi_init_vid_buf(ubi, &vidb, conso->buf);
	vidh = ubi_get_vid_hdr(&vidb);

	for (i = 0; i < nhdrs; i++) {
		vidh[i].data_pad = cpu_to_be32(vol->data_pad);
		vidh[i].sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
		vidh[i].vol_id = cpu_to_be32(vol->vol_id);
		vidh[i].lnum = cpu_to_be32(cpeb->lnums[i]);
		vidh[i].compat = ubi_get_compat(ubi, vol->vol_id);
		vidh[i].vol_type = UBI_VID_DYNAMIC;
		vidh[i].vol_mode = UBI_VID_MODE_MLC_SAFE;
		vidh[i].lpos = i;
		ubi_io_prepare_vid_hdr(ubi, &vidh[i]);
	}

	err = ubi_io_write(ubi, conso->buf, cpeb->pnum, offset,
			   ubi->min_io_size, UBI_IO_MODE_NORMAL);
	if (err)
		goto err_unlock;

	ubi_eba_update_consolidated_lebs(vol, cpeb, pdescs);

	mutex_lock(&conso->lock);
	reset_conso(vol);
	mutex_unlock(&conso->lock);

	for (i = 0; i < nhdrs; i++)
		ubi_eba_leb_write_unlock(vol, cpeb->lnums[i]);

	for (i = 0; i < nhdrs; i++) {
		if (pdescs[i])
			ubi_wl_put_peb(ubi, pdescs[i], 0);
	}

	ubi_eba_gc_pebs(vol);

	return 0;

err_unlock:
	for (locked--; locked >= 0; locked--)
		ubi_eba_leb_write_unlock(vol, cpeb->lnums[locked]);

	kfree(pdescs);

	return err;
}

static bool conso_cancelled(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;
	bool ret = false;

	mutex_lock(&conso->lock);
	if (conso->cancel)
		ret = true;
	mutex_unlock(&conso->lock);

	return ret;
}

static int conso_step(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;
	struct ubi_device *ubi = vol->ubi;
	int err = 0;

	if (!conso->dst.cpeb) {
		err = start_conso(vol);
		if (err)
			return err;
	}

	/* Check if consolidation has been cancelled. */
	if (conso_cancelled(vol)) {
		err = -EBUSY;
		goto cancel;
	}

	if (conso->dst.ldesc.lpos == ubi->max_lebs_per_peb - 1 &&
	    conso->src.loffset == vol->leb_size) {
		err = finish_conso(vol);
	} else {
		err = continue_conso(vol);
	}

	if (err && err != -EAGAIN)
		goto cancel;

	/* Check again if consolidation has been cancelled. */
	if (conso_cancelled(vol)) {
		err = -EBUSY;
		goto cancel;
	}

	return err;

cancel:
	cancel_conso(vol);

	return err;
}

static void conso_work(struct work_struct *work)
{
	struct ubi_consolidation *conso =
			container_of(work, struct ubi_consolidation,
				     work);
	struct ubi_volume *vol = conso->vol;
	int err;

	err = conso_step(vol);
	if (err && err != -EAGAIN)
		dbg_eba("consolidation failed %d", err);

	/*
	 * TODO: try to be more aggressive when UBI_CONSO_REQUIRED is returned.
	 */
	if (err == -EAGAIN ||
	    ubi_eba_get_conso_req(vol) >= UBI_CONSO_RECOMMENDED)
		schedule_work(work);
}

void ubi_conso_schedule(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;

	ubi_assert(conso);

	schedule_work(&conso->work);
}

void ubi_conso_rearm(struct ubi_volume *vol)
{
	if (ubi_eba_get_conso_req(vol) >= UBI_CONSO_RECOMMENDED)
		ubi_conso_schedule(vol);
}

int ubi_conso_init(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso;

	if (vol->vol_mode != UBI_VOL_MODE_MLC_SAFE)
		return 0;

	conso = kzalloc(sizeof(*conso), GFP_KERNEL);
	if (!conso)
		return -ENOMEM;

	conso->buf = kmalloc(vol->ubi->min_io_size, GFP_KERNEL);
	if  (!conso->buf) {
		kfree(conso);
		return -ENOMEM;
	}

	mutex_init(&conso->lock);
	INIT_WORK(&conso->work, conso_work);
	conso->vol = vol;
	vol->conso = conso;

	reset_conso(vol);


	return 0;
}

void ubi_conso_cleanup(struct ubi_volume *vol)
{
	struct ubi_consolidation *conso = vol->conso;

	if (vol->vol_mode != UBI_VOL_MODE_MLC_SAFE)
		return;

	ubi_assert(conso);

	cancel_work_sync(&conso->work);
	kfree(conso->buf);
	kfree(conso);
	vol->conso = NULL;
}
