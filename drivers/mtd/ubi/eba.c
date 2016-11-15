/*
 * Copyright (c) International Business Machines Corp., 2006
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
 * Author: Artem Bityutskiy (Битюцкий Артём)
 */

/*
 * The UBI Eraseblock Association (EBA) sub-system.
 *
 * This sub-system is responsible for I/O to/from logical eraseblock.
 *
 * Although in this implementation the EBA table is fully kept and managed in
 * RAM, which assumes poor scalability, it might be (partially) maintained on
 * flash in future implementations.
 *
 * The EBA sub-system implements per-logical eraseblock locking. Before
 * accessing a logical eraseblock it is locked for reading or writing. The
 * per-logical eraseblock locking is implemented by means of the lock tree. The
 * lock tree is an RB-tree which refers all the currently locked logical
 * eraseblocks. The lock tree elements are &struct ubi_ltree_entry objects.
 * They are indexed by (@vol_id, @lnum) pairs.
 *
 * EBA also maintains the global sequence counter which is incremented each
 * time a logical eraseblock is mapped to a physical eraseblock and it is
 * stored in the volume identifier header. This means that each VID header has
 * a unique sequence number. The sequence number is only increased an we assume
 * 64 bits is enough to never overflow.
 */

#include <linux/slab.h>
#include <linux/crc32.h>
#include <linux/err.h>
#include "eba.h"

/* Number of physical eraseblocks reserved for atomic LEB change operation */
#define EBA_RESERVED_PEBS 1


/**
 * ubi_next_sqnums - get a pool of contiguous sequence numbers.
 * @ubi: UBI device description object
 * @num: number of sequence numbers to reserve
 *
 * This function returns a pool of contiguous sequence numbers to use, which is
 * just the current global sequence counter value. It also increases the global
 * sequence counter accordingly.
 */
unsigned long long ubi_next_sqnums(struct ubi_device *ubi, int num)
{
	return atomic64_add_return(num, &ubi->global_sqnum);
}

/**
 * next_sqnum - get next sequence number.
 * @ubi: UBI device description object
 *
 * This function returns next sequence number to use, which is just the current
 * global sequence counter value. It also increases the global sequence
 * counter.
 */
unsigned long long ubi_next_sqnum(struct ubi_device *ubi)
{
	return atomic64_inc_return(&ubi->global_sqnum);
}

/**
 * ubi_eba_reserve_peb - reserve a PEB
 * @vol: UBI volume
 *
 * Reserve a PEB for a future write access.
 * Return 0 in case of success, an error code otherwise.
 */
int ubi_eba_reserve_peb(struct ubi_volume *vol)
{
	if (!vol->eba_tbl_ops->reserve_peb)
		return 0;

	return vol->eba_tbl_ops->reserve_peb(vol);
}

/**
 * ubi_eba_release_peb - release a previously reserved PEB
 * @vol: UBI volume
 *
 * Release a PEB. Usually done when you unmap a LEB.
 * Return 0 in case of success, an error code otherwise.
 */
void ubi_eba_release_peb(struct ubi_volume *vol)
{
	if (vol->eba_tbl_ops->release_peb)
		vol->eba_tbl_ops->release_peb(vol);
}

/**
 * ubi_eba_vid_hdr_fill_vol_info - fill a VID header with volume information
 * @vol: volume description object
 * @vid_hdr: VID header to fill
 */
void ubi_eba_vid_hdr_fill_vol_info(const struct ubi_volume *vol,
				   struct ubi_vid_hdr *vid_hdr)
{
	if (vol->vol_type == UBI_STATIC_VOLUME)
		vid_hdr->vol_type = UBI_VID_STATIC;
	else
		vid_hdr->vol_type = UBI_VID_DYNAMIC;

	switch(vol->vol_mode) {
	case UBI_VOL_MODE_SLC:
		vid_hdr->vol_mode = UBI_VID_MODE_SLC;
		break;
	case UBI_VOL_MODE_MLC_SAFE:
		vid_hdr->vol_mode = UBI_VID_MODE_MLC_SAFE;
		break;
	case UBI_VOL_MODE_NORMAL:
	default:
		ubi_assert(vol->vol_mode == UBI_VOL_MODE_NORMAL);
		vid_hdr->vol_mode = UBI_VID_MODE_NORMAL;
		break;
	}

	vid_hdr->vol_id = cpu_to_be32(vol->vol_id);
	vid_hdr->compat = ubi_get_compat(vol->ubi, vol->vol_id);
	vid_hdr->data_pad = cpu_to_be32(vol->data_pad);
}

/**
 * ubi_eba_get_ldesc - get information about a LEB
 * @vol: volume description object
 * @lnum: logical eraseblock number
 * @ldesc: the LEB descriptor to fill
 *
 * Used to query information about a specific LEB.
 * It is currently only returning the physical position of the LEB, but will be
 * extended to provide more information.
 */
void ubi_eba_get_ldesc(struct ubi_volume *vol, int lnum,
		       struct ubi_eba_leb_desc *ldesc)
{
	ubi_assert(vol && vol->eba_tbl && vol->eba_tbl_ops);
	ubi_assert(vol->eba_tbl_ops->get_ldesc);

	vol->eba_tbl_ops->get_ldesc(vol, lnum, ldesc);
}

/**
 * ubi_eba_destroy_table - destroy an EBA table
 * @tbl: the table to destroy
 *
 * Destroy an EBA table.
 */
void ubi_eba_destroy_table(const struct ubi_volume *vol,
			   struct ubi_eba_table *tbl)
{
	if (!tbl)
		return;

	vol->eba_tbl_ops->destroy(tbl);
}

/**
 * ubi_eba_copy_table - copy the EBA table attached to vol into another table
 * @vol: volume containing the EBA table to copy
 * @dst: destination
 * @nentries: number of entries to copy
 *
 * Copy the EBA table stored in vol into the one pointed by dst.
 */
void ubi_eba_copy_table(struct ubi_volume *vol, struct ubi_eba_table *dst,
			int nentries)
{
	ubi_assert(vol->eba_tbl_ops && vol->eba_tbl_ops->copy);
	ubi_assert(dst && vol && vol->eba_tbl);

	vol->eba_tbl_ops->copy(vol, dst, nentries);
}

/**
 * ubi_eba_replace_table - assign a new EBA table to a volume
 * @vol: volume containing the EBA table to copy
 * @tbl: new EBA table
 *
 * Assign a new EBA table to the volume and release the old one.
 */
void ubi_eba_replace_table(struct ubi_volume *vol, struct ubi_eba_table *tbl)
{
	struct ubi_eba_table *otbl = vol->eba_tbl;

	ubi_eba_destroy_table(vol, otbl);
	vol->eba_tbl = tbl;
}

/**
 * ltree_lookup - look up the lock tree.
 * @vol: UBI volume description object
 * @lnum: logical eraseblock number
 *
 * This function returns a pointer to the corresponding &struct ubi_ltree_entry
 * object if the logical eraseblock is locked and %NULL if it is not.
 * @vol->ltree_lock has to be locked.
 */
static struct ubi_ltree_entry *ltree_lookup(struct ubi_volume *vol, int lnum)
{
	struct rb_node *p;

	p = vol->ltree.rb_node;
	while (p) {
		struct ubi_ltree_entry *le;

		le = rb_entry(p, struct ubi_ltree_entry, rb);

		if (lnum < le->lnum)
			p = p->rb_left;
		else if (lnum > le->lnum)
			p = p->rb_right;
		else
			return le;
	}

	return NULL;
}

/**
 * ltree_add_entry - add new entry to the lock tree.
 * @vol: UBI volume description object
 * @lnum: logical eraseblock number
 *
 * This function adds new entry for logical eraseblock (@vol_id, @lnum) to the
 * lock tree. If such entry is already there, its usage counter is increased.
 * Returns pointer to the lock tree entry or %-ENOMEM if memory allocation
 * failed.
 */
static struct ubi_ltree_entry *ltree_add_entry(struct ubi_volume *vol,
					       int lnum)
{
	struct ubi_ltree_entry *le, *le1, *le_free;

	le = kmalloc(sizeof(struct ubi_ltree_entry), GFP_NOFS);
	if (!le)
		return ERR_PTR(-ENOMEM);

	le->users = 0;
	init_rwsem(&le->mutex);
	le->lnum = lnum;

	mutex_lock(&vol->ltree_lock);
	le1 = ltree_lookup(vol, lnum);

	if (le1) {
		/*
		 * This logical eraseblock is already locked. The newly
		 * allocated lock entry is not needed.
		 */
		le_free = le;
		le = le1;
	} else {
		struct rb_node **p, *parent = NULL;

		/*
		 * No lock entry, add the newly allocated one to the
		 * @vol->ltree RB-tree.
		 */
		le_free = NULL;

		p = &vol->ltree.rb_node;
		while (*p) {
			parent = *p;
			le1 = rb_entry(parent, struct ubi_ltree_entry, rb);

			ubi_assert(lnum != le1->lnum);
			if (lnum < le1->lnum)
				p = &(*p)->rb_left;
			else
				p = &(*p)->rb_right;
		}

		rb_link_node(&le->rb, parent, p);
		rb_insert_color(&le->rb, &vol->ltree);
	}
	le->users += 1;
	mutex_unlock(&vol->ltree_lock);

	kfree(le_free);
	return le;
}

/**
 * leb_read_lock - lock logical eraseblock for reading.
 * @vol: UBI volume description object
 * @lnum: logical eraseblock number
 *
 * This function locks a logical eraseblock for reading. Returns zero in case
 * of success and a negative error code in case of failure.
 */
static int leb_read_lock(struct ubi_volume *vol, int lnum)
{
	struct ubi_ltree_entry *le;

	le = ltree_add_entry(vol, lnum);
	if (IS_ERR(le))
		return PTR_ERR(le);
	down_read(&le->mutex);
	return 0;
}

/**
 * leb_read_trylock - try to lock logical eraseblock for reading.
 * @volume: volume object
 * @lnum: logical eraseblock number
 *
 * This function locks a logical eraseblock for reading if there is no
 * contention and does nothing if there is contention. Returns %0 in case of
 * success, %1 in case of contention, and and a negative error code in case of
 * failure.
 */
static int leb_read_trylock(struct ubi_volume *vol, int lnum)
{
	struct ubi_ltree_entry *le;

	le = ltree_add_entry(vol, lnum);
	if (IS_ERR(le))
		return PTR_ERR(le);
	if (down_read_trylock(&le->mutex))
		return 0;

	/* Contention, cancel */
	mutex_lock(&vol->ltree_lock);
	le->users -= 1;
	ubi_assert(le->users >= 0);
	if (le->users == 0) {
		rb_erase(&le->rb, &vol->ltree);
		kfree(le);
	}
	mutex_unlock(&vol->ltree_lock);

	return 1;
}

/**
 * ubi_eba_leb_read_trylock - try to lock logical eraseblock for reading.
 * @vol: volume object
 * @lnum: logical eraseblock number
 *
 * Wrapper around leb_read_trylock() for external users.
 */
int ubi_eba_leb_read_trylock(struct ubi_volume *vol, int lnum)
{
	return leb_read_trylock(vol, lnum);
}

/**
 * leb_read_unlock - unlock logical eraseblock.
 * @vol: UBI volume description object
 * @lnum: logical eraseblock number
 */
static void leb_read_unlock(struct ubi_volume *vol, int lnum)
{
	struct ubi_ltree_entry *le;

	mutex_lock(&vol->ltree_lock);
	le = ltree_lookup(vol, lnum);
	le->users -= 1;
	ubi_assert(le->users >= 0);
	up_read(&le->mutex);
	if (le->users == 0) {
		rb_erase(&le->rb, &vol->ltree);
		kfree(le);
	}
	mutex_unlock(&vol->ltree_lock);
}

/**
 * ubi_eba_leb_read_unlock - unlock a logical eraseblock for reading.
 * @vol: volume object
 * @lnum: logical eraseblock number
 *
 * Wrapper around leb_read_unlock() for external users.
 */
void ubi_eba_leb_read_unlock(struct ubi_volume *vol, int lnum)
{
	leb_read_unlock(vol, lnum);
}

/**
 * leb_write_lock - lock logical eraseblock for writing.
 * @vol: UBI volume description object
 * @lnum: logical eraseblock number
 *
 * This function locks a logical eraseblock for writing. Returns zero in case
 * of success and a negative error code in case of failure.
 */
static int leb_write_lock(struct ubi_volume *vol, int lnum)
{
	struct ubi_ltree_entry *le;

	le = ltree_add_entry(vol, lnum);
	if (IS_ERR(le))
		return PTR_ERR(le);
	down_write(&le->mutex);
	return 0;
}

/**
 * leb_write_lock - lock logical eraseblock for writing.
 * @vol: volume object
 * @lnum: logical eraseblock number
 *
 * This function locks a logical eraseblock for writing if there is no
 * contention and does nothing if there is contention. Returns %0 in case of
 * success, %1 in case of contention, and and a negative error code in case of
 * failure.
 */
static int leb_write_trylock(struct ubi_volume *vol, int lnum)
{
	struct ubi_ltree_entry *le;

	le = ltree_add_entry(vol, lnum);
	if (IS_ERR(le))
		return PTR_ERR(le);
	if (down_write_trylock(&le->mutex))
		return 0;

	/* Contention, cancel */
	mutex_lock(&vol->ltree_lock);
	le->users -= 1;
	ubi_assert(le->users >= 0);
	if (le->users == 0) {
		rb_erase(&le->rb, &vol->ltree);
		kfree(le);
	}
	mutex_unlock(&vol->ltree_lock);

	return 1;
}

/**
 * ubi_eba_leb_write_unlock - try to lock a logical eraseblock.
 * @vol: volume object
 * @lnum: logical eraseblock number
 *
 * Wrapper around leb_write_trylock() for external users.
 */
int ubi_eba_leb_write_trylock(struct ubi_volume *vol, int lnum)
{
	return leb_write_trylock(vol, lnum);
}

/**
 * leb_write_unlock - unlock logical eraseblock.
 * @vol: UBI volume description object
 * @lnum: logical eraseblock number
 */
static void leb_write_unlock(struct ubi_volume *vol, int lnum)
{
	struct ubi_ltree_entry *le;

	mutex_lock(&vol->ltree_lock);
	le = ltree_lookup(vol, lnum);
	le->users -= 1;
	ubi_assert(le->users >= 0);
	up_write(&le->mutex);
	if (le->users == 0) {
		rb_erase(&le->rb, &vol->ltree);
		kfree(le);
	}
	mutex_unlock(&vol->ltree_lock);
}

/**
 * ubi_eba_leb_write_unlock - unlock logical eraseblock.
 * @vol: volume object
 * @lnum: logical eraseblock number
 *
 * Wrapper around leb_write_unlock() for external users.
 */
void ubi_eba_leb_write_unlock(struct ubi_volume *vol, int lnum)
{
	leb_write_unlock(vol, lnum);
}

/**
 * ubi_eba_is_mapped - check if a LEB is mapped.
 * @vol: volume description object
 * @lnum: logical eraseblock number
 *
 * This function returns true if the LEB is mapped, false otherwise.
 */
bool ubi_eba_is_mapped(struct ubi_volume *vol, int lnum)
{
	struct ubi_eba_leb_desc ldesc;

	ubi_eba_get_ldesc(vol, lnum, &ldesc);

	return ldesc.pnum >= 0;
}

/**
 * ubi_eba_invalidate_entry - invalidate an entry in the EBA table.
 * @vol: volume description object
 * @lnum: logical eraseblock number
 *
 * This function invalidate the LEB to PEB mapping of LEB @lnum.
 * Must be called with ->fm_eba_sem held in read mode.
 *
 * Return the PEB descriptor pointing to the PEB containing LEB @lnum, NULL if
 * the LEB is already unmapped, and an error pointer otherwise.
 * If a valid PEB desc is returned, it should be freed using ubi_wl_put_peb().
 */
static struct ubi_peb_desc *ubi_eba_invalidate_entry(struct ubi_volume *vol,
						     int lnum)
{
	struct ubi_peb_desc *pdesc;
	struct ubi_eba_leb_desc nldesc;

	pdesc = vol->eba_tbl_ops->invalidate_entry(vol, lnum);

	ubi_eba_get_ldesc(vol, lnum, &nldesc);
	ubi_assert(nldesc.lnum == lnum);
	ubi_assert(nldesc.pnum == UBI_LEB_UNMAPPED);
	ubi_assert(nldesc.lpos == UBI_EBA_NA);

	return pdesc;
}

/**
 * ubi_eba_update_leb - invalidate an entry in the EBA table.
 * @vol: volume description object
 * @ldesc: logical eraseblock descriptor
 *
 * This function updates the LEB to PEB mapping of LEB @lnum.
 * Must be called with ->fm_eba_sem held in read mode.
 *
 * Return the PEB descriptor pointing to the old PEB containing LEB @lnum,
 * NULL if the LEB was previously unmapped, and an error pointer otherwise.
 * If a valid PEB desc is returned, it should be freed using ubi_wl_put_peb().
 */
static struct ubi_peb_desc *ubi_eba_update_entry(struct ubi_volume *vol,
					const struct ubi_eba_leb_desc *ldesc)
{
	struct ubi_eba_leb_desc nldesc;

	return vol->eba_tbl_ops->update_entry(vol, ldesc);

	ubi_eba_get_ldesc(vol, ldesc->lnum, &nldesc);

	ubi_assert(ldesc->lnum == nldesc.lnum);
	ubi_assert(ldesc->pnum == nldesc.pnum);
	ubi_assert(ldesc->lpos == nldesc.lpos);
}

/**
 * ubi_eba_init_entry - initialize an EBA table entry.
 * @vol: volume description object
 * @aleb: the LEB description retrieved at attach time
 *
 * Fill an EBA table entry with the attach information.
 *
 * Returns 0 in case of success, an error code otherwise.
 */
static int ubi_eba_init_entry(struct ubi_volume *vol,
			      struct ubi_ainf_leb *aleb)
{
	return vol->eba_tbl_ops->init_entry(vol, aleb);
}

int ubi_eba_gc_pebs(struct ubi_volume *vol)
{
	if (vol->eba_tbl_ops->gc_pebs)
		return vol->eba_tbl_ops->gc_pebs(vol);

	return 0;
}

int ubi_eba_select_leb_for_conso(struct ubi_volume *vol,
				 const struct ubi_consolidated_peb *cpeb,
				 struct ubi_eba_leb_desc *ldesc)
{
	ubi_assert(vol->eba_tbl_ops->select_leb_for_conso);

	return vol->eba_tbl_ops->select_leb_for_conso(vol, cpeb, ldesc);
}

void ubi_eba_update_consolidated_lebs(struct ubi_volume *vol,
				 struct ubi_consolidated_peb *cpeb,
				 struct ubi_peb_desc **pdescs)
{
	ubi_assert(vol->eba_tbl_ops->update_consolidated_lebs);

	vol->eba_tbl_ops->update_consolidated_lebs(vol, cpeb, pdescs);
}

enum ubi_eba_conso_req ubi_eba_get_conso_req(struct ubi_volume *vol)
{
	ubi_assert(vol->eba_tbl_ops->get_conso_req);

	return vol->eba_tbl_ops->get_conso_req(vol);
}

/**
 * ubi_eba_unmap_leb - un-map logical eraseblock.
 * @ubi: UBI device description object
 * @vol: volume description object
 * @lnum: logical eraseblock number
 *
 * This function un-maps logical eraseblock @lnum and schedules corresponding
 * physical eraseblock for erasure. Returns zero in case of success and a
 * negative error code in case of failure.
 */
int ubi_eba_unmap_leb(struct ubi_device *ubi, struct ubi_volume *vol,
		      int lnum)
{
	int err, vol_id = vol->vol_id, pnum;
	struct ubi_peb_desc *pdesc;

	if (ubi->ro_mode)
		return -EROFS;

	err = leb_write_lock(vol, lnum);
	if (err)
		return err;

	down_read(&ubi->eba_sem);
	pdesc = ubi_eba_invalidate_entry(vol, lnum);
	up_read(&ubi->eba_sem);

	if (IS_ERR(pdesc)) {
		err = PTR_ERR(pdesc);
	} else if (pdesc) {
		dbg_eba("erase LEB %d:%d, PEB %d", vol_id, lnum, pnum);

		err = ubi_wl_put_peb(ubi, pdesc, 0);
	}

	leb_write_unlock(vol, lnum);

	if (!err)
		err = ubi_eba_gc_pebs(vol);

	return err;
}

/**
 * ubi_eba_io_mode_and_offs - extract the I/O mode and offset from a LEB
 *			      descriptor.
 * @vol: volume description object
 * @ldesc: logical eraseblock descriptor
 * @io_mode: io_mode variable to fill
 * @offset: offset variable to fill
 *
 * This function analyzes the vol_mode and LEB descriptor to extract the
 * appropriate I/O mode and offset.
 * This is particularly usefull when you want to read some data from an
 * MLC safe volume and don't know if the LEB has been consolidated or not.
 * Returns 0 in case of success, an error code otherwise.
 */
static int ubi_eba_io_mode_and_offs(struct ubi_volume *vol,
				    const struct ubi_eba_leb_desc *ldesc,
				    enum ubi_io_mode *io_mode, int *offset)
{
	*offset = 0;

	switch (vol->vol_mode) {
	case UBI_VOL_MODE_NORMAL:
		*io_mode = UBI_IO_MODE_NORMAL;
		break;
	case UBI_VOL_MODE_SLC:
		*io_mode = UBI_IO_MODE_SLC;
		break;
	case UBI_VOL_MODE_MLC_SAFE:
		if (ldesc->lpos == UBI_EBA_NA) {
			*io_mode = UBI_IO_MODE_SLC;
		} else {
			*io_mode = UBI_IO_MODE_NORMAL;
			*offset = vol->leb_size * ldesc->lpos;
		}

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * ubi_eba_read_leb_data - read LEB data.
 * @vol: volume description object
 * @ldesc: logical eraseblock descriptor
 * @buf: buffer to store data in
 * @loffset: LEB offset
 * @len: data length
 *
 * This is a convenient wrapper abstracting the I/O mode that was used to
 * write into a PEB.
 * Returns 0 in case of success, an error code otherwise.
 */
int ubi_eba_read_leb_data(struct ubi_volume *vol,
			  const struct ubi_eba_leb_desc *ldesc,
			  void *buf, int loffset, int len)
{
	struct ubi_device *ubi = vol->ubi;
	enum ubi_io_mode io_mode;
	int offset, err;

	err = ubi_eba_io_mode_and_offs(vol, ldesc, &io_mode, &offset);
	if (err)
		return err;

	offset += loffset;

	return ubi_io_read_data(ubi, buf, ldesc->pnum, offset, len, io_mode);
}

/**
 * read_leb_data - read LEB data.
 * @vol: volume description object
 * @ldesc: logical eraseblock descriptor
 * @buf: buffer to store data in
 * @loffset: LEB offset
 * @len: data length
 *
 * This is a convenient wrapper abstracting the I/O mode that you should use to
 * write into a PEB.
 * Returns 0 in case of success, an error code otherwise.
 */
int ubi_eba_write_leb_data(struct ubi_volume *vol,
			   const struct ubi_eba_leb_desc *ldesc,
			   const void *buf, int loffset, int len)
{
	struct ubi_device *ubi = vol->ubi;
	enum ubi_io_mode io_mode;
	int offset, err;

	err = ubi_eba_io_mode_and_offs(vol, ldesc, &io_mode, &offset);
	if (err)
		return err;

	offset += loffset;

	return ubi_io_write_data(ubi, buf, ldesc->pnum, offset, len, io_mode);
}

/**
 * ubi_eba_read_leb - read data.
 * @ubi: UBI device description object
 * @vol: volume description object
 * @lnum: logical eraseblock number
 * @buf: buffer to store the read data
 * @offset: offset from where to read
 * @len: how many bytes to read
 * @check: data CRC check flag
 *
 * If the logical eraseblock @lnum is unmapped, @buf is filled with 0xFF
 * bytes. The @check flag only makes sense for static volumes and forces
 * eraseblock data CRC checking.
 *
 * In case of success this function returns zero. In case of a static volume,
 * if data CRC mismatches - %-EBADMSG is returned. %-EBADMSG may also be
 * returned for any volume type if an ECC error was detected by the MTD device
 * driver. Other negative error cored may be returned in case of other errors.
 */
int ubi_eba_read_leb(struct ubi_device *ubi, struct ubi_volume *vol, int lnum,
		     void *buf, int offset, int len, int check)
{
	int err, scrub = 0, vol_id = vol->vol_id;
	struct ubi_vid_io_buf *vidb;
	struct ubi_vid_hdr *vid_hdr;
	uint32_t uninitialized_var(crc);
	struct ubi_eba_leb_desc ldesc;

	err = leb_read_lock(vol, lnum);
	if (err)
		return err;

	ubi_eba_get_ldesc(vol, lnum, &ldesc);
	if (ldesc.pnum < 0) {
		/*
		 * The logical eraseblock is not mapped, fill the whole buffer
		 * with 0xFF bytes. The exception is static volumes for which
		 * it is an error to read unmapped logical eraseblocks.
		 */
		dbg_eba("read %d bytes from offset %d of LEB %d:%d (unmapped)",
			len, offset, vol_id, lnum);
		leb_read_unlock(vol, lnum);
		ubi_assert(vol->vol_type != UBI_STATIC_VOLUME);
		memset(buf, 0xFF, len);
		return 0;
	}

	dbg_eba("read %d bytes from offset %d of LEB %d:%d, PEB %d",
		len, offset, vol_id, lnum, ldesc.pnum);

	if (vol->vol_type == UBI_DYNAMIC_VOLUME)
		check = 0;

retry:
	if (check) {
		vidb = ubi_alloc_vid_buf(ubi, GFP_NOFS);
		if (!vidb) {
			err = -ENOMEM;
			goto out_unlock;
		}

		vid_hdr = ubi_get_vid_hdr(vidb);

		err = ubi_io_read_vid_hdr(ubi, ldesc.pnum, vidb, 1);
		if (err && err != UBI_IO_BITFLIPS) {
			if (err > 0) {
				/*
				 * The header is either absent or corrupted.
				 * The former case means there is a bug -
				 * switch to read-only mode just in case.
				 * The latter case means a real corruption - we
				 * may try to recover data. FIXME: but this is
				 * not implemented.
				 */
				if (err == UBI_IO_BAD_HDR_EBADMSG ||
				    err == UBI_IO_BAD_HDR) {
					ubi_warn(ubi, "corrupted VID header at PEB %d, LEB %d:%d",
						 ldesc.pnum, vol_id, lnum);
					err = -EBADMSG;
				} else {
					/*
					 * Ending up here in the non-Fastmap case
					 * is a clear bug as the VID header had to
					 * be present at scan time to have it referenced.
					 * With fastmap the story is more complicated.
					 * Fastmap has the mapping info without the need
					 * of a full scan. So the LEB could have been
					 * unmapped, Fastmap cannot know this and keeps
					 * the LEB referenced.
					 * This is valid and works as the layer above UBI
					 * has to do bookkeeping about used/referenced
					 * LEBs in any case.
					 */
					if (ubi->fast_attach) {
						err = -EBADMSG;
					} else {
						err = -EINVAL;
						ubi_ro_mode(ubi);
					}
				}
			}
			goto out_free;
		} else if (err == UBI_IO_BITFLIPS)
			scrub = 1;

		ubi_assert(lnum < be32_to_cpu(vid_hdr->used_ebs));
		ubi_assert(len == be32_to_cpu(vid_hdr->data_size));

		crc = be32_to_cpu(vid_hdr->data_crc);
		ubi_free_vid_buf(vidb);
	}

	err = ubi_eba_read_leb_data(vol, &ldesc, buf, offset, len);
	if (err) {
		if (err == UBI_IO_BITFLIPS)
			scrub = 1;
		else if (mtd_is_eccerr(err)) {
			if (vol->vol_type == UBI_DYNAMIC_VOLUME)
				goto out_unlock;
			scrub = 1;
			if (!check) {
				ubi_msg(ubi, "force data checking");
				check = 1;
				goto retry;
			}
		} else
			goto out_unlock;
	}

	if (check) {
		uint32_t crc1 = crc32(UBI_CRC32_INIT, buf, len);
		if (crc1 != crc) {
			ubi_warn(ubi, "CRC error: calculated %#08x, must be %#08x",
				 crc1, crc);
			err = -EBADMSG;
			goto out_unlock;
		}
	}

	if (scrub)
		err = ubi_wl_scrub_peb(ubi, ldesc.pnum);

	leb_read_unlock(vol, lnum);
	return err;

out_free:
	ubi_free_vid_buf(vidb);
out_unlock:
	leb_read_unlock(vol, lnum);
	return err;
}

/**
 * ubi_eba_read_leb_sg - read data into a scatter gather list.
 * @ubi: UBI device description object
 * @vol: volume description object
 * @lnum: logical eraseblock number
 * @sgl: UBI scatter gather list to store the read data
 * @offset: offset from where to read
 * @len: how many bytes to read
 * @check: data CRC check flag
 *
 * This function works exactly like ubi_eba_read_leb(). But instead of
 * storing the read data into a buffer it writes to an UBI scatter gather
 * list.out_free
 */
int ubi_eba_read_leb_sg(struct ubi_device *ubi, struct ubi_volume *vol,
			struct ubi_sgl *sgl, int lnum, int offset, int len,
			int check)
{
	int to_read;
	int ret;
	struct scatterlist *sg;

	for (;;) {
		ubi_assert(sgl->list_pos < UBI_MAX_SG_COUNT);
		sg = &sgl->sg[sgl->list_pos];
		if (len < sg->length - sgl->page_pos)
			to_read = len;
		else
			to_read = sg->length - sgl->page_pos;

		ret = ubi_eba_read_leb(ubi, vol, lnum,
				       sg_virt(sg) + sgl->page_pos, offset,
				       to_read, check);
		if (ret < 0)
			return ret;

		offset += to_read;
		len -= to_read;
		if (!len) {
			sgl->page_pos += to_read;
			if (sgl->page_pos == sg->length) {
				sgl->list_pos++;
				sgl->page_pos = 0;
			}

			break;
		}

		sgl->list_pos++;
		sgl->page_pos = 0;
	}

	return ret;
}

/**
 * try_recover_peb - try to recover from write failure.
 * @vol: volume description object
 * @pnum: the physical eraseblock to recover
 * @lnum: logical eraseblock number
 * @buf: data which was not written because of the write failure
 * @offset: offset of the failed write
 * @len: how many bytes should have been written
 * @vidb: VID buffer
 * @retry: whether the caller should retry in case of failure
 *
 * This function is called in case of a write failure and moves all good data
 * from the potentially bad physical eraseblock to a good physical eraseblock.
 * This function also writes the data which was not written due to the failure.
 * Returns 0 in case of success, and a negative error code in case of failure.
 * In case of failure, the %retry parameter is set to false if this is a fatal
 * error (retrying won't help), and true otherwise.
 */
static int try_recover_peb(struct ubi_volume *vol, int pnum, int lnum,
			   const void *buf, int offset, int len,
			   struct ubi_vid_io_buf *vidb, bool *retry)
{
	struct ubi_device *ubi = vol->ubi;
	struct ubi_vid_hdr *vid_hdr = ubi_get_vid_hdr(vidb);
	int new_pnum, err, data_size;
	struct ubi_peb_desc *pdesc = NULL;
	enum ubi_io_mode io_mode;
	uint32_t crc;

	*retry = false;

	new_pnum = ubi_wl_get_peb(ubi);
	if (new_pnum < 0) {
		err = new_pnum;
		goto out_put;
	}

	ubi_msg(ubi, "recover PEB %d, move data to PEB %d",
		pnum, new_pnum);

	err = ubi_io_read_vid_hdr(ubi, pnum, vidb, 1);
	if (err && err != UBI_IO_BITFLIPS) {
		if (err > 0)
			err = -EIO;
		goto out_put;
	}

	vid_hdr = ubi_get_vid_hdr(vidb);
	ubi_assert(vid_hdr->vol_type == UBI_VID_DYNAMIC);

	io_mode = ubi_io_mode_from_vid_hdr(vid_hdr);

	mutex_lock(&ubi->buf_mutex);
	memset(ubi->peb_buf + offset, 0xFF, len);

	/* Read everything before the area where the write failure happened */
	if (offset > 0) {
		err = ubi_io_read_data(ubi, ubi->peb_buf, pnum, 0,
				       offset, io_mode);
		if (err && err != UBI_IO_BITFLIPS)
			goto out_unlock;
	}

	*retry = true;

	memcpy(ubi->peb_buf + offset, buf, len);

	data_size = offset + len;
	crc = crc32(UBI_CRC32_INIT, ubi->peb_buf, data_size);
	vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
	vid_hdr->copy_flag = 1;
	vid_hdr->data_size = cpu_to_be32(data_size);
	vid_hdr->data_crc = cpu_to_be32(crc);
	err = ubi_io_write_vid_hdr(ubi, new_pnum, vidb);
	if (err)
		goto out_unlock;

	err = ubi_io_write_data(ubi, ubi->peb_buf, new_pnum, 0, data_size,
				io_mode);

out_unlock:
	mutex_unlock(&ubi->buf_mutex);

	if (!err) {
		struct ubi_eba_leb_desc ldesc;

		ldesc.lnum = lnum;
		ldesc.pnum = new_pnum;
		ldesc.lpos = UBI_EBA_NA;

		pdesc = ubi_eba_update_entry(vol, &ldesc);
	}

out_put:
	up_read(&ubi->eba_sem);

	if (!err) {
		ubi_msg(ubi, "data was successfully recovered");
	} else if (new_pnum >= 0) {
		/*
		 * Bad luck? This physical eraseblock is bad too? Crud. Let's
		 * try to get another one.
		 */
		pdesc = ubi_alloc_pdesc(ubi, GFP_NOFS);
		if (!pdesc)
			return -ENOMEM;

		pdesc->pnum = new_pnum;
		pdesc->lnums[0] = lnum;
		pdesc->vol_id = vol->vol_id;

		ubi_warn(ubi, "failed to write to PEB %d", new_pnum);
	}

	if (pdesc)
		ubi_wl_put_peb(ubi, pdesc, 1);

	ubi_eba_gc_pebs(vol);

	return err;
}

/**
 * recover_peb - recover from write failure.
 * @ubi: UBI device description object
 * @pnum: the physical eraseblock to recover
 * @vol_id: volume ID
 * @lnum: logical eraseblock number
 * @buf: data which was not written because of the write failure
 * @offset: offset of the failed write
 * @len: how many bytes should have been written
 *
 * This function is called in case of a write failure and moves all good data
 * from the potentially bad physical eraseblock to a good physical eraseblock.
 * This function also writes the data which was not written due to the failure.
 * Returns 0 in case of success, and a negative error code in case of failure.
 * This function tries %UBI_IO_RETRIES before giving up.
 */
static int recover_peb(struct ubi_device *ubi, int pnum, int vol_id, int lnum,
		       const void *buf, int offset, int len)
{
	int err, idx = vol_id2idx(ubi, vol_id), tries;
	struct ubi_volume *vol = ubi->volumes[idx];
	struct ubi_vid_io_buf *vidb;

	vidb = ubi_alloc_vid_buf(ubi, GFP_NOFS);
	if (!vidb)
		return -ENOMEM;

	for (tries = 0; tries <= UBI_IO_RETRIES; tries++) {
		bool retry;

		err = try_recover_peb(vol, pnum, lnum, buf, offset, len, vidb,
				      &retry);
		if (!err || !retry)
			break;

		ubi_msg(ubi, "try again");
	}

	ubi_free_vid_buf(vidb);

	return err;
}

/**
 * ubi_eba_try_write_vid_and_data - try to write VID header and data to a new
 *				    PEB.
 * @vol: volume description object
 * @lnum: logical eraseblock number
 * @vidb: the VID buffer to write
 * @buf: buffer containing the data
 * @offset: where to start writing data
 * @len: how many bytes should be written
 *
 * This function tries to write VID header and data belonging to logical
 * eraseblock @lnum of volume @vol to a new physical eraseblock. Returns zero
 * in case of success and a negative error code in case of failure.
 * In case of error, it is possible that something was still written to the
 * flash media, but may be some garbage.
 */
int ubi_eba_try_write_vid_and_data(struct ubi_volume *vol, int lnum,
				   struct ubi_vid_io_buf *vidb,
				   const void *buf, int offset, int len)
{
	struct ubi_vid_hdr *vid_hdr = ubi_get_vid_hdr(vidb);
	struct ubi_eba_leb_desc nldesc, oldesc;
	struct ubi_device *ubi = vol->ubi;
	int err, vol_id = vol->vol_id;
	struct ubi_peb_desc *pdesc;
	enum ubi_io_mode io_mode;

	ubi_eba_get_ldesc(vol, lnum, &oldesc);
	nldesc.lpos = UBI_EBA_NA;
	nldesc.lnum = lnum;
	nldesc.pnum = ubi_wl_get_peb(ubi);
	if (nldesc.pnum < 0) {
		err = nldesc.pnum;
		goto out_put;
	}

	dbg_eba("write VID hdr and %d bytes at offset %d of LEB %d:%d, PEB %d",
		len, offset, vol_id, nldesc.lnum, nldesc.pnum);

	io_mode = ubi_io_mode_from_vid_hdr(vid_hdr);

	err = ubi_io_write_vid_hdr(ubi, nldesc.pnum, vidb);
	if (err) {
		ubi_warn(ubi, "failed to write VID header to LEB %d:%d, PEB %d",
			 vol_id, nldesc.lnum, nldesc.pnum);
		goto out_put;
	}

	if (len) {
		err = ubi_eba_write_leb_data(vol, &nldesc, buf, offset, len);
		if (err) {
			ubi_warn(ubi,
				 "failed to write %d bytes at offset %d of LEB %d:%d, PEB %d",
				 len, offset, vol_id, nldesc.lnum,
				 nldesc.pnum);
			goto out_put;
		}
	}

	if (!err)
		pdesc = ubi_eba_update_entry(vol, &nldesc);

out_put:
	up_read(&ubi->eba_sem);

	if (err && nldesc.pnum >= 0) {
		pdesc = ubi_alloc_pdesc(ubi, GFP_NOFS);
		if (!pdesc)
			return -ENOMEM;

		pdesc->pnum = nldesc.pnum;
		pdesc->lnums[0] = lnum;
		pdesc->vol_id = vol_id;
	}

	if (pdesc)
		ubi_wl_put_peb(ubi, pdesc, err ? 1 : 0);

	ubi_eba_gc_pebs(vol);

	return err;
}

/**
 * prepare_leb_write - prepare for a write operation on a LEB.
 * @vol: volume description object
 * @ldesc: LEB descriptor
 * @offset: offset the write will start at
 *
 * This function prepares everything for a write access.
 * On MLC safe volumes, this is used to unconsolidated LEBs if they have
 * previously been consolidated.
 * This function returns 0 on success, an error code otherwise.
 */
static int prepare_leb_write(struct ubi_volume *vol,
			     struct ubi_eba_leb_desc *ldesc, int offset)
{
	if (!vol->eba_tbl_ops->prepare_leb_write)
		return 0;

	return vol->eba_tbl_ops->prepare_leb_write(vol, ldesc, offset);
}

/**
 * ubi_eba_write_leb - write data to dynamic volume.
 * @ubi: UBI device description object
 * @vol: volume description object
 * @lnum: logical eraseblock number
 * @buf: the data to write
 * @offset: offset within the logical eraseblock where to write
 * @len: how many bytes to write
 *
 * This function writes data to logical eraseblock @lnum of a dynamic volume
 * @vol. Returns zero in case of success and a negative error code in case
 * of failure. In case of error, it is possible that something was still
 * written to the flash media, but may be some garbage.
 * This function retries %UBI_IO_RETRIES times before giving up.
 */
int ubi_eba_write_leb(struct ubi_device *ubi, struct ubi_volume *vol, int lnum,
		      const void *buf, int offset, int len)
{
	int err, tries, vol_id = vol->vol_id;
	struct ubi_vid_io_buf *vidb;
	struct ubi_eba_leb_desc ldesc;
	struct ubi_vid_hdr *vid_hdr;
	bool switch_to_ro = true;
	bool peb_reserved = false;

	if (ubi->ro_mode)
		return -EROFS;

	err = leb_write_lock(vol, lnum);
	if (err)
		return err;

	ubi_eba_get_ldesc(vol, lnum, &ldesc);

	err = prepare_leb_write(vol, &ldesc, offset);
	if (err)
		goto out;

	if (ldesc.pnum >= 0) {
		dbg_eba("write %d bytes at offset %d of LEB %d:%d, PEB %d",
			len, offset, vol_id, lnum, ldesc.pnum);

		err = ubi_eba_write_leb_data(vol, &ldesc, buf, offset, len);
		if (err) {
			ubi_warn(ubi, "failed to write data to PEB %d",
				 ldesc.pnum);
			if (err == -EIO && ubi->bad_allowed)
				err = recover_peb(ubi, ldesc.pnum, vol_id,
						  lnum, buf, offset, len);
		}

		goto out;
	}

	err = ubi_eba_reserve_peb(vol);
	if (err) {
		switch_to_ro = false;
		goto out;
	}

	peb_reserved = true;

	/*
	 * The logical eraseblock is not mapped. We have to get a free physical
	 * eraseblock and write the volume identifier header there first.
	 */
	vidb = ubi_alloc_vid_buf(ubi, GFP_NOFS);
	if (!vidb) {
		leb_write_unlock(vol, lnum);
		switch_to_ro = false;
		err = -ENOMEM;
		goto out;
	}

	vid_hdr = ubi_get_vid_hdr(vidb);

	ubi_eba_vid_hdr_fill_vol_info(vol, vid_hdr);
	vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
	vid_hdr->lnum = cpu_to_be32(lnum);

	for (tries = 0; tries <= UBI_IO_RETRIES; tries++) {
		err = ubi_eba_try_write_vid_and_data(vol, lnum, vidb, buf,
						     offset, len);
		if (err != -EIO || !ubi->bad_allowed)
			break;

		/*
		 * Fortunately, this is the first write operation to this
		 * physical eraseblock, so just put it and request a new one.
		 * We assume that if this physical eraseblock went bad, the
		 * erase code will handle that.
		 */
		vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
		ubi_msg(ubi, "try another PEB");
	}

	ubi_free_vid_buf(vidb);


out:
	if (err) {
		if (switch_to_ro)
			ubi_ro_mode(ubi);

		if (peb_reserved)
			ubi_eba_release_peb(vol);
	}

	leb_write_unlock(vol, lnum);

	return err;
}

/**
 * ubi_eba_write_leb_st - write data to static volume.
 * @ubi: UBI device description object
 * @vol: volume description object
 * @lnum: logical eraseblock number
 * @buf: data to write
 * @len: how many bytes to write
 * @used_ebs: how many logical eraseblocks will this volume contain
 *
 * This function writes data to logical eraseblock @lnum of static volume
 * @vol. The @used_ebs argument should contain total number of logical
 * eraseblock in this static volume.
 *
 * When writing to the last logical eraseblock, the @len argument doesn't have
 * to be aligned to the minimal I/O unit size. Instead, it has to be equivalent
 * to the real data size, although the @buf buffer has to contain the
 * alignment. In all other cases, @len has to be aligned.
 *
 * It is prohibited to write more than once to logical eraseblocks of static
 * volumes. This function returns zero in case of success and a negative error
 * code in case of failure.
 */
int ubi_eba_write_leb_st(struct ubi_device *ubi, struct ubi_volume *vol,
			 int lnum, const void *buf, int len, int used_ebs)
{
	int err, tries, data_size = len;
	struct ubi_vid_io_buf *vidb;
	struct ubi_vid_hdr *vid_hdr;
	uint32_t crc;

	if (ubi->ro_mode)
		return -EROFS;

	if (lnum == used_ebs - 1)
		/* If this is the last LEB @len may be unaligned */
		len = ALIGN(data_size, ubi->min_io_size);
	else
		ubi_assert(!(len & (ubi->min_io_size - 1)));

	vidb = ubi_alloc_vid_buf(ubi, GFP_NOFS);
	if (!vidb)
		return -ENOMEM;

	vid_hdr = ubi_get_vid_hdr(vidb);

	err = leb_write_lock(vol, lnum);
	if (err)
		goto out;

	err = ubi_eba_reserve_peb(vol);
	if (err)
		goto out_unlock_leb;

	ubi_eba_vid_hdr_fill_vol_info(vol, vid_hdr);
	vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
	vid_hdr->lnum = cpu_to_be32(lnum);

	crc = crc32(UBI_CRC32_INIT, buf, data_size);

	vid_hdr->data_size = cpu_to_be32(data_size);
	vid_hdr->used_ebs = cpu_to_be32(used_ebs);
	vid_hdr->data_crc = cpu_to_be32(crc);

	ubi_assert(!ubi_eba_is_mapped(vol, lnum));

	for (tries = 0; tries <= UBI_IO_RETRIES; tries++) {
		err = ubi_eba_try_write_vid_and_data(vol, lnum, vidb, buf, 0,
						     len);
		if (err != -EIO || !ubi->bad_allowed)
			break;

		vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
		ubi_msg(ubi, "try another PEB");
	}

	if (err) {
		ubi_eba_release_peb(vol);
		ubi_ro_mode(ubi);
	}

out_unlock_leb:
	leb_write_unlock(vol, lnum);

out:
	ubi_free_vid_buf(vidb);

	return err;
}

/*
 * ubi_eba_atomic_leb_change - change logical eraseblock atomically.
 * @ubi: UBI device description object
 * @vol: volume description object
 * @lnum: logical eraseblock number
 * @buf: data to write
 * @len: how many bytes to write
 *
 * This function changes the contents of a logical eraseblock atomically. @buf
 * has to contain new logical eraseblock data, and @len - the length of the
 * data, which has to be aligned. This function guarantees that in case of an
 * unclean reboot the old contents is preserved. Returns zero in case of
 * success and a negative error code in case of failure.
 *
 * UBI reserves one LEB for the "atomic LEB change" operation, so only one
 * LEB change may be done at a time. This is ensured by @ubi->alc_mutex.
 */
int ubi_eba_atomic_leb_change(struct ubi_device *ubi, struct ubi_volume *vol,
			      int lnum, const void *buf, int len)
{
	int err, tries, vol_id = vol->vol_id;
	struct ubi_vid_io_buf *vidb;
	struct ubi_vid_hdr *vid_hdr;
	uint32_t crc;

	if (ubi->ro_mode)
		return -EROFS;

	vidb = ubi_alloc_vid_buf(ubi, GFP_NOFS);
	if (!vidb)
		return -ENOMEM;

	vid_hdr = ubi_get_vid_hdr(vidb);

	mutex_lock(&ubi->alc_mutex);
	err = leb_write_lock(vol, lnum);
	if (err)
		goto out_mutex;

	err = ubi_eba_reserve_peb(vol);
	if (err)
		goto out_unlock_leb;

	ubi_eba_vid_hdr_fill_vol_info(vol, vid_hdr);
	vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
	vid_hdr->lnum = cpu_to_be32(lnum);

	/*
	 * Special case when data length is zero. In this case we don't
	 * need to set the copy flag, the data_size and data_crc.
	 */
	if (len) {
		crc = crc32(UBI_CRC32_INIT, buf, len);

		vid_hdr->data_size = cpu_to_be32(len);
		vid_hdr->copy_flag = 1;
		vid_hdr->data_crc = cpu_to_be32(crc);
	}

	dbg_eba("change LEB %d:%d", vol_id, lnum);

	for (tries = 0; tries <= UBI_IO_RETRIES; tries++) {
		err = ubi_eba_try_write_vid_and_data(vol, lnum, vidb, buf, 0,
						     len);
		if (err != -EIO || !ubi->bad_allowed)
			break;

		vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
		ubi_msg(ubi, "try another PEB");
	}

	/*
	 * This flash device does not admit of bad eraseblocks or
	 * something nasty and unexpected happened. Switch to read-only
	 * mode just in case.
	 */
	if (err) {
		ubi_eba_release_peb(vol);
		ubi_ro_mode(ubi);
	}

out_unlock_leb:
	leb_write_unlock(vol, lnum);

out_mutex:
	mutex_unlock(&ubi->alc_mutex);
	ubi_free_vid_buf(vidb);
	return err;
}

/**
 * ubi_eba_copy_leb - copy logical eraseblock.
 * @ubi: UBI device description object
 * @from: physical eraseblock number from where to copy
 * @to: physical eraseblock number where to copy
 * @vid_hdr: VID header of the @from physical eraseblock
 *
 * This function copies logical eraseblock from physical eraseblock @from to
 * physical eraseblock @to. The @vid_hdr buffer may be changed by this
 * function. Returns:
 *   o %0 in case of success;
 *   o %MOVE_CANCEL_RACE, %MOVE_TARGET_WR_ERR, %MOVE_TARGET_BITFLIPS, etc;
 *   o a negative error code in case of failure.
 */
int ubi_eba_copy_leb(struct ubi_device *ubi, int from, int to,
		     struct ubi_vid_io_buf *vidb)
{
	int vol_id, lnum, idx, nhdrs;
	struct ubi_vid_hdr *vid_hdr = ubi_get_vid_hdr(vidb);
	struct ubi_volume *vol;

	ubi_assert(rwsem_is_locked(&ubi->eba_sem));

	vol_id = be32_to_cpu(vid_hdr->vol_id);
	nhdrs = ubi_get_nhdrs(vidb);

	dbg_wl("copy LEB %d:%d, PEB %d to PEB %d", vol_id, lnum, from, to);

	idx = vol_id2idx(ubi, vol_id);
	mutex_lock(&ubi->volumes_lock);
	/*
	 * Note, we may race with volume deletion, which means that the volume
	 * this logical eraseblock belongs to might be being deleted. Since the
	 * volume deletion un-maps all the volume's logical eraseblocks, it will
	 * be locked in 'ubi_wl_put_peb()' and wait for the WL worker to finish.
	 */
	vol = ubi->volumes[idx];
	mutex_unlock(&ubi->volumes_lock);
	if (!vol) {
		/* No need to do further work, cancel */
		dbg_wl("volume %d is being removed, cancel", vol_id);
		return MOVE_CANCEL_RACE;
	}

	return vol->eba_tbl_ops->copy_peb(vol, from, to, vidb);
}

/**
 * ubi_eba_create_table - allocate a new EBA table and initialize it with all
 *			  LEBs unmapped
 * @vol: volume containing the EBA table to copy
 * @nentries: number of entries in the table
 *
 * Allocate a new EBA table and initialize it with all LEBs unmapped.
 * Returns a valid pointer if it succeed, an ERR_PTR() otherwise.
 */
struct ubi_eba_table *ubi_eba_create_table(struct ubi_volume *vol,
					   int nentries)
{
	if (!vol->eba_tbl_ops) {
		/*
		 * This is the first time ubi_eba_create_table() is called
		 * on this volume, and vol->eba_tbl_ops has not been
		 * initialized yet.
		 */
		switch(vol->vol_mode) {
		case UBI_VOL_MODE_NORMAL:
		case UBI_VOL_MODE_SLC:
			vol->eba_tbl_ops = &ubi_eba_sleb_ops;
			break;
		case UBI_VOL_MODE_MLC_SAFE:
			vol->eba_tbl_ops = &ubi_eba_mleb_ops;
			break;
		default:
			return ERR_PTR(-EINVAL);
		}
	}

	return vol->eba_tbl_ops->create(vol, nentries);
}

/**
 * print_rsvd_warning - warn about not having enough reserved PEBs.
 * @ubi: UBI device description object
 *
 * This is a helper function for 'ubi_eba_init()' which is called when UBI
 * cannot reserve enough PEBs for bad block handling. This function makes a
 * decision whether we have to print a warning or not. The algorithm is as
 * follows:
 *   o if this is a new UBI image, then just print the warning
 *   o if this is an UBI image which has already been used for some time, print
 *     a warning only if we can reserve less than 10% of the expected amount of
 *     the reserved PEB.
 *
 * The idea is that when UBI is used, PEBs become bad, and the reserved pool
 * of PEBs becomes smaller, which is normal and we do not want to scare users
 * with a warning every time they attach the MTD device. This was an issue
 * reported by real users.
 */
static void print_rsvd_warning(struct ubi_device *ubi,
			       struct ubi_attach_info *ai)
{
	/*
	 * The 1 << 18 (256KiB) number is picked randomly, just a reasonably
	 * large number to distinguish between newly flashed and used images.
	 */
	if (ai->max_sqnum > (1 << 18)) {
		int min = ubi->beb_rsvd_level / 10;

		if (!min)
			min = 1;
		if (ubi->beb_rsvd_pebs > min)
			return;
	}

	ubi_warn(ubi, "cannot reserve enough PEBs for bad PEB handling, reserved %d, need %d",
		 ubi->beb_rsvd_pebs, ubi->beb_rsvd_level);
	if (ubi->corr_peb_count)
		ubi_warn(ubi, "%d PEBs are corrupted and not used",
			 ubi->corr_peb_count);
}

/**
 * self_check_eba - run a self check on the EBA table constructed by fastmap.
 * @ubi: UBI device description object
 * @ai_fastmap: UBI attach info object created by fastmap
 * @ai_scan: UBI attach info object created by scanning
 *
 * Returns < 0 in case of an internal error, 0 otherwise.
 * If a bad EBA table entry was found it will be printed out and
 * ubi_assert() triggers.
 */
int self_check_eba(struct ubi_device *ubi, struct ubi_attach_info *ai_fastmap,
		   struct ubi_attach_info *ai_scan)
{
	int i, j, num_volumes, ret = 0;
	int **scan_eba, **fm_eba;
	struct ubi_ainf_volume *av;
	struct ubi_volume *vol;
	struct ubi_ainf_leb *aleb;
	struct rb_node *rb;

	num_volumes = ubi->vtbl_slots + UBI_INT_VOL_COUNT;

	scan_eba = kmalloc(sizeof(*scan_eba) * num_volumes, GFP_KERNEL);
	if (!scan_eba)
		return -ENOMEM;

	fm_eba = kmalloc(sizeof(*fm_eba) * num_volumes, GFP_KERNEL);
	if (!fm_eba) {
		kfree(scan_eba);
		return -ENOMEM;
	}

	for (i = 0; i < num_volumes; i++) {
		vol = ubi->volumes[i];
		if (!vol)
			continue;

		scan_eba[i] = kmalloc(vol->reserved_lebs * sizeof(**scan_eba),
				      GFP_KERNEL);
		if (!scan_eba[i]) {
			ret = -ENOMEM;
			goto out_free;
		}

		fm_eba[i] = kmalloc(vol->reserved_lebs * sizeof(**fm_eba),
				    GFP_KERNEL);
		if (!fm_eba[i]) {
			ret = -ENOMEM;
			goto out_free;
		}

		for (j = 0; j < vol->reserved_lebs; j++)
			scan_eba[i][j] = fm_eba[i][j] = UBI_LEB_UNMAPPED;

		av = ubi_find_av(ai_scan, idx2vol_id(ubi, i));
		if (!av)
			continue;

		ubi_rb_for_each_entry(rb, aleb, &av->root, node)
			scan_eba[i][aleb->lnum] = ubi_ainf_get_pnum(aleb->peb);

		av = ubi_find_av(ai_fastmap, idx2vol_id(ubi, i));
		if (!av)
			continue;

		ubi_rb_for_each_entry(rb, aleb, &av->root, node)
			fm_eba[i][aleb->lnum] = ubi_ainf_get_pnum(aleb->peb);

		for (j = 0; j < vol->reserved_lebs; j++) {
			if (scan_eba[i][j] != fm_eba[i][j]) {
				if (scan_eba[i][j] == UBI_LEB_UNMAPPED ||
					fm_eba[i][j] == UBI_LEB_UNMAPPED)
					continue;

				ubi_err(ubi, "LEB:%i:%i is PEB:%i instead of %i!",
					vol->vol_id, j, fm_eba[i][j],
					scan_eba[i][j]);
				ubi_assert(0);
			}
		}
	}

out_free:
	for (i = 0; i < num_volumes; i++) {
		if (!ubi->volumes[i])
			continue;

		kfree(scan_eba[i]);
		kfree(fm_eba[i]);
	}

	kfree(scan_eba);
	kfree(fm_eba);
	return ret;
}

/**
 * ubi_eba_init - initialize the EBA sub-system using attaching information.
 * @ubi: UBI device description object
 * @ai: attaching information
 *
 * This function returns zero in case of success and a negative error code in
 * case of failure.
 */
int ubi_eba_init(struct ubi_device *ubi, struct ubi_attach_info *ai)
{
	int i, err, num_volumes;
	struct ubi_ainf_volume *av;
	struct ubi_volume *vol;
	struct ubi_ainf_leb *aleb;
	struct rb_node *rb;

	dbg_eba("initialize EBA sub-system");

	mutex_init(&ubi->alc_mutex);

	atomic64_set(&ubi->global_sqnum, ai->max_sqnum + 1);
	num_volumes = ubi->vtbl_slots + UBI_INT_VOL_COUNT;

	for (i = 0; i < num_volumes; i++) {
		struct ubi_eba_table *tbl;

		vol = ubi->volumes[i];
		if (!vol)
			continue;

		cond_resched();

		vol->direct_writes = true;
		mutex_init(&vol->ltree_lock);
		vol->ltree = RB_ROOT;

		tbl = ubi_eba_create_table(vol, vol->reserved_lebs);
		if (IS_ERR(tbl)) {
			err = PTR_ERR(tbl);
			goto out_free;
		}

		ubi_eba_replace_table(vol, tbl);

		err = ubi_conso_init(vol);
		if (err)
			goto out_free;

		av = ubi_find_av(ai, idx2vol_id(ubi, i));
		if (!av)
			continue;

		ubi_rb_for_each_entry(rb, aleb, &av->root, node) {
			if (aleb->lnum >= vol->reserved_lebs) {
				/*
				 * This may happen in case of an unclean reboot
				 * during re-size.
				 */
				ubi_remove_aleb(av, aleb, &ai->erase);
			} else {
				err = ubi_eba_init_entry(vol, aleb);
				if (err)
					goto out_free;
			}
		}
	}

	if (ubi->avail_pebs < EBA_RESERVED_PEBS) {
		ubi_err(ubi, "no enough physical eraseblocks (%d, need %d)",
			ubi->avail_pebs, EBA_RESERVED_PEBS);
		if (ubi->corr_peb_count)
			ubi_err(ubi, "%d PEBs are corrupted and not used",
				ubi->corr_peb_count);
		err = -ENOSPC;
		goto out_free;
	}
	ubi->avail_pebs -= EBA_RESERVED_PEBS;
	ubi->rsvd_pebs += EBA_RESERVED_PEBS;

	if (ubi->bad_allowed) {
		ubi_calculate_reserved(ubi);

		if (ubi->avail_pebs < ubi->beb_rsvd_level) {
			/* No enough free physical eraseblocks */
			ubi->beb_rsvd_pebs = ubi->avail_pebs;
			print_rsvd_warning(ubi, ai);
		} else
			ubi->beb_rsvd_pebs = ubi->beb_rsvd_level;

		ubi->avail_pebs -= ubi->beb_rsvd_pebs;
		ubi->rsvd_pebs  += ubi->beb_rsvd_pebs;
	}

	dbg_eba("EBA sub-system is initialized");
	return 0;

out_free:
	for (i = 0; i < num_volumes; i++) {
		if (!ubi->volumes[i])
			continue;
		ubi_conso_cleanup(vol);
		ubi_eba_replace_table(ubi->volumes[i], NULL);
	}
	return err;
}
