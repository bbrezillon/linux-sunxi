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

#include "ubi.h"

enum ubi_eba_conso_req {
	UBI_CONSO_IMPOSSIBLE,
	UBI_CONSO_POSSIBLE,
	UBI_CONSO_RECOMMENDED,
	UBI_CONSO_REQUIRED,
};

/**
 * struct ubi_eba_table_ops - EBA table operations
 * @create: create an EBA table
 * @destroy: destroy an EBA table
 * @copy: copy the EBA table attached to vol into a new one
 * @reserve_peb: reserve a PEB
 * @release_peb: release a PEB that has previously been reserved
 * @get_ldesc: fill a LEB descriptor with the appropriate info
 * @invalidate_entry: invalidate an EBA entry
 * @update_entry: update an EBA entry
 * @init_entry: initialize an EBA entry using attach information
 * @prepare_leb_write: prepare everything for a write access
 */
struct ubi_eba_table_ops {
	struct ubi_eba_table *(*create)(struct ubi_volume *vol, int nentries);
	void (*destroy)(struct ubi_eba_table *tbl);
	void (*copy)(struct ubi_volume *vol, struct ubi_eba_table *dst,
		     int nentries);
	int (*reserve_peb)(struct ubi_volume *vol);
	void (*release_peb)(struct ubi_volume *vol);
	void (*get_ldesc)(struct ubi_volume *vol, int lnum,
			  struct ubi_eba_leb_desc *ldesc);
	struct ubi_peb_desc *(*invalidate_entry)(struct ubi_volume *vol,
						 int lnum);
	struct ubi_peb_desc *(*update_entry)(struct ubi_volume *vol,
				     const struct ubi_eba_leb_desc *ldesc);
	int (*gc_pebs)(struct ubi_volume *vol);
	int (*init_entry)(struct ubi_volume *vol, struct ubi_ainf_leb *aleb);
	int (*prepare_leb_write)(struct ubi_volume *vol,
				 struct ubi_eba_leb_desc *ldesc, int offset);
	int (*copy_peb)(struct ubi_volume *vol, int from, int to,
			struct ubi_vid_io_buf *vidb);
	int (*select_leb_for_conso)(struct ubi_volume *vol,
				    const struct ubi_consolidated_peb *cpeb,
				    struct ubi_eba_leb_desc *ldesc);
	void (*update_consolidated_lebs)(struct ubi_volume *vol,
					 struct ubi_consolidated_peb *cpeb,
					 struct ubi_peb_desc **pdescs);
	enum ubi_eba_conso_req (*get_conso_req)(struct ubi_volume *vol);
};


/**
 * struct ubi_eba_table - dummy EBA table object
 *
 * This structure must be inherited by the real implementation.
 */
struct ubi_eba_table {
};

int ubi_eba_reserve_peb(struct ubi_volume *vol);
void ubi_eba_release_peb(struct ubi_volume *vol);
int ubi_eba_read_leb_data(struct ubi_volume *vol,
			  const struct ubi_eba_leb_desc *ldesc,
			  void *buf, int loffset, int len);
int ubi_eba_write_leb_data(struct ubi_volume *vol,
			   const struct ubi_eba_leb_desc *ldesc,
			   const void *buf, int loffset, int len);
void ubi_eba_vid_hdr_fill_vol_info(const struct ubi_volume *vol,
				   struct ubi_vid_hdr *vid_hdr);
int ubi_eba_try_write_vid_and_data(struct ubi_volume *vol, int lnum,
				   struct ubi_vid_io_buf *vidb,
				   const void *buf, int offset, int len);

int ubi_eba_select_leb_for_conso(struct ubi_volume *vol,
				 const struct ubi_consolidated_peb *cpeb,
				 struct ubi_eba_leb_desc *ldesc);
void ubi_eba_update_consolidated_lebs(struct ubi_volume *vol,
				 struct ubi_consolidated_peb *cpeb,
				 struct ubi_peb_desc **pdescs);
enum ubi_eba_conso_req ubi_eba_get_conso_req(struct ubi_volume *vol);
int ubi_eba_leb_read_trylock(struct ubi_volume *vol, int lnum);
void ubi_eba_leb_read_unlock(struct ubi_volume *vol, int lnum);
int ubi_eba_leb_write_trylock(struct ubi_volume *vol, int lnum);
void ubi_eba_leb_write_unlock(struct ubi_volume *vol, int lnum);

/**
 * is_error_sane - check whether a read error is sane.
 * @err: code of the error happened during reading
 *
 * This is a helper function for 'ubi_eba_copy_leb()' which is called when we
 * cannot read data from the target PEB (an error @err happened). If the error
 * code is sane, then we treat this error as non-fatal. Otherwise the error is
 * fatal and UBI will be switched to R/O mode later.
 *
 * The idea is that we try not to switch to R/O mode if the read error is
 * something which suggests there was a real read problem. E.g., %-EIO. Or a
 * memory allocation failed (-%ENOMEM). Otherwise, it is safer to switch to R/O
 * mode, simply because we do not know what happened at the MTD level, and we
 * cannot handle this. E.g., the underlying driver may have become crazy, and
 * it is safer to switch to R/O mode to preserve the data.
 *
 * And bear in mind, this is about reading from the target PEB, i.e. the PEB
 * which we have just written.
 */
static inline int is_error_sane(int err)
{
	if (err == -EIO || err == -ENOMEM || err == UBI_IO_BAD_HDR ||
	    err == UBI_IO_BAD_HDR_EBADMSG || err == -ETIMEDOUT)
		return 0;
	return 1;
}

/**
 * ubi_get_compat - get compatibility flags of a volume.
 * @ubi: UBI device description object
 * @vol_id: volume ID
 *
 * This function returns compatibility flags for an internal volume. User
 * volumes have no compatibility flags, so %0 is returned.
 */
static inline int ubi_get_compat(const struct ubi_device *ubi, int vol_id)
{
	if (vol_id == UBI_LAYOUT_VOLUME_ID)
		return UBI_LAYOUT_VOLUME_COMPAT;
	return 0;
}


extern const struct ubi_eba_table_ops ubi_eba_sleb_ops;
extern const struct ubi_eba_table_ops ubi_eba_mleb_ops;
