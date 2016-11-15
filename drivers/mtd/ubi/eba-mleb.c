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
#include <linux/wait.h>
#include "eba.h"

/**
 * struct ubi_eba_mleb_entry - structure encoding a LEB -> PEB for MLC_SAFE
 *			       volumes.
 * @node: used to classify the LEB in one of the category
 * @consolidated: set to 1 if the LEB has been consolidated
 * @invalid: set to 1 if the LEB is invalid but part of PEB where one or more
 *	     LEBs are still valid
 * @pnum: the physical eraseblock number attached to the LEB
 * @cpeb: PEB information when the LEB has been consolidated
 *
 * This structure is encoding a LEB -> PEB association.
 */
struct ubi_eba_mleb_entry {
	struct list_head node;
	unsigned int consolidated:1;
	unsigned int invalid:1;
	unsigned int nzombies:30;
	union {
		int pnum;
		struct ubi_consolidated_peb *cpeb;
	};
};

/**
 *
 */
struct ubi_eba_mleb_gc_entry {
	struct list_head node;
	struct ubi_peb_desc *pdesc;
};

/**
 * struct ubi_eba_mleb_list - LEB classification list object.
 * @list: internal list_head object
 * @count: object counter
 */
struct ubi_eba_mleb_list {
	struct list_head list;
	int count;
};

/**
 * struct ubi_eba_mleb_table - LEB -> PEB association information for MLC_SAFE
 *			       volumes.
 * @base: base EBA table
 * @entries: the LEB to PEB mapping (one entry per LEB)
 * @consolidated: bitmap encoding whether a LEB has been consolidated or not
 * @open: list containing all the open LEBs (those that are not consolidated)
 * @closed.clean: list containing all LEBs that have been consolidated and
 *		  are part of a PEB that contains only valid LEBs
 * @closed.dirty: lists containing LEBs that are consolidated and part of PEBs
 *		  that are partially dirty. The list index is used to encode
 *		  the number of valid LEBs in a PEB (first dirty list contains
 *		  only LEBs that are the last valid one in the consolidated
 *		  PEB, 2nd dirty list contains LEBs that are sharing a PEB with
 *		  another valid LEB, ...)
 * @free_pebs: number of free PEBs. This is used to track the number of trigger
 *	       consolidation when we are running out of free PEBs.
 *	       This number is only representing the number a free PEBs from the
 *	       volume point of view. Which means, the UBI device might be out of
 *	       free PEBs and may have to flush the erase work queue to produce
 *	       some
 * @conso: consolidation work
 * @lock: lock protecting the EBA table resources
 *
 * This structure is private to the EBA logic and should be kept here.
 * It is encoding the LEB to PEB association table, and is subject to
 * changes.
 */
struct ubi_eba_mleb_table {
	struct ubi_eba_table base;
	struct ubi_eba_mleb_entry *entries;
	struct ubi_eba_mleb_list open;
	struct {
		struct ubi_eba_mleb_list clean;
		struct ubi_eba_mleb_list *dirty;
	} closed;
	struct ubi_eba_mleb_list corrupted;
	struct ubi_eba_mleb_list gc;
	int nzombies;
	int free_pebs;
	int consolidable_lebs;
	int conso_high_wm;
	int conso_low_wm;
	wait_queue_head_t wait_peb;
	struct mutex lock;
};

static struct ubi_eba_mleb_table *to_mleb_table(struct ubi_eba_table *tbl)
{
	return container_of(tbl, struct ubi_eba_mleb_table, base);
}

static void mleb_list_add(struct ubi_eba_mleb_list *list,
			  struct ubi_eba_mleb_entry *entry)
{
	ubi_assert(list_empty(&entry->node));

	list_add(&entry->node, &list->list);
	list->count++;
}

static void mleb_list_add_tail(struct ubi_eba_mleb_list *list,
			       struct ubi_eba_mleb_entry *entry)
{
	ubi_assert(list_empty(&entry->node));

	list_add_tail(&entry->node, &list->list);
	list->count++;
}

static void mleb_list_del(struct ubi_eba_mleb_list *list,
			  struct ubi_eba_mleb_entry *entry)
{
	ubi_assert(!list_empty(&entry->node));

	list_del_init(&entry->node);
	list->count--;
}

static int mleb_list_count(const struct ubi_eba_mleb_list *list)
{
	return list->count;
}

static void mleb_list_init(struct ubi_eba_mleb_list *list)
{
	INIT_LIST_HEAD(&list->list);
	list->count = 0;
}

static void mleb_init_entry(struct ubi_eba_mleb_entry *entry)
{
	INIT_LIST_HEAD(&entry->node);
	entry->pnum = UBI_LEB_UNMAPPED;
	entry->consolidated = false;
	entry->invalid = true;
	entry->nzombies = 0;
}

#define mleb_list_for_each_entry(e, l)				\
	list_for_each_entry(e, &(l)->list, node)

#define mleb_list_for_each_entry_reverse(e, l)			\
	list_for_each_entry_reverse(e, &(l)->list, node)

static inline int mleb_entry_to_lnum(struct ubi_eba_mleb_table *tbl,
				     struct ubi_eba_mleb_entry *entry)
{
	unsigned long idx = (unsigned long)entry - (unsigned long)tbl->entries;

	idx /= sizeof(*entry);

	return idx;
}

static void dump_mleb_stats(struct ubi_volume *vol)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	int rsvd_pebs = vol->reserved_pebs;

	if (rsvd_pebs > 17)
		rsvd_pebs--;

	ubi_assert(tbl->consolidable_lebs == mleb_list_count(&tbl->closed.dirty[0]) + mleb_list_count(&tbl->open));
	ubi_assert(tbl->consolidable_lebs >= 0 && vol->reserved_pebs >= 0 &&
		   mleb_list_count(&tbl->open) >= 0 && mleb_list_count(&tbl->closed.dirty[0]) >= 0 &&
		   mleb_list_count(&tbl->closed.clean) >= 0);
	ubi_assert(rsvd_pebs >= mleb_list_count(&tbl->closed.dirty[0]) + mleb_list_count(&tbl->closed.clean) + mleb_list_count(&tbl->open) + tbl->free_pebs);
}

static enum ubi_eba_conso_req ubi_eba_mleb_conso_req(struct ubi_volume *vol)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	int rsvd_pebs = vol->reserved_pebs;
	enum ubi_eba_conso_req ret = UBI_CONSO_IMPOSSIBLE;

	mutex_lock(&tbl->lock);
	/*
	 * 16 PEBs is the limit we have to start consolidating LEBs. If we
	 * are below this limit, all LEBs are stored in SLC mode, and
	 * consolidation is disabled.
	 */
	if (rsvd_pebs <= 16) {
		dump_mleb_stats(vol);
		goto out;
	}

	/*
	 * If we have less than max_lebs_per_peb consolidable LEBs we can't
	 * consolidate.
	 */
	if (tbl->consolidable_lebs < vol->ubi->max_lebs_per_peb) {
		dump_mleb_stats(vol);
		goto out;
	}

	/* Some users are waiting for free PEBs. */
	if (waitqueue_active(&tbl->wait_peb)) {
		ret = UBI_CONSO_REQUIRED;
		goto out;
	}

	/*
	 * Recommend consolidation when the number of free PEBs is inside
	 * the low and high watermark range.
	 */
	if (tbl->free_pebs <= tbl->conso_high_wm &&
	    tbl->free_pebs >= tbl->conso_low_wm)
		ret = UBI_CONSO_RECOMMENDED;
	else
		ret = UBI_CONSO_POSSIBLE;

out:
	/*
	 * Make sure we never run into a case where consolidation is
	 * required but impossible.
	 */
	if (ret == UBI_CONSO_IMPOSSIBLE)
		ubi_assert(!waitqueue_active(&tbl->wait_peb));

	mutex_unlock(&tbl->lock);

	return ret;
}

static int ubi_eba_mleb_reserve_peb(struct ubi_volume *vol)
{
	struct ubi_eba_mleb_table *tbl;

	tbl = to_mleb_table(vol->eba_tbl);

	mutex_lock(&tbl->lock);
	while (tbl->free_pebs < 1) {
		dump_mleb_stats(vol);
		mutex_unlock(&tbl->lock);
		ubi_conso_schedule(vol);
		wait_event(tbl->wait_peb, tbl->free_pebs > 0);
		mutex_lock(&tbl->lock);
	}
	tbl->free_pebs--;
	mutex_unlock(&tbl->lock);

	/* Rearm consolidation just in case. */
	ubi_conso_rearm(vol);

	return 0;
}

static void ubi_eba_mleb_release_peb(struct ubi_volume *vol)
{
	struct ubi_eba_mleb_table *tbl;

	tbl = to_mleb_table(vol->eba_tbl);

	mutex_lock(&tbl->lock);
	tbl->free_pebs++;
	wake_up(&tbl->wait_peb);
	mutex_unlock(&tbl->lock);
}


static void ubi_eba_mleb_get_ldesc(struct ubi_volume *vol, int lnum,
				  struct ubi_eba_leb_desc *ldesc)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_eba_mleb_entry *entry = &tbl->entries[lnum];

	ldesc->lnum = lnum;
	if (entry->invalid) {
		ldesc->pnum = UBI_LEB_UNMAPPED;
		ldesc->lpos = UBI_EBA_NA;
	} else if (entry->consolidated) {
		struct ubi_consolidated_peb *cpeb = entry->cpeb;
		int i;

		ldesc->pnum = cpeb->pnum;
		for (i = 0; i < vol->ubi->max_lebs_per_peb; i++) {
			if (cpeb->lnums[i] == lnum) {
				ldesc->lpos = i;
				break;
			}
		}

		ubi_assert(i < vol->ubi->max_lebs_per_peb);
	} else {
		ldesc->pnum = entry->pnum;
		ldesc->lpos = UBI_EBA_NA;
	}
}

static int cpeb_refcount(struct ubi_volume *vol,
			 struct ubi_consolidated_peb *cpeb)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_device *ubi = vol->ubi;
	int i, refcount = 0;

	for (i = 0; i < ubi->max_lebs_per_peb; i++) {
		if (cpeb->lnums[i] >= vol->reserved_lebs ||
		    !tbl->entries[cpeb->lnums[i]].consolidated ||
		    tbl->entries[cpeb->lnums[i]].cpeb != cpeb ||
		    tbl->entries[cpeb->lnums[i]].invalid)
			continue;

		refcount++;
	}

	return refcount;
}

static int cpeb_nzombies(struct ubi_volume *vol,
			 struct ubi_consolidated_peb *cpeb)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_device *ubi = vol->ubi;
	int i, nzombies = 0;

	for (i = 0; i < ubi->max_lebs_per_peb; i++) {
		if (cpeb->lnums[i] >= vol->reserved_lebs ||
		    !tbl->entries[cpeb->lnums[i]].consolidated ||
		    tbl->entries[cpeb->lnums[i]].cpeb != cpeb)
			continue;

		nzombies += tbl->entries[cpeb->lnums[i]].nzombies;
	}

	return nzombies;
}

static struct ubi_peb_desc *entry_to_pdesc(struct ubi_volume *vol,
					   struct ubi_eba_mleb_entry *entry)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_device *ubi = vol->ubi;
	struct ubi_peb_desc *pdesc;
	int i;

	pdesc = ubi_alloc_pdesc(ubi, GFP_NOFS);
	if (!pdesc)
		return NULL;

	pdesc->vol_id = vol->vol_id;

	if (entry->consolidated) {
		struct ubi_consolidated_peb *cpeb = entry->cpeb;

		ubi_assert(cpeb);

		pdesc->pnum = cpeb->pnum;
		for (i = 0; i < ubi->max_lebs_per_peb; i++)
			pdesc->lnums[i] = cpeb->lnums[i];
	} else {
		ubi_assert(entry->pnum >= 0);
		pdesc->pnum = entry->pnum;
		pdesc->lnums[0] = mleb_entry_to_lnum(tbl, entry);
	}

	return pdesc;
}

static void cond_add_to_gc(struct ubi_volume *vol,
			   struct ubi_eba_mleb_entry *entry)
{
	struct ubi_device *ubi = vol->ubi;
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_consolidated_peb *cpeb = entry->cpeb;
	int i;

	if (!entry->invalid || entry->nzombies)
		return;

	if (entry->consolidated) {
		struct ubi_eba_mleb_entry *tmp;
		int lnum = mleb_entry_to_lnum(tbl, entry);

		for (i = 0; i < ubi->max_lebs_per_peb; i++) {
			if (cpeb->lnums[i] == lnum)
				continue;

			tmp = &tbl->entries[cpeb->lnums[i]];

			/* We still have a valid reference to this PEB. */
			if (!tmp->invalid && tmp->consolidated &&
			    tmp->cpeb == cpeb)
				return;
		}
	}

	pr_info("%s:%i add LEB %d:%d (PEB %d) to GC\n", __func__, __LINE__,
		vol->vol_id, mleb_entry_to_lnum(tbl, entry),
		entry->consolidated ? cpeb->pnum : entry->pnum);
	mleb_list_add_tail(&tbl->gc, entry);
}

struct ubi_peb_desc *remove_from_gc(struct ubi_volume *vol,
				    struct ubi_eba_mleb_entry *entry)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_peb_desc *pdesc;

	ubi_assert(entry->invalid);

	if (list_empty(&entry->node))
		return NULL;

	pdesc = entry_to_pdesc(vol, entry);
	if (!pdesc)
		return ERR_PTR(-ENOMEM);

	mleb_list_del(&tbl->gc, entry);

	if (entry->consolidated) {
		kfree(entry->cpeb);
		entry->consolidated = false;
	}

	entry->pnum = UBI_LEB_UNMAPPED;
	pr_info("%s:%i\n", __func__, __LINE__);

	return pdesc;
}

static struct ubi_peb_desc *
ubi_eba_invalidate_cpeb_entry(struct ubi_volume *vol,
			      struct ubi_eba_mleb_entry *entry,
			      bool upd)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_device *ubi = vol->ubi;
	struct ubi_consolidated_peb *cpeb = entry->cpeb;
	struct ubi_eba_mleb_entry *tmp_entry = NULL;
	struct ubi_peb_desc *pdesc = NULL;
	struct ubi_eba_mleb_list *src, *dirty = NULL;
	int i, lnum, refcount = 0, nzombies = 0;
	bool found = false;

	lnum = mleb_entry_to_lnum(tbl, entry);
	ubi_assert(lnum >= 0);

	for (i = 0; i < ubi->max_lebs_per_peb; i++) {
		if (cpeb->lnums[i] >= vol->reserved_lebs ||
		    !tbl->entries[cpeb->lnums[i]].consolidated ||
		    tbl->entries[cpeb->lnums[i]].cpeb != cpeb)
			continue;

		/*
		 * Do not count the zombies behind the LEB we are invalidating
		 * if the invalidation is just a transitional state followed by
		 * an update.
		 */
		if (!upd || cpeb->lnums[i] != lnum)
			nzombies += tbl->entries[cpeb->lnums[i]].nzombies;

		if (tbl->entries[cpeb->lnums[i]].invalid)
			continue;

		if (cpeb->lnums[i] == lnum)
			found = true;

		/*
		 * Remove the first valid LEB from it's classification list
		 * (the other entries of a consolidated PEBs are not
		 * classified).
		 */
		if (!tmp_entry)
			tmp_entry = &tbl->entries[cpeb->lnums[i]];

		refcount++;
	}

	ubi_assert(tmp_entry);
	ubi_assert(found);
	ubi_assert(refcount);

	if (refcount == ubi->max_lebs_per_peb) {
		src = &tbl->closed.clean;
		tbl->consolidable_lebs += ubi->max_lebs_per_peb - 1;
	} else {
		src = &tbl->closed.dirty[refcount - 1];
		tbl->consolidable_lebs--;
	}

	mleb_list_del(src, tmp_entry);

	/*
	 * We are the last user and there's no zombie LEBs hidden by this PEB,
	 * free it.
	 */
	if (refcount == 1 && !nzombies) {
		pdesc = entry_to_pdesc(vol, entry);
		if (!pdesc)
			return ERR_PTR(-ENOMEM);

		for (i = 0; i < ubi->max_lebs_per_peb; i++) {
			tmp_entry = &tbl->entries[cpeb->lnums[i]];

			if (cpeb->lnums[i] >= vol->reserved_lebs ||
			    (tmp_entry->consolidated && tmp_entry->cpeb == cpeb))
				continue;

			ubi_assert(tmp_entry->nzombies > 0);
			tmp_entry->nzombies--;
			tbl->nzombies--;

			cond_add_to_gc(vol, tmp_entry);
		}
	}

	/*
	 * We have several dirty lists. The dirty list is selected based on the
	 * number of valid LEBs present in the consolidated PEB. This allows
	 * better selection of consolidable LEBs (for example, on TLC NANDs you
	 * might prefer to first pick LEBs that are alone in their PEB to
	 * produce more free PEBs, or combine LEBs from 2 different dirty lists
	 * to always produce at least 2 free PEBs at each consolidation step.
	 */
	if (refcount > 1)
		dirty = &tbl->closed.dirty[refcount - 2];

	/* Re-insert the first valid LEB in the appropriate dirty list. */
	for (i = 0; dirty && i < ubi->max_lebs_per_peb; i++) {
		if (cpeb->lnums[i] >= vol->reserved_lebs ||
		    !tbl->entries[cpeb->lnums[i]].consolidated ||
		    tbl->entries[cpeb->lnums[i]].cpeb != cpeb ||
		    cpeb->lnums[i] == lnum)
			continue;

		if (tbl->entries[cpeb->lnums[i]].cpeb == cpeb) {
			tmp_entry = &tbl->entries[cpeb->lnums[i]];
			mleb_list_add(dirty, tmp_entry);
			break;
		}
	}

	if (pdesc) {
		ubi_assert(cpeb_refcount(vol, cpeb) == 1 &&
			   entry->nzombies == cpeb_nzombies(vol, cpeb));
		for (i = 0; i < ubi->max_lebs_per_peb; i++) {
			if (cpeb->lnums[i] >= vol->reserved_lebs ||
			    cpeb->lnums[i] == lnum ||
			    !tbl->entries[cpeb->lnums[i]].consolidated ||
			    tbl->entries[cpeb->lnums[i]].cpeb != cpeb)
				continue;

			ubi_assert(tbl->entries[cpeb->lnums[i]].invalid);
			tbl->entries[cpeb->lnums[i]].consolidated = false;
			tbl->entries[cpeb->lnums[i]].pnum = UBI_LEB_UNMAPPED;
		}

		kfree(cpeb);
	} else {
		ubi_assert(cpeb_refcount(vol, cpeb) ||
			   cpeb_nzombies(vol, cpeb));
	}

	return pdesc;
}

static struct ubi_peb_desc *invalidate_entry_unlocked(struct ubi_volume *vol,
						      int lnum, bool upd)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_eba_mleb_entry *entry;
	struct ubi_peb_desc *pdesc = NULL;

	entry = &tbl->entries[lnum];

	if (entry->invalid) {
		pdesc = remove_from_gc(vol, entry);
		if (IS_ERR(pdesc))
			return pdesc;

		if (upd) {
			if (entry->consolidated) {
				int refcount, nzombies;
				struct ubi_consolidated_peb *cpeb;

				ubi_assert(!pdesc);
				cpeb = entry->cpeb;
				refcount = cpeb_refcount(vol, cpeb);
				nzombies = cpeb_nzombies(vol, cpeb);

				ubi_assert(refcount || nzombies);
				if (refcount || nzombies > entry->nzombies) {
					/*
					 * We are about to hide an invalid LEB,
					 * increase the zombie counter.
					 */
					entry->nzombies++;
					tbl->nzombies++;
				} else {
					/*
					 * This LEB was the only thing
					 * preventing the consolidated PEB
					 * erasure, release it.
					 */
					pdesc = entry_to_pdesc(vol, entry);
					if (!pdesc)
						return ERR_PTR(-ENOMEM);

				}
			} else if (entry->pnum >= 0) {
				/*
				 * The PEB was retained to hide an invalid
				 * LEB. We're about to update this LEB,
				 * free the previous PEB.
				 */
				ubi_assert(entry->nzombies);
				ubi_assert(!pdesc);
				pdesc = entry_to_pdesc(vol, entry);
				if (!pdesc)
					return ERR_PTR(-ENOMEM);
			}
		}
	} else if (entry->consolidated) {
		pdesc = ubi_eba_invalidate_cpeb_entry(vol, entry, upd);
		if (IS_ERR(pdesc))
			return pdesc;

		if (upd && !pdesc) {
			/*
			 * The PEB was not released, this means we have a
			 * zombie LEB in the wild.
			 */
			entry->nzombies++;
			tbl->nzombies++;
		}
	} else {
		/* The LEB is not consolidated. */
		ubi_assert(entry->pnum >= 0);

		/*
		 * Make sure we are not hiding zombies before releasing the
		 * PEB.
		 */
		if (!entry->nzombies || upd) {
			pdesc = entry_to_pdesc(vol, entry);
			if (!pdesc)
				return ERR_PTR(-ENOMEM);
		}

		tbl->consolidable_lebs--;
		mleb_list_del(&tbl->open, entry);
	}

	entry->invalid = true;

	if (pdesc) {
		ubi_assert(!entry->nzombies || upd);
		entry->consolidated = false;
		entry->pnum = UBI_LEB_UNMAPPED;
		tbl->free_pebs++;
		wake_up(&tbl->wait_peb);
	}

	return pdesc;
}

static struct ubi_peb_desc *
ubi_eba_mleb_invalidate_entry(struct ubi_volume *vol, int lnum)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_peb_desc *pdesc;

	ubi_conso_cond_cancel(vol, lnum);

	mutex_lock(&tbl->lock);
	pdesc = invalidate_entry_unlocked(vol, lnum, false);
	mutex_unlock(&tbl->lock);

	return pdesc;
}

static int ubi_eba_mleb_gc_pebs(struct ubi_volume *vol)
{
	struct ubi_device *ubi = vol->ubi;
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_eba_mleb_entry *entry;
	struct ubi_peb_desc *pdesc;
	int err = 0;

	mutex_lock(&tbl->lock);
	while (mleb_list_count(&tbl->gc)) {
		entry = list_first_entry(&tbl->gc.list,
					 struct ubi_eba_mleb_entry, node);
		pdesc = remove_from_gc(vol, entry);
		if (IS_ERR(pdesc)) {
			err = PTR_ERR(pdesc);
			break;
		}

		tbl->free_pebs++;

		/*
		 * Release EBA table lock when calling ubi_wl_put_peb() to
		 * avoid contention.
		 */
		mutex_unlock(&tbl->lock);
		err = ubi_wl_put_peb(ubi, pdesc, 0);
		if (err)
			return err;

		mutex_lock(&tbl->lock);
	}

	if (tbl->free_pebs > 0)
		wake_up(&tbl->wait_peb);

	dump_mleb_stats(vol);

	mutex_unlock(&tbl->lock);

	return err;
}

static struct ubi_peb_desc *ubi_eba_mleb_update_entry(struct ubi_volume *vol,
					const struct ubi_eba_leb_desc *ldesc)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_eba_mleb_entry *entry;
	struct ubi_peb_desc *pdesc;

	ubi_assert(ldesc->lpos == UBI_EBA_NA);

	entry = &tbl->entries[ldesc->lnum];

	ubi_conso_cond_cancel(vol, ldesc->lnum);

	mutex_lock(&tbl->lock);

	pdesc = invalidate_entry_unlocked(vol, ldesc->lnum, true);
	if (IS_ERR(pdesc))
		goto out;

	tbl->consolidable_lebs++;

	entry->invalid = false;
	entry->consolidated = false;
	entry->pnum = ldesc->pnum;

	/*
	 * LEB content has just been updated, put the LEB at the beginning
	 * of the open list.
	 */
	mleb_list_add(&tbl->open, entry);

out:
	mutex_unlock(&tbl->lock);
	return pdesc;
}

static bool already_consolidated(struct ubi_volume *vol,
				 const struct ubi_consolidated_peb *cpeb,
				 int lnum)
{
	int i;

	for (i = 0; i < vol->ubi->max_lebs_per_peb; i++) {
		if (cpeb->lnums[i] == lnum)
			return true;

		if (cpeb->lnums[i] == UBI_UNKNOWN)
			break;
	}

	ubi_assert(i < vol->ubi->max_lebs_per_peb);

	return false;
}

static struct ubi_eba_mleb_entry *
first_consolidable_entry(struct ubi_volume *vol,
			 const struct ubi_consolidated_peb *cpeb,
			 struct ubi_eba_mleb_list *pool)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_eba_mleb_entry *entry;

	mleb_list_for_each_entry_reverse(entry, pool) {
		int lnum = mleb_entry_to_lnum(tbl, entry);

		/*
		 * The LEB is already locked in write mode, which means it
		 * is about to be updated. Skip it.
		 */
		if (ubi_eba_leb_read_trylock(vol, lnum))
			continue;

		/*
		 * We release the lock for now, to let UBI users cancel the
		 * consolidation if they want.
		 */
		ubi_eba_leb_read_unlock(vol, lnum);

		if (!already_consolidated(vol, cpeb, lnum))
			return entry;
	}

	return NULL;
}

static int ubi_eba_mleb_select_leb_for_conso(struct ubi_volume *vol,
				const struct ubi_consolidated_peb *cpeb,
				struct ubi_eba_leb_desc *ldesc)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_eba_mleb_entry *entry = NULL;
	int lnum;

	mutex_lock(&tbl->lock);

	dump_mleb_stats(vol);
	/*
	 * FIXME: For simplicity, we only try to consolidate dirty PEBs if
	 * they contain only one valid LEB. This should work fine for SLC
	 * NANDs, but can be a problem for TLC ones.
	 *
	 * If there's no dirty PEBs, pick the oldest open one.
	 */
	entry = first_consolidable_entry(vol, cpeb, &tbl->closed.dirty[0]);
	if (!entry)
		entry = first_consolidable_entry(vol, cpeb, &tbl->open);

	if (entry) {

		lnum = mleb_entry_to_lnum(tbl, entry);
		ubi_eba_get_ldesc(vol, lnum, ldesc);
	}

	mutex_unlock(&tbl->lock);

	return entry ? 0 : -ENOENT;
}

static void ubi_eba_mleb_update_consolidated_lebs(struct ubi_volume *vol,
					struct ubi_consolidated_peb *cpeb,
					struct ubi_peb_desc **pdescs)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_device *ubi = vol->ubi;
	int i, nhdrs = ubi->max_lebs_per_peb;

	down_read(&ubi->eba_sem);
	mutex_lock(&tbl->lock);

	/*
	 * Consolidation took one PEB. Reserve it before we potentially release
	 * PEBs in the following loop.
	 * Note that we might end-up with a negative value here, but it should
	 * be back to normal after the LEBs that have just been consolidated
	 * are invalidated.
	 */
	tbl->free_pebs--;

	for (i = 0; i < nhdrs; i++) {
		int lnum = cpeb->lnums[i];
		struct ubi_eba_mleb_entry *entry;
		struct ubi_eba_leb_desc ldesc;

		ubi_eba_get_ldesc(vol, lnum, &ldesc);

		entry = &tbl->entries[lnum];

		ubi_assert(!entry->invalid);

		/* First invalidate the previous entry. */
		pdescs[i] = invalidate_entry_unlocked(vol, lnum, true);

		/* Now update it with the new definition. */
		entry->invalid = false;
		entry->consolidated = true;
		entry->cpeb = cpeb;

		/* Only add the first LEB to the classification list. */
		if (!i)
			mleb_list_add(&tbl->closed.clean, entry);

		ubi_eba_get_ldesc(vol, lnum, &ldesc);
		ubi_assert(ldesc.lnum == lnum);
		ubi_assert(ldesc.pnum == cpeb->pnum);
		ubi_assert(ldesc.lpos == i);
	}

	dump_mleb_stats(vol);

	mutex_unlock(&tbl->lock);
	up_read(&ubi->eba_sem);
}

static void ubi_eba_mleb_destroy_table(struct ubi_eba_table *base)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(base);

	if (!tbl)
		return;

	ubi_assert(!waitqueue_active(&tbl->wait_peb));
	kfree(tbl->closed.dirty);
	kfree(tbl->entries);
	kfree(tbl);
}

static void ubi_eba_mleb_copy_table(struct ubi_volume *vol,
				   struct ubi_eba_table *dst_base,
				   int nentries)
{
	struct ubi_eba_mleb_table *src, *dst;
	int lebs_per_cpeb = vol->ubi->max_lebs_per_peb;
	struct ubi_eba_mleb_entry *entry;
	int i, lnum;

	src = to_mleb_table(vol->eba_tbl);
	dst = to_mleb_table(dst_base);

	for (i = 0; i < nentries; i++) {
		dst->entries[i].consolidated = src->entries[i].consolidated;

		if (src->entries[i].consolidated) {
			/*
			 * No need to copy the cpeb resource, only
			 * ubi_leb_unmap() should do that.
			 */
			dst->entries[i].cpeb = src->entries[i].cpeb;
		} else {
			dst->entries[i].pnum = src->entries[i].pnum;
		}
	}

	mleb_list_for_each_entry(entry, &src->open) {
		lnum = mleb_entry_to_lnum(src, entry);
		mleb_list_add_tail(&dst->open, &dst->entries[lnum]);
	}

	mleb_list_for_each_entry(entry, &src->closed.clean) {
		lnum = mleb_entry_to_lnum(src, entry);
		mleb_list_add_tail(&dst->closed.clean, &dst->entries[lnum]);
	}

	for (i = 0; i < lebs_per_cpeb; i++) {
		struct ubi_eba_mleb_list *dirty = &src->closed.dirty[i];

		mleb_list_for_each_entry(entry, dirty) {
			lnum = mleb_entry_to_lnum(src, entry);
			mleb_list_add_tail(dirty, &dst->entries[lnum]);
		}
	}

	dst->consolidable_lebs = src->consolidable_lebs;
	dst->free_pebs = src->free_pebs;
}


static int ubi_eba_mleb_init_entry(struct ubi_volume *vol,
				  struct ubi_ainf_leb *aleb)
{
	struct ubi_device *ubi = vol->ubi;
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	struct ubi_eba_mleb_entry *entry = &tbl->entries[aleb->lnum];

	if (aleb->peb->consolidated) {
		struct ubi_consolidated_peb *cpeb = aleb->peb->mleb.cpeb;
		struct ubi_eba_mleb_entry *first = NULL;
		int i, refcnt = 0;

		entry->cpeb = cpeb;
		entry->consolidated = true;
		entry->invalid = false;

		/*
		 * Count the number of entries referencing the cpeb object and
		 * keep a pointer to the first valid entry.
		 */
		for (i = 0; i < ubi->max_lebs_per_peb; i++) {
			int lnum = cpeb->lnums[i];

			/*
			 * This can happen if the volume was shrunk and some
			 * removed LEBs where sharing a PEB with valid ones.
			 */
			if (lnum >= vol->reserved_lebs)
				continue;

			if (tbl->entries[lnum].consolidated &&
			    tbl->entries[lnum].cpeb == cpeb) {
				if (!first)
					first = &tbl->entries[lnum];
				refcnt++;
			}
		}

		ubi_assert(refcnt);

		/*
		 * If the runtime refcount matches the attach time one, this
		 * means we initialized all EBA entries pointing to the
		 * consolidated PEB element.
		 * Now we can add the first entry to the appropriate
		 * closed list.
		 */
		if (refcnt == aleb->peb->mleb.refcnt) {
			struct ubi_eba_mleb_list *list;

			if (refcnt == ubi->max_lebs_per_peb) {
				list = &tbl->closed.clean;
			} else {
				list = &tbl->closed.dirty[refcnt - 1];
				tbl->consolidable_lebs += refcnt;
			}

			mleb_list_add_tail(list, first);
			tbl->free_pebs--;

			/*
			 * Steal the cpeb object so that it won't be freed when
			 * the apeb object is released.
			 */
			aleb->peb->mleb.cpeb = NULL;

			for (i = 0; i < ubi->max_lebs_per_peb; i++) {
				if (cpeb->lnums[i] >= vol->reserved_lebs ||
				    (tbl->entries[cpeb->lnums[i]].consolidated &&
				     tbl->entries[cpeb->lnums[i]].cpeb == cpeb))
					continue;

				tbl->entries[cpeb->lnums[i]].nzombies++;
				tbl->nzombies++;
			}
		}
	} else {
		entry->pnum = aleb->peb->sleb.pnum;
		entry->invalid = false;

		/*
		 * FIXME: should at least be ordered by sqnum (even if this
		 * is not exactly reflecting the Least Recently Updated
		 * logic), but fastmap does not provide this information.
		 * Let's rely on the runtime re-ordering for now: the list
		 * will be slowly re-ordered as soon as the UBI user starts
		 * updating LEBs.
		 */
		mleb_list_add_tail(&tbl->open, entry);
		tbl->consolidable_lebs++;
		tbl->free_pebs--;
	}

	dump_mleb_stats(vol);

	return 0;
}

static int ubi_eba_mleb_prepare_leb_write(struct ubi_volume *vol,
					 struct ubi_eba_leb_desc *ldesc,
					 int len)
{
	struct ubi_device *ubi = vol->ubi;
	struct ubi_vid_io_buf vidb;
	struct ubi_vid_hdr *vid_hdr;
	void *buf = ubi->peb_buf + ubi->leb_start;
	int err, lnum = ldesc->lnum, tries;
	unsigned long long sqnum = 0;
	u32 crc;

	/* The LEB is not mapped or not consolidated, return directly. */
	if (ldesc->pnum == UBI_LEB_UNMAPPED || ldesc->lpos == UBI_EBA_NA)
		return 0;

	/* We are about to unconsolidate a LEB: reserve a PEB. */
	err = ubi_eba_mleb_reserve_peb(vol);
	if (err)
		return err;

	/* First read the existing data */
	mutex_lock(&ubi->buf_mutex);
	ubi_init_vid_buf(ubi, &vidb, ubi->peb_buf + ubi->vid_hdr_offset);
	vid_hdr = ubi_get_vid_hdr(&vidb);

	/* Initialize the VID header */
	ubi_eba_vid_hdr_fill_vol_info(vol, vid_hdr);
	vid_hdr->lnum = cpu_to_be32(lnum);

	if (len) {
		err = ubi_eba_read_leb_data(vol, ldesc, buf, 0, len);
		if (err && err != UBI_IO_BITFLIPS)
			goto out_unlock_buf;

		crc = crc32(UBI_CRC32_INIT, buf, len);
		vid_hdr->data_size = cpu_to_be32(len);
		vid_hdr->copy_flag = 1;
		vid_hdr->data_crc = cpu_to_be32(crc);
	}

	vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
	sqnum = vid_hdr->sqnum;

	for (tries = 0; tries <= UBI_IO_RETRIES; tries++) {
		err = ubi_eba_try_write_vid_and_data(vol, ldesc->lnum, &vidb,
						     buf, 0, len);
		if (err != -EIO || !ubi->bad_allowed)
			break;

		/*
		 * Fortunately, this is the first write operation to this
		 * physical eraseblock, so just put it and request a new one.
		 * We assume that if this physical eraseblock went bad, the
		 * erase code will handle that.
		 */
		vid_hdr->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
		sqnum = vid_hdr->sqnum;
		ubi_msg(ubi, "try another PEB");
	}

out_unlock_buf:
	mutex_unlock(&ubi->buf_mutex);

	/* Update the LEB descriptor. */
	if (!err) {
		ubi_eba_get_ldesc(vol, lnum, ldesc);
	} else {
		ubi_eba_release_peb(vol);
	}

	return err;
}


static struct ubi_eba_table *
ubi_eba_mleb_create_table(struct ubi_volume *vol, int nentries)
{
	int lebs_per_peb = vol->ubi->max_lebs_per_peb;
	struct ubi_eba_mleb_table *tbl;
	int i;

	tbl = kzalloc(sizeof(*tbl), GFP_KERNEL);
	if (!tbl)
		return ERR_PTR(-ENOMEM);

	mutex_init(&tbl->lock);
	init_waitqueue_head(&tbl->wait_peb);

	tbl->entries = kmalloc_array(nentries, sizeof(*tbl->entries),
				     GFP_KERNEL);
	if (!tbl->entries)
		goto err;

	mleb_list_init(&tbl->gc);
	mleb_list_init(&tbl->corrupted);
	mleb_list_init(&tbl->open);
	mleb_list_init(&tbl->closed.clean);

	tbl->closed.dirty = kzalloc((lebs_per_peb - 1) *
				    sizeof(*tbl->closed.dirty),
				    GFP_KERNEL);
	if (!tbl->closed.dirty)
		goto err;

	for (i = 0; i < lebs_per_peb - 1; i++)
		mleb_list_init(&tbl->closed.dirty[i]);

	for (i = 0; i < nentries; i++)
		mleb_init_entry(&tbl->entries[i]);

	tbl->free_pebs = vol->reserved_pebs;

	/*
	 * Under 16 PEBs we do not bother consolidating things: everything
	 * is written in SLC mode. Over this limit, we need to reserve one
	 * PEB for consolidation.
	 */
	if (vol->reserved_pebs > UBI_MIN_SLC_LEBS + 1)
		tbl->free_pebs--;

	/*
	 * When we run out of PEB, consolidating ahead of time is not so smart.
	 * The consolidation will only be done on demand in this context.
	 */
	tbl->conso_low_wm = DIV_ROUND_UP(vol->slc_ratio *
					 vol->reserved_lebs,
					 100);
	if (tbl->conso_low_wm < UBI_MIN_SLC_LEBS)
		tbl->conso_low_wm  = UBI_MIN_SLC_LEBS;

	/*
	 * Do not consolidate if we have more than 30% of PEBs that are
	 * free.
	 */
	tbl->conso_high_wm = (30 * vol->reserved_pebs) / 100;
	if (tbl->conso_high_wm < UBI_MIN_SLC_LEBS)
		tbl->conso_high_wm = UBI_MIN_SLC_LEBS;

	return &tbl->base;

err:
	ubi_eba_mleb_destroy_table(&tbl->base);

	return ERR_PTR(-ENOMEM);
}



static int fill_leb_vid_and_data(struct ubi_volume *vol, int to,
				 struct ubi_vid_io_buf *svidb,
				 struct ubi_vid_io_buf *dvidb, void *buf,
				 int idx)
{
	int lnum, data_size, aldata_size, err;
	struct ubi_device *ubi = vol->ubi;
	struct ubi_vid_hdr *svidh, *dvidh;
	struct ubi_eba_leb_desc ldesc;
	u32 crc;

	svidh = ubi_get_vid_hdr(svidb) + idx;
	dvidh = ubi_get_vid_hdr(dvidb) + idx;
	lnum = be32_to_cpu(svidh->lnum);

	*dvidh = *svidh;

	lnum = be32_to_cpu(svidh->lnum);
	if (lnum < 0) {
		memset(buf, 0, vol->leb_size);
		return 0;
	}

	ubi_eba_get_ldesc(vol, lnum, &ldesc);

	/* Mark the VID entry as invalid. */
	if (ldesc.pnum != to) {
		memset(buf, 0, vol->leb_size);
		dvidh->lpos = UBI_VID_LPOS_INVALID;
	}

	if (svidh->vol_type == UBI_VID_STATIC) {
		data_size = be32_to_cpu(svidh->data_size);
		aldata_size = ALIGN(data_size, ubi->min_io_size);
	} else {
		data_size = aldata_size =
			    vol->leb_size - be32_to_cpu(svidh->data_pad);
	}

	dbg_wl("read %d bytes of data", aldata_size);
	err = ubi_eba_read_leb_data(vol, &ldesc, buf, 0, aldata_size);
	if (err && err != UBI_IO_BITFLIPS) {
		ubi_warn(ubi, "error %d while reading data from PEB %d",
			 err, ldesc.pnum);
		return MOVE_SOURCE_RD_ERR;
	}

	if (ldesc.lpos == UBI_EBA_NA) {
		/*
		 * Now we have to calculate how much data we have to copy.
		 * In case of a static volume it is fairly easy - the VID
		 * header contains the data size. In case of a dynamic volume
		 * it is more difficult - we have to read the contents, cut
		 * 0xFF bytes from the end and copy only the first part. We
		 * must do this to avoid writing 0xFF bytes as it may have some
		 * side-effects. And not only this. It is important not to
		 * include those 0xFFs to CRC because later the they may be
		 * filled by data.
		 */
		if (svidh->vol_type == UBI_VID_DYNAMIC) {
			aldata_size = data_size = ubi_calc_data_len(ubi, buf,
								    data_size);

			cond_resched();
			crc = crc32(UBI_CRC32_INIT, buf, data_size);
			cond_resched();

			/*
			 * It may turn out to be that the whole @from physical
			 * eraseblock contains only 0xFF bytes. Then we have
			 * to only write the VID header and do not write any
			 * data. This also means we should not set
			 * @vid_hdr->copy_flag, @vid_hdr->data_size, and
			 * @vid_hdr->data_crc.
			 */
			if (data_size > 0) {
				dvidh->data_size = cpu_to_be32(data_size);
				dvidh->data_crc = cpu_to_be32(crc);
			}
		}

		if (data_size > 0)
			dvidh->copy_flag = 1;
	} else {
		/*
		 * The copy flag will not be set for consolidated LEBs, because we
		 * put the real VID information on the last page, which guarantees
		 * that previous data have been correctly written, which in turns
		 * make CRC check useless.
		 */
		data_size = 0;
	}

	return 0;
}

static int lock_lebs_for_copy(struct ubi_volume *vol,
			      struct ubi_vid_io_buf *vidb, int from)
{
	struct ubi_vid_hdr *vidh = ubi_get_vid_hdr(vidb);
	int err, lnum, i, valid_hdrs = 0, nhdrs = ubi_get_nhdrs(vidb);
	struct ubi_eba_leb_desc ldesc;

	for (i = 0; i < nhdrs; i++) {
		lnum = be32_to_cpu(vidh[i].lnum);

		/* Ignore invalid LEBs. */
		if (lnum < 0)
			continue;

		/*
		 * We do not want anybody to write to this logical eraseblock
		 * while we are moving it, so lock it.
		 *
		 * Note, we are using non-waiting locking here, because we
		 * cannot sleep on the LEB, since it may cause deadlocks.
		 * Indeed, imagine a task is unmapping the LEB which is mapped
		 * to the PEB we are going to move (@from). This task locks
		 * the LEB and goes sleep in the 'ubi_wl_put_peb()' function on
		 * the @ubi->move_mutex. In turn, we are holding
		 * @ubi->move_mutex and go sleep on the LEB lock. So, if the
		 * LEB is already locked, we just do not move it and return
		 * %MOVE_RETRY. Note, we do not return %MOVE_CANCEL_RACE here
		 * because we do not know the reasons of the contention - it
		 * may be just a normal I/O on this LEB, so we want to re-try.
		 */
		err = ubi_eba_leb_write_trylock(vol, lnum);
		if (err) {
			dbg_wl("contention on LEB %d:%d, cancel", vol->vol_id,
			       lnum);
			err = MOVE_RETRY;
			goto err_unlock;
		}

		ubi_eba_get_ldesc(vol, lnum, &ldesc);
		if (ldesc.pnum != from) {
			dbg_wl("LEB %d:%d is no longer mapped to PEB %d, mapped to PEB %d, cancel",
			       vol->vol_id, lnum, from, ldesc.pnum);
		} else {
			valid_hdrs++;
		}
	}

	/*
	 * The LEBs might have been put meanwhile, and the task which put it is
	 * probably waiting on @ubi->move_mutex. No need to continue the work,
	 * cancel it.
	 */
	if (!valid_hdrs) {
		err = MOVE_CANCEL_RACE;
		goto err_unlock;
	}

	return 0;

err_unlock:
	for (i--; i >= 0; i--) {
		lnum = be32_to_cpu(vidh[i].lnum);
		if (lnum < 0)
			continue;

		ubi_eba_get_ldesc(vol, lnum, &ldesc);
		if (ldesc.pnum != from)
			continue;

		ubi_eba_leb_write_unlock(vol, lnum);
	}

	return err;
}

static void unlock_lebs_after_copy(struct ubi_volume *vol,
				   struct ubi_vid_io_buf *vidb, int from)
{
	struct ubi_vid_hdr *vidh = ubi_get_vid_hdr(vidb);
	int i, lnum, nhdrs = ubi_get_nhdrs(vidb);
	struct ubi_eba_leb_desc ldesc;

	for (i = 0; i < nhdrs; i++) {
		lnum = be32_to_cpu(vidh[i].lnum);
		if (lnum < 0)
			continue;

		ubi_eba_get_ldesc(vol, lnum, &ldesc);
		if (ldesc.pnum != from)
			continue;

		ubi_eba_leb_write_trylock(vol, lnum);
	}
}

static int ubi_eba_mleb_copy_peb(struct ubi_volume *vol, int from, int to,
				 struct ubi_vid_io_buf *svidb)
{
	struct ubi_eba_mleb_table *tbl = to_mleb_table(vol->eba_tbl);
	int err, lnum, aldata_size, nhdrs, i;
	struct ubi_vid_hdr *dvidh, *svidh = ubi_get_vid_hdr(svidb);
	struct ubi_vid_io_buf dvidb;
	struct ubi_device *ubi = vol->ubi;
	unsigned long long sqnum;
	enum ubi_io_mode io_mode;
	void *buf;

	err = lock_lebs_for_copy(vol, svidb, from);
	if (err)
		return err;

	/*
	 * OK, now the LEBs are locked and we can safely start moving the
	 * PEB content. Since this function utilizes the @ubi->peb_buf buffer
	 * which is shared with some other functions - we lock the buffer by
	 * taking the @ubi->buf_mutex.
	 */
	mutex_lock(&ubi->buf_mutex);

	/*
	 * Initialize the destination VID buffer objects. The position of this
	 * buffer depends on the type of PEB: at the end of the PEB for
	 * consolidated PEBs, and the beginning for non-consolidated ones.
	 */
	if (nhdrs > 1)
		ubi_init_vid_buf(ubi, &dvidb,
				 ubi->peb_buf + ubi->peb_size -
				 ubi->min_io_size);
	else
		ubi_init_vid_buf(ubi, &dvidb,
				 ubi->peb_buf + ubi->vid_hdr_aloffset);

	dvidh = ubi_get_vid_hdr(&dvidb);
	sqnum = ubi_next_sqnums(ubi, nhdrs);

	for (i = 0; i < nhdrs; i++) {
		buf = ubi->peb_buf + ubi->leb_start + (vol->leb_size * i);
		err = fill_leb_vid_and_data(vol, to, svidb, &dvidb, buf, i);
		if (err)
			goto out_unlock_buf;

		dvidh[i].sqnum = cpu_to_be64(sqnum++);
	}

	if (nhdrs > 1) {
		int pad_size, pad_offs;

		/*
		 * Prepare the VID headers placed at the end of the
		 * consolidated PEB.
		 */
		for (i = 0; i < nhdrs; i++)
			ubi_io_prepare_vid_hdr(ubi, &dvidh[i]);

		/* Initialize the dummy header. */
		ubi_init_vid_buf(ubi, &dvidb,
				 ubi->peb_buf + ubi->vid_hdr_aloffset);
		dvidh = ubi_get_vid_hdr(&dvidb);
		dvidh->lpos = UBI_VID_LPOS_CONSOLIDATED;

		/* Pad unused pages in the PEB with zeros. */
		pad_offs = ubi->peb_size -
			   (ubi->min_io_size * 2 * (ubi->max_lebs_per_peb - 1));
		pad_size = ubi->peb_size - pad_offs - ubi->min_io_size;
		memset(ubi->peb_buf + pad_offs, 0, pad_size);

		/* Adjust data_size. */
		aldata_size = ubi->peb_size - ubi->leb_start;

		io_mode = UBI_IO_MODE_NORMAL;
	} else {
		aldata_size = ALIGN(be32_to_cpu(dvidh->data_size),
				    ubi->min_io_size);

		io_mode = UBI_IO_MODE_SLC;
	}

	/* Write the VID header. */
	err = ubi_io_write_vid_hdr(ubi, to, &dvidb);
	if (err) {
		if (err == -EIO)
			err = MOVE_TARGET_WR_ERR;
		goto out_unlock_buf;
	}

	cond_resched();

	/* Read the VID header back and check if it was written correctly */
	err = ubi_io_read_vid_hdr(ubi, to, &dvidb, 1);
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

	if (aldata_size > 0) {
		err = ubi_io_write_data(ubi, ubi->peb_buf + ubi->leb_start, to,
					0, aldata_size,	io_mode);
		if (err) {
			if (err == -EIO)
				err = MOVE_TARGET_WR_ERR;
			goto out_unlock_buf;
		}

		cond_resched();
	}

	/* Now update the EBA table. */
	down_read(&ubi->eba_sem);
	mutex_lock(&tbl->lock);

	if (nhdrs > 1) {
		for (i = 0; i < nhdrs; i++) {
			lnum = be32_to_cpu(svidh[i].lnum);
			if (lnum < 0)
				continue;

			if (!tbl->entries[lnum].consolidated)
				continue;

			if (tbl->entries[lnum].cpeb->pnum != from)
				continue;

			/* Update the consolidated PEB entry. */
			tbl->entries[lnum].cpeb->pnum = to;
			break;
		}
		ubi_assert(i < nhdrs);
	} else {
		ubi_assert(!tbl->entries[lnum].consolidated);
		ubi_assert(tbl->entries[lnum].pnum == from);
		tbl->entries[lnum].pnum = to;
	}
	mutex_unlock(&tbl->lock);
	up_read(&ubi->eba_sem);

out_unlock_buf:
	mutex_unlock(&ubi->buf_mutex);
	unlock_lebs_after_copy(vol, svidb, from);
	return err;
}

const struct ubi_eba_table_ops ubi_eba_mleb_ops = {
	.create = ubi_eba_mleb_create_table,
	.destroy = ubi_eba_mleb_destroy_table,
	.copy = ubi_eba_mleb_copy_table,
	.reserve_peb = ubi_eba_mleb_reserve_peb,
	.release_peb = ubi_eba_mleb_release_peb,
	.get_ldesc = ubi_eba_mleb_get_ldesc,
	.invalidate_entry = ubi_eba_mleb_invalidate_entry,
	.update_entry = ubi_eba_mleb_update_entry,
	.gc_pebs = ubi_eba_mleb_gc_pebs,
	.init_entry = ubi_eba_mleb_init_entry,
	.prepare_leb_write = ubi_eba_mleb_prepare_leb_write,
	.copy_peb = ubi_eba_mleb_copy_peb,
	.select_leb_for_conso = ubi_eba_mleb_select_leb_for_conso,
	.update_consolidated_lebs = ubi_eba_mleb_update_consolidated_lebs,
	.get_conso_req = ubi_eba_mleb_conso_req,
};
