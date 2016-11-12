#include "ubi.h"

struct ubi_jnl_op {
	struct list_head node;
	enum ubi_jnl_node_type type;
};

struct ubi_jnl_erase_peb {
	const struct ubi_peb_desc *pdesc;
	int torture;
};

struct ubi_jnl_scrub_peb {
	int pnum;
};

struct ubi_jnl_mark_peb_bad {
	int pnum;
};

struct ubi_jnl_peb_op {
	struct ubi_jnl_op base;
	enum ubi_jnl_peb_op_type type;
	union {
		struct ubi_jnl_erase_peb erase;
		struct ubi_jnl_scrub_peb scrub;
		struct ubi_jnl_mark_peb_bad mark_bad;
	};
};

static inline struct ubi_jnl_peb_op *to_peb_op(struct ubi_jnl_op *base)
{
	if (base->type == UBI_JNL_PEB_NODE)
		return NULL;

	return container_of(base, struct ubi_jnl_peb_op, base);
}

struct ubi_jnl_leb_op {
	struct ubi_jnl_op base;
	enum ubi_jnl_leb_op_type type;
	int vol_id;
	struct ubi_eba_leb_desc ldesc;
};

static inline struct ubi_jnl_leb_op *to_leb_op(struct ubi_jnl_op *base)
{
	if (base->type == UBI_JNL_LEB_NODE)
		return NULL;

	return container_of(base, struct ubi_jnl_leb_op, base);
}

struct ubi_jnl_refill_peb_pool_op {
	struct ubi_jnl_op base;
	int pnum;
};

/*
struct ubi_jnl_peb_op {
	struct list_head node;
	struct ubi_jnl_peb_node def;
};

struct ubi_jnl_leb_op {
	struct list_head node;
	struct ubi_jnl_leb_node def;
};
*/

struct ubi_journal_peb_pool {
	int size;
	int avail;
	int start;
	int pnums[];
};

struct ubi_jnl_buf {
	void *buf;
	int used;
	int avail;
	struct ubi_jnl_op *op;
};

struct ubi_journal {
	int nlebs;
	int *pnums;
	int head_lnum;
	int head_offset;
	int tail_lnum;
	int tail_offset;
	struct ubi_journal_peb_pool ppool;
	struct list_head ops;
	struct mutex lock;
	int size;
	int nerase;
	struct ubi_jnl_buf jbuf;
	int max_summary_size;
	int pool_size;
};

static void init_jnl_op(struct ubi_jnl_op *op, enum ubi_jnl_node_type type)
{
	ubi_assert(type == UBI_JNL_LEB_NODE || type == UBI_JNL_PEB_NODE);

	INIT_LIST_HEAD(&op->node);
	op->type = type;
}

static struct ubi_jnl_peb_op *alloc_peb_op(u8 id)
{
	struct ubi_jnl_peb_op *op;

	ubi_assert(id == UBI_JNL_ERASE_PEB || id == UBI_JNL_SCRUB_PEB ||
		   id == UBI_JNL_MARK_PEB_BAD);

	op = kzalloc(sizeof(*op), GFP_NOFS);
	if (!op)
		return NULL;

	init_jnl_op(&op->base, UBI_JNL_PEB_NODE);
	op->type = id;

	return op;
}

static void add_peb_op(struct ubi_journal *jnl, struct ubi_jnl_op *op)
{
	ubi_assert(list_empty(op->node));

	/* TODO: control the jnl size. */
	jnl->size += sizeof(struct ubi_jnl_peb_node);
	list_add_tail(&op->node, &jnl->ops);
}

static void remove_peb_op(struct ubi_journal *jnl, struct ubi_jnl_op *op)
{
	ubi_assert(!list_empty(op->node));

	/* TODO: control the jnl size. */
	jnl->size -= sizeof(sizeof(struct ubi_jnl_peb_node));
	list_del(&op->node);
}

static struct ubi_jnl_leb_op *alloc_leb_op(u8 id)
{
	struct ubi_jnl_leb_op *op;

	ubi_assert(id == UBI_JNL_UNMAP_LEB || id == UBI_JNL_MAP_OR_UPD_LEB);

	op = kzalloc(sizeof(*op), GFP_NOFS);
	if (!op)
		return NULL;

	init_jnl_op(&op->base, UBI_JNL_LEB_NODE);
	op->type = id;

	return op;
}

static void add_leb_op(struct ubi_journal *jnl, struct ubi_jnl_op *op)
{
	ubi_assert(list_empty(op->node));

	/* TODO: control the jnl size. */
	jnl->size += sizeof(struct ubi_jnl_leb_node);
	list_add_tail(&op->node, &jnl->ops);
}

static void remove_leb_op(struct ubi_journal *jnl, struct ubi_jnl_op *op)
{
	ubi_assert(!list_empty(op->node));

	/* TODO: control the jnl size. */
	jnl->size -= sizeof(struct ubi_jnl_leb_node);
	list_del(&op->node);
}

static int get_peb_op_pnum(struct ubi_jnl_peb_op *op)
{
	switch(op->type) {
	case UBI_JNL_ERASE_PEB:
		return op->erase.pdesc->pnum;
	case UBI_JNL_SCRUB_PEB:
		return op->scrub.pnum;
	case UBI_JNL_MARK_PEB_BAD:
		return op->mark_bad.pnum;
	default:
		break;
	}

	return -EINVAL;
}

int ubi_jnl_put_peb(struct ubi_device *ubi, const struct ubi_peb_desc *pdesc,
		    int torture)
{
	struct ubi_journal *jnl = ubi->jnl;
	struct ubi_jnl_peb_op *op, *iter_peb_op;
	struct ubi_jnl_op *iter, *tmp;

	if (!ubi->jnl)
		return ubi_wl_put_peb(ubi, pdesc, torture);

	op = alloc_peb_op(UBI_JNL_ERASE_PEB);
	if (!op)
		return -ENOMEM;

	op->erase.pdesc = pdesc;
	op->erase.torture = torture;

	/* Remove all entries that are invalidated by this erase operation. */
	list_for_each_entry_safe(iter, tmp, &jnl->ops, node) {
		iter_peb_op = to_peb_op(iter);
		if (!iter_peb_op || pdesc->pnum != get_peb_op_pnum(op))
			continue;

		ubi_assert(iter_peb_op->type != UBI_JNL_MARK_PEB_BAD);

		remove_peb_op(jnl, iter);
	}

	add_peb_op(jnl, op);

	kfree(pdesc);

	return 0;
}

int ubi_jnl_scrub_peb(struct ubi_device *ubi, int pnum)
{
	struct ubi_journal *jnl = ubi->jnl;
	struct ubi_jnl_peb_op *op, *iter_peb_op;
	struct ubi_jnl_op *iter, *tmp;

	if (!ubi->jnl)
		return ubi_wl_scrub_peb(ubi, pnum);

	op = alloc_peb_op(UBI_JNL_SCRUB_PEB);
	if (!op)
		return -ENOMEM;

	op->scrub.pnum = pnum;

	/* Remove all entries that are invalidated by this erase operation. */
	list_for_each_entry_safe(iter, tmp, &jnl->ops, node) {
		iter_peb_op = to_peb_op(iter);
		if (!iter_peb_op || pnum != get_peb_op_pnum(op))
			continue;

		ubi_assert(iter_peb_op->type != UBI_JNL_MARK_PEB_BAD &&
			   iter_peb_op->type != UBI_JNL_ERASE_PEB);
		remove_peb_op(jnl, iter);
	}

	add_peb_op(jnl, op);

	return 0;
}

int ubi_jnl_mark_peb_bad(struct ubi_device *ubi, int pnum)
{
	struct ubi_journal *jnl = ubi->jnl;
	struct ubi_jnl_peb_op *op, *iter_peb_op;
	struct ubi_jnl_op *iter, *tmp;

	if (!ubi->jnl)
		return 0;

	op = alloc_peb_op(UBI_JNL_MARK_PEB_BAD);
	if (!op)
		return -ENOMEM;

	op->mark_bad.pnum = pnum;

	/* Remove all entries that are invalidated by this erase operation. */
	list_for_each_entry_safe(iter, tmp, &jnl->ops, node) {
		iter_peb_op = to_peb_op(iter);
		if (!iter_peb_op || pnum != get_peb_op_pnum(op))
			continue;

		remove_peb_op(jnl, iter);
	}

	add_peb_op(jnl, op);

	return 0;
}

int ubi_jnl_unmap_leb(struct ubi_volume *vol,
		      const struct ubi_eba_leb_desc *ldesc)
{
	struct ubi_journal *jnl = vol->ubi->jnl;
	struct ubi_jnl_leb_op *op, *iter_leb_op;
	struct ubi_jnl_op *iter, *tmp;


	if (!jnl)
		return 0;

	op = alloc_leb_op(UBI_JNL_UNMAP_LEB);
	if (!op)
		return -ENOMEM;

	op->vol_id = vol->vol_id;
	op->ldesc = *ldesc;

	/* Remove all entries that are invalidated by this operation. */
	list_for_each_entry_safe(iter, tmp, &jnl->ops, node) {
		iter_leb_op = to_leb_op(iter);
		if (!iter_leb_op ||
		    op->vol_id != iter_leb_op->vol_id ||
		    ldesc->lnum != iter_leb_op->ldesc.lnum)
			continue;

		remove_leb_op(jnl, iter);
	}

	add_leb_op(jnl, op);

	return 0;
}

int ubi_jnl_map_or_update_leb(struct ubi_volume *vol,
			      const struct ubi_eba_leb_desc *ldesc)
{
	struct ubi_journal *jnl = vol->ubi->jnl;
	struct ubi_jnl_leb_op *op, *iter_leb_op;
	struct ubi_jnl_op *iter, *tmp;

	if (!jnl)
		return 0;

	op = alloc_leb_op(UBI_JNL_UNMAP_LEB);
	if (!op)
		return -ENOMEM;

	op->vol_id = vol->vol_id;
	op->ldesc = *ldesc;

	/* Remove all entries that are invalidated by this operation. */
	list_for_each_entry_safe(iter, tmp, &jnl->ops, node) {
		iter_leb_op = to_leb_op(iter);
		if (!iter_leb_op ||
		    op->vol_id != iter_leb_op->vol_id ||
		    ldesc->lnum != iter_leb_op->ldesc.lnum)
			continue;

		remove_leb_op(jnl, iter);
	}

	add_leb_op(jnl, op);

	return 0;
}

static int fill_jbuf_with_peb_op(struct ubi_device *ubi, struct ubi_jnl_op *op)
{
	struct ubi_jnl_buf *jbuf = ubi->jnl->jbuf;
	struct ubi_jnl_peb_op *peb_op = to_peb_op(op);
	struct ubi_jnl_peb_node pnode = { };
	int pnum;

	switch(peb_op->type) {
	case UBI_JNL_ERASE_PEB:
		pnum = peb_op->erase.pdesc->pnum;
		if (peb_op->erase.torture)
			pnode.flags |= UBI_JNL_TORTURE_PEB_FLG;
		break;
	case UBI_JNL_SCRUB_PEB:
		pnum = peb_op->scrub.pnum;
		break;
	case UBI_JNL_MARK_PEB_BAD:
		pnum = peb_op->mark_bad.pnum;
		break;
	default:
		return -EINVAL;
	}

	if (jbuf->used + sizeof(*pnode) > jbuf->avail)
		return -ENOSPC;

	pnode.type = UBI_JNL_PEB_NODE;
	pnode.op = peb_op->type;
	pnode.pnum = cpu_to_be32(pnum);

	memcpy(jbuf->buf + jbuf->used, &pnode, sizeof(pnode));
	jbuf->used += sizeof(*pnode);

	return 0;
}

static int fill_jbuf_with_leb_op(struct ubi_device *ubi, struct ubi_jnl_op *op)
{
	struct ubi_jnl_buf *jbuf = ubi->jnl->jbuf;
	struct ubi_jnl_leb_op *leb_op = to_leb_op(op);
	struct ubi_jnl_leb_node lnode = { };

	if (leb_op->type != UBI_JNL_UNMAP_LEB &&
	    leb_op->type != UBI_JNL_MAP_OR_UPD_LEB)
		return -EINVAL;

	if (jbuf->used + sizeof(*lnode) > jbuf->avail)
		return -ENOSPC;

	lnode.type = UBI_JNL_LEB_NODE;
	lnode.op = leb_op->type;
	lnode.vol_id = cpu_to_be32(leb_op->vol_id);
	lnode.lnum = cpu_to_be32(leb_op->ldesc.lnum);
	lnode.lpos = leb_op->ldesc.lpos;
	lnode.pnum = cpu_to_be32(leb_op->ldesc.pnum);

	memcpy(jbuf->buf + jbuf->used, &lnode, sizeof(lnode));
	jbuf->used += sizeof(*lnode);

	return 0;
}

static int fill_jbuf_with_op(struct ubi_device *ubi, struct ubi_jnl_op *op)
{
	int ret;

	switch(op->type) {
	case UBI_JNL_PEB_NODE:
		ret = fill_jbuf_with_peb_op(ubi, op);
		break;
	case UBI_JNL_LEB_NODE:
		ret = fill_jbuf_with_leb_op(ubi, op);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int exec_peb_op(struct ubi_device *ubi, struct ubi_jnl_op *op)
{
	struct ubi_jnl_peb_op *peb_op = to_peb_op(op);

	switch (peb_op->type) {
	case UBI_JNL_ERASE_PEB:
		return ubi_wl_put_peb(ubi, peb_op->erase.pdesc,
				      peb_op->erase.torture);
	case UBI_JNL_SCRUB_PEB:
		return ubi_wl_scrub_peb(ubi, peb_op->scrub.pnum);
	case UBI_JNL_MARK_PEB_BAD:
		return ubi_wl_mark_bad(ubi, peb_op->mark_bad.pnum);
	}

	return -EINVAL;
}

static int exec_ops(struct ubi_device *ubi)
{
	struct ubi_jnl_op *op, *iter, *tmp;
	struct ubi_journal *jnl = ubi->jnl;
	int ret;

	list_for_each_entry_safe(iter, tmp, &jnl->ops, node) {
		switch (op->type) {
		case UBI_JNL_PEB_NODE:
			ret = exec_peb_op(ubi, op);
			if (ret)
				return ret;
			break;
		case UBI_JNL_LEB_NODE:
			/* Nothing to do for LEB operations. */
			break;

		default:
			return -EINVAL;
		}
	}

	return ret;
}

static int ubi_jnl_update(struct ubi_device *ubi)
{
	struct ubi_jnl_update_node *upd;
	struct ubi_jnl_move_node *move;
	struct ubi_jnl_op *op, *iter, *tmp;
	struct ubi_journal *jnl = ubi->jnl;
	struct ubi_jnl_buf *jbuf = &jnl->jbuf;
	int ret, remaining, size, leb_size;
	u32 crc;

	leb_size = ubi_io_leb_size(ubi, UBI_IO_MODE_SLC);
	jbuf->avail = leb_size - offset;
	if (jbuf->avail > ubi->max_write_size) {
		jbuf->avail = ubi->max_write_size;
	} else {
		/* Reserve space for the move node. */
		jbuf->avail -= sizeof(struct ubi_jnl_move_node);
	}

	memset(jbuf->buf, 0, ubi->max_write_size);

	upd = jbuf->buf;
	jbuf->used = sizeof(*upd);

	list_for_each_entry_safe(iter, tmp, &jnl->ops, node) {
		ret = fill_jbuf_with_op(ubi, iter);
		if (ret)
			break;
		jbuf->op = op;
	}

	/* FIXME: cleanup everything */
	if (ret != -ENOSPC)
		return ret;

	upd->type = UBI_JNL_UPDATE_NODE;
	upd->data_size = cpu_to_be32(jbuf->used - sizeof(*upd));
	crc = crc32(UBI_CRC32_INIT, jbuf->buf + sizeof(*upd),
		    jbuf->used - sizeof(*upd));
	upd->data_crc = cpu_to_be32(crc);
	crc = crc32(UBI_CRC32_INIT, upd, sizeof(*upd) - sizeof(upd->crc));
	upd->crc = cpu_to_be32(crc);

	size = ALIGN(jbuf->used, ubi->min_io_size);

	remaining = leb_size - jnl->head_offset - size;

	/*
	 * If the remaining space is not enough to store an update node +
	 * a journal move node, then it's useless to delay the journal move
	 * operation.
	 */
	if (remaining <= sizeof(*upd) + sizeof(*move)) {
		int opnum;

		move = jbuf->buf + jbuf->used;
		jbuf->used += sizeof(*move);
		memset(&move, 0, sizeof(move));
		move.type = UBI_JNL_MOVE_NODE;
		move.pnum = jnl->pnums[1];

		/* TODO: write summary node at the beginning of the new journal. */
		jnl->pnums[0] = jnl->pnums[1];
		jnl->head_lnum = (jnl->head_lnum + 1) % jnl->nlebs;
		jnl->head_offset = 0;

		/* FIXME: not sure we can call ubi_wl_put_peb() in this context. */
		/* TODO: Refill the pool? */

	}

	ret = ubi_io_write_data(ubi, jbuf->buf, jnl->pnums[0],
				jnl->head_offset, size, UBI_IO_MODE_SLC);
	if (ret)
		return ret;

	jnl->head_offset += size;

	return ret;
}

static void fill_vol_node(struct ubi_device *ubi, struct ubi_volume *vol)
{
	struct ubi_jnl_buf *jbuf = &ubi->jnl->jbuf;
	struct ubi_jnl_vol_node *vnode;
	struct ubi_jnl_eba_pnum_node *pnode;
	struct ubi_jnl_eba_lpos_node *lnode = NULL;

	int i;

	vnode = jbuf->buf + jbuf->used;
	jbuf->used += sizeof(*vnode);

	pnode = jbuf->buf + jbuf->used;
	jbuf->used += sizeof(*pnode) +
		      (vol->reserved_lebs * sizeof(__be32));
	jbuf->used = ALIGN(jbuf->used, 8);
	pnode->type = UBI_JNL_EBA_PNUM_NODE;

	if (vol->vol_mode == UBI_VOL_MODE_MLC_SAFE) {
		lnode = jbuf->buf + jbuf->used;
		jbuf->used += sizeof(*lnode);
		jbuf->used += vol->reserved_lebs * sizeof(u8);
		jbuf->used = ALIGN(jbuf->used, 8);
		lnode->type = UBI_JNL_EBA_LPOS_NODE;
	}

	for (i = 0; i < vol->reserved_lebs; i++) {
		struct ubi_eba_leb_desc ldesc;

		ubi_eba_get_ldesc(vol, i, &ldesc);

		pnode->pnums[i] = cpu_to_be32(ldesc.pnum);

		if (lnode)
			lnode->lpos[i] = ldesc->lpos;
	}

	vnode->type = UBI_JNL_VOL_NODE;
	vnode->vol_id = cpu_to_be32(vol->vol_id);
	if (vol->vol_type == UBI_STATIC_VOLUME)
		vnode->vol_type = UBI_VID_STATIC;
	else
		vnode->vol_type = UBI_VID_DYNAMIC;
	vnode->vol_mode = ubi_vol_mode_to_vid_hdr(vol);
	vnode->used_ebs = cpu_to_be32(vol->used_ebs);
	vnode->last_eb_bytes = cpu_to_be32(vol->last_eb_bytes);
	vnode->data_pad = cpu_to_be32(vol->data_pad);
}

static void fill_vol_summary_node(struct ubi_device *ubi)
{
	struct ubi_jnl_buf *jbuf = &ubi->jnl->jbuf;
	struct ubi_jnl_vol_summary_node *vnode;
	int i, nvols = 0;

	vnode = jbuf->buf + jbuf->used;
	jbuf->used += sizeof(*vnode);

	for (i = 0; i < UBI_INT_VOL_COUNT + UBI_MAX_VOLUMES; i++) {
		struct ubi_volume *vol = ubi->volumes[i];

		if (!vol)
			continue;

		fill_vol_node(ubi, vol);
		nvols++;
	}

	vnode->type = UBI_JNL_VOL_SUMMARY_NODE;
	vnode->nvols = cpu_to_be32(nvols);
}

static void fill_peb_status_summary_node(struct ubi_device *ubi)
{
	struct ubi_jnl_buf *jbuf = &ubi->jnl->jbuf;
	struct ubi_jnl_peb_status_summary_node *snode;
	struct ubi_work *ubi_wrk;
	struct ubi_wl_entry *e;
	struct rb_node *tmp_rb;
	int i;

	snode = jbuf->buf + jbuf->used;
	jbuf->used += sizeof(*snode) + (ubi->peb_count * sizeof(__u8));
	jbuf->used = ALIGN(jbuf->used, 8);

	snode->type = UBI_JNL_PEB_STATUS_SUMMARY_NODE;

	ubi_for_each_free_peb(ubi, e, tmp_rb)
		snode->status[e->pnum] = UBI_JNL_PEB_STATUS_FREE;

	ubi_for_each_used_peb(ubi, e, tmp_rb)
		snode->status[e->pnum] = UBI_JNL_PEB_STATUS_USED;

	ubi_for_each_protected_peb(ubi, i, e)
		snode->status[e->pnum] = UBI_JNL_PEB_STATUS_PROTECTED;

	ubi_for_each_scrub_peb(ubi, e, tmp_rb)
		snode->status[e->pnum] = UBI_JNL_PEB_STATUS_SCRUBING;

	list_for_each_entry(ubi_wrk, &ubi->works, list) {
		if (!ubi_is_erase_work(ubi_wrk))
			continue;

		snode->status[e->pnum] = UBI_JNL_PEB_STATUS_ERASING;
	}
}

static void fill_peb_ec_summary_node(struct ubi_device *ubi)
{
	struct ubi_jnl_buf *jbuf = &ubi->jnl->jbuf;
	struct ubi_jnl_peb_ec_summary_node *enode;
	int i;

	enode = jbuf->buf + jbuf->used;
	jbuf->used += sizeof(*enode) + (ubi->peb_count * sizeof(__be32));
	jbuf->used = ALIGN(jbuf->used, 8);

	enode->type = UBI_JNL_PEB_EC_SUMMARY_NODE;

	for (i = 0; i < ubi->peb_count; i++)
		enode->ec[i] = cpu_to_be32(ubi_wl_get_ec(ubi, i));
}

static void fill_layout_node(struct ubi_device *ubi)
{
	struct ubi_journal *jnl = ubi->jnl;
	struct ubi_jnl_buf *jbuf = &jnl->jbuf;
	struct ubi_jnl_layout_node *lnode;
	int i;

	lnode = jbuf->buf + jbuf->used;
	jbuf->used += sizeof(*lnode) + (jnl->nlebs * sizeof(__be32));
	jbuf->used = ALIGN(jbuf->used, 8);

	lnode->type = UBI_JNL_LAYOUT_NODE;
	lnode->npebs = cpu_to_be32(jnl->nlebs);

	for (i = 0; i < jnl->nlebs; i++)
		lnode->pnums[i] = cpu_to_be32(jnl->pnums[i]);
}

static int ubi_jnl_write_summary(struct ubi_device *ubi)
{
	int i, new_tail, new_head, nlebs, offs;
	int leb_size, len, ret, pad;
	struct ubi_journal *jnl = ubi->jnl;
	struct ubi_jnl_buf *jbuf = &jnl->jbuf;
	struct ubi_jnl_summary_node *snode;
	struct ubi_vid_io_buf vidb;
	struct ubi_vid_hdr *vidh;
	u32 crc;

	memset(jbuf->buf, 0, jnl->max_summary_size);

	snode = jbuf->buf;
	jbuf->used = sizeof(*snode);

	fill_layout_node(ubi);
	fill_peb_ec_summary_node(ubi);
	fill_peb_status_summary_node(ubi);
	fill_vol_summary_node(ubi);

	snode->data_size = cpu_to_be32(jbuf->used - sizeof(*snode));
	crc = crc32(UBI_CRC32_INIT, jbuf->buf + sizeof(*snode),
		    jbuf->used - sizeof(*snode));
	snode->data_crc = cpu_to_be32(crc);
	snode->npebs = cpu_to_be32(ubi->peb_count);
	snode->type = UBI_JNL_SUMMARY_NODE;

	pad = ALIGN(jbuf->used, ubi->min_io_size) - jbuf->used;
	memset(jbuf->buf + jbuf->used, 0, pad);
	jbuf->used += pad;

	leb_size = ubi_io_leb_size(ubi, UBI_IO_MODE_SLC);
	nlebs = DIV_ROUND_UP(jbuf->used, leb_size);

	/* TODO: write the summary on the flash. */
	new_tail = (jnl->head_lnum + 1) % jnl->nlebs;
	new_head = new_tail;

	for (offs = 0; offs < jbuf->used; offs += len) {
		int pnum = jnl->pnums[new_head];
		int loffset = offs % leb_size;

		len = min(ubi->max_write_size, jbuf->used - offs);
		len = min(len, leb_size - loffset);
		ret = ubi_io_write_data(ubi, jbuf->buf, pnum, loffset, len,
					UBI_IO_MODE_SLC);
		if (ret)
			return ret;

		loffset += len;
		if (loffset == leb_size)
			new_head++;
	}

	ubi_init_vid_buf(ubi, &vidb, jbuf->buf);
	vidh = ubi_get_vid_hdr(&vidb);

	vidh->vol_type = UBI_VID_DYNAMIC;
	vidh->vol_mode = UBI_VID_MODE_SLC;
	vidh->vol_id = cpu_to_be32(UBI_JNL_VOLUME_ID);
	vidh->compat = UBI_COMPAT_REJECT;

	/* Erase the old journal and refill the journal LEBs pool. */
	for (i = jnl->tail_lnum; i < new_tail; i = (i + 1) % jnl->nlebs) {
		struct ubi_peb_desc *pdesc;
		int pnum;

		/* FIXME: NOFS ??? */
		pdesc = ubi_alloc_pdesc(ubi, GFP_NOFS);
		if (!pdesc)
			return -ENOMEM;

		pdesc->pnum = jnl->pnums[new_head];
		pdesc->vol_id = UBI_JNL_VOLUME_ID;
		pdesc->lnums[0] = i;

		ret = ubi_wl_sync_erase(ubi, pdesc, 0);
		if (ret)
			return ret;

		/* Create and call a non-blocking function? */
		pnum = ubi_wl_get_peb(ubi);
		if (ret < 0)
			return pnum;

		vidh->sqnum = cpu_to_be64(ubi_next_sqnum(ubi));
		vidh->lnum = cpu_to_be32(i);

		ret = ubi_io_write_vid_hdr(ubi, pnum, vidb);
		if (ret)
			return ret;

		jnl->pnums[new_head] = pnum;
	}

	return 0;
}

static int ubi_jnl_scan(struct ubi_device *ubi, int head, int tail)
{

}

static int scan_ec(struct ubi_device *ubi, struct ubi_attach_info *ai,
		   int pnum, int *next_pnum)
{
	struct ubi_ec_hdr *ech = ai->ech;
	int err, bitflips = 0, ec_err, ec;
	int version, image_seq;

	err = ubi_io_read_ec_hdr(ubi, pnum, ech, 0);
	if (err < 0)
		return err;

	switch (err) {
	case 0:
		break;
	case UBI_IO_BITFLIPS:
		bitflips = 1;
		break;
	case UBI_IO_FF:
		ai->empty_peb_count += 1;
		return add_to_list(ai, pnum, UBI_UNKNOWN, UBI_UNKNOWN,
				   UBI_UNKNOWN, 0, &ai->erase);
	case UBI_IO_FF_BITFLIPS:
		ai->empty_peb_count += 1;
		return add_to_list(ai, pnum, UBI_UNKNOWN, UBI_UNKNOWN,
				   UBI_UNKNOWN, 1, &ai->erase);
	case UBI_IO_BAD_HDR_EBADMSG:
	case UBI_IO_BAD_HDR:
		/*
		 * We have to also look at the VID header, possibly it is not
		 * corrupted. Set %bitflips flag in order to make this PEB be
		 * moved and EC be re-created.
		 */
		ec = UBI_UNKNOWN;
		bitflips = 1;
		break;
	default:
		ubi_err(ubi, "'ubi_io_read_ec_hdr()' returned unknown code %d",
			err);
		return -EINVAL;
	}

	if (ec_err)
		return ec_err;


	/* Initialize the version value to the EC header one. */
	version = ech->version;

	ec = be64_to_cpu(ech->ec);
	if (ec > UBI_MAX_ERASECOUNTER) {
		/*
		 * Erase counter overflow. The EC headers have 64 bits
		 * reserved, but we anyway make use of only 31 bit
		 * values, as this seems to be enough for any existing
		 * flash. Upgrade UBI and use 64-bit erase counters
		 * internally.
		 */
		ubi_err(ubi, "erase counter overflow, max is %d",
			UBI_MAX_ERASECOUNTER);
		ubi_dump_ec_hdr(ech);
		return -EINVAL;
	}

	/* Mark the PEB as scanned. */
	set_bit(pnum, ai->ec_scan_map);
	ai->scanned_ec++;

	/* FIXME: add the apeb entry to the un-attached list. */

	/*
	 * Make sure that all PEBs have the same image sequence number.
	 * This allows us to detect situations when users flash UBI
	 * images incorrectly, so that the flash has the new UBI image
	 * and leftovers from the old one. This feature was added
	 * relatively recently, and the sequence number was always
	 * zero, because old UBI implementations always set it to zero.
	 * For this reasons, we do not panic if some PEBs have zero
	 * sequence number, while other PEBs have non-zero sequence
	 * number.
	 */
	image_seq = be32_to_cpu(ech->image_seq);
	if (!ubi->image_seq)
		ubi->image_seq = image_seq;
	if (image_seq && ubi->image_seq != image_seq) {
		ubi_err(ubi, "bad image sequence number %d in PEB %d, expected %d",
			image_seq, pnum, ubi->image_seq);
		ubi_dump_ec_hdr(ech);
		return -EINVAL;
	}

	/* UBI journal had been supported in version 2. */
	if (version < 2)
		return -ENOTSUPP;

	/*
	 * If both head an tail are set to zero, this means this PEB knows
	 * nothing about the UBI journal.
	 * Start iterating at the first EC header that was not scanned.
	 */
	if (!ech->jnl_head && !ech->jnl_tail) {
		*next_pnum = find_first_zero_bit(ai->ec_scan_map,
						 ubi->peb_count);
		return 0;
	}

	*next_pnum = ech->jnl_head;

	return 0;
}

int ubi_jnl_attach(struct ubi_device *ubi, struct ubi_attach_info *ai)
{
	int pnum = 0, err;

	while (ai->scanned_ec < ubi->peb_count) {
		err = scan_ec(ubi, ai, pnum, &pnum);
		if (err)
			return err;
	}

	return 0;
}

static int scan_jnl(struct ubi_device *ubi)
{
	struct ubi_journal *jnl = ubi->jnl;

}

void ubi_jnl_fill_ec_hdr(struct ubi_device *ubi, struct ubi_ec_hdr *ech)
{
	struct ubi_journal *jnl = ubi->jnl;

	ech->jnl_tail = cpu_to_be32(jnl->pnums[jnl->tail_lnum]);
	ech->jnl_head = cpu_to_be32(jnl->pnums[jnl->head_lnum]);
}

//int ubi_jnl_move(struct ubi_device *ubi, int pnum)
//{
//	struct ubi_journal *jnl = ubi->jnl;
//	struct ubi_jnl_move_node move;
//	int ret, opnum;
//	u32 crc;
//
//	memset(&move, 0, sizeof(move));
//	move.type = UBI_JNL_MOVE_NODE;
//	move.pnum = jnl->pnums[1];
//	crc = crc32(UBI_CRC32_INIT, &move, sizeof(move) - sizeof(u32));
//	move.crc = cpu_to_be32(move.pnum);
//
//	ret = ubi_io_write_data(ubi, &move, jnl->pnums[0], jnl->loffset,
//				sizeof(move), UBI_IO_MODE_SLC);
//	if (ret)
//		return ret;
//
//	/* TODO: write summary node at the beginning of the new journal. */
//	opnum = jnl->pnums[0];
//	jnl->pnums[0] = jnl->pnums[1];
//	jnl->loffset = 0;
//
//	ret = ubi_io_sync_erase(ubi, opnum, 0);
//	if (ret)
//		return ret;
//
//	/* TODO: reserve a new PEB. */
//	jnl->pnums[1] = -1;
//
//	return 0;
//}
//
//int ubi_jnl_update(struct ubi_device *ubi)
//{
//	struct ubi_journal *jnl = ubi->jnl;
//	struct ubi_jnl_erase_peb *epeb;
//	void *buf;
//
//
//	return 0;
//}

#define UBI_JNL_PEB_POOL_SIZE		64

static int calc_jnl_size_in_lebs(struct ubi_device *ubi, int summary_size)
{
	int size, leb_size, nlebs;

	leb_size = ubi_io_leb_size(ubi, UBI_IO_MODE_SLC);
	nlebs = DIV_ROUND_UP(summary_size, leb_size);

	/*
	 * We need to be able to store 2 summaries, to guarantee that the old
	 * one is still valid while we're writing the new one.
	 */
	nlebs *= 2;

	/*
	 * If the summary takes more than 1 PEB, or the remaining space in the
	 * summary PEB is less than 1/4 the leb_size, reserve an extra PEB for
	 * the journal.
	 */
	size = leb_size - (summary_size % leb_size);
	if (nlebs > 2 || size < leb_size / 4)
		nlebs++;

	return nlebs;
}

static void calc_max_summary_size(struct ubi_device *ubi)
{
	struct ubi_journal *jnl = ubi->jnl;
	int size = 0, max_lebs, leb_size, num_jnl_lebs;

	size += sizeof(struct ubi_jnl_summary_node);
	size += sizeof(struct ubi_jnl_peb_pool_node) +
		UBI_JNL_MAX_PEB_POOL_SIZE * sizeof(__be32);

	size += sizeof(struct ubi_jnl_peb_ec_summary_node) +
		(ubi->peb_count * sizeof(__be32));
	size = ALIGN(size, 8);

	size += sizeof(struct ubi_jnl_peb_status_summary_node) +
		(ubi->peb_count * sizeof(__u8));
	size = ALIGN(size, 8);

	size += sizeof(struct ubi_jnl_vol_summary_node) +
		((sizeof(struct ubi_jnl_vol_node) +
		  sizeof(struct ubi_jnl_eba_pnum_node) +
		  sizeof(struct ubi_jnl_eba_lpos_node)) *
		 (UBI_MAX_VOLUMES + UBI_INT_VOL_COUNT));

	/*
	 * Count the maximum number of LEBs.
	 */
	max_lebs = ubi->peb_count * ubi->max_lebs_per_peb;
	size += max_lebs * sizeof(__be32);
	size = ALIGN(size, 8);
	size += max_lebs * sizeof(__u8);;
	size = ALIGN(size, 8);

	num_jnl_lebs = calc_jnl_size_in_pebs(ubi, size);

	/*
	 * We double the number of PEB slots here, just in case the new size
	 * cross a LEB boundary, and then we calculate the final number of
	 * PEBs.
	 */
	size += sizeof(struct ubi_jnl_layout_node) +
		(num_jnl_lebs * 2 * sizeof(__be32));
	size = ALIGN(size, max(8, ubi->max_write_size));

	num_jnl_lebs = calc_jnl_size_in_lebs(ubi, size);

	jnl->max_summary_size = size;
}

static int init_jbuf(struct ubi_device *ubi)
{
	struct ubi_journal *jnl = ubi->jnl;
	struct ubi_jnl_buf *jbuf = &jnl->jbuf;
	int size;

	calc_max_summary_size(ubi);

	size = max(jnl->max_summary_size, ubi->max_write_size);
	size = max(size, ubi->vid_hdr_alsize);

	jbuf->buf = vmalloc(size, GFP_KERNEL);
	if (!jbuf->buf)
		return -ENOMEM;

	return 0;
}

int ubi_jnl_init(struct ubi_device *ubi)
{
	struct ubi_journal *jnl;
	int ret;

	jnl = kzalloc(sizeof(jnl), GFP_KERNEL);
	if (!jnl)
		return -ENOMEM;

	ret = init_jbuf(ubi);
	if (ret)
		return ret;

	return 0;

err:
	kfree(jnl);
	return ret;
}

void ubi_jnl_cleanup(struct ubi_device *ubi)
{

}
