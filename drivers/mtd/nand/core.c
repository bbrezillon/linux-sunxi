#include <linux/mtd/nand2.h>

static LIST_HEAD(controllers);
static DEFINE_MUTEX(lock);

//static LIST_HEAD(manufacturers);
//static DEFINE_MUTEX(lock);

static inline void nand_operation_init(struct nand_operation *op)
{
	memset(op, 0, sizeof(op));
}

void nand_op_read_large_page(struct nand_device *dev,
			     struct nand_operation *op,
			     int page, int column, void *buf, size_t len)
{
	nand_operation_init(op);

	/* Adjust columns for 16 bit buswidth */
	if (dev->options & NAND_BUSWIDTH_16)
		column >>= 1;

	op.ncmds = 2;
	op.cmds[0] = NAND_CMD_READ0;
	op.cmds[1] = NAND_CMD_READSTART;
	op.naddrs[0] = 4;
	op.addrs[0][0] = column;
	op.addrs[0][1] = column >> 8;
	op.addrs[0][2] = page;
	op.addrs[0][3] = page >> 8;
	if (dev->mtd.size > (128 << 20)) {
		op.addrs[0][3] = page >> 16;
		op.naddrs[0]++;
	}

	op.wait_ready[1] = true;
	op.dir = NAND_OP_INPUT;
	op.data[0].in = buf;
	op.len[0] = len;
}

void nand_op_read_small_page(struct nand_device *dev,
			     struct nand_operation *op,
			     int page, int column, void *buf, size_t len)
{
	nand_operation_init(op);

	/* Adjust columns for 16 bit buswidth */
	if (dev->options & NAND_BUSWIDTH_16)
		column >>= 1;

	op.ncmds = 1;
	if (column < 256)
		op.cmds[0] = NAND_CMD_READ0;
	else if (column < dev->mtd.writesize)
		op.cmds[0] = NAND_CMD_READ1;
	else
		op.cmds[0] = NAND_CMD_READOOB;

	op.naddrs[0] = 3;
	op.addrs[0][0] = column;
	op.addrs[0][1] = page;
	op.addrs[0][2] = page >> 8;
	if (dev->mtd.size > (32 << 20)) {
		op.addrs[0][3] = page >> 16;
		op.naddrs[0]++;
	}

	op.wait_ready[1] = true;
	op.dir = NAND_OP_INPUT;
	op.data[0].in = buf;
	op.len[0] = len;
}

void nand_op_start_read_cache(struct nand_device *dev,
			      struct nand_operation *op,
			      int page, int column, void *buf, size_t len)
{
	nand_operation_init(op);

	op.ncmds = 3;
	op.cmds[0] = NAND_CMD_READ0;
	op.cmds[1] = NAND_CMD_READSTART;
	op.cmds[2] = NAND_CMD_READCACHE;
	op.naddrs[0] = 4;
	op.addrs[0][0] = column;
	op.addrs[0][1] = column >> 8;
	op.addrs[0][2] = page;
	op.addrs[0][3] = page >> 8;
	if (dev->mtd.size > (128 << 20)) {
		op.addrs[0][3] = page >> 16;
		op.naddrs[0]++;
	}

	op.wait_ready[1] = true;
	op.wait_ready[2] = true;
	op.dir = NAND_OP_INPUT;
	op.data[1].in = buf;
	op.len[1] = len;
}

void nand_op_rnd_read_cache(struct nand_device *dev, struct nand_operation *op,
			    void *buf, size_t len)
{
	nand_operation_init(op);

	op.ncmds = 1;
	op.cmds[0] = NAND_CMD_READCACHE;
	op.wait_ready[0] = true;
	op.dir = NAND_OP_INPUT;
	op.data[0].in = buf;
	op.len[0] = len;
}

void nand_op_read_cache(struct nand_device *dev, struct nand_operation *op,
			void *buf, size_t len)
{
	nand_operation_init(op);

	op.ncmds = 1;
	op.cmds[0] = NAND_CMD_READCACHE;
	op.wait_ready[0] = true;
	op.dir = NAND_OP_INPUT;
	op.data[0].in = buf;
	op.len[0] = len;
}

void nand_op_last_read_cache(struct nand_device *dev,
			     struct nand_operation *op,
			     void *buf, size_t len)
{
	nand_operation_init(op);

	op.ncmds = 1;
	op.cmds[0] = NAND_CMD_LASTREADCACHE;
	op.wait_ready[0] = true;
	op.dir = NAND_OP_INPUT;
	op.data[0].in = buf;
	op.len[0] = len;
}

static int nand_raw_read_cached(struct nand_device *dev, int page, int column,
				void *buf, size_t len)
{
	size_t chunksize, remains;
	struct nand_operation op;
	int chipnr, ret;

	chipnr = (int)(page >> (dev->chip_shift - dev->page_shift));
	dev->controller->ops->select(dev, chipnr);

	if (len < dev->mtd.writesize - column) {
		nand_op_read_large_page(dev, &op, page, column, buf, len);
		return dev->controller->ops->launch(dev, &op);
	}

	nand_op_read_large_page(dev, &op, page, column, NULL, 0);
	ret = dev->controller->ops->launch(dev, &op);
	if (ret)
		return ret;

	for (remains = len; remains; buf += chunksize) {
		chunksize = remains < (dev->mtd.writesize - column) ?
			    remains : (dev->mtd.writesize - column);
		remains -= chunksize;

		if (remains <= dev->mtd.writesize)
			nand_op_read_cache(dev, &op, buf, len);
		else
			nand_op_last_read_cache(dev, &op, buf, len);

		ret = dev->controller->ops->launch(dev, &op);
		if (ret)
			return ret;

		column = 0;
	}

	dev->controller->ops->select(dev, -1);

	return 0;
}

static int nand_basic_ecc_read(struct nand_device *dev, int page, int column,
			       void *buf, size_t len)
{
	struct nand_basic_ecc_controller *ecc_ctrl =
			to_basic_ecc_controller(dev->ecc.desc->controller);
	struct nand_operation op;
	int chipnr;

	chipnr = (int)(page >> (dev->chip_shift - dev->page_shift));
	dev->controller->ops->select(dev, chipnr);

	ecc_ctrl->ops->enable();
	while (1) {

	}
	ecc_ctrl->ops->disable();

	return 0;
}

static void nand_op_readid(struct nand_operation *op, u8 addr, void *buf,
			   size_t len)
{
	uint8_t *tmp = NULL;

	nand_operation_init(op);

	op->ncmds = 1;
	op->cmds[0] = NAND_CMD_READID;
	op->naddrs[0] = 1;
	op->addrs[0][0] = addr;
	op->dir = NAND_OP_INPUT;
	op->len[0] = len;
	op->data[0].in = buf;
}

//static int nand_read(struct nand_device *dev, int page, int column, )

//static int nand_do_read_ops(struct mtd_info *mtd, loff_t from,
//			    struct mtd_oob_ops *ops)
//{
//	int chipnr, page, realpage, col, bytes, aligned, oob_required;
//	struct nand_chip *chip = mtd->priv;
//	int ret = 0;
//	uint32_t readlen = ops->len;
//	uint32_t oobreadlen = ops->ooblen;
//	uint32_t max_oobsize = ops->mode == MTD_OPS_AUTO_OOB ?
//		mtd->oobavail : mtd->oobsize;
//
//	uint8_t *bufpoi, *oob, *buf;
//	int use_bufpoi;
//	unsigned int max_bitflips = 0;
//	int retry_mode = 0;
//	bool ecc_fail = false;
//
//	chipnr = (int)(from >> chip->chip_shift);
//	chip->select_chip(mtd, chipnr);
//
//	realpage = (int)(from >> chip->page_shift);
//	page = realpage & chip->pagemask;
//
//	col = (int)(from & (mtd->writesize - 1));
//
//	buf = ops->datbuf;
//	oob = ops->oobbuf;
//	oob_required = oob ? 1 : 0;
//
//	while (1) {
//		unsigned int ecc_failures = mtd->ecc_stats.failed;
//
//		bytes = min(mtd->writesize - col, readlen);
//		aligned = (bytes == mtd->writesize);
//
//		if (!aligned)
//			use_bufpoi = 1;
//		else if (chip->options & NAND_USE_BOUNCE_BUFFER)
//			use_bufpoi = !virt_addr_valid(buf);
//		else
//			use_bufpoi = 0;
//
//		/* Is the current page in the buffer? */
//		if (realpage != chip->pagebuf || oob) {
//			bufpoi = use_bufpoi ? chip->buffers->databuf : buf;
//
//			if (use_bufpoi && aligned)
//				pr_debug("%s: using read bounce buffer for buf@%p\n",
//						 __func__, buf);
//
//read_retry:
//			nand_readpage_op(mtd, page, 0, NULL, 0);
//
//			/*
//			 * Now read the page into the buffer.  Absent an error,
//			 * the read methods return max bitflips per ecc step.
//			 */
//			if (unlikely(ops->mode == MTD_OPS_RAW))
//				ret = chip->ecc.read_page_raw(mtd, chip, bufpoi,
//							      oob_required,
//							      page);
//			else if (!aligned && NAND_HAS_SUBPAGE_READ(chip) &&
//				 !oob)
//				ret = chip->ecc.read_subpage(mtd, chip,
//							col, bytes, bufpoi,
//							page);
//			else
//				ret = chip->ecc.read_page(mtd, chip, bufpoi,
//							  oob_required, page);
//			if (ret < 0) {
//				if (use_bufpoi)
//					/* Invalidate page cache */
//					chip->pagebuf = -1;
//				break;
//			}
//
//			max_bitflips = max_t(unsigned int, max_bitflips, ret);
//
//			/* Transfer not aligned data */
//			if (use_bufpoi) {
//				if (!NAND_HAS_SUBPAGE_READ(chip) && !oob &&
//				    !(mtd->ecc_stats.failed - ecc_failures) &&
//				    (ops->mode != MTD_OPS_RAW)) {
//					chip->pagebuf = realpage;
//					chip->pagebuf_bitflips = ret;
//				} else {
//					/* Invalidate page cache */
//					chip->pagebuf = -1;
//				}
//				memcpy(buf, chip->buffers->databuf + col, bytes);
//			}
//
//			if (unlikely(oob)) {
//				int toread = min(oobreadlen, max_oobsize);
//
//				if (toread) {
//					oob = nand_transfer_oob(chip,
//						oob, ops, toread);
//					oobreadlen -= toread;
//				}
//			}
//
//			if (chip->options & NAND_NEED_READRDY) {
//				/* Apply delay or wait for ready/busy pin */
//				if (!chip->dev_ready)
//					udelay(chip->chip_delay);
//				else
//					nand_wait_ready(mtd);
//			}
//
//			if (mtd->ecc_stats.failed - ecc_failures) {
//				if (retry_mode + 1 < chip->read_retries) {
//					retry_mode++;
//					ret = nand_setup_read_retry(mtd,
//							retry_mode);
//					if (ret < 0)
//						break;
//
//					/* Reset failures; retry */
//					mtd->ecc_stats.failed = ecc_failures;
//					goto read_retry;
//				} else {
//					/* No more retry modes; real failure */
//					ecc_fail = true;
//				}
//			}
//
//			buf += bytes;
//		} else {
//			memcpy(buf, chip->buffers->databuf + col, bytes);
//			buf += bytes;
//			max_bitflips = max_t(unsigned int, max_bitflips,
//					     chip->pagebuf_bitflips);
//		}
//
//		readlen -= bytes;
//
//		/* Reset to retry mode 0 */
//		if (retry_mode) {
//			ret = nand_setup_read_retry(mtd, 0);
//			if (ret < 0)
//				break;
//			retry_mode = 0;
//		}
//
//		if (!readlen)
//			break;
//
//		/* For subsequent reads align to page boundary */
//		col = 0;
//		/* Increment page address */
//		realpage++;
//
//		page = realpage & chip->pagemask;
//		/* Check, if we cross a chip boundary */
//		if (!page) {
//			chipnr++;
//			chip->select_chip(mtd, -1);
//			chip->select_chip(mtd, chipnr);
//		}
//	}
//	chip->select_chip(mtd, -1);
//
//	ops->retlen = ops->len - (size_t) readlen;
//	if (oob)
//		ops->oobretlen = ops->ooblen - oobreadlen;
//
//	if (ret < 0)
//		return ret;
//
//	if (ecc_fail)
//		return -EBADMSG;
//
//	return max_bitflips;
//}

static ssize_t
modalias_show(struct device *dev, struct device_attribute *a, char *buf)
{
	const struct nand_device *nand = dev_to_nand(dev);
	int len;

	return sprintf(buf, NAND_MODALIAD_FMT "\n", nand->id[0], nand->id[0]);
}
static DEVICE_ATTR_RO(modalias);

static struct attribute *nand_dev_attrs[] = {
	&dev_attr_modalias.attr,
	NULL,
};
ATTRIBUTE_GROUPS(nand_dev);

static int nand_match_device(struct device *dev, struct device_driver *drv)
{
	const struct nand_device *nand = to_spi_device(dev);
	const struct nand_driver *ndrv = to_spi_driver(drv);
	const struct nand_device_id *id;

	for(id = ndrv->id_table; id; id++) {
		if (id->manufacturer != nand->id[0])
			continue;

		if (id->device == NAND_ANY_DEVICE ||
		    id->device == nand->id[1])
			return 1;
	}

	return 0;
}

static int nand_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	const struct nand_device *nand = dev_to_nand(dev);
	int rc;

	return add_uevent_var(env, "MODALIAS=" NAND_MODALIAS_FMT,
			      nand->id[0], nand->id[1]);
}

struct bus_type nand_bus_type = {
	.name		= "nand",
	.dev_groups	= nand_dev_groups,
	.match		= nand_match_device,
	.uevent		= nand_uevent,
};

static inline void nand_adjust_8bit_buffer(struct nand_device *dev, void *dst,
					 const void *src, size_t len)
{
	int i;

	if (dev->hw_interface.buswidth == 16)
		memcpy(dst, src, len);

	for (i = 0; i < len; i++)
		((u8 *)dst)[i] = ((u8 *)src)[i * 2];
}

int nand_device_add(struct nand_device *dev)
{
	int len = dev->hw_interface.buswidth == 16 ? 4 : 2;
	struct nand_controller *ctrl = dev->controller;
	struct nand_operation op;
	u8 buf[4], id[2];
	int ret, i;

	if (dev->hw_interface.cs.num <= 0)
		return -EINVAL;

	for (i = 0; i < ctrl->numcs; i++) {
		ctrl->ops->select(dev, i);
		nand_op_readid(&op, 0x00, buf, len);
		ret = ctrl->ops->launch(dev, &op);
		if (ret)
			return ret;

		nand_adjust_8bit_buffer(dev, id, buf, sizeof(id));

		if (!i)
			memcpy(dev->id, id, sizeof(dev->id));
		else if (memcmp(dev->id, id, sizeof(dev->id)))
			return -EINVAL;
	}

	ret = device_add(&dev->dev);
	if (ret)
		return ret;

	return 0;
}

static void nand_device_release(struct device *dev)
{
	struct nand_device *nand = dev_to_nand(dev);

	/* spi masters may cleanup for released devices */
	if (nand->controller->ops->cleanup)
		nand->controller->ops->cleanup(nand);

	put_device(&nand->controller->dev);
	kfree(nand);
}

struct nand_device *nand_device_alloc(struct nand_controller *ctrl)
{
	struct nand_device *nand;

	if (!ctrl || !get_device(&ctrl->dev))
		return NULL;

	nand = kzalloc(sizeof(*nand), GFP_KERNEL);
	if (!nand) {
		put_device(&ctrl->dev);
		return NULL;
	}

	nand->controller = ctrl;
	nand->dev.parent = &ctrl->dev;
	nand->dev.bus = &nand_bus_type;
	nand->dev.release = nand_device_release;
	device_initialize(&nand->dev);

	return nand;
}
EXPORT_SYMBOL_GPL(spi_alloc_device);

#if defined(CONFIG_OF)
static struct nand_device *
of_nand_device_register(struct nand_controller *ctrl, struct device_node *np)
{
	struct nand_device *nand;
	struct property *prop;
	const __be32 *p;
	int *ids, ret;
	u32 value;

	/* Alloc an spi_device */
	nand = nand_alloc_device(ctrl);
	if (!nand) {
		dev_err(&ctrl->dev, "nand_device alloc error for %s\n",
			np->full_name);
		return ERR_PTR(-ENOMEM);
	}

	/* Device address */
	ret = of_property_count_u32_elems(np, "reg");
	if (ret <= 0) {
		if (!ret)
			ret = -EINVAL;
		dev_err(&ctrl->dev, "%s has no valid 'reg' property (%d)\n",
			np->full_name, ret);
		goto err_out;
	}

	ids = devm_kzalloc(&nand->dev, sizeof(*ids) * ret, GFP_KERNEL);
	if (ids) {
		ret = -ENOMEM;
		goto err_out;
	}
	nand->hw_interface.cs.num = ret;
	nand->hw_interface.cs.num = ids;

	of_property_for_each_u32(np, "reg", prop, p, value)
		*ids++ = value;

	ret = nand_device_add(ctrl);
	if (ret) {
		dev_err(&ctrl->dev, "nand_device register error %s\n",
			np->full_name);
		goto err_out;
	}

	return nand;

err_out:
	put_device(&nand->dev);
	return ERR_PTR(ret);
}

static void of_nand_devices_register(struct nand_controller *ctrl)
{
	struct device_node *np = ctrl->dev.of_node, *nc;
	struct nand_device *nand;

	if (!np)
		return;

	for_each_available_child_of_node(np, nc) {
		nand = of_nand_device_register(ctrl, nc);
		if (IS_ERR(nand))
			dev_warn(&ctrl->dev, "Failed to create NAND device for %s\n",
				nc->full_name);
	}
}
#else
static void of_nand_devices_register(struct nand_controller *ctrl) { }
#endif

static void nand_devices_register(struct nand_controller *ctrl)
{
	of_nand_devices_register(ctrl);
}

int nand_device_register(struct nand_controller *ctrl, struct nand_device *dev)
{
	int ret;

	dev->controller = ctrl;

	ret = ctrl->ops->add(dev);
	if (ret)
		return ret;

	mutex_lock(&ctrl->devices.lock);
	list_add_tail(&dev->node, &ctrl->devices.list);
	mutex_unlock(&ctrl->devices.lock);

	return 0;
}

#define NAND_ID_MAX_LEN		16

//static nand_controller_scan(struct nand_controller *ctrl)
//{
//	struct nand_operation op;
//	u8 id[NAND_ID_MAX_LEN];
//	int i;
//
//	for (i = 0; i < ctrl->numcs; i++) {
//		nand_op_readid(&op, 0x00, void *buf,
//					   size_t len)
//	}
//}

static void nand_controller_release(struct device *dev)
{
	struct nand_controller *ctrl;

	ctrl = container_of(dev, struct nand_controller, dev);
	kfree(ctrl);
}

static struct class nand_controller_class = {
	.name		= "nand-controller",
	.owner		= THIS_MODULE,
	.dev_release	= nand_controller_release,
};

struct nand_controller *nand_controller_alloc(struct device *dev, unsigned size)
{
	struct nand_controller *ctrl;

	if (!dev)
		return NULL;

	ctrl = kzalloc(size + sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return NULL;

	device_initialize(&ctrl->dev);
	ctrl->dev.class = &nand_controller_class;
	ctrl->dev.parent = get_device(dev);
	nand_controller_set_devdata(ctrl, &ctrl[1]);

	return ctrl;
}
EXPORT_SYMBOL_GPL(nand_controller_alloc);

int nand_controller_register(struct nand_controller *ctrl)
{
	if (ctrl->numcs <= 0)
		return -EINVAL;

	mutex_init(&ctrl->lock);
	init_waitqueue_head(&ctrl->wq);
	ctrl->active = NULL;
	device_add(&ctrl->dev);
	INIT_LIST_HEAD(&ctrl->devices.list);
	mutex_init(&ctrl->devices.lock);

	mutex_lock(&lock);
	list_add_tail(&ctrl->node, &controllers);
	mutex_unlock(&lock);

	nand_devices_register(ctrl);

	return 0;
}

int nand_controller_unregister(struct nand_controller *ctrl)
{
	return 0;
}
