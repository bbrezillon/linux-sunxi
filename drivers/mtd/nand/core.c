#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mtd/nand2.h>
#include <linux/of.h>

static LIST_HEAD(controllers);
static DEFINE_MUTEX(lock);

//static LIST_HEAD(manufacturers);
//static DEFINE_MUTEX(lock);

//static inline void nand_operation_init(struct nand_operation *op,
//				       struct nand_device *dev, int die)
//{
//	memset(op, 0, sizeof(op));
//	op->dev = dev;
//	op->chip = chip;
//
//}

void nand_op_read_large_page(struct nand_operation *op,
			     struct nand_device *dev, int chip,
			     int page, int column, void *buf, size_t len)
{
	nand_operation_init(op, dev, chip);

	/* Adjust columns for 16 bit buswidth */
	if (dev->hw_interface.buswidth == 16)
		column >>= 1;

	op->ncmds = 2;
	op->cmds[0] = NAND_CMD_READ0;
	op->cmds[1] = NAND_CMD_READSTART;
	op->naddrs = 4;
	op->addrs[0] = column;
	op->addrs[1] = column >> 8;
	op->addrs[2] = page;
	op->addrs[3] = page >> 8;
	if (dev->mtd.size > (128 << 20)) {
		op->addrs[3] = page >> 16;
		op->naddrs++;
	}

	op->flags = NAND_OP_WAIT_LUN_RDY;
	op->dir = NAND_OP_INPUT;
	op->data.in = buf;
	op->data.len = len;
}

void nand_op_read_small_page(struct nand_operation *op,
			     struct nand_device *dev, int chip,
			     int page, int column, void *buf, size_t len)
{
	nand_operation_init(op, dev, chip);

	/* Adjust columns for 16 bit buswidth */
	if (dev->hw_interface.buswidth == 16)
		column >>= 1;

	op->ncmds = 1;
	if (column < 256)
		op->cmds[0] = NAND_CMD_READ0;
	else if (column < dev->mtd.writesize)
		op->cmds[0] = NAND_CMD_READ1;
	else
		op->cmds[0] = NAND_CMD_READOOB;

	op->naddrs = 3;
	op->addrs[0] = column;
	op->addrs[1] = page;
	op->addrs[2] = page >> 8;
	if (dev->mtd.size > (32 << 20)) {
		op->addrs[3] = page >> 16;
		op->naddrs++;
	}

	op->flags = NAND_OP_WAIT_LUN_RDY;
	op->dir = NAND_OP_INPUT;
	op->data.in = buf;
	op->data.len = len;
}

void nand_op_rnd_read_cache(struct nand_operation *op,
			    struct nand_device *dev, int chip,
			    void *buf, size_t len)
{
	nand_operation_init(op, dev, chip);

	op->ncmds = 1;
	op->cmds[0] = NAND_CMD_READCACHE;
	op->flags = NAND_OP_WAIT_LUN_RDY;
	op->dir = NAND_OP_INPUT;
	op->data.in = buf;
	op->data.len = len;
}

void nand_op_read_cache(struct nand_operation *op,
			struct nand_device *dev, int chip,
			void *buf, size_t len)
{
	nand_operation_init(op, dev, chip);

	op->ncmds = 1;
	op->cmds[0] = NAND_CMD_READCACHE;
	op->flags = NAND_OP_WAIT_LUN_RDY;
	op->dir = NAND_OP_INPUT;
	op->data.in = buf;
	op->data.len = len;
}

void nand_op_last_read_cache(struct nand_operation *op,
		    	     struct nand_device *dev, int chip,
			     void *buf, size_t len)
{
	nand_operation_init(op, dev, chip);

	op->ncmds = 1;
	op->cmds[0] = NAND_CMD_LASTREADCACHE;
	op->flags = NAND_OP_WAIT_LUN_RDY;
	op->dir = NAND_OP_INPUT;
	op->data.in = buf;
	op->data.len = len;
}

//static int nand_raw_read_cached(struct nand_device *dev, int page, int column,
//				void *buf, size_t len)
//{
//	size_t chunksize, remains;
//	struct nand_operation op;
//	int chipnr, ret;
//
//	chipnr = (page * dev->mtd.writesize) / dev->chipsize;
//
//	if (len < dev->mtd.writesize - column) {
//		nand_op_read_large_page(&op, dev, chipnr, page,
//					column, buf, len);
//		return nand_controller_exec_op_sync(&op);
//	}
//
//	nand_op_read_large_page(&op, dev, chipnr, page, column, NULL, 0);
//	ret = nand_controller_exec_op_sync(&op);
//	if (ret)
//		return ret;
//
//	for (remains = len; remains; buf += chunksize) {
//		chunksize = remains < (dev->mtd.writesize - column) ?
//			    remains : (dev->mtd.writesize - column);
//		remains -= chunksize;
//
//		if (remains <= dev->mtd.writesize)
//			nand_op_read_cache(&op, dev, chipnr, buf, len);
//		else
//			nand_op_last_read_cache(&op, dev, chipnr, buf, len);
//
//		ret = nand_controller_exec_op_sync(&op);
//		if (ret)
//			return ret;
//
//		column = 0;
//	}
//
//	return 0;
//}
//
//static int nand_basic_ecc_read(struct nand_device *dev, int page, int column,
//			       void *buf, size_t len)
//{
//	struct nand_basic_ecc_controller *ecc_ctrl =
//			to_basic_ecc_controller(dev->ecc.desc->controller);
////	struct nand_operation op;
//	int chipnr;
//
//	chipnr = (page * dev->mtd.writesize) / dev->chipsize;
//
//	ecc_ctrl->ops->enable(dev);
//	while (1) {
//
//	}
//	ecc_ctrl->ops->disable(dev);
//
//	return 0;
//}

static void nand_op_readid(struct nand_operation *op,
			   struct nand_device *dev, int chip,
			   u8 addr, void *buf, size_t len)
{
	nand_operation_init(op, dev, chip);

	op->ncmds = 1;
	op->cmds[0] = NAND_CMD_READID;
	op->naddrs = 1;
	op->addrs[0] = addr;
	op->dir = NAND_OP_INPUT;
	op->data.len = len;
	op->data.in = buf;
	op->flags = NAND_OP_8BIT_MODE;
}

static void nand_op_read_param(struct nand_operation *op,
			       struct nand_device *dev, int chip,
			       u8 addr, void *buf, size_t len)
{
	nand_operation_init(op, dev, chip);

	op->ncmds = 1;
	op->cmds[0] = NAND_CMD_PARAM;
	op->naddrs = 1;
	op->addrs[0] = addr;
	op->dir = NAND_OP_INPUT;
	op->flags = NAND_OP_WAIT_DEV_RDY;
	op->data.len = len;
	op->data.in = buf;
	op->flags = NAND_OP_8BIT_MODE;
}

static void nand_op_rnd_read(struct nand_operation *op,
			     struct nand_device *dev, int chip,
			     int column, void *buf, size_t len)
{
	nand_operation_init(op, dev, chip);

	op->ncmds = 2;
	op->cmds[0] = NAND_CMD_RNDOUT;
	op->cmds[1] = NAND_CMD_RNDOUTSTART;
	op->naddrs = 2;
	op->addrs[0] = column;
	op->addrs[1] = column >> 8;
	op->dir = NAND_OP_INPUT;
	op->data.len = len;
	op->data.in = buf;
	op->flags = NAND_OP_8BIT_MODE;
}

/*
int nand_basic_launch_seq(struct nand_device *dev,
			  const struct nand_operation_seq *seq)
{
	int ret, i;

	for (i = 0; i < seq->nops; i++) {
		ret = dev->controller->ops->queue_op(dev, &seq->ops[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(nand_basic_launch_seq);
*/

//int nand_controller_queue_op(struct nand_operation *op)
//{
//	struct nand_controller *ctrl = op->dev->controller;
//	unsigned long flags;
//
//	spin_lock_irqsave(&ctrl->lock, flags);
//	list_add_tail(&op->node, &ctrl->queue);
//	if (!ctrl->cur)
//		ctrl->cur = op;
//	else
//		op = NULL;
//	spin_unlock_irqrestore(&ctrl->lock, flags);
//
//	if (!op)
//		return 0;
//
//	return ctrl->ops->launch_op(op);
//}
//EXPORT_SYMBOL_GPL(nand_controller_queue_op);
//
//int nand_controller_queue_seq(struct nand_operation_seq *seq)
//{
//	struct nand_controller *ctrl = seq->ops[0].dev->controller;
//	struct nand_operation *op = NULL;
//	unsigned long flags;
//	int i;
//
//	spin_lock_irqsave(&ctrl->lock, flags);
//	for (i = 0; i < seq->nops; i++)
//		list_add_tail(&seq->ops[i].node, &ctrl->queue);
//
//	if (!ctrl->cur) {
//		ctrl->cur = &seq->ops[0];
//		op = ctrl->cur;
//	}
//	spin_unlock_irqrestore(&ctrl->lock, flags);
//
//	if (!op)
//		return 0;
//
//	return ctrl->ops->launch_op(op);
//}
//EXPORT_SYMBOL_GPL(nand_controller_queue_seq);
//
//static void nand_controller_op_complete(struct nand_operation *op)
//{
//	struct completion *comp = op->priv;
//
//	complete(comp);
//}
//
//int nand_controller_exec_op_sync(struct nand_operation *op)
//{
//	DECLARE_COMPLETION_ONSTACK(comp);
//
//	op->complete = nand_controller_op_complete;
//	op->priv = &comp;
//	nand_controller_exec_op_async(op);
//	wait_for_completion(&comp);
//
//	return op->result;
//}

static ssize_t
modalias_show(struct device *dev, struct device_attribute *a, char *buf)
{
	const struct nand_device *nand = dev_to_nand(dev);

	return sprintf(buf, NAND_MODALIAS_FMT "\n", nand->id[0], nand->id[0]);
}
static DEVICE_ATTR_RO(modalias);

static struct attribute *nand_dev_attrs[] = {
	&dev_attr_modalias.attr,
	NULL,
};
ATTRIBUTE_GROUPS(nand_dev);

//static int nand_match_device(struct device *dev, struct device_driver *drv)
//{
//	const struct nand_device *nand = to_spi_device(dev);
//	const struct nand_driver *ndrv = to_spi_driver(drv);
//	const struct nand_device_id *id;
//
//	for(id = ndrv->id_table; id; id++) {
//		if (id->manufacturer != nand->id[0])
//			continue;
//
//		if (id->device == NAND_ANY_DEVICE ||
//		    id->device == nand->id[1])
//			return 1;
//	}
//
//	return 0;
//}

static int nand_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	const struct nand_device *nand = dev_to_nand(dev);

	return add_uevent_var(env, "MODALIAS=" NAND_MODALIAS_FMT,
			      nand->id[0], nand->id[1]);
}

struct bus_type nand_bus_type = {
	.name		= "nand",
	.dev_groups	= nand_dev_groups,
//	.match		= nand_match_device,
	.uevent		= nand_uevent,
};

static int nand_apply_conservative_timings(struct nand_device *dev)
{
	/* TODO: handle DDR interfaces */
	dev->hw_interface.timings.sdr =
			*onfi_async_timing_mode_to_sdr_timings(0);

	return dev->controller->ops->apply_timings(dev);
}

static u16 nand_onfi_crc16(const void *buf, size_t len)
{
	const u8 *p = buf;
	u16 crc = 0x4F4E;
	int i;

	while (len--) {
		crc ^= *p++ << 8;
		for (i = 0; i < 8; i++)
			crc = (crc << 1) ^ ((crc & 0x8000) ? 0x8005 : 0);
	}

	return crc;
}

static int nand_onfi_init(struct nand_device *dev)
{
	struct nand_onfi_params *params = &dev->sw_interface.onfi.params;
	size_t len = dev->hw_interface.buswidth == 16 ? 4 : 2;
	struct nand_operation op;
	int ret, i;
	u8 id[4];

	nand_op_readid(&op, dev, 0, 0x20, id, len);
	ret = nand_controller_exec_op_sync(&op);
	if (ret)
		return ret;

	if (memcmp(id, "ONFI", sizeof("ONFI")))
		return -ENOTSUPP;

	for (i = 0; i < 3; i++) {
		u16 crc;

		if (!i)
			nand_op_read_param(&op, dev, 0, 0x0,
					   params, sizeof(*params));
		else
			nand_op_rnd_read(&op, dev, 0, i * sizeof(*params),
					 params, sizeof(*params));

		ret = nand_controller_exec_op_sync(&op);
		if (ret)
			return ret;

		crc = nand_onfi_crc16(params, sizeof(params) - 2);
		if (crc == le16_to_cpu(params->crc))
			break;
	}

	if (i == 3)
		return -ENOTSUPP;

	dev->mtd.erasesize = 1 << (fls(le32_to_cpu(params->pages_per_block)) - 1);
	dev->mtd.erasesize *= dev->mtd.writesize;

	dev->mtd.oobsize = le16_to_cpu(params->spare_bytes_per_page);

	/* See erasesize comment */
	dev->size = 1 << (fls(le32_to_cpu(params->blocks_per_lun)) - 1);
	dev->size *= (u64)dev->mtd.erasesize * params->lun_count;
	dev->bits_per_cell = params->bits_per_cell;

	if (nand_onfi_feature(dev) & ONFI_FEATURE_16_BIT_BUS)
		dev->hw_interface.buswidth = 16;
	else
		dev->hw_interface.buswidth = 8;

	if (params->ecc_bits != 0xff) {
		dev->ecc.requirements.strength = params->ecc_bits;
		dev->ecc.requirements.step = 512;
	} else if ((nand_onfi_feature(dev) & ONFI_FEATURE_EXT_PARAM_PAGE)) {
//		/* The Extended Parameter Page is supported since ONFI 2.1. */
//		if (nand_flash_detect_ext_param_page(mtd, chip, p))
//			pr_warn("Failed to detect ONFI extended param page\n");
	} else {
		pr_warn("Could not retrieve ONFI ECC requirements\n");
	}

	return 0;
}

static int nand_sw_interface_init(struct nand_device *dev)
{
	return 0;
}

int nand_device_add(struct nand_device *dev)
{
	size_t len = dev->hw_interface.buswidth == 16 ? 4 : 2;
	struct nand_controller *ctrl = dev->controller;
	struct nand_operation op;
	u8 id[2];
	int ret, i;

	if (dev->hw_interface.cs.num <= 0)
		return -EINVAL;

	if (ctrl->ops->initialize) {
		ret = ctrl->ops->initialize(dev);
		if (ret)
			return ret;
	}

	ret = nand_apply_conservative_timings(dev);
	if (ret)
		goto err_cleanup;

	for (i = 0; i < ctrl->numcs; i++) {
		nand_op_readid(&op, dev, i, 0x00, id, len);
		ret = nand_controller_exec_op_sync(&op);
		if (ret)
			goto err_cleanup;

		if (!i) {
			memcpy(dev->id, id, sizeof(dev->id));
		} else if (memcmp(dev->id, id, sizeof(dev->id))) {
			ret = -EINVAL;
			goto err_cleanup;
		}
	}

	ret = device_add(&dev->dev);
	if (ret)
		goto err_cleanup;

	return 0;

err_cleanup:
	if (ctrl->ops->cleanup)
		ctrl->ops->cleanup(dev);

	return ret;
}

static void nand_device_release(struct device *dev)
{
	struct nand_device *nand = dev_to_nand(dev);

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
EXPORT_SYMBOL_GPL(nand_device_alloc);

#if defined(CONFIG_OF)
static struct nand_device *
of_nand_device_register(struct nand_controller *ctrl, struct device_node *np)
{
	struct nand_device *nand;
	struct property *prop;
	const __be32 *p;
	int *ids, ret;
	u32 value;

	nand = nand_device_alloc(ctrl);
	if (!nand) {
		dev_err(&ctrl->dev, "nand_device alloc error for %s\n",
			np->full_name);
		return ERR_PTR(-ENOMEM);
	}

	nand->dev.of_node = np;

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
	nand->hw_interface.cs.ids = ids;

	of_property_for_each_u32(np, "reg", prop, p, value)
		*ids++ = value;

	ret = nand_device_add(nand);
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
	int ret;

	if (ctrl->numcs <= 0)
		return -EINVAL;

	if (!ctrl->ops ||
	    !ctrl->ops->apply_timings || !ctrl->ops->queue_op ||
	    !ctrl->ops->queue_seq)
	spin_lock_init(&ctrl->lock);
	init_waitqueue_head(&ctrl->wq);
	ctrl->active = NULL;
	INIT_LIST_HEAD(&ctrl->devices.list);
	mutex_init(&ctrl->devices.lock);

	ret = device_add(&ctrl->dev);
	if (ret)
		return ret;

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
