#include <linux/mtd/nand.h>

struct nand_device;
struct nand_controller;
struct nand_ecc_controller;

enum nand_action_type {
	NAND_OP_ACTION,
	NAND_PRIV_ACTION,
};

enum nand_action_state {
	NAND_ACTION_CREATED,
	NAND_ACTION_EXECUTED,
	NAND_ACTION_WAITING,
	NAND_ACTION_DONE,
};

struct nand_action {
	struct list_head node;
	enum nand_action_type type;
	int status;
};

//enum nand_operation_dir {
//	NAND_OP_NONE,
//	NAND_OP_INPUT,
//	NAND_OP_OUTPUT,
//};

#define NAND_OP_DIR_MSK		GENMASK(1, 0)
#define NAND_OP_NO_DATA		(0 << 0)
#define NAND_OP_INPUT		(1 << 0)
#define NAND_OP_OUTPUT		(2 << 0)
#define NAND_OP_WAIT_RDY_MSK	GENMASK(3, 2)
#define NAND_OP_DO_NOT_WAIT	(0 << 2)
#define NAND_OP_WAIT_LUN_RDY	(1 << 2)
#define NAND_OP_WAIT_DEV_RDY	(2 << 2)
#define NAND_OP_8BIT_MODE	BIT(4)

struct nand_operation {
	struct nand_action action;
//	struct list_head node;
//	struct nand_device *dev;
//	int chip;
	u32 flags;
	u8 ncmds;
	u8 cmds[2];
	u8 naddrs;
	u8 addrs[5];
//	bool waitready;
	int page;
	int column;
	enum nand_operation_dir dir;
	struct {
		union {
			const u8 *out;
			u8 *in;
		};
		size_t len;
	} data;
	unsigned long timeout;
//	bool eightbitmode;

//	void (*complete)(struct nand_operation *op);
//	int result;
//	void *priv;
};

struct nand_priv_action {
	struct nand_action action;
	int (*func)(struct nand_priv_action *action);
};

struct nand_action_seq {
	struct list_head node;
	struct list_head actions;
	struct nand_device *dev;
	int die;
	struct nand_action *cur;
	void *priv;
//	int nops;
//	struct nand_operation *ops;
};

struct nand_controller_ops {
	int (*initialize)(struct nand_device *dev);
	void (*cleanup)(struct nand_device *dev);
	int (*add)(struct nand_device *dev);
	int (*remove)(struct nand_device *dev);
	int (*queue)(struct nand_action_seq *seq);
//	int (*queue_op)(struct nand_operation *op);
//	int (*queue_seq)(struct nand_operation_seq *seq);
//	int (*launch_op)(struct nand_operation *op);
	int (*apply_timings)(struct nand_device *dev);
	bool (*supported)(struct nand_controller *controller,
			  const struct nand_operation *op);
};

struct nand_devices {
	struct list_head list;
	struct mutex lock;
};

struct nand_controller {
	struct list_head node;
	struct device dev;
	spinlock_t lock;
	struct nand_device *active;
	wait_queue_head_t wq;
	const struct nand_controller_ops *ops;
	struct list_head queue;
	struct nand_operation *cur;
	struct nand_devices devices;
	int numcs;
};

struct nand_basic_controller_ops {
	int (*launch_op)(struct nand_device *dev,
		         const struct nand_operation *op);
};

struct nand_basic_controller {
	struct nand_controller base;
	struct nand_basic_controller_ops *ops;
	struct list_head queue;
	spinlock_t lock;
};

struct nand_driver {
	struct device_driver driver;
	const struct nand_device_id *id_table;
	int (*probe)(struct nand_device *dev);
	int (*remove)(struct nand_device *dev);
};

enum nand_sw_interface_type {
	NAND_NONSTD,
	NAND_ONFI,
	NAND_JEDEC,
};

struct nand_onfi {
	int version;
	struct nand_onfi_params params;
};

struct nand_jedec {
	int version;
	struct nand_jedec_params params;
};

struct nand_sw_interface {
	enum nand_sw_interface_type type;
	union {
		struct nand_onfi onfi;
		struct nand_jedec jedec;
	};
};

struct nand_ecc_controller_ops {
	int (*read)(struct nand_device *dev, int page, int column,
		    void *buf, size_t len);
	int (*write)(struct nand_device *dev, int page, int column,
		     const void *buf, size_t len);

	int (*read_page_raw)(struct nand_device *dev,
			     void *buf, int oob_required, int page);
	int (*write_page_raw)(struct nand_device *dev,
			      const void *buf, int oob_required);
	int (*read_page)(struct nand_device *dev,
			 void *buf, int oob_required, int page);
	int (*read_subpage)(struct nand_device *dev,
			    uint32_t offs, uint32_t len, void *buf, int page);
	int (*write_subpage)(struct nand_device *dev,
			     uint32_t offset, uint32_t data_len,
			     const void *data_buf, int oob_required);
	int (*write_page)(struct nand_device *dev,
			  const void *buf, int oob_required);
	int (*write_oob_raw)(struct nand_device *dev, int page);
	int (*read_oob_raw)(struct nand_device *dev, int page);
	int (*read_oob)(struct nand_device *dev, int page);
	int (*write_oob)(struct nand_device *dev, int page);

	bool (*supported)(struct nand_ecc_controller *controller,
			  const struct nand_operation *op);
};

enum nand_page_region_type {
	NAND_PAYLOAD_DATA,
	NAND_OOB_AVAILABLE,
	NAND_OOB_ECC,
};

struct nand_page_range {
	int start;
	int stop;
};

struct nand_page_region {
	enum nand_page_region_type type;
	struct nand_page_range phys;
	struct nand_page_range virt;
};

struct nand_page_layout {
	int nregions;
	struct nand_page_area *regions;
};

struct nand_ecc_controller {
	struct nand_ecc_controller_ops *ops;
};

struct nand_ecc_requirements {
	int step;
	int strength;
};

struct nand_ecc_desc {
	int step;
	int strength;
	struct nand_page_layout pagelayout;
	struct nand_ecclayout *ecclayout;
	struct nand_ecc_controller *controller;
};

struct nand_ecc {
	struct nand_ecc_requirements requirements;
	struct nand_ecc_desc *desc;
};

struct nand_basic_ecc_controller_ops {
	int (*enable)(struct nand_device *dev);
	int (*disable)(struct nand_device *dev);
	int (*reset)(struct nand_device *dev);
	int (*read_chunk)(struct nand_device *dev, struct nand_operation *op,
			  int id);
	int (*read_page)(struct nand_device *dev, struct nand_operation *op);
	int (*write_subpage)(struct nand_device *dev,
			     struct nand_operation *op, int id);
	int (*write_page)(struct nand_device *dev, struct nand_operation *op);
};

struct nand_basic_ecc_controller {
	struct nand_ecc_controller base;
	struct nand_basic_ecc_controller_ops *ops;
};

static inline struct nand_basic_ecc_controller *
to_basic_ecc_controller(struct nand_ecc_controller *controller)
{
	return container_of(controller, struct nand_basic_ecc_controller,
			    base);
}

struct nand_bad_block {
	int badblockpos;
	int badblockbits;
	uint8_t *bbt;
	struct nand_bbt_descr *bbt_td;
	struct nand_bbt_descr *bbt_md;

	struct nand_bbt_descr *badblock_pattern;
};

enum nand_cs_type {
	NAND_CS_GPIO,
	NAND_CS_PRIV,
};

struct nand_cs {
	enum nand_cs_type type;
	int id;
	union {
		struct gpio_desc *gpio;
		void *data;
	};
	int nusers;
};

enum nand_rb_type {
	NAND_RB_NONE,
	NAND_RB_GPIO,
	NAND_RB_PRIV,
};

struct nand_rb {
	enum nand_rb_type type;
	int id;
	union {
		struct gpio_desc *gpio;
		void *data;
	};
	int nusers;
};

enum nand_hw_interface_type {
	NAND_SDR,
};

struct nand_hw_interface {
	int buswidth;
	enum nand_hw_interface_type type;
	union {
		struct nand_sdr_timings sdr;
	} timings;
};

enum nand_die_type {
	NAND_CS_DIE,
	NAND_LUN_DIE
};

struct nand_die {
	enum nand_die_type type;
	struct list_head queue;
	struct action_seq *cur;
	struct nand_cs *cs;
	struct nand_rb *rb;
};

struct nand_layout {
	int page_shift;
	int phys_erase_shift;
	int bbt_erase_shift;
	int pagemask;

	int die_size;
	int die_shift;
	int ndies;
	struct nand_die *dies;
};

struct nand_device {
	struct list_head node;
	struct device dev;
	struct mtd_info mtd;
	u8 id[2];

	u64 size;
	struct nand_layout layout;

	u8 bits_per_cell;

	/*
	unsigned int options;
	unsigned int bbt_options;

	int page_shift;
	int phys_erase_shift;
	int bbt_erase_shift;
	int chip_shift;
	int numchips;
	u64 chipsize;
	int pagemask;
	int pagebuf;
	unsigned int pagebuf_bitflips;
	int subpagesize;
	u8 bits_per_cell;
	*/

	struct nand_hw_interface hw_interface;
	struct nand_sw_interface sw_interface;

	struct nand_controller *controller;
	void *controller_priv;

	int read_retries;

	flstate_t state;

	struct nand_ecc ecc;

	struct nand_buffers *buffers;

	struct nand_bad_block bb;

	void *priv;
};

static inline int nand_onfi_feature(struct nand_device *dev)
{
	return dev->sw_interface.type == NAND_ONFI ?
	       le16_to_cpu(dev->sw_interface.onfi.params.features) : 0;
}

static inline void nand_set_ctrldata(struct nand_device *dev, void *data)
{
	dev->controller_priv = data;
}

static inline void *nand_get_ctrldata(struct nand_device *dev)
{
	return dev->controller_priv;
}

static inline struct nand_device *dev_to_nand(struct device *dev)
{
	return container_of(dev, struct nand_device, dev);
}

static inline  struct nand_device *mtd_to_nand(struct mtd_info *mtd)
{
	return container_of(mtd, struct nand_device, mtd);
}

static inline void *nand_controller_get_devdata(struct nand_controller *ctrl)
{
	return dev_get_drvdata(&ctrl->dev);
}

static inline void nand_controller_set_devdata(struct nand_controller *ctrl,
					       void *data)
{
	dev_set_drvdata(&ctrl->dev, data);
}

struct nand_controller *nand_controller_alloc(struct device *dev, unsigned size);

int nand_controller_register(struct nand_controller *ctrl);

int nand_controller_unregister(struct nand_controller *ctrl);

int nand_basic_launch_seq(struct nand_device *dev,
			  const struct nand_operation_seq *seq);

//static inline int nand_controller_exec_op_async(struct nand_operation *op)
//{
//	return op->dev->controller->ops->queue_op(op);
//}
//
//int nand_controller_exec_op_sync(struct nand_operation *op);

static inline void nand_action_seq_init(struct nand_action_seq *seq,
					struct nand_device *dev, int die)
{
	INIT_LIST_HEAD(&seq->actions);
	seq->dev = dev;
	seq->die = die;
}
