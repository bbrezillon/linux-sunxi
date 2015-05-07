#include <linux/mtd/nand.h>

struct nand_device;

enum nand_operation_dir {
	NAND_OP_NONE,
	NAND_OP_INPUT,
	NAND_OP_OUTPUT,
};

struct nand_operation {
	u8 ncmds;
	u8 cmds[3];
	u8 naddrs[2];
	u8 addrs[2][5];
	bool wait_ready[3];
	int page[2];
	int column[2];
	enum nand_operation_dir dir;
	union {
		const u8 *out;
		u8 *in;
	} data[2];
	size_t len[2];
};

struct nand_controller_ops {
	int (*add)(struct nand_controller *controller,
		   struct nand_device *dev);
	int (*remove)(struct nand_controller *controller,
		      struct nand_device *dev);
	void (*select)(struct nand_device *dev, int chip);
	int (*launch)(struct nand_device *dev,
		      const struct nand_operation *op);
	bool (*supported)(struct nand_controller *controller,
			  const struct nand_operation *op);
};

struct nand_controller {
	struct device dev;
	struct mutex lock;
	struct nand_device *active;
	wait_queue_head_t wq;
	const struct nand_controller_ops *ops;
	struct nand_device **devices;
};

struct nand_basic_controller_ops {
	void (*write_buf)(struct nand_device *dev, const void *buf, int len);
	void (*read_buf)(struct nand_device *dev, void *buf, int len);
	void (*cmd_ctrl)(struct nand_device *dev, int dat, unsigned int ctrl);
	int (*waitfunc)(struct nand_device *dev);
};

struct nand_basic_controller {
	struct nand_controller base;
	struct nand_basic_controller_ops *ops;
};

enum nand_interface_type {
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

struct nand_interface {
	enum nand_interface_type type;
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

struct nand_device {
	struct mtd_info mtd;
	union {
		struct nand_sdr_timings sdr;
	} timings;

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

	struct nand_interface interface;

	struct nand_controller *controller;
	void *controller_priv;

	int read_retries;

	flstate_t state;

	struct nand_ecc ecc;

	struct nand_buffers *buffers;

	struct nand_bad_block bb;

	void *priv;
};

static struct nand_device *mtd_to_nand(struct mtd_info *mtd)
{
	return container_of(mtd, struct nand_device, mtd);
}
