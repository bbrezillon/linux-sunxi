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
	u32 len[2];
};

struct nand_controller_ops {
	void (*select)(struct nand_device *dev, int chip);
	int (*launch)(struct nand_device *dev,
		      const struct nand_operation *op);
};

struct nand_controller {
	struct mutex lock;
	struct nand_device *active;
	wait_queue_head_t wq;
	const struct nand_controller_ops *ops;
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
	int (*read_page_raw)(struct nand_device *dev, struct nand_chip *chip,
			     void *buf, int oob_required, int page);
	int (*write_page_raw)(struct nand_device *dev, struct nand_chip *chip,
			      const void *buf, int oob_required);
	int (*read_page)(struct nand_device *dev, struct nand_chip *chip,
			 void *buf, int oob_required, int page);
	int (*read_subpage)(struct nand_device *dev, struct nand_chip *chip,
			    uint32_t offs, uint32_t len, void *buf, int page);
	int (*write_subpage)(struct nand_device *dev, struct nand_chip *chip,
			     uint32_t offset, uint32_t data_len,
			     const void *data_buf, int oob_required);
	int (*write_page)(struct nand_device *dev, struct nand_chip *chip,
			  const void *buf, int oob_required);
	int (*write_oob_raw)(struct nand_device *dev, struct nand_chip *chip,
			     int page);
	int (*read_oob_raw)(struct nand_device *dev, struct nand_chip *chip,
			    int page);
	int (*read_oob)(struct nand_device *dev, struct nand_chip *chip,
			int page);
	int (*write_oob)(struct nand_device *dev, struct nand_chip *chip,
			 int page);
};

struct nand_ecc_controller {
	int steps;
	int size;
	int bytes;
	int total;
	int strength;
	int prepad;
	int postpad;
	struct nand_ecclayout *layout;
	struct nand_ecc_controller_ops *ops;
};

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
	u16 ecc_strength_ds;
	u16 ecc_step_ds;
	int onfi_timing_mode_default;

	struct nand_interface interface;

	struct nand_controller *controller;

	int read_retries;

	flstate_t state;

	struct nand_ecc_controller *ecc;
	struct nand_buffers *buffers;

	struct nand_bad_block bb;

	void *priv;
};

static struct nand_device *mtd_to_nand(struct mtd_info *mtd)
{
	return container_of(mtd, struct nand_device, mtd);
}
