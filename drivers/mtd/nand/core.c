#include <linux/mtd/nand.h>

static int nand_do_read_ops(struct mtd_info *mtd, loff_t from,
			    struct mtd_oob_ops *ops)
{
	int chipnr, page, realpage, col, bytes, aligned, oob_required;
	struct nand_chip *chip = mtd->priv;
	int ret = 0;
	uint32_t readlen = ops->len;
	uint32_t oobreadlen = ops->ooblen;
	uint32_t max_oobsize = ops->mode == MTD_OPS_AUTO_OOB ?
		mtd->oobavail : mtd->oobsize;

	uint8_t *bufpoi, *oob, *buf;
	int use_bufpoi;
	unsigned int max_bitflips = 0;
	int retry_mode = 0;
	bool ecc_fail = false;

	chipnr = (int)(from >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	realpage = (int)(from >> chip->page_shift);
	page = realpage & chip->pagemask;

	col = (int)(from & (mtd->writesize - 1));

	buf = ops->datbuf;
	oob = ops->oobbuf;
	oob_required = oob ? 1 : 0;

	while (1) {
		unsigned int ecc_failures = mtd->ecc_stats.failed;

		bytes = min(mtd->writesize - col, readlen);
		aligned = (bytes == mtd->writesize);

		if (!aligned)
			use_bufpoi = 1;
		else if (chip->options & NAND_USE_BOUNCE_BUFFER)
			use_bufpoi = !virt_addr_valid(buf);
		else
			use_bufpoi = 0;

		/* Is the current page in the buffer? */
		if (realpage != chip->pagebuf || oob) {
			bufpoi = use_bufpoi ? chip->buffers->databuf : buf;

			if (use_bufpoi && aligned)
				pr_debug("%s: using read bounce buffer for buf@%p\n",
						 __func__, buf);

read_retry:
			nand_readpage_op(mtd, page, 0, NULL, 0);

			/*
			 * Now read the page into the buffer.  Absent an error,
			 * the read methods return max bitflips per ecc step.
			 */
			if (unlikely(ops->mode == MTD_OPS_RAW))
				ret = chip->ecc.read_page_raw(mtd, chip, bufpoi,
							      oob_required,
							      page);
			else if (!aligned && NAND_HAS_SUBPAGE_READ(chip) &&
				 !oob)
				ret = chip->ecc.read_subpage(mtd, chip,
							col, bytes, bufpoi,
							page);
			else
				ret = chip->ecc.read_page(mtd, chip, bufpoi,
							  oob_required, page);
			if (ret < 0) {
				if (use_bufpoi)
					/* Invalidate page cache */
					chip->pagebuf = -1;
				break;
			}

			max_bitflips = max_t(unsigned int, max_bitflips, ret);

			/* Transfer not aligned data */
			if (use_bufpoi) {
				if (!NAND_HAS_SUBPAGE_READ(chip) && !oob &&
				    !(mtd->ecc_stats.failed - ecc_failures) &&
				    (ops->mode != MTD_OPS_RAW)) {
					chip->pagebuf = realpage;
					chip->pagebuf_bitflips = ret;
				} else {
					/* Invalidate page cache */
					chip->pagebuf = -1;
				}
				memcpy(buf, chip->buffers->databuf + col, bytes);
			}

			if (unlikely(oob)) {
				int toread = min(oobreadlen, max_oobsize);

				if (toread) {
					oob = nand_transfer_oob(chip,
						oob, ops, toread);
					oobreadlen -= toread;
				}
			}

			if (chip->options & NAND_NEED_READRDY) {
				/* Apply delay or wait for ready/busy pin */
				if (!chip->dev_ready)
					udelay(chip->chip_delay);
				else
					nand_wait_ready(mtd);
			}

			if (mtd->ecc_stats.failed - ecc_failures) {
				if (retry_mode + 1 < chip->read_retries) {
					retry_mode++;
					ret = nand_setup_read_retry(mtd,
							retry_mode);
					if (ret < 0)
						break;

					/* Reset failures; retry */
					mtd->ecc_stats.failed = ecc_failures;
					goto read_retry;
				} else {
					/* No more retry modes; real failure */
					ecc_fail = true;
				}
			}

			buf += bytes;
		} else {
			memcpy(buf, chip->buffers->databuf + col, bytes);
			buf += bytes;
			max_bitflips = max_t(unsigned int, max_bitflips,
					     chip->pagebuf_bitflips);
		}

		readlen -= bytes;

		/* Reset to retry mode 0 */
		if (retry_mode) {
			ret = nand_setup_read_retry(mtd, 0);
			if (ret < 0)
				break;
			retry_mode = 0;
		}

		if (!readlen)
			break;

		/* For subsequent reads align to page boundary */
		col = 0;
		/* Increment page address */
		realpage++;

		page = realpage & chip->pagemask;
		/* Check, if we cross a chip boundary */
		if (!page) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}
	}
	chip->select_chip(mtd, -1);

	ops->retlen = ops->len - (size_t) readlen;
	if (oob)
		ops->oobretlen = ops->ooblen - oobreadlen;

	if (ret < 0)
		return ret;

	if (ecc_fail)
		return -EBADMSG;

	return max_bitflips;
}
