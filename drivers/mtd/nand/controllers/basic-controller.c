#include <linux/mtd/nand2.h>

static inline struct nand_basic_controller *
to_basic_controller(struct nand_controller *controller)
{
	return container_of(controller, struct nand_basic_controller, base);
}

int nand_basic_launch(struct nand_device *dev, const struct nand_operation *op)
{
	struct nand_basic_controller *ctrl =
					to_basic_controller(dev->controller);
	int ret, i;

	if (!ctrl->ops->cmd_ctrl)
		return -ENOTSUPP;

	if (op->ncmds)
		ctrl->ops->cmd_ctrl(dev, op->cmds[0],
				    NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	for (i = 0; i < op->naddrs[0]; i++)
		ctrl->ops->cmd_ctrl(dev, op->addrs[0][i],
				    (i ? 0 : NAND_CTRL_CHANGE) | NAND_NCE |
				    NAND_ALE);

	ctrl->ops->cmd_ctrl(dev, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	if (op->dir == NAND_OP_OUTPUT && op->data[0].out)
		ctrl->ops->write_buf(dev, op->data[0].out, op->len[0]);

	if (op->wait_ready[0]) {
		ret = ctrl->ops->waitfunc(dev);
		if (ret)
			return ret;
	}

	if (op->ncmds > 1) {
		ctrl->ops->cmd_ctrl(dev, op->cmds[1],
				    NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		ctrl->ops->cmd_ctrl(dev, NAND_CMD_NONE,
				    NAND_NCE | NAND_CTRL_CHANGE);
	}

	if (op->wait_ready[1]) {
		ret = ctrl->ops->waitfunc(dev);
		if (ret)
			return ret;
	}

	if (op->dir == NAND_OP_INPUT && op->data[0].in)
		ctrl->ops->read_buf(dev, op->data[0].in, op->len[0]);

	if (op->ncmds < 3)
		return 0;

	for (i = 0; i < op->naddrs[1]; i++)
		ctrl->ops->cmd_ctrl(dev, op->addrs[1][i],
				    (i ? 0 : NAND_CTRL_CHANGE) |
				    NAND_NCE | NAND_ALE);

	ctrl->ops->cmd_ctrl(dev, op->cmds[2],
			    NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
	ctrl->ops->cmd_ctrl(dev, NAND_CMD_NONE,
			    NAND_NCE | NAND_CTRL_CHANGE);

	if (op->dir == NAND_OP_OUTPUT && op->data[1].out)
		ctrl->ops->write_buf(dev, op->data[1].out, op->len[1]);

	if (op->wait_ready[2]) {
		ret = ctrl->ops->waitfunc(dev);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(nand_basic_launch);

bool nand_basic_support(struct nand_basic_controller,
			const struct nand_operation *op)
{
	return true;
}
EXPORT_SYMBOL_GPL(nand_basic_support);
