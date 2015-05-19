/*
 * Copyright (C) 2015 Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>

#define to_clk_factor(_hw) container_of(_hw, struct clk_factor, hw)

static unsigned long __get_mult(struct clk_factor *factor,
				unsigned long rate,
				unsigned long parent_rate)
{
	if (factor->flags & CLK_FACTOR_ROUND_CLOSEST)
		return DIV_ROUND_CLOSEST(rate, parent_rate);

	return rate / parent_rate;
}

static unsigned long clk_factor_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct clk_factor *factor = to_clk_factor(hw);
	unsigned long val;
	
	val = clk_readl(factor->reg) >> factor->shift;
	val &= GENMASK(factor->width, 0);

	if (!val && factor->flags & CLK_FACTOR_ZERO_BYPASS)
		val = 1;
	
	return parent_rate * val;
}

static bool __is_best_rate(unsigned long rate, unsigned long new,
			   unsigned long best, unsigned long flags)
{
	if (flags & CLK_FACTOR_ROUND_CLOSEST)
		return abs(rate - new) < abs(rate - best);

	return new >= rate && new < best;
}

static unsigned long clk_factor_bestmult(struct clk_hw *hw, unsigned long rate,
					 unsigned long *best_parent_rate,
					 u8 width, unsigned long flags)
{
	unsigned long orig_parent_rate = *best_parent_rate;
	unsigned long parent_rate, current_rate, best_rate = ~0;
	unsigned int i, bestmult = 0;

	if (!(__clk_get_flags(hw->clk) & CLK_SET_RATE_PARENT))
		return rate / *best_parent_rate;

	for (i = 1; i < ((1 << width) - 1); i++) {
		if (rate * i == orig_parent_rate) {
			/*
			 * This is the best case for us if we have a
			 * perfect match without changing the parent
			 * rate.
			 */
			*best_parent_rate = orig_parent_rate;
			return i;
		}

		parent_rate = clk_hw_round_rate(clk_hw_get_parent(hw),
						rate / i);
		current_rate = parent_rate * i;

		if (__is_best_rate(rate, current_rate, best_rate, flags)) {
			bestmult = i;
			best_rate = current_rate;
			*best_parent_rate = parent_rate;
		}
	}

	return bestmult;
}

static long clk_factor_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *parent_rate)
{
	struct clk_factor *factor = to_clk_factor(hw);
	unsigned long mult = clk_factor_bestmult(hw, rate, parent_rate,
						 factor->width, factor->flags);

	return *parent_rate * mult;
}

static int clk_factor_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk_factor *factor = to_clk_factor(hw);
	unsigned long mult = __get_mult(factor, rate, parent_rate);
	unsigned long uninitialized_var(flags);
	unsigned long val;

	if (factor->lock)
		spin_lock_irqsave(factor->lock, flags);

	val = clk_readl(factor->reg);
	val &= ~GENMASK(factor->width + factor->shift, factor->shift);
	val |= mult << factor->shift;
	clk_writel(val, factor->reg);

	if (factor->lock)
		spin_unlock_irqrestore(factor->lock, flags);

	return 0;
}

const struct clk_ops clk_factor_ops = {
	.recalc_rate	= clk_factor_recalc_rate,
	.round_rate	= clk_factor_round_rate,
	.set_rate	= clk_factor_set_rate,
};
EXPORT_SYMBOL_GPL(clk_factor_ops);

struct clk *clk_register_factor(struct device *dev, const char *name,
				const char *parent_name, unsigned long flags,
				void __iomem *reg, u8 shift, u8 width,
				u8 clk_factor_flags, spinlock_t *lock)
{
	struct clk_init_data init;
	struct clk_factor *factor;
	struct clk *clk;

	factor = kmalloc(sizeof(*factor), GFP_KERNEL);
	if (!factor) {
		pr_err("%s: could not allocate fixed factor clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &clk_factor_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	factor->reg = reg;
	factor->shift = shift;
	factor->width = width;
	factor->flags = clk_factor_flags;
	factor->lock = lock;
	factor->hw.init = &init;
	
	clk = clk_register(dev, &factor->hw);
	if (IS_ERR(clk))
		kfree(factor);

	return clk;
}
EXPORT_SYMBOL_GPL(clk_register_factor);

void clk_unregister_factor(struct clk *clk)
{
	struct clk_factor *factor;
	struct clk_hw *hw;

	hw = __clk_get_hw(clk);
	if (!hw)
		return;

	factor = to_clk_factor(hw);

	clk_unregister(clk);
	kfree(factor);
}
EXPORT_SYMBOL_GPL(clk_unregister_factor);
