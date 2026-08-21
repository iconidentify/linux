// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMGR device power state driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/reset-controller.h>
#include <linux/module.h>

#define APPLE_PMGR_RESET        BIT(31)
#define APPLE_PMGR_AUTO_ENABLE  BIT(28)
#define APPLE_PMGR_PS_AUTO      GENMASK(27, 24)
#define APPLE_PMGR_PS_MIN       GENMASK(19, 16)
#define APPLE_PMGR_PS_RESET     BIT(12)
#define APPLE_PMGR_BUSY         BIT(11)
#define APPLE_PMGR_DEV_DISABLE  BIT(10)
#define APPLE_PMGR_WAS_CLKGATED BIT(9)
#define APPLE_PMGR_WAS_PWRGATED BIT(8)
#define APPLE_PMGR_PS_ACTUAL    GENMASK(7, 4)
#define APPLE_PMGR_PS_TARGET    GENMASK(3, 0)

#define APPLE_PMGR_FLAGS        (APPLE_PMGR_WAS_CLKGATED | APPLE_PMGR_WAS_PWRGATED)

#define APPLE_PMGR_PS_ACTIVE    0xf
#define APPLE_PMGR_PS_CLKGATE   0x4
#define APPLE_PMGR_PS_PWRGATE   0x0

#define APPLE_PMGR_PS_SET_TIMEOUT 100
#define APPLE_PMGR_RESET_TIME 1

struct apple_pmgr_ps {
	struct device *dev;
	struct generic_pm_domain genpd;
	struct reset_controller_dev rcdev;
	struct regmap *regmap;
	u32 offset;
	u32 min_state;
	bool force_disable;
	bool force_reset;
	bool externally_clocked;
};

/*
 * Diagnostics for newer Apple SoCs whose iBoot/firmware ownership contract is
 * not yet understood.  These leave every provider and its initial state
 * intact, but can suppress the otherwise unconditional probe-time AUTO_ENABLE
 * write on domains that firmware already left active.  Runtime genpd
 * transitions keep their existing behaviour.  Bits 0-3 select the four
 * T8140 display domains implicated by measured delayed faults, bit 4 selects
 * active Main PMGR domains outside the complete ANS/APCIe hierarchy, and bit
 * 5 selects active Nub PMGR domains.  Bits 6-11 select each member of the
 * T8140 ANS/APCIe hierarchy individually.  Bits 12-13 select the two observed
 * Nub SMC I2C domains individually.  Bit assignments are local to this
 * diagnostic and default to no behavioural change.
 */
static bool apple_pmgr_no_auto_enable_on_probe;
static unsigned int apple_pmgr_preserve_auto_probe_mask;
static bool apple_pmgr_trace_writes;
static unsigned int apple_pmgr_suppress_write_mask;

#define APPLE_PMGR_T8140_DISP_SYS       BIT(0)
#define APPLE_PMGR_T8140_DISPEXT0_SYS   BIT(1)
#define APPLE_PMGR_T8140_DISP_FE        BIT(2)
#define APPLE_PMGR_T8140_DISPEXT0_FE    BIT(3)
#define APPLE_PMGR_T8140_MAIN_NON_NVME  BIT(4)
#define APPLE_PMGR_T8140_NUB_ALL        BIT(5)
#define APPLE_PMGR_T8140_APCIE_GP       BIT(6)
#define APPLE_PMGR_T8140_ANS            BIT(7)
#define APPLE_PMGR_T8140_APCIE_SYS_GP   BIT(8)
#define APPLE_PMGR_T8140_APCIE_ST       BIT(9)
#define APPLE_PMGR_T8140_APCIE_SYS_ST   BIT(10)
#define APPLE_PMGR_T8140_APCIE_PHY_SW   BIT(11)
#define APPLE_PMGR_T8140_NUB_SMC_I2CM1  BIT(12)
#define APPLE_PMGR_T8140_NUB_SMC_I2CM2  BIT(13)
#define APPLE_PMGR_T8140_NVME_MASK      GENMASK(11, 6)
#define APPLE_PMGR_T8140_DIAGNOSTIC_MASK GENMASK(13, 0)

static int __init apple_pmgr_no_auto_probe_param(char *value)
{
	return kstrtobool(value, &apple_pmgr_no_auto_enable_on_probe);
}
early_param("apple_pmgr.no_auto_probe", apple_pmgr_no_auto_probe_param);

static int __init apple_pmgr_preserve_auto_probe_mask_param(char *value)
{
	unsigned int mask;
	int ret;

	ret = kstrtouint(value, 0, &mask);
	if (ret)
		return ret;
	if (mask & ~APPLE_PMGR_T8140_DIAGNOSTIC_MASK)
		return -EINVAL;

	apple_pmgr_preserve_auto_probe_mask = mask;
	return 0;
}
early_param("apple_pmgr.preserve_auto_probe_mask",
	    apple_pmgr_preserve_auto_probe_mask_param);

static int __init apple_pmgr_trace_writes_param(char *value)
{
	return kstrtobool(value, &apple_pmgr_trace_writes);
}
early_param("apple_pmgr.trace_writes", apple_pmgr_trace_writes_param);

static int __init apple_pmgr_suppress_write_mask_param(char *value)
{
	unsigned int mask;
	int ret;

	ret = kstrtouint(value, 0, &mask);
	if (ret)
		return ret;
	if (mask & ~APPLE_PMGR_T8140_DIAGNOSTIC_MASK)
		return -EINVAL;

	apple_pmgr_suppress_write_mask = mask;
	return 0;
}
early_param("apple_pmgr.suppress_write_mask",
	    apple_pmgr_suppress_write_mask_param);

#define genpd_to_apple_pmgr_ps(_genpd) container_of(_genpd, struct apple_pmgr_ps, genpd)
#define rcdev_to_apple_pmgr_ps(_rcdev) container_of(_rcdev, struct apple_pmgr_ps, rcdev)

static unsigned int apple_pmgr_t8140_diagnostic_bit(struct apple_pmgr_ps *ps)
{
	struct resource parent;

	if (!of_device_is_compatible(ps->dev->of_node,
				     "apple,t8140-pmgr-pwrstate"))
		return 0;

	switch (ps->offset) {
	case 0x280:
		return APPLE_PMGR_T8140_DISP_SYS;
	case 0x288:
		return APPLE_PMGR_T8140_DISPEXT0_SYS;
	case 0x318:
		return APPLE_PMGR_T8140_DISP_FE;
	case 0x320:
		return APPLE_PMGR_T8140_DISPEXT0_FE;
	case 0x258:
		return APPLE_PMGR_T8140_APCIE_GP;
	case 0x278:
		return APPLE_PMGR_T8140_ANS;
	case 0x2a0:
		return APPLE_PMGR_T8140_APCIE_SYS_GP;
	case 0x310:
		return APPLE_PMGR_T8140_APCIE_ST;
	case 0x3a0:
		return APPLE_PMGR_T8140_APCIE_SYS_ST;
	case 0x3c0:
		return APPLE_PMGR_T8140_APCIE_PHY_SW;
	default:
		break;
	}

	if (of_address_to_resource(ps->dev->of_node->parent, 0, &parent))
		return 0;
	if (parent.start == 0x308280000ULL && ps->offset == 0x8030)
		return APPLE_PMGR_T8140_NUB_ALL |
		       APPLE_PMGR_T8140_NUB_SMC_I2CM1;
	if (parent.start == 0x308280000ULL && ps->offset == 0x8038)
		return APPLE_PMGR_T8140_NUB_ALL |
		       APPLE_PMGR_T8140_NUB_SMC_I2CM2;
	if (parent.start == 0x308280000ULL)
		return APPLE_PMGR_T8140_NUB_ALL;
	if (parent.start == 0x300700000ULL)
		return APPLE_PMGR_T8140_MAIN_NON_NVME;

	return 0;
}

static int apple_pmgr_write(struct apple_pmgr_ps *ps, u32 value,
			    const char *reason)
{
	unsigned int bit = apple_pmgr_t8140_diagnostic_bit(ps);

	if (apple_pmgr_suppress_write_mask & bit) {
		dev_info(ps->dev,
			 "diagnostic: PMGR write suppressed reason=%s offset=0x%x value=0x%08x mask=0x%x\n",
			 reason, ps->offset, value,
			 apple_pmgr_suppress_write_mask);
		return 0;
	}

	if (apple_pmgr_trace_writes)
		dev_info(ps->dev,
			 "diagnostic: PMGR write reason=%s offset=0x%x value=0x%08x\n",
			 reason, ps->offset, value);

	return regmap_write(ps->regmap, ps->offset, value);
}

static int apple_pmgr_update_bits(struct apple_pmgr_ps *ps, u32 mask,
				  u32 value, const char *reason)
{
	unsigned int bit = apple_pmgr_t8140_diagnostic_bit(ps);

	if (apple_pmgr_suppress_write_mask & bit) {
		dev_info(ps->dev,
			 "diagnostic: PMGR update suppressed reason=%s offset=0x%x mask=0x%08x value=0x%08x suppress_mask=0x%x\n",
			 reason, ps->offset, mask, value,
			 apple_pmgr_suppress_write_mask);
		return 0;
	}

	if (apple_pmgr_trace_writes)
		dev_info(ps->dev,
			 "diagnostic: PMGR update reason=%s offset=0x%x mask=0x%08x value=0x%08x\n",
			 reason, ps->offset, mask, value);

	return regmap_update_bits(ps->regmap, ps->offset, mask, value);
}

static int apple_pmgr_ps_set(struct generic_pm_domain *genpd, u32 pstate, bool auto_enable)
{
	int ret;
	struct apple_pmgr_ps *ps = genpd_to_apple_pmgr_ps(genpd);
	u32 reg, cur;

	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret < 0)
		return ret;

	/* Resets are synchronous, and only work if the device is powered and clocked. */
	if (reg & APPLE_PMGR_RESET && pstate != APPLE_PMGR_PS_ACTIVE)
		dev_err(ps->dev, "PS %s: powering off with RESET active\n",
			genpd->name);

	if (pstate != APPLE_PMGR_PS_ACTIVE && (ps->force_disable || ps->force_reset)) {
		u32 reg_pre = reg & ~(APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS);

		if (ps->force_disable)
			reg_pre |= APPLE_PMGR_DEV_DISABLE;
		if (ps->force_reset)
			reg_pre |= APPLE_PMGR_PS_RESET;

		apple_pmgr_write(ps, reg_pre, "set-pre-disable-reset");

		ret = regmap_read_poll_timeout_atomic(
			ps->regmap, ps->offset, cur,
			(cur & (APPLE_PMGR_DEV_DISABLE | APPLE_PMGR_PS_RESET)) ==
			(reg_pre & (APPLE_PMGR_DEV_DISABLE | APPLE_PMGR_PS_RESET)), 1,
			APPLE_PMGR_PS_SET_TIMEOUT);

		if (ret < 0)
			dev_err(ps->dev, "PS %s: Failed to set reset/disable bits (now: 0x%x)\n",
				genpd->name, reg);
	}

	reg &= ~(APPLE_PMGR_DEV_DISABLE | APPLE_PMGR_PS_RESET |
		 APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS | APPLE_PMGR_PS_TARGET);
	reg |= FIELD_PREP(APPLE_PMGR_PS_TARGET, pstate);

	dev_dbg(ps->dev, "PS %s: pwrstate = 0x%x: 0x%x\n", genpd->name, pstate, reg);

	apple_pmgr_write(ps, reg, "set-target");

	if (ps->externally_clocked && pstate == APPLE_PMGR_PS_ACTIVE) {
		/*
		 * If this clock domain requires an external clock, then
		 * consider the "clock gated" state to be good enough.
		 */
		ret = regmap_read_poll_timeout_atomic(
			ps->regmap, ps->offset, cur,
			FIELD_GET(APPLE_PMGR_PS_ACTUAL, cur) >= APPLE_PMGR_PS_CLKGATE, 1,
			APPLE_PMGR_PS_SET_TIMEOUT);
	} else {
		ret = regmap_read_poll_timeout_atomic(
			ps->regmap, ps->offset, cur,
			FIELD_GET(APPLE_PMGR_PS_ACTUAL, cur) == pstate, 1,
			APPLE_PMGR_PS_SET_TIMEOUT);
	}

	if (ret < 0)
		dev_err(ps->dev, "PS %s: Failed to reach power state 0x%x (now: 0x%x)\n",
			genpd->name, pstate, reg);

	if (auto_enable) {
		/* Not all devices implement this; this is a no-op where not implemented. */
		reg |= APPLE_PMGR_AUTO_ENABLE;
		apple_pmgr_write(ps, reg, "set-auto-enable");
	}

	return ret;
}

static bool apple_pmgr_ps_is_active(struct apple_pmgr_ps *ps)
{
	u32 reg = 0;

	regmap_read(ps->regmap, ps->offset, &reg);
	/*
	 * We consider domains as active if they are actually on, or if they have auto-PM
	 * enabled and the intended target is on.
	 */
	return (FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == APPLE_PMGR_PS_ACTIVE ||
		(FIELD_GET(APPLE_PMGR_PS_TARGET, reg) == APPLE_PMGR_PS_ACTIVE &&
		 reg & APPLE_PMGR_AUTO_ENABLE));
}

static bool apple_pmgr_preserve_auto_on_probe(struct apple_pmgr_ps *ps)
{
	if (apple_pmgr_no_auto_enable_on_probe)
		return true;

	return apple_pmgr_preserve_auto_probe_mask &
	       apple_pmgr_t8140_diagnostic_bit(ps);
}

static void apple_pmgr_log_t8140_nvme_probe_state(struct apple_pmgr_ps *ps)
{
	struct resource parent;
	u32 reg;

	if (!(apple_pmgr_preserve_auto_probe_mask &
	      APPLE_PMGR_T8140_NVME_MASK) ||
	    !of_device_is_compatible(ps->dev->of_node,
				     "apple,t8140-pmgr-pwrstate") ||
	    of_address_to_resource(ps->dev->of_node->parent, 0, &parent) ||
	    parent.start != 0x300700000ULL)
		return;

	switch (ps->offset) {
	case 0x258:
	case 0x278:
	case 0x2a0:
	case 0x310:
	case 0x3a0:
	case 0x3c0:
		break;
	default:
		return;
	}

	if (regmap_read(ps->regmap, ps->offset, &reg))
		return;

	dev_info(ps->dev,
		 "diagnostic: inherited NVMe PMGR state 0x%08x actual=0x%x target=0x%x auto=%u was_clk=%u was_pwr=%u\n",
		 reg, (unsigned int)FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg),
		 (unsigned int)FIELD_GET(APPLE_PMGR_PS_TARGET, reg),
		 !!(reg & APPLE_PMGR_AUTO_ENABLE),
		 !!(reg & APPLE_PMGR_WAS_CLKGATED),
		 !!(reg & APPLE_PMGR_WAS_PWRGATED));
}

static int apple_pmgr_ps_power_on(struct generic_pm_domain *genpd)
{
	return apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_ACTIVE, true);
}

static int apple_pmgr_ps_power_off(struct generic_pm_domain *genpd)
{
	return apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_PWRGATE, false);
}

static int apple_pmgr_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	unsigned long flags;

	spin_lock_irqsave(&ps->genpd.slock, flags);

	if (ps->genpd.status == GENPD_STATE_OFF)
		dev_err(ps->dev, "PS 0x%x: asserting RESET while powered down\n", ps->offset);

	dev_dbg(ps->dev, "PS 0x%x: assert reset\n", ps->offset);
	/* Quiesce device before asserting reset */
	apple_pmgr_update_bits(ps, APPLE_PMGR_FLAGS | APPLE_PMGR_DEV_DISABLE,
			       APPLE_PMGR_DEV_DISABLE, "reset-assert-disable");
	apple_pmgr_update_bits(ps, APPLE_PMGR_FLAGS | APPLE_PMGR_RESET,
			       APPLE_PMGR_RESET, "reset-assert");

	spin_unlock_irqrestore(&ps->genpd.slock, flags);

	return 0;
}

static int apple_pmgr_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	unsigned long flags;

	spin_lock_irqsave(&ps->genpd.slock, flags);

	dev_dbg(ps->dev, "PS 0x%x: deassert reset\n", ps->offset);
	apple_pmgr_update_bits(ps, APPLE_PMGR_FLAGS | APPLE_PMGR_RESET, 0,
			       "reset-deassert");
	apple_pmgr_update_bits(ps, APPLE_PMGR_FLAGS | APPLE_PMGR_DEV_DISABLE, 0,
			       "reset-deassert-enable");

	if (ps->genpd.status == GENPD_STATE_OFF)
		dev_err(ps->dev, "PS 0x%x: RESET was deasserted while powered down\n", ps->offset);

	spin_unlock_irqrestore(&ps->genpd.slock, flags);

	return 0;
}

static int apple_pmgr_reset_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	int ret;

	ret = apple_pmgr_reset_assert(rcdev, id);
	if (ret)
		return ret;

	usleep_range(APPLE_PMGR_RESET_TIME, 2 * APPLE_PMGR_RESET_TIME);

	return apple_pmgr_reset_deassert(rcdev, id);
}

static int apple_pmgr_reset_status(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	u32 reg = 0;

	regmap_read(ps->regmap, ps->offset, &reg);

	return !!(reg & APPLE_PMGR_RESET);
}

static const struct reset_control_ops apple_pmgr_reset_ops = {
	.assert		= apple_pmgr_reset_assert,
	.deassert	= apple_pmgr_reset_deassert,
	.reset		= apple_pmgr_reset_reset,
	.status		= apple_pmgr_reset_status,
};

static int apple_pmgr_reset_xlate(struct reset_controller_dev *rcdev,
				  const struct of_phandle_args *reset_spec)
{
	return 0;
}

static int apple_pmgr_ps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct apple_pmgr_ps *ps;
	struct regmap *regmap;
	struct of_phandle_iterator it;
	int ret;
	const char *name;
	bool active;

	regmap = syscon_node_to_regmap(node->parent);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ps = devm_kzalloc(dev, sizeof(*ps), GFP_KERNEL);
	if (!ps)
		return -ENOMEM;

	ps->dev = dev;
	ps->regmap = regmap;

	ret = of_property_read_string(node, "label", &name);
	if (ret < 0) {
		dev_err(dev, "missing label property\n");
		return ret;
	}

	ret = of_property_read_u32(node, "reg", &ps->offset);
	if (ret < 0) {
		dev_err(dev, "missing reg property\n");
		return ret;
	}

	ps->genpd.flags |= GENPD_FLAG_IRQ_SAFE;
	ps->genpd.name = name;
	ps->genpd.power_on = apple_pmgr_ps_power_on;
	ps->genpd.power_off = apple_pmgr_ps_power_off;

	ret = of_property_read_u32(node, "apple,min-state", &ps->min_state);
	if (ret == 0 && ps->min_state <= APPLE_PMGR_PS_ACTIVE)
		apple_pmgr_update_bits(ps, APPLE_PMGR_FLAGS | APPLE_PMGR_PS_MIN,
				       FIELD_PREP(APPLE_PMGR_PS_MIN, ps->min_state),
				       "probe-min-state");

	if (of_property_read_bool(node, "apple,force-disable"))
		ps->force_disable = true;

	if (of_property_read_bool(node, "apple,force-reset"))
		ps->force_reset = true;

	if (of_property_read_bool(node, "apple,externally-clocked"))
		ps->externally_clocked = true;

	active = apple_pmgr_ps_is_active(ps);
	apple_pmgr_log_t8140_nvme_probe_state(ps);
	if (of_property_read_bool(node, "apple,always-on")) {
		ps->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
		if (!active) {
			dev_warn(dev, "always-on domain %s is not on at boot\n", name);
			/* Turn it on so pm_genpd_init does not fail */
			active = apple_pmgr_ps_power_on(&ps->genpd) == 0;
		}
	} else if (active) {
		ps->genpd.flags |= GENPD_FLAG_DEFER_OFF | GENPD_FLAG_ACTIVE_WAKEUP;
	}

	/* Turn on auto-PM if the domain is already on. */
	if (active && !apple_pmgr_preserve_auto_on_probe(ps))
		apple_pmgr_update_bits(ps, APPLE_PMGR_FLAGS | APPLE_PMGR_AUTO_ENABLE,
				       APPLE_PMGR_AUTO_ENABLE,
				       "probe-auto-enable");
	else if (active)
		dev_info(dev,
			 "diagnostic: preserving firmware AUTO_ENABLE state for %s (mask=0x%x)\n",
			 name, apple_pmgr_preserve_auto_probe_mask);

	ret = pm_genpd_init(&ps->genpd, NULL, !active);
	if (ret < 0) {
		dev_err(dev, "pm_genpd_init failed\n");
		return ret;
	}

	ret = of_genpd_add_provider_simple(node, &ps->genpd);
	if (ret < 0) {
		dev_err(dev, "of_genpd_add_provider_simple failed\n");
		return ret;
	}

	of_for_each_phandle(&it, ret, node, "power-domains", "#power-domain-cells", -1) {
		struct of_phandle_args parent, child;

		parent.np = it.node;
		parent.args_count = of_phandle_iterator_args(&it, parent.args, MAX_PHANDLE_ARGS);
		child.np = node;
		child.args_count = 0;
		ret = of_genpd_add_subdomain(&parent, &child);

		if (ret == -EPROBE_DEFER) {
			of_node_put(parent.np);
			goto err_remove;
		} else if (ret < 0) {
			dev_err(dev, "failed to add to parent domain: %d (%s -> %s)\n",
				ret, it.node->name, node->name);
			of_node_put(parent.np);
			goto err_remove;
		}
	}

	/*
	 * Do not participate in regular PM; parent power domains are handled via the
	 * genpd hierarchy.
	 */
	pm_genpd_remove_device(dev);

	ps->rcdev.owner = THIS_MODULE;
	ps->rcdev.nr_resets = 1;
	ps->rcdev.ops = &apple_pmgr_reset_ops;
	ps->rcdev.of_node = dev->of_node;
	ps->rcdev.of_reset_n_cells = 0;
	ps->rcdev.of_xlate = apple_pmgr_reset_xlate;

	ret = devm_reset_controller_register(dev, &ps->rcdev);
	if (ret < 0)
		goto err_remove;

	return 0;
err_remove:
	of_genpd_del_provider(node);
	pm_genpd_remove(&ps->genpd);
	return ret;
}

static const struct of_device_id apple_pmgr_ps_of_match[] = {
	{ .compatible = "apple,t8103-pmgr-pwrstate" },
	{ .compatible = "apple,pmgr-pwrstate" },
	{}
};

MODULE_DEVICE_TABLE(of, apple_pmgr_ps_of_match);

static struct platform_driver apple_pmgr_ps_driver = {
	.probe = apple_pmgr_ps_probe,
	.driver = {
		.name = "apple-pmgr-pwrstate",
		.of_match_table = apple_pmgr_ps_of_match,
	},
};

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("PMGR power state driver for Apple SoCs");

module_platform_driver(apple_pmgr_ps_driver);
