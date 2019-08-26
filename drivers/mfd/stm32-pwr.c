// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) STMicroelectronics 2017 - All Rights Reserved
 * Author: Pascal Paillet <p.paillet@st.com> for STMicroelectronics.
 */

#include <linux/arm-smccc.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <asm/exception.h>

#define NB_WAKEUPPINS 6

#define STM32_SVC_PWR 0x82001001
#define STM32_WRITE 0x1
#define STM32_SET_BITS 0x2
#define STM32_CLEAR_BITS 0x3

#define SMC_PWR_BASE 0x50001000
// PWR Registers
#define WKUPCR 0x20
#define WKUPFR 0x24
#define MPUWKUPENR 0x28

#define WKUP_FLAGS_MASK GENMASK(5, 0)

// WKUPCR bits definition
#define WKUP_EDGE_SHIFT 8
#define WKUP_PULL_SHIFT 16
#define WKUP_PULL_MASK GENMASK(1, 0)

enum wkup_pull_setting {
	WKUP_NO_PULL = 0,
	WKUP_PULL_UP,
	WKUP_PULL_DOWN,
	WKUP_PULL_RESERVED
};

#define SMC(class, op, offset, val)				\
{								\
	struct arm_smccc_res res;				\
	arm_smccc_smc(class, op, SMC_PWR_BASE + offset, val,	\
		      0, 0, 0, 0, &res);			\
}								\

struct stm32_pwr_data {
	void __iomem *base;		/* IO Memory base address */
	struct irq_domain *domain;	/* Domain for this controller */
	int irq;			/* Parent interrupt */
};

static void stm32_pwr_irq_ack(struct irq_data *d)
{
	pr_debug("irq:%lu\n", d->hwirq);
	SMC(STM32_SVC_PWR, STM32_SET_BITS, WKUPCR, BIT(d->hwirq));
}

static void stm32_pwr_irq_mask(struct irq_data *d)
{
	pr_debug("irq:%lu\n", d->hwirq);
	SMC(STM32_SVC_PWR, STM32_CLEAR_BITS, MPUWKUPENR, BIT(d->hwirq));
}

static void stm32_pwr_irq_unmask(struct irq_data *d)
{
	pr_debug("irq:%lu\n", d->hwirq);
	SMC(STM32_SVC_PWR, STM32_SET_BITS, MPUWKUPENR, BIT(d->hwirq));
}

static int stm32_pwr_irq_set_type(struct irq_data *d, unsigned int flow_type)
{
	struct stm32_pwr_data *priv = d->domain->host_data;
	int pin_id = d->hwirq;
	u32 wkupcr;
	int en;

	pr_debug("irq:%lu\n", d->hwirq);

	en = readl_relaxed(priv->base + MPUWKUPENR) & BIT(pin_id);
	/* reference manual request to disable the wakeup pin while
	 * changing the edge detection setting
	 */
	if (en)
		stm32_pwr_irq_mask(d);

	wkupcr = readl_relaxed(priv->base + WKUPCR);
	switch (flow_type & IRQ_TYPE_SENSE_MASK) {
	case IRQF_TRIGGER_FALLING:
		wkupcr |= (1 << (WKUP_EDGE_SHIFT + pin_id));
		break;
	case IRQF_TRIGGER_RISING:
		wkupcr &= ~(1 << (WKUP_EDGE_SHIFT + pin_id));
		break;
	default:
		return -EINVAL;
	}

	SMC(STM32_SVC_PWR, STM32_WRITE, WKUPCR, wkupcr);

	if (en)
		stm32_pwr_irq_unmask(d);

	return 0;
}

#ifdef CONFIG_SMP
int stm32_pwr_set_affinity_parent(struct irq_data *data,
				  const struct cpumask *dest, bool force)
{
	struct stm32_pwr_data *priv = data->domain->host_data;
	struct irq_data *parent = irq_get_irq_data(priv->irq);

	if (parent->chip && parent->chip->irq_set_affinity)
		return parent->chip->irq_set_affinity(parent, dest, force);

	return IRQ_SET_MASK_OK_DONE;
}
#endif

static struct irq_chip stm32_pwr_irq_chip = {
	.name = "stm32-pwr-irq",
	.irq_ack = stm32_pwr_irq_ack,
	.irq_mask = stm32_pwr_irq_mask,
	.irq_unmask = stm32_pwr_irq_unmask,
	.irq_set_type = stm32_pwr_irq_set_type,
#ifdef CONFIG_SMP
	.irq_set_affinity = stm32_pwr_set_affinity_parent,
#endif
};

static int stm32_pwr_irq_set_pull_config(struct irq_domain *d, int pin_id,
					 enum wkup_pull_setting config)
{
	struct stm32_pwr_data *priv = d->host_data;
	u32 wkupcr;

	pr_debug("irq:%d pull config:0x%x\n", pin_id, config);

	if (config >= WKUP_PULL_RESERVED) {
		pr_err("%s: bad irq pull config\n", __func__);
		return -EINVAL;
	}

	wkupcr = readl_relaxed(priv->base + WKUPCR);
	wkupcr &= ~((WKUP_PULL_MASK) << (WKUP_PULL_SHIFT + pin_id * 2));
	wkupcr |= (config & WKUP_PULL_MASK) << (WKUP_PULL_SHIFT + pin_id * 2);

	SMC(STM32_SVC_PWR, STM32_WRITE, WKUPCR, wkupcr);

	return 0;
}

static int stm32_pwr_xlate(struct irq_domain *d, struct device_node *ctrlr,
			   const u32 *intspec, unsigned int intsize,
			   irq_hw_number_t *out_hwirq, unsigned int *out_type)
{
	if (WARN_ON(intsize < 3)) {
		pr_err("%s: bad irq config parameters\n", __func__);
		return -EINVAL;
	}

	*out_hwirq = intspec[0];
	*out_type = intspec[1] & (IRQ_TYPE_SENSE_MASK);

	return stm32_pwr_irq_set_pull_config(d, intspec[0], intspec[2]);
}

static int stm32_pwr_alloc(struct irq_domain *d, unsigned int virq,
			   unsigned int nr_irqs, void *data)
{
	struct irq_fwspec *fwspec = data;
	irq_hw_number_t hwirq;

	hwirq = fwspec->param[0];
	irq_domain_set_info(d, virq, hwirq, &stm32_pwr_irq_chip, d->host_data,
			    handle_edge_irq, NULL, NULL);

	return 0;
}

static const struct irq_domain_ops stm32_pwr_irq_domain_ops = {
	.alloc = stm32_pwr_alloc,
	.xlate = stm32_pwr_xlate,
	.free = irq_domain_free_irqs_common,
};

/*
 * Handler for the cascaded IRQ.
 */
static void stm32_pwr_handle_irq(struct irq_desc *desc)
{
	struct stm32_pwr_data  *priv = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 wkupfr, wkupenr, i;

	chained_irq_enter(chip, desc);

	wkupfr = readl_relaxed(priv->base + WKUPFR);
	wkupenr = readl_relaxed(priv->base + MPUWKUPENR);

	for (i = 0; i < NB_WAKEUPPINS; i++) {
		if ((wkupfr & BIT(i)) && (wkupenr & BIT(i))) {
			pr_debug("handle wkup irq:%d\n", i);
			generic_handle_irq(irq_find_mapping(priv->domain, i));
		}
	}

	chained_irq_exit(chip, desc);
}

static int __init stm32_pwr_init(struct device_node *np,
				 struct device_node *parent)
{
	struct stm32_pwr_data *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = of_iomap(np, 0);
	if (IS_ERR(priv->base)) {
		pr_err("%s: Unable to map IO memory\n", __func__);
		ret = PTR_ERR(priv->base);
		goto out_free;
	}

	/* Disable all wake-up pins */
	SMC(STM32_SVC_PWR, STM32_WRITE, MPUWKUPENR, 0);
	/* Clear all interrupts flags */
	SMC(STM32_SVC_PWR, STM32_SET_BITS, WKUPCR, WKUP_FLAGS_MASK);

	priv->domain = irq_domain_add_linear(np, NB_WAKEUPPINS,
					     &stm32_pwr_irq_domain_ops, priv);
	if (!priv->domain) {
		pr_err("%s: Unable to add irq domain!\n", __func__);
		ret = -ENOMEM;
		goto out_unmap;
	}

	priv->irq = irq_of_parse_and_map(np, 0);
	if (priv->irq < 0) {
		pr_err("failed to get PWR IRQ\n");
		ret = priv->irq;
		goto out_domain;
	}

	irq_set_chained_handler_and_data(priv->irq,
					 stm32_pwr_handle_irq, priv);

	of_node_clear_flag(np, OF_POPULATED);

	return 0;

out_domain:
	irq_domain_remove(priv->domain);
out_unmap:
	iounmap(priv->base);
out_free:
	kfree(priv);
	return ret;
}

static int __init stm32_pwr_of_init(struct device_node *np,
				    struct device_node *parent)
{
	return stm32_pwr_init(np, parent);
}

IRQCHIP_DECLARE(stm32mp1_pwr_irq, "st,stm32mp1-pwr", stm32_pwr_of_init);
