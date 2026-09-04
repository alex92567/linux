// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PM8350B haptics driver
 *
 * Copyright (c) 2026, Oleksii Onchul <oleksiionchul@gmail.com>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define HAP_CFG_REVISION2_REG			0x01

#define HAP_CFG_EN_CTL_REG			0x46
#define HAPTICS_EN_BIT				BIT(7)

#define HAP_CFG_VMAX_REG			0x48
#define VMAX_STEP_UV				50000
#define DEFAULT_VMAX_UV				5000000
#define MAX_VMAX_UV				11000000

#define HAP_CFG_DRV_WF_SEL_REG			0x49
#define DRV_WF_FMT_BIT				BIT(4)
#define DRV_WF_SEL_MASK				GENMASK(1, 0)

#define HAP_CFG_SPMI_PLAY_REG			0x4c
#define PLAY_EN_BIT				BIT(7)
#define HAP_CFG_FAULT_CLR_REG			0x66
#define SC_CLR_BIT				BIT(2)
#define AUTO_RES_ERR_CLR_BIT			BIT(1)
#define HPWR_RDY_FAULT_CLR_BIT			BIT(0)

#define HAP_CFG_VSET_CFG_REG			0x68
#define FORCE_VREG_RDY_BIT			BIT(0)

#define HAP_CFG_CAL_EN_REG			0x72
#define CAL_RC_CLK_MASK				GENMASK(3, 2)
#define CAL_RC_CLK_SHIFT			2
#define CAL_RC_CLK_DISABLED_VAL			0
#define CAL_RC_CLK_AUTO_VAL			1

#define HAP_CFG_AUTORES_CFG_REG			0x63
#define AUTORES_EN_BIT				BIT(7)

#define HAP_CFG_TLRA_OL_HIGH_REG		0x5c
#define TLRA_OL_MSB_MASK			GENMASK(3, 0)
#define TLRA_OL_LSB_MASK			GENMASK(7, 0)
#define TLRA_STEP_US				5
#define TLRA_MAX_US				20475

#define HAP_PTN_REVISION2_REG			0x01
#define HAP_PTN_DIRECT_PLAY_REG			0x26
#define DIRECT_PLAY				1

#define HAP_BOOST_STATUS4_REG			0x0b
#define BOOST_DTEST1_STATUS_BIT			BIT(0)
#define HAP_BOOST_HW_CTRL_FOLLOW_REG		0x41
#define FOLLOW_HW_EN_BIT			BIT(7)
#define HAP_BOOST_VREG_EN_REG			0x46
#define VREG_EN_BIT				BIT(7)

#define PBS_ARG_REG				0x42
#define HAP_VREG_ON_VAL				0x1
#define HAP_VREG_OFF_VAL			0x2
#define PBS_TRIG_SET_REG			0xe5
#define PBS_TRIG_SET_VAL			0x1

#define HBOOST_OFF_DELAY_MS			2000
#define HBOOST_WAIT_READY_COUNT			100
#define HBOOST_WAIT_READY_INTERVAL_US		200

enum pm8350b_haptics_wf {
	PM8350B_HAP_WF_SQUARE,
	PM8350B_HAP_WF_SINE,
	PM8350B_HAP_WF_NO_MODULATION,
};

struct pm8350b_haptics {
	struct device *dev;
	struct input_dev *input;
	struct regmap *regmap;
	struct nvmem_device *pbs_nvmem;
	struct work_struct play_work;
	struct delayed_work hboost_off_work;
	struct mutex lock; /* serializes hardware access and playback state */
	u32 cfg_base;
	u32 ptn_base;
	u32 hboost_base;
	u32 vmax_uv;
	u32 lra_period_us;
	u8 speed;
	bool active;
	bool is_erm;
	bool hboost_enabled;
	bool hboost_open_loop;
};

static int pm8350b_haptics_write(struct pm8350b_haptics *hap, u32 base,
				 u32 offset, u8 val)
{
	return regmap_write(hap->regmap, base + offset, val);
}

static int pm8350b_haptics_update_bits(struct pm8350b_haptics *hap, u32 base,
				       u32 offset, u8 mask, u8 val)
{
	return regmap_update_bits(hap->regmap, base + offset, mask, val);
}

static int pm8350b_haptics_read(struct pm8350b_haptics *hap, u32 base,
				u32 offset, unsigned int *val)
{
	return regmap_read(hap->regmap, base + offset, val);
}

static int pm8350b_haptics_program_hboost(struct pm8350b_haptics *hap,
					  bool on)
{
	u8 val;
	int error;

	if (!hap->pbs_nvmem || hap->hboost_open_loop)
		return 0;

	val = on ? HAP_VREG_ON_VAL : HAP_VREG_OFF_VAL;
	error = nvmem_device_write(hap->pbs_nvmem, PBS_ARG_REG, 1, &val);
	if (error < 0)
		return error;

	val = PBS_TRIG_SET_VAL;
	error = nvmem_device_write(hap->pbs_nvmem, PBS_TRIG_SET_REG, 1, &val);
	if (error < 0)
		return error;

	hap->hboost_enabled = on;

	return 0;
}

static int pm8350b_haptics_set_hboost(struct pm8350b_haptics *hap, bool on)
{
	if (hap->hboost_enabled == on)
		return 0;

	return pm8350b_haptics_program_hboost(hap, on);
}

static int pm8350b_haptics_wait_hboost_ready(struct pm8350b_haptics *hap)
{
	unsigned int val;
	int error;
	int i;

	if (!hap->pbs_nvmem || hap->hboost_enabled)
		return 0;

	for (i = 0; i < HBOOST_WAIT_READY_COUNT; i++) {
		error = pm8350b_haptics_read(hap, hap->hboost_base,
					     HAP_BOOST_VREG_EN_REG, &val);
		if (error)
			return error;

		if (val & VREG_EN_BIT) {
			error = pm8350b_haptics_read(hap, hap->hboost_base,
						     HAP_BOOST_HW_CTRL_FOLLOW_REG,
						     &val);
			if (error)
				return error;

			/* HBoost is already available in open-loop mode. */
			if (!(val & FOLLOW_HW_EN_BIT)) {
				hap->hboost_open_loop = true;
				return 0;
			}
		} else {
			error = pm8350b_haptics_read(hap, hap->hboost_base,
						     HAP_BOOST_STATUS4_REG, &val);
			if (error)
				return error;

			if (!(val & BOOST_DTEST1_STATUS_BIT))
				return 0;
		}

		usleep_range(HBOOST_WAIT_READY_INTERVAL_US,
			     HBOOST_WAIT_READY_INTERVAL_US + 50);
	}

	return -EBUSY;
}

static int pm8350b_haptics_module_enable(struct pm8350b_haptics *hap, bool on)
{
	return pm8350b_haptics_write(hap, hap->cfg_base, HAP_CFG_EN_CTL_REG,
				     on ? HAPTICS_EN_BIT : 0);
}

static void pm8350b_haptics_disable_module(void *data)
{
	struct pm8350b_haptics *hap = data;
	int error;

	error = pm8350b_haptics_module_enable(hap, false);
	if (error)
		dev_warn(hap->dev, "failed to disable haptics module: %d\n",
			 error);
}

static int pm8350b_haptics_enable_autores(struct pm8350b_haptics *hap, bool on)
{
	return pm8350b_haptics_update_bits(hap, hap->cfg_base,
					   HAP_CFG_AUTORES_CFG_REG,
					   AUTORES_EN_BIT,
					   on ? AUTORES_EN_BIT : 0);
}

static int pm8350b_haptics_set_vmax(struct pm8350b_haptics *hap, u32 vmax_uv)
{
	if (vmax_uv < VMAX_STEP_UV || vmax_uv > MAX_VMAX_UV ||
	    vmax_uv % VMAX_STEP_UV)
		return -EINVAL;

	return pm8350b_haptics_write(hap, hap->cfg_base, HAP_CFG_VMAX_REG,
				     vmax_uv / VMAX_STEP_UV);
}

static int pm8350b_haptics_set_lra_period(struct pm8350b_haptics *hap,
					  u32 period_us)
{
	u16 val;
	int error;

	if (!period_us)
		return 0;

	if (period_us > TLRA_MAX_US)
		return -EINVAL;

	val = period_us / TLRA_STEP_US;
	error = pm8350b_haptics_write(hap, hap->cfg_base,
				      HAP_CFG_TLRA_OL_HIGH_REG,
				      (val >> 8) & TLRA_OL_MSB_MASK);
	if (error)
		return error;

	return pm8350b_haptics_write(hap, hap->cfg_base,
				     HAP_CFG_TLRA_OL_HIGH_REG + 1,
				     val & TLRA_OL_LSB_MASK);
}

static int pm8350b_haptics_toggle_rc_clk(struct pm8350b_haptics *hap)
{
	int error;

	error = pm8350b_haptics_update_bits(hap, hap->cfg_base,
					    HAP_CFG_CAL_EN_REG,
					    CAL_RC_CLK_MASK,
					    CAL_RC_CLK_DISABLED_VAL <<
					    CAL_RC_CLK_SHIFT);
	if (error)
		return error;

	return pm8350b_haptics_update_bits(hap, hap->cfg_base,
					   HAP_CFG_CAL_EN_REG,
					   CAL_RC_CLK_MASK,
					   CAL_RC_CLK_AUTO_VAL <<
					   CAL_RC_CLK_SHIFT);
}

static void pm8350b_haptics_force_stop(struct pm8350b_haptics *hap)
{
	int error;

	error = pm8350b_haptics_write(hap, hap->cfg_base,
				      HAP_CFG_SPMI_PLAY_REG, DIRECT_PLAY);
	if (error)
		dev_warn(hap->dev, "failed to disable play during forced stop: %d\n",
			 error);

	error = pm8350b_haptics_update_bits(hap, hap->cfg_base,
					    HAP_CFG_VSET_CFG_REG,
					    FORCE_VREG_RDY_BIT, 0);
	if (error)
		dev_warn(hap->dev,
			 "failed to clear force-vreg-ready during forced stop: %d\n",
			 error);

	/* Force an off command even if the preceding on command failed. */
	error = pm8350b_haptics_program_hboost(hap, false);
	if (error)
		dev_warn(hap->dev, "failed to disable hboost during forced stop: %d\n",
			 error);

	hap->active = false;
}

static int pm8350b_haptics_start(struct pm8350b_haptics *hap, u8 amplitude)
{
	bool was_active = hap->active;
	int error;

	cancel_delayed_work(&hap->hboost_off_work);

	error = pm8350b_haptics_wait_hboost_ready(hap);
	if (error)
		return error;

	error = pm8350b_haptics_set_hboost(hap, true);
	if (error)
		goto rollback;

	error = pm8350b_haptics_write(hap, hap->cfg_base,
				      HAP_CFG_FAULT_CLR_REG,
				      SC_CLR_BIT | AUTO_RES_ERR_CLR_BIT |
				      HPWR_RDY_FAULT_CLR_BIT);
	if (error)
		goto rollback;

	error = pm8350b_haptics_update_bits(hap, hap->cfg_base,
					    HAP_CFG_VSET_CFG_REG,
					    FORCE_VREG_RDY_BIT,
					    FORCE_VREG_RDY_BIT);
	if (error)
		goto rollback;

	error = pm8350b_haptics_toggle_rc_clk(hap);
	if (error)
		goto rollback;

	error = pm8350b_haptics_set_vmax(hap, hap->vmax_uv);
	if (error)
		goto rollback;

	error = pm8350b_haptics_write(hap, hap->ptn_base,
				      HAP_PTN_DIRECT_PLAY_REG, amplitude);
	if (error)
		goto rollback;

	error = pm8350b_haptics_enable_autores(hap, !hap->is_erm);
	if (error)
		goto rollback;

	error = pm8350b_haptics_write(hap, hap->cfg_base,
				      HAP_CFG_SPMI_PLAY_REG,
				      PLAY_EN_BIT | DIRECT_PLAY);
	if (error)
		goto rollback;

	hap->active = true;

	return 0;

rollback:
	if (!was_active)
		pm8350b_haptics_force_stop(hap);

	return error;
}

static int pm8350b_haptics_stop(struct pm8350b_haptics *hap)
{
	int error;

	error = pm8350b_haptics_write(hap, hap->cfg_base,
				      HAP_CFG_SPMI_PLAY_REG, DIRECT_PLAY);
	if (error)
		return error;

	error = pm8350b_haptics_update_bits(hap, hap->cfg_base,
					    HAP_CFG_VSET_CFG_REG,
					    FORCE_VREG_RDY_BIT, 0);
	if (error)
		return error;

	hap->active = false;
	schedule_delayed_work(&hap->hboost_off_work,
			      msecs_to_jiffies(HBOOST_OFF_DELAY_MS));

	return 0;
}

static void pm8350b_haptics_play_work(struct work_struct *work)
{
	struct pm8350b_haptics *hap =
		container_of(work, struct pm8350b_haptics, play_work);
	int error;

	mutex_lock(&hap->lock);

	if (hap->speed)
		error = pm8350b_haptics_start(hap, hap->speed);
	else if (hap->active)
		error = pm8350b_haptics_stop(hap);
	else
		error = 0;

	if (error)
		dev_err(hap->dev, "failed to %s haptics: %d\n",
			hap->speed ? "start" : "stop", error);

	mutex_unlock(&hap->lock);
}

static void pm8350b_haptics_hboost_off_work(struct work_struct *work)
{
	struct pm8350b_haptics *hap =
		container_of(to_delayed_work(work), struct pm8350b_haptics,
			     hboost_off_work);
	int error;

	mutex_lock(&hap->lock);
	if (!hap->active) {
		error = pm8350b_haptics_set_hboost(hap, false);
		if (error)
			dev_err(hap->dev, "failed to disable hboost: %d\n",
				error);
	}
	mutex_unlock(&hap->lock);
}

static int pm8350b_haptics_play_effect(struct input_dev *dev, void *data,
				       struct ff_effect *effect)
{
	struct pm8350b_haptics *hap = input_get_drvdata(dev);
	u16 magnitude;

	magnitude = max(effect->u.rumble.strong_magnitude,
			effect->u.rumble.weak_magnitude);

	hap->speed = magnitude >> 8;
	schedule_work(&hap->play_work);

	return 0;
}

static void pm8350b_haptics_close(struct input_dev *dev)
{
	struct pm8350b_haptics *hap = input_get_drvdata(dev);

	hap->speed = 0;
	cancel_work_sync(&hap->play_work);
	cancel_delayed_work_sync(&hap->hboost_off_work);

	mutex_lock(&hap->lock);
	pm8350b_haptics_force_stop(hap);
	mutex_unlock(&hap->lock);
}

static int pm8350b_haptics_parse_addr(struct platform_device *pdev,
				      unsigned int index, u32 *addr)
{
	const __be32 *reg;

	reg = of_get_address(pdev->dev.of_node, index, NULL, NULL);
	if (!reg)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing reg entry %u\n", index);

	*addr = be32_to_cpu(*reg);

	return 0;
}

static int pm8350b_haptics_hw_init(struct pm8350b_haptics *hap)
{
	unsigned int val;
	int error;

	error = regmap_read(hap->regmap, hap->cfg_base + HAP_CFG_REVISION2_REG,
			    &val);
	if (error)
		return error;

	error = regmap_read(hap->regmap, hap->ptn_base + HAP_PTN_REVISION2_REG,
			    &val);
	if (error)
		return error;

	error = pm8350b_haptics_module_enable(hap, true);
	if (error)
		return error;

	error = devm_add_action_or_reset(hap->dev,
					 pm8350b_haptics_disable_module, hap);
	if (error)
		return error;

	error = pm8350b_haptics_set_vmax(hap, hap->vmax_uv);
	if (error)
		return error;

	error = pm8350b_haptics_update_bits(hap, hap->cfg_base,
					    HAP_CFG_DRV_WF_SEL_REG,
					    DRV_WF_SEL_MASK,
					    hap->is_erm ?
					    PM8350B_HAP_WF_NO_MODULATION :
					    PM8350B_HAP_WF_SINE);
	if (error)
		return error;

	error = pm8350b_haptics_update_bits(hap, hap->cfg_base,
					    HAP_CFG_DRV_WF_SEL_REG,
					    DRV_WF_FMT_BIT, 0);
	if (error)
		return error;

	error = pm8350b_haptics_update_bits(hap, hap->cfg_base,
					    HAP_CFG_CAL_EN_REG,
					    CAL_RC_CLK_MASK,
					    CAL_RC_CLK_AUTO_VAL <<
					    CAL_RC_CLK_SHIFT);
	if (error)
		return error;

	return pm8350b_haptics_set_lra_period(hap, hap->lra_period_us);
}

static int pm8350b_haptics_probe(struct platform_device *pdev)
{
	struct pm8350b_haptics *hap;
	struct input_dev *input;
	int error;

	hap = devm_kzalloc(&pdev->dev, sizeof(*hap), GFP_KERNEL);
	if (!hap)
		return -ENOMEM;

	hap->dev = &pdev->dev;
	hap->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!hap->regmap)
		return -ENODEV;

	error = pm8350b_haptics_parse_addr(pdev, 0, &hap->cfg_base);
	if (error)
		return error;

	error = pm8350b_haptics_parse_addr(pdev, 1, &hap->ptn_base);
	if (error)
		return error;

	error = pm8350b_haptics_parse_addr(pdev, 2, &hap->hboost_base);
	if (error)
		return error;

	hap->vmax_uv = DEFAULT_VMAX_UV;
	device_property_read_u32(&pdev->dev, "qcom,vmax-microvolt",
				 &hap->vmax_uv);

	device_property_read_u32(&pdev->dev, "qcom,lra-period-us",
				 &hap->lra_period_us);
	hap->is_erm = device_property_read_bool(&pdev->dev, "qcom,use-erm");

	if (of_find_property(pdev->dev.of_node, "nvmem", NULL)) {
		hap->pbs_nvmem = devm_nvmem_device_get(&pdev->dev,
						       "hap_cfg_sdam");
		if (IS_ERR(hap->pbs_nvmem))
			return dev_err_probe(&pdev->dev, PTR_ERR(hap->pbs_nvmem),
					     "failed to get hap_cfg_sdam nvmem\n");
	}

	mutex_init(&hap->lock);
	INIT_WORK(&hap->play_work, pm8350b_haptics_play_work);
	INIT_DELAYED_WORK(&hap->hboost_off_work,
			  pm8350b_haptics_hboost_off_work);

	error = pm8350b_haptics_hw_init(hap);
	if (error)
		return dev_err_probe(&pdev->dev, error,
				     "failed to initialize haptics\n");

	input = devm_input_allocate_device(&pdev->dev);
	if (!input)
		return -ENOMEM;

	hap->input = input;
	input->name = "pm8350b_haptics";
	input->id.version = 1;
	input->close = pm8350b_haptics_close;
	input_set_drvdata(input, hap);
	input_set_capability(input, EV_FF, FF_RUMBLE);

	error = input_ff_create_memless(input, NULL,
					pm8350b_haptics_play_effect);
	if (error)
		return dev_err_probe(&pdev->dev, error,
				     "failed to create FF device\n");

	error = input_register_device(input);
	if (error)
		return dev_err_probe(&pdev->dev, error,
				     "failed to register input device\n");

	platform_set_drvdata(pdev, hap);

	return 0;
}

static void pm8350b_haptics_remove(struct platform_device *pdev)
{
	struct pm8350b_haptics *hap = platform_get_drvdata(pdev);

	pm8350b_haptics_close(hap->input);
}

static void pm8350b_haptics_shutdown(struct platform_device *pdev)
{
	struct pm8350b_haptics *hap = platform_get_drvdata(pdev);

	pm8350b_haptics_close(hap->input);
	pm8350b_haptics_disable_module(hap);
}

static int pm8350b_haptics_suspend(struct device *dev)
{
	struct pm8350b_haptics *hap = dev_get_drvdata(dev);

	pm8350b_haptics_close(hap->input);

	return pm8350b_haptics_module_enable(hap, false);
}

static int pm8350b_haptics_resume(struct device *dev)
{
	struct pm8350b_haptics *hap = dev_get_drvdata(dev);

	return pm8350b_haptics_module_enable(hap, true);
}

static DEFINE_SIMPLE_DEV_PM_OPS(pm8350b_haptics_pm_ops,
				pm8350b_haptics_suspend,
				pm8350b_haptics_resume);

static const struct of_device_id pm8350b_haptics_of_match[] = {
	{ .compatible = "qcom,pm8350b-haptics" },
	{ }
};
MODULE_DEVICE_TABLE(of, pm8350b_haptics_of_match);

static struct platform_driver pm8350b_haptics_driver = {
	.probe = pm8350b_haptics_probe,
	.remove = pm8350b_haptics_remove,
	.shutdown = pm8350b_haptics_shutdown,
	.driver = {
		.name = "pm8350b-haptics",
		.pm = pm_sleep_ptr(&pm8350b_haptics_pm_ops),
		.of_match_table = pm8350b_haptics_of_match,
	},
};
module_platform_driver(pm8350b_haptics_driver);

MODULE_AUTHOR("Oleksii Onchul <oleksiionchul@gmail.com>");
MODULE_DESCRIPTION("Qualcomm PM8350B haptics driver");
MODULE_LICENSE("GPL");
