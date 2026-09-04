// SPDX-License-Identifier: GPL-2.0
/*
 * Awinic AW21009/AW21012/AW21018 LED driver
 *
 * Copyright (c) 2026, Oleksii Onchul <oleksiionchul@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/fwnode.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define AW210XX_BRIGHTNESS_MAX          255
#define AW210XX_MAX_CURRENT_UA          40000
#define AW210XX_DEFAULT_CURRENT_UA      16000
#define AW210XX_MIN_CURRENT_UA          157

#define AW210XX_REG_GCR                 0x20
#define AW210XX_REG_BR00L               0x21
#define AW210XX_REG_UPDATE              0x45
#define AW210XX_REG_SL00                0x46
#define AW210XX_REG_GCCR                0x58
#define AW210XX_REG_GCR2                0x61
#define AW210XX_REG_GCFG                0x8b
#define AW210XX_REG_RESET               0x70

#define AW210XX_GCR_CHIPEN              BIT(0)
#define AW210XX_GCR_APSE                BIT(7)
#define AW210XX_GCR_CLKFRQ_MASK         GENMASK(6, 4)
#define AW210XX_GCR_CLKFRQ_16MHZ        FIELD_PREP(AW210XX_GCR_CLKFRQ_MASK, 0)
#define AW210XX_GCR_PWMRES_MASK         GENMASK(2, 1)
#define AW210XX_GCR_PWMRES_8BIT         FIELD_PREP(AW210XX_GCR_PWMRES_MASK, 0)

#define AW210XX_GCR2_RGBMD              BIT(0)
#define AW210XX_GCR2_SBMD               BIT(1)

#define AW210XX_GROUP_DISABLE           0x40
#define AW210XX_UPDATE_LATCH            0x00
#define AW210XX_GCCR_MAX                0xff
#define AW210XX_SCALE_MAX               0xff

#define AW21018_CHIPID                  0x02
#define AW21012_CHIPID                  0x22
#define AW21009_CHIPID                  0x12

struct aw210xx_chipdef {
	u8 chipid;
	u8 channels;
};

struct aw210xx;

struct aw210xx_led {
	struct led_classdev cdev;
	struct aw210xx *chip;
	u8 channel;
	u8 scale;
};

struct aw210xx {
	struct regmap *regmap;
	/* Serializes register access and LED state changes. */
	struct mutex lock;
	struct gpio_desc *enable_gpio;

	const struct aw210xx_chipdef *cdef;
	u8 chipid;

	struct aw210xx_led *leds;
	u8 num_leds;
};

static const struct regmap_config aw210xx_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW210XX_REG_GCFG,
};

static const struct aw210xx_chipdef aw21009_cdef = {
	.chipid = AW21009_CHIPID,
	.channels = 9,
};

static const struct aw210xx_chipdef aw21012_cdef = {
	.chipid = AW21012_CHIPID,
	.channels = 12,
};

static const struct aw210xx_chipdef aw21018_cdef = {
	.chipid = AW21018_CHIPID,
	.channels = 18,
};

static const struct aw210xx_chipdef *aw210xx_chipdef_by_chipid(u8 chipid)
{
	switch (chipid) {
	case AW21009_CHIPID:
		return &aw21009_cdef;
	case AW21012_CHIPID:
		return &aw21012_cdef;
	case AW21018_CHIPID:
		return &aw21018_cdef;
	default:
		return NULL;
	}
}

static const struct aw210xx_chipdef *aw210xx_chipdef_by_name(const char *name)
{
	if (!name)
		return NULL;
	if (!strcmp(name, "aw21009"))
		return &aw21009_cdef;
	if (!strcmp(name, "aw21012"))
		return &aw21012_cdef;
	if (!strcmp(name, "aw21018"))
		return &aw21018_cdef;
	return NULL;
}

static int aw210xx_write_channel(struct aw210xx *chip, u8 channel, u16 brightness)
{
	unsigned int reg_l = AW210XX_REG_BR00L + channel * 2;
	u8 value = brightness;
	int ret;

	ret = regmap_write(chip->regmap, reg_l, value);
	if (ret)
		return ret;

	return regmap_write(chip->regmap, reg_l + 1, 0);
}

static u8 aw210xx_current_to_scale(u32 max_uA)
{
	u32 scale;

	/*
	 * At maximum brightness and GCCR=0xff, the output current is
	 * 40mA * SL / 256 for a valid REXT of at least 4kOhm.
	 */
	scale = max_uA * 256 / AW210XX_MAX_CURRENT_UA;

	return min_t(u32, scale, AW210XX_SCALE_MAX);
}

static int aw210xx_update(struct aw210xx *chip)
{
	return regmap_write(chip->regmap, AW210XX_REG_UPDATE, AW210XX_UPDATE_LATCH);
}

static int aw210xx_brightness_set_blocking(struct led_classdev *cdev,
					   enum led_brightness brightness)
{
	struct aw210xx_led *led = container_of(cdev, struct aw210xx_led, cdev);
	struct aw210xx *chip = led->chip;
	int ret;

	mutex_lock(&chip->lock);

	ret = aw210xx_write_channel(chip, led->channel, brightness);
	if (!ret)
		ret = aw210xx_update(chip);

	mutex_unlock(&chip->lock);

	return ret;
}

static int aw210xx_read_chipid(struct aw210xx *chip, u8 *chipid)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chip->regmap, AW210XX_REG_RESET, &val);
	if (ret)
		return ret;

	*chipid = val;
	return 0;
}

static void aw210xx_disable_action(void *data)
{
	struct aw210xx *chip = data;

	if (chip->enable_gpio)
		gpiod_set_value_cansleep(chip->enable_gpio, 0);
}

static int aw210xx_enable(struct aw210xx *chip)
{
	if (!chip->enable_gpio)
		return 0;

	gpiod_set_value_cansleep(chip->enable_gpio, 1);
	/* Datasheet power-up timing is in the hundreds of usec range. */
	fsleep(2500);
	return 0;
}

static int aw210xx_init_chip(struct aw210xx *chip)
{
	int ret;

	ret = regmap_update_bits(chip->regmap, AW210XX_REG_GCR,
				 AW210XX_GCR_CLKFRQ_MASK | AW210XX_GCR_PWMRES_MASK,
				 AW210XX_GCR_CLKFRQ_16MHZ |
				 AW210XX_GCR_PWMRES_8BIT);
	if (ret)
		return ret;

	ret = regmap_update_bits(chip->regmap, AW210XX_REG_GCR,
				 AW210XX_GCR_CHIPEN | AW210XX_GCR_APSE,
				 AW210XX_GCR_CHIPEN | AW210XX_GCR_APSE);
	if (ret)
		return ret;

	/* Wait for the internal oscillator to stabilize after enabling the chip. */
	fsleep(200);

	ret = regmap_update_bits(chip->regmap, AW210XX_REG_GCR2,
				 AW210XX_GCR2_SBMD | AW210XX_GCR2_RGBMD, 0);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, AW210XX_REG_GCFG, AW210XX_GROUP_DISABLE);
	if (ret)
		return ret;

	return 0;
}

static int aw210xx_init_led(struct device *dev, struct aw210xx_led *led,
			    struct fwnode_handle *fwnode)
{
	enum led_default_state default_state;
	unsigned int brightness;
	u32 max_brightness = AW210XX_BRIGHTNESS_MAX;
	u32 max_uA = AW210XX_DEFAULT_CURRENT_UA;
	int ret;

	if (fwnode_property_present(fwnode, "max-brightness")) {
		ret = fwnode_property_read_u32(fwnode, "max-brightness",
					       &max_brightness);
		if (ret || !max_brightness ||
		    max_brightness > AW210XX_BRIGHTNESS_MAX)
			return dev_err_probe(dev, -EINVAL,
					     "Invalid max-brightness\n");
	}

	if (fwnode_property_present(fwnode, "led-max-microamp")) {
		ret = fwnode_property_read_u32(fwnode, "led-max-microamp",
					       &max_uA);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Invalid led-max-microamp\n");
	}

	if (max_uA < AW210XX_MIN_CURRENT_UA ||
	    max_uA > AW210XX_MAX_CURRENT_UA)
		return dev_err_probe(dev, -EINVAL,
				     "led-max-microamp out of range: %u\n", max_uA);

	led->cdev.max_brightness = max_brightness;
	led->scale = aw210xx_current_to_scale(max_uA);

	default_state = led_init_default_state_get(fwnode);
	switch (default_state) {
	case LEDS_DEFSTATE_ON:
		brightness = max_brightness;
		if (fwnode_property_present(fwnode, "default-brightness")) {
			ret = fwnode_property_read_u32(fwnode,
						       "default-brightness",
						       &brightness);
			if (ret)
				return dev_err_probe(dev, ret,
						     "Invalid default-brightness\n");
			brightness = min(brightness, max_brightness);
		}
		break;
	case LEDS_DEFSTATE_KEEP:
		return dev_err_probe(dev, -EINVAL,
				     "default-state keep is unsupported\n");
	default:
		brightness = LED_OFF;
		break;
	}

	led->cdev.brightness = brightness;

	ret = regmap_write(led->chip->regmap, AW210XX_REG_SL00 + led->channel,
			   led->scale);
	if (ret)
		return ret;

	return aw210xx_write_channel(led->chip, led->channel, brightness);
}

static int aw210xx_register_dt_leds(struct device *dev, struct aw210xx *chip)
{
	struct fwnode_handle *child;
	unsigned long channels = 0;
	u32 count = device_get_child_node_count(dev);
	u32 i = 0;
	u8 ch;
	int ret;

	if (!count)
		return dev_err_probe(dev, -EINVAL,
				     "No LED child nodes found\n");
	if (count > chip->cdef->channels)
		return dev_err_probe(dev, -EINVAL,
				     "Too many LED child nodes: %u\n", count);

	chip->leds = devm_kcalloc(dev, count, sizeof(*chip->leds), GFP_KERNEL);
	if (!chip->leds)
		return -ENOMEM;

	device_for_each_child_node(dev, child) {
		struct aw210xx_led *led;
		struct led_init_data init_data = {};
		u32 reg;

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "LED child missing reg property\n");
		}

		if (reg >= chip->cdef->channels) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "LED reg out of range: %u\n", reg);
		}

		if (channels & BIT(reg)) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "Duplicate LED channel: %u\n", reg);
		}
		channels |= BIT(reg);

		led = &chip->leds[i];
		led->chip = chip;
		led->channel = reg;
		led->cdev.brightness_set_blocking = aw210xx_brightness_set_blocking;
		led->cdev.flags |= LED_CORE_SUSPENDRESUME;
		init_data.fwnode = child;

		mutex_lock(&chip->lock);
		ret = aw210xx_init_led(dev, led, child);
		mutex_unlock(&chip->lock);
		if (ret) {
			fwnode_handle_put(child);
			return ret;
		}

		ret = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
		if (ret) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, ret,
					     "Failed to register LED on channel %u\n", reg);
		}

		i++;
	}

	chip->num_leds = i;

	mutex_lock(&chip->lock);
	for (ch = 0; ch < chip->cdef->channels; ch++) {
		if (channels & BIT(ch))
			continue;

		ret = regmap_write(chip->regmap, AW210XX_REG_SL00 + ch, 0);
		if (ret)
			goto out_unlock;

		ret = aw210xx_write_channel(chip, ch, 0);
		if (ret)
			goto out_unlock;
	}

	ret = regmap_write(chip->regmap, AW210XX_REG_GCCR, AW210XX_GCCR_MAX);
	if (!ret)
		ret = aw210xx_update(chip);

out_unlock:
	mutex_unlock(&chip->lock);
	return ret;
}

static int aw210xx_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	const struct aw210xx_chipdef *match_cdef;
	const struct i2c_device_id *id;
	const struct aw210xx_chipdef *detected_cdef;
	struct aw210xx *chip;
	u8 chipid;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);

	mutex_init(&chip->lock);

	chip->regmap = devm_regmap_init_i2c(client, &aw210xx_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "Failed to allocate regmap\n");

	chip->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(chip->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->enable_gpio),
				     "Failed to get enable GPIO\n");

	ret = aw210xx_enable(chip);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, aw210xx_disable_action, chip);
	if (ret)
		return ret;

	ret = aw210xx_read_chipid(chip, &chipid);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read chip ID\n");

	chip->chipid = chipid;
	detected_cdef = aw210xx_chipdef_by_chipid(chipid);
	if (!detected_cdef)
		return dev_err_probe(dev, -ENODEV,
				     "Unsupported chip id: 0x%02x\n", chipid);

	match_cdef = device_get_match_data(dev);
	if (!match_cdef) {
		id = i2c_client_get_device_id(client);
		match_cdef = aw210xx_chipdef_by_name(id ? id->name : NULL);
	}

	if (match_cdef && match_cdef->chipid != chipid)
		return dev_err_probe(dev, -ENODEV,
				     "DT/i2c-id mismatch: chip id 0x%02x\n", chipid);

	chip->cdef = detected_cdef;

	mutex_lock(&chip->lock);
	ret = aw210xx_init_chip(chip);
	mutex_unlock(&chip->lock);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize chip\n");

	ret = aw210xx_register_dt_leds(dev, chip);
	if (ret)
		return ret;

	dev_info(dev, "AW210xx detected (id=0x%02x, channels=%u, leds=%u)\n",
		 chip->chipid, chip->cdef->channels, chip->num_leds);

	return 0;
}

static int aw210xx_suspend(struct device *dev)
{
	struct aw210xx *chip = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&chip->lock);

	ret = regmap_update_bits(chip->regmap, AW210XX_REG_GCR,
				 AW210XX_GCR_CHIPEN, 0);
	if (ret)
		goto out_unlock;

	if (chip->enable_gpio)
		gpiod_set_value_cansleep(chip->enable_gpio, 0);

out_unlock:
	mutex_unlock(&chip->lock);

	return ret;
}

static int aw210xx_resume(struct device *dev)
{
	struct aw210xx *chip = dev_get_drvdata(dev);
	u8 i;
	int ret;

	mutex_lock(&chip->lock);

	ret = aw210xx_enable(chip);
	if (ret)
		goto out_unlock;

	ret = aw210xx_init_chip(chip);
	if (ret)
		goto out_unlock;

	for (i = 0; i < chip->num_leds; i++) {
		ret = regmap_write(chip->regmap,
				   AW210XX_REG_SL00 + chip->leds[i].channel,
				   chip->leds[i].scale);
		if (ret)
			goto out_unlock;

		ret = aw210xx_write_channel(chip, chip->leds[i].channel,
					    chip->leds[i].cdev.brightness);
		if (ret)
			goto out_unlock;
	}

	ret = regmap_write(chip->regmap, AW210XX_REG_GCCR, AW210XX_GCCR_MAX);
	if (!ret)
		ret = aw210xx_update(chip);

out_unlock:
	mutex_unlock(&chip->lock);

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(aw210xx_pm_ops, aw210xx_suspend, aw210xx_resume);

static const struct of_device_id aw210xx_of_match[] = {
	{ .compatible = "awinic,aw21009", .data = &aw21009_cdef },
	{ .compatible = "awinic,aw21012", .data = &aw21012_cdef },
	{ .compatible = "awinic,aw21018", .data = &aw21018_cdef },
	{ }
};
MODULE_DEVICE_TABLE(of, aw210xx_of_match);

static const struct i2c_device_id aw210xx_i2c_id[] = {
	{ "aw21009" },
	{ "aw21012" },
	{ "aw21018" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw210xx_i2c_id);

static struct i2c_driver aw210xx_driver = {
	.driver = {
		.name = "aw210xx-led",
		.of_match_table = aw210xx_of_match,
		.pm = pm_sleep_ptr(&aw210xx_pm_ops),
	},
	.probe = aw210xx_probe,
	.id_table = aw210xx_i2c_id,
};
module_i2c_driver(aw210xx_driver);

MODULE_AUTHOR("Oleksii Onchul <oleksiionchul@gmail.com>");
MODULE_DESCRIPTION("AW210XX LED driver");
MODULE_LICENSE("GPL");
