#define DT_DRV_COMPAT liteon_ltr390

#include "ltr390.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(ltr390, CONFIG_SENSOR_LOG_LEVEL);

static int ltr390_write_byte(const struct device *dev, uint8_t reg, uint8_t data)
{
	const struct ltr390_config *config = dev->config;

	int ret = i2c_write_dt(&config->i2c, &data, sizeof(data));
	if (ret < 0) {
		LOG_ERR("write[%02X]: %u", reg, ret);
		return ret;
	}
	return ret;
}

static int ltr390_read_byte(const struct device *dev, uint8_t reg, uint8_t *buff)
{
	const struct ltr390_config *config = dev->config;

	int ret = i2c_write_read_dt(&config->i2c, &reg, sizeof(reg), buff, 1);

	if (ret < 0) {
		LOG_ERR("Read reg %x error: %d", reg, ret);
		return ret;
	}

	return ret;
}

static int ltr390_init(const struct device *dev)
{
	const struct ltr390_config *config = dev->config;
	struct ltr390_data *data = dev->data;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	uint8_t buff = 0;
	int ret = 0;

	ret = ltr390_read_byte(dev, LTR390_PART_ID, &buff);
	if (buff != LTR390_ID || ret < 0) {
		LOG_ERR("Can't read ID: %d", ret);
		return -EINVAL;
	}

	return 0;
}

static int ltr390_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	return 0;
}

static int ltr390_channel_get(const struct device *dev,
				enum sensor_channel chan,
				struct sensor_value *val)
{
	return 0;
}


static const struct sensor_driver_api ltr390_driver_api = {
	.sample_fetch = ltr390_sample_fetch,
	.channel_get = ltr390_channel_get,
};


#define LTR390_DEFINE(inst)									\
	static struct ltr390_data ltr390_data_##inst;						\
												\
	static const struct ltr390_config ltr390_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ltr390_init, PM_DEVICE_DT_INST_GET(inst),		\
			      &ltr390_data_##inst, &ltr390_config_##inst, POST_KERNEL,		\
			      CONFIG_SENSOR_INIT_PRIORITY, &ltr390_driver_api);			\

DT_INST_FOREACH_STATUS_OKAY(LTR390_DEFINE)