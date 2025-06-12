#define DT_DRV_COMPAT liteon_ltr390

#include "ltr390.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#define UV_COUNT       160
#define UV_SENSITIVITY 2300 

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

static int ltr390_read_data(const struct device *dev, uint8_t reg, 
		uint8_t *buff, size_t data_len)
{
	const struct ltr390_config *config = dev->config;

	int ret = i2c_write_read_dt(&config->i2c, &reg, sizeof(reg), buff,
			data_len);

	if (ret < 0) {
		LOG_ERR("Read reg %x error: %d", reg, ret);
		return ret;
	}

	return ret;
}

static int ltr390_check_id(const struct device *dev)
{
	uint8_t buff = 0;
	int ret = 0;

	ret = ltr390_read_byte(dev, LTR390_PART_ID, &buff);
	if (ret < 0)
	{
		return ret;
	}
	else
	if (buff != LTR390_ID)
	{
		return -EINVAL;
	}

	return 0;
}

static int ltr390_enable(const struct device *dev)
{
	uint8_t ctl_reg_val = 0;
	int ret;

	ret = ltr390_read_byte(dev, LTR390_MAIN_CTRL, &ctl_reg_val);
	if (ret < 0)
	{
		return ret;
	}

	ctl_reg_val |= (1 << 1); // enable bit

	ret = ltr390_write_byte(dev, LTR390_MAIN_CTRL, ctl_reg_val);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int ltr390_set_gain(const struct device *dev, enum ltr390_gain gain)
{
	int ret;

	if (gain > GAIN_18)
	{
		LOG_ERR("Wrong gain: %d", gain);
		return -EINVAL;
	}

	ret = ltr390_write_byte(dev, LTR390_GAIN, gain);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}


static int ltr390_set_rate(const struct device *dev, enum ltr390_rate rate)
{
	int ret;

	if (ret > RATE_2000MS)
	{
		return -EINVAL;
	}

	uint8_t meas_rate = 0;
	ret = ltr390_read_byte(dev, LTR390_MEAS_RATE, &meas_rate);
	if (ret < 0)
	{
		return ret;
	}

	meas_rate = (meas_rate & 0x8F) | rate;

	ret = ltr390_write_byte(dev, LTR390_MEAS_RATE, meas_rate);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int ltr390_set_resolution(const struct device *dev, 
		enum ltr390_resolution resolution)
{
	int ret;

	if (resolution > RESOLUTION_13BIT_TIME12_5MS)
	{
		return -EINVAL;
	}

	uint8_t meas_rate = 0;
	ret = ltr390_read_byte(dev, LTR390_MEAS_RATE, &meas_rate);
	if (ret < 0)
	{
		return ret;
	}

	meas_rate = (meas_rate & 0x8F) | resolution << 4;

	ret = ltr390_write_byte(dev, LTR390_MEAS_RATE, meas_rate);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int ltr390_init(const struct device *dev)
{
	const struct ltr390_config *config = dev->config;
	int ret;

	if (!device_is_ready(config->i2c.bus))
	{
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	ret = ltr390_check_id(dev);
	if (ret < 0)
	{
		LOG_ERR("Check ID failed: %d", ret);
		return ret;
	}

	ret = ltr390_enable(dev);
	if (ret < 0)
	{
		LOG_ERR("Enable failed: %d", ret);
		return ret;
	}

	ret = ltr390_set_gain(dev, GAIN_3);
	if (ret < 0)
	{
		LOG_ERR("Set gain failed: %d", ret);
		return ret;
	}

	ret = ltr390_set_rate(dev, RATE_100MS);
	if (ret < 0)
	{
		LOG_ERR("Set rate failed: %d", ret);
		return ret;
	}

	ret = ltr390_set_resolution(dev, RESOLUTION_18BIT_TIME100MS);
	if (ret < 0)
	{
		LOG_ERR("Set resolution failed: %d", ret);
		return ret;
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