#define DT_DRV_COMPAT liteon_ltr390

#include "ltr390.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#define LTR390_UV_SENSITIVITY 2300 
#define WFAC 1.0f
#define MAX_CONVERISON_TIME 1000

LOG_MODULE_REGISTER(ltr390, CONFIG_SENSOR_LOG_LEVEL);

static int ltr390_write_byte(const struct device *dev, uint8_t reg, uint8_t byte)
{
	const struct ltr390_config *config = dev->config;

	uint8_t buff[] = {reg, byte};
	int ret = i2c_write_dt(&config->i2c, buff, sizeof(buff));
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

static int ltr390_disable_meas(const struct device *dev)
{
	struct ltr390_data *data = dev->data;
	uint8_t ctl_reg_val = 0;
	int ret;

	ret = ltr390_read_byte(dev, LTR390_MAIN_CTRL, &ctl_reg_val);
	if (ret < 0)
	{
		return ret;
	}

	ctl_reg_val &= ~(1 << 1);

	ret = ltr390_write_byte(dev, LTR390_MAIN_CTRL, ctl_reg_val);
	if (ret < 0)
	{
		return ret;
	}
	data->enabled = true;

	return 0;
}

static int ltr390_enable_meas(const struct device *dev)
{
	struct ltr390_data *data = dev->data;
	uint8_t ctl_reg_val = 0;
	int ret;

	ret = ltr390_read_byte(dev, LTR390_MAIN_CTRL, &ctl_reg_val);
	if (ret < 0)
	{
		return ret;
	}

	ctl_reg_val |= (1 << 1);

	ret = ltr390_write_byte(dev, LTR390_MAIN_CTRL, ctl_reg_val);
	if (ret < 0)
	{
		return ret;
	}
	data->enabled = true;

	return 0;
}

static int ltr390_is_data_ready(const struct device *dev)
{
	uint8_t status_reg_val = 0;
	int ret;

	ret = ltr390_read_byte(dev, LTR390_MAIN_STATUS, &status_reg_val);
	if (ret < 0)
	{
		return ret;
	}

	if (status_reg_val & (1 << 3))
	{
		return 1; // data ready
	}

	return 0; // data not ready
}

static int ltr390_set_gain(const struct device *dev, ltr390_gain_t gain)
{
	int ret;

	if (gain > LTR390_GAIN_18)
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


static int ltr390_set_rate(const struct device *dev, ltr390_rate_t rate)
{
	int ret;

	if (rate > LTR390_RATE_2000MS)
	{
		return -EINVAL;
	}

	uint8_t meas_rate = 0;
	ret = ltr390_read_byte(dev, LTR390_MEAS_RATE, &meas_rate);
	if (ret < 0)
	{
		return ret;
	}

	meas_rate &= meas_rate & 0xF8;
	meas_rate |= rate;

	ret = ltr390_write_byte(dev, LTR390_MEAS_RATE, meas_rate);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int ltr390_set_resolution(const struct device *dev, 
		ltr390_resolution_t resolution)
{
	int ret;

	if (resolution > LTR390_RES_13BIT_TIME12_5MS)
	{
		return -EINVAL;
	}

	uint8_t meas_rate = 0;
	ret = ltr390_read_byte(dev, LTR390_MEAS_RATE, &meas_rate);
	if (ret < 0)
	{
		return ret;
	}

	meas_rate &= 0x8F;
	meas_rate |= resolution << 4;

	ret = ltr390_write_byte(dev, LTR390_MEAS_RATE, meas_rate);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int ltr390_set_mode(const struct device *dev, 
		ltr390_mode_t mode)
{
	int ret;

	if (mode > LTR390_MODE_UVS)
	{
		return -EINVAL;
	}

	uint8_t ctl_reg_val = 0;

	ret = ltr390_read_byte(dev, LTR390_MAIN_CTRL, &ctl_reg_val);
	if (ret < 0)
	{
		return ret;
	}

	ctl_reg_val &= ~(1 << 3);
	ctl_reg_val |= (mode << 3);

	ret = ltr390_write_byte(dev, LTR390_MAIN_CTRL, ctl_reg_val);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int32_t ltr390_get_raw_uvs_data(const struct device *dev, uint32_t *buff)
{
	int ret;

	ret = ltr390_read_data(dev, LTR390_UVSDATA, (uint8_t *)buff, 3);
	if (ret < 0)
	{
		return ret;
	}
	return 0;
}

static int32_t ltr390_get_raw_als_data(const struct device *dev, uint32_t *buff)
{
	int ret;

	ret = ltr390_read_data(dev, LTR390_ALSDATA, (uint8_t *)buff, 3);
	if (ret < 0)
	{
		return ret;
	}
	return 0;
}

static int ltr390_sample_fetch(const struct device *dev,
				 enum sensor_channel chan);

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

	ret = ltr390_set_gain(dev, config->gain);
	if (ret < 0)
	{
		LOG_ERR("Set gain failed: %d", ret);
		return ret;
	}

	ret = ltr390_set_rate(dev, config->data_rate);
	if (ret < 0)
	{
		LOG_ERR("Set rate failed: %d", ret);
		return ret;
	}

	ret = ltr390_set_resolution(dev, config->resolution);
	if (ret < 0)
	{
		LOG_ERR("Set resolution failed: %d", ret);
		return ret;
	}

	ltr390_sample_fetch(dev, SENSOR_CHAN_LTR390_UVI);
	ltr390_sample_fetch(dev, SENSOR_CHAN_LIGHT);
	ltr390_sample_fetch(dev, SENSOR_CHAN_LTR390_UVI);
	ltr390_sample_fetch(dev, SENSOR_CHAN_LIGHT);
	ltr390_sample_fetch(dev, SENSOR_CHAN_LIGHT);
	ltr390_sample_fetch(dev, SENSOR_CHAN_LIGHT);

	return 0;
}

static const uint8_t gain_factor[] ={
	[LTR390_GAIN_1] = 1,
	[LTR390_GAIN_3] = 3,
	[LTR390_GAIN_6] = 6,
	[LTR390_GAIN_9] = 9,
	[LTR390_GAIN_18] = 18,
};

static const float res_factor[] ={
	[LTR390_RES_20BIT_TIME400MS] = 4.0,
	[LTR390_RES_19BIT_TIME200MS] = 2.0,
	[LTR390_RES_18BIT_TIME100MS] = 1.0,
	[LTR390_RES_17BIT_TIME50MS] = 0.5,
	[LTR390_RES_16BIT_TIME25MS] = 0.25,
	[LTR390_RES_13BIT_TIME12_5MS] = 0.03125,
};

static inline void ltr390_get_uvi_from_raw(const struct device *dev)
{
	const struct ltr390_config *config = dev->config;
	struct ltr390_data *data = dev->data;

	data->uvi = (float)(data->raw_uvi_data) * (float)(WFAC) / 
			((gain_factor[config->gain] / 
			gain_factor[LTR390_GAIN_18]) * 
			(res_factor[config->resolution] / 
			res_factor[LTR390_RES_20BIT_TIME400MS]) * 
			(float)(LTR390_UV_SENSITIVITY));
} 

static inline void ltr390_get_lux_from_raw(const struct device *dev)
{
	const struct ltr390_config *config = dev->config;
	struct ltr390_data *data = dev->data;

	data->lux = 0.6f * (float)(data->raw_lux_data) * (float)(WFAC)/ 
			(gain_factor[config->gain] * 
			res_factor[config->resolution]);
} 

static int ltr390_sample_fetch(const struct device *dev,
				 enum sensor_channel chan)
{
	struct ltr390_data *data = dev->data;
	int ret;

	switch (chan)
	{
		case SENSOR_CHAN_LIGHT:
			ret = ltr390_set_mode(dev, LTR390_MODE_ALS);
			if (ret < 0)
			{
				LOG_ERR("Error setting ALS mode: %d", ret);
				return ret;
			}
			break;
		case SENSOR_CHAN_LTR390_UVI:
			ret = ltr390_set_mode(dev, LTR390_MODE_UVS);
			if (ret < 0)
			{
				LOG_ERR("Error setting UVS mode: %d", ret);
				return ret;
			}
			break;
		default:
			return -ENOTSUP;
	}

	ltr390_enable_meas(dev);


	int64_t start_time = k_uptime_get();

	while (1)
	{
		if (k_uptime_get() - start_time > MAX_CONVERISON_TIME)
		{
			LOG_ERR("Waiting data ready timed out!");
			ltr390_disable_meas(dev);

			return -ETIMEDOUT;
		}

		ret = ltr390_is_data_ready(dev);
		if (ret == 1)
		{
			break;
		}
		if (ret < 0)
		{
			LOG_ERR("Error waiting data ready: %d", ret);
			return ret;
		}

		k_msleep(5);
	}

	ltr390_disable_meas(dev);

	switch (chan)
	{
		case SENSOR_CHAN_LTR390_UVI:
			ret = ltr390_get_raw_uvs_data(dev, &data->raw_uvi_data);
			if (ret < 0)
			{
				LOG_ERR("Error getting raw UVS: %d", ret);
				return ret;
			}
			ltr390_get_uvi_from_raw(dev);
			break;
		case SENSOR_CHAN_LIGHT:
			ret = ltr390_get_raw_als_data(dev, &data->raw_lux_data);
			if (ret < 0)
			{
				LOG_ERR("Error getting raw ALS: %d", ret);
				return ret;
			}
			ltr390_get_lux_from_raw(dev);
			break;
		default:
			return -ENOTSUP;
	}

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
		.gain = DT_INST_ENUM_IDX(inst, gain),						\
		.resolution = LTR390_RES_ENUM_SIZE - DT_INST_ENUM_IDX(inst, resolution) - 1,	\
		.data_rate = DT_INST_ENUM_IDX(inst, data_rate),					\
	};											\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ltr390_init, PM_DEVICE_DT_INST_GET(inst),		\
			      &ltr390_data_##inst, &ltr390_config_##inst, POST_KERNEL,		\
			      CONFIG_SENSOR_INIT_PRIORITY, &ltr390_driver_api);			\

DT_INST_FOREACH_STATUS_OKAY(LTR390_DEFINE)