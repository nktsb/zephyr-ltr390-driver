#define DT_DRV_COMPAT liteon_ltr390

#include "ltr390.h"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(ltr390, CONFIG_SENSOR_LOG_LEVEL);

static int ltr390_init(const struct device *dev)
{
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


static const struct sensor_driver_api vcnl4040_driver_api = {
	.sample_fetch = ltr390_sample_fetch,
	.channel_get = ltr390_channel_get,
};


#define LTR390_DEFINE(inst)									\
	static struct ltr390_data ltr390_data_##inst;					\
												\
	static const struct ltr390_config ltr390_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
		IF_ENABLED(CONFIG_LTR390_TRIGGER,						\
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),))	\
		.led_i = DT_INST_ENUM_IDX(inst, led_current),					\
		.led_dc = DT_INST_ENUM_IDX(inst, led_duty_cycle),				\
		.als_it = DT_INST_ENUM_IDX(inst, als_it),					\
		.proxy_it = DT_INST_ENUM_IDX(inst, proximity_it),				\
		.proxy_type = DT_INST_ENUM_IDX(inst, proximity_trigger),			\
	};											\
												\
	PM_DEVICE_DT_INST_DEFINE(inst, ltr390_pm_action);					\
												\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ltr390_init, PM_DEVICE_DT_INST_GET(inst),		\
			      &ltr390_data_##inst, &ltr390_config_##inst, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &ltr390_driver_api);		\

DT_INST_FOREACH_STATUS_OKAY(LTR390_DEFINE)