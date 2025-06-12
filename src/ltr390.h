#ifndef LTR390_DISPLAY_DRIVER_H__
#define LTR390_DISPLAY_DRIVER_H__

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define SENSOR_CHAN_LTR390_UVI SENSOR_CHAN_PRIV_START

#define LTR390_MAIN_CTRL	    (0x00) // Main control register
#define LTR390_MEAS_RATE	    (0x04) // Resolution and data rate
#define LTR390_GAIN		    (0x05) // ALS and UVS gain range
#define LTR390_PART_ID		    (0x06) // Part id/revision register
#define LTR390_MAIN_STATUS	    (0x07) // Main status register
#define LTR390_ALSDATA		    (0x0D) // ALS data lowest byte, 3 byte
#define LTR390_UVSDATA		    (0x10) // UVS data lowest byte, 3 byte
#define LTR390_INT_CFG		    (0x19) // Interrupt configuration
#define LTR390_INT_PST		    (0x1A) // Interrupt persistance config
#define LTR390_THRESH_UP	    (0x21) // Upper threshold, low byte, 3 byte
#define LTR390_THRESH_LOW	    (0x24) // Lower threshold, low byte, 3 byte

//measurement Gain Range.

#define LTR390_ID		    (0xb2)

typedef enum ltr390_mode
{
	LTR390_MODE_ALS,
	LTR390_MODE_UVS,
} ltr390_mode_t;

typedef enum ltr390_gain {
	LTR390_GAIN_1,
	LTR390_GAIN_3,
	LTR390_GAIN_6,
	LTR390_GAIN_9,
	LTR390_GAIN_18,
} ltr390_gain_t;

typedef enum ltr390_rate {
	LTR390_RATE_25MS,
	LTR390_RATE_50MS,
	LTR390_RATE_100MS,
	LTR390_RATE_200MS,
	LTR390_RATE_500MS,
	LTR390_RATE_1000MS,
	LTR390_RATE_2000MS,
} ltr390_rate_t;

typedef enum ltr390_resolution {
	LTR390_RES_20BIT_TIME400MS,
	LTR390_RES_19BIT_TIME200MS,
	LTR390_RES_18BIT_TIME100MS,
	LTR390_RES_17BIT_TIME50MS,
	LTR390_RES_16BIT_TIME25MS,
	LTR390_RES_13BIT_TIME12_5MS,

	LTR390_RES_ENUM_SIZE,
} ltr390_resolution_t;

struct ltr390_config {
	struct i2c_dt_spec i2c;
	ltr390_gain_t gain;
	ltr390_resolution_t resolution;
	ltr390_rate_t data_rate;
	ltr390_mode_t mode;
};

struct ltr390_data {
	uint32_t raw_lux_data;
	uint32_t raw_uvi_data;
	float uvi;
	float lux;

	bool enabled;
};

#endif /* LTR390_DISPLAY_DRIVER_H__ */