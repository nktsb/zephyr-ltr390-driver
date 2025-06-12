#ifndef LTR390_DISPLAY_DRIVER_H__
#define LTR390_DISPLAY_DRIVER_H__

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

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

enum ltr390_gain {
	GAIN_1 = 0x0,
	GAIN_3 = 0x1,
	GAIN_6 = 0x2,
	GAIN_9 = 0x3,
	GAIN_18 = 0x4,
};

enum ltr390_rate {
	RATE_25MS = (0x0),
	RATE_50MS = (0x1),
	RATE_100MS = (0x2),
	RATE_200MS = (0x3),
	RATE_500MS = (0x4),
	RATE_1000MS = (0x5),
	RATE_2000MS = (0x6),
};

enum ltr390_resolution {
	RESOLUTION_20BIT_TIME400MS = 0X00,
	RESOLUTION_19BIT_TIME200MS = 0X10,
	RESOLUTION_18BIT_TIME100MS = 0X20,
	RESOLUTION_17BIT_TIME50MS = 0x3,
	RESOLUTION_16BIT_TIME25MS = 0x40,
	RESOLUTION_13BIT_TIME12_5MS = 0x50,
};

struct ltr390_config {
	struct i2c_dt_spec i2c;
};

struct ltr390_data {
	uint8_t uvi;
	uint32_t lux;
};

#endif /* LTR390_DISPLAY_DRIVER_H__ */