#ifndef LTR390_DISPLAY_DRIVER_H__
#define LTR390_DISPLAY_DRIVER_H__

#include <zephyr/kernel.h>

struct ltr390_config {
	struct i2c_dt_spec i2c;
};

struct ltr390_data {

};

#endif /* LTR390_DISPLAY_DRIVER_H__ */