## LTR390 I2C Driver for Zephyr RTOS

This is a Zephyr RTOS driver for the Lite-On LTR390 UV/ambient light sensor (using I2C).
It supports measurement of UV index and ambient light, with configurable resolution, gain, and data rate.

## Features

- I2C interface support for communication with the LTR390 sensor.
- Supports two measurement modes: UV Index ("uv-index") and Ambient Light ("ambient-light").
- Configurable resolution (13-bit or 16-bit), gain, and data rate.
- Deferred initialization support.

## Adding Driver To Zephyr Project

### CMakeLists.txt

To include the driver in your Zephyr project, specify its path in the CMakeLists.txt file as follows:

```
set(ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/ltr390-zephyr-driver  # Change it to your path
)
```

### Configuration in prj.conf

Add the following configuration option to your prj.conf to enable the LTR390 driver:

```
CONFIG_I2C=y
CONFIG_SENSOR=y
CONFIG_LTR390=y
```

### Example I2C Configuration in Device Tree

Ensure that the I2C interface is properly configured in the Device Tree. Below is an example configuration for the I2C0 interface:

```
&i2c0 {
    compatible = "nordic,nrf-twim";
    status = "okay";
    clock-frequency = <I2C_BITRATE_FAST>;
    pinctrl-0 = <&i2c0_default>;
    pinctrl-1 = <&i2c0_sleep>;
    pinctrl-names = "default", "sleep";

    ltr390: ltr390@53 {
        compatible = "liteon,ltr390";
        reg = <0x53>;
        gain = <3>;            // Gain: 1, 3, 6, 9, 18
        data-rate = <500>;     // Data rate in ms (100, 200, 500, 1000, 2000)
        resolution = <16>;     // 13 or 16 bits
        mode = "uv-index";     // or "ambient-light"
        zephyr,deferred-init;
    };
};
```

**Supported modes:**

```
"uv-index"
"ambient-light"
```

## Contributing

Pull requests and issues are always welcome! Feel free to contribute to make this project better.