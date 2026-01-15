# Device Tree Configuration (DTS)

## File Organization

```
boards/nucleo_f756zg/
├── nucleo_f756zg.dts       # Root DTS (includes others)
├── nucleo_f756zg.yaml      # Board metadata
└── Kconfig.board           # Board Kconfig options

dts/
├── custom_sensors.dtsi     # BME280, MPU6050 nodes
├── custom_uarts.dtsi       # UART overrides (UART2, UART3, UART6)
└── custom_pinctrl.dtsi     # Pin muxing (optional)
```

## Root DTS Structure (nucleo_f756zg.dts)

```dts
/dts-v1/;
#include <st/stm32f756zgtx.dtsi>          // STM32 core
#include <st/stm32f7xx-pinctrl.dtsi>      // Pin macros

/ {
    model = "STMicroelectronics STM32F756ZG Nucleo";
    compatible = "st,stm32f756";

    chosen {
        zephyr,console = &uart3;           // Debug output
        zephyr,shell-uart = &uart3;
        zephyr,code-partition = &code_partition;
    };

    aliases {
        i2c-0 = &i2c1;                     // Alias for code
        spi-0 = &spi1;
        uart-0 = &uart2;                   // Modem
        uart-1 = &uart3;                   // Debug
        uart-2 = &uart6;                   // GPS
        led0 = &status_led;
    };
};

// Include sensor-specific DTS
#include "../dts/custom_sensors.dtsi"
#include "../dts/custom_uarts.dtsi"
```

## I2C Sensor Nodes (custom_sensors.dtsi)

```dts
&i2c1 {
    status = "okay";
    clock-frequency = <I2C_BITRATE_STANDARD>;  // 400 kHz
    
    bme280: bme280@77 {
        compatible = "bosch,bme280";
        reg = <0x77>;
        label = "BME280";
    };
    
    mpu6050: mpu6050@68 {
        compatible = "invensense,mpu6050";
        reg = <0x68>;
        label = "MPU6050";
    };
};
```

## UART Configuration (custom_uarts.dtsi)

```dts
&uart2 {
    status = "okay";
    current-speed = <115200>;
    pinctrl-0 = <&uart2_default>;
    label = "MODEM";
};

&uart3 {
    status = "okay";
    current-speed = <115200>;
    pinctrl-0 = <&uart3_default>;
    label = "DEBUG";
};

&uart6 {
    status = "okay";
    current-speed = <9600>;      // GPS is 9600 baud
    pinctrl-0 = <&uart6_default>;
    label = "GPS";
};
```

## GPIO Node (Status LED)

```dts
&gpio1 {
    status = "okay";
    
    status_led: status-led {
        gpios = <5 GPIO_ACTIVE_HIGH>;   // PA5 (pin 5 of GPIO port A)
        label = "Status LED";
    };
};

/ {
    aliases {
        led0 = &status_led;             // Reference in code: DT_ALIAS(led0)
    };
};
```

## SPI Configuration (if needed later)

```dts
&spi1 {
    status = "okay";
    pinctrl-0 = <&spi1_default>;
    
    cs-gpios = <&gpio1 3 GPIO_ACTIVE_LOW>;   // PA3 as chip select
    
    w25q64: w25q64@0 {
        compatible = "jedec,spi-nor";
        reg = <0>;
        spi-max-frequency = <10000000>;       // 10 MHz
        size = <0x400000>;                    // 4 MB
        has-dpd;
        has-lock-bits;
        partitions {
            compatible = "fixed-partitions";
            #address-cells = <1>;
            #size-cells = <1>;
            
            data_partition: partition@0 {
                label = "data";
                reg = <0x0 0x010000>;         // 64 KB for ring buffer
            };
        };
    };
};
```

## Accessing DTS Nodes in C Code

**Get device from DTS**:
```c
const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(bme280));
const struct device *dev = DEVICE_DT_GET(DT_ALIAS(led0));
const struct device *dev = DEVICE_DT_GET(DT_PATH(gpio1));
```

**Check if device exists**:
```c
if (!device_is_ready(dev)) {
    printk("Device not found\n");
    return;
}
```

**Get GPIO spec from DTS**:
```c
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
gpio_pin_toggle_dt(&led);  // Toggle
```

## Common DTS Issues & Fixes

| Problem | Symptom | Solution |
|---------|---------|----------|
| Wrong I2C address | Device not found | Check AD0 (MPU6050) or SDO (BME280) pin state in HW |
| Missing compatible | Driver doesn't bind | Verify DTS `compatible` string matches driver binding |
| UART baud wrong | Garbled console output | Check UART node `current-speed = <115200>` |
| GPIO pin wrong | LED doesn't toggle | Verify `gpios = <PIN_NUM ...>` matches schematic |
| Node disabled | Device always null | Check `status = "okay"` in DTS node |
| Pinctrl undefined | Build error | Include `stm32f7xx-pinctrl.dtsi` at top |

## Verification Commands

```bash
# Check DTS compilation
west build -t dts_check

# View compiled device tree (after build)
cat build/zephyr/nucleof756zg.dtb | strings | grep -i uart3

# List all compatible strings (to find drivers)
grep -r "compatible" dts/

# Flash new DTS
west flash --runner stm32cubeprogrammer
```

---

**DTS is hardware abstraction. Change pins/addresses only in DTS, not code.**
