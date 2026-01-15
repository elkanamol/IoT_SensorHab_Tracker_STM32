# Configuration Guide: FreeRTOS → Zephyr

## Overview

Zephyr uses **Kconfig** (build-time) and **prj.conf** (per-project). No runtime config files. All settings compile-time or flash-stored (Zephyr Settings subsystem).

---

## File Organization

```
project_root/
├── prj.conf              ← Main project config (device, modules, flags)
├── boards/
│   └── nucleo_f756zg.conf ← Board-specific overrides
├── Kconfig               ← Project Kconfig (if custom options needed)
└── src/
    └── config.h          ← App constants (can migrate to prj.conf)
```

---

## Core FreeRTOS → Zephyr Mapping

### OS Kernel

| FreeRTOS | Zephyr | prj.conf Setting |
|----------|--------|------------------|
| `#define configTICK_RATE_HZ 1000` | Default: 100 Hz | `CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=100` |
| `#define configMINIMAL_STACK_SIZE 128` | Stack per thread | `CONFIG_MAIN_STACK_SIZE=2048` |
| `#define configTOTAL_HEAP_SIZE 30000` | Dynamic heap | `CONFIG_HEAP_MEM_POOL_SIZE=30000` |
| `#define configUSE_PREEMPTION 1` | Default: enabled | N/A (built-in) |
| `#define configUSE_MUTEXES 1` | Built-in | N/A (always available) |

### Device Tree

| FreeRTOS | Zephyr |
|----------|--------|
| CubeMX → clock config | **boards/nucleo_f756zg.dts** |
| GPIO init (HAL_GPIO_Init) | GPIO device tree node |
| Pin alternate functions | `pinctrl-0 = <&...>` |
| Clock tree config | Device tree properties |

**FreeRTOS example**:
```c
// CubeMX generates
void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}
```

**Zephyr equivalent** (device tree):
```dts
&gpioe {
    led {
        gpios = <13 GPIO_ACTIVE_HIGH>;  // PE13
        label = "user_led";
    };
};
```

---

## I2C Configuration

### FreeRTOS (config.h)

```c
#define I2C_TIMEOUT_MS 100
#define BME280_I2C_ADDR 0x77
#define MPU6050_I2C_ADDR 0x68
#define I2C_CLOCK_SPEED 400000  // 400 kHz
```

### Zephyr (prj.conf)

```ini
# I2C 1: Sensors (BME280, MPU6050)
CONFIG_I2C=y
CONFIG_I2C_STM32=y

# Sensor drivers
CONFIG_BME280=y
CONFIG_BME280_MODE_NORMAL=y
CONFIG_MPU6050=y
CONFIG_MPU6050_ACCEL_RANGE_16G=y

# Logging
CONFIG_LOG=y
CONFIG_LOG_MODE_IMMEDIATE=y
```

### Device Tree (nucleo_f756zg.dts)

```dts
&i2c1 {
    pinctrl-0 = <&i2c1_scl_pb8 &i2c1_sda_pb9>;
    pinctrl-names = "default";
    status = "okay";
    clock-frequency = <400000>;  // 400 kHz
    
    bme280@77 {
        compatible = "bosch,bme280";
        reg = <0x77>;
    };
    
    mpu6050@68 {
        compatible = "invensense,mpu6050";
        reg = <0x68>;
    };
};
```

---

## UART Configuration

### FreeRTOS (config.h)

```c
#define UART2_BAUD 115200    // Modem RC7120
#define UART3_BAUD 115200    // Debug console
#define UART6_BAUD 9600      // GPS module
#define UART_DMA_ENABLED 1
```

### Zephyr (prj.conf)

```ini
# UART drivers
CONFIG_UART=y
CONFIG_SERIAL=y
CONFIG_UART_STM32=y

# UART 2: Modem
CONFIG_UART_2=y
CONFIG_UART_2_BAUD_RATE=115200

# UART 3: Debug
CONFIG_UART_3=y
CONFIG_UART_3_BAUD_RATE=115200

# UART 6: GPS
CONFIG_UART_6=y
CONFIG_UART_6_BAUD_RATE=9600

# DMA for UART
CONFIG_UART_ASYNC_API=y
CONFIG_DMA=y
CONFIG_DMA_STM32=y
```

### Device Tree (nucleo_f756zg.dts)

```dts
&uart2 {
    pinctrl-0 = <&uart2_tx_pa2 &uart2_rx_pa3>;
    pinctrl-names = "default";
    current-speed = <115200>;
    status = "okay";
};

&uart3 {
    pinctrl-0 = <&uart3_tx_pb10 &uart3_rx_pb11>;
    pinctrl-names = "default";
    current-speed = <115200>;
    status = "okay";
};

&uart6 {
    pinctrl-0 = <&uart6_tx_pc6 &uart6_rx_pc7>;
    pinctrl-names = "default";
    current-speed = <9600>;
    status = "okay";
};
```

---

## Flash Memory (W25Q64)

### FreeRTOS (config.h)

```c
#define FLASH_SIZE 65536           // 64 KB ring buffer
#define SECTOR_SIZE 4096           // 4 KB erase sector
#define WRITE_TIMEOUT_MS 500
```

### Zephyr (prj.conf)

```ini
# External flash driver
CONFIG_SPI=y
CONFIG_SPI_STM32=y
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_W25Q=y

# Settings subsystem (for flash storage)
CONFIG_NVS=y
CONFIG_NVS_LOG_LEVEL_DBG=y
```

### Device Tree

```dts
&spi1 {
    pinctrl-0 = <&spi1_sck_pa5 &spi1_mosi_pa7 &spi1_miso_pa6>;
    pinctrl-names = "default";
    cs-gpios = <&gpiod 4 GPIO_ACTIVE_LOW>;
    status = "okay";
    
    flash@0 {
        compatible = "jedec,spi-nor";
        reg = <0>;
        spi-max-frequency = <10000000>;  // 10 MHz
        size = <65536>;                   // 64 KB
        status = "okay";
    };
};
```

---

## Threading Configuration

### FreeRTOS (config.h)

```c
#define BME280_TASK_STACK 512
#define MPU6050_TASK_STACK 512
#define GPS_TASK_STACK 1024
#define DATALOGGER_TASK_STACK 2048
#define MODEM_MQTT_TASK_STACK 4096
#define SYSTEM_MONITOR_TASK_STACK 1024

#define BME280_TASK_PRIORITY 4
#define MPU6050_TASK_PRIORITY 4
#define GPS_TASK_PRIORITY 3
#define DATALOGGER_TASK_PRIORITY 5
#define MODEM_MQTT_TASK_PRIORITY 2
#define SYSTEM_MONITOR_TASK_PRIORITY 6
```

### Zephyr (prj.conf)

```ini
# Thread stack sizes
CONFIG_BME280_STACK_SIZE=512
CONFIG_MPU6050_STACK_SIZE=512
CONFIG_GPS_STACK_SIZE=1024
CONFIG_DATALOGGER_STACK_SIZE=2048
CONFIG_MODEM_MQTT_STACK_SIZE=4096
CONFIG_SYSTEM_MONITOR_STACK_SIZE=1024

# Priority 0 (highest) to 99 (lowest)
# Zephyr: lower number = higher priority (opposite of FreeRTOS)
CONFIG_BME280_PRIORITY=7
CONFIG_MPU6050_PRIORITY=7
CONFIG_GPS_PRIORITY=8
CONFIG_DATALOGGER_PRIORITY=6
CONFIG_MODEM_MQTT_PRIORITY=9
CONFIG_SYSTEM_MONITOR_PRIORITY=5
```

---

## Logging & Debug

### FreeRTOS (config.h)

```c
#define DEBUG_LEVEL 3       // 0=none, 1=error, 2=warn, 3=info, 4=debug
#define ENABLE_UART_LOG 1
#define LOG_BUFFER_SIZE 256
```

### Zephyr (prj.conf)

```ini
# Global logging
CONFIG_LOG=y
CONFIG_LOG_MODE_IMMEDIATE=y
CONFIG_LOG_PRINTK=y
CONFIG_PRINTK=y

# Default log level
CONFIG_LOG_DEFAULT_LEVEL=3  # 0=none, 1=err, 2=wrn, 3=inf, 4=dbg

# Module-specific levels
CONFIG_BME280_LOG_LEVEL=3
CONFIG_MPU6050_LOG_LEVEL=3
CONFIG_MODEM_LOG_LEVEL=4
CONFIG_ZBUS_LOG_LEVEL=3

# Backends
CONFIG_LOG_BACKEND_UART=y
CONFIG_LOG_BACKEND_UART_0=y
CONFIG_LOG_STRDUP_MAX_STRING=64
```

---

## Real-Time Constraints

### FreeRTOS (config.h)

```c
#define configUSE_TICKLESS_IDLE 0      // Power saving
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configCHECK_FOR_STACK_OVERFLOW 2
```

### Zephyr (prj.conf)

```ini
# Real-time performance
CONFIG_PREEMPT_ENABLED=y
CONFIG_THREAD_STACK_INFO=y
CONFIG_THREAD_NAME=y

# Stack checking (catch overflows)
CONFIG_STACK_PROTECTION=y

# Timing
CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC=216000000  # CPU clock
```

---

## Custom Configuration (Optional)

If you have app-specific constants, create **Kconfig** (top level):

```ini
menu "Application Configuration"

config BME280_SAMPLE_INTERVAL_MS
    int "BME280 sample interval (ms)"
    default 1000
    range 100 10000

config MQTT_PUBLISH_INTERVAL_S
    int "MQTT publish interval (seconds)"
    default 60
    range 10 3600

endmenu
```

Then reference in **prj.conf**:
```ini
CONFIG_BME280_SAMPLE_INTERVAL_MS=2000
CONFIG_MQTT_PUBLISH_INTERVAL_S=120
```

Access in C code:
```c
#include <zephyr/kernel.h>
#define SAMPLE_MS CONFIG_BME280_SAMPLE_INTERVAL_MS
k_msleep(SAMPLE_MS);
```

---

## Build Verification

**Verify configuration applied**:
```bash
# Generate config header (view all settings)
west build

# Check specific setting
grep "CONFIG_I2C_STM32" build/zephyr/.config

# View final device tree
cat build/zephyr/zephyr.dts
```

---

## Migration Checklist

- [ ] Create **prj.conf** with all core settings (I2C, UART, threads)
- [ ] Create **boards/nucleo_f756zg.dts** with all device nodes
- [ ] Update **config.h** → remove FreeRTOS, keep app constants only
- [ ] Test I2C scan: `west build && west flash && picocom /dev/ttyACM0`
- [ ] Check logs: all modules initialized
- [ ] Verify thread priorities (lower = higher priority in Zephyr)
- [ ] Test DMA: UART and SPI transfers work
- [ ] Build with warnings → 0 errors

---

**Zephyr: declarative config (prj.conf), not procedural (init functions).**
