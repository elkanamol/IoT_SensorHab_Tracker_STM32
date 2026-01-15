# Common Tasks & Debugging

## Task: Add a New Sensor

### Steps

1. **Get sensor driver specs**
   - I2C address, clock speed, registers, sample format
   - Example: BME280 @ 0x77, 400 kHz I2C, pressure/temp/humidity

2. **Add to device tree** (boards/nucleo_f756zg.dts)
   ```dts
   &i2c1 {
       new_sensor@2e {
           compatible = "vendor,sensor-name";
           reg = <0x2e>;
       };
   };
   ```

3. **Update prj.conf**
   ```ini
   CONFIG_NEW_SENSOR=y
   CONFIG_NEW_SENSOR_TRIGGER=y
   ```

4. **Create sensor task** (src/sensors/new_sensor_task.c)
   ```c
   #include <zephyr/kernel.h>
   #include <zephyr/drivers/sensor.h>
   #include <zephyr/logging/log.h>
   
   LOG_MODULE_REGISTER(new_sensor);
   
   K_THREAD_DEFINE(new_sensor_tid, 512, new_sensor_task, NULL, NULL, NULL, 7, 0, 0);
   
   void new_sensor_task(void) {
       const struct device *dev = DEVICE_DT_GET_ANY(vendor_sensor_name);
       if (!device_is_ready(dev)) {
           LOG_ERR("Device not ready");
           return;
       }
       
       while (1) {
           struct sensor_value val[3];
           sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
           sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &val[0]);
           
           struct sensor_event event = {
               .timestamp_ms = k_uptime_get(),
               .accel_x = val[0].val1 + val[0].val2 / 1000000.0,
           };
           zbus_chan_pub(&sensor_data_channel, &event, K_NO_WAIT);
           
           k_msleep(1000);
       }
   }
   ```

5. **Build & test**
   ```bash
   west build && west flash
   # Check logs for "Device ready" message
   ```

---

## Task: Add a Shell Command

### Example: Temperature query command

1. **Create shell module** (src/shell_commands.c)
   ```c
   #include <zephyr/shell/shell.h>
   
   static int cmd_temp(const struct shell *sh, size_t argc, char **argv) {
       if (argc != 1) {
           shell_error(sh, "Usage: temp");
           return -EINVAL;
       }
       shell_print(sh, "Temperature: %.2f°C", last_temperature);
       return 0;
   }
   
   SHELL_CMD_REGISTER(temp, NULL, "Get current temperature", cmd_temp);
   ```

2. **Enable in prj.conf**
   ```ini
   CONFIG_SHELL=y
   CONFIG_SHELL_BACKEND_SERIAL=y
   CONFIG_SHELL_LOG_BACKEND=n  # Don't double-log
   ```

3. **Test**
   ```bash
   west build && west flash
   # Via terminal (picocom /dev/ttyACM0 at 115200):
   uart:~$ temp
   Temperature: 22.45°C
   ```

---

## Task: Debug Device Not Ready

### Symptom

`LOG_ERR("Device not ready")` message at startup or when accessing sensor.

### What Does It Mean?

**Device found but initialization failed.** The device tree node exists and matches a driver's `compatible` string, but the driver failed to initialize the hardware. Common causes:

- I2C/SPI bus not ready (no clock, pins not configured)
- Hardware doesn't respond (sensor not powered, wrong address)
- Driver misconfiguration (missing CONFIG option, bad pinctrl)

### Diagnosis Flowchart

**Step 1: Does device tree node exist?**

```bash
grep "&i2c1" build/zephyr/zephyr.dts
```

- ✅ **Found**: Continue to Step 2
- ❌ **Not found**: Add node to `boards/nucleo_f756zg.dts` (see device tree guide)

**Step 2: Does node have correct `compatible` string?**

```bash
grep -A 3 "bme280" build/zephyr/zephyr.dts
```

Expected output:
```
bme280: bme280@77 {
    compatible = "bosch,bme280";
```

- ✅ **Correct**: Continue to Step 3
- ❌ **Typo or missing**: Fix in `boards/nucleo_f756zg.dts`, must match driver binding

**Step 3: Is the driver enabled in prj.conf?**

```bash
grep "CONFIG_BME280\|CONFIG_I2C" build/zephyr/.config
```

Expected:
```
CONFIG_BME280=y
CONFIG_I2C=y
```

- ✅ **Both present**: Continue to Step 4
- ❌ **Missing**: Add to `prj.conf`:
  ```ini
  CONFIG_I2C=y
  CONFIG_BME280=y
  ```

**Step 4: Is I2C interface itself ready?**

Add diagnostic logging to see if I2C initialized:

```c
void bme280_task(void) {
    // Step 4A: Check I2C device exists
    const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c)) {
        LOG_ERR("I2C1 not ready - bus not initialized");
        return;
    }
    LOG_INF("I2C1 ready ✓");
    
    // Step 4B: Check sensor device
    const struct device *bme280 = DEVICE_DT_GET_ANY(bosch_bme280);
    if (bme280 == NULL) {
        LOG_ERR("Sensor not found in device tree");
        return;
    }
    LOG_INF("Sensor found ✓");
    
    // Step 4C: Check sensor initialization
    if (!device_is_ready(bme280)) {
        LOG_ERR("Sensor not ready. Check power, I2C address, pin config");
        LOG_ERR("Kernel status = %d", bme280->state);
        return;
    }
    LOG_INF("Sensor ready ✓");
}
```

- ✅ **I2C1 ready message**: Hardware configured correctly, continue to Step 5
- ❌ **I2C1 not ready**: Verify pinctrl in DTS for I2C1 (SDA, SCL pins)

**Step 5: Is sensor at correct I2C address?**

Run I2C scan (Phase 2 has template):

```bash
# After running i2c_scan from Phase 2
# Should see: "Found device @ 0x77" (BME280)
# If not: Check hardware schematic, verify sensor powered
```

- ✅ **Sensor detected at 0x77**: Hardware connection OK, likely driver init issue
- ❌ **Not detected**: Check power supply, solder joints, or wrong address configured

### Common Causes Summary

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| "Device not ready" + I2C1 OK | Sensor not responding | Check power, address, solder |
| "Device not ready" + I2C1 NOT ready | Bus not initialized | Verify I2C pinctrl in DTS |
| "Device not found" | Missing DTS node | Add node to boards/nucleo_f756zg.dts |
| "Device found but init fail" | Missing CONFIG | Add CONFIG_BME280=y to prj.conf |
| "No device found" (NULL) | Compatible string mismatch | Verify `compatible = "bosch,bme280"` |

---

---

## Task: Optimize Performance

### Reduce Response Latency

**Problem**: GPS fix takes 5+ seconds

**Solution**: Pre-allocate buffers, avoid dynamic memory
```c
// BAD: Allocates 256 bytes each fix
void gps_task(void) {
    while (1) {
        char *buffer = k_malloc(256);  // Slow, fragmentation
        parse_nmea(buffer);
        k_free(buffer);
    }
}

// GOOD: Static buffer, reuse
void gps_task(void) {
    static char buffer[256];  // Pre-allocated
    while (1) {
        parse_nmea(buffer);
    }
}
```

### Reduce Power Consumption

**Problem**: Battery drain 500 mA continuous

**Solution**: Use idle states
```ini
# prj.conf
CONFIG_PM=y
CONFIG_PM_DEVICE=y
CONFIG_TICKLESS_KERNEL=y
```

**Code**:
```c
void idle_task(void) {
    while (1) {
        k_sleep(K_SECONDS(60));  // Allow Tickless Idle
    }
}
```

### Monitor Memory Usage

```bash
west build
# Check .map file
grep "Total" build/zephyr/zephyr.map

# Or in prj.conf:
CONFIG_MEMORY_PROTECTION_UNIT=y
CONFIG_STACK_PROTECTION=y
CONFIG_THREAD_MONITOR=y
```

---

## Task: Test Zbus Connectivity

### Verify publishers & subscribers

```c
// Add to any task
void test_zbus(void) {
    struct sensor_event event = {
        .timestamp_ms = k_uptime_get(),
        .temperature_c = 25.5,
    };
    
    int ret = zbus_chan_pub(&sensor_data_channel, &event, K_NO_WAIT);
    LOG_INF("Publish result: %d (0=success)", ret);
}
```

### Check message queue

```c
ZBUS_OBSERVER_DEFINE(test_observer, 16);

void test_subscriber(void) {
    struct zbus_obs_node obs_node;
    zbus_chan_add_obs(&sensor_data_channel, &obs_node, K_FOREVER);
    
    struct sensor_event event;
    while (1) {
        int ret = zbus_chan_fetch(&sensor_data_channel, &event, K_MSEC(5000));
        if (ret == 0) {
            LOG_INF("Received: T=%.2f", event.temperature_c);
        } else {
            LOG_WRN("No message (timeout or error %d)", ret);
        }
    }
}
```

---

## Common Mistakes & Fixes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| **Struct size mismatch** | Garbage data | Check `sizeof(sensor_event)` matches publisher/subscriber |
| **Zbus queue full** | "Message dropped" warnings | Increase `CONFIG_ZBUS_MESSAGE_QUEUE_SIZE` |
| **Thread priority inverted** | Lower priority task blocks higher | Use `K_PRIO_COOP_FALL_BACK(n)` or mutex |
| **Device tree typo** | "Device not ready" | Verify exact `compatible` string in binding |
| **Logging enabled without backend** | No output | Add `CONFIG_LOG_BACKEND_UART=y` |
| **UART overflow** | Garbled output | Reduce CONFIG_LOG_DEFAULT_LEVEL or enable DMA |
| **Stack overflow** | Silent crash | Increase CONFIG_[TASK]_STACK_SIZE |
| **Missing K_THREAD_DEFINE** | Task never runs | Ensure K_THREAD_DEFINE in .c file (not .h) |
| **Incorrect pin mapping** | GPIO doesn't work | Check PINCTRL section in device tree |
| **I2C arbitration loss** | Sensor reads fail | Reduce clock speed (CONFIG_I2C_CLOCK_FREQUENCY) |

---

## Testing Checklist

### Unit Test (single module)

```c
// src/tests/test_bme280.c
#include <zephyr/ztest.h>
#include "../sensors/bme280.h"

ZTEST(sensors, test_bme280_read) {
    const struct device *dev = DEVICE_DT_GET_ANY(bosch_bme280);
    zassert_not_null(dev, "BME280 device not found");
    zassert_true(device_is_ready(dev), "BME280 not ready");
    
    struct sensor_value val;
    zassert_equal(0, sensor_sample_fetch(dev), "Sample fetch failed");
    zassert_equal(0, sensor_channel_get(dev, SENSOR_CHAN_TEMP, &val), "Get failed");
}
```

**Run**:
```bash
west build -t run
```

### Integration Test (Zbus)

```c
void test_sensor_to_datalogger(void) {
    struct sensor_event event = {
        .temperature_c = 22.5,
    };
    
    // Publish from sensor task
    int ret = zbus_chan_pub(&sensor_data_channel, &event, K_NO_WAIT);
    zassert_equal(0, ret, "Publish failed");
    
    // Datalogger task fetches
    struct sensor_event fetched;
    ret = zbus_chan_fetch(&sensor_data_channel, &fetched, K_MSEC(100));
    zassert_equal(0, ret, "Fetch failed");
    zassert_equal(22, (int)fetched.temperature_c, "Data mismatch");
}
```

---

## Debugging Tools

### Kernel Debug

```ini
# prj.conf
CONFIG_KERNEL_LOGGER=y
CONFIG_KERNEL_LOG_LEVEL=4  # DEBUG
CONFIG_THREAD_MONITOR=y
```

**View threads**:
```bash
# Via shell
uart:~$ kernel version
uart:~$ thread info
```

### Memory Inspector

```c
void debug_memory(void) {
    LOG_INF("Stack space: %d bytes", k_thread_stack_space_get(k_current_get()));
}
```

### Device Tree Verify

```bash
# View final DTS (generated at build)
cat build/zephyr/zephyr.dts | grep -A5 "&i2c1"
```

---

**Start simple: get 1 sensor working, then add others. Test each phase before moving forward.**
