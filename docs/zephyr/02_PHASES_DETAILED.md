# Phase-by-Phase Implementation Guide

Each phase is self-contained with Objective → Components → Deliverables → Validation.

**Folder structure to create** (incrementally):
```
src/
├── main.c (Phase 0, grows through Phase 8)
├── messages.c (Phase 1: Zbus channels)
├── messages.h
├── sensors/
│   ├── bme280_task.c (Phase 3)
│   ├── mpu6050_task.c (Phase 4)
│   └── sensor_common.h
├── storage/datalogger_task.c (Phase 5)
├── gps/gps_task.c (Phase 6)
└── modem/modem_mqtt_task.c (Phase 7)

boards/nucleo_f756zg/
├── nucleo_f756zg.dts (all nodes, grows)
└── CMakeLists.txt (optional)

CMakeLists.txt (workspace root, Phase 0)
prj.conf (workspace root, Phase 0, grows)
```

---

## Phase 0: Kickoff

**Objective**: Build system works. LED blinks at 1 Hz.

**Components**:
- Zephyr SDK installed (ARM GCC, west)
- STM32 HAL abstraction layer
- GPIO driver

**Deliverables** (files to create):
- **CMakeLists.txt** — workspace root (new)
- **prj.conf** — workspace root (new)
- **boards/nucleo_f756zg/nucleo_f756zg.dts** — boards/ dir + file (new)
- **src/main.c** — src/ dir + file (new)

**Validation**:
- [ ] `west build` completes, 0 errors
- [ ] Executable < 30 KB (flash only, not RAM)
- [ ] LED (PA5) blinks visibly at 1 Hz
- [ ] Power on/off: LED blinks immediately

**Code Template** (src/main.c):
```c
void main(void) {
    const struct device *led = DEVICE_DT_GET(DT_ALIAS(led0));
    
    while (1) {
        gpio_pin_toggle_dt(&led_spec);  // LED toggle
        k_msleep(500);                   // 500ms = 1 Hz blink
    }
}
```

**DTS Template** (custom LED node):
```dts
&gpio1 {
    status_led: status-led {
        gpios = <5 GPIO_ACTIVE_HIGH>;
        label = "Status LED";
    };
};

/ {
    aliases {
        led0 = &status_led;
    };
};
```

---

## Phase 1: UART & Logging

**Objective**: Debug console at 115200 baud. `LOG_INF()` messages visible.

**Components**:
- UART3 driver (STM32 UART)
- Logging subsystem (Zephyr native)
- Console backend

**Deliverables** (files to create/edit):
- **boards/nucleo_f756zg/nucleo_f756zg.dts** — edit: add UART3 node
- **prj.conf** — edit: append CONFIG_LOG + CONFIG_UART settings
- **src/main.c** — edit: add LOG_INF() at startup

**Validation**:
- [ ] Serial monitor @ 115200 receives startup message
- [ ] `LOG_INF("test")` appears in console
- [ ] `LOG_DBG()` visible when CONFIG_LOG_LEVEL=4
- [ ] No garbage characters (baud rate confirmed)
- [ ] Timestamps optional (can disable for speed)

**prj.conf**:
```
CONFIG_UART=y
CONFIG_LOG=y
CONFIG_LOG_MODE_IMMEDIATE=y
CONFIG_LOG_BACKEND_UART=y
CONFIG_LOG_DEFAULT_LEVEL=3
```

**DTS** (custom_uarts.dtsi):
```dts
&uart3 {
    status = "okay";
    current-speed = <115200>;
    pinctrl-0 = <&uart3_default>;
};
```

---

## Phase 2: I2C Infrastructure

**Objective**: I2C1 responsive. Sensor addresses 0x77 and 0x68 detected.

**Components**:
- I2C1 driver (STM32 I2C, 400 kHz)
- BME280 & MPU6050 device nodes
- I2C bus scanning utility

**Deliverables**:
- `dts/custom_sensors.dtsi` (I2C device nodes)
- `src/i2c_scan.c` (utility to list addresses)
- `src/main.c` (call i2c_scan on startup)

**Validation**:
- [ ] `i2c_scan` output shows `0x68` and `0x77`
- [ ] No timeout/NAK errors in log
- [ ] WHO_AM_I reads return correct values (0xA1 for BME280, 0x68 for MPU6050)
- [ ] Repeated scans stable (no intermittent failures)

**DTS** (custom_sensors.dtsi):
```dts
&i2c1 {
    status = "okay";
    
    bme280: bme280@77 {
        compatible = "bosch,bme280";
        reg = <0x77>;
    };
    
    mpu6050: mpu6050@68 {
        compatible = "invensense,mpu6050";
        reg = <0x68>;
    };
};
```

**Code** (i2c_scan.c):
```c
void i2c_scan(void) {
    const struct device *dev = DEVICE_DT_GET(DT_ALIAS(i2c_0));
    uint8_t buf[1];
    
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        int ret = i2c_read(dev, buf, 1, addr);
        if (ret == 0) {
            printk("Found device @ 0x%02x\n", addr);
        }
    }
}
```

---

## Phase 3: BME280 Sensor

**Objective**: BME280 reads every 1 second. Data plausible. Publishes to Zbus.

**Components**:
- BME280 driver (Zephyr sensor subsystem)
- Zbus channel (sensor_data_channel)
- Thread for periodic reads

**Deliverables**:
- `src/messages.h` (Zbus struct definitions)
- `src/messages.c` (Zbus channel registration)
- `src/sensors/bme280_task.c` (read loop)
- `src/main.c` (spawn thread)

**Validation**:
- [ ] Temperature ~22-25°C (room temp)
- [ ] Humidity 30-60% (depends on environment)
- [ ] Pressure 1000-1020 hPa (sea level ±100m)
- [ ] Values update every 1 second
- [ ] Zbus publish succeeds (return 0 from zbus_chan_pub)

**Message** (messages.h):
```c
struct sensor_event {
    uint64_t timestamp_ms;
    float temperature_c;
    float humidity_pct;
    float pressure_pa;
    // ... more fields for MPU6050 later
};
```

**Thread** (bme280_task.c):
```c
void bme280_task(void *unused) {
    const struct device *bme280 = DEVICE_DT_GET(DT_NODELABEL(bme280));
    
    while (1) {
        struct sensor_channels data;
        sensor_channel_get(bme280, SENSOR_CHAN_AMBIENT_TEMP, &data);
        
        struct sensor_event event = {
            .timestamp_ms = k_uptime_get(),
            .temperature_c = sensor_value_to_double(&data),
        };
        
        zbus_chan_pub(&sensor_data_channel, &event, K_NO_WAIT);
        k_msleep(1000);
    }
}
```

---

## Phase 4: MPU6050 Sensor

**Objective**: MPU6050 reads every 1 second. Accel/gyro plausible. Shares I2C1.

**Components**:
- MPU6050 driver (Zephyr sensor subsystem)
- Shared I2C1 (kernel handles serialization)
- Continuation of sensor_data_channel publishes

**Deliverables**:
- `src/sensors/mpu6050_task.c` (read loop)
- Update `messages.h` (add accel/gyro fields)

**Validation**:
- [ ] Accel ~9.8 m/s² on Z-axis (gravity)
- [ ] Accel X/Y ~0 when level
- [ ] Gyro X/Y/Z ~0 when stationary
- [ ] Values change when board moved/rotated
- [ ] Both sensors read without timeouts (I2C not congested)

**Key Point**: Kernel serializes I2C1 access. No app-level mutex code.

**Thread** (mpu6050_task.c):
```c
void mpu6050_task(void *unused) {
    const struct device *mpu6050 = DEVICE_DT_GET(DT_NODELABEL(mpu6050));
    
    while (1) {
        struct sensor_channels accel, gyro;
        sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_XYZ, &accel);
        sensor_channel_get(mpu6050, SENSOR_CHAN_GYRO_XYZ, &gyro);
        
        // Publish to sensor_data_channel (datalogger subscribes)
        k_msleep(1000);
    }
}
```

---

## Phase 5: Flash Ring Buffer

**Objective**: Data written to W25Q64. Survives power cycle. Ring buffer working.

**Components**:
- W25Q64 SPI driver
- Ring buffer algorithm (circular FIFO)
- Datalogger task (aggregator)
- Flash write synchronization

**Deliverables**:
- `src/io/datalogger.c` (ring buffer, aggregation)
- `src/io/datalogger.h` (public API)
- Update Zbus (log_data_channel)

**Validation**:
- [ ] Data written to flash every 60s
- [ ] Power cycle: data persists
- [ ] Ring buffer wraps (oldest data overwritten)
- [ ] CRC check passes (if implemented)
- [ ] Read-back matches written data

**Algorithm**:
```
Ring buffer [64 KB] addresses 0x00000 - 0x0FFFF
- Head pointer: where next write occurs
- Tail pointer: oldest unread record
- When head wraps: oldest records overwritten
- Records: [timestamp | temp | humidity | pressure | accel_x | ... | crc]
```

**datalogger_task**:
- Subscribe to sensor_data_channel (all sensor reads)
- Subscribe to gps_fix_channel (GPS data)
- Aggregate every 60 seconds into one record
- Write record to ring buffer
- Publish to mqtt_publish_channel (if connected)

---

## Phase 6: GPS Integration

**Objective**: NMEA parsing working. Valid fix detected. Lat/lon correct.

**Components**:
- UART6 driver (async RX with DMA)
- NMEA parser (lwgps or native)
- GPS thread
- Zbus gps_fix_channel

**Deliverables**:
- `src/sensors/gps_task.c` (RX handler, parser)
- Update `messages.h` (gps_event struct)
- DTS UART6 node

**Validation**:
- [ ] UART6 receives NMEA sentences (`$GPGGA...`)
- [ ] Sentences parsed without corruption
- [ ] Lat/lon extracted and printed
- [ ] `gps_fix_channel` published when fix valid
- [ ] Correct coordinates (test outdoors or simulator)

**GPS Data Format**:
- **Lat**: -90 to +90 degrees
- **Lon**: -180 to +180 degrees
- **Alt**: meters above mean sea level
- **Satellites**: 0-24
- **Fix Quality**: 0=no fix, 1=GPS fix, 2=DGPS fix

**NMEA Example**:
```
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
         time    lat      lon      fix sats hdop alt
```

---

## Phase 7: Modem & MQTT

**Objective**: Modem connects to network. MQTT publishes to ThingSpeak. Data visible.

**Components**:
- RC7120 AT command handler
- Zephyr modem subsystem (UART2)
- MQTT client library (or AT+MQTT commands)
- Modem task & state machine

**Deliverables**:
- `src/modem/modem_task.c` (AT handler, state machine)
- `src/modem/mqtt.c` (publish wrapper)
- DTS UART2 node

**Validation**:
- [ ] Modem initializes (`AT OK`)
- [ ] Network registered (SIM + antenna)
- [ ] MQTT connects to broker
- [ ] First publish succeeds
- [ ] Data visible on ThingSpeak dashboard
- [ ] MQTT ACK received

**AT Commands** (RC7120):
```
AT                          # Test
AT+CFUN=1                   # Power on
AT+CPIN?                    # SIM check
AT+CREG?                    # Network registration
AT+CGACT=1,1                # Activate PDP
AT+CMQTTNEW                 # MQTT new
AT+CMQTTSTART               # MQTT start
AT+CMQTTSUB=0,1,"topic",1   # Subscribe
AT+CMQTTPUB=0,0,0,"data"    # Publish
```

**State Machine**:
```
POWER_OFF → INITIALIZING → SEARCHING_NETWORK → 
REGISTERED → CONNECTING_MQTT → CONNECTED → 
(PUBLISH_LOOP) → DISCONNECT → SLEEP
```

---

## Phase 8: Integration & Hardening (1-2 weeks)

**Objective**: Full system stable. All sensors continuous. Graceful degradation.

**Components**:
- System monitor task (health checks)
- Error recovery (exponential backoff)
- Watchdog timer (kernel restart on deadlock)
- Performance monitoring (optional)

**Deliverables**:
- `src/system/system_monitor.c` (health task)
- `src/system/errors.c` (recovery strategies)
- Error counters & logging
- 24-hour test harness

**Validation**:
- [ ] No crashes during extended operation
- [ ] All sensors reading continuously
- [ ] Modem recovers from disconnect
- [ ] Flash ring buffer never corrupts
- [ ] GPS optional (ok if no fix)
- [ ] System monitor alerts working
- [ ] Watchdog never triggers (if configured)

**Health Checks**:
- Thread stacks: Check watermark, alert if > 80%
- Queue depths: Alert if > threshold
- I2C errors: Count, alert if > N per minute
- Modem status: Check AT connectivity
- Flash status: Verify latest record timestamp

**Recovery**:
- Sensor read failure: Retry 3x with backoff
- Modem offline: Skip MQTT, continue local logging
- I2C timeout: Reset bus (if driver supports)
- Task deadlock: Watchdog timer restarts MCU

---

## Testing Checklist

- [ ] Each phase has validation criteria
- [ ] Code compiles with 0 warnings
- [ ] No undefined references or missing headers
- [ ] Static analysis (if available) passes
- [ ] Integration tests verify Zbus message flow
- [ ] Load test: All threads active simultaneously
- [ ] Stress test: Sensor reads + MQTT publish + GPIO
- [ ] Power cycle: Data persists, recovery smooth

---

## Build & Flash Workflow

```bash
# Build current phase
west build -b nucleo_f756zg

# Flash
west flash --runner stm32cubeprogrammer

# Monitor
picocom /dev/ttyACM0 -b 115200  # Linux/Mac
# or
putty /dev/COM3 -serial -sercfg 115200  # Windows

# Rebuild on changes
west build --no-cmake  # Fast rebuild
```

---

**Each phase is a natural stopping point. Tag in git: `v1-phase0`, `v2-phase1`, etc.**
