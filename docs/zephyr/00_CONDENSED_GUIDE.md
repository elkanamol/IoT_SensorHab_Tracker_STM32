# Zephyr RTOS Migration - Condensed Documentation

## Overview

This condensed documentation provides essential information for implementing IoT SensorHub Tracker in Zephyr RTOS. Documents are organized by implementation phase order, with redundancy removed and instructions clarified.

---

## 1. QUICK START (READ FIRST)

### Architecture at a Glance

**Hardware**: STM32F756ZG with I2C1 (BME280@0x77, MPU6050@0x68), SPI1 (W25Q64), UART2/3/6 (Modem/Debug/GPS)

**Software**: 6 Zephyr threads → Zbus (6 channels) → Cloud

**Key Shift**: No ISR callbacks, no DMA management, no mutex code → Kernel handles transparently

---

## 2. HARDWARE PROTOCOLS & SPECIFICATIONS

Extract from FreeRTOS config.h. **Must know these values.**

### I2C Bus 1 (400 kHz, PB9/PB8)
| Device | Address | Protocol | Notes |
|--------|---------|----------|-------|
| BME280 | 0x77 | I2C | Env: temp, humidity, pressure |
| MPU6050 | 0x68 | I2C | IMU: accel (±16g), gyro (±2000°/s) |

**Oversampling** (BME280):
- Temperature: 2x
- Pressure: 4x
- Humidity: 1x

### SPI Bus 1 (10 MHz)
| Device | Function | Size | Notes |
|--------|----------|------|-------|
| W25Q64 | Flash memory | 64 KB ring buffer | 4KB sectors, 256-byte pages |

### UART Interfaces
| Port | Baud | Device | Protocol |
|------|------|--------|----------|
| UART2 | 115200 | RC7120 modem | AT commands, MQTT |
| UART3 | 115200 | Debug console | Logging, shell |
| UART6 | 9600 | Neo-6M GPS | NMEA 0183 |

---

## 3. DESIGN PRINCIPLES

### Three Core Concepts

1. **Kernel-Managed Hardware**: Drivers abstract ISR/DMA. No app-level callbacks.
2. **Pub/Sub Messaging**: 6 Zbus channels replace FreeRTOS queues.
3. **Device Tree Configuration**: Hardware settings in .dts files, not code.

### Zbus Channels (Message Pub/Sub)

| Channel | Publisher | Data | Frequency |
|---------|-----------|------|-----------|
| sensor_data | BME280, MPU6050 | T/H/P/Accel/Gyro | 1-2 Hz |
| gps_fix | GPS task | Lat/Lon/Alt/Satellites | 0-1 Hz |
| mqtt_publish | Datalogger | Aggregated data | Every 60s |
| log_data | Datalogger | Record for flash | Every 60s |
| modem_event | Modem task | Connection state | ~1 Hz |
| system_event | All tasks | Errors, health | Variable |

---

## 4. IMPLEMENTATION PHASES (8 SEQUENTIAL)

Each phase has **Objective → Components → Deliverables → Validation**.

### Phase 0: Kickoff
**Objective**: Build system works, LED blinks.
- **Deliverables**: CMakeLists.txt, prj.conf (minimal), GPIO node in DTS
- **Validate**: `west build` succeeds, LED blinks at 1 Hz
- **Files**: main.c (blinky), boards/nucleo_f756zg.dts

### Phase 1: UART & Logging
**Objective**: Debug console operational.
- **Deliverables**: UART3 driver node (DTS), logging configured
- **Validate**: `LOG_INF()` messages visible @ 115200 baud
- **Config**: `CONFIG_LOG=y`, `CONFIG_LOG_MODE_IMMEDIATE=y`

### Phase 2: I2C Bus
**Objective**: I2C1 responsive, sensor addresses detected.
- **Deliverables**: I2C1 node (DTS), sensor device nodes (BME280@0x77, MPU6050@0x68)
- **Validate**: `i2c_read()` returns WHO_AM_I registers (0xA1 for BME280, 0x68 for MPU6050)
- **Config**: `CONFIG_I2C=y`, `CONFIG_I2C_STM32_INTERRUPT=y`

### Phase 3: BME280 Sensor Task
**Objective**: Read environmental data, publish to Zbus.
- **Deliverables**: bme280_task.c, publishes sensor_data_channel
- **Validate**: Plausible temp/humidity/pressure values in logs
- **Task**: Read every 1000ms, publish via Zbus

### Phase 4: MPU6050 Sensor Task
**Objective**: Read IMU data, publish to Zbus.
- **Deliverables**: mpu6050_task.c, publishes sensor_data_channel
- **Validate**: Accel/gyro values change with board motion
- **Task**: Read every 1000ms, share I2C1 (kernel-serialized)

### Phase 5: Flash Storage
**Objective**: Ring buffer writes, data persistence.
- **Deliverables**: datalogger_task.c, ring buffer implementation
- **Validate**: Data written to flash, survives power cycle
- **Task**: Aggregate sensor data, write every 60s, read/verify

### Phase 6: GPS Task
**Objective**: Parse NMEA, extract position data.
- **Deliverables**: gps_task.c, NMEA parser, publishes gps_fix_channel
- **Validate**: Correct lat/lon when outdoors (or simulator)
- **Task**: UART6 async RX, parse, publish when fix valid

### Phase 7: Modem & MQTT
**Objective**: Cloud transmission via cellular.
- **Deliverables**: modem_task.c, AT command handler, MQTT publish
- **Validate**: Data appears on ThingSpeak, MQTT ACK received
- **Task**: AT commands (connect, publish), use modem subsystem

### Phase 8: Integration & Testing (1-2 weeks)
**Objective**: Full system stable.
- **Deliverables**: Error handling, task synchronization, watchdog
- **Validate**: No crashes, data continuous, recovery on failure
- **Task**: System monitor, exponential backoff, graceful degradation

---

## 5. DEVICE TREE ORGANIZATION

**File Structure**:
```
boards/nucleo_f756zg/
├── nucleo_f756zg.dts       (root, includes others)
├── nucleo_f756zg.yaml      (board metadata)
└── Kconfig.board           (board-specific Kconfig)

dts/
├── custom_sensors.dtsi     (BME280, MPU6050 nodes)
├── custom_uarts.dtsi       (UART overrides)
└── custom_pinctrl.dtsi     (pin muxing)
```

**Example: I2C Sensor Nodes** (custom_sensors.dtsi)
```dts
&i2c1 {
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

**Key Point**: Device tree binds hardware to drivers automatically. No code registration needed.

---

## 6. THREAD MODEL & DATA FLOW

### 6 Concurrent Threads

```
bme280_task     ─┐
mpu6050_task    ─┼─→ sensor_data_channel ─→ datalogger_task ─→ [Flash Ring Buffer]
                 │                                           ├─→ [MQTT Queue]
gps_task        ─┴─→ gps_fix_channel ────────────────────────┘

datalogger_task ─→ mqtt_publish_channel → modem_mqtt_task → [ThingSpeak]

system_monitor  ─→ system_event_channel (health checks)
```

**Thread Stack Sizes** (from FreeRTOS analysis):
- bme280_task: 1024 words
- mpu6050_task: 1024 words
- gps_task: 1536 words
- datalogger_task: 2048 words
- modem_mqtt_task: 1024 words
- system_monitor: 512 words

**Total**: ~8KB (RAM usage minimal)

---

## 7. CONFIGURATION CONVERSION

Convert FreeRTOS config.h → Zephyr prj.conf

**Example Mappings**:

| FreeRTOS | Zephyr (prj.conf) |
|----------|------------------|
| CONFIG_BME280_MEASUREMENT_INTERVAL_MS=1000 | `CONFIG_APP_BME280_INTERVAL_MS=1000` |
| CONFIG_MQTT_BROKER_HOSTNAME="..." | `CONFIG_APP_MQTT_HOST="..."` |
| CONFIG_TASK_STACK_SIZE_MQTT=1024 | `CONFIG_APP_MODEM_STACK=1024` |

**Kconfig Template** (zephyr/Kconfig):
```
config APP_BME280_INTERVAL_MS
    int "BME280 read interval (ms)"
    default 1000

config APP_MQTT_HOST
    string "MQTT broker hostname"
    default "mqtt3.thingspeak.com"
```

---

## 8. ZBUS MESSAGE STRUCTURES

Define in `src/messages.h`:

```c
struct sensor_event {
    uint64_t timestamp_ms;
    float temperature_c, humidity_pct, pressure_pa;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
};

struct gps_event {
    uint64_t timestamp_ms;
    float latitude_deg, longitude_deg, altitude_m;
    bool fix_valid;
    uint8_t satellites;
};

struct mqtt_event {
    uint64_t timestamp_ms;
    float temperature_c, humidity_pct, pressure_pa;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float latitude_deg, longitude_deg, altitude_m;
    bool gps_valid;
};
```

**Register Channels** (messages.c):
```c
ZBUS_CHAN_DEFINE(sensor_data_channel, struct sensor_event, ...);
ZBUS_CHAN_DEFINE(gps_fix_channel, struct gps_event, ...);
ZBUS_CHAN_DEFINE(mqtt_publish_channel, struct mqtt_event, ...);
// ... (3 more channels)
```

---

## 9. CRITICAL DESIGN DECISIONS

| Decision | Why | Trade-off |
|----------|-----|-----------|
| Single Zbus (not multiple queues) | Loose coupling, scalable | Single point of congestion (unlikely) |
| Kernel-managed DMA/ISR | Simpler code, no bugs | Less control, performance opaque |
| Device tree hardware config | Version-controllable, reusable | Learning curve for DTS syntax |
| 400 kHz I2C (not 1 MHz) | Reliable, proven in FreeRTOS | Slower transactions (~200 µs each) |
| Ring buffer on W25Q64 (not EEPROM) | Large capacity (64KB), cheap | Wear leveling complexity |
| UART async (not polling) | CPU efficient | More complex GPS parsing |

---

## 10. ERROR HANDLING STRATEGY

**Per-Sensor Recovery**:
- 3 consecutive read failures → retry with exponential backoff (100ms, 500ms, 2s)
- 5 failures → publish system_event_channel with error, skip this cycle
- N consecutive errors → log critical, continue (don't crash task)

**System Monitor**:
- Watch thread activity, CPU usage, queue depths
- If modem offline → log warning, skip MQTT (keep logging local)
- If sensors offline → health indicator blink pattern

**Graceful Degradation**:
- Partial failure → keep collecting available data
- Total failure → restart task, log event, alert user (LED pattern)

---

## 11. BUILD & FLASH COMMANDS

```bash
# Setup (first time)
west init -m https://github.com/zephyrproject-rtos/zephyr nucleo_zephyr
cd nucleo_zephyr
west update

# Build
cd nucleo_f756_zephyr_app
west build -b nucleo_f756zg

# Flash
west flash --runner stm32cubeprogrammer

# Monitor
picocom /dev/ttyACM0 -b 115200
```

---

## 12. TESTING & VALIDATION CHECKLIST

### Phase-by-Phase Validation

**Phase 0** ✓
- [ ] Build succeeds without warnings
- [ ] LED blinks steady 1 Hz
- [ ] No console errors

**Phase 1** ✓
- [ ] `LOG_INF("test")` appears in console
- [ ] Serial monitor @ 115200 baud receives data

**Phase 2** ✓
- [ ] I2C scan detects 0x77, 0x68
- [ ] `i2c_read()` returns correct WHO_AM_I values

**Phase 3** ✓
- [ ] BME280 reads every 1 second
- [ ] Temperature/humidity plausible
- [ ] Values published to sensor_data_channel

**Phase 4** ✓
- [ ] MPU6050 reads every 1 second
- [ ] Accel values 0-1g at rest, change with motion
- [ ] Gyro zeroed at rest

**Phase 5** ✓
- [ ] Data writes to flash every 60 seconds
- [ ] Power cycle: data persists
- [ ] CRC verification passes

**Phase 6** ✓
- [ ] GPS generates valid NMEA
- [ ] Correct lat/lon when outdoors
- [ ] Publish to gps_fix_channel

**Phase 7** ✓
- [ ] Modem connects to network
- [ ] MQTT publish succeeds
- [ ] Data visible on ThingSpeak

**Phase 8** ✓
- [ ] 24-hour stability test: 0 crashes
- [ ] All sensors continuous
- [ ] Network recovery tested (modem offline → resume)

---

## 13. COMMON MISTAKES & SOLUTIONS

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Missing DTS node | Driver never binds | Add node to .dts, verify compatible string |
| I2C address wrong | i2c_read() NAK/timeout | Check AD0 pin (MPU6050), SDO pin (BME280) |
| Zbus message size mismatch | Compile error | Ensure struct definition matches ZBUS_CHAN_DEFINE |
| Task never created | Code exists but not running | Check k_thread_create() in main.c |
| Modem AT timeout | "Command timeout" | Check UART baud (115200), modem powered on |
| GPS no fix | gps_valid=false | GPS needs clear sky view or simulator |
| Ring buffer full | Oldest data overwritten | Increase CONFIG_DATALOGGER_FLASH_RING_BUFFER_SIZE |

---

## 14. EXTERNAL REFERENCES

- **Zephyr Official**: https://docs.zephyrproject.org/
- **Zephyr API**: https://docs.zephyrproject.org/latest/develop/api/
- **Device Tree**: https://docs.zephyrproject.org/latest/build/dts/index.html
- **Zbus**: https://docs.zephyrproject.org/latest/services/zbus.html
- **STM32F756**: https://www.st.com/datasheet/stm32f756
- **BME280**: https://www.bosch-sensortec.com/media/boschsensortec_content/bst/products/datasheets/bst-bme280-ds002.pdf
- **MPU6050**: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

---

**This condensed guide contains essentials. Reference specific sections for detail.**

For detailed architecture, phases, or DTS syntax: see original documents.
For quick answers: use table of contents above.
