# Zbus Pub/Sub Messaging

## Overview

Zbus replaces FreeRTOS queues. Single pub/sub hub with 6 message channels. Publishers unaware of subscribers. Subscribers notified automatically.

## 6 Message Channels

### 1. sensor_data_channel
**Data**: Environmental + motion sensor readings
```c
struct sensor_event {
    uint64_t timestamp_ms;
    float temperature_c, humidity_pct, pressure_pa;    // BME280
    float accel_x, accel_y, accel_z;                   // MPU6050
    float gyro_x, gyro_y, gyro_z;
};
```
**Publishers**: bme280_task, mpu6050_task  
**Subscribers**: datalogger_task, system_monitor_task  
**Frequency**: 1-2 Hz

### 2. gps_fix_channel
**Data**: Position, only published when fix is valid
```c
struct gps_event {
    uint64_t timestamp_ms;
    float latitude_deg, longitude_deg, altitude_m;
    uint8_t num_satellites;
    bool fix_valid;
};
```
**Publishers**: gps_task  
**Subscribers**: datalogger_task, modem_mqtt_task  
**Frequency**: 0-1 Hz (depends on outdoor access)

### 3. mqtt_publish_channel
**Data**: Aggregated data ready for cloud
```c
struct mqtt_event {
    uint64_t timestamp_ms;
    float temperature_c, humidity_pct, pressure_pa;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float latitude_deg, longitude_deg, altitude_m;
    bool gps_valid;
};
```
**Publishers**: datalogger_task  
**Subscribers**: modem_mqtt_task  
**Frequency**: Every 60 seconds

### 4. log_data_channel
**Data**: Record for persistent flash storage
```c
struct log_record {
    uint64_t timestamp_ms;
    uint32_t record_id;
    // ... all sensor data
    uint32_t crc;
};
```
**Publishers**: datalogger_task  
**Subscribers**: (flash writer in datalogger task)  
**Frequency**: Every 60 seconds

### 5. modem_event_channel
**Data**: Network & MQTT state changes
```c
struct modem_event {
    uint64_t timestamp_ms;
    enum { OFFLINE, SEARCHING, REGISTERED, CONNECTED, ERROR } state;
    uint8_t signal_strength;  // 0-31
};
```
**Publishers**: modem_mqtt_task  
**Subscribers**: system_monitor_task  
**Frequency**: State changes + periodic (every 60s)

### 6. system_event_channel
**Data**: Errors, warnings, health alerts
```c
struct system_event {
    uint64_t timestamp_ms;
    enum { INFO, WARNING, ERROR, CRITICAL } severity;
    char message[64];
};
```
**Publishers**: All tasks  
**Subscribers**: system_monitor_task, logger  
**Frequency**: On-demand

---

## Message Registration (messages.c)

```c
#include <zephyr/zbus.h>

// Define message types (in messages.h)
struct sensor_event { ... };
struct gps_event { ... };
// ... etc

// Register channels (messages.c)
ZBUS_CHAN_DEFINE(
    sensor_data_channel,
    struct sensor_event,
    NULL,
    NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0)
);

ZBUS_CHAN_DEFINE(
    gps_fix_channel,
    struct gps_event,
    NULL,
    NULL,
    ZBUS_OBSERVERS(
        ZBUS_OBSERVER_NAMED(datalogger_listener),
        ZBUS_OBSERVER_NAMED(mqtt_listener)
    ),
    ZBUS_MSG_INIT(0)
);

// ... (4 more channels)
```

---

## Publishing Messages

**From sensor task**:
```c
void bme280_task(void) {
    struct sensor_event event = {
        .timestamp_ms = k_uptime_get(),
        .temperature_c = 22.5,
        .humidity_pct = 45.0,
        .pressure_pa = 101325.0,
    };
    
    int ret = zbus_chan_pub(&sensor_data_channel, &event, K_NO_WAIT);
    if (ret != 0) {
        LOG_ERR("Failed to publish: %d", ret);
    }
}
```

**With timeout** (wait if queue full):
```c
zbus_chan_pub(&mqtt_publish_channel, &event, K_MSEC(100));
```

---

## Subscribing to Messages

**Static subscription in Kconfig**:
```c
ZBUS_OBSERVER_DEFINE(datalogger_listener, 8);  // 8 = queue depth
```

**Subscribe in task**:
```c
void datalogger_task(void) {
    struct zbus_obs_node obs_node;
    zbus_chan_add_obs(&sensor_data_channel, &obs_node, K_FOREVER);
    
    struct sensor_event event;
    while (1) {
        zbus_chan_fetch(&sensor_data_channel, &event, K_FOREVER);
        LOG_INF("Temp: %.2f°C", event.temperature_c);
    }
}
```

**Or with listener callback**:
```c
static void sensor_listener(const struct zbus_channel *chan) {
    const struct sensor_event *event = zbus_chan_const_msg(chan);
    LOG_INF("Received: T=%.2f", event->temperature_c);
}

ZBUS_CHAN_ADD_OBS(sensor_data_channel, sensor_listener, 0);
```

---

## Queue Depth Sizing

Each channel has a queue (buffer size). If queue full, message is dropped (default) or sender blocks.

**Calculation**:
- Message size: ~60 bytes (sensor_event)
- Subscribers: 2 (datalogger, monitor)
- Frequency: 1-2 Hz
- Queue depth: 8-16 (enough for 8-16 second burst)
- RAM per channel: 60 * 16 = 960 bytes

**Total for 6 channels**: ~6 KB

**Zephyr default**: 16 messages per channel. Adjust if needed:
```c
#define CONFIG_ZBUS_MESSAGE_QUEUE_SIZE 32
```

---

## Error Handling

**If message dropped**:
```c
int ret = zbus_chan_pub(&channel, &event, K_NO_WAIT);
if (ret == -ENOMSG) {
    LOG_WRN("Message queue full, dropped");
}
```

**If subscriber slow**:
```c
// Increase queue depth in ZBUS_OBSERVER_DEFINE
ZBUS_OBSERVER_DEFINE(slow_listener, 32);  // 32 messages buffered
```

**If message corrupted**:
```c
// Use CRC or sequence numbers (optional)
struct mqtt_event {
    uint64_t timestamp_ms;
    uint32_t sequence;
    float temperature_c;
    uint32_t crc;  // CRC32 of all fields before this
};
```

---

## FreeRTOS vs Zephyr Comparison

| Aspect | FreeRTOS | Zephyr Zbus |
|--------|----------|-------------|
| **IPC** | `xQueueCreate()` for each pair | Single hub, 6 channels |
| **Coupling** | Tight (sender knows receiver) | Loose (publish once) |
| **Subscribers** | New queue for each | Automatic broadcast |
| **Config** | Runtime dynamic | Build-time static |
| **Scaling** | N queues for N tasks | Fixed 6 channels |
| **Thread-safe** | Yes (kernel serializes) | Yes (kernel serializes) |

**Example FreeRTOS**:
```c
xQueueCreate(10, sizeof(SensorData_t));  // Sensor → Datalogger
xQueueCreate(10, sizeof(SensorData_t));  // Sensor → MQTT
xQueueCreate(10, sizeof(SensorData_t));  // Sensor → Monitor
// ... 3 queues for 1 data type
```

**Example Zephyr**:
```c
ZBUS_CHAN_DEFINE(sensor_data_channel, ...);
// Datalogger, MQTT, monitor subscribe automatically
```

---

## Common Issues

| Issue | Symptom | Fix |
|-------|---------|-----|
| No messages received | Subscriber log empty | Check `zbus_chan_add_obs()`, verify publisher exists |
| "Message dropped" | Frequent warnings | Increase queue depth (`ZBUS_OBSERVER_DEFINE` param) |
| Compile error "undefined struct" | Build fails | Add struct definition to messages.h, not .c |
| Subscriber called with old data | Stale values | Fetch latest with `zbus_chan_fetch()` inside loop |
| Message corruption | Garbled data | Check struct size, alignment, CRC |

---

**Zbus is powerful. One publish, N subscribers. No manual routing.**
