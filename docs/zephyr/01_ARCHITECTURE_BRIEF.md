# Architecture Summary

**Greenfield redesign**: IoT SensorHub from FreeRTOS → Zephyr RTOS. Kernel manages hardware (DMA, ISR, mutexes). Application threads communicate via Zbus pub/sub.

## Hardware

- **MCU**: STM32F756ZG, 216 MHz, 256 KB RAM
- **I2C1**: BME280 (0x77), MPU6050 (0x68), 400 kHz
- **SPI1**: W25Q64 flash, 10 MHz
- **UART2**: RC7120 modem, 115200 baud
- **UART3**: Debug console, 115200 baud
- **UART6**: GPS, 9600 baud (NMEA)
- **GPIO**: LED (PA5)

## Threads (6 total)

| Thread | Purpose | Frequency |
|--------|---------|-----------|
| bme280_task | Read temp/humidity/pressure | 1 Hz |
| mpu6050_task | Read accel/gyro | 1 Hz |
| gps_task | Parse NMEA, extract position | 1 Hz |
| datalogger_task | Aggregate sensors, log to flash | Every 60s |
| modem_mqtt_task | Publish to cloud | Every 60s |
| system_monitor_task | Health checks | 1 Hz |

## Zbus Channels (6 message buses)

| Channel | Data | Pub/Sub |
|---------|------|---------|
| sensor_data_channel | T/H/P/Accel/Gyro | bme280, mpu6050 → datalogger, monitor |
| gps_fix_channel | Lat/Lon/Alt | gps_task → datalogger, mqtt |
| mqtt_publish_channel | Aggregated data | datalogger → mqtt_task |
| log_data_channel | Record for flash | datalogger → flash writer |
| modem_event_channel | Network state | modem_task → mqtt_task, monitor |
| system_event_channel | Errors, health | All → monitor, logger |

## Design Rationale

### Why 6 Threads?

**Isolation & Modularity**: Each sensor (BME280, MPU6050) runs independently. Failure of one doesn't block others.

- **Sensors (3 threads)**: Read at 1 Hz from I2C/UART without blocking each other
- **Datalogger (1 thread)**: Subscribes to all sensors, aggregates every 60s
- **Modem (1 thread)**: Handles AT commands, publishes via MQTT
- **Monitor (1 thread)**: Watches thread health, catches errors

**Parallelism benefit**: GPS reads while BME280 reads simultaneously (kernel serializes I2C hardware).

**Trade-off**: 6 threads = ~1% kernel overhead. Cost: 16 KB RAM for stacks. Benefit: Easier to extend (add 7th thread for new sensor without touching existing code).

### Why Zbus Over Message Queues?

**Traditional FreeRTOS approach**: Create 3 xQueues (sensor→datalogger, gps→datalogger, datalogger→modem). Explicit create/destroy, buffer management.

**Zephyr Zbus approach**:

- **Single message hub**: All 6 channels share one internal message router
- **Type-safe**: Compiler verifies struct sizes at compile time
- **Automatic routing**: Publisher doesn't know subscribers exist; they auto-receive
- **No dynamic allocation**: Fixed channels defined at startup
- **ISR-safe**: Can publish from interrupt handler (ISR context)

**Real-world example**:

```c
// FreeRTOS: Sensor publishes to queue, datalogger must pull it
// Zbus: Sensor publishes to channel, datalogger auto-receives (loose coupling)
// Result: Add new monitor task → Just subscribe, no queue changes needed
```

### Why Device Tree Over CubeMX Pin Configuration?

**CubeMX approach**: GUI generates code; changing one pin requires full GUI regeneration + recompilation.

**Device tree (.dts) approach**:

- All hardware defined in human-readable text
- Change a pin: Edit .dts, rebuild (5 seconds)
- No code generation needed
- Drivers read .dts at boot, configure themselves
- Portable across projects (reuse common nodes)

**Example trade-off**:

```text
CubeMX: UART3 RX on PA10 → Regenerate project → 15 min recompile
Zephyr: UART3 RX on PA10 → Change DTS → 5 sec rebuild + reflash
```

### Priority Order (Why These Numbers?)

| Thread | Priority | Reason |
|--------|----------|--------|
| **system_monitor** | 5 (highest) | Detects errors first; can interrupt sensors if health critical |
| **datalogger** | 6 | Saves data; must complete aggregation before next 60s cycle |
| **sensor readers** | 8 | Medium; read at 1 Hz, not time-critical |
| **gps_task** | 8 | Same; async UART doesn't block |
| **modem_mqtt** | 9 (lowest) | Network delays OK; shouldn't block sensors if cloud is slow |

**Why not equal priority?** With identical priorities, thread scheduler picks arbitrarily. Explicit priorities ensure predictable behavior during simultaneous wake-ups (e.g., all threads wake at 1 Hz).

## Design Shift: FreeRTOS → Zephyr

| Aspect | FreeRTOS | Zephyr |
|--------|----------|--------|
| **Threads** | xTaskCreate | k_thread_create |
| **IPC** | xQueueCreate (3 queues) | Zbus (1 channel hub) |
| **I2C Locking** | xSemaphoreTake (mutex) | Kernel serializes |
| **DMA** | HAL callbacks (app code) | Driver internal |
| **ISR** | Manual callbacks | Kernel delivery |
| **Hardware Config** | CubeMX GUI | Device tree (.dts) |
| **Build Config** | config.h macros | prj.conf + Kconfig |

## Key Architecture Decisions

1. **Single Zbus Hub**: All IPC through 6 channels, not multiple queues.
2. **Kernel-Managed Hardware**: No application ISR/DMA code.
3. **Device Tree Configuration**: All hardware in .dts, not code.
4. **400 kHz I2C**: Conservative speed, proven reliable.
5. **Ring Buffer on W25Q64**: 64 KB persistent storage.
6. **Async UART (GPS)**: CPU efficient, DMA-backed.

## Data Flow

```
Sensors (T/H/P/Accel/Gyro) → Zbus → Datalogger → Flash (ring buffer)
GPS → Zbus → Datalogger → Datalogger (add to record)
Record Complete (60s) → MQTT Task → Modem (AT) → Cloud
System Monitor watches all, publishes errors/health
```

## Error Handling

- **Sensor failure**: 3 retries with backoff, skip cycle if fail
- **I2C bus timeout**: Log warning, try next second
- **Modem offline**: Skip MQTT, continue local logging
- **Flash full**: Overwrite oldest data (ring buffer)
- **Total failure**: System monitor alerts, task restarts
- **Power loss**: Ring buffer survives, resume on boot

## 8 Implementation Phases

| Phase | Deliverable |
|-------|-------------|
| 0 | LED blinky, build system |
| 1 | UART logging operational |
| 2 | I2C bus detected, sensors alive |
| 3 | BME280 reading, publishing |
| 4 | MPU6050 reading, publishing |
| 5 | Ring buffer writes to flash |
| 6 | GPS parsing, valid fix detected |
| 7 | Modem AT commands, MQTT publish |
| 8 | System stable |
