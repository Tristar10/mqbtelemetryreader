# MQB Telemetry Reader

Reads engine data from a VW/Audi MQB car with a Simos18 ECU through the OBD
port and (eventually) feeds it to RaceChrono alongside OpenHaldex-S3.

Target car: Audi 8S TT, Simos18.10. Hardware: LilyGo T-2CAN (ESP32-S3).

## Status: v0.0.1 - test firmware

This version only checks what the ECU answers and how fast:

- Reads Simos18 values with UDS `0x22` (ReadDataByIdentifier) on `0x7E0`/`0x7E8`.
- Tries each value alone, then several values in one request, then measures
  the live read rate.
- Serves a small test page on its own Wi-Fi access point.

Not yet: forwarding to OpenHaldex / RaceChrono, sleep when the car is off.

## Wiring

| OBD port | T-2CAN |
|---|---|
| Pin 6 (CAN-H) | CANHA |
| Pin 14 (CAN-L) | CANLA |
| Pin 4 or 5 (GND) | GND |
| Pin 16 (+12 V) | 12-24V |

Pin 16 is powered even with the car off and this version has no sleep mode:
unplug it when parked.

## Values

| Key | DID | Scaling |
|---|---|---|
| rpm | F40C | u16 / 4 |
| timing | 2004 | s16 / 100 deg |
| map | 39C0 | u16 / 10 kPa |
| ambient | 13CA | u16 / 120.6 kPa |
| torque | 437C | s16 / 10 Nm |
| pedal | 1070 | u16 / 10.24 % |
| iat | 1001 | u8 * 0.75 - 48 C |
| knock1-4 | 200A-200D | s16 / 100 deg (probe only) |

Boost = MAP - ambient. Addresses come from the SimosTools mode-22 PID list.

## Build and flash

```bash
~/.platformio/penv/bin/pio run -t upload
```

## Testing

1. Ignition on (engine running for live values).
2. Join Wi-Fi `MQBTelemetry` (password `mqbtelemetry`).
3. Open `http://192.168.4.1/`, press **Run probe**, then **Start live**.

API: `GET /api/status`, `GET /api/read`, `POST /api/probe`.

## Versioning

Patch digit bumps on every change; minor digit bumps when a change is
verified on the car. Version lives in `include/version.h`.
