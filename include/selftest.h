#pragma once

#include <ArduinoJson.h>

// Bench loopback test between the T-2CAN's two CAN channels.
// Wire CANHA<->CANHB and CANLA<->CANLB (car disconnected), then run it.
// Channel A = ESP32 TWAI (the OBD channel), channel B = on-board MCP2515.
// The MCP2515 is put back to sleep afterwards so it never ACKs on the car bus.
void selftestRun(JsonObject out);

// Puts channel B (MCP2515) to sleep. Called at boot so it never ACKs on its own.
void selftestParkChannelB();
