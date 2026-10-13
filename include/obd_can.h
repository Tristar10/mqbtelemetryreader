#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

bool obdCanInit();
// Reinstalls the driver: clears bus-off state and resets all error counters.
bool obdCanRestart();
void obdCanWriteStatusJson(JsonObject out);

#define ISOTP_MAX_PAYLOAD 512

struct isotp_result_t {
  bool ok;
  const char* error;  // nullptr when ok
  uint16_t len;
  uint8_t data[ISOTP_MAX_PAYLOAD];
  uint32_t elapsedMs;
};

// Sends one ISO-TP request (single or multi frame) to ECU_REQUEST_ID and
// waits for the complete response from ECU_RESPONSE_ID. UDS "response
// pending" (7F xx 78) replies are skipped and the wait is extended.
bool isotpRequest(const uint8_t* req, uint16_t reqLen, isotp_result_t& out, uint32_t timeoutMs = 500);
