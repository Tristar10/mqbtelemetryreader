#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

// Simos18 measuring values read with UDS ReadDataByIdentifier (0x22).
// Addresses and scaling come from the SimosTools PID list (mode 22).
struct simos_did_t {
  uint16_t id;
  const char* key;   // JSON key
  const char* name;
  const char* unit;
  uint8_t len;       // response data length in bytes
  bool sign;
  float scale;       // value = raw * scale + offset
  float offset;
};

extern const simos_did_t k_simos_dids[];
extern const size_t k_simos_did_count;

// The set read together in one multi-DID request for live data.
extern const uint16_t k_live_dids[];
extern const size_t k_live_did_count;

// One multi-DID read of the live set. Writes values (and derived boost) to out.
bool simosReadLive(JsonObject out);

// Full diagnostic run: each DID alone, multi-DID single-frame and multi-frame
// requests, and a short rate test. Takes a few seconds.
void simosProbe(JsonObject out);
