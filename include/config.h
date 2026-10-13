#pragma once

// LilyGo T-2CAN: the ESP32-S3 internal TWAI controller drives the
// CANHA/CANLA screw terminals. Wire the OBD port there:
//   OBD pin 6  -> CANHA
//   OBD pin 14 -> CANLA
//   OBD pin 4/5 -> GND
#define OBD_CAN_TX_PIN 7
#define OBD_CAN_RX_PIN 6

// On-board MCP2515 (CANHB/CANLB). Only used by the bench self-test.
#define MCP2515_CS_PIN 10
#define MCP2515_SCLK_PIN 12
#define MCP2515_MOSI_PIN 11
#define MCP2515_MISO_PIN 13
#define MCP2515_RST_PIN 9

// Engine ECU (Simos18) physical diagnostic addressing on the OBD port.
#define ECU_REQUEST_ID 0x7E0
#define ECU_RESPONSE_ID 0x7E8

// ISO-TP padding byte for outgoing frames.
#define ISOTP_PAD 0xAA

// Access point used for testing.
#define AP_SSID "MQBTelemetry"
#define AP_PASSWORD "mqbtelemetry"
