#include "selftest.h"

#include <SPI.h>
#include <driver/twai.h>
#include <mcp2515.h>

#include "config.h"

static MCP2515 g_mcp(MCP2515_CS_PIN, 10000000, &SPI);
static bool g_spi_started = false;

static bool mcpStart() {
  pinMode(MCP2515_RST_PIN, OUTPUT);
  digitalWrite(MCP2515_RST_PIN, HIGH);
  delay(10);
  digitalWrite(MCP2515_RST_PIN, LOW);
  delay(10);
  digitalWrite(MCP2515_RST_PIN, HIGH);
  delay(10);
  if (!g_spi_started) {
    SPI.begin(MCP2515_SCLK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN, MCP2515_CS_PIN);
    g_spi_started = true;
  }
  return g_mcp.reset() == MCP2515::ERROR_OK && g_mcp.setBitrate(CAN_500KBPS) == MCP2515::ERROR_OK &&
         g_mcp.setNormalMode() == MCP2515::ERROR_OK;
}

void selftestParkChannelB() {
  if (mcpStart()) {
    g_mcp.setSleepMode();
  }
}

void selftestRun(JsonObject out) {
  twai_status_info_t before = {};
  twai_get_status_info(&before);

  if (!mcpStart()) {
    out["ok"] = false;
    out["error"] = "mcp2515_init_failed";
    g_mcp.setSleepMode();
    return;
  }

  // Clear anything pending on both sides.
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
  }
  struct can_frame f;
  while (g_mcp.readMessage(&f) == MCP2515::ERROR_OK) {
  }

  // A -> B: TWAI transmits, MCP2515 receives (and ACKs).
  bool a_to_b = false;
  twai_message_t tx = {};
  tx.identifier = 0x123;
  tx.data_length_code = 8;
  for (uint8_t i = 0; i < 8; i++) {
    tx.data[i] = 0xA0 + i;
  }
  const bool a_sent = twai_transmit(&tx, pdMS_TO_TICKS(50)) == ESP_OK;
  uint32_t started = millis();
  while (a_sent && millis() - started < 200) {
    if (g_mcp.readMessage(&f) == MCP2515::ERROR_OK) {
      a_to_b = f.can_id == 0x123 && f.can_dlc == 8 && f.data[0] == 0xA0 && f.data[7] == 0xA7;
      break;
    }
    delay(1);
  }

  // B -> A: MCP2515 transmits, TWAI receives (and ACKs).
  bool b_to_a = false;
  struct can_frame fb = {};
  fb.can_id = 0x321;
  fb.can_dlc = 8;
  for (uint8_t i = 0; i < 8; i++) {
    fb.data[i] = 0xB0 + i;
  }
  const bool b_sent = g_mcp.sendMessage(&fb) == MCP2515::ERROR_OK;
  if (b_sent && twai_receive(&m, pdMS_TO_TICKS(200)) == ESP_OK) {
    b_to_a = m.identifier == 0x321 && m.data_length_code == 8 && m.data[0] == 0xB0 && m.data[7] == 0xB7;
  }

  // Never leave channel B active: it would ACK frames if wired to the car.
  g_mcp.setSleepMode();

  twai_status_info_t after = {};
  twai_get_status_info(&after);

  out["aSent"] = a_sent;
  out["aToB"] = a_to_b;
  out["bSent"] = b_sent;
  out["bToA"] = b_to_a;
  out["busErrorsDuring"] = after.bus_error_count - before.bus_error_count;
  out["txErrorsAfter"] = after.tx_error_counter;
  out["ok"] = a_to_b && b_to_a;
}
