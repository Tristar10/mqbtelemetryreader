#include "obd_can.h"

#include <driver/twai.h>
#include <string.h>

#include "config.h"

static bool g_ready = false;

static bool installDriver(twai_mode_t mode) {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)OBD_CAN_TX_PIN, (gpio_num_t)OBD_CAN_RX_PIN, mode);
  g.rx_queue_len = 64;
  g.tx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }
  return true;
}

static void uninstallDriver() {
  twai_stop();
  twai_driver_uninstall();
}

bool obdCanInit() {
  g_ready = installDriver(TWAI_MODE_NORMAL);
  return g_ready;
}

bool obdCanRestart() {
  if (g_ready) {
    uninstallDriver();
    g_ready = false;
  }
  return obdCanInit();
}

void obdCanSniff(uint32_t durationMs, JsonObject out) {
  struct seen_t {
    uint32_t id;
    bool extd;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t count;
  };
  static seen_t seen[48];
  size_t seen_count = 0;
  uint32_t total = 0;
  uint32_t dropped_ids = 0;

  if (g_ready) {
    uninstallDriver();
    g_ready = false;
  }
  const bool listening = installDriver(TWAI_MODE_LISTEN_ONLY);
  out["listenOnly"] = listening;

  const uint32_t started = millis();
  while (listening && millis() - started < durationMs) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(20)) != ESP_OK) {
      continue;
    }
    total++;
    size_t i = 0;
    while (i < seen_count && !(seen[i].id == m.identifier && seen[i].extd == (bool)m.extd)) {
      i++;
    }
    if (i == seen_count) {
      if (seen_count == sizeof(seen) / sizeof(seen[0])) {
        dropped_ids++;
        continue;
      }
      seen[i] = {m.identifier, (bool)m.extd, 0, {0}, 0};
      seen_count++;
    }
    seen[i].dlc = m.data_length_code;
    memcpy(seen[i].data, m.data, 8);
    seen[i].count++;
  }

  twai_status_info_t st = {};
  if (listening) {
    twai_get_status_info(&st);
    out["busErrors"] = st.bus_error_count;
    out["rxErrors"] = st.rx_error_counter;
    uninstallDriver();
  }
  obdCanInit();

  out["seconds"] = durationMs / 1000.0f;
  out["frames"] = total;
  out["idsNotListed"] = dropped_ids;
  JsonArray ids = out["ids"].to<JsonArray>();
  for (size_t i = 0; i < seen_count; i++) {
    char idbuf[12];
    snprintf(idbuf, sizeof(idbuf), seen[i].extd ? "0x%08lX" : "0x%03lX", (unsigned long)seen[i].id);
    char databuf[3 * 8 + 1] = {0};
    for (uint8_t b = 0; b < seen[i].dlc && b < 8; b++) {
      snprintf(&databuf[b * 3], 4, b ? " %02X" : "%02X", seen[i].data[b]);
    }
    JsonObject row = ids.add<JsonObject>();
    row["id"] = idbuf;
    row["count"] = seen[i].count;
    row["last"] = databuf;
  }
}

static const char* stateName(twai_state_t s) {
  switch (s) {
  case TWAI_STATE_STOPPED:
    return "stopped";
  case TWAI_STATE_RUNNING:
    return "running";
  case TWAI_STATE_BUS_OFF:
    return "bus_off";
  case TWAI_STATE_RECOVERING:
    return "recovering";
  }
  return "unknown";
}

// Bring the controller back after bus-off (e.g. adapter unplugged).
static void recoverIfNeeded() {
  twai_status_info_t st = {};
  if (twai_get_status_info(&st) != ESP_OK) {
    return;
  }
  if (st.state == TWAI_STATE_BUS_OFF) {
    twai_initiate_recovery();
  } else if (st.state == TWAI_STATE_STOPPED) {
    twai_start();
  }
}

void obdCanWriteStatusJson(JsonObject out) {
  out["ready"] = g_ready;
  twai_status_info_t st = {};
  if (!g_ready || twai_get_status_info(&st) != ESP_OK) {
    return;
  }
  out["state"] = stateName(st.state);
  out["txErrors"] = st.tx_error_counter;
  out["rxErrors"] = st.rx_error_counter;
  out["busErrors"] = st.bus_error_count;
  out["txFailed"] = st.tx_failed_count;
  out["rxMissed"] = st.rx_missed_count;
}

static void drainRx() {
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
  }
}

static bool sendFrame(const uint8_t* data, uint8_t len) {
  twai_message_t m = {};
  m.identifier = ECU_REQUEST_ID;
  m.data_length_code = 8;
  memset(m.data, ISOTP_PAD, 8);
  memcpy(m.data, data, len);
  return twai_transmit(&m, pdMS_TO_TICKS(20)) == ESP_OK;
}

// Waits for the next frame from the ECU until deadline (millis()).
static bool receiveFrame(twai_message_t& m, uint32_t deadline) {
  for (;;) {
    const int32_t remaining = (int32_t)(deadline - millis());
    if (remaining <= 0) {
      return false;
    }
    if (twai_receive(&m, pdMS_TO_TICKS(remaining)) != ESP_OK) {
      continue;
    }
    if (!m.extd && !m.rtr && m.identifier == ECU_RESPONSE_ID && m.data_length_code >= 1) {
      return true;
    }
  }
}

static void separationDelay(uint8_t stmin) {
  if (stmin == 0) {
    return;
  }
  if (stmin <= 0x7F) {
    delay(stmin);
  } else if (stmin >= 0xF1 && stmin <= 0xF9) {
    delayMicroseconds((stmin - 0xF0) * 100);
  } else {
    delay(127);  // reserved values: use the maximum
  }
}

static bool sendRequest(const uint8_t* req, uint16_t len, uint32_t deadline, const char*& err) {
  uint8_t buf[8];
  if (len <= 7) {
    buf[0] = (uint8_t)len;
    memcpy(&buf[1], req, len);
    if (!sendFrame(buf, len + 1)) {
      err = "tx_failed";
      return false;
    }
    return true;
  }

  // First frame
  buf[0] = 0x10 | ((len >> 8) & 0x0F);
  buf[1] = len & 0xFF;
  memcpy(&buf[2], req, 6);
  if (!sendFrame(buf, 8)) {
    err = "tx_failed";
    return false;
  }
  uint16_t sent = 6;
  uint8_t sn = 1;

  while (sent < len) {
    // Wait for flow control
    uint8_t block_size = 0;
    uint8_t stmin = 0;
    for (;;) {
      twai_message_t fc;
      if (!receiveFrame(fc, deadline)) {
        err = "no_flow_control";
        return false;
      }
      const uint8_t pci = fc.data[0];
      if (pci == 0x30) {
        block_size = fc.data[1];
        stmin = fc.data[2];
        break;
      }
      if (pci == 0x31) {
        continue;  // wait
      }
      if (pci == 0x32) {
        err = "flow_control_overflow";
        return false;
      }
      // Ignore anything else while waiting for flow control.
    }

    uint8_t in_block = 0;
    while (sent < len && (block_size == 0 || in_block < block_size)) {
      separationDelay(stmin);
      const uint16_t chunk = (len - sent) > 7 ? 7 : (len - sent);
      buf[0] = 0x20 | (sn & 0x0F);
      memcpy(&buf[1], &req[sent], chunk);
      if (!sendFrame(buf, chunk + 1)) {
        err = "tx_failed";
        return false;
      }
      sent += chunk;
      sn++;
      in_block++;
    }
  }
  return true;
}

static bool receiveResponse(isotp_result_t& out, uint32_t deadline, const char*& err) {
  twai_message_t m;
  if (!receiveFrame(m, deadline)) {
    err = "timeout";
    return false;
  }

  const uint8_t type = m.data[0] >> 4;
  if (type == 0x0) {
    const uint8_t len = m.data[0] & 0x0F;
    if (len == 0 || len > 7 || len > m.data_length_code - 1) {
      err = "bad_single_frame";
      return false;
    }
    memcpy(out.data, &m.data[1], len);
    out.len = len;
    return true;
  }

  if (type != 0x1) {
    err = "unexpected_frame";
    return false;
  }

  const uint16_t total = ((uint16_t)(m.data[0] & 0x0F) << 8) | m.data[1];
  if (total > ISOTP_MAX_PAYLOAD) {
    err = "response_too_large";
    return false;
  }
  memcpy(out.data, &m.data[2], 6);
  uint16_t got = 6;

  const uint8_t fc[3] = {0x30, 0x00, 0x00};  // continue, no block limit, no delay
  if (!sendFrame(fc, 3)) {
    err = "tx_failed";
    return false;
  }

  uint8_t expected_sn = 1;
  while (got < total) {
    if (!receiveFrame(m, deadline)) {
      err = "timeout_consecutive";
      return false;
    }
    if ((m.data[0] >> 4) != 0x2) {
      continue;
    }
    if ((m.data[0] & 0x0F) != (expected_sn & 0x0F)) {
      err = "sequence_error";
      return false;
    }
    const uint16_t chunk = (total - got) > 7 ? 7 : (total - got);
    memcpy(&out.data[got], &m.data[1], chunk);
    got += chunk;
    expected_sn++;
  }
  out.len = total;
  return true;
}

bool isotpRequest(const uint8_t* req, uint16_t reqLen, isotp_result_t& out, uint32_t timeoutMs) {
  out.ok = false;
  out.error = nullptr;
  out.len = 0;
  out.elapsedMs = 0;

  if (!g_ready) {
    out.error = "can_not_ready";
    return false;
  }
  if (reqLen == 0 || reqLen > 4095) {
    out.error = "bad_request_length";
    return false;
  }

  recoverIfNeeded();
  drainRx();

  const uint32_t started = millis();
  uint32_t deadline = started + timeoutMs;
  const char* err = nullptr;

  if (!sendRequest(req, reqLen, deadline, err)) {
    out.error = err;
    out.elapsedMs = millis() - started;
    return false;
  }

  for (;;) {
    if (!receiveResponse(out, deadline, err)) {
      out.error = err;
      break;
    }
    // 7F <sid> 78 = response pending: keep waiting for the real answer.
    if (out.len >= 3 && out.data[0] == 0x7F && out.data[1] == req[0] && out.data[2] == 0x78) {
      deadline = millis() + 2000;
      continue;
    }
    out.ok = true;
    break;
  }
  out.elapsedMs = millis() - started;

  // An unacknowledged frame is retransmitted forever by the controller, which
  // would keep the car's diagnostic bus busy. After any failure, restart the
  // driver to drop pending/retrying frames.
  if (!out.ok) {
    obdCanRestart();
  }
  return out.ok;
}
