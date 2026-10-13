#include "simos.h"

#include "obd_can.h"

static const uint8_t k_sid_read = 0x22;
static const uint8_t k_sid_read_positive = 0x62;

// clang-format off
const simos_did_t k_simos_dids[] = {
  // id      key           name                  unit   len sign  scale                    offset
  {0xF40C, "rpm",        "Engine speed",       "rpm", 2, false, 0.25f,                   0.0f},
  {0x2004, "timing",     "Ignition timing avg", "deg", 2, true,  0.01f,                   0.0f},
  {0x39C0, "map",        "MAP",                "kPa", 2, false, 0.1f,                    0.0f},
  {0x13CA, "ambient",    "Ambient pressure",   "kPa", 2, false, 1.0f / 120.60176665439f, 0.0f},
  {0x437C, "torque",     "Torque actual",      "Nm",  2, true,  0.1f,                    0.0f},
  {0x1070, "pedal",      "Pedal position",     "%",   2, false, 1.0f / 10.24f,           0.0f},
  {0x1001, "iat",        "Intake air temp",    "C",   1, false, 0.75f,                  -48.0f},
  {0x200A, "knock1",     "Knock retard cyl 1", "deg", 2, true,  0.01f,                   0.0f},
  {0x200B, "knock2",     "Knock retard cyl 2", "deg", 2, true,  0.01f,                   0.0f},
  {0x200C, "knock3",     "Knock retard cyl 3", "deg", 2, true,  0.01f,                   0.0f},
  {0x200D, "knock4",     "Knock retard cyl 4", "deg", 2, true,  0.01f,                   0.0f},
};
// clang-format on
const size_t k_simos_did_count = sizeof(k_simos_dids) / sizeof(k_simos_dids[0]);

const uint16_t k_live_dids[] = {0xF40C, 0x2004, 0x39C0, 0x13CA, 0x437C, 0x1070, 0x1001};
const size_t k_live_did_count = sizeof(k_live_dids) / sizeof(k_live_dids[0]);

static const simos_did_t* findDid(uint16_t id) {
  for (size_t i = 0; i < k_simos_did_count; i++) {
    if (k_simos_dids[i].id == id) {
      return &k_simos_dids[i];
    }
  }
  return nullptr;
}

static float decode(const simos_did_t& d, const uint8_t* p) {
  int32_t raw;
  if (d.len == 1) {
    raw = d.sign ? (int32_t)(int8_t)p[0] : (int32_t)p[0];
  } else {
    const uint16_t u = ((uint16_t)p[0] << 8) | p[1];
    raw = d.sign ? (int32_t)(int16_t)u : (int32_t)u;
  }
  return raw * d.scale + d.offset;
}

static String hex(const uint8_t* p, uint16_t len) {
  static const char* digits = "0123456789ABCDEF";
  String s;
  s.reserve(len * 3);
  for (uint16_t i = 0; i < len; i++) {
    if (i) {
      s += ' ';
    }
    s += digits[p[i] >> 4];
    s += digits[p[i] & 0x0F];
  }
  return s;
}

static String didHex(uint16_t id) {
  char buf[7];
  snprintf(buf, sizeof(buf), "0x%04X", id);
  return String(buf);
}

static uint16_t buildRequest(const uint16_t* ids, size_t count, uint8_t* req) {
  uint16_t n = 0;
  req[n++] = k_sid_read;
  for (size_t i = 0; i < count; i++) {
    req[n++] = ids[i] >> 8;
    req[n++] = ids[i] & 0xFF;
  }
  return n;
}

// Writes NRC info when the ECU answered 7F 22 xx. Returns true if it was negative.
static bool writeNegative(const isotp_result_t& r, JsonObject out) {
  if (r.len >= 3 && r.data[0] == 0x7F) {
    char nrc[5];
    snprintf(nrc, sizeof(nrc), "0x%02X", r.data[2]);
    out["negative"] = true;
    out["nrc"] = nrc;
    return true;
  }
  return false;
}

// Parses a positive multi-DID response: 62 [id_hi id_lo data...]...
// Returns the number of values decoded, or -1 on a malformed response.
static int parseResponse(const isotp_result_t& r, JsonObject values) {
  if (r.len < 1 || r.data[0] != k_sid_read_positive) {
    return -1;
  }
  int count = 0;
  uint16_t i = 1;
  while (i + 2 <= r.len) {
    const uint16_t id = ((uint16_t)r.data[i] << 8) | r.data[i + 1];
    const simos_did_t* d = findDid(id);
    if (!d || i + 2 + d->len > r.len) {
      return -1;
    }
    values[d->key] = decode(*d, &r.data[i + 2]);
    i += 2 + d->len;
    count++;
  }
  return i == r.len ? count : -1;
}

static void addBoost(JsonObject values) {
  if (values["map"].is<float>() && values["ambient"].is<float>()) {
    const float boost_kpa = values["map"].as<float>() - values["ambient"].as<float>();
    values["boost_kpa"] = boost_kpa;
    values["boost_psi"] = boost_kpa * 0.1450377f;
  }
}

static isotp_result_t g_result;  // large buffer: keep off the stack

bool simosReadLive(JsonObject out) {
  uint8_t req[1 + 2 * 16];
  const uint16_t len = buildRequest(k_live_dids, k_live_did_count, req);
  const bool ok = isotpRequest(req, len, g_result, 500);
  out["ms"] = g_result.elapsedMs;
  if (!ok) {
    out["ok"] = false;
    out["error"] = g_result.error;
    return false;
  }
  if (writeNegative(g_result, out)) {
    out["ok"] = false;
    return false;
  }
  JsonObject values = out["values"].to<JsonObject>();
  if (parseResponse(g_result, values) < 0) {
    out["ok"] = false;
    out["error"] = "malformed_response";
    out["raw"] = hex(g_result.data, g_result.len);
    return false;
  }
  addBoost(values);
  out["ok"] = true;
  return true;
}

static void runMulti(const char* name, const uint16_t* ids, size_t count, JsonObject row) {
  uint8_t req[1 + 2 * 16];
  const uint16_t len = buildRequest(ids, count, req);
  row["name"] = name;
  row["request"] = hex(req, len);
  const bool ok = isotpRequest(req, len, g_result, 500);
  row["ms"] = g_result.elapsedMs;
  if (!ok) {
    row["ok"] = false;
    row["error"] = g_result.error;
    return;
  }
  row["raw"] = hex(g_result.data, g_result.len);
  if (writeNegative(g_result, row)) {
    row["ok"] = false;
    return;
  }
  JsonObject values = row["values"].to<JsonObject>();
  const int n = parseResponse(g_result, values);
  row["ok"] = n == (int)count;
  if (n < 0) {
    row["error"] = "malformed_response";
  }
  addBoost(values);
}

void simosProbe(JsonObject out) {
  // 1. Each DID on its own: tells us which addresses this ECU accepts.
  JsonArray single = out["single"].to<JsonArray>();
  for (size_t i = 0; i < k_simos_did_count; i++) {
    const simos_did_t& d = k_simos_dids[i];
    JsonObject row = single.add<JsonObject>();
    row["did"] = didHex(d.id);
    row["name"] = d.name;
    uint8_t req[3] = {k_sid_read, (uint8_t)(d.id >> 8), (uint8_t)(d.id & 0xFF)};
    const bool ok = isotpRequest(req, sizeof(req), g_result, 500);
    row["ms"] = g_result.elapsedMs;
    if (!ok) {
      row["ok"] = false;
      row["error"] = g_result.error;
      continue;
    }
    row["raw"] = hex(g_result.data, g_result.len);
    if (writeNegative(g_result, row)) {
      row["ok"] = false;
      continue;
    }
    if (g_result.len == 3 + d.len && g_result.data[0] == k_sid_read_positive) {
      row["ok"] = true;
      row["value"] = decode(d, &g_result.data[3]);
      row["unit"] = d.unit;
    } else {
      row["ok"] = false;
      row["error"] = "unexpected_length";
    }
  }

  // 2. Multi-DID requests: does the ECU accept several IDs at once?
  JsonArray multi = out["multi"].to<JsonArray>();
  runMulti("3 DIDs (single frame)", k_live_dids, 3, multi.add<JsonObject>());
  runMulti("live set (multi frame)", k_live_dids, k_live_did_count, multi.add<JsonObject>());

  // 3. Rate test: repeated live reads for ~2 s.
  JsonObject rate = out["rate"].to<JsonObject>();
  const uint32_t started = millis();
  uint32_t good = 0;
  uint32_t bad = 0;
  uint32_t worst = 0;
  JsonDocument scratch;
  while (millis() - started < 2000) {
    scratch.clear();
    if (simosReadLive(scratch.to<JsonObject>())) {
      good++;
    } else {
      bad++;
    }
    const uint32_t ms = scratch["ms"] | 0;
    if (ms > worst) {
      worst = ms;
    }
  }
  const float seconds = (millis() - started) / 1000.0f;
  rate["reads"] = good;
  rate["failed"] = bad;
  rate["hz"] = good / seconds;
  rate["worstMs"] = worst;
}
