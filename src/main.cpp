#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>

#include "config.h"
#include "obd_can.h"
#include "selftest.h"
#include "simos.h"
#include "version.h"

static WebServer server(80);

static const char k_index_html[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MQB Telemetry Reader</title>
<style>
body{font-family:system-ui,sans-serif;margin:16px;background:#111;color:#eee}
button{font-size:16px;padding:10px 14px;margin:4px 4px 12px 0;border-radius:8px;border:0;background:#2d6cdf;color:#fff}
table{border-collapse:collapse;width:100%;max-width:480px}td{padding:6px;border-bottom:1px solid #333}
td:last-child{text-align:right;font-variant-numeric:tabular-nums}
pre{white-space:pre-wrap;word-break:break-all;background:#1c1c1c;padding:8px;border-radius:8px;font-size:12px}
.muted{color:#999}
</style></head><body>
<h2>MQB Telemetry Reader <span class="muted" id="ver"></span></h2>
<button id="live">Start live</button><button id="probe">Run probe</button><button id="selftest">Bench self-test</button>
<div id="status" class="muted"></div>
<table id="values"></table>
<pre id="out"></pre>
<script>
const $=id=>document.getElementById(id);
let timer=null;
fetch('/api/status').then(r=>r.json()).then(j=>{$('ver').textContent='v'+j.version});
const units={rpm:'rpm',timing:'deg',map:'kPa',ambient:'kPa',torque:'Nm',pedal:'%',iat:'C',boost_kpa:'kPa',boost_psi:'psi'};
async function tick(){
  try{
    const j=await (await fetch('/api/read')).json();
    $('status').textContent=(j.ok?'OK':'Error: '+(j.error||j.nrc))+' - '+j.ms+' ms';
    if(j.values){$('values').innerHTML=Object.entries(j.values).map(([k,v])=>
      `<tr><td>${k}</td><td>${v.toFixed(2)} ${units[k]||''}</td></tr>`).join('');}
  }catch(e){$('status').textContent='Request failed';}
  if(timer)timer=setTimeout(tick,100);
}
$('live').onclick=()=>{if(timer){clearTimeout(timer);timer=null;$('live').textContent='Start live';}
  else{timer=setTimeout(tick,0);$('live').textContent='Stop live';}};
$('probe').onclick=async()=>{
  if(timer){clearTimeout(timer);timer=null;$('live').textContent='Start live';}
  $('out').textContent='Running probe (up to ~10 s)...';
  const j=await (await fetch('/api/probe',{method:'POST'})).json();
  $('out').textContent=JSON.stringify(j,null,2);
};
$('selftest').onclick=async()=>{
  if(timer){clearTimeout(timer);timer=null;$('live').textContent='Start live';}
  $('out').textContent='Running self-test (needs CANHA-CANHB and CANLA-CANLB jumpers)...';
  const j=await (await fetch('/api/selftest',{method:'POST'})).json();
  $('out').textContent=JSON.stringify(j,null,2);
};
</script></body></html>)HTML";

static void sendJson(JsonDocument& doc) {
  String body;
  serializeJson(doc, body);
  server.send(200, "application/json", body);
}

static void handleStatus() {
  JsonDocument doc;
  doc["version"] = MTR_VERSION;
  doc["uptimeMs"] = millis();
  obdCanWriteStatusJson(doc["can"].to<JsonObject>());
  sendJson(doc);
}

static void handleRead() {
  JsonDocument doc;
  simosReadLive(doc.to<JsonObject>());
  sendJson(doc);
}

static void handleProbe() {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root["version"] = MTR_VERSION;
  obdCanRestart();  // start from clean error counters
  obdCanWriteStatusJson(root["can"].to<JsonObject>());
  simosProbe(root);
  obdCanWriteStatusJson(root["canAfter"].to<JsonObject>());

  String body;
  serializeJson(doc, body);
  Serial.println(body);
  server.send(200, "application/json", body);
}

static void handleSelftest() {
  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root["version"] = MTR_VERSION;
  obdCanRestart();
  selftestRun(root);
  obdCanWriteStatusJson(root["can"].to<JsonObject>());
  sendJson(doc);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\nMQB Telemetry Reader v%s\n", MTR_VERSION);

  if (obdCanInit()) {
    Serial.println("OBD CAN started (500 kbit/s)");
  } else {
    Serial.println("OBD CAN init FAILED");
  }

  selftestParkChannelB();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("AP %s ready at %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, [] { server.send_P(200, "text/html", k_index_html); });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/read", HTTP_GET, handleRead);
  server.on("/api/probe", HTTP_POST, handleProbe);
  server.on("/api/selftest", HTTP_POST, handleSelftest);
  server.begin();
}

void loop() {
  server.handleClient();
  delay(1);
}
