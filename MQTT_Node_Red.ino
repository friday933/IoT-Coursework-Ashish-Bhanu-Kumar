#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>

const char* WIFI_SSID     = "WIFI_SSID";
const char* WIFI_PASSWORD = "WIFI_PASSWORD";

const char* MQTT_HOST = "test.mosquitto.org";
const uint16_t MQTT_PORT = 1883;
const char* DEVICE_ID = "esp32-dht01";

#define DHTPIN   4
#define DHTTYPE  DHT11
#define PIXEL_PIN 5
#define PIXEL_COUNT 1
#define PIXEL_BRIGHTNESS 30

uint32_t sampleIntervalMs = 5000;
float smoothAlpha = 0.2f;
float sTemp = NAN;
float sHum  = NAN;
bool lastAnom = false;
bool overrideActive = false;
uint32_t overrideUntilMs = 0;

enum LedMode { LED_OFF, LED_SOLID, LED_PULSE, LED_ALERT };
LedMode ledMode = LED_PULSE;
uint32_t ledColor = 0x00FF00;

WiFiClient wifiNet;
PubSubClient mqtt(wifiNet);

WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, "pool.ntp.org", 0, 60 * 1000);

DHT dht(DHTPIN, DHTTYPE);
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

String tState  = String("iot/") + DEVICE_ID + "/state";
String tTele   = String("iot/") + DEVICE_ID + "/telemetry";
String tEvents = String("iot/") + DEVICE_ID + "/events";
String tCmd    = String("iot/") + DEVICE_ID + "/cmd";
String tAck    = String("iot/") + DEVICE_ID + "/ack";

float dewPointC(float c, float rh) {
  const float a = 17.62f, b = 243.12f;
  float gamma = (a * c) / (b + c) + log(rh / 100.0f);
  return (b * gamma) / (a - gamma);
}

uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void setPixel(uint32_t color, uint8_t br) {
  strip.setBrightness(br);
  for (int i = 0; i < PIXEL_COUNT; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void driveLed(bool anomaly) {
  uint32_t now = millis();

  if (overrideActive && now < overrideUntilMs) {
    uint8_t phase = (now / 200) % 2 ? 255 : 5;
    setPixel(rgb(255, 0, 0), phase);
    return;
  }

  if (overrideActive && now >= overrideUntilMs) {
    overrideActive = false;
    ledMode = LED_PULSE;
    ledColor = rgb(0, 255, 0);
  }

  if (anomaly) {
    uint8_t phase = (now / 250) % 2 ? 255 : 5;
    setPixel(rgb(255, 0, 0), phase);
    return;
  }

  switch (ledMode) {
    case LED_OFF:
      setPixel(rgb(0, 0, 0), 0);
      break;

    case LED_SOLID:
      setPixel(ledColor, PIXEL_BRIGHTNESS);
      break;

    case LED_PULSE: {
        float s = (sin(2 * PI * ((now % 4000) / 4000.0)) + 1.0) * 0.5;
        uint8_t b = 5 + (uint8_t)(s * (PIXEL_BRIGHTNESS - 5));
        setPixel(ledColor, b);
        break;
      }

    case LED_ALERT:
      setPixel(rgb(255, 255, 0),
               ((now / 400) % 2) ? PIXEL_BRIGHTNESS : 5);
      break;
  }
}

void publishState(bool retained = true) {
  StaticJsonDocument<256> doc;
  doc["deviceId"]    = DEVICE_ID;
  doc["fw"]          = "1.2.0";
  doc["intervalMs"]  = sampleIntervalMs;
  doc["alpha"]       = smoothAlpha;
  doc["wifiRssi"]    = WiFi.RSSI();
  doc["ts"]          = ntp.getEpochTime();
  doc["ledMode"]     = (int)ledMode;
  doc["override"]    = overrideActive;
  char buf[256];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(tState.c_str(), buf, retained);
}

void publishAck(const char* what, const char* status, const char* msgTxt = nullptr) {
  StaticJsonDocument<256> doc;
  doc["ts"]    = ntp.getEpochTime();
  doc["ok"]    = (String(status) == "ok");
  doc["what"]  = what;
  if (msgTxt)  doc["msg"] = msgTxt;
  char b[256];
  serializeJson(doc, b, sizeof(b));
  mqtt.publish(tAck.c_str(), b);
}

void publishEvent(const char* type, const char* detail) {
  StaticJsonDocument<256> ev;
  ev["ts"]    = ntp.getEpochTime();
  ev["type"]  = type;
  ev["info"]  = detail;
  char out[256];
  serializeJson(ev, out, sizeof(out));
  mqtt.publish(tEvents.c_str(), out);
}

void handleCommand(JsonDocument& cmd) {

  bool changed = false;

  if (cmd.containsKey("manualOverride")) {
    bool ov = cmd["manualOverride"];
    if (ov) {
      overrideActive = true;
      overrideUntilMs = millis() + 10000UL;
      ledMode = LED_ALERT;
      ledColor = rgb(255, 0, 0);
      publishAck("manualOverride", "ok", "override ON");
      publishEvent("manual_override", "OVERRIDE_ON");
    } else {
      overrideActive = false;
      ledMode = LED_PULSE;
      ledColor = rgb(0, 255, 0);
      publishAck("manualOverride", "ok", "override OFF");
      publishEvent("manual_override", "OVERRIDE_OFF");
    }
    changed = true;
  }

  if (cmd.containsKey("led")) {
    JsonVariant l = cmd["led"];

    if (l.containsKey("mode")) {
      int m = l["mode"];
      if (m >= 0 && m <= 3) {
        ledMode = (LedMode)m;
        changed = true;
      }
    }

    if (l.containsKey("color")) {
      JsonArray c = l["color"].as<JsonArray>();
      if (c.size() == 3) {
        int r = c[0];
        int g = c[1];
        int b = c[2];
        r = constrain(r, 0, 255);
        g = constrain(g, 0, 255);
        b = constrain(b, 0, 255);
        ledColor = rgb(r, g, b);
        changed = true;
      }
    }

    publishAck("led", "ok", "LED config updated");
    publishEvent("led_update", "LED state changed");
  }

  if (cmd.containsKey("interval")) {
    uint32_t v = cmd["interval"];
    if (v >= 1000 && v <= 60000) {
      sampleIntervalMs = v;
      changed = true;
      publishAck("interval", "ok", "interval updated");
    } else {
      publishAck("interval", "err", "range 1000..60000");
    }
  }

  if (cmd.containsKey("smooth")) {
    float a = cmd["smooth"];
    if (a >= 0.01f && a <= 1.0f) {
      smoothAlpha = a;
      changed = true;
      publishAck("smooth", "ok", "alpha updated");
    } else {
      publishAck("smooth", "err", "range 0.01..1.0");
    }
  }

  if (changed) {
    publishState();
  }
}

void onMqtt(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (!err) {
    handleCommand(doc);
  }
}

void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t t0 = millis();
  Serial.print("Connecting to WiFi SSID: ");
  Serial.println(WIFI_SSID);

  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi FAILED (timeout). Rebooting in 3s...");
    delay(3000);
    ESP.restart();
  } else {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  }
}

void ensureMqtt() {
  if (mqtt.connected()) return;

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);

  String clientId = String(DEVICE_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  StaticJsonDocument<128> lwt;
  lwt["offline"] = true;
  char lwtBuf[128];
  serializeJson(lwt, lwtBuf, sizeof(lwtBuf));

  while (!mqtt.connected()) {
    Serial.print("MQTT connecting...");
    if (mqtt.connect(clientId.c_str(),
                     nullptr, nullptr,
                     tState.c_str(), 1, true, lwtBuf)) {
      Serial.println("connected");
      mqtt.subscribe(tCmd.c_str());
      publishState(true);
      publishEvent("startup", "node rebooted / online");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" retrying in 1s");
      delay(1000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIXEL_PIN, OUTPUT);
  strip.begin();
  strip.show();
  dht.begin();
  ensureWifi();
  ntp.begin();
  ntp.update();
  ensureMqtt();
}

void loop() {
  ensureWifi();
  ntp.update();
  ensureMqtt();
  mqtt.loop();

  if (overrideActive && millis() > overrideUntilMs) {
    overrideActive = false;
    ledMode = LED_PULSE;
    ledColor = rgb(0, 255, 0);
    publishAck("manualOverride", "ok", "override expired");
    publishEvent("manual_override", "OVERRIDE_AUTO_CLEAR");
  }

  static uint32_t lastSample = 0;
  uint32_t now = millis();

  if (now - lastSample >= sampleIntervalMs) {
    lastSample = now;

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println("DHT read failed");
      publishEvent("sensor_error", "DHT read failed");
    } else {
      if (isnan(sTemp)) {
        sTemp = t;
        sHum  = h;
      } else {
        sTemp = smoothAlpha * t + (1.0f - smoothAlpha) * sTemp;
        sHum  = smoothAlpha * h + (1.0f - smoothAlpha) * sHum;
      }
      float dew = dewPointC(sTemp, sHum);
      bool anom = false;
      if (sTemp < -10 || sTemp > 60 || sHum < 0 || sHum > 100) {
        anom = true;
      }

      if (anom && !lastAnom) {
        publishEvent("anomaly", "telemetry out of bounds");
      }
      lastAnom = anom;
      StaticJsonDocument<320> tele;
      tele["ts"]    = ntp.getEpochTime();
      tele["tempC"] = sTemp;
      tele["hum"]   = sHum;
      tele["dewC"]  = dew;
      tele["anomaly"] = anom;
      char buf[320];
      serializeJson(tele, buf, sizeof(buf));
      mqtt.publish(tTele.c_str(), buf);
      Serial.print("PUB -> ");
      Serial.println(buf);
    }
  }
  driveLed(lastAnom);
}