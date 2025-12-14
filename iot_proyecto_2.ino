#include <ESP8266WiFi.h>
#include <DHT.h>

// ============= CONFIG WIFI =============
// const char* ssid = "Clerd";
// const char* password = "Elena1720clerd";
const char* ssid = "Redmi Note 9 Pro";
const char* password = "177013si";

// ============= CONFIG THINGSBOARD =============
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <Server_Side_RPC.h>

const char TOKEN[] = "ccxz31zy0gbdpy8hgomj";
constexpr char TB_SERVER[] = "iot.ceisufro.cl";
constexpr uint16_t TB_PORT = 1883;

constexpr uint32_t MAX_MESSAGE_SIZE = 256U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);


// ================= RELÉ =================
#define RELAY_PIN D1

bool relayActive = false;
unsigned long relayOffAt = 0;


// ================= RPC =================
Server_Side_RPC<1U, 1U> rpc;

const std::array<IAPI_Implementation*, 1U> apis = {
  &rpc
};

ThingsBoard tb(mqttClient, (uint16_t)MAX_MESSAGE_SIZE, (uint16_t)MAX_MESSAGE_SIZE, Default_Max_Stack_Size, apis);

// ================= RPC CALLBACK =================
void processRelayRpc(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("RPC recibido: relay");

  if (!data.containsKey("time")) {
    response["error"] = "Missing time";
    return;
  }

  unsigned long duration = data["time"]; // ms

  digitalWrite(RELAY_PIN, LOW);
  relayActive = true;
  relayOffAt = millis() + duration;

  Serial.print("Relé activado por ");
  Serial.print(duration);
  Serial.println(" ms");

  response["status"] = "ON";
}

// Registro de métodos RPC
const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{ "relay", processRelayRpc }
};

// ================= CONEXIÓN TB =================
void reconnectTB() {
  while (!tb.connected()) {
    Serial.print("Conectando a ThingsBoard... ");

    if (tb.connect(TB_SERVER, TOKEN, TB_PORT)) {
      Serial.println("Conectado!");

      // Suscripcaión RPC
      if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Error al suscribirse a RPC");
      }

    } else {
      Serial.println("Fallo al conectar. Reintentando en 3s...");
      delay(3000);
    }
  }
}


// ============= SENSORES =============
#define ADC_PIN A0

// Sensor de agua
#define WATER_PWR_PIN D7
int waterValue = 0;

// Sensor UV
#define UV_PWR_PIN D5
float uvIntensity = 0.0;

// DHT11
#define DHT_SENSOR_PIN  D6
#define DHT_SENSOR_TYPE DHT11
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

// Timer para telemetría
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 5000; // ms

// UTILIDADES UV SENSOR
int averageAnalogRead(int pinToRead) {
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  return runningValue / numberOfReadings;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// ============= WIFI =============
void initWiFi() {
  Serial.print("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Conectado!");
}


// ================================
void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);

  pinMode(WATER_PWR_PIN, OUTPUT);
  digitalWrite(WATER_PWR_PIN, LOW);

  pinMode(UV_PWR_PIN, OUTPUT);
  digitalWrite(UV_PWR_PIN, LOW);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  dht_sensor.begin();

  initWiFi();
  reconnectTB();
}

// ================================
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    initWiFi();
  }

  if (!tb.connected()) {
    reconnectTB();
  }

  tb.loop();

  // -------------------- RELÉ --------------------
  if (relayActive && millis() > relayOffAt) {
    digitalWrite(RELAY_PIN, HIGH);
    relayActive = false;
    Serial.println("Relé apagado por timeout");
  }

  // ===================== TELEMETRÍA =====================
  unsigned long now = millis();
  if (now - lastTelemetry >= TELEMETRY_INTERVAL) {
    lastTelemetry = now;

    // -------------------- DHT --------------------
    float humi = dht_sensor.readHumidity();
    float tempC = dht_sensor.readTemperature();

    if (isnan(humi) || isnan(tempC)) {
      Serial.println("Error leyendo DHT11");
    }

    // -------------------- SENSOR DE AGUA --------------------
    digitalWrite(WATER_PWR_PIN, HIGH);
    delay(10);
    waterValue = analogRead(ADC_PIN);
    digitalWrite(WATER_PWR_PIN, LOW);

    // -------------------- SENSOR UV --------------------
    digitalWrite(UV_PWR_PIN, HIGH);
    delay(10);
    int uvLevel = averageAnalogRead(ADC_PIN);

    float outputVoltage = 3.3 * uvLevel / 1024.0;
    uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
    digitalWrite(UV_PWR_PIN, LOW);

    // -------------------- Serial Debug --------------------
    Serial.print("Agua: ");       Serial.print(waterValue);
    Serial.print(" | UV: ");      Serial.print(uvIntensity);
    Serial.print(" | Temp: ");    Serial.print(tempC);
    Serial.print(" | Humedad: "); Serial.println(humi);

    // -------------------- Envío Telemetría --------------------
    tb.sendTelemetryData("temperature", tempC);
    tb.sendTelemetryData("humidity", humi);
    tb.sendTelemetryData("water_level", waterValue);
    tb.sendTelemetryData("uv_intensity", uvIntensity);
  }
}
