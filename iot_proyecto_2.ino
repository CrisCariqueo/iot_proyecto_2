#include <ESP8266WiFi.h>
#include <DHT.h>

// ============= CONFIG WIFI =============
const char* ssid = "Clerd";
const char* password = "Elena1720clerd";

// ============= CONFIG THINGSBOARD =============
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

const char TOKEN[] = "ccxz31zy0gbdpy8hgomj";
constexpr char TB_SERVER[] = "iot.ceisufro.cl";   // <-- Cambia si usas tu propio servidor
constexpr uint16_t TB_PORT = 1883;

constexpr uint32_t MAX_MESSAGE_SIZE = 256U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// Cliente MQTT simple (sin RPC ni atributos)
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);

// ThingsBoard (sin APIs extra)
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

// ============= SENSORES =============

// Water sensor
#define W_PWR_PIN D7
#define SIGNAL_PIN A0
int waterValue = 0;

// Humidity and temp sensor
#define DHT_SENSOR_PIN  D6
#define DHT_SENSOR_TYPE DHT11
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

// Timer para telemetría
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 5000; // 5 segundos

// ==================================================
// WIFI
// ==================================================
void initWiFi() {
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi");
}

// ==================================================
void reconnectTB() {
  while (!tb.connected()) {
    Serial.print("Conectando a ThingsBoard... ");

    if (tb.connect(TB_SERVER, TOKEN, TB_PORT)) {
      Serial.println("Conectado!");
    } else {
      Serial.println("Fallo al conectar. Reintentando en 3s...");
      delay(3000);
    }
  }
}

// ==================================================
void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);

  pinMode(W_PWR_PIN, OUTPUT);
  digitalWrite(W_PWR_PIN, LOW);

  dht_sensor.begin();
  initWiFi();
}

// ==================================================
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    initWiFi();
  }

  if (!tb.connected()) {
    reconnectTB();
  }

  tb.loop(); // Mantener MQTT activa

  // === TELEMETRÍA CADA 5s ===
  unsigned long now = millis();
  if (now - lastTelemetry >= TELEMETRY_INTERVAL) {
    lastTelemetry = now;

    // --- Leer DHT11 ---
    float humi = dht_sensor.readHumidity();
    float tempC = dht_sensor.readTemperature();

    if (isnan(humi) || isnan(tempC)) {
      Serial.println("Error leyendo DHT11");
    }

    // --- Leer sensor de agua ---
    digitalWrite(W_PWR_PIN, HIGH);
    delay(10);
    waterValue = analogRead(SIGNAL_PIN);
    digitalWrite(W_PWR_PIN, LOW);

    // --- Mostrar en Serial ---
    Serial.print("Temp: "); Serial.print(tempC); Serial.print(" °C  |  ");
    Serial.print("Humedad: "); Serial.print(humi); Serial.print(" %  |  ");
    Serial.print("Nivel Agua: "); Serial.println(waterValue);

    // --- Enviar Telemetría ---
    tb.sendTelemetryData("temperature", tempC);
    tb.sendTelemetryData("humidity", humi);
    tb.sendTelemetryData("water_level", waterValue);
  }
}
