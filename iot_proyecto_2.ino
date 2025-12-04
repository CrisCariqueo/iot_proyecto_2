#include <ESP8266WiFi.h>
#include <DHT.h>

// ============= CONFIG WIFI =============
const char* ssid = "Clerd";
const char* password = "Elena1720clerd";

// ============= CONFIG THINGSBOARD =============
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

const char TOKEN[] = "ccxz31zy0gbdpy8hgomj";
constexpr char TB_SERVER[] = "iot.ceisufro.cl";
constexpr uint16_t TB_PORT = 1883;

constexpr uint32_t MAX_MESSAGE_SIZE = 256U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

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
const unsigned long TELEMETRY_INTERVAL = 5000; // 5s

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
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi");
}


// ================================
void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);

  pinMode(WATER_PWR_PIN, OUTPUT);
  digitalWrite(WATER_PWR_PIN, LOW);

  pinMode(UV_PWR_PIN, OUTPUT);
  digitalWrite(UV_PWR_PIN, LOW);

  dht_sensor.begin();
  initWiFi();
}


// ================================
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    initWiFi();
  }

  if (!tb.connected()) {
    reconnectTB();
  }

  tb.loop(); // Mantener MQTT activa

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
