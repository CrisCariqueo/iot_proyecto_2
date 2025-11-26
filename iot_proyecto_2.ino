#include <ESP8266WiFi.h>
#include <DHT.h>

#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>
#include <ThingsBoard.h>

// Configuración WiFi
const char* ssid = "Clerd";
const char* password = "Elena1720clerd";

// Configuración MQTT para ThingsBoard
const char token[] = "ccxz31zy0gbdpy8hgomj";
constexpr char tb_server[] = "hostName";
constexpr uint16_t tb_port = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 256U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

WiFiClient mqttClient;
Arduino_MQTT_Client mqttClient(wifiClient);

// ################################### NI IDEA DE LO QUE PASA ACA ###################################
// Initialize used apis
Server_Side_RPC<3U, 5U> rpc;
Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;
Shared_Attribute_Update<3U, MAX_ATTRIBUTES> shared_update;

const std::array<IAPI_Implementation*, 3U> apis = {
    &rpc,
    &attr_request,
    &shared_update
};
// ################################### NI IDEA DE LO QUE PASA ACA ###################################

// Initialize ThingsBoard instance with the maximum needed buffer size, stack size and the apis we want to use
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, Default_Max_Stack_Size, apis);

// === DATA ===
// Water Sensor
#define W_PWR_PIN D7
#define SIGNAL_PIN A0

int waterValue = 0;  // variable to store the water sensor's value

// Humidity Sensor
#define DHT_SENSOR_PIN  D6 // The ESP8266 pin D7 connected to DHT11 sensor
#define DHT_SENSOR_TYPE DHT11

DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

int registration_counter = 0;
// === DATA ===

void initWiFi() {
  Serial.println("Conectando a internet...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado");
}

void setup() {
  Serial.begin(9600);

  pinMode(W_PWR_PIN, OUTPUT);    // Configure D7 pin as an OUTPUT
  digitalWrite(W_PWR_PIN, LOW);  // turn the sensor OFF

  dht_sensor.begin();            // initialize the DHT sensor

  initWiFi();                    // Conectar a WiFi
}

const bool reconnect() {
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  initWiFi();
  return true;
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publicar datos cada 10 segundos
  static unsigned long lastMsg = 0;
  unsigned long now = millis();

  if (now - lastMsg > 10000) {
    // HUMIDITY SENSOR
    float humi  = dht_sensor.readHumidity();                // read humidity
    float temperature_C = dht_sensor.readTemperature();     // read temperature in Celsius
    float temperature_F = dht_sensor.readTemperature(true); // read temperature in Fahrenheit

    if ( isnan(temperature_C) || isnan(temperature_F) || isnan(humi)) { // check whether the reading is successful or not
      Serial.println("Failed to read from DHT sensor!");
    } else {
      Serial.print("Humidity: "); Serial.print(humi); Serial.print("%");
      Serial.print("  |  ");
      Serial.print("Temperature: ");  
      Serial.print(temperature_C);  Serial.print("°C  ~  ");
      Serial.print(temperature_F);  Serial.println("°F");
    }

    // WATER SENSOR
    digitalWrite(W_PWR_PIN, HIGH);   // turn the sensor ON
    delay(10);                       // wait 10 milliseconds
    waterValue = analogRead(SIGNAL_PIN);  // read the analog value from sensor
    digitalWrite(W_PWR_PIN, LOW);    // turn the sensor OFF

    Serial.print("Water level: ");
    Serial.println(waterValue);


    lastMsg = now;

    // Crear el mensaje JSON
    String payload = "{\"tmp\":" + String(temperature_C) +
                      ",\"hum\":" + String(humi) +
                      ",\"wtr\":" + String(waterValue) +
                      "}";

    registration_counter += 1;
    Serial.print("\nRegistration N°");
    Serial.println(registration_counter);

    tb.sendTelemetryData("temperature", temperature_C);
    tb.sendTelemetryData("humidity", humi);
    tb.sendTelemetryData("water_lvl", waterValue);
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    tb.sendAttributeData("channel", WiFi.channel());
  }
}