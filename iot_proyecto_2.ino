#include <ESP8266WiFi.h>
#include <DHT.h>
#include <PubSubClient.h>

// Configuración WiFi
const char* ssid = "Clerd";
const char* password = "Elena1720clerd";

// Configuración MQTT para Kaa IoT
const char* mqtt_server = "mqtt.cloud.kaaiot.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "kp1/d3skmqqm6fhc73agdbpg-v1/dcx/PVM1Xoqjhx/json";

WiFiClient espClient;
PubSubClient client(espClient);

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

void setup() {
  Serial.begin(9600);

  pinMode(W_PWR_PIN, OUTPUT);    // Configure D7 pin as an OUTPUT
  digitalWrite(W_PWR_PIN, LOW);  // turn the sensor OFF

  dht_sensor.begin();            // initialize the DHT sensor

  // Conectar a WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");

  // Configurar servidor MQTT
  client.setServer(mqtt_server, mqtt_port);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    // Intenta conectar (usa un ID único si tienes varios dispositivos)
    if (client.connect("ESP32_KaaClient")) {
      Serial.println("conectado");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" reintentando en 5 segundos");
      delay(5000);
    }
  }
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

    // Publicar en el topic
    if (client.publish(mqtt_topic, payload.c_str())) {
      Serial.println("Mensaje publicado: " + payload);
    } else {
      Serial.println("Error al publicar");
    }
  }
}