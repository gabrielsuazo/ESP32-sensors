#include "DHT.h"
#include <WiFi.h>           // WiFi control for ESP32
#include <ThingsBoard.h>    // ThingsBoard SDK
#include <ESPmDNS.h>
#include <Arduino.h>
#include "DFRobot_VEML7700.h"

// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// WiFi access point
#define WIFI_AP_NAME        "OnePlus 3"
// WiFi password
#define WIFI_PASSWORD       "zacvnwo1"

// See https://thingsboard.io/docs/getting-started-guides/helloworld/ 
// to understand how to obtain an access token
#define TOKEN               "nP550IxMWW2Y3VldhWi0"
// ThingsBoard server instance.
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD    115200

// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
// the Wifi radio's status
int status = WL_IDLE_STATUS;


// DHT object
#define DHTTYPE DHT11
// ESP32 pin used to query DHT22
#define DHT_PIN 4
DHT dht(DHT_PIN, DHTTYPE); 

//MQ135 sensor
#define MQ135PIN      34
#define MAXPIN        35
//Pour le capteur MQ135, cette valeur est différente pour chaque capteur mq135
#define RZERO         76.63

//B5W sensor
#define PIN_VOUT1     16
#define PIN_VOUT2     17
#define PIN_VTH       26
//Pour le capteur de particule
int count=0;
static int counts_idx = 0;
static int counts20sec_vout1 = 0;
static int counts20sec_vout2 = 0;

//Pour le capteur MAX4466
const int sampleWindow = 2000; 
unsigned int sample;

//VEML sensor
DFRobot_VEML7700 als;

//Variables globales
float temp=0;
float hum=0;
float lux=0;
int particules=0;
float decibels=0;

// Main application loop delay
int quant = 20;

// Period of sending a temperature/humidity data.
int send_delay = 2000;

// Time passed after temperature/humidity data was sent, milliseconds.
int send_passed = 0;

// Set to true if application is subscribed for the RPC messages.
bool subscribed = false;

// Setup an application
void setup() {
  // Initialize serial for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();
  als.begin();
  dht.begin();
}

// Main application loop
void loop() {
  delay(quant);

  send_passed += quant;

  // Reconnect to WiFi, if needed
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
    return;
  }

  // Reconnect to ThingsBoard, if needed
  if (!tb.connected()) {
    subscribed = false;

    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
    }
  }


  // Check if it is a time to send DHT22 temperature and humidity
  if (send_passed > send_delay) {
    Serial.println("Sending data...");

    // Uploads new telemetry to ThingsBoard using MQTT. 
    // See https://thingsboard.io/docs/reference/mqtt-api/#telemetry-upload-api 
    // for more details

    hum = dht.readHumidity(); 
    temp = dht.readTemperature(); 
    als.getALSLux(lux);  
    if (isnan(hum) || isnan(temp)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      tb.sendTelemetryFloat("temperature", temp);
      tb.sendTelemetryFloat("humidity", hum);
      tb.sendTelemetryFloat("lux", lux);
    }

    send_passed = 0;
  }

  // Process messages
  tb.loop();
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}
