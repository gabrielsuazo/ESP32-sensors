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

//Pour les capteurs MAX4466 et B5W
const int sampleWindow = 1000; 
unsigned int sample;

//VEML sensor
DFRobot_VEML7700 als;

//Variables globales
float temp=0;
float hum=0;
float lux=0;
float airquality=0;
volatile int particules=0;
volatile float decibels=0;

// Main application loop delay
int quant = 20;

// Period of sending a temperature/humidity data.
int send_delay = 2000;

// Time passed after temperature/humidity data was sent, milliseconds.
int send_passed = 0;

// Set to true if application is subscribed for the RPC messages.
bool subscribed = false;

//declaration des taches
TaskHandle_t soundTask;
TaskHandle_t particlesTask;
//booleans qui indiquent lorsque la data est prete
volatile boolean particulesReady = false;
volatile boolean soundReady = false;

//declaration des fonctions
void particulesTask(void * parameters){
  for(;;){//loop infini
    if (particulesReady == false){ //s'il faut preparer une nouvelle mesure
      uint8_t prv_gpio1 = LOW;
      uint8_t prv_gpio2 = LOW;
      unsigned long startMillis= millis(); 
      volatile int counts_vout1 = 0;
      volatile int counts_vout2 = 0;
      while (millis() - startMillis < sampleWindow) {
          //delayMicroseconds(250);   // période d'échantillonage 250 [usec](voir spec fabricant)
          uint8_t cur_gpio1 = digitalRead(PIN_VOUT1);
          uint8_t cur_gpio2 = digitalRead(PIN_VOUT2);
          if (cur_gpio1 == HIGH && prv_gpio1 == LOW) {
              counts_vout1++;
          }
          if (cur_gpio2 == HIGH && prv_gpio2 == LOW) {
              counts_vout2++;
          }
          prv_gpio1 = cur_gpio1;
          prv_gpio2 = cur_gpio2;
      }
      particules = counts_vout1 - counts_vout2;
      particulesReady = true;
   }
    // Pause the task for 500ms
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
    
}

void dbm(void * parameters){
  for(;;){//loop infini
    if (soundReady == false){ //s'il faut preparer une nouvelle mesure
      unsigned long startMillis= millis();  // Début du temps d'échantillongage 
      unsigned int peakToPeak = 0;   // peak-to-peak 
      unsigned int signalMax = 0;
      unsigned int signalMin = 4095;
    
      // On collecte des données 
      while (millis() - startMillis < sampleWindow){
        sample = analogRead(MAXPIN);
        if (sample <= 4095)  
        {
          if (sample > signalMax)
          {
            signalMax = sample;  // On prend la valeur max
          }
          else if (sample < signalMin)
          {
            signalMin = sample;  // On prend la valeur min
          }
        }
      }
      peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
      float volts = (peakToPeak * 3.3) /4095;  // conversion en volts
      
      decibels = 20*log(peakToPeak/3.3) - 44; // conversion en dB
      Serial.println(decibels);
      soundReady = true;
    }
    // Pause the task for 500ms
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ppm(){  
  int val = analogRead(MQ135PIN);
  float res =  ((4095./(float)val) * 3.3 - 1.)*10.0;
  airquality = (116.6020682 * pow((res/76.63), -2.769034857));
}

void temperature_humidity(){
  
//use the functions which are supplied by library.
hum = dht.readHumidity();
// Read temperature as Celsius (the default)
temp = dht.readTemperature();
// Check if any reads failed and exit early (to try again).
  if (isnan(hum) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
  } 
}

// Setup of the application
void setup() {
  // Initialize serial for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();
  als.begin();
  dht.begin();
  pinMode(MAXPIN,INPUT);
  pinMode(PIN_VOUT1, INPUT);
  pinMode(PIN_VOUT2, INPUT);
  pinMode(PIN_VTH, OUTPUT);
  //Lecture de la tension sur le pin VTH (librairie analogwrite ne marche pas
  dacWrite(PIN_VTH, 0.125 / (3.3 / 1024));

  xTaskCreatePinnedToCore(
    particulesTask,    // Function that should be called
    "Lecture Particules",   // Name of the task (for debugging)
    1000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL,            // Task handle
    0                // Core you want to run the task on (0 or 1)
  );

  xTaskCreatePinnedToCore(
    dbm,    // Function that should be called
    "Lecture DBM",   // Name of the task (for debugging)
    1000,            // Stack size (bytes)
    NULL,            // Parameter to pass
    1,               // Task priority
    NULL,            // Task handle
    1                // Core you want to run the task on (0 or 1)
  );

}

// Main application loop
void loop() {
  
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


  // Check if it is a time to send the data
  if (send_passed > send_delay) {
    Serial.println("Sending data...");

    // Uploads new telemetry to ThingsBoard using MQTT. 
    // See https://thingsboard.io/docs/reference/mqtt-api/#telemetry-upload-api 
    // for more details

    temperature_humidity();
    als.getALSLux(lux); 
    ppm(); 
    
    tb.sendTelemetryFloat("temperature", temp);
    tb.sendTelemetryFloat("humidity", hum);
    tb.sendTelemetryFloat("lux", lux);
    tb.sendTelemetryFloat("airquality", airquality);
    while (!particulesReady || !soundReady){
      }
    if (particulesReady){
      tb.sendTelemetryFloat("particules", particules);
      particulesReady = false;
    }
    if (soundReady){
      tb.sendTelemetryFloat("decibels", decibels);
      soundReady = false;
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
