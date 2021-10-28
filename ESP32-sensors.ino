//Librairies

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Arduino.h>
#include "DFRobot_VEML7700.h"
#include "DHT.h"

//Define variables et ports

//our sensor is DHT11 typed
#define DHTTYPE DHT11

// DHT Sensor
uint8_t DHTPin = 4;

#define MQ135PIN      34
#define MAXPIN        35

//Pour le capteur MQ135, cette valeur est différente pour chaque capteur mq135
#define RZERO         76.63

#define PIN_VOUT1     16
#define PIN_VOUT2     17
#define PIN_VTH       26


//Variables globales

float temp=0;
float hum=0;
float lux=0;
int particules=0;
float decibels=0;

// Declaration du DHT sensor.
DHT dht(DHTPin, DHTTYPE); 

//Pour le capteur MAX4466
const int sampleWindow = 2000; 
unsigned int sample;

//Pour la connexion Wi-Fi
const char* ssid = "Sense";//  "PAT13935 5252";
const char* password = "i90QcOPdtcVL"; //"B|10i859";
WebServer server(8081);//port 8081
DFRobot_VEML7700 als;

//Taille du JSON
StaticJsonDocument<600> firstJSON;
char buffer[600];

//Pour le capteur de particule
int count=0;
static int counts_idx = 0;
static int counts20sec_vout1 = 0;
static int counts20sec_vout2 = 0;

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

      
      Serial.print("VOUT1 :");
      Serial.print(counts_vout1);
      Serial.print("[count], VOUT2 :");
      Serial.print(counts_vout2);
      Serial.print("[count], VOUT1-VOUT2 :");
      Serial.print(counts_vout1 - counts_vout2);
      Serial.print("   [count]  = ");
      
      particules = counts_vout1 - counts_vout2;
      Serial.println(particules);
      particulesReady = true;
   }
    // Pause the task for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
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
      soundReady = true;
      
      //Serial.println("volts");
      //Serial.println(volts);
      Serial.println("decibels");
      Serial.println(decibels);
      Serial.println("");
    }
    // Pause the task for 500ms
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

float ppm(){  

  int val = analogRead(MQ135PIN);
  float res =  ((4095./(float)val) * 3.3 - 1.)*10.0;
  float ppm = 116.6020682 * pow((res/76.63), -2.769034857);
  
  Serial.println("ppm : ");
  Serial.println(ppm);
  Serial.println();
  
  return ppm;
}

void temperature(){
  
//use the functions which are supplied by library.
hum = dht.readHumidity();
Serial.println("hum: ");
Serial.println(hum);
Serial.println("");
// Read temperature as Celsius (the default)
temp = dht.readTemperature();
Serial.println("temp: ");
Serial.println(temp);
Serial.println("");
// Check if any reads failed and exit early (to try again).
  if (isnan(hum) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
  } 
}

//Mise en place du JSON
void setupJson(){
  firstJSON["all"]["temperature"] = 0;
  firstJSON["all"]["humidity"] = 0;
  firstJSON["all"]["brightness"] = 0;
  firstJSON["all"]["particle"] = 0;
  firstJSON["all"]["airquality"] = 0;
  firstJSON["all"]["sound"] = 0;
}

//Lectures des mesures et mise en place dans le JSON
void updateData(){
  while (!particulesReady || !soundReady){
  }
  temperature();
  als.getALSLux(lux);
  Serial.println("lux: ");
  Serial.println(lux);
  Serial.println("");
  firstJSON["all"]["temperature"] = temp;
  firstJSON["all"]["humidity"] = hum;
  firstJSON["all"]["brightness"] = lux;
  firstJSON["all"]["particle"] = particules;
  firstJSON["all"]["airquality"] = ppm();
  firstJSON["all"]["sound"] = decibels;

  particulesReady = false;
  soundReady = false;
  Serial.println("Succesfull read");
  Serial.println("");
}

//Fonction setup Wifi
void setupWifi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);

  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}
/*
server.on("/get-message", HTTP_GET, [](AsyncWebServerRequest *request) {
  StaticJsonDocument<100> data;
  if (request->hasParam("message"))
  {
    data["message"] = request->getParam("message")->value();
  }
  else {
    data["message"] = "No message parameter";
  }
  String response;
  serializeJson(data, response);
  request->send(200, "application/json", response);
});*/

void handleRoot() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buffer);
  Serial.println("request send : ");
  Serial.println(buffer);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  Serial.println("handleNotFound() ");
  Serial.println(message);
}

void setup() {
  
  //Mise en place de la liaison série avec le moniteur
  Serial.begin(115200);
  
  //Configuration des pins
  pinMode(DHTPin, INPUT);
  
  //call begin to start dht and als sensors
  dht.begin();
  als.begin();
  
  //pinMode(MQ135PIN,INPUT);
  pinMode(MAXPIN,INPUT);
  
  
  pinMode(PIN_VOUT1, INPUT);
  pinMode(PIN_VOUT2, INPUT);
  
  pinMode(PIN_VTH, OUTPUT);
  //Lecture de la tension sur le pin VTH (librairie analogwrite ne marche pas
  dacWrite(PIN_VTH, 0.125 / (3.3 / 1024));
  //dacWrite(PIN_VTH, 0.5 / (3.3 / 256));
  //analogWrite(PIN_VTH, 0.125 / (3.3 / 1024));

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
  
  //Mise en place de la Wi-Fi
  //setupWifi();
  
  //Mise en place du Json
  setupJson();
}

void loop() {
  //On s'occupe du serveur
  //server.handleClient();
  //On récupère les mesures physiques
  updateData();
  serializeJson(firstJSON, buffer);
}
