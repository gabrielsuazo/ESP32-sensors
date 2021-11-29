
// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID "TMPLQrBiNVeP"
#define BLYNK_DEVICE_NAME "ESP32sensors"
#define BLYNK_AUTH_TOKEN "tuCubZAOwxDeAFWPxJMtFDqoUJ1u_uHj"

// Comment this out to disable prints and save space
//#define BLYNK_PRINT Serial

//Librairies
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESPmDNS.h>
#include <Arduino.h>
#include "DFRobot_VEML7700.h"
#include "DHT.h"

char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Freebox-8CB84E";
char pass[] = "incisis@-obstem-perdebita6-verbulis@";

//Define variables et ports

BlynkTimer timer;

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

DFRobot_VEML7700 als;

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

//Lectures des mesures 
// This function sends the data every three seconds to the correspondent Virtual Pins.
void updateData(){
  while (!particulesReady || !soundReady){
  }
  temperature();
  als.getALSLux(lux);
  Serial.println("lux: ");
  Serial.println(lux);
  Serial.println("");
  Blynk.virtualWrite(V4, temp);
  Blynk.virtualWrite(V1, hum);
  Blynk.virtualWrite(V0, lux);
  Blynk.virtualWrite(V2, particules);
  Blynk.virtualWrite(V3, ppm());
  Blynk.virtualWrite(V5, decibels);

  particulesReady = false;
  soundReady = false;
  Serial.println("Succesfull read");
  Serial.println("");
}


// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{

}

void setup()
{
  // Debug console
  Serial.begin(115200);

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

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

  // Setup a function to be called every 3 seconds
  timer.setInterval(3000L, updateData);
}

void loop()
{
  Blynk.run();
  timer.run();

}
