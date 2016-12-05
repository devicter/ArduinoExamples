/*
This sketch is for a DevDuino 4.0 
http://www.seeedstudio.com/depot/devDuino-Sensor-Node-V4-ATmega-328-Integrated-temperature-humidity-sensor-p-2279.html?cPath=19_22
and MySensors 2.0.0
 
 modified
 4 Oct 2016
 by greengo
*/

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

#define MY_NODE_ID               AUTO

#define MY_RF24_CHANNEL          71

// Connected we have to move CE/CSN pins for NRF radio
#define MY_RF24_CE_PIN            8
#define MY_RF24_CS_PIN            7

#include <MySensors.h>
#include "HTU21D.h"
#include <Wire.h>
#include <RunningAverage.h>

// Define a static node address, remove if you want auto address assignment
#define SKETCH_NAME "devDuino SNv4.0"
#define SKETCH_VERSION "1.0.0"

#define AVERAGES 2

// How many milli seconds between each measurement
#define MEASURE_INTERVAL 50000 //for Real Work 50 sec for internal use Home

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL 30 

// LED blinks during data transmission. Greater battery energy consumption!
#define LED_BLINK_WAIT_TRANSMIT  //turn off for external use (Street)

#define TEMP_TRANSMIT_THRESHOLD 0.5
#define HUMI_TRANSMIT_THRESHOLD 0.5

// Pin definitions
#define LED_PIN                   9 // LED 

// Child sensor ID's
#define CHILD_ID_TEMP             0
#define CHILD_ID_HUM              1
// Uncomment the line below, to transmit battery voltage as a normal sensor value
#define CHILD_ID_BATT_SENSOR      2

int oldBattPct = 0;
float temp = 0;

MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);

#ifdef CHILD_ID_BATT_SENSOR
MyMessage msgBatt(CHILD_ID_BATT_SENSOR, V_VOLTAGE);
#endif

// Global settings
int measureCount = 0;

int measureCountTemp = 0;
int measureCountHum = 0;

boolean ota_enabled = false; 
int sendBattery = 0;
boolean highfreq = true;
boolean transmission_occured = false;
float sendVCC; 

HTU21D myHumidity;

// Storage of old measurements
float lastTemperature = 0;
int lastHumidity = 0;
long lastBattery = 0;

RunningAverage raHum(AVERAGES);

/****************************************************
 * Setup code 
 ****************************************************/
void setup() {
 
  // initialize digital pin 9 as an output.
  pinMode(LED_PIN, OUTPUT); 

myHumidity.begin(); 

raHum.clear();

sendTempHumidityMeasurements(true);

sendBattLevel(true);
  digitalWrite(LED_PIN, HIGH);
  wait(500);
 send(msgBatt.set(sendVCC,2));
  digitalWrite(LED_PIN, LOW);
}

void presentation()  
{   
 // Send the sketch version information to the gateway and Controller
 sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
   
 // Register all sensors to gw (they will be created as child devices)
 // Register sensors (id, type, description, ack back)
   present(CHILD_ID_TEMP,S_TEMP, "TEMP"); 
   present(CHILD_ID_HUM, S_HUM, "HUM"); 
#ifdef CHILD_ID_BATT_SENSOR
   present(CHILD_ID_BATT_SENSOR, S_MULTIMETER, "VOLT");
#endif      
}
/***********************************************
 *  Main loop function
 ***********************************************/
void loop() {
  measureCount ++;

  measureCountTemp ++;
  measureCountHum ++;

  sendBattery ++;
  
  bool forceTransmit = false;
  transmission_occured = false;
  if ((measureCount == 3) && highfreq) 
  {
    clock_prescale_set(clock_div_8); // Switch to 1Mhz for the reminder of the sketch, save power.
    highfreq = false;
  } 
  
  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }
    
  sendTempHumidityMeasurements(forceTransmit);
  
  if (sendBattery > 40) 
  {
     sendBattLevel(forceTransmit); // Not needed to send battery info that often
     sendBattery = 0;
  }

  sleep(MEASURE_INTERVAL);  
}
/*********************************************
 * Sends temperature and humidity sensor
 *
 * Parameters
 * - force : Forces transmission of a value (even if it's the same as previous measurement)
 *********************************************/
void sendTempHumidityMeasurements(bool force)
{
  bool tx = force;
    bool tx1 = force;

    //get the Temperature and Humidity from the onboard sensor.
  float temp = myHumidity.readTemperature();
  int humidity = myHumidity.readHumidity();
  
  raHum.addValue(humidity);
  
  float diffTemp = abs(lastTemperature - temp);
  float diffHum = abs(lastHumidity - raHum.getAverage());

  //Serial.print(F("TempDiff :"));Serial.println(diffTemp);
  //Serial.print(F("HumDiff  :"));Serial.println(diffHum); 

  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;
  if (tx) {
   measureCountTemp = 0;  
  // LED 
#ifdef LED_BLINK_WAIT_TRANSMIT
   digitalWrite(LED_PIN, HIGH);      
    send(msgTemp.set(temp,1));
   digitalWrite(LED_PIN, LOW);
    }
 #else
   send(msgTemp.set(temp,1));
    }
#endif  
  
  if (isnan(diffHum)) tx1 = true; 
  if (diffHum > HUMI_TRANSMIT_THRESHOLD) tx1 = true;

  if (tx1) {
     measureCountHum = 0;
 // LED 
#ifdef LED_BLINK_WAIT_TRANSMIT
   digitalWrite(LED_PIN, HIGH);      
    send(msgHum.set(humidity));
   digitalWrite(LED_PIN, LOW);
 #else
   send(msgHum.set(humidity));
#endif 
  }  
 //   Serial.print("T: ");Serial.println(temp);
 //   Serial.print("H: ");Serial.println(humidity);
 //   Serial.print("measureCount: ");Serial.println(measureCount);
 //   Serial.print("sendBattery: ");Serial.println(sendBattery);

    lastTemperature = temp;
    lastHumidity = humidity;
    transmission_occured = true;
}

/********************************************
 *
 * Sends battery information (battery percentage)
 *
 * Parameters
 * - force : Forces transmission of a value
 *
 *******************************************/
void sendBattLevel(bool force)
{
  if (force) lastBattery = -1;
  long vcc = readVcc();
   sendVCC = vcc/1000.0;

  if (vcc != lastBattery) {
    lastBattery = vcc;

#ifdef CHILD_ID_BATT_SENSOR
    send(msgBatt.set(sendVCC,2));
#endif

    // Calculate percentage
    vcc = vcc - 1900; // subtract 1.9V from vcc, as this is the lowest voltage we will operate at
    
    long percent = vcc / 14.0;
    sendBatteryLevel(percent);
    transmission_occured = true;
  }
}
/*******************************************
 *
 * Internal battery ADC measuring 
 *
 *******************************************/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADcdMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
//  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
   result = 1149900L / result; // Calculate Vcc (in mV); 1 149 900 = 1.124*1023*1000
  return result; // Vcc in millivolts
 
}
