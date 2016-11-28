/*
This sketch is for a Energy Monitor v 1.0 
and MySensors 2.0

 modified
 17 Oct 2016
 by greengo
 */

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_NODE_ID       AUTO

#define SKETCH_NAME "Energy Monitor v1.0"
#define SKETCH_VERSION "2.0.0"

// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

#define MY_RF24_CHANNEL  70

// Connected we have to move CE/CSN pins for NRF radio
#define MY_RF24_CE_PIN   7
#define MY_RF24_CS_PIN   8

#include <SPI.h>
#include <MySensors.h>
#include "EmonLib.h"
#include <Time.h>
#include <LCD5110_Graph_SPI.h>

LCD5110 myGLCD(5,6,3);
//extern unsigned char SmallFont[];
extern uint8_t SmallFont[];

#define WINDOW 15

// How many milli seconds between each measurement
#define MEASURE_INTERVAL    1000 //for Debug 1 sec

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL 600 //every 10 min

unsigned long SLEEP_TIME = 1000; // Sleep time between reads (in milliseconds)

int TEMP_TRANSMIT_THRESHOLD = 0; //35 Watt

#define CHILD_ID_POWER 0
#define CHILD_ID_FLOATING_CUR_SENSOR 1 //floating sensor

EnergyMonitor emon;

unsigned long CHECK_TIME = millis();

MyMessage IrmsMsg(CHILD_ID_POWER, V_WATT);
MyMessage kWhMsg(CHILD_ID_POWER, V_KWH);
MyMessage FloatingMsg(CHILD_ID_FLOATING_CUR_SENSOR, V_VAR1);

// Global settings
int measureCount = 0;
boolean transmission_occured = false;

boolean timeReceived = false;
unsigned long lastUpdate = 0, lastRequest = 0;

// Storage of old measurements
double realWatt = 0;
float realkWt = 0;
double Irms;
float lastIrms = 0;
int Floating;
float PMin = 99;
float PMax = 0;

int stateFloating = 0;

long wattsum  = 0;
long wattsumS = 0;
//int seconds   = 0;
double wh     = 0;
double kwh    = 0;
double whS    = 0;
double kwhS   = 0;

void setup()  
{ 
 double cIrms = 0;  
 //   emon.current(0, 111.1);       // Current: input pin, calibration.
  emon.current(0, 11.0); 

 double Irms[WINDOW];   
         Irms[0] = emon.calcIrms(1480); // первое значение при измерении явно "кривое"
  //    Serial.println("calculate delta");
      for (int i=0; i<WINDOW; i++) {
        Irms[i] = emon.calcIrms(1480);
     cIrms = cIrms + Irms[i];
        delay(100);      
      } 
      
sendPowerMeasurements(true);
  wait(500);
  
  //request Time
  requestTime();
  
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
}

void presentation()  
{   
//  // Send the sketch version information to the gateway and Controller
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);

  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_POWER, S_POWER, "POWER");
//  present(CHILD_ID_POWER_COUNTER, S_CUSTOM, "COUNTER");
  present(CHILD_ID_FLOATING_CUR_SENSOR, S_CUSTOM, "Float");
}

// This is called when a new time value was received
void receiveTime(unsigned long time) {
  Serial.print("Time value received: ");
  Serial.println(time);
  // Ok, set incoming time
  setTime(time);
  timeReceived = true;
}

void loop()      
{       
   unsigned long now = millis();

  // If no time has been received yet, request it every 10 second from controller
  // When time has been received, request update every hour
  if ((!timeReceived && now - lastRequest > (unsigned long)10 * 1000)
      || (timeReceived && now - lastRequest > (unsigned long)60 * 1000 * 60)) {
    // Request time from controller.
    Serial.println("Requesting time");
    Serial.println("Time now: ");
    Serial.println(timeReceived);
    wait(300);
    requestTime();
    lastRequest = now;
  }
  //*********************************
   
   displayUpdate();
   
  int HH = hour();
  int MM = minute();
  int SS = second();
   
  unsigned long NOW_TIME = millis();
  
  if(NOW_TIME - CHECK_TIME >= SLEEP_TIME)
  { 
  measureCount ++;
  
    double Irms = emon.calcIrms(1480);  // Calculate Irms only
    realWatt  = (emon.Irms * 220);  // Extract Real Power into variable
    long watt = Irms * 220.0;
    realkWt   = (Irms * 0.220);

    wattsum = wattsum + watt;
    wattsumS = wattsumS + watt;
    //   seconds++;

/*
    Serial.print("    Real TIME   =    ");
    Serial.print(hour());
    Serial.print(" : ");
    Serial.print(minute());
    Serial.println();
*/
    // hours KW reading
    if ( (MM >= 59) & (SS >= 59)) {
      wh = wh + wattsum / 3600;
      kwh = wh / 1000;
      send(kWhMsg.set(kwh, 4)); // Send kwh value to gw

      wattsum = 0;
      wh = 0;
      //   seconds = 0;
      // end of hourly KW reading
    }
  
    // fixing minimum and maximum values
  if ( realkWt > PMax) PMax = realkWt;
  if ( realkWt < PMin &&  realkWt > 0.05) PMin = realkWt;   
  
  
  bool forceTransmit = false;
  transmission_occured = false;
 
  
  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
   
      send(kWhMsg.set(kwh, 4)); // Send kwh value to gw 
    
  }
     sendPowerMeasurements(forceTransmit);
   
 // Serial.print(" measureCount: ");
 // Serial.println(measureCount); 
 // Serial.print(" ");
  
       CHECK_TIME = NOW_TIME;
     } 
  }

void sendPowerMeasurements(bool force)
{
  bool tx = force;       
  
 // Set relay to last known state (using eeprom storage)
  TEMP_TRANSMIT_THRESHOLD = loadState(CHILD_ID_FLOATING_CUR_SENSOR);
  Floating = TEMP_TRANSMIT_THRESHOLD;

  //  Serial.print("TEMP_TRANSMIT_THRESHOLD: ");Serial.println(TEMP_TRANSMIT_THRESHOLD);

  float diffIrms = abs(lastIrms - realWatt);

  //  Serial.print(F("IrmsDiff :"));Serial.println(diffIrms);

  if (diffIrms > TEMP_TRANSMIT_THRESHOLD) tx = true;

  if (tx) {
    measureCount = 0;

    //   Serial.print("Watt: ");Serial.println(realWatt);
  
    send(IrmsMsg.set(realWatt, 1));

    lastIrms = realWatt;
    transmission_occured = true;
  }
/*
  //  Serial.print("seconds : "); Serial.println(seconds);
  Serial.print(" ***************** ");
  Serial.print("wh : "); Serial.println(wh);
  Serial.print(" ");
  Serial.print("kwhS : "); Serial.println(kwhS, 4);
  Serial.print(" ***************** ");
  Serial.print("wattsum : "); Serial.println(wattsum);
  Serial.print(" ");
  Serial.print("wattsumS : "); Serial.println(wattsumS);
  Serial.print(" ");
  Serial.print("Irms : "); Serial.println(Irms, 4);
  Serial.print(" ");
  Serial.print("kwh : "); Serial.println(kwh, 4);
  Serial.println(" ");
  Serial.print("Watt: "); Serial.println(realWatt);
  Serial.println(" ");
  Serial.print("realkWt : ");Serial.println(realkWt);
*/
  
}  
  // LCD Nokia 
 void displayUpdate()
 {
// myGLCD.clrScr();
  
 char tbuf[8];
 char sbuf[12];
    
    dtostrf(realWatt,5,2,tbuf);
    sprintf(sbuf, " %s Watt", tbuf);
    myGLCD.print(sbuf, 20, 0);   //от края, высота 
    myGLCD.print("PWR:", 0, 0);   //от края, высота  
      
    dtostrf(Irms,5,2,tbuf);
    sprintf(sbuf, " %s Amp", tbuf);
    myGLCD.print(sbuf, 20, 10);   //от края, высота 
    myGLCD.print("IRM:", 0, 10);   //от края, высота 
    
    dtostrf(PMin,5,2,tbuf);
    sprintf(sbuf, " %s kWt", tbuf);
    myGLCD.print(sbuf, 20, 20);   //от края, высота 
    myGLCD.print("MiN:", 0, 20);   //от края, высота 
    
    dtostrf(PMax,5,2,tbuf);
    sprintf(sbuf, " %s kWt", tbuf);
    myGLCD.print(sbuf, 20, 30);   //от края, высота 
    myGLCD.print("MaX:", 0, 30);   //от края, высота 
    
    dtostrf(Floating,5,2,tbuf);
    sprintf(sbuf, " %s Watt", tbuf);
    myGLCD.print(sbuf, 20, 40);   //от края, высота 
    myGLCD.print("flt:", 0, 40);   //от края, высота 
    
    myGLCD.update(); 
    
}
//*********************************************
 void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if (message.isAck()) {
     Serial.println("This is an ack from gateway");
  }

  if (message.type == V_VAR1) {
     // Change state
     stateFloating = message.getInt();
 
     // Store state in eeprom
     saveState(CHILD_ID_FLOATING_CUR_SENSOR, stateFloating);
    
     // Write some debug info
     Serial.print("Incoming change for sensor:");
     Serial.print(message.sensor);
     Serial.print(", Delta =: ");
     Serial.println(message.getInt());
   //  Serial.print(", New status: ");
   //  Serial.println(message.getBool());
   } 
 }   
