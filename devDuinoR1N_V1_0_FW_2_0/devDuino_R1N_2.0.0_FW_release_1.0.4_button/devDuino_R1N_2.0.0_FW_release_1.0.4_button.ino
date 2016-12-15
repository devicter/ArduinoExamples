/*
This sketch is for a devDuino Relay 1 Node v1.0 with the support OTA 
http://www.seeedstudio.com/wiki/DevDuino_R1N_V1.0
Based on the framework, from Mysensors.org
http://www.mysensors.org

Support wireless updates available via 
Software MYSController
http://www.mysensors.org/controller/myscontroller
Web controller MyController.org
http://www.mycontroller.org/#/home 

Used bootloader from devicter.ru
https://github.com/devicter/bootloaders
Designed on the basis of Lowpowerlab 
https://lowpowerlab.com/guide/moteino/

 modified
 24 Nov 2016
 by greengo
 
 * REVISION HISTORY
 * Version 1.0 - Created by greengo
 * 
 */
 
 // Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

#define MY_NODE_ID              AUTO

#define MY_RF24_CHANNEL         10//70

// Enable to support OTA for this node (needs DualOptiBoot boot-loader to fully work)
#define MY_OTA_FIRMWARE_FEATURE

// Connected we have to move CE/CSN pins for NRF radio
#define MY_RF24_CE_PIN           7
#define MY_RF24_CS_PIN           6

#define MY_DEFAULT_TX_LED_PIN    9  // Transfer led pin
// Inverses the behavior of leds
#define MY_WITH_LEDS_BLINKING_INVERSE

#define SKETCH_NAME "devDuino R1N v1.0"
#define SKETCH_VERSION "1.0.4"

// Load mysensors library	
#include <MySensors.h>
#include <Bounce2.h>

#ifndef MY_OTA_FIRMWARE_FEATURE
#include "drivers/SPIFlash/SPIFlash.cpp"
#endif

SPIFlash flash(8, 0x1F65);

#define CHILD_ID_RELAY            0
#define CHILD_ID_CURRENT_SENSOR   1

#define LED_PIN                   9 // Led
#define CURRENT_SENSOR_PIN       A3 // Current sensor

#define BUTTON_PIN                4 // Button

#define RELAY_PIN_1              10 //  Relay 

#define RELAY_ON                  1
#define RELAY_OFF                 0
 
// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL 601 //every 10 min
//#define FORCE_TRANSMIT_INTERVAL 5 //every 10 min

unsigned long SLEEP_TIME = 1000; // Sleep time between reads (in milliseconds)

#define CURRENT_TRANSMIT_THRESHOLD  0.20

unsigned long CHECK_TIME = millis();

int buttonState = LOW;
boolean lightState = false;
int measureCount = 0;
float lastCurrent = 0;
boolean transmission_occured = false;

float sensorValue = 0;
int countvalues = 50;     // how many values must be averaged
float ZeroLevel = 514;    // Zero level
float kVolt = 0.0506;     // conversion factor

// Instantiate a Bounce object
Bounce debouncer = Bounce();

MyMessage msgRelayStatus(CHILD_ID_RELAY, V_LIGHT);
MyMessage msgWatt(CHILD_ID_CURRENT_SENSOR, V_CURRENT);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 9 as an output.
  pinMode(LED_PIN, OUTPUT);
     // Then set relay pins in output mode
  pinMode(RELAY_PIN_1, OUTPUT);
   // Setup the button
  pinMode(BUTTON_PIN,INPUT);
  digitalWrite(BUTTON_PIN,HIGH);
  // After setting up the button, setup Bounce object
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(5);  
  // Then set current sensor pins in input mode
  pinMode(CURRENT_SENSOR_PIN , INPUT);
  digitalWrite(CURRENT_SENSOR_PIN, HIGH);
   
//   lightState = gw.loadState(CHILD_ID_RELAY);
//  digitalWrite(RELAY_PIN_1, lightState?RELAY_ON:RELAY_OFF);

  digitalWrite(RELAY_PIN_1, lightState);
  wait(50);
  sendPowerMeasurements(true);
  send(msgRelayStatus.set(lightState), true);
//  request(CHILD_ID_RELAY, V_LIGHT);

  Serial.print("Version: ");
  Serial.print(SKETCH_VERSION);
  Serial.println(" READY");
  Serial.flush();
}

void presentation()  
{   
// Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  
  present(CHILD_ID_RELAY, S_BINARY, "LIGHT");
  present(CHILD_ID_CURRENT_SENSOR, S_MULTIMETER, "AMP");  
}

// the loop function runs over and over again forever
void loop() {
  
  pushButton();
   
  unsigned long NOW_TIME = millis();
  
  if(NOW_TIME - CHECK_TIME >= SLEEP_TIME)
  { 
  measureCount ++; 
   bool forceTransmit = false;
   transmission_occured = false;
 
 if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
    send(msgRelayStatus.set(lightState));
  }
   sendPowerMeasurements(forceTransmit);
   
      CHECK_TIME = NOW_TIME;
     }      
  }

void sendPowerMeasurements(bool force) {
   
   bool tx = force;
 
int CurrentValue = Current(CURRENT_SENSOR_PIN);

float  current = kVolt * (CurrentValue - ZeroLevel);
  
   wait(350);
   
   if (current < 2.80) current = 0;
   else
      current = current;

// float realWatt = (current * 220);
      
  float diffCurrent = abs(lastCurrent - current);
   
//   Serial.print(F(" >diffCurrent :"));Serial.println(diffCurrent);
   
   if (diffCurrent > CURRENT_TRANSMIT_THRESHOLD) tx = true;
   if (tx) {
    measureCount = 0;
     
     send(msgWatt.set(current,1));
  }
   
    lastCurrent = current;    
    transmission_occured = true;
}

void pushButton(){ 
  
  boolean stateChanged = debouncer.update();
  int state = debouncer.read();

  // Detect the falling edge
   if ( stateChanged && state == LOW ) {
    
       if ( buttonState == LOW ) {
         buttonState = HIGH;
       } else {
         buttonState = LOW;        
       }
   digitalWrite(RELAY_PIN_1, buttonState);
   send(msgRelayStatus.set(buttonState));
   lightState = buttonState;    
   }  
}

int Current(int sensorPin) {
  
  float TMPsensorValue = 0;
  int count = 0;
 
  for (count =0; count < countvalues; count++) {
    // read the value from the sensor:
    TMPsensorValue = analogRead(sensorPin);
 //   wait(30);
    // make average value
    sensorValue = (sensorValue+TMPsensorValue)/2;
    }
  return sensorValue;  
  }
//*******************************************
void receive(const MyMessage &message) {

  if ((message.type == V_LIGHT) && (message.sensor == CHILD_ID_RELAY)&& !mGetAck(message)) {
     // Change relay state
     lightState = message.getBool();
     digitalWrite(RELAY_PIN_1, lightState?RELAY_ON:RELAY_OFF);
     // Store state in eeprom
     //gw.saveState(CHILD_ID_RELAY, lightState); 
       buttonState = lightState;     
   } 
}
