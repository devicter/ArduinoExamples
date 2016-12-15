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
 */
 // Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_SPECIAL_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

#define MY_NODE_ID             AUTO

// Enable repeater functionality for this node
//#define MY_REPEATER_FEATURE

#define MY_RF24_CHANNEL       XXX

// Enable to support OTA for this node (needs DualOptiBoot boot-loader to fully work)
#define MY_OTA_FIRMWARE_FEATURE

// Connected we have to move CE/CSN pins for NRF radio
#define MY_RF24_CE_PIN         7
#define MY_RF24_CS_PIN         6

#define MY_DEFAULT_TX_LED_PIN  9  // Transfer led pin
// Inverses the behavior of leds
#define MY_WITH_LEDS_BLINKING_INVERSE
 
#define SKETCH_NAME "OTA R1N Ready"
#define SKETCH_VERSION "0.0.1"

#define RELAY_PIN_1             10 //  Relay 

// Load mysensors library	
#include <MySensors.h>

#ifndef MY_OTA_FIRMWARE_FEATURE
#include "drivers/SPIFlash/SPIFlash.cpp"
#endif
SPIFlash flash(8, 0x1F65);

void presentation()  
{   
// Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
}

void before() { 
     // Then set relay pins in output mode
  pinMode(RELAY_PIN_1, OUTPUT);
}
     
void setup() 
{
  digitalWrite(RELAY_PIN_1, 0);
  
  Serial.print("Version: ");
  Serial.print(SKETCH_VERSION);
  Serial.println(" READY");
  Serial.flush();
}

void loop()
{

}
