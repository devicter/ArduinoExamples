  /**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 *
 * REVISION HISTORY
 * Version 1.0 - Created by greengo
 
 * Support Forum: http://devicter.blogspot.ru/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 * The ArduinoGateway prints data received from sensors on the serial link. 
 * The gateway accepts input on seral which will be sent out on radio network.
 *
 * The SGW code is designed for Raspberry PI nFR24 Add-on 328p / 16MHz
 *
 * Wire connections (OPTIONAL):
 * - Inclusion button should be connected between digital pin 3 and GND  
 * - RX/TX/ERR leds need to be connected between +5V (anode) and digital pin 6/5/4 with resistor 270-330R in a series
 *
 * LEDs (OPTIONAL):
 * - To use the feature, uncomment MY_LEDS_BLINKING_FEATURE in MyConfig.h
 * - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
 * - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
 * - ERR (red) - fast blink on error during transmission error or recieve crc error 
 * 
 */
// Enable debug prints to serial monitor
//#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

#define MY_RF24_CHANNEL 71

// Set LOW transmit power level as default, if you have an amplified NRF-module and
// power your radio separately with a good regulator you can turn up PA level. 
#define MY_RF24_PA_LEVEL RF24_PA_HIGH

// Enable serial gateway
#define MY_GATEWAY_SERIAL

#define MY_RF24_CE_PIN 10
#define MY_RF24_CS_PIN  9

// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 200

// Inverses the behavior of leds
#define MY_WITH_LEDS_BLINKING_INVERSE

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
#define MY_INCLUSION_BUTTON_FEATURE

#define SKETCH_NAME "SGW nRF24 Add-on"
#define SKETCH_VERSION "1.0.0"

// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60 
// Digital pin used for inclusion mode button
#define MY_INCLUSION_MODE_BUTTON_PIN  3 

// Uncomment to override default HW configurations
#define MY_DEFAULT_ERR_LED_PIN 4  // Error led pin (red)
#define MY_DEFAULT_TX_LED_PIN  5  // Transfer led pin (yellow)
#define MY_DEFAULT_RX_LED_PIN  6  // Receive led pin (green)

#include <SPI.h>
#include <MySensors.h>  

void setup() { 
  // Setup locally attached sensors
}

void presentation() {
 // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION); 
}

void loop() { 
  // Send locally attached sensor data here 
}
