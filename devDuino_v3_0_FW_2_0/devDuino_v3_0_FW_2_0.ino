/*
 This sketch is for a devDuino SN v3 
http://www.seeedstudio.com/wiki/DevDuino_Sensor_Node_V3.0_(ATmega_328)
and MySensors 2.0.0

This sketch is a modification of code written
Version 1.3 - Thomas Bowman Mørch
for sensor Sensebender Micro
http://www.mysensors.org/hardware/micro

 modified
 05 Oct 2016
 by greengo
 */

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24

#define MY_NODE_ID               AUTO

#define MY_RF24_CHANNEL          XXX

// Connected we have to move CE/CSN pins for NRF radio
#define MY_RF24_CE_PIN            8
#define MY_RF24_CS_PIN            7

#include <MySensors.h>

#define SKETCH_NAME "devDuino SNv3.0"
#define SKETCH_VERSION "1.0.0"

// How many milli seconds between each measurement
#define MEASURE_INTERVAL 60000 //for Debug 60 sec

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to the controller
#define FORCE_TRANSMIT_INTERVAL 20 //30 

//LED Blink wait for LED blinks during data transmission. Greater battery energy consumption!
#define LED_BLINK_WAIT_TRANSMIT    

#define TEMP_TRANSMIT_THRESHOLD 0.5

#define LED_PIN                 9 // LED 

#define CHILD_ID_TEMP           0
#define CHILD_ID_BATT_SENSOR    2

#define TEMP_SENSE_PIN   A2  // Input pin for the Temp sensor MCP9700
float TEMP_SENSE_OFFSET = -0.01;

float temp = 0;

// Global settings
int measureCount = 0;
boolean ota_enabled = false; 
int sendBattery = 0;
boolean highfreq = true;
boolean transmission_occured = false;
float sendVCC; 

// Storage of old measurements
float lastTemperature = -100;
long lastBattery = -100;

MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

#ifdef CHILD_ID_BATT_SENSOR
MyMessage msgBatt(CHILD_ID_BATT_SENSOR, V_VOLTAGE);
#endif

// the setup function runs once when you press reset or power the board
void setup() {
 
  // initialize digital pin 9 as an output.
  pinMode(LED_PIN, OUTPUT); 
 
sendTempMeasurements(true);
sendBattLevel(true);

  digitalWrite(LED_PIN, HIGH);
  wait(100);
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
   present(CHILD_ID_BATT_SENSOR, S_MULTIMETER, "VOLT");    
}

// the loop function runs over and over again forever
void loop() {
  measureCount ++;
  sendBattery ++;
  bool forceTransmit = false;
  transmission_occured = false;
  if ((measureCount == 3) && highfreq) 
  {
    if (!ota_enabled) clock_prescale_set(clock_div_8); // Switch to 1Mhz for the reminder of the sketch, save power.
    highfreq = false;
  } 
  
  if (measureCount > FORCE_TRANSMIT_INTERVAL) { // force a transmission
    forceTransmit = true; 
    measureCount = 0;
  }

  sendTempMeasurements(forceTransmit);
  if (sendBattery > 40) 
  {
     sendBattLevel(forceTransmit); // Not needed to send battery info that often
     sendBattery = 0;
  }

  smartSleep(MEASURE_INTERVAL);  
}
/********************************************
 *
 * Sends battery information (battery percentage)
 *
 * Parameters
 * - force : Forces transmission of a value
 *
 *******************************************/

void sendTempMeasurements(bool force)
{
  bool tx = force;

 float temp = readMCP9700(TEMP_SENSE_PIN,TEMP_SENSE_OFFSET); //temp pin and offset for calibration
   
  float diffTemp = abs(lastTemperature - temp);

//  Serial.print(F("TempDiff :"));Serial.println(diffTemp);

  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) tx = true;

  if (tx) {
    measureCount = 0;
     
//    Serial.print("T: ");Serial.println(temp);
 // LED 
#ifdef LED_BLINK_WAIT_TRANSMIT
   digitalWrite(LED_PIN, HIGH);
   wait(100);   
     send(msgTemp.set(temp,1));
   digitalWrite(LED_PIN, LOW);
#else
     send(msgTemp.set(temp,1));
#endif
    lastTemperature = temp;
    transmission_occured = true;
  }
}

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
 * Internal TEMP sensor 
 *
 *******************************************/
float readMCP9700(int pin,float offset)

{
  analogReference(INTERNAL);
  
  analogRead(A0); //perform a dummy read to clear the adc
  delay(20);
    
  for (int n=0;n<5;n++)
    analogRead(pin);
  
  int adc=analogRead(pin);
  float tSensor=((adc*(1.1/1024.0))-0.5+offset)*100;
  float error=244e-6*(125-tSensor)*(tSensor - -40.0) + 2E-12*(tSensor - -40.0)-2.0;
  float temp=tSensor-error;
 
  return temp;
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
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts 
}
