#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdio.h>
#include <stdlib.h>
#include "arduino2.h"                                                                                                                                                                                                                                                                                                                                                                                              7
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define STACK_SIZE    200

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// DataPacket Structure
typedef struct DataPacket{
  int16_t type;
  int16_t armID;
  int16_t accelHand[3];
  int16_t gyroID;
  int16_t gyroscope[3];
  int16_t thighID;
  int16_t accelThigh[3];
  int16_t powerID;
  int16_t voltage;
  int16_t current;
  int16_t power;
} DataPacket;

// Packet Declaration
DataPacket packet[2];
DataPacket data;
int numberOfPackets = 0;

// Device ID
int16_t ARM_ID = 0, GYRO_ID = 1, THIGH_ID = 2, POWER_ID = 3;

// Packet code
int16_t ACK = 0, NAK = 1, HELLO = 2, READ = 3, WRITE = 4, DATA_RESP = 5;

// Create Sempaphore
SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();

// Class default I2C address is AD0 low = 0x68
MPU6050 accelgyro;

// Constants
const int CURRENT_SENSOR_PIN = A0;    // Input pin for measuring Volt
const float RS = 0.1;                 // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;            // Reference voltage for analog read
const int VOLTAGE_SENSOR_PIN = A1;
const GPIO_pin_t gyroFirst = DP30;
const GPIO_pin_t accSec = DP29;
const GPIO_pin_t accThird = DP31;
const TickType_t xPeriod = 16;
/*int gyroFirst = 30;
int accSec = 29;
int accThird = 31;*/

// Function to initialise the data packet siwth some values
void initializeDataPacket(){
  data.type = WRITE;
  data.armID = ARM_ID;
  data.gyroID = GYRO_ID;
  data.thighID = THIGH_ID;
  data.powerID = POWER_ID;
}

// Method to Handshake with the RaspberryPI
void handshake() {
  int reply;
  int handshake_flag = 1;
  
  while (handshake_flag == 1) {
    if (Serial1.available()) {
      reply = Serial1.read();
      if (reply == HELLO) {
        Serial1.println(ACK);
      }
      if (reply == ACK) {
        handshake_flag = 0;
      }
    }
  }
}

// Functiont to Serial1lize the data packet
void Serial1ize(int16_t *_buffer){
  int16_t checksum = 0;
  _buffer[0] = numberOfPackets;
  memcpy(_buffer+1, &packet, (size_t) sizeof(packet));
  for(int i = 1; i < 35; i++){
    checksum ^= _buffer[i];
  } 
  _buffer[35] = checksum;
}

/*// Functiont to Serial1lize the data packet
int Serial1ize(int16_t *_buffer){
  int16_t checksum = 0;
  _buffer[0] = numberOfPackets;
  memcpy(_buffer+1, &packet, (size_t) sizeof(packet));
  int bufferSize = sizeof(packet)+1;
  for(int i = 1; i < bufferSize; i++){
    checksum ^= _buffer[i];
  } 
  _buffer[bufferSize] = checksum;
  return bufferSize;
}*/

/**
 * Function to read data from all sensors
 * Hand Accelerometer, Gyroscope, Thigh Accelerometer
 */
void readDataFromSensors(){
  // Obtain Gyroscope Reading
  //digitalWrite(gyroFirst, LOW);
  //digitalWrite(accThird, HIGH);
  digitalWrite2f(gyroFirst, LOW);
  digitalWrite2f(accThird, HIGH);
  accelgyro.getRotation(&data.gyroscope[0], &data.gyroscope[1], &data.gyroscope[2]);
  
  // Change to reading of hand accelerometer
  //digitalWrite(gyroFirst, HIGH);
  //digitalWrite(accSec, LOW);
  digitalWrite2f(gyroFirst, HIGH);
  digitalWrite2f(accSec, LOW);
  accelgyro.getAcceleration(&data.accelHand[0], &data.accelHand[1], &data.accelHand[2]);
  
  // Change to reading of Thigh accelerometer
  //digitalWrite(accSec, HIGH);
  //digitalWrite(accThird, LOW);
  digitalWrite2f(accSec, HIGH);
  digitalWrite2f(accThird, LOW);
  accelgyro.getAcceleration(&data.accelThigh[0], &data.accelThigh[1], &data.accelThigh[2]);  
}

/**
 * Task to read data from the power circuit
 * Variables obtained: Voltage and Current
 */
void readDataFromPowerCircuit(){
  // Read Raw values from the INA169 board and voltage divider
  float currentSensorValue = analogRead(CURRENT_SENSOR_PIN);
  float voltageSensorValue = analogRead(VOLTAGE_SENSOR_PIN);
  
  // Remap the ADC value into a voltage number (5V reference)
  currentSensorValue = (currentSensorValue * VOLTAGE_REF) / 1023;
  voltageSensorValue = (voltageSensorValue * VOLTAGE_REF) / 1023;
  
  // Voltage reading is obtained from divider.
  // Multiply reading by 2 to get the true voltage value of the batteries
  data.voltage = voltageSensorValue * 2 *1000;
  
  // Follow the equation given by the INA169 datasheet to
  // determine the current flowing through RS. Assume RL = 10k
  // Is = (Vout x 1k) / (RS x RL)
  data.current = (currentSensorValue / (10 * RS))*1000;
  
  // Power = Voltage * Current
  data.power = (voltageSensorValue * 2 * (currentSensorValue / (10 * RS)))*1000;
}

/**
 * Task to read data at full cycle
 * Task is to be read before sending
 */
void readDataAtFullCycle(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      readDataFromSensors();
      readDataFromPowerCircuit();
      packet[numberOfPackets] = data;
      numberOfPackets++;

      // Give Semaphore
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod/ portTICK_PERIOD_MS);
  }
}

/**
 * Task to read data every half cycle
 * readDataAtHalfCycle
 */
void readDataAtHalfCycle(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  vTaskDelayUntil(&xLastWakeTime, (xPeriod/2)/ portTICK_PERIOD_MS);
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){      
      readDataFromSensors();
      readDataFromPowerCircuit();
      packet[numberOfPackets] = data;
      numberOfPackets++;

      // Give Semaphore
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod/ portTICK_PERIOD_MS);
  }
}

/**
 * Task to send Data to the RasberryPi
 * Steps:
 *  1) Package Data
 *  2) Send Packaged Data to RasberryPi
 */
void sendDataToRaspberryPi(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
 
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){ 
      int trys = 0;
      int reply;

      // Loop once if not received
      while (trys < 2){
        int16_t _buffer[36];
        Serial1ize(_buffer);
        for(int i=0; i < 36; i++){
          Serial1.println(_buffer[i]);
        }
          
        
        /*if(!Serial1.available()){
          vTaskDelay(10/ portTICK_PERIOD_MS);
        }*/

        while(!Serial1.available()){
          
        }
        
        if (Serial1.available()) {
          reply = Serial1.read();
          if (reply == ACK) {
            trys = 10;
          }
        }
        trys++;
      }
      numberOfPackets = 0;
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod/ portTICK_PERIOD_MS);
  }
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000L);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // To speed up analog read
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
  //pinMode(gyroFirst, OUTPUT);
  //pinMode(accSec, OUTPUT);
  //pinMode(accThird, OUTPUT);

  //digitalWrite(gyroFirst, HIGH);
  //digitalWrite(accSec, HIGH);
  //digitalWrite(accThird, HIGH);

  pinMode2f(gyroFirst, OUTPUT);
  pinMode2f(accSec, OUTPUT);
  pinMode2f(accThird, OUTPUT);

  digitalWrite2f(gyroFirst, HIGH);
  digitalWrite2f(accSec, HIGH);
  digitalWrite2f(accThird, HIGH);
  
  Serial1.begin(115200);
    
  handshake();
  accelgyro.initialize();
  initializeDataPacket();
  xSemaphoreGive(semaphore);
  
  // Create Tasks
  xTaskCreate(readDataAtFullCycle, "readDataAtFullCycle", STACK_SIZE, (void *) NULL, 3, NULL);
  xTaskCreate(readDataAtHalfCycle, "readDataAtHalfCycle", STACK_SIZE, (void *) NULL, 2, NULL);
  xTaskCreate(sendDataToRaspberryPi, "sendDataToRaspberryPi", STACK_SIZE, (void *) NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {
  // Digital Input Disable on Analogue Pins
  // When this bit is written logic one, the digital input buffer on the corresponding ADC pin is disabled.
  // The corresponding PIN Register bit will always read as zero when this bit is set. When an
  // analogue signal is applied to the ADC7..0 pin and the digital input from this pin is not needed, this
  // bit should be written logic one to reduce power consumption in the digital input buffer.
   
  #if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // Mega with 2560
  DIDR0 = 0xFF;
  DIDR2 = 0xFF;
  #elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p
  DIDR0 = 0xFF;
   
  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino with 328p
  DIDR0 = 0x3F;
   
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
  DIDR0 = 0xF3;
  DIDR2 = 0x3F;
  #endif
   
  // Analogue Comparator Disable
  // When the ACD bit is written logic one, the power to the Analogue Comparator is switched off.
  // This bit can be set at any time to turn off the Analogue Comparator.
  // This will reduce power consumption in Active and Idle mode.
  // When changing the ACD bit, the Analogue Comparator Interrupt must be disabled by clearing the ACIE bit in ACSR.
  // Otherwise an interrupt can occur when the ACD bit is changed.
  ACSR &= ~_BV(ACIE);
  ACSR |= _BV(ACD);
   
  // There are several macros provided in the header file to actually put
  // the device into sleep mode.
  // SLEEP_MODE_IDLE (0)
  // SLEEP_MODE_ADC (_BV(SM0))
  // SLEEP_MODE_PWR_DOWN (_BV(SM1))
  // SLEEP_MODE_PWR_SAVE (_BV(SM0) | _BV(SM1))
  // SLEEP_MODE_STANDBY (_BV(SM1) | _BV(SM2))
  // SLEEP_MODE_EXT_STANDBY (_BV(SM0) | _BV(SM1) | _BV(SM2))
   
  set_sleep_mode( SLEEP_MODE_IDLE );
   
  portENTER_CRITICAL();
  sleep_enable();
   
  // Only if there is support to disable the brown-out detection.
  #if defined(BODS) && defined(BODSE)
  sleep_bod_disable();
  #endif
   
  portEXIT_CRITICAL();
  sleep_cpu(); // good night.
   
  // Ugh. I've been woken up. Better disable sleep mode.
  sleep_reset(); // sleep_reset is faster than sleep_disable() because it clears all sleep_mode() bits.
}
