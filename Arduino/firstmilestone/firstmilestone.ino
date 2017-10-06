#include <Arduino.h>
#include <avr/io.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdio.h>
#include <stdlib.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define STACK_SIZE    200

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
DataPacket data;

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
int gyroFirst = 30;
int accSec = 29;
int accThird = 31;

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
    if (Serial.available()) {
      reply = Serial.read();
      if (reply == HELLO) {
        Serial.println(ACK);
      }
      if (reply == ACK) {
        handshake_flag = 0;
      }
    }
  }
}

// Functiont to Seriallize the data packet
void Serialize(int16_t *_buffer){
  int16_t checksum = 0;
  memcpy(_buffer, &data, (size_t) sizeof(data));
  for(int i = 0; i <= 16; i++){
    checksum ^= _buffer[i];
  } 
  _buffer[17] = checksum;
}

/**
 * Task to read data from all sensors
 * Hand Accelerometer, Gyroscope, Thigh Accelerometer
 */
void readDataFromSensors(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 16;
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      // Obtain Gyroscope Reading
      digitalWrite(gyroFirst, LOW);
      digitalWrite(accSec, HIGH);
      digitalWrite(accThird, HIGH);
      accelgyro.getRotation(&data.gyroscope[0], &data.gyroscope[1], &data.gyroscope[2]);
      
      // Change to reading of hand accelerometer
      digitalWrite(gyroFirst, HIGH);
      digitalWrite(accSec, LOW);
      digitalWrite(accThird, HIGH);
      accelgyro.getAcceleration(&data.accelHand[0], &data.accelHand[1], &data.accelHand[2]);
    
      // Change to reading of Thigh accelerometer
      digitalWrite(gyroFirst, HIGH);
      digitalWrite(accSec, HIGH);
      digitalWrite(accThird, LOW);
      accelgyro.getAcceleration(&data.accelThigh[0], &data.accelThigh[1], &data.accelThigh[2]);
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod/ portTICK_PERIOD_MS);
  }
}

/**
 * Task to read data from the power circuit
 * Variables obtained: Voltage and Current
 */
void getPower(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 16;
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){      
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
  const TickType_t xPeriod = 16;
  
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){ 
      int trys = 0;
      int reply;

      // Loop once if not received
      while (trys < 2){
        int16_t _buffer[18];
        Serialize(_buffer);
        for(int i=0; i <= 17; i++)
          Serial.println(_buffer[i]);

        /*if(!Serial.available()){
          vTaskDelay(10/ portTICK_PERIOD_MS);
        }*/
        
        if (Serial.available()) {
          reply = Serial.read();
          if (reply == ACK) {
            trys = 10;
          }
        }
        trys++;
      }
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod/ portTICK_PERIOD_MS);
  }
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
    
  pinMode(gyroFirst, OUTPUT);
  pinMode(accSec, OUTPUT);
  pinMode(accThird, OUTPUT);
  
  Serial.begin(115200);
    
  handshake();
  accelgyro.initialize();
  initializeDataPacket();
  xSemaphoreGive(semaphore);
  
  // Create Tasks
  xTaskCreate(readDataFromSensors, "readDataFromSensors", STACK_SIZE, (void *) NULL, 3, NULL);
  xTaskCreate(getPower, "getPower", STACK_SIZE, (void *) NULL, 2, NULL);
  xTaskCreate(sendDataToRaspberryPi, "sendDataToRaspberryPi", STACK_SIZE, (void *) NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {
  
}
