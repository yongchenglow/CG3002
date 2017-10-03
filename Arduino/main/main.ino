#include <Arduino.h>
#include <avr/io.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdio.h>
#include <stdlib.h>

#define STACK_SIZE    200

/**
 * DataPacket Structure
 */
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
} DataPacket;

/* Packet Declaration */
DataPacket data;

/* Device ID */
int16_t ARM_ID = 0, GYRO_ID = 1, THIGH_ID = 2, POWER_ID = 3;

/* Packet code */
int16_t ACK = 0, NAK = 1, HELLO = 2, READ = 3, WRITE = 4, DATA_RESP = 5;

SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();

/**
 * Initialise the data Packet with some values
 */
void initializeDataPacket(){
  data.type = DATA_RESP;
  data.armID = ARM_ID;
  data.gyroID = GYRO_ID;
  data.thighID = THIGH_ID;
  data.powerID = POWER_ID;
}

/**
 * Handshake with RaspberryPI
 */
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

/**
 * Method to Serial1ize the data packet
 */
void Serial1ize(int16_t *_buffer){
  int16_t checksum = 0;
  _buffer[0] = 17;
  memcpy(_buffer+1, &data, (size_t) sizeof(data));
  for(int i = 1; i <= 16; i++){
    checksum ^= _buffer[i];
  } 
  _buffer[17] = checksum;
}

/**
 * Method to read data from all sensors
 * Hand Accelerometer, Gyroscope, Thigh Accelerometer
 */
void readDataFromSensors(void *p){
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 100;

  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      
      /* Accelerometer Hand Values */
      data.accelHand[0] = rand()-rand();
      data.accelHand[1] = rand()-rand();
      data.accelHand[2] = rand()-rand();

      /* Gyroscope Values */
      data.gyroscope[0] = rand()-rand();
      data.gyroscope[1] = rand()-rand();
      data.gyroscope[2] = rand()-rand();

      /* Accelerometer Thigh Values */
      data.accelThigh[0] = rand()-rand();
      data.accelThigh[1] = rand()-rand();
      data.accelThigh[2] = rand()-rand();

      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * Method to read data from the power circuit
 * Variables obtained: Voltage and Current
 */
void getPower(void *p){
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 100;

  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      /* Power Circuit Values */
      data.voltage = rand()-rand();
      data.current = rand()-rand();
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * Method to send Data to the RasberryPi
 * Steps:
 *  1) Package Data
 *  2) Send Packaged Data to RasberryPi
 */
void sendDataToRaspberryPi(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 100;
  
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){ 
      int trys = 0;
      int reply;
      // Loop once if not received
      while (trys < 3){
        int16_t _buffer[18];
        Serial1ize(_buffer);
        for(int i=0; i <= 17; i++)
          Serial1.println(_buffer[i]);
        
        if (Serial1.available()) {
            reply = Serial1.read();
          if (reply == ACK) {
            trys = 10;
            xSemaphoreGive(semaphore);
          } else {
            trys++;
          }
        } else {
          trys++;
        }
      }
      if(trys == 3)
        xSemaphoreGive(semaphore);
      
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void setup() {
  Serial1.begin(9600);
  handshake();
  initializeDataPacket();
  xSemaphoreGive(semaphore);
  
  /* Task Creation */
  xTaskCreate(readDataFromSensors, "readDataFromSensors", STACK_SIZE, (void *) NULL, 3, NULL);
  xTaskCreate(getPower, "getPower", STACK_SIZE, (void *) NULL, 2, NULL);
  xTaskCreate(sendDataToRaspberryPi, "sendDataToRaspberryPi", STACK_SIZE, (void *) NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {
  
}
