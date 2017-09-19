#include <Arduino.h>
#include <avr/io.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <CRCGenerator.h>

#define STACK_SIZE    200

/**
 * Should we create a data structure to store the data?
 */

 /* Device ID */
int ARM_ID = 0, GYRO_ID = 1, THIGH_ID = 2;

/* Packet code */
int ACK = 0, NAK = 1, HELLO = 2, READ = 3, WRITE = 4, DATA_RESP = 5;

/* Sensor values */
float accelx_hand_value = 0;
float accely_hand_value = 0;
float accelz_hand_value = 0;

float gyrox_value = 0;
float gyroy_value = 0;
float gyroz_value = 0;

float accelx_thigh_value = 0;
float accely_thigh_value = 0;
float accelz_thigh_value = 0;

SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();

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

void readDataFromSensors(void *p){
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 500;

  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      xSemaphoreGive(semaphore);

      /**
       * For hardware:
       * 1) Read data from sensors
       * 2) Package the data
       * 3) Store the data
       */
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void getVoltage(void *p){
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 500;

  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      xSemaphoreGive(semaphore);

      /**
       * For hardware:
       * 1) Read voltage
       * 2) Store the data
       */
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void sendDataToRaspberryPi(void *p){
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 500;
  
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void setup() {
  Serial.begin(9600);
  handshake();
}

void loop() {
  xTaskCreate(readDataFromSensors, "readDataFromSensors", STACK_SIZE, (void *) NULL, 3, NULL);
  xTaskCreate(getVoltage, "getVoltage", STACK_SIZE, (void *) NULL, 2, NULL);
  xTaskCreate(sendDataToRaspberryPi, "sendDataToRaspberryPi", STACK_SIZE, (void *) NULL, 1, NULL);
  vTaskStartScheduler();
}
