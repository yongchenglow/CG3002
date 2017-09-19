#include <Arduino.h>
#include <avr/io.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#define STACK_SIZE    200

/**
 * Should we create a data structure to store the data?
 */

int accelx_hand_value = 0;
int accely_hand_value = 0;
int accelz_hand_value = 0;

int gyrox_value = 0;
int gyroy_value = 0;
int gyroz_value = 0;

int accelx_thigh_value = 0;
int accely_thigh_value = 0;
int accelz_thigh_value = 0;

int ACK = 0;
int NAK = 1;
int HELLO = 2;
int reply;
int handshake_flag;

SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();

void handshake() {
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
      /**
       * For firmware:
       * 1) Package all data into a single dtring
       * 2) Send data to the RasberryPI
       */
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void setup() {
  // Report sate 9600
  Serial.begin(9600);
  handshake_flag = 1;
}

void loop() {
  handshake();
  xTaskCreate(readDataFromSensors, "readDataFromSensors", STACK_SIZE, (void * ) NULL, 3, NULL);
  xTaskCreate(getVoltage, "getVoltage", STACK_SIZE, (void *) NULL, 2, NULL);
  xTaskCreate(sendDataToRaspberryPi, "sendDataToRaspberryPi", STACK_SIZE, (void * ) NULL, 1, NULL);
  vTaskStartScheduler();
}
