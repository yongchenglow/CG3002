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

SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();

void setup() {
  // Report sate 9600
  Serial.begin(115200);
}

void loop() {
  // Task to read data from Sensors
  xTaskCreate(readDataFromSensors, "readDataFromSensors", STACK_SIZE, (void * ) NULL, 2, NULL);

  // Task to send data to RaspberryPi
  xTaskCreate(sendDataToRaspberryPi, "sendDataToRaspberryPi", STACK_SIZE, (void * ) NULL, 1, NULL);

  // Start Scheduler
  vTaskStartScheduler();
}

void readDataFromSensors(void *p){
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 500;

  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      xSemaphoreGive(semaphore);

      /**
       * 1) Read data from sensors
       * 2) Package the data
       * 3) Store the data
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
       * 1) Send data to the RasberryPI
       */
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}
