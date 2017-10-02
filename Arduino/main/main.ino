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
 * Create a data structure to store the data?
 */

/* Device ID */
int ARM_ID = 0, GYRO_ID = 1, THIGH_ID = 2, POWER_ID = 3;

/* Packet code */
int ACK = 0, NAK = 1, HELLO = 2, READ = 3, WRITE = 4, DATA_RESP = 5;

/* Hand Accelerometer Value */
int16_t accelx_hand_value = 0;
int16_t accely_hand_value = 0;
int16_t accelz_hand_value = 0;

/* Gyroscope Value */
int16_t gyrox_value = 0;
int16_t gyroy_value = 0;
int16_t gyroz_value = 0;

/* Thigh Accelerometer Value */
int16_t accelx_thigh_value = 0;
int16_t accely_thigh_value = 0;
int16_t accelz_thigh_value = 0;

/* Power Circuit Values */
int16_t voltage = 0;
int16_t current = 0;

/** 
 *  Byte Array for Sending Data
 *  int16_t to take in a value between -32768 to 32767
 */
int16_t package[20];

SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();

/**
 * Handshake with RaspberryPI
 */
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

/**
 * Method to calculate checkSum
 * checkSum = b1 XOR b2 ... XOR bn
 */
int16_t getCheckSum(int16_t *package) {
  int i;
  int upperLimit = package[0];
  int16_t value = 0;
  int16_t checkSum = 0;  
 
  for (i = 1; i < upperLimit; i++) {
    value = package[i];
    checkSum ^= value;
  }
  return checkSum;
}

/**
 * Method to read data from all sensors
 * Hand Accelerometer, Gyroscope, Thigh Accelerometer
 */
void readDataFromSensors(void *p){
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 500;

  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
       
      /* Accelerometer Hand Values */
      accelx_hand_value = rand()-rand();
      accely_hand_value = rand()-rand();
      accelz_hand_value = rand()-rand();

      /* Gyroscope Values */
      gyrox_value = rand()-rand();
      gyroy_value = rand()-rand();
      gyroz_value = rand()-rand();

      /* Accelerometer Thigh Values */
      accelx_thigh_value = rand()-rand();
      accely_thigh_value = rand()-rand();
      accelz_thigh_value = rand()-rand();

      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/**
 * Method to read data from the power circuit
 * Variables obtained: Voltage and Current
 */
void getVoltage(void *p){
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = 500;

  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      /* Power Circuit Values */
      voltage = rand()-rand();
      current = rand()-rand();
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
  const TickType_t xPeriod = 500;
  
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){ 

      /* Standard Data packaging*/
      package[0] = 17;
      package[1] = DATA_RESP;
      package[2] = ARM_ID;
      package[3] = accelx_hand_value;
      package[4] = accely_hand_value;
      package[5] = accelz_hand_value;
      package[6] = GYRO_ID;
      package[7] = gyrox_value;
      package[8] = gyroy_value;
      package[9] = gyroz_value;
      package[10] = THIGH_ID;
      package[11] = accelx_thigh_value;
      package[12] = accely_thigh_value;
      package[13] = accelz_thigh_value;
      package[14] = POWER_ID;
      package[15] = voltage;
      package[16] = current;
      package[17] = getCheckSum(package);

      /* Send Packaged data */
      for( int i = 0; i <= package[0]; i++){
        Serial.println(package[i]);
      }
      
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void setup() {
  Serial.begin(9600);
  handshake();
  xSemaphoreGive(semaphore);
  
  /* Task Creation */
  xTaskCreate(readDataFromSensors, "readDataFromSensors", STACK_SIZE, (void *) NULL, 3, NULL);
  xTaskCreate(getVoltage, "getVoltage", STACK_SIZE, (void *) NULL, 2, NULL);
  xTaskCreate(sendDataToRaspberryPi, "sendDataToRaspberryPi", STACK_SIZE, (void *) NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {
  
}
