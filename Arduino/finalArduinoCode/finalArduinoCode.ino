#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <stdio.h>
#include <stdlib.h>
#include <arduino2.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>

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
  int16_t timeTaken;
} DataPacket;

// Class default I2C address is AD0 low = 0x68
MPU6050 accelgyro(0x68);

// Device ID
int16_t ARM_ID = 0, GYRO_ID = 1, THIGH_ID = 2, POWER_ID = 3;

// Packet code
int16_t ACK = 0, NAK = 1, HELLO = 2, READ = 3, WRITE = 4, DATA_RESP = 5;

// Constants
const int BUFFER_SIZE = 20;
const TickType_t PERIOD = 16;           
const float RS = 0.1;                 // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;            // Reference voltage for analog read
const int VOLTAGE_SENSOR_PIN = A1;
const int CURRENT_SENSOR_PIN = A0;
const GPIO_pin_t GYROSCOPE = DP30;
const GPIO_pin_t LEFT_HAND_ACCELEROMETER = DP29;
const GPIO_pin_t RIGHT_HAND_ACCELEROMETER = DP31;

// Create a buffer
int16_t _buffer[BUFFER_SIZE][20];
int frontOfBuffer = 0;
int backOfBuffer = 0;
int bufferFullFlag = 0;

// Create a Data Packet
DataPacket data;

// Create Sempaphore
SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();

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
        Serial1.write(ACK);
      }
      if (reply == ACK) {
        handshake_flag = 0;
      }
    }
  }
}

/**
 * Function to serialize the data packet
 */// Functiont to Seriallize the data packet
void serialize(int16_t *packet){
  int16_t checksum = 0;
  memcpy(packet, &data, (size_t) sizeof(data));
  for(int i = 0; i < 18; i++)
    checksum ^= packet[i];
  packet[18] = checksum;
}

/**
 * Function to read data from all sensors
 * Left Hand Accelerometer, Gyroscope, Right Hand Accelerometer
 */
void readDataFromSensors(){
  // Obtain Gyroscope Reading
  digitalWrite2f(GYROSCOPE, LOW);
  digitalWrite2f(RIGHT_HAND_ACCELEROMETER, HIGH);
  accelgyro.getRotation(&data.gyroscope[0], &data.gyroscope[1], &data.gyroscope[2]);
  
  // Change to reading of left hand accelerometer
  digitalWrite2f(GYROSCOPE, HIGH);
  digitalWrite2f(LEFT_HAND_ACCELEROMETER, LOW);
  accelgyro.getAcceleration(&data.accelHand[0], &data.accelHand[1], &data.accelHand[2]);
  
  // Change to reading of right hand accelerometer
  digitalWrite2f(LEFT_HAND_ACCELEROMETER, HIGH);
  digitalWrite2f(RIGHT_HAND_ACCELEROMETER, LOW);
  accelgyro.getAcceleration(&data.accelThigh[0], &data.accelThigh[1], &data.accelThigh[2]);  
}

/**
 * Function to read data from the power circuit
 * Variables obtained: Voltage, Current, Power, TimeTaken
 */
void readDataFromPowerCircuit(){
  float currentSensorValue = 0;
  float voltageSensorValue = 0;

  // Read Raw values from the INA169 board and voltage divider
  currentSensorValue = analogRead(CURRENT_SENSOR_PIN);
  voltageSensorValue = analogRead(VOLTAGE_SENSOR_PIN);
  
  // Remap the ADC value into a voltage number (5V reference)
  currentSensorValue = (currentSensorValue * VOLTAGE_REF) / 1023;
  voltageSensorValue = (voltageSensorValue * VOLTAGE_REF) / 1023;
  
  // Voltage reading is obtained from divider.
  // Multiply reading by 2 to get the true voltage value of the batteries
  data.voltage = int16_t((voltageSensorValue * 2)*1000);
  
  // Follow the equation given by the INA169 datasheet to
  // determine the current flowing through RS. Assume RL = 10k
  // Is = (Vout x 1k) / (RS x RL)
  data.current = int16_t((currentSensorValue / (10 * RS))*1000);
  
  // Power = Voltage * Current
  data.power = int16_t((voltageSensorValue * 2 * (currentSensorValue / (10 * RS)))*1000);

  // Calculate the time taken in miliseconds taken for the cycle
  data.timeTaken = PERIOD/2;
}

/**
 * Function to read and package the data
 */
void readAndPackageData(){
  // Read the data if the buffer is not full
  if(bufferFullFlag == 0){
    readDataFromSensors();
    readDataFromPowerCircuit();
    serialize(_buffer[backOfBuffer]);
    backOfBuffer = (backOfBuffer + 1)%BUFFER_SIZE;
  }

  // Warns the system if the buffer is full
  if((backOfBuffer + 1)%BUFFER_SIZE == frontOfBuffer){
    bufferFullFlag = 1;
  } else {
    bufferFullFlag = 0;
  }
}

/**
 * Function to put the MCU to sleep
 * Does not disable any interrupts and block processing
 */
void sleep (){
  // Digital Input Disable on Analogue Pins
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
  ACSR &= ~_BV(ACIE);
  ACSR |= _BV(ACD);
  
  set_sleep_mode( SLEEP_MODE_IDLE );
  portENTER_CRITICAL();
  sleep_enable();
  
  #if defined(BODS) && defined(BODSE)
  sleep_bod_disable();
  #endif
  
  portEXIT_CRITICAL();
  sleep_cpu();
  sleep_reset();
}

/**
 * Task to Read and Package data
 * Task is executed at Half Cycle
 */
void readDataAtHalfCycle(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  vTaskDelayUntil(&xLastWakeTime, (PERIOD/2)/ portTICK_PERIOD_MS);
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      readAndPackageData();
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, PERIOD/ portTICK_PERIOD_MS);
  }
}

/**
 * Task to Read and Package data
 * Task is execute at Full Cycle
 * Task is to be read before sending
 */
void readDataAtFullCycle(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      readAndPackageData();
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, PERIOD/ portTICK_PERIOD_MS);
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
      int reply;
      int numberOfPacketsInBuffer;

      // Find the number of elements in the queue
      if(frontOfBuffer < backOfBuffer){
        numberOfPacketsInBuffer = backOfBuffer - frontOfBuffer;
      } else {
        numberOfPacketsInBuffer = backOfBuffer + BUFFER_SIZE - frontOfBuffer;
      }

      // Send the data to the RaspberryPi
      for(int i = 0; i < numberOfPacketsInBuffer; i++){
        Serial.print("[");
        for(int j = 0; j < 19; j++){
            byte buf[2];
            buf[0] = _buffer[(frontOfBuffer+i)%BUFFER_SIZE][j] & 255;
            buf[1] = (_buffer[(frontOfBuffer+i)%BUFFER_SIZE][j] >> 8) & 255;
            Serial1.write(buf, sizeof(buf));
            Serial.print(_buffer[(frontOfBuffer+i)%BUFFER_SIZE][j]);
            Serial.print(", ");
        }
        Serial.println("]");
      }

      // Waiting for Acknowledgement
      while(!Serial1.available()){
        
      }

      // Receive the Acknowledgement for the first packet
      if (Serial1.available()) {
        reply = Serial1.read();
        if (reply == ACK) {
          frontOfBuffer = (frontOfBuffer+1)%10;
        }
      }

      // Waiting for Acknowledgement
      while(!Serial1.available()){
        
      }

      // Receive the Acknowledge for the second packet
      if (Serial1.available()) {
        reply = Serial1.read();
        if (reply == ACK) {
          frontOfBuffer = (frontOfBuffer+1)%10;
        }
      }
      xSemaphoreGive(semaphore);
    }
    vTaskDelayUntil(&xLastWakeTime, PERIOD/ portTICK_PERIOD_MS);
  }
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial1.begin(115200);
  Serial.begin(115200);

  // Initialise the gyroscope
  accelgyro.initialize();
  
  // Change the Range of GyroScoep to 500
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

  // Change the range of the Accelerometer to 4G
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

  // Set a Low Pass filter with a WECT 4.9ms
  accelgyro.setDLPFMode(3);

  // Test Connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Initialise the Accelerometer and Gyroscope pins
  pinMode2f(GYROSCOPE, OUTPUT);
  pinMode2f(LEFT_HAND_ACCELEROMETER, OUTPUT);
  pinMode2f(RIGHT_HAND_ACCELEROMETER, OUTPUT);

  // Set All pins to high
  digitalWrite2f(GYROSCOPE, HIGH);
  digitalWrite2f(LEFT_HAND_ACCELEROMETER, HIGH);
  digitalWrite2f(RIGHT_HAND_ACCELEROMETER, HIGH);

  handshake();
  initializeDataPacket();
  xSemaphoreGive(semaphore);
  
  // Create Tasks
  xTaskCreate(readDataAtHalfCycle, "readDataAtHalfCycle", STACK_SIZE, (void *) NULL, 3, NULL);
  xTaskCreate(readDataAtFullCycle, "readDataAtFullCycle", STACK_SIZE, (void *) NULL, 2, NULL);
  xTaskCreate(sendDataToRaspberryPi, "sendDataToRaspberryPi", STACK_SIZE, (void *) NULL, 1, NULL);
  vTaskStartScheduler();
}

/**
 * Loop method to reduce power
 */
void loop() {
  sleep();
}
