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

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

#define STACK_SIZE    200

// Fast AnalogRead
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
  int16_t timeTaken;
} DataPacket;

// Class default I2C address is AD0 low = 0x68
MPU6050 accelgyro(0x68);

const int bufferSize = 20;
int16_t _buffer[bufferSize][20];
int frontOfBuffer = 0;
int backOfBuffer = 0;
int bufferFullFlag = 0;
int start = 0;
int16_t idleTime = 0;
unsigned long cumilativeIdleVoltage = 0;
unsigned long cumilativeIdleCurrent = 0;
DataPacket data;

// Device ID
int16_t ARM_ID = 0, GYRO_ID = 1, THIGH_ID = 2, POWER_ID = 3;

// Packet code
int16_t ACK = 0, NAK = 1, HELLO = 2, READ = 3, WRITE = 4, DATA_RESP = 5, STOP = 6;

// Create Sempaphore
SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();

// Constants
const int CURRENT_SENSOR_PIN = A0;    // Input pin for measuring Volt
const float RS = 0.1;                 // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;            // Reference voltage for analog read
const int VOLTAGE_SENSOR_PIN = A1;
const GPIO_pin_t gyroFirst = DP30;
const GPIO_pin_t accSec = DP29;
const GPIO_pin_t accThird = DP31;
const TickType_t xPeriod = 16;
float currentSensorValue = 0;
float voltageSensorValue = 0;

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
        start = 1;
      }
    }
  }
}

// Functiont to Seriallize the data packet
void Serialize(int16_t *packet){
  int16_t checksum = 0;
  packet[0] = backOfBuffer;
  memcpy(packet+1, &data, (size_t) sizeof(data));
  for(int i = 1; i < 19; i++)
    checksum ^= packet[i];
    
  packet[19] = checksum;
}

/**
 * Function to read data from all sensors
 * Hand Accelerometer, Gyroscope, Thigh Accelerometer
 */
void readDataFromSensors(){
  // Obtain Gyroscope Reading
  digitalWrite2f(gyroFirst, LOW);
  digitalWrite2f(accThird, HIGH);
  accelgyro.getRotation(&data.gyroscope[0], &data.gyroscope[1], &data.gyroscope[2]);
  
  // Change to reading of hand accelerometer
  digitalWrite2f(gyroFirst, HIGH);
  digitalWrite2f(accSec, LOW);
  accelgyro.getAcceleration(&data.accelHand[0], &data.accelHand[1], &data.accelHand[2]);
  
  // Change to reading of Thigh accelerometer
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
  data.timeTaken = xPeriod/2;

  if(idleTime != 0){
    cumilativeIdleVoltage += data.voltage;
    cumilativeIdleCurrent += data.current;
    data.timeTaken = data.timeTaken + idleTime;
    data.voltage = cumilativeIdleVoltage/idleTime/16.0;
    data.current = cumilativeIdleCurrent/idleTime/16.0;
    data.power = data.voltage * data.current;
    idleTime = 0;
    cumilativeIdleVoltage = 0;
    cumilativeIdleCurrent = 0;
  }
}

/**
 * Function to read and Package the data
 */
void readAndPackageData(){
  // Read the data if the buffer is not full
  if(bufferFullFlag == 0){
    readDataFromSensors();
    readDataFromPowerCircuit();
    Serialize(_buffer[backOfBuffer]);
    backOfBuffer = (backOfBuffer + 1)%bufferSize;
  }

  // Warns the system if the buffer is full
  if((backOfBuffer + 1)%bufferSize == frontOfBuffer){
    bufferFullFlag = 1;
  } else {
    bufferFullFlag = 0;
  }
}

/**
 * Task to read data at full cycle
 * Task is to be read before sending
 */
void readDataAtFullCycle(void *p){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;){
    if(xSemaphoreTake(semaphore, (TickType_t) portMAX_DELAY) == pdTRUE){
      if(start == 1){
        readAndPackageData();
      }
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
      if(start == 1){      
        readAndPackageData();
      }
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
      if(start == 1){
        int trys = 0;
        int reply;
        int numberOfPacketsInBuffer;
  
        if(frontOfBuffer < backOfBuffer){
          numberOfPacketsInBuffer = backOfBuffer - frontOfBuffer;
        } else {
          numberOfPacketsInBuffer = backOfBuffer + bufferSize - frontOfBuffer;
        }
  
        for(int i = 0; i < numberOfPacketsInBuffer; i++){
          Serial.print("[");
          for(int j = 0; j < 20; j++){
              byte buf[2];
              buf[0] = _buffer[(frontOfBuffer+i)%bufferSize][j] & 255;
              buf[1] = (_buffer[(frontOfBuffer+i)%bufferSize][j] >> 8) & 255;
              Serial1.write(buf, sizeof(buf));
              Serial.print(_buffer[(frontOfBuffer+i)%bufferSize][j]);
              Serial.print(", ");
          }
          Serial.println("]");
        }
  
        while(!Serial1.available()){
          
        }
        
        if (Serial1.available()) {
          reply = Serial1.read();
          if (reply == ACK) {
            frontOfBuffer = (frontOfBuffer+1)%10;
          } else if (reply == STOP){
            start = 0;
          }
           memset(_buffer, 0, bufferSize);
        }
  
        while(!Serial1.available()){
          
        }
        
        if (Serial1.available()) {
          reply = Serial1.read();
          if (reply == ACK) {
            frontOfBuffer = (frontOfBuffer+1)%10;
          } else if (reply == STOP){
            start = 0;
          }
          memset(_buffer, 0, bufferSize);
        }
      } else {
        int reply;
        if (Serial1.available()){
          reply = Serial1.read();
          if(reply == ACK){
            start = 1;
          }
        }

        readDataFromPowerCircuit();
        idleTime += 16;
        cumilativeIdleVoltage += data.voltage;
        cumilativeIdleCurrent += data.current;
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
  
  Serial1.begin(115200);
  Serial.begin(115200);
    
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(MPU6050_ACCEL_FS_4);
  accelgyro.setFullScaleAccelRange(MPU6050_GYRO_FS_500);
  accelgyro.setDLPFMode(3);

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode2f(gyroFirst, OUTPUT);
  pinMode2f(accSec, OUTPUT);
  pinMode2f(accThird, OUTPUT);

  digitalWrite2f(gyroFirst, HIGH);
  digitalWrite2f(accSec, HIGH);
  digitalWrite2f(accThird, HIGH);

  handshake();
  /*
   * MPU6050_ACCEL_FS_2          0x00
   * MPU6050_ACCEL_FS_4          0x01
   * MPU6050_ACCEL_FS_8          0x02
   * MPU6050_ACCEL_FS_16         0x03
   * accelgyro.setFullScaleGyroRange(uint8_t range)
   * 
   * MPU6050_GYRO_FS_250         0x00
   * MPU6050_GYRO_FS_500         0x01
   * MPU6050_GYRO_FS_1000        0x02
   * MPU6050_GYRO_FS_2000        0x03
   * accelgyro.setFullScaleAccelRange(uint8_t range)
   * 
   *          |   ACCELEROMETER    |           GYROSCOPE
   * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
   * ---------+-----------+--------+-----------+--------+-------------
   * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
   * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
   * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
   * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
   * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
   * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
   * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
   * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
   * accelgyro.setDLPFMode(uint8_t mode)
   */
  initializeDataPacket();
  xSemaphoreGive(semaphore);
  
  // Create Tasks
  xTaskCreate(readDataAtFullCycle, "readDataAtFullCycle", STACK_SIZE, (void *) NULL, 3, NULL);
  xTaskCreate(readDataAtHalfCycle, "readDataAtHalfCycle", STACK_SIZE, (void *) NULL, 2, NULL);
  xTaskCreate(sendDataToRaspberryPi, "sendDataToRaspberryPi", STACK_SIZE, (void *) NULL, 1, NULL);
  vTaskStartScheduler();
}

/**
 * Loop method to reduce power
 */
void loop() {
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
