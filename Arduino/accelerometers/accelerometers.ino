#include <Wire.h>
const int MPU_addr = 0x69; //I2C address of MPU-6050 when AD0 is low
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int accFirst = 30;
int accSec = 31;
int accThird = 29;

void setup() {
  pinMode(accFirst, OUTPUT);
  pinMode(accSec, OUTPUT);
  pinMode(accThird, OUTPUT);
  digitalWrite(accFirst, HIGH); // Address set to 0x69 if AD0 is set to HIGH
  digitalWrite(accSec, LOW); // Address set to 0x68 if AD0 is set to LOW
  digitalWrite(accThird, LOW);
   
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); //PWR_MGMT_1 register
  Wire.write(0); // set to zero to wake up the MPU-6050
  Wire.endTransmission(true);
  Serial.begin(9600);
  Serial.print("Starting reading\n");
}

void loop() {
  Serial.print("First Accelerometer\n");
  readAccele();
  
  // Change to reading of second accelerometer
  digitalWrite(accFirst, LOW);
  digitalWrite(accSec, HIGH);
  delay(1000);
  
  Serial.print("Second Accelerometer\n");
  readAccele();

  // Change to reading of third accelerometer
  digitalWrite(accSec, LOW);
  digitalWrite(accThird, HIGH);
  delay(1000);

  Serial.print("Third Accelerometer\n");
  readAccele();

  // Change to reading of first accelerometer
  digitalWrite(accThird, LOW);
  digitalWrite(accFirst, HIGH);
  delay(5000);
}

void readAccele() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); //ACCEL_AXOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
}

