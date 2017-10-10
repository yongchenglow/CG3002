/*
 11-14-2013
 SparkFun Electronics 2013
 Shawn Hymel

 This code is public domain but you buy me a beer if you use this 
 and we meet someday (Beerware license).

 Based on: https://learn.sparkfun.com/tutorials/ina169-breakout-board-hookup-guide
  
 Description:

 This sketch shows how to use the SparkFun INA169 Breakout
 Board. As current passes through the shunt resistor (Rs), a
 voltage is generated at the Vout pin. Use an analog read and
 some math to determine the current. The current value is
 displayed through the Serial Monitor.

 Hardware connections:

 Uno Pin    INA169 Board    Function

 +5V        VCC             Power supply
 GND        GND             Ground
 A0         VOUT            Analog voltage measurement

 VIN+ and VIN- need to be connected inline with the positive
 DC power rail of a load (e.g. an Arduino, an LED, etc.).

 */

// Constants
const int CURRENT_SENSOR_PIN = A0;    // Input pin for measuring Vout
const float RS = 0.1;                 // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;            // Reference voltage for analog read
const int VOLTAGE_SENSOR_PIN = A1;

// Global Variables
float sensorValue;   // Variable to store value from analog read
float current;       // Calculated current value
float voltage;
float power;

void setup() {

  // Initialize serial monitor
  Serial.begin(9600);

}

void loop() {

  // Read Raw values from the INA169 board and voltage divider
  sensorValue = analogRead(CURRENT_SENSOR_PIN);
  voltage = analogRead(VOLTAGE_SENSOR_PIN);

  // Remap the ADC value into a voltage number (5V reference)
  sensorValue = (sensorValue * VOLTAGE_REF) / 1023;
  voltage = (voltage * VOLTAGE_REF) / 1023;
  
  //Voltage reading is obtained from divider.
  //Multiply reading by 2 to get the true voltage value of the batteries
  voltage = voltage * 2;

  // Follow the equation given by the INA169 datasheet to
  // determine the current flowing through RS. Assume RL = 10k
  // Is = (Vout x 1k) / (RS x RL)
  current = sensorValue / (10 * RS);

  //Power = Voltage * Current
  power = voltage * current;

  // Output value (in amps) to the serial monitor to 3 decimal
  // places
  Serial.print(current, 3);
  Serial.println(" A");
  
  Serial.print(voltage, 3);
  Serial.println(" V");
  
  Serial.print(power, 3);
  Serial.println(" W");
  
  Serial.println();

  // Delay program for a few milliseconds
  delay(500);

}

