#include <Arduino.h>

#define CURRENT_SENSOR_PIN A0
#define VOLTAGE_PIN A1
#define SHUNT_RESISTOR 0.1
#define SAMPLE_SIZE 200
#define SAMPLING_DELAY 2

float voltage_sum;
float current_sum;

unsigned long prev_time;
unsigned long curr_time;
float time;

float energy_consumed;

void setup(){
	Serial.begin(38400);

	energy_consumed = 0;
	
	prev_time = 0;
	
	voltage_sum = 0;
	current_sum = 0;
}

void loop() {
	
	unsigned long voltage_sample_sum = 0;
	unsigned long current_sample_sum = 0;
	
	//Summing N samples; 1 sample per 2ms
	for(int i=0; i<SAMPLE_SIZE; i++){
		voltage_sample_sum = voltage_sample_sum + analogRead(VOLTAGE_PIN);
		current_sample_sum = current_sample_sum + analogRead(CURRENT_SENSOR_PIN);
		delay(SAMPLING_DELAY);
	}
	
	//logging the timestamp and duration at the end of the sampling
	curr_time = millis();
	time = (curr_time - prev_time)/1000.0/60.0/60.0; //length of time for the N sampled data, in terms of hours
	prev_time = curr_time;
	
	//divide sample sum by N to find average
	//remap digital values to analog
	float voltage = remap(voltage_sample_sum/SAMPLE_SIZE);
	float current = remap(current_sample_sum/SAMPLE_SIZE);
	
	//Multiply reading by 2 to get the true voltage value of the batteries
	//Since voltage reading is obtained from divider that halfs the volatage.
	voltage = voltage *2.0; //----------------------------------------> send this over serial
	
	//Based on datasheet of current sensor
	// Is = (Vout x 1k) / (RS x RL)
	current = current/(10 * SHUNT_RESISTOR); //---------------------> send this over serial
	
	float power = voltage * current;
	energy_consumed = energy_consumed + (power * time);
	
	
	
	
	
	
	
	// Printing to serial monitor
	Serial.print(current, 4);
	Serial.println(" A");
	
	Serial.print(voltage, 4);
	Serial.println(" V");
	
	Serial.print(power, 4);
	Serial.println(" W");
	
	Serial.print(energy_consumed, 6);
	Serial.println(" Wh\n");
}

// remap values from analogRead from 0-1023 to 0v-5v
float remap(float raw_value){
	return raw_value * 5 / 1023.0;
}