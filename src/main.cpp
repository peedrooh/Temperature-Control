#include <Arduino.h>
#include <Adafruit_MLX90614.h>
#include <SPI.h>

#include "PID/PID.h"


/* ############################################################################ */
/* ######################## HALL EFFECT SENSOR SECTION ######################## */
/* ############################################################################ */
#define ACS712_PIN 14 // Arduino Nano analog 0
unsigned long ACS712_SENSITIVITY = 0.185; // (mV per Ampere) This is a constante related to the sensor model, 5A in this case

bool hall_effect_begin() {
  pinMode(ACS712_PIN, INPUT);
  return true;
}

double measure_current_amps() {
  // Read the analog input
  uint16_t sensorValue = analogRead(ACS712_PIN);

  // Convert the analog value to voltage
  // Every ACS712 has a random voltage offset in it's out 
  // pin. In our case, this random number is 2.05V.
  double voltage = (sensorValue * (5.0 / 1023.0)) - 2.05;

  // Calculate the current in amperes
  double current_amps = voltage / ACS712_SENSITIVITY;

  return current_amps;
}


/* ############################################################################ */
/* ######################## TEMPERATURE SENSOR SECTION ######################## */
/* ############################################################################ */
#define MLX90614_I2C_ADDRESS 0x5A
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

bool temp_sensor_begin() {
  if(!mlx.begin(MLX90614_I2C_ADDRESS)) {
    return false;
  }

  return true;
}

double read_temp(bool is_degrees_celcius = true) {
  if(is_degrees_celcius)
    return mlx.readObjectTempC() + 2;

  return mlx.readObjectTempF() + 4;
}


/* #################################################################### */
/* ######################## HEAT PLATE SECTION ######################## */
/* #################################################################### */
#define HEAT_PLATE_PIN 12

bool heat_plate_begin() {
  pinMode(HEAT_PLATE_PIN, OUTPUT);
  return true;
}

void heat_plate_turn_on(bool is_full_cicle = true, uint8_t duty_percentage = 100) {
  if(duty_percentage == 0){
    digitalWrite(HEAT_PLATE_PIN, LOW);
    return;
  }
  int full_cicle_period_microseconds = 16383;
  double current = measure_current_amps();
  if (is_full_cicle) {
    // Wait cicle start
    while(current <= 0.05 && current >= -0.05) {
      current = measure_current_amps();
    }

    uint8_t full_cicles_on = duty_percentage / 10;
    for (int i = 0; i < full_cicles_on; i++) {
      digitalWrite(HEAT_PLATE_PIN, HIGH);
      delayMicroseconds(full_cicle_period_microseconds);
    }

    uint8_t full_cicles_off = 10 - full_cicles_on;
    for (int i = 0; i < full_cicles_off; i++) {
      digitalWrite(HEAT_PLATE_PIN, LOW);
      delayMicroseconds(full_cicle_period_microseconds);
    }
  } else { // dimmerize
    // Wait cicle start
    while(current <= 0.05 && current >= -0.05) {
      current = measure_current_amps();
    }
    
    // Turn the heat plate on for 10 ac main periods
    for (int i = 0; i < 10; i++) {
      // It will turn the heat place for x% of a single cicle on and
      // 100%  - x% of a cicle off.
      digitalWrite(HEAT_PLATE_PIN, HIGH);
      delayMicroseconds(full_cicle_period_microseconds * (duty_percentage / 100.0));
      digitalWrite(HEAT_PLATE_PIN, LOW);
      delayMicroseconds((full_cicle_period_microseconds * (1 - (duty_percentage / 100.0))));
    }
  }
}


/* ####################################################################### */
/* ######################## PIDController SECTION ######################## */
/* ####################################################################### */

PIDController pid = PIDController(0.78, 0.01, 0.4, 100, 0, 100);

void setup() {

  Serial.begin(9600);
  while(!Serial);

  if(!hall_effect_begin()) {
    Serial.println("Error connecting to ACS712 sensor. Check wiring.");
    while (1);
  }

  if(!temp_sensor_begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }

  if(!heat_plate_begin()) {
    Serial.println("Error connecting to Heat Plate. Check wiring.");
    while (1);
  }

  pid.setSetpoint(60);
}

void loop() {

  double current_temperature = read_temp(true);

  double duty_cycle = pid.compute(current_temperature);

  Serial.print("Temperature: ");
  Serial.print(current_temperature);
  Serial.print("; Duty Cycle: ");
  Serial.println(duty_cycle);

  heat_plate_turn_on(false, duty_cycle);
}