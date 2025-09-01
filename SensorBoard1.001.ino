#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include "SixDOF.h"
#include "PHT.h"
#include <Adafruit_MS8607.h>
#include <math.h>
#include "GPS.h"
// #include "LaunchState.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 5
#define LSM_SCK 18
#define LSM_MISO 19
#define LSM_MOSI 23
SixDOF _6DOF;
PHT Alt;
MPU9250 mpu;
TwoWire I2C_two(1);
uint16_t measurement_delay_us = 65535;  // Delay between measurements for testing
GPS GPS1;
// LaunchState Halya;



void setup() {
  Serial.begin(115200);
  
  while (!Serial) {
    Serial.print("Serial Failed to start");
    delay(10);
  }

  GPS1.startGPS();
  delay(500);

  if(!(_6DOF.start_6DOF())){
      Serial.println("6DOF Failed to start");
    }
  // Altimeter
  Alt.startPHT();
  // 9 DOF
  I2C_two.begin(21, 22);
  I2C_two.setClock(400000);
  delay(1000);
  if (!mpu.setup(0x68, MPU9250Setting(), I2C_two)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(1000);
    }
  }
}

void loop() {
//GPS
  
//6DOF
  Serial.print(_6DOF.printSensorData());
  Serial.print(String(_6DOF.check_IGNITABLE()) + "\n");
//Altimeter
  Serial.print(Alt.printReadings());
//9DOF  
  Serial.print("(9DOF)  ");         //9DOF tweaking
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    Serial.print(String(mpu.getAcc(prev_ms))+ " ");
    Serial.print(String(mpu.getGyro(prev_ms))+ "\n"+ "\n"); 
  }
  GPS1.printInfo();
  delay(250);
  // Halya.HalyaStateMachine(_6DOF, Alt, mpu);
}


















// Initialization: Initial state where sensors are set up and variables initialized.
// Pre-Launch: Reads sensor data, calculates altitude, and checks for ignition conditions.
// Ignition 1: Triggers ignition 1 and waits for a short delay.
// Ignition 2: Triggers ignition 2 (if certain conditions are met).
// Post-Launch: Reads sensor data and transmits telemetry (optional).
// Transitions between States:

// Initialization -> Pre-Launch: After successful initialization.
// Pre-Launch -> Pre-Launch: Stays in this state until ignition conditions are met.
// Pre-Launch -> Ignition 1: When IGNITABLE becomes true, rate_of_change is non-positive, and altitude is above 900 meters.
// Ignition 1 -> Ignition 2: After a short delay (1 second in your code).
// Ignition 2 (optional) -> Post-Launch: If a second ignition stage exists and its conditions are met.
// Pre-Launch/Ignition 1/Ignition 2 -> Error (optional): If any critical sensor readings fail or errors occur.

