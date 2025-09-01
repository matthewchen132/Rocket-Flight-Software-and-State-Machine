#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include "SixDOF.h"
#include "PHT.h"
#include <Adafruit_MS8607.h>
#include <math.h>
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


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    Serial.print("Serial Failed to start");
    delay(10);
  }
  if(!(_6DOF.start_6DOF())){
      Serial.println("6DOF Failed to start");
    }
  // Altimeter
  Alt.startPHT();
  // 9 DOF
  I2C_two.begin(21, 22);
  I2C_two.setClock(400000);
  delay(2000);
  if (!mpu.setup(0x68, MPU9250Setting(), I2C_two)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
}

void loop() {
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
  // HalyaStateMachine(_6DOF, Alt, mpu);
delay(200);

  //  TO DO list:
  //  (1) update the loop where AltArray has a count larger than its length. currently,
  //  (2) implement the return_average function
  //  (3) testing
  //  (4) CAN


























  // int count = 0
  // bool ignition1 = false;
  //   case (count < READINGS_LENGTH): {
  //     AltArray[count] = altitude;
  //       ms8607.getEvent(&pressure, &temp, &humidity); //maybe wrong
  //   } break;

  //   case (count >= READINGS_LENGTH): {
  //     for (int i = 0; i < READINGS_LENGTH - 1; i++){
  //     AltArray[i] = AltArray[i+1];
  //     }
  //     AltArray[READINGS_LENGTH - 1] = altitude; //does altitude update in the loop? I think that it would just set everything in AltArray to the same value
  //     rate_of_change = return_average(AltArray, READINGS_LENGTH);
  //   } break;

  //   case (Net_Accel > 20): {
  //     IGNITABLE = true;
  //   } break;
  // }

  // switch (true) {
  //   case (IGNITABLE && rate_of_change <=0 && altitude >= 900): {
  //     digitalWrite(1, HIGH);
  //     Serial.print("Ignition 1 occured");
  //     ignition1 = true;
  //     delay(1000); //delay is so that ignition 2 doesnt happen out of nowhere
  //   } break;

  //   case (IGNITABLE && rate_of_change <=0 && altitude <= 1000 && ignition1): {
  //     digitalWrite(2, HIGH);
  //     Serial.print("Ignition 2 occured");
  //   } break;
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

