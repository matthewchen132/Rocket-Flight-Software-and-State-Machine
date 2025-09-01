#include <Arduino.h>
// #include "MPU9250.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include "SixDOF.h"
#include "PHT.h"
#include <Adafruit_MS8607.h>
#include <math.h>
#include "GPS.h"
#include <map>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/twai.h"
#include <task.h>
#include <queue.h>
#include "LaunchState.h"
#include <ESP32-TWAI-CAN.hpp>
#include <math.h>
#include <iostream>
#include <sstream>
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 38
#define LSM_SCK 37
#define LSM_MISO 35
#define LSM_MOSI 36
#define PHT_SDA 42
#define PHT_SCL 41
#define ICM_SDA 39
#define ICM_SCL 40
#define CAN_TX 16
#define CAN_RX 17
constexpr int N_DEC = 5;
constexpr float g = 9.81;
constexpr float target_altitude = 116.0;
constexpr int num_sustained_accels = 10;
// Task handles
GPS GPS_NAV;
String canMessage;
int counter = 0;
volatile bool pinStatusUpdated[64] = {};
volatile int pinStatus[64][5] = {};
JsonDocument sensorDataGlobal;

struct imu_output{
  float ax;
  float ay;
  float az;
  float wx;
  float wy;
  float wz;
};
struct coord{
  double lat_;
  double long_;
};

static double altitude;
static std::vector<double> accels;
double net_accel;
static imu_output imu_data;
static coord gps_data;
SemaphoreHandle_t altMutex = xSemaphoreCreateMutex(); // dataserialize
SemaphoreHandle_t imuMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t gpsMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t stateMachineMutex = xSemaphoreCreateMutex();
TickType_t timestamp = 0;

twai_message_t txMessage;
CanFrame rxFrame;
int canTXRXcount[2] = {0, 0};
String messageToCAN(String code);
LaunchState current_state1 = LaunchState::PreIgnition;
SixDOF _6DOF;
// MPU9250 mpu;
TwoWire I2C1(0); // Default I2C bus
PHT Alt(I2C1);
TwoWire I2C2(1);                       // Secondary I2C bus
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
GPS GPS1;
// LaunchState Halya;
int groundLevelAltitudeTest = 0;
std::map<string, bool> statesMapTest;
double AltArrayTest[10];
double previousMedianTest = 0;
bool CHECK = false;
const int rows = 4;
const int cols = 50;
int sendCAN = 1;
int waitforADS = 0;
int printADS[4] = {1, 1, 1, 1};
int printCAN = 1;
String loopprint;
int cycledelay = 2;
int preignitionCount = 0;
// Define a queue to hold the JSON data
QueueHandle_t jsonQueue;
uint8_t integerPartToHex(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);
  if (integerPart > 255)
  {
    integerPart = 255;
  }
  return 0x00 + static_cast<uint8_t>(integerPart);
}
uint8_t decimalPartToHex(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);
  double decimalPart = dataValue - integerPart;
  int scaledDecimal = static_cast<int>(decimalPart * 100);
  // if (scaledDecimal > 255)
  // {
  //     scaledDecimal = 255;
  // }
  uint8_t hexValue = 0x00 + static_cast<uint8_t>(scaledDecimal);
  return hexValue;
}
uint8_t tens_ones_digits(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);
  int first_two = integerPart % 100;
  //  if (integerPart > 255)
  //    {
  //        integerPart = 255;
  //    }
  return 0x00 + static_cast<uint8_t>(first_two);
}
uint8_t thous_hunds_digits(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);
  int last_two = integerPart / 100;
  //  if (integerPart > 255)
  //    {
  //        integerPart = 255;
  //    }
  return 0x00 + static_cast<uint8_t>(last_two);
}
double hextoDecimal(uint8_t hex_thous_hunds, uint8_t hex_tens_ones, uint8_t hex_fraction, uint8_t hex_ex_prec)
{

  double thousHundsResult = static_cast<double>(hex_thous_hunds);
  double tensOnesResult = static_cast<double>(hex_tens_ones);
  double fractionResult = static_cast<double>(hex_fraction);
  double exPrecResult = static_cast<double>(hex_ex_prec);

  return thousHundsResult * 100 + tensOnesResult + fractionResult * (0.01) + exPrecResult * 0.0001;
}
void command2pin(String solboardIDnum, char command, char mode)
{
  twai_message_t txMessage_command;
  int ID = solboardIDnum.toInt();
  txMessage_command.identifier = ID * 0x10 + 0x0F; // Solenoid Board WRITE COMMAND 0xIDf
  txMessage_command.flags = TWAI_MSG_FLAG_EXTD;    // Example flags (extended frame)
  txMessage_command.data_length_code = 8;          // Example data length (8 bytes)
  txMessage_command.data[0] = 0xFF;                // Sol 0 //Be default, over CAN hex FF does not trigger valid response
  txMessage_command.data[1] = 0xFF;                // Sol 1 //HOWEVER, String FXXX over serial may update frequency
  txMessage_command.data[2] = 0xFF;                // Sol 2
  txMessage_command.data[3] = 0xFF;                // Sol 3
  txMessage_command.data[4] = 0xFF;                // Sol 4
  txMessage_command.data[5] = 0xFF;                // NIL
  txMessage_command.data[6] = 0xFF;                // NIL
  txMessage_command.data[7] = 0xFF;                // NIL
  int statusPin;
  if (command == '0')
  {
    statusPin = 0;
  }
  else if (command == '1')
  {
    statusPin = 1;
  }
  else if (command == '2')
  {
    statusPin = 2;
  }
  else if (command == '3')
  {
    statusPin = 3;
  }
  else if (command == '4')
  {
    statusPin = 4;
  }
  if (mode == '0')
  {
    pinStatus[ID][statusPin] = 0;
    txMessage_command.data[statusPin] = 0;
  }
  else if (mode == '1')
  {
    pinStatus[ID][statusPin] = 1;
    txMessage_command.data[statusPin] = 1;
  }
  else
  {
    Serial.println("Invalid mode");
  }
  twai_transmit(&txMessage_command, pdMS_TO_TICKS(1));
  Serial.println("CAN sent");
}
double returnMedianTest(double arr[], int number)
{
  double temp[number];
  memcpy(temp, arr, sizeof(temp));
  std::sort(temp, temp + number);

  if (number % 2 == 0)
  {
    return (temp[number / 2 - 2] + temp[number / 2 - 1] + temp[number / 2] + temp[number / 2 + 1]) / 4.0;
  }
  else
  {
    return (temp[number / 2 - 1] + temp[number / 2] + temp[number / 2 + 1] + temp[number / 2 + 2] + temp[number / 2 + 3]) / 5.0;
    ;
  }
}
void allSol()
{
  delay(100);
  command2pin("01", '1', '1');
  delay(100);
  command2pin("01", '2', '1');
  // command2pin("01", '2', '1');
  //  delay(100);
  //   command2pin("01", '1', '0');
  //  command2pin("01", '2', '1');
  //  delay(100);
  //   command2pin("01", '2', '0');
  //  command2pin("01", '3', '1');
  //  delay(100);
  //   command2pin("01", '3', '0');
}
double calculateRateOfChangeTest(double AltArray[], int READINGS_LENGTH)
{
  double currentMedian = returnMedianTest(AltArray, READINGS_LENGTH);

  double rate_of_change = currentMedian - previousMedianTest;

  previousMedianTest = currentMedian;

  return rate_of_change;
}

// RX Code
int num_decreasing_alts = 0;
int rocCount = 0;

void BabyStateMachine(void *stateMachineMutex){
  while(1){
    switch (current_state1){
    case LaunchState::PreIgnition:
    {
      /* TODO:
      some code here to check Initialization worked out. Sync with the setup function
      */
     if(xSemaphoreTake(imuMutex,0) == pdTRUE){
      if (net_accel >= 2*g){
        preignitionCount += 1;
        Serial.println("\nPulling 2 G's. Number of 2G readings required: " +String(num_sustained_accels-preignitionCount) + "\n" );
      }
      if (preignitionCount > num_sustained_accels){
        command2pin("01", '0', '1'); // LED / visual indicator
        Serial.println("\nIgnition was detected through sustained forces > 2G. \n");
        current_state1 = LaunchState::Ignition_to_Apogee;
        vTaskDelay(1000);
      }
      xSemaphoreGive(imuMutex);
      break;
     }
    }
    //I COMMENTED OUT RISAB'S IGNITION TO APOGEE, IT IS USEFUL BUT I HAVE A BAREBONES CHECKER RN.
    /*case LaunchState::Ignition_to_Apogee:
    {
      static int count = 0;

      // Read sensor values
      double PHT_alt = _Alt.getAltitude();
      double GPS_alt = _GPS1.getAltitude();
      double bottom_alt = bottomAlt; // Using the provided bottom altitude

      // Error detection
      bool PHT_error = (PHT_alt == 0);
      // Serial.println("PHT error: " + String(PHT_error));
      bool GPS_error = !(_GPS1.fix);
      // Serial.println("GPS error: " + String(GPS_error));
      bool bottom_error = (bottom_alt == 0);
      // Serial.println("Bottom Sensors Error: " + String(bottom_error));

      // Calculate final altitude with weighted agreement between sensors
      double final_altitude = 0;
      int agreement_count = 0;

      // Check for agreement between PHT and GPS (primary sensors)
      bool PHT_GPS_agree = (!PHT_error && !GPS_error && fabs(PHT_alt - GPS_alt) < 0.5);
      // Check for agreement between PHT and bottom alt
      bool PHT_bottom_agree = (!PHT_error && !bottom_error && fabs(PHT_alt - bottom_alt) < 0.5);
      // Check for agreement between GPS and bottom alt
      bool GPS_bottom_agree = (!GPS_error && !bottom_error && fabs(GPS_alt - bottom_alt) < 0.5);
      // Priority 1: At least two sensors agree (with preference for PHT-GPS agreement)
      if (PHT_GPS_agree){
        // Strong preference for PHT-GPS agreement
        final_altitude = (PHT_alt * 0.6 + GPS_alt * 0.4); // Weight PHT slightly more
        agreement_count = 2;
      }
      else if (PHT_bottom_agree || GPS_bottom_agree){
        // Secondary preference for other agreements
        if (PHT_bottom_agree && GPS_bottom_agree){
          // All three agree (within margins)
          final_altitude = (PHT_alt + GPS_alt + bottom_alt) / 3.0;
          agreement_count = 3;
        }
        else if (PHT_bottom_agree){
          final_altitude = (PHT_alt * 0.5 + bottom_alt * 0.5); // Weight PHT more
          agreement_count = 2;
        }
        else{                                                      // GPS_bottom_agree
          final_altitude = (GPS_alt * 0.5 + bottom_alt * 0.5); // Weight GPS more
          agreement_count = 2;
        }
      }
      // Priority 2: Use available sensors with preference order
      else{
        // Prefer PHT if available
        if (!PHT_error){
          final_altitude = PHT_alt;
          // If GPS is also available but doesn't agree, we might want to know
          if (!GPS_error){
            // Large discrepancy - might want to flag this
            if (fabs(PHT_alt - GPS_alt) > 2.0){
              Serial.println("Warning: Large PHT-GPS altitude discrepancy");
            }
          }
        }
        // Fall back to GPS if PHT not available
        else if (!GPS_error){
          final_altitude = GPS_alt;
        }
        // Final fallback to bottom altitude
        else if (!bottom_error){
          final_altitude = bottom_alt;
        }
        else{
          // All sensors failed - this should be handled as an error
          Serial.println("Error: All altitude sensors failed!");
          // Might want to implement recovery logic here
          final_altitude = 0; // Or maintain last known good value
        }
      }
      // Store the altitude values for rate of change calculation
      Serial.println("Final Altitude = " + String(final_altitude));
      Serial.println("Count = " + String(count));
      AltArrayTest[count % 10] = final_altitude;
      count++;

      // Calculate rate of change and check for apogee
      if (count >= 10)
      {
        double rate_of_change = calculateRateOfChangeTest(AltArrayTest, 10);
        Serial.println("Rate of change: " + String(rate_of_change));
        // More conservative apogee detection with additional checks
        if ((rate_of_change < 0)){
          Serial.println("Apogee detected - transitioning to descent phase");
          command2pin("01", '2', '1'); // purely an indicator LED
          current_state1 = LaunchState::_1000ft;
          rocCount = 0;
          count = 0;
        }
        else{
          // Reset counter if condition isn't consistently met
          if (rocCount > 0)
            rocCount--;
        }
      }
      break;
    }*/
    case LaunchState::Ignition_to_Apogee:{
      /*Code here to find the most trusted and reliable sensor:
        TODO
      */
      if(xSemaphoreTake(altMutex,0) ==pdTRUE){
        double best_altitude = altitude;
        xSemaphoreGive(altMutex);
        vTaskDelay(pdMS_TO_TICKS(11)); // delay to get a refreshed altitude reading to compare to.
        if(altitude - best_altitude <=0){
          num_decreasing_alts += 1;
        } 
        else{
          num_decreasing_alts = 0;
        }
      }
      if(num_decreasing_alts >= N_DEC){
        current_state1 = LaunchState::_1000ft;
        Serial.println("\nApogee was Detected!");
        command2pin("06", '2', '1'); // CORRESPONDING PIN TO DROGUE EMATCH
        timestamp = xTaskGetTickCount();
      }
      break;
    }
    case LaunchState::_1000ft:
    {
      if(xTaskGetTickCount() - timestamp <= pdMS_TO_TICKS(200)){
        break; // prevents false MAIN deployment from black powder pressurization
      }
      Serial.println("\n\nSearching for target altitude:\n\n");
      if(altitude <= target_altitude){
        current_state1 = LaunchState::Descent;
      }
      break;
    }
    // case LaunchState::_900ft:
    // {
    //   double PHT_alt = _Alt.getAltitude();
    //   double GPS_alt = _GPS1.getAltitude();
    //   double bottom_alt = bottomAlt;
    //   bool PHT_error = (PHT_alt == 0);
    //   bool GPS_error = !(_GPS1.fix);
    //   bool bottom_error = (bottom_alt == 0);
    //   double final_altitude = 0;
    //   if (!PHT_error && !GPS_error)
    //   {
    //     final_altitude = (PHT_alt + GPS_alt) / 2;
    //   }
    //   else if (!PHT_error)
    //   {
    //     final_altitude = PHT_alt;
    //   }
    //   else if (!GPS_error)
    //   {
    //     final_altitude = GPS_alt;
    //   }
    //   else if (!bottom_error)
    //   {
    //     final_altitude = bottom_alt;
    //   }
    //   else
    //   {
    //     break;
    //   }
    //   Serial.println("Current Altitude: " + String(final_altitude) + " ft");
    //   // Transition when below 900 ft
    //   if (final_altitude < 900)
    //   {
    //     Serial.println("900 ft threshold reached! Moving to 800 ft state.");
    //     command2pin("06", '1', '1'); // Next parachute action
    //     current_state1 = LaunchState::_800ft;
    //   }
    //   break;
    // }
    // case LaunchState::_800ft:
    // {
    //   double PHT_alt = _Alt.getAltitude();
    //   double GPS_alt = _GPS1.getAltitude();
    //   double bottom_alt = bottomAlt;
    //   bool PHT_error = (PHT_alt == 0);
    //   bool GPS_error = !(_GPS1.fix);
    //   bool bottom_error = (bottom_alt == 0);
    //   double final_altitude = 0;
    //   if (!PHT_error && !GPS_error)
    //   {
    //     final_altitude = (PHT_alt + GPS_alt) / 2;
    //   }
    //   else if (!PHT_error)
    //   {
    //     final_altitude = PHT_alt;
    //   }
    //   else if (!GPS_error)
    //   {
    //     final_altitude = GPS_alt;
    //   }
    //   else if (!bottom_error)
    //   {
    //     final_altitude = bottom_alt;
    //   }
    //   else
    //   {
    //     break;
    //   }
    //   Serial.println("Current Altitude: " + String(final_altitude) + " ft");
    //   // Transition when below 800 ft
    //   if (final_altitude < 800)
    //   {
    //     Serial.println("800 ft threshold reached! Moving to descent phase.");
    //     command2pin("06", '2', '1'); // Final parachute action
    //     current_state1 = LaunchState::Descent;
    //   }
    //   break;
    // }
    case LaunchState::Descent:
    {
      command2pin("06", '3', '1'); // MAIN CHUTE Deployment
      vTaskDelay(5000);
      current_state1 = LaunchState::Touchdown;
      break;
    }
    case LaunchState::Touchdown:
    {
      Serial.println("\n\nRocket has touched down!");
      Serial.println("Location:" + String(gps_data.lat_) + "," + String(gps_data.long_));
      break;
    }
    }
    vTaskDelay(10);
  }
}

uint8_t extraPrecision(double dataValue)
{
  int integerPart = static_cast<int>(dataValue);
  double decimalPart = dataValue - integerPart;
  int scaledDecimal = static_cast<int>(decimalPart * 10000);
  int filteredDecimal = scaledDecimal % 100;
  uint8_t hexValue = 0x00 + static_cast<uint8_t>(filteredDecimal);
  return hexValue;
}
double returnAverageTest(double arr[], int number)
{
  double sum = 0;
  for (int count = 0; count < number - 1; count++)
  {
    sum += arr[count];
  }
  return sum / number;
}
// TWAI/CAN RECIEVE MESSAGE
void commandTask(String can_code)
{
  // String canMessage = *(String *)pvParameters;  // Cast and dereference the passed parameter for FreeRTOS specific format

  while (1)
  {
    // Using the passed message
    String message = can_code;

    // Extract solboardIDnum, command, and mode from the message
    String solboardIDnum = message.substring(0, message.length() - 2);
    char command = message[message.length() - 2];
    char mode = message[message.length() - 1];

    switch (command)
    {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
      command2pin(solboardIDnum, command, mode); // sends command to activate the solenoid HIGH
      break;
    default:
      vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for a while
      yield();
      vTaskDelay(10);
    }
  }
}
void printTwaiStatus()
{
  twai_status_info_t status;
  twai_get_status_info(&status);
  Serial.print("Status:  ");
  switch (status.state)
  {
  case (TWAI_STATE_STOPPED):
    Serial.println("Stopped");
    break;
  case (TWAI_STATE_RUNNING):
    Serial.println("Running");
    break;
  case (TWAI_STATE_BUS_OFF):
    Serial.println("Bus Off");
    break;
  case (TWAI_STATE_RECOVERING):
    Serial.println("Recovering");
    break;
  default:
    Serial.println("Unknown");
    break;
  }
  Serial.print("Tx Error Counter: ");
  Serial.println(status.tx_error_counter);
  Serial.print("Rx Error Counter: ");
  Serial.println(status.rx_error_counter);
  Serial.print("Bus Error Count: ");
  Serial.println(status.bus_error_count);
  Serial.print("Messages to Tx: ");
  Serial.println(status.msgs_to_tx);
  Serial.print("Messages to Rx: ");
  Serial.println(status.msgs_to_rx);
}
void start_twai(){
  
pinMode(CAN_TX, OUTPUT);
pinMode(CAN_RX, INPUT);
// Config CAN Speed
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// Install and start the TWAI driver
esp_err_t canStatus = twai_driver_install(&g_config, &t_config, &f_config);
if (canStatus == ESP_OK)
{
  Serial.println("CAN Driver installed");
}
else
{
  Serial.println("CAN Driver installation failed");
}
if (twai_start() != ESP_OK)
{
  Serial.println("Error starting TWAI!");
}
// starts TWAI
Serial.println("Staring CAN bus...");
ESP32Can.setRxQueueSize(8);
ESP32Can.setTxQueueSize(8);
}
// RTOS TASKS:
void AltimeterTask(void *mutex){
  auto *Mutex = static_cast<SemaphoreHandle_t>(mutex);
  configASSERT(Mutex);
  TickType_t altimeter_delay = pdMS_TO_TICKS(100);
  while(1){
    if(xSemaphoreTake(Mutex, portMAX_DELAY)){ // take the mutex if not occupied and delay indefinitely while modifying altitude.
      altitude = Alt.getAltitude();
      // Serial.println("Altimeter (100ms Delay):" + String(altitude));
      xSemaphoreGive(Mutex);
    }
    vTaskDelay(altimeter_delay);
  }
}
void IMU6_Task(void *mutex){
  auto *Mutex = static_cast<SemaphoreHandle_t>(mutex);
  configASSERT(Mutex);
  TickType_t IMU_delay = pdMS_TO_TICKS(25); // IMU Reading delay
  while(1){
    if(xSemaphoreTake(Mutex,portMAX_DELAY)){
      accels = _6DOF.getAcceleration();
      net_accel = _6DOF.getNetAccel();
      // Serial.println("IMU net acceleration (25ms Delay):" + String(net_accel));
      xSemaphoreGive(Mutex);
    }
    vTaskDelay(IMU_delay);
  }
}
void GPS_Task(void *mutex){ // NOTE: This task is simulating gps values. Values are NOT ACCURATE!!!
  auto *Mutex = static_cast<SemaphoreHandle_t>(mutex);
  while(1){
    if(xSemaphoreTake(Mutex, portMAX_DELAY)){
      gps_data.lat_ = 32.2;
      gps_data.long_ = 34.2;
      // Serial.println("GPS (100ms Delay):" + String(gps_data.lat_) + "," + String(gps_data.long_)); 
      xSemaphoreGive(Mutex);
    }
    vTaskDelay(100); 
  }
}

void setup(){
Serial.begin(115200);
  while (!Serial){
  Serial.print("Serial Failed to start");
  delay(10);
}
start_twai();
if (!(_6DOF.start_6DOF()))
{
  Serial.println("6DOF Failed to start");
}
I2C1.begin(42, 41);
if (!Alt.connectSensor())
{
  Serial.println("Error connecting to PHT sensor");
}
else
{
  Serial.println("Connected to Alt sensor");
  Alt.setSensorConfig();
}
GPS1.startGPS();
delay(10);

//RTOS Tasks:
  // Core 0: Sensor Concurrency
  xTaskCreatePinnedToCore(AltimeterTask, "read_altimeter", 2048, altMutex, 2, nullptr, 0);
  xTaskCreatePinnedToCore(IMU6_Task, "Six_DOF_Task", 4096, imuMutex, 1, nullptr, 0); 
  xTaskCreatePinnedToCore(GPS_Task, "GPS_Task", 2048, gpsMutex, 3, nullptr, 0); 
  // Core 1: State Machine
  xTaskCreatePinnedToCore(BabyStateMachine, "GPS_Task", 2048, stateMachineMutex, 4, nullptr, 1); 
}

void loop(){
vTaskDelete(nullptr); // removes the loop from executing
}
