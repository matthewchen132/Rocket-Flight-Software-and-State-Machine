#include <Wire.h>
#include <ADS1X15.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"

// Task handles
TaskHandle_t transmitTaskHandle;
TaskHandle_t receiveTaskHandle;

// Function prototypes
void transmitTask(void *pvParameters);
void receiveTask(void *pvParameters);
void commandTask(void *pvParameters);

volatile int pinStatus[5] = {0,0,0,0,0};

StaticJsonDocument<512> sensorData;

// CAN TWAI message to send
twai_message_t txMessage;

// Pins used to connect to CAN bus transceiver:
#define CAN_RX 2 //using protoboard pins, RevA onwards://10
#define CAN_TX 1 //using protoboard pins, RevA onwards://9
int canTXRXcount[2] = { 0, 0 };


// Create instances of TwoWire for different I2C interfaces
TwoWire I2C_one = TwoWire(0);
TwoWire I2C_two = TwoWire(1);

const int baudrate = 921600;
const int rows = 4;
const int cols = 50;
int sendCAN = 1;

//printing helper variables
int waitforADS = 0;
int printADS[4] = { 1, 1, 1, 1 };
int printCAN = 1;

String loopprint;

int cycledelay = 2;


void setup() {
  // Start the I2C interfaces
  Serial.begin(baudrate);  // Initialize Serial communication
  Serial.println("Begin");

  pinMode(CAN_TX, OUTPUT);
  pinMode(CAN_RX, INPUT);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install and start the TWAI driver
  esp_err_t canStatus = twai_driver_install(&g_config, &t_config, &f_config);
  if (canStatus == ESP_OK) {
    Serial.println("CAN Driver installed");
  } else {
    Serial.println("CAN Driver installation failed");
  }
  twai_start();
  // Prepare the message to send
  txMessage.identifier = 0x01;           // Example identifier
  txMessage.flags = TWAI_MSG_FLAG_EXTD;  // Example flags (extended frame)
  txMessage.data_length_code = 8;        // Example data length (8 bytes)
  txMessage.data[0] = 0xFF;              // Reserved for message type
  txMessage.data[1] = 0xFF;              // Sensor 1: Temperature
  txMessage.data[2] = 0xFF;              // Sensor 2: Humidity
  txMessage.data[3] = 0xFF;              // Sensor 3: X axis Acceleration
  txMessage.data[4] = 0xFF;              // Sensor 4: Y axis Acceleration
  txMessage.data[5] = 0xFF;              // Sensor 5: Z axis Acceleration
  txMessage.data[6] = 0xFF;              // Sensor 6: Power Use in W
  txMessage.data[7] = 0xFF;              // Reserved for message Count

  Serial.println("CAN Started");
  twai_transmit(&txMessage, pdMS_TO_TICKS(1));

  // Create transmit task
  //xTaskCreatePinnedToCore(transmitTask, "Transmit Task", 4096, NULL, 1, &transmitTaskHandle, 0);
  // Create receive task
  xTaskCreatePinnedToCore(receiveTask, "Receive Task", 2048, NULL, 2, &receiveTaskHandle, 1);
  xTaskCreatePinnedToCore(commandTask, "CommandTask", 4096, NULL, 1, NULL, 0);

  Serial.println("RTOS Tasks Created");
}


void transmitTask(void *pvParameters) {
  static int count;
  Serial.println("Start Transmit DAQ");
  while (1) {
    if (sendCAN == 1) {
      twai_transmit(&txMessage, pdMS_TO_TICKS(1));
      Serial.println("sent");
    }
    waitforADS = 0;
    Serial.print("Send Count: ");
    Serial.print(count++);
    Serial.println();
    // Serial.println(transmission_err == ESP_OK);
    // Serial.println(transmission_err == ESP_ERR_INVALID_ARG);
    // Serial.println(transmission_err == ESP_ERR_TIMEOUT);
    // Serial.println(transmission_err == ESP_FAIL);
    // Serial.println(transmission_err == ESP_ERR_INVALID_STATE);
    // Serial.println(transmission_err == ESP_ERR_NOT_SUPPORTED);
    delay(cycledelay);  // Delay for a second before reading again
  }
}

void command2pin(char command, char mode){ //mode 0: send and receive
    twai_message_t txMessage_command;
    txMessage_command.identifier = 0x5F;           // Solenoid Board WRITE COMMAND 0x5f
    txMessage_command.flags = TWAI_MSG_FLAG_EXTD;  // Example flags (extended frame)
    txMessage_command.data_length_code = 8;        // Example data length (8 bytes)
    txMessage_command.data[0] = 0xFF;              // Reserved for message type
    txMessage_command.data[1] = 0xFF;              // Sensor 1: Temperature
    txMessage_command.data[2] = 0xFF;              // Sensor 2: Humidity
    txMessage_command.data[3] = 0xFF;              // Sensor 3: X axis Acceleration
    txMessage_command.data[4] = 0xFF;              // Sensor 4: Y axis Acceleration
    txMessage_command.data[5] = 0xFF;              // Sensor 5: Z axis Acceleration
    txMessage_command.data[6] = 0xFF;              // Sensor 6: Power Use in W
    txMessage_command.data[7] = 0xFF;              // Reserved for message Count
  
    int statusPin;
    if (command == '0') {statusPin = 0;}
    else if (command == '1') {statusPin = 1;}
    else if (command == '2') {statusPin = 2;}
    else if (command == '3') {statusPin = 3;}
    else if (command == '4') {statusPin = 4;}
  
    if (mode == '0') {
      pinStatus[statusPin] = 0;
    } else if (mode == '1') {
      pinStatus[statusPin] = 1;
    } else {
      Serial.println("Invalid mode");
    }
  
    for (int i = 0; i < 5; i++){
      txMessage_command.data[i] = pinStatus[i];
      /*
      Serial.print("\t Pin ");
      Serial.print(i);
      Serial.print(":");
      Serial.print(pinStatus[i]);
      */
    }
  
    //Serial.println();
  
    twai_transmit(&txMessage_command, pdMS_TO_TICKS(1));
    //Serial.println("CAN sent");
      
  }

  // activate_SOL("1", 1, 1); turns on board 1 sol 1
/* 
*/
void activate_SOL(String solboardIDnum, char command, char mode){ // solboardIDnum is "1" for board 0x01, command = solenoid number (0,1,2,3,4 etc), mode = 0 for off, 1 for on
  twai_message_t txMessage_command;
  int ID = solboardIDnum.toInt();
  txMessage_command.identifier = ID*0x10 + 0x0F; // Solenoid Board WRITE COMMAND 0xIDf
  txMessage_command.flags = TWAI_MSG_FLAG_EXTD;  // Example flags (extended frame)
  txMessage_command.data_length_code = 5;        // Example data length (8 bytes)
  //Solenoid Commands
  //By default, over CAN hex FF does not trigger valid response
  //HOWEVER, String FXXX over serial may update frequency
  txMessage_command.data[0] = 0xFF;              // Sol 0 Apogee E-match 1 
  txMessage_command.data[1] = 0xFF;              // Sol 1 Apogee E-match 2
  txMessage_command.data[2] = 0xFF;              // Sol 2 Main Chute 1
  txMessage_command.data[3] = 0xFF;              // Sol 3 Main Chute 2


  int statusPin;
  if (command == '0') {statusPin = 0;}
  else if (command == '1') {statusPin = 1;}
  else if (command == '2') {statusPin = 2;}
  else if (command == '3') {statusPin = 3;}
  else if (command == '4') {statusPin = 4;}

  if (mode == '0') {
    pinStatus[statusPin] = 0;
  } else if (mode == '1') {
    pinStatus[statusPin] = 1;
  } else {
    Serial.println("Invalid mode");
  }
  /*
  if (pinStatusUpdated[ID]) {
    for (int i = 0; i < 5; i++){
      // supposed to keep track of pin, unneccessary. Also, weird bug rn
      //txMessage_command.data[i] = pinStatus[ID][i];
      Serial.print("\t Pin ");
      Serial.print(i);
      Serial.print(":");
      Serial.print(pinStatus[i]);
    }
    //Serial.println();
  }
  */
  twai_transmit(&txMessage_command, pdMS_TO_TICKS(1));
  //xSemaphoreGive(mutex_d);
  Serial.println("Solenoid " + String(statusPin) + "activated");
}

void receiveTask(void *pvParameters) {
  static String receivedCANmessagetoprint;
  Serial.println("Starting receive task...");
  delay(1000);
  while (1) {
    if (printCAN == 1) {
      twai_message_t rxMessage;
      esp_err_t receiveerror = twai_receive(&rxMessage, pdMS_TO_TICKS(50));
      if (receiveerror == ESP_OK) {

        if (String(rxMessage.identifier, HEX) == "51"){ //Check if Write Command issued at 0x51 (Current Status)
        char modes[2] = {'0','1'};
          for (int i = 0; i < 5; i++){
            pinStatus[i] = rxMessage.data[i];
          }
        }
        
        sensorData["BoardID"] = String(rxMessage.identifier, HEX);
        sensorData["SensorType"] =  String((rxMessage.identifier - 0x10*(rxMessage.identifier/0x10)), HEX);

        //Print the received message
        canTXRXcount[1] += 1;
        receivedCANmessagetoprint += "Total Can Received: ";
        receivedCANmessagetoprint += String(canTXRXcount[1]) + "\n";
        receivedCANmessagetoprint += "Received Message - ID: 0x";
        receivedCANmessagetoprint += String(rxMessage.identifier, HEX);
        receivedCANmessagetoprint += ", DLC: ";
        receivedCANmessagetoprint += String(rxMessage.data_length_code);
        receivedCANmessagetoprint += ", Data: ";
        for (uint8_t i = 0; i < rxMessage.data_length_code; i++) {
          receivedCANmessagetoprint += String(rxMessage.data[i], HEX);
          receivedCANmessagetoprint += " ";
          sensorData["Sensors"][i] = rxMessage.data[i];
        }
        receivedCANmessagetoprint += "\n";
        serializeJson(sensorData, Serial);
        Serial.println();
        //Serial.println(receivedCANmessagetoprint);
        receivedCANmessagetoprint = "";
      }
    }
  }
}


void commandTask(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    char command = Serial.read();
    char mode = Serial.read();;
    switch (command) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
        command2pin(command,mode); // 
        break;
      default:
        //Serial.println("Invalid command");
        delay(10);
    }
  }
}


void loop() {
  static String loopmessagetoprint;
  yield();
  delay(1000);
  Serial.println("CAN Reader Active");
}