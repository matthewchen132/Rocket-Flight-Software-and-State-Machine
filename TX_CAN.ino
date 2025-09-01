#include "ESP32-TWAI-CAN.hpp"
#include "PHT.h"
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "GPS.h"
#include "SixDOF.h"
// Define CAN pins (ESP32 DevKit)
#define CAN_TX 16
#define CAN_RX 17
PHT Alt;
GPS GPS1;
CanFrame txFrame;
SixDOF _6DOF;
// CAN bus speed (adjust as needed)
const int CAN_SPEED = 500; // 500 kbps
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
uint8_t extraPrecision(double dataValue)
{
    int integerPart = static_cast<int>(dataValue);
    double decimalPart = dataValue - integerPart;
    int scaledDecimal = static_cast<int>(decimalPart * 10000);
    int filteredDecimal = scaledDecimal % 100;
    // if (scaledDecimal > 255)
    // {
    //     scaledDecimal = 255;
    // }
    uint8_t hexValue = 0x00 + static_cast<uint8_t>(filteredDecimal);
}
void setup()
{
    ESP32Can.setPins(16, 17);
    Serial.begin(115200);
    Alt.startPHT();
    GPS1.startGPS();
    if (!(_6DOF.start_6DOF()))
    {
        Serial.println("6DOF Failed to start");
    }
    pinMode(CAN_TX, OUTPUT);
    pinMode(CAN_RX, INPUT);
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
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
    // twai_start();
    // Initialize CAN bus at specified speed and define TX/RX pins
    // initialization
    if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED), CAN_TX, CAN_RX))
    {
        Serial.println("CAN bus initialization failed!");
        while (1)
            ;
    }
    else
    {
        Serial.println("CAN bus started!");
    }
    ESP32Can.setRxQueueSize(8);
    ESP32Can.setTxQueueSize(8);
    txFrame.identifier = 0x01;          // Example identifier
    txFrame.flags = TWAI_MSG_FLAG_EXTD; // Example flags (extended frame)
    txFrame.data_length_code = 8;       // Example data length (8 bytes)
    txFrame.data[0] = 0xFF;             // Reserved for message type
    txFrame.data[1] = 0xFF;             // Sensor 1: Temperature
    txFrame.data[2] = 0xFF;             // Sensor 2: Humidity
    txFrame.data[3] = 0xFF;             // Sensor 3: X axis Acceleration
    txFrame.data[4] = 0xFF;             // Sensor 4: Y axis Acceleration
    txFrame.data[5] = 0xFF;             // Sensor 5: Z axis Acceleration
    txFrame.data[6] = 0xFF;             // Sensor 6: Power Use in W
    txFrame.data[7] = 0xFF;             // Reserved for message Count
    Serial.println("CAN Started");
    twai_transmit(&txFrame, pdMS_TO_TICKS(1));
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
void loop()
{
    static uint32_t lastStamp = 0;
    uint32_t currentStamp = millis();
    if (currentStamp - lastStamp > 100)
    { // Send message every second
        // PHT
        lastStamp = currentStamp;
        // Returns number of messages in queue
        Serial.println(ESP32Can.inTxQueue());
        txFrame.identifier = 0xAA; // Replace with your desired CAN ID
        txFrame.extd = 0;
        txFrame.data_length_code = 6;
        nmea_float_t GPS_latitude = GPS1.latitude;
        nmea_float_t GPS_longitude = GPS1.longitude;
        double GPS_latitude_degrees = GPS1.convertToDegrees(GPS_latitude);
        Serial.print(GPS_latitude_degrees);
        txFrame.data[0] = integerPartToHex(GPS_latitude_degrees);
        txFrame.data[1] = decimalPartToHex(GPS_latitude_degrees);
        txFrame.data[2] = extraPrecision(GPS_latitude_degrees);
        double GPS_longitude_degrees = GPS1.convertToDegrees(GPS_longitude);
        Serial.print(GPS_longitude_degrees);
        txFrame.data[3] = integerPartToHex(GPS_longitude_degrees);
        txFrame.data[4] = decimalPartToHex(GPS_longitude_degrees); // Best to use 0xAA (0b10101010) instead of 0
        txFrame.data[5] = extraPrecision(GPS_longitude_degrees);
        // ... fill other data bytes if needed
        if (ESP32Can.writeFrame(&txFrame), 100)
        {
            printTwaiStatus();
            Serial.println("GPS Frame transmitted successfully!");
        }
        else
        {
            printTwaiStatus();
            Serial.println("GPS Frame transmission failed!");
        }

        // PHT FRAME
        txFrame.identifier = 0xBB;    // Different CAN ID for second frame
        txFrame.extd = 0;             // Standard frame
        txFrame.data_length_code = 8; // Data length
        double altitude_reading = Alt.getAltitude();
        Serial.println(altitude_reading);
        txFrame.data[0] = integerPartToHex(altitude_reading);
        txFrame.data[1] = decimalPartToHex(altitude_reading);
        altitude_reading = Alt.getAltitude();
        Serial.println(altitude_reading);
        txFrame.data[2] = integerPartToHex(altitude_reading);
        txFrame.data[3] = decimalPartToHex(altitude_reading); // Best to use 0xAA (0b10101010) instead of 0
        altitude_reading = Alt.getAltitude();
        Serial.println(altitude_reading);
        txFrame.data[4] = integerPartToHex(altitude_reading); // CAN works better this way as it needs
        txFrame.data[5] = decimalPartToHex(altitude_reading); // to avoid bit-stuffing
        altitude_reading = Alt.getAltitude();
        Serial.println(altitude_reading);
        txFrame.data[6] = integerPartToHex(altitude_reading);
        txFrame.data[7] = decimalPartToHex(altitude_reading);

        if (ESP32Can.writeFrame(&txFrame))
        {
            Serial.println("PHT frame transmitted successfully!");
        }
        else
        {
            Serial.println("Failed to transmit PHT frame.");
        }

        // 6DOF Frame --> Accelerometer
        txFrame.identifier = 0xCC;    // Different CAN ID for second frame
        txFrame.extd = 0;             // Standard frame
        txFrame.data_length_code = 6; // Data length
        vector<double> accelArr = _6DOF.getAcceleration();
        txFrame.data[0] = integerPartToHex(accelArr[0]);
        txFrame.data[1] = decimalPartToHex(accelArr[0]);
        txFrame.data[2] = integerPartToHex(accelArr[1]);
        txFrame.data[3] = decimalPartToHex(accelArr[1]); // Best to use 0xAA (0b10101010) instead of 0
        txFrame.data[4] = integerPartToHex(accelArr[2]); // CAN works better this way as it needs
        txFrame.data[5] = decimalPartToHex(accelArr[2]); // to avoid bit-stuffing

        if (ESP32Can.writeFrame(&txFrame))
        {
            Serial.println("6DOF Accelerometer Data frame transmitted successfully!");
        }
        else
        {
            Serial.println("Failed to transmit 6DOF Accelerometer Data .");
        }

        // 6DOF Frame --> Gyroscope
        txFrame.identifier = 0xCD;    // Different CAN ID for second frame
        txFrame.extd = 0;             // Standard frame
        txFrame.data_length_code = 6; // Data length
        vector<double> gyroArr = _6DOF.getGyro();
        txFrame.data[0] = integerPartToHex(gyroArr[0]);
        txFrame.data[1] = decimalPartToHex(gyroArr[0]);
        txFrame.data[2] = integerPartToHex(gyroArr[1]);
        txFrame.data[3] = decimalPartToHex(gyroArr[1]); // Best to use 0xAA (0b10101010) instead of 0
        txFrame.data[4] = integerPartToHex(gyroArr[2]); // CAN works better this way as it needs
        txFrame.data[5] = decimalPartToHex(gyroArr[2]); // to avoid bit-stuffing

        if (ESP32Can.writeFrame(&txFrame))
        {
            Serial.println("6DOF Gyroscope Data frame transmitted successfully!");
        }
        else
        {
            Serial.println("Failed to transmit 6DOF Gyroscope Data .");
        }

        if (ESP32Can.writeFrame(&txFrame))
        {
            Serial.println("6DOF Orientation Data frame transmitted successfully!");
        }
        else
        {
            Serial.println("Failed to transmit 6DOF Orientation Data .");
        }
    }

    twai_status_info_t status;
    twai_get_status_info(&status);
    if (status.msgs_to_tx == 0 && status.tx_error_counter == 0)
    {
        Serial.println("Frame transmitted and acknowledged!");
    }
    else if (status.tx_error_counter > 0)
    {
        Serial.println("Transmission error occurred!");
    }
    else
    {
        Serial.println("Frame still in Tx queue, transmission may have failed!");
    }
    delay(1000);
    //    txFrame.data[0] = 0xBB;
    //    txFrame.data[1] = 0xDD;
    //    txFrame.data[2] = 0xBB;
    //    txFrame.data[3] = 0xDD;
    //    txFrame.data[4] = 0xBB;
    //    txFrame.data[5] = 0xDD;
    //    txFrame.data[6] = 0xBB;
    //    txFrame.data[7] = 0xDD;
    //
    //    if (ESP32Can.writeFrame(&txFrame)) {
    //      Serial.println("Second frame transmitted");
    //    }
    //   //  && status.tx_error_counter == 0){}
}