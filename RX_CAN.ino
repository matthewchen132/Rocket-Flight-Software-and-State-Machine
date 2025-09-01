#include <ESP32-TWAI-CAN.hpp>
#include <stdlib.h>
#include <math.h>

// Define CAN pins (ESP32 DevKit)
#define CAN_TX 16
#define CAN_RX 17

// CAN bus speed (adjust as needed)
const int CAN_SPEED = 500; // 500 kbps

CanFrame rxFrame;

double hextoDecimal(uint8_t hexadecimal_int, uint8_t hexadecimal_fraction)
{

    double integerResult = static_cast<double>(hexadecimal_int);

    double fractionResult = static_cast<double>(hexadecimal_fraction);

    return integerResult + fractionResult * (0.01);
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
void setup()
{
    Serial.begin(115200);

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
    // Initialize CAN bus at specified speed and define TX/RX pins
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
}

void loop()
{
    if (ESP32Can.readFrame(rxFrame, 1000))
    { // Wait for frame with 10ms timeout
        printTwaiStatus();
        Serial.printf("Received frame: ID=0x%03X, Length=%d\n", rxFrame.identifier, rxFrame.data_length_code);
        // Process received data
        for (int i = 0; i < rxFrame.data_length_code; i += 2)
        {
            Serial.printf("Data[%d]:", i, rxFrame.data[i]);
            Serial.println(hextoDecimal(rxFrame.data[i], rxFrame.data[i + 1]));
        }
    }
    else
    {
        printTwaiStatus();
        Serial.println("No Frame read yet");
    }
}
