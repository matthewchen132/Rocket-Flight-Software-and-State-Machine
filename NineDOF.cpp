#include <Adafruit_ICM20948.h>
#include "NineDOF.h"

#define LSM_CS 38
#define LSM_SCK 37
#define LSM_MISO 36
#define LSM_MOSI 35

Adafruit_ICM20948 icm;

bool IGNITABLE = false;
double Net_Accel;
const double GRAVITY = 9.81;
double deltaTime = 0.01;

bool apogeeReached = false;
double vertical_velocity = 0.0;
double vertical_position = 0.0;
double previous_vertical_velocity = 0.0;
double previous_altitude = 0.0;

double velocity_x = 0.0;
double velocity_y = 0.0;
double velocity_z = 0.0;

double position_x = 0.0;
double position_y = 0.0;
double position_z = 0.0;

NineDOF::NineDOF()
{
}

Adafruit_ICM20948 icm = Adafruit_ICM20948();

// Function definition
bool NineDOF::start_9DOF()
{
    Serial.println("Adafruit ICM20948 test!");
    if (!icm.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI))
    {
        Serial.println("Failed to find ICM20948 chip");
        return (false);
    }
    Serial.println("ICM20948 Found!");
    return true;
}

// Optional function definition (if needed in SixDOF.h)
String NineDOF::printSensorData()
{
    sensors_event_t accel1;
    sensors_event_t gyro1;
    sensors_event_t temp1;
    sensors_event_t mag1;
    icm.getEvent(&accel1, &gyro1, &temp1, &mag1);
    double Ax = accel1.acceleration.x;
    double Ay = accel1.acceleration.y;
    double Az = accel1.acceleration.z;
    Net_Accel = sqrt((pow(Ax, 2) + pow(Ay, 2) + pow(Az, 2)));
    return "(9DOF)" + String(Ax) + "," + String(Ay) + "," + String(Az) + "," + String(gyro1.gyro.x) + "," + String(gyro1.gyro.y) + "," + String(gyro1.gyro.z) + "," + String(mag1.magnetic.x) + "," + String(mag1.magnetic.y) + "," + String(mag1.magnetic.z) + "\n";
}

vector<double> NineDOF::getAcceleration()
{
    sensors_event_t accel1;
    sensors_event_t gyro1;
    sensors_event_t temp1;
    sensors_event_t mag1;
    icm.getEvent(&accel1, &gyro1, &temp1, &mag1);
    return {accel1.acceleration.x, accel1.acceleration.y, accel1.acceleration.z};
}
vector<double> NineDOF::getMag()
{
    sensors_event_t accel1;
    sensors_event_t gyro1;
    sensors_event_t temp1;
    sensors_event_t mag1;
    icm.getEvent(&accel1, &gyro1, &temp1, &mag1);
    return {mag1.magnetic.x, mag1.magnetic.y, mag1.magnetic.z};
}

vector<double> NineDOF::getGyro()
{
    sensors_event_t gyro1;
    return {static_cast<double>(gyro1.gyro.x), static_cast<double>(gyro1.gyro.y), static_cast<double>(gyro1.gyro.z)};
}

bool NineDOF::_init(int32_t sensor_id)
{
    return true; // Example return value, modify as needed
}

bool NineDOF::check_IGNITABLE()
{ // is there a way to use switch-cases here? Idk how to make cases for all values >10
    Serial.print("    Net Acceleration: " + String(Net_Accel) + ", ");
    if (Net_Accel > 68.6)
    {
        IGNITABLE = true;
    }
    else
    {
        switch (int(Net_Accel))
        {
        default:
            return IGNITABLE;
            break;
        }
    }
    return IGNITABLE;
}
bool NineDOF::checkReadings()
{
    vector<double> accelData = getAcceleration();
    vector<double> gyroData = getGyro();
    for (int i = 0; i < accelData.size(); i++)
    {
        if (accelData[i] != 0)
        {
            return true;
        }
    }
    for (int i = 0; i < gyroData.size(); i++)
    {
        if (gyroData[i] != 0)
        {
            return true;
        }
    }
    return false;
}

// need to call manually in main.cpp file
// void SixDOF::updateQuaternionFilter()
// {
//   vector<double> accelData = getAcceleration();
//   vector<double> gyroData = getGyro();

//   double gyroX = gyroData[0];
//   double gyroY = gyroData[1];
//   double gyroZ = gyroData[2];

//   double accelX = accelData[0];
//   double accelY = accelData[1];
//   double accelZ = accelData[2];

//   filter.update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, 0, 0, 0, quaternion);
// }



vector<double> NineDOF::getPositions()
{
    return {position_x, position_y, position_z};
}

vector<double> NineDOF::getVelocities()
{
    return {velocity_x, velocity_y, velocity_z};
}
