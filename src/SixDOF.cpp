// Basic demo for accelerometer & gyro readings from Adafruit
// LSM6DSO32 sensor
#include "SixDOF.h"
#include <Adafruit_LSM6DSO32.h>
#include "KalmanFilter.h"
#include <SimpleKalmanFilter.h>
// For SPI mode, we need a CS pin
#define LSM_CS 39
#define LSM_SCK 37
#define LSM_SDO 35
#define LSM_SDA 36

bool IGNITABLE = false;
double Net_Accel;
const double GRAVITY = 9.81;
double deltaTime = 0.1; // adjust based on delay

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

double pitch = 0.0;
double roll = 0.0;
double yaw = 0.0;
unsigned long lastUpdateTime = 0;

Adafruit_LSM6DSO32 dso32;
SimpleKalmanFilter accelXFilter(1, 1, 0.1);
SimpleKalmanFilter accelYFilter(1, 1, 0.1);
SimpleKalmanFilter accelZFilter(1, 1, 0.1);

SixDOF::SixDOF()
{
}

// Function definition
bool SixDOF::start_6DOF()
{
  Serial.println("Adafruit LSM6DSOX test!");
  if (!dso32.begin_SPI(LSM_CS, LSM_SCK, LSM_SDO, LSM_SDA))
  {
    Serial.println("Failed to find LSM6DSOX chip");
    return (false);
  }
  Serial.println("LSM6DSOX Found!");
  return true;
}

// Optional function definition (if needed in SixDOF.h)
String SixDOF::printSensorData()
{
  sensors_event_t accel1;
  sensors_event_t gyro1;
  sensors_event_t temp1;
  dso32.getEvent(&accel1, &gyro1, &temp1);
  double Ax = accel1.acceleration.x;
  double Ay = accel1.acceleration.y;
  double Az = accel1.acceleration.z;
  Net_Accel = sqrt((pow(Ax, 2) + pow(Ay, 2) + pow(Az, 2)));
  return "6DOF Ax" + String(Ax) + ", " + "Ay: " + String(Ay) + ", " + "Az: " + String(Az) + "," + String(gyro1.gyro.x) + "," + String(gyro1.gyro.y) + "," + String(gyro1.gyro.z) + "," + "\n";
}

vector<double> SixDOF::getAcceleration()
{
  sensors_event_t accel1;
  sensors_event_t gyro1;
  sensors_event_t temp1;
  dso32.getEvent(&accel1, &gyro1, &temp1);
  return {accel1.acceleration.x, accel1.acceleration.y, accel1.acceleration.z};
}

double SixDOF::updateVerticalVelocity()
{
  std::vector<double> accelData = getAcceleration();
  double accelZ = accelData[2];

  vertical_velocity += accelZ * deltaTime;

  // if (vertical_velocity < 0 && previous_vertical_velocity >= 0 && !apogeeReached)
  // {
  //   apogeeReached = true;
  //   Serial.println("Apogee reached!");
  // }

  previous_vertical_velocity = vertical_velocity;

  Serial.print("Vertical Velocity: ");
  Serial.print(vertical_velocity);
  Serial.println(" m/s");

  return vertical_velocity;
}

double SixDOF::updateVerticalAltitude()
{
  // Update vertical position based on the current velocity
  vertical_position += vertical_velocity * deltaTime;

  previous_altitude = vertical_position;

  Serial.print("Altitude: ");
  Serial.print(vertical_position);
  Serial.println(" m");

  return vertical_position;
}

vector<double> SixDOF::getGyro()
{
  sensors_event_t accel1;
  sensors_event_t gyro1;
  sensors_event_t temp1;
  dso32.getEvent(&accel1, &gyro1, &temp1);
  return {static_cast<double>(gyro1.gyro.x), static_cast<double>(gyro1.gyro.y), static_cast<double>(gyro1.gyro.z)};
}

bool SixDOF::_init(int32_t sensor_id)
{
  return true; // Example return value, modify as needed
}

bool SixDOF::check_IGNITABLE()
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
bool SixDOF::checkReadings()
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
double predict_alt(double measurement_noise, double z_accel, double curr_alt, double poll_rate){
  double predicted_alt = curr_alt + z_accel*poll_rate;
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

// vector<double> SixDOF::quaternionToEuler()
// {
//   float w = quaternion[0];
//   float x = quaternion[1];
//   float y = quaternion[2];
//   float z = quaternion[3];
//   float roll = atan2(2 * (x * w + y * z), 1 - 2 * (x * x + y * y));
//   float pitch = asin(2 * (y * w - z * x));
//   float yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
//   return {static_cast<double>(roll), static_cast<double>(pitch), static_cast<double>(yaw)};
// }

void SixDOF::updateVelocities()
{
  // Get acceleration data
  std::vector<double> accelData = getFilteredAccelerations();

  double accelX = accelData[0];
  double accelY = accelData[1];
  double accelZ = accelData[2];

  velocity_x += accelX * deltaTime;
  velocity_y += accelY * deltaTime;
  velocity_z += accelZ * deltaTime;

  Serial.print("Velocity X: ");
  Serial.print(velocity_x);
  Serial.print(" m/s, Velocity Y: ");
  Serial.print(velocity_y);
  Serial.print(" m/s, Velocity Z: ");
  Serial.print(velocity_z);
  Serial.println(" m/s");
}

void SixDOF::updatePositions()
{
  position_x += velocity_x * deltaTime;
  position_y += velocity_y * deltaTime;
  position_z += velocity_z * deltaTime;

  Serial.print("Position X: ");
  Serial.print(position_x);
  Serial.print(" m, Position Y: ");
  Serial.print(position_y);
  Serial.print(" m, Position Z: ");
  Serial.print(position_z);
  Serial.println(" m");
}

vector<double> SixDOF::getPositions()
{
  return {position_x, position_y, position_z};
}

vector<double> SixDOF::getVelocities()
{
  return {velocity_x, velocity_y, velocity_z};
}

double SixDOF::getNetAccel()
{
  sensors_event_t accel1;
  sensors_event_t gyro1;
  sensors_event_t temp1;
  dso32.getEvent(&accel1, &gyro1, &temp1);
  double Ax = accel1.acceleration.x;
  double Ay = accel1.acceleration.y;
  double Az = accel1.acceleration.z;
  Net_Accel = sqrt((pow(Ax, 2) + pow(Ay, 2) + pow(Az, 2)));
  return(Net_Accel);
}

vector<double> SixDOF::getFilteredAccelerations()
{
  sensors_event_t accel1;
  sensors_event_t gyro1;
  sensors_event_t temp1;
  dso32.getEvent(&accel1, &gyro1, &temp1);
  return {accelXFilter.updateEstimate(accel1.acceleration.x), accelYFilter.updateEstimate(accel1.acceleration.y), accelZFilter.updateEstimate(accel1.acceleration.z)};
}