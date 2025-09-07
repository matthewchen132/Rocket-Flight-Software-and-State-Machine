#ifndef _SixDOF_H
#define _SixDOF_H

#include "Arduino.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_Sensor.h>
#include <vector>

using namespace std;

// extern Adafruit_LSM6DSO32 dso32;

class SixDOF : public Adafruit_LSM6DSOX
{
public:
  SixDOF();
  bool start_6DOF();
  String printSensorData();
  double Net_Accel;
  bool IGNITABLE;
  bool check_IGNITABLE();
  vector<double> getAcceleration();
  vector<double> getGyro();
  double getNetAccel();
  double predict_alt(double measurement_noise, double z_accel, double curr_alt);
  void updateQuaternionFilter();
  vector<double> quaternionToEuler();
  bool checkReadings();
  double updateVerticalVelocity();
  double updateVerticalAltitude();
  void updateVelocities();
  void updatePositions();
  vector<double> getVelocities();
  vector<double> getPositions();
  // double getDirection();
  vector<double> getFilteredAccelerations();
  double R = 0.148251666;
  double azw_variance = 10*8.03006E-05; // variance of the z accel in world frame, found from exp. data 
  double bias_variance = .00001; //initial estimate, increase to stop velocity drift.
  int read_delay = 10;

private:
  float quaternion[4];
  bool _init(int32_t sensor_id);
};

#endif
