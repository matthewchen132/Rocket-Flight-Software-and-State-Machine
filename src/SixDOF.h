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

private:
  float quaternion[4];
  bool _init(int32_t sensor_id);
};

#endif
