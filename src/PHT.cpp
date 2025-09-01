#include "PHT.h"
#define SEA_LEVEL_PRESSURE_PA 101325.0

// Remove this constructor definition if it is already defined in the header file or elsewhere.
bool PHT::connectSensor()
{
  isConnected = (barometer.connect() == 0);
  return isConnected;
}

void PHT::setSensorConfig()
{
  if (isConnected)
  {
    barometer.setSamples(MS5xxx_CMD_ADC_4096);
    // barometer.setDelay(1000);
    barometer.setTempC();
    barometer.setPressPa();
    barometer.setTOffset(-200);
    barometer.setPOffset(5);
    seaLevelPressure = barometer.getSeaLevel(112.776);
  }
}

void PHT::updateData()
{
  if (isConnected)
  {
    barometer.checkUpdates();
    if (barometer.isReady())
    {
      temperature = barometer.GetTemp();
      pressure = barometer.GetPres();
      altitude = barometer.getAltitude(false);
      if (seaLevelPressure == 0)
        seaLevelPressure = barometer.getSeaLevel(112.776);
    }
  }
}
double PHT::getPressTempRTOS(){
  barometer.checkUpdates(); // 8-8-25 Matthew Addition
}

void PHT::printData()
{
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" meters");
  Serial.print("Sea Level Pressure: ");
  Serial.print(seaLevelPressure);
  Serial.println(" Pa");
}

double PHT::getPressure()
{
  barometer.checkUpdates();
  return barometer.GetPres();
}
double PHT::getTemperature()
{
  barometer.checkUpdates();
  return barometer.GetTemp();
}
double PHT::getAltitude()
{
  return 44330.0 * (1.0 - pow(getPressure() / SEA_LEVEL_PRESSURE_PA, 0.1903));
}
double PHT::getSeaLevelPressure() { return seaLevelPressure; }









double PHT::getKalmanFilteredAltitude(float measurementNoise, float processNoise, float estimate, float error){
// This is goated documentation https://www.kalmanfilter.net/alphabeta.html
  R = measurementNoise;
  Q = processNoise; // 30.87^2 meters? mach 1.8 = 617.4 m/s, with estimated 20 samples per second (due to change), 30.87^2
  x = estimate;
  P = error;  
}

double PHT::update_1D(float Z, float Q, float R,  float time_elapsed){
    // Q how much your state might drift between measurements
    /* R is MEASUREMENT variance, think "how much could my altitude change in the next time step while holding still"? measures in units of variance (state units)^2
        Say that we know altitude changes +- 1.2 meters per reading. variance = 1.2^2

        In future, Find Q through sampling Q 1000 times, calculating mean, and finding average distance of each point from the mean. 
          numerator += (x_mean - x_n)**2
          std_dev_x_n = sqrt((numerator)/(1000))
    */
    // Z is the MEASUREMENT
    // X is the PREDICTION, found through PHT.getAltitude()
    // P is ESTIMATE variance

    // finds initial x value
    if(!hasInitialValue){
      x = Z; // Z is the measurement we got from getAltitude() in the main function
      P = 5 * 1.44; // set the Estimate variance to be 5 * the Measurement vairance, low trust in the original readings.
      hasInitialValue = true;
    }

    P = P + Q;
    // compute the Kalman gain (K), 
    float K = P / (P+R);
    // compute the estimate of altitude by adding previous height + kalman_gain*(difference between measurement and previous)
    x = x + K*(Z - x);
    return x;
}

double PHT::update_2D(){
  // todo 
}