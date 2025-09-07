#include "PHT.h"
#define SEA_LEVEL_PRESSURE_PA 101325.0
constexpr float R = 0.06139392;//Measurement Noise (R).
constexpr float dt = 10.0; //sample rate in ms.
constexpr float F[2][2] = {
                           {1, dt}, 
                           {0, 1}     
                          };
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
  return 0.0; // wip
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

double PHT::update_1D(float process_noise_Q, float accel_z, float dt_ms){
  //Sensor Noise is already found through idle height variance^2 (R = 0.06139392)
  double sigma_a = accel_z * accel_z;
}

double PHT::update_2D(){
  // todo 
}