// PHT.h
#ifndef PHT_H
#define PHT_H

#include <Wire.h>
#include <MS5x_halya.h>
#include <iostream>

class PHT
{
public:
    PHT(TwoWire &i2c): i2cBus(i2c), barometer(&i2c){
        std::cout << "Used Default Constructor";
    };
    bool connectSensor();
    void setSensorConfig();
    void updateData();
    void printData();
    double getPressTempRTOS();
    double getPressure();
    double getTemperature();
    double getAltitude();
    double getSeaLevelPressure();
    double getKalmanFilteredAltitude(float measurementNoise, float processNoise, float estimate, float error);
    double update_1D(float process_noise_Q, float accel_z, float dt_ms); // <- Stationary
    double update_2D(); // <- Moving
    double hasInitialValue = false;
    double R = 0.06139392;//Measurement Noise (R).
    double dt = 10.0; //10ms delay
    double x = 0.0;
    double P = 10*R; // I don't really trust the initial measurement, make the estimate variance at 5 x the known sensor variance 

private:
    TwoWire &i2cBus;
    MS5x barometer;
    bool isConnected;
    double pressure;
    double temperature;
    double altitude;
    double seaLevelPressure;
    double Q;
};

#endif // PHT_H