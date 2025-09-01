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
        R = 0.031754118; // std. deviation ~ 0.178 <- Measurement deviations found through recording static points
        P = 10*R; // I don't really trust the initial measurement, make the variance at 5 x the known sensor variance
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
    double update_1D(float measurement, float Q, float R, float time_elapsed); // <- Stationary
    double update_2D(); // <- Moving
    double hasInitialValue = false;
    double x = 0.00;
    double R = 0.031754118;
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