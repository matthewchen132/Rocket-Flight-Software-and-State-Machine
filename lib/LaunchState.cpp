#include "LaunchState.h"
#include <Arduino.h>
#include "SixDOF.h"
#include "PHT.h"
#include "MPU9250.h"
#include "GPS.h"
#include <map>
using namespace std;
#define RATE_THRESHOLD 0.05
#define ERROR_RANGE 25
// values below should be fixed
#define EXPECTED_NETACCEL 100
#define IGNITE_THRESHOLD 15

const double FAIL_TIME = 30000;

const double EXPECTED_ALTITUDE = 1000;
const double EXPECTED_ACCELERATION = 9.8;
LaunchState current_state = LaunchState::PreIgnition;
const int READINGS_LENGTH = 15;
double rate_of_change_1;
double rate_of_change_2;
double AltArray[READINGS_LENGTH];
double AltArray2[READINGS_LENGTH];
double VelArray[READINGS_LENGTH];
double VelArray2[READINGS_LENGTH];
int count = 0;
double previousMedian = EXPECTED_ALTITUDE;

double groundLevelAltitude;
std::map<string, bool> statesMap;

double returnAverage(double arr[], int number)
{
    double sum = 0;
    for (int count = 0; count < number - 1; count++)
    {
        sum += arr[count];
    }
    return sum / number;
}

double returnMedian(double arr[], int number)
{
    double temp[number];
    memcpy(temp, arr, sizeof(temp));
    std::sort(temp, temp + number);

    if (number % 2 == 0)
    {
        return (temp[number / 2 - 1] + temp[number / 2]) / 2.0;
    }
    else
    {
        return temp[number / 2];
    }
}

double calculateRateOfChange(double AltArray[], int READINGS_LENGTH)
{
    double currentMedian = returnMedian(AltArray, READINGS_LENGTH);

    double rate_of_change = currentMedian - previousMedian;

    previousMedian = currentMedian;

    return rate_of_change;
}

bool check_IGNITABLE(SixDOF &_6DOF, PHT &alt, MPU9250 &mpu)
{
    double netAccel = _6DOF.getNetAccel();
    if (fabs(netAccel - EXPECTED_NETACCEL) < IGNITE_THRESHOLD)
    {
    }
    // check mpu against 7 Gs
    // check altitude rate of change against 10 m/s
    return false;
}
// in the main loop, we will get sensor data from 6dof, altimeter, and mpu.
// then we will run HalyaStateMachine, which can determine correct state of the flight and run prioritized code for that state
void HalyaStateMachine(SixDOF &_6DOF1, PHT &alt1, MPU9250 &mpu1, GPS &gps1, SixDOF &_6DOF2, PHT &alt2, MPU9250 &mpu2, GPS &gps2)
{
    static int retryCount = 0;
    const int MAX_RETRIES = 5;

    switch (current_state)
    {
    case LaunchState::PreIgnition:
    {
        while (!_6DOF1.check_IGNITABLE())
        {
            delay(4000); // Long delay to conserve power
        }

        bool is_6DOF_working = _6DOF1.checkReadings();
        bool is_altimeter_working = alt1.getAltitude() != 0;
        bool is_mpu_working = mpu1.update() && mpu1.getAcc(millis()) != 0 & mpu1.getGyro(millis()) != 0;
        bool is_gps_working = gps1.readingCheck();

        if (is_6DOF_working & is_altimeter_working & is_mpu_working & is_gps_working)
        {
            groundLevelAltitude = alt1.getAltitude();
            statesMap["_6DOF"] = true;
            statesMap["altimeter"] = true;
            statesMap["mpu"] = true;
            statesMap["gps"] = true;
        }
        else
        {
            if (!is_6DOF_working)
                Serial.println("Warning: 6DOF sensor failure.");
            if (!is_altimeter_working)
                Serial.println("Warning: Altimeter failure.");
            if (!is_mpu_working)
                Serial.println("Warning: MPU failure.");

            delay(20);
            retryCount++;
            if (retryCount == MAX_RETRIES)
            {
                Serial.print("Continuing with limited functionality.");
                statesMap["_6DOF"] = is_6DOF_working;
                statesMap["altimeter"] = is_altimeter_working;
                statesMap["mpu"] = is_mpu_working;
                statesMap["gps"] = is_gps_working;
            }
        }

        current_state = LaunchState::Ignition_to_Apogee;
        Serial.println("halya ignition!");
        break;
    }

    case LaunchState::Ignition_to_Apogee:
    {
        // iterative count
        static int count = 0;
        // error flags for each of the sensors
        bool PHT1_error = (alt1.getAltitude() == 0);
        bool PHT2_error = (alt2.getAltitude() == 0);
        bool GPS1_error = gps1.readingCheck();
        bool GPS2_error = gps2.readingCheck();
        bool IMU1_error = !_6DOF1.checkReadings();
        bool IMU2_error = !_6DOF2.checkReadings();

        double altReading1 = 0, altReading2 = 0;
        bool usePHT1 = true, usePHT2 = true;

        // Check if both PHTs are working and if they agree
        if (!PHT1_error && !PHT2_error)
        {
            if (fabs(alt1.getAltitude() - alt2.getAltitude()) < ERROR_RANGE)
            {
                altReading1 = alt1.getAltitude();
                altReading2 = alt2.getAltitude();
            }
            // if they do not agree we check with the GPS
            else
            {
                // check whether GPS has errors or not and then compare values accordingly
                double gpsAvgAltitude = 0;
                if (!GPS1_error || !GPS2_error)
                {
                    gpsAvgAltitude = (GPS1_error) ? gps2.getAltitude() : (GPS2_error) ? gps1.getAltitude()
                                                                                      : (gps1.getAltitude() + gps2.getAltitude()) / 2.0;

                    // based on the GPS reading, we make sure to see which PHT it agrees with
                    // POTENTIAL ERROR CASE: if the GPS altitude is equally distant from both points or within error range of both,
                    // maybe we should take the average of both points
                    if (fabs(alt1.getAltitude() - gpsAvgAltitude) < ERROR_RANGE)
                    {
                        altReading1 = alt1.getAltitude();
                        usePHT2 = false;
                    }
                    else if (fabs(alt2.getAltitude() - gpsAvgAltitude) < ERROR_RANGE)
                    {
                        altReading2 = alt2.getAltitude();
                        usePHT1 = false;
                    }
                }
                // check if IMUs work or not
                else if (!IMU1_error || !IMU2_error)
                {
                    // If GPS is not available, fall back to IMU to verify PHT
                    double imuAvgAltitude = (!IMU1_error && !IMU2_error) ? (_6DOF1.updateVerticalAltitude() + _6DOF2.updateVerticalAltitude()) / 2.0 : (!IMU1_error) ? _6DOF1.updateVerticalAltitude()
                                                                                                                                                                     : _6DOF2.updateVerticalAltitude();
                    // check if within error range
                    // same POTENTIAL ERROR CASE
                    if (fabs(alt1.getAltitude() - imuAvgAltitude) < ERROR_RANGE)
                    {
                        altReading1 = alt1.getAltitude();
                        usePHT2 = false;
                    }
                    else if (fabs(alt2.getAltitude() - imuAvgAltitude) < ERROR_RANGE)
                    {
                        altReading2 = alt2.getAltitude();
                        usePHT1 = false;
                    }
                }
            }
        }

        // check if we only have to use PHT 1
        if (!PHT1_error && usePHT1 && (PHT2_error || !usePHT2))
        {
            altReading1 = alt1.getAltitude();
            altReading2 = altReading1;
        }
        // check if we only have to use PHT 2
        else if (!PHT2_error && usePHT2 && (PHT1_error || !usePHT1))
        {
            altReading1 = alt2.getAltitude();
            altReading2 = altReading1;
        }

        // Step 3: If both PHTs are invalid, fall back to GPS
        if ((PHT1_error || !usePHT1) && (PHT2_error || !usePHT2))
        {
            if (!GPS1_error || !GPS2_error)
            {
                altReading1 = (!GPS1_error && !GPS2_error) ? (gps1.getAltitude() + gps2.getAltitude()) / 2.0 : (!GPS1_error) ? gps1.getAltitude()
                                                                                                                             : gps2.getAltitude();
                altReading2 = altReading1;
            }
            else if (!IMU1_error || !IMU2_error)
            {
                // Step 4: Check individual IMUs if GPS fails
                altReading1 = (!IMU1_error && !IMU2_error) ? (_6DOF1.updateVerticalAltitude() + _6DOF2.updateVerticalAltitude()) / 2.0 : (!IMU1_error) ? _6DOF1.updateVerticalAltitude()
                                                                                                                                                       : _6DOF2.updateVerticalAltitude();
                altReading2 = altReading1;
            }
        }

        AltArray[count % READINGS_LENGTH] = altReading1;
        AltArray2[count % READINGS_LENGTH] = altReading2;
        count++;

        // Step 5: Check for apogee based on calculated rate of change
        if (count >= READINGS_LENGTH)
        {
            double rate_of_change_1 = calculateRateOfChange(AltArray, READINGS_LENGTH);
            double rate_of_change_2 = calculateRateOfChange(AltArray2, READINGS_LENGTH);

            if ((fabs(rate_of_change_1) < RATE_THRESHOLD && rate_of_change_1 < 0) ||
                (fabs(rate_of_change_2) < RATE_THRESHOLD && rate_of_change_2 < 0))
            {
                Serial.println("Halya has reached apogee!");
                current_state = LaunchState::Thousand_ft;
            }
            count = 0;
        }
        delay(100); // Delay for sensor updates
        break;
    }

    case LaunchState::Thousand_ft:
    {
        bool PHT1_error = (alt1.getAltitude() == 0);
        bool PHT2_error = (alt2.getAltitude() == 0);
        bool GPS1_error = gps1.readingCheck();
        bool GPS2_error = gps2.readingCheck();
        static unsigned long startTime = millis(); // Track the start time of this state

        double altitudeReading = 0;
        bool usePHT1 = !PHT1_error;
        bool usePHT2 = !PHT2_error;

        // Step 1: Check if either PHT is working and use the lowest working altitude
        if (!PHT1_error && !PHT2_error)
        {
            altitudeReading = min(alt1.getAltitude(), alt2.getAltitude());
        }
        else if (usePHT1)
        {
            altitudeReading = alt1.getAltitude();
        }
        else if (usePHT2)
        {
            altitudeReading = alt2.getAltitude();
        }
        // Step 2: If both PHTs fail, use GPS as fallback
        else if (!GPS1_error && !GPS2_error)
        {
            altitudeReading = min(gps1.getAltitude(), gps2.getAltitude());
        }
        else if (!GPS1_error)
        {
            altitudeReading = gps1.getAltitude();
        }
        else if (!GPS2_error)
        {
            altitudeReading = gps2.getAltitude();
        }
        // Step 3: If no sensor data is valid, fall back to timed deployment after 30 seconds
        else if (millis() - startTime >= FAIL_TIME)
        {
            Serial.println("No altitude data - deploying parachute based on timeout.");
            // deployMainParachute();
            current_state = LaunchState::Descent;
            break;
        }

        // Step 4: Deploy parachute based on altitude thresholds
        if (altitudeReading <= 1750)
        {
            Serial.println("Reached 1750 meters - preparing for parachute deployment.");
            // deploy parachute
            current_state = LaunchState::Descent;
        }
        else if (altitudeReading <= 1500)
        {
            Serial.println("Reached 1500 meters - deploying parachute.");
            // deploy parachute
            current_state = LaunchState::Descent;
        }
        else if (altitudeReading <= 1250)
        {
            Serial.println("Reached 1250 meters - final check for parachute deployment.");
            // deploy parachute
            current_state = LaunchState::Descent;
        }

        break;
    }

    case LaunchState::Descent:
    {
        current_state = LaunchState::Touchdown;
        break;
        // send GPS data
    }

    case LaunchState::Touchdown:
    {
        // Print GPS readings if required
        // send GPS data
        break;
    }
    }
}
