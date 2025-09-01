#ifndef LaunchState_h
#include <Arduino.h>
enum class LaunchState{
    PreIgnition,
    Ignition_to_Apogee,
    Thousand_ft,
    Descent,
    Touchdown,
};
#endif