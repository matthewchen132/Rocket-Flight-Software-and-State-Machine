#ifndef LaunchState_h
#include <Arduino.h>

enum class LaunchState{
    PreIgnition,
    Ignition_to_Apogee,
    _1000ft,
    _900ft,
    _800ft,
    Descent,
    Touchdown,
};
#endif