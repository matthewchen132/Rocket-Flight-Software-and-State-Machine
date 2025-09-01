#ifndef _GPS_H
#include <Adafruit_GPS.h>

#define GPSECHO true
#define TX_PIN 20 // connect RX on gps
#define RX_PIN 19 // connect TX on gps
#define GPSSerial Serial1
class GPS : public Adafruit_GPS
{
public:
  GPS();
  void startGPS();
  void printInfo();
  bool readingCheck();
  double convertToDegrees(int32_t coordinate);
  uint8_t extraPrecision(double coordinate);
  String representAsCoordinates(uint8_t latitude_integer, uint8_t latitude_decimal, uint8_t latitude_precision, uint8_t longitude_integer, uint8_t longitude_decimal, uint8_t longitude_precision);
  double getAltitude();
};
#endif
