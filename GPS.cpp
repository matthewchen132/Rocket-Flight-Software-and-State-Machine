#include "GPS.h"

// Connect to the GPS on the hardware port

Adafruit_GPS GPS_1(&GPSSerial);
uint32_t timer = millis();
GPS::GPS()
{
}

void GPS::startGPS()
{
  Serial1.begin(9600, SERIAL_8N1, TX_PIN, RX_PIN);
  GPS_1.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS_1.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS_1.sendCommand(PGCMD_ANTENNA);
  delay(1000);

  GPSSerial.println(PMTK_Q_RELEASE);
}

nmea_float_t getLatitude()
{
  Serial.println("Getting Latitude.");
  char c = GPS_1.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c)
      Serial.print(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS_1.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    // Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    Serial.print(GPS_1.lastNMEA());
    if (!GPS_1.parse(GPS_1.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;                           // we can fail to parse a sentence in which case we should just wait for another
  }

  if (GPS_1.fix)
  {
    return GPS_1.latitude;
  }
  else
  {
    Serial.println("no fix for latitude");
    return -1; // no fix
  }
}

nmea_float_t getLongitude()
{
  Serial.println("Getting Longitude");
  char c = GPS_1.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c)
      Serial.print(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS_1.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    // Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    Serial.print(GPS_1.lastNMEA());
    if (!GPS_1.parse(GPS_1.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;                           // we can fail to parse a sentence in which case we should just wait for another
  }

  if (GPS_1.fix)
  {
    return GPS_1.longitude;
  }
  else
  {
    Serial.println("no fix for longitude");
    return -1; // no fix
  }
}

void GPS::printInfo()
{

  char c = GPS_1.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c)
      Serial.print(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS_1.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    // Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    Serial.print(GPS_1.lastNMEA());
    if (!GPS_1.parse(GPS_1.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;                           // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  // timer = millis(); // reset the timer

  Serial.print("\nTime: ");
  if (GPS_1.hour < 10)
  {
    Serial.print('0');
  }
  Serial.print(GPS_1.hour, DEC);
  Serial.print(':');
  if (GPS_1.minute < 10)
  {
    Serial.print('0');
  }
  Serial.print(GPS_1.minute, DEC);
  Serial.print(':');
  if (GPS_1.seconds < 10)
  {
    Serial.print('0');
  }
  Serial.print(GPS_1.seconds, DEC);
  Serial.print('.');
  if (GPS_1.milliseconds < 10)
  {
    Serial.print("00");
  }
  else if (GPS_1.milliseconds > 9 && GPS_1.milliseconds < 100)
  {
    Serial.print("0");
  }
  Serial.println(GPS_1.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS_1.day, DEC);
  Serial.print('/');
  Serial.print(GPS_1.month, DEC);
  Serial.print("/20");
  Serial.println(GPS_1.year, DEC);
  Serial.print("Fix: ");
  Serial.print((int)GPS_1.fix);
  Serial.print(" quality: ");
  Serial.println((int)GPS_1.fixquality);
  if (GPS_1.fix)
  {
    Serial.print("Location: ");
    Serial.print(GPS_1.latitude, 4);
    Serial.print(GPS_1.lat);
    Serial.print(", ");
    Serial.print(GPS_1.longitude, 4);
    Serial.println(GPS_1.lon);

    Serial.print("Speed (knots): ");
    Serial.println(GPS_1.speed);
    Serial.print("Angle: ");
    Serial.println(GPS_1.angle);
    Serial.print("Altitude: ");
    Serial.println(GPS_1.altitude);
    Serial.print("Satellites: ");
    Serial.println((int)GPS_1.satellites);
    Serial.print("Antenna status: ");
    Serial.println((int)GPS_1.antenna);
  }
  // Serial.println();
}



// write a more comprehensive funciton
bool GPS::readingCheck()
{
  if (GPS_1.altitude == 0)
  {
    return false;
  }
  return true;
}

double GPS::convertToDegrees(int32_t coordinate)
{
  // dd mm.mmmm
  int coordinateVal = int(coordinate);
  if (coordinateVal < 0)
  {
    coordinateVal *= -1;
  }
  return coordinate / pow(10, 7);
}

uint8_t GPS::extraPrecision(double coordinate)
{
  int integerPart = static_cast<int>(coordinate);
  double decimalPart = coordinate - integerPart;
  int scaledDecimal = static_cast<int>(decimalPart * 10000);
  int filteredDecimal = scaledDecimal % 100;
  // if (scaledDecimal > 255)
  // {
  //     scaledDecimal = 255;
  // }
  uint8_t hexValue = 0x00 + static_cast<uint8_t>(filteredDecimal);
  return hexValue;
}

String GPS::representAsCoordinates(uint8_t latitude_integer, uint8_t latitude_decimal, uint8_t latitude_precision, uint8_t longitude_integer, uint8_t longitude_decimal, uint8_t longitude_precision)
{
  double latitudeIntegerResult = static_cast<double>(latitude_integer);

  double latitudeFractionResult = static_cast<double>(latitude_decimal);

  double latitudePrecision = static_cast<double>(latitude_precision);

  double longitudeIntegerResult = static_cast<double>(longitude_integer);

  double longitudeFractionResult = static_cast<double>(longitude_decimal);

  double longitudePrecision = static_cast<double>(longitude_precision);

  return static_cast<String>(latitudeIntegerResult + 0.01 * latitudeFractionResult + 0.0001 * latitudePrecision) + "N, " + static_cast<String>(longitudeIntegerResult + longitudeFractionResult * 0.01 + longitudePrecision * 0.0001) + "W";
}

double GPS::getAltitude()
{
  return GPS_1.altitude;
}