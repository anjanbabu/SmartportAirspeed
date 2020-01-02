//MS4525DO
#include <Wire.h>   //I2C library 0x28H
byte fetch_pressure(unsigned int *p_Pressure); //convert value to byte data type
//Smoothing
const int numReadings = 20;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average
#define TRUE 1
#define FALSE 0


#include "FrSkySportSensor.h"
#include "FrSkySportSensorAss.h"
#include "FrSkySportTelemetry.h"
#if !defined(TEENSY_HW)
#include "SoftwareSerial.h"
#endif

FrSkySportSensorAss ass; // Create ASS sensor with default ID
#ifdef POLLING_ENABLED
FrSkySportTelemetry telemetry(true); // Create telemetry object with polling
#else
FrSkySportTelemetry telemetry; // Create telemetry object without polling
#endif

void setup()
{
  //MS4525DO
  Wire.begin();
  //delay(500);
  //Smoothing
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // Configure the telemetry serial port and sensors (remember to use & to specify a pointer to sensor)
#if defined(TEENSY_HW)
  telemetry.begin(FrSkySportSingleWireSerial::SERIAL_3, &ass);
#else
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_12, &ass);
#endif
}

void loop()
{
  MS4525DO ();
  // Set airspeed sensor (ASS) data
  ass.setData(average);  // Airspeed in km/h

#ifdef POLLING_ENABLED
  // Set receiver data to be sent in case the polling is enabled (so no actual receiver is used)
  telemetry.setData(90,    // RSSI value (0-100, 0 = no telemetry, 100 = full signal)
                    4.9);  // RxBatt (voltage supplied to the receiver) value in volts (0.0-13.2)
#endif

  // Send the telemetry data, note that the data will only be sent for sensors
  // that are being polled at given moment
  telemetry.send();
}

byte MS4525DO () {
  byte _status;
  unsigned int P_dat = 0;
  unsigned int T_dat;
  double PR;
  double V;
  double VV;
  _status = fetch_pressure(&P_dat);
  PR = (double)((P_dat - 819.15) / (14744.7)) ; //((P_dat - 819.15) / (14744.7));
  PR = (PR - 0.49060678) ; //(PR - 0.49060678)
  PR = abs(PR);
  V = ((PR * 13789.5144) / 1.225); //((PR * 13789.5144) / 1.225);
  VV = (sqrt((V)));
  //Smoothing
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = VV;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  //Serial.print("speed m/s :");
  //Serial.println(average);
}

byte fetch_pressure(unsigned int *p_P_dat) {
  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
  address = 0x28;
  Wire.beginTransmission(address);
  Wire.endTransmission();
  //delay(100);
  Wire.requestFrom((int)address, (int) 4);//Request 4 bytes need 4 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
  Wire.endTransmission();
  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;
  *p_P_dat = P_dat;
}
