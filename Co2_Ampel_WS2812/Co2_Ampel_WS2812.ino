/*
  Reading CO2, humidity and temperature from the SCD30 + BME680
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT.
  Open Serial Monitor at 115200 baud.
*/


 // BME680 Air Quality: https://randomnerdtutorials.com/esp32-bme680-sensor-arduino/
 
 
#include <Wire.h>
#include "Zanshin_BME680.h"  // Include the BME680 Sensor library
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include "Freenove_WS2812_Lib_for_ESP32.h"

#define LEDS_COUNT  8
#define LEDS_PIN  12
#define CHANNEL   0

const uint32_t SERIAL_SPEED = 115200;  ///< Set the baud rate for Serial I/O
unsigned long messurementCounter = 0;
int32_t SCD30_Co2, SCD30_Humidity, SCD30_Temperature;
int32_t BME680_Temperature, BME680_Humidity, BME680_Pressure, BME680_Gas;  // BME readings
float temp_altitude;

Freenove_ESP32_WS2812 ledsOut = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, CHANNEL, TYPE_GRB);
SCD30 SCD30_AirSensor;
BME680_Class BME680; 

float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  /*!
  @brief     This converts a pressure measurement into a height in meters
  @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
  @param[in] press    Pressure reading from BME680
  @param[in] seaLevel Sea-Level pressure in millibars
  @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()


void setup()
{
  Serial.begin(SERIAL_SPEED);
  Serial.println("CO2 AMPEL with ESP32 + BME680 + SCD30");
  ledsOut.begin();
  Wire.begin();


  if (SCD30_AirSensor.begin() == false)
  {
    Serial.println("SCD30 Air sensor not detected.");
    delay(5000);
  }

  while (!BME680.begin(I2C_STANDARD_MODE))  { // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds
  delay(2000);
}

void loop()
{
  messurementCounter++;
  Serial.println("");
  Serial.print("Start messurement #");
  Serial.println(messurementCounter);
  Serial.println();
  
  if (SCD30_AirSensor.dataAvailable())
  {
    SCD30_Co2 = SCD30_AirSensor.getCO2();
    SCD30_Temperature = SCD30_AirSensor.getTemperature();
    SCD30_Humidity = SCD30_AirSensor.getHumidity();
    
    Serial.print("SCD30: co2(ppm): ");
    Serial.println(SCD30_Co2);
    
    Serial.print("SCD30: temp(C):  ");
    Serial.println(SCD30_Temperature);

    Serial.print("SCD30: Humidity(%): ");
    Serial.println(SCD30_Humidity, 1);
  }
  else
    Serial.println("SCD30 w/o data..");  //The SCD30 has data ready every two seconds
    

 
  static char     buf[16];                        // sprintf text buffer
 // static float    alt;                            // Temporary variable
 
  BME680.getSensorData(BME680_Temperature, BME680_Humidity, BME680_Pressure, BME680_Gas);   // Get readings
  temp_altitude = altitude(BME680_Pressure); 
  
  Serial.print("BME680: temp(C):");
  sprintf(buf, "%3d.%02d", (int8_t)(BME680_Temperature / 100), (uint8_t)(BME680_Temperature % 100));  // Temp in decidegrees
  Serial.println(buf);

  Serial.print("BME680: Humidity(%):");
  sprintf(buf, "%3d.%03d", (int8_t)(BME680_Humidity / 1000),
          (uint16_t)(BME680_Humidity % 1000));  // Humidity milli-pct
  Serial.println(buf);

  Serial.print("BME680: Pressure(hPa): ");
  sprintf(buf, "%4d.%02d", (int16_t)(BME680_Pressure / 100),
          (uint8_t)(BME680_Pressure % 100));  // Pressure Pascals
  Serial.println(buf);
  
  Serial.print("BME680: Pressure to Altitude(m): ");                                              
  sprintf(buf, "%5d.%02d", (int16_t)(temp_altitude), ((uint8_t)(temp_altitude * 100) % 100));  // Altitude meters
  Serial.println(buf);
  
  Serial.print("BME680: AirQuality:");
  sprintf(buf, "%4d.%02d\n", (int16_t)(BME680_Gas / 100), (uint8_t)(BME680_Gas % 100));  // Resistance milliohms
  Serial.println(buf);

  for (int j = 0; j < 255; j += 2) {
    for (int i = 0; i < LEDS_COUNT; i++) {
      ledsOut.setLedColorData(i, ledsOut.Wheel((i * 256 / LEDS_COUNT + j) & 255));
    }
    ledsOut.show();
    delay(5);
  }  
  
  delay(5000);
}
