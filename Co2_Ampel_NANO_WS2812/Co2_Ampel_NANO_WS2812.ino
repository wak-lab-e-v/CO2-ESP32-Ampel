/*
  Reading CO2, humidity and temperature from the SCD30 + BME680
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT.
  Open Serial Monitor at 115200 baud.
*/
// #define BME680
 #define U8DISPLAY

 // BME680 Air Quality: https://randomnerdtutorials.com/esp32-bme680-sensor-arduino/
 
 
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include <NeoPixelBus.h>
#ifdef BME680
  #include "Zanshin_BME680.h"  // Include the BME680 Sensor library
#endif
#ifdef U8DISPLAY
  #include <U8g2lib.h>
#endif
const uint16_t PixelCount = 7; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 2;  // make sure to set this to the correct pin, ignored for Esp8266

#define colorSaturation 32
#define RotSchwelle 2000
#define GruenSchwelle 1000
#define OrangeSchwelle  ((RotSchwelle-GruenSchwelle)/2+GruenSchwelle)


const uint32_t SERIAL_SPEED = 115200;  ///< Set the baud rate for Serial I/O
unsigned long messurementCounter = 0;
int32_t SCD30_Co2, SCD30_Humidity, SCD30_Temperature;
#ifdef BME680
  int32_t BME680_Temperature, BME680_Humidity, BME680_Pressure, BME680_Gas;  // BME readings
#endif
#ifdef U8DISPLAY
  U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);
#endif
float temp_altitude;
int ErrorCountdown;



SCD30 SCD30_AirSensor;
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);


RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor orange(colorSaturation, colorSaturation*165/255, 0);
RgbColor white(colorSaturation);
RgbColor black(0);

HslColor hslRed(red);
HslColor hslGreen(green);
HslColor hslBlue(blue);
HslColor hslWhite(white);
HslColor hslBlack(black);

HslColor fHSL;
RgbColor fRGB;
float    Ampel;

  


#ifdef BME680
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
#endif

#ifdef U8DISPLAY
void HandleDisplay()
{
  u8g2.firstPage();
  do {
  draw();
  } while (u8g2.nextPage());
  delay(50);
}

void draw(void)
{
  char oledTxt[25] = "";
  
  u8g2.setFont(u8g_font_helvR08); // font change – helvB14
  
  u8g2.drawStr(1, 10, "CO2 ppm");
  snprintf (oledTxt, 25, "%d" , SCD30_Co2);
  u8g2.drawStr(63, 10, oledTxt);
  //u8g2.setFont(u8g_font_unifont); // font change – standard
  u8g2.drawStr(1, 21, "Feuchte");
  snprintf (oledTxt, 25, "%d %%" , SCD30_Humidity);
  u8g2.drawStr(63, 21, oledTxt);
  u8g2.drawStr(1, 32, "Temperatur");
  snprintf (oledTxt, 25, "%d C" , SCD30_Temperature);
  u8g2.drawStr(63, 32, oledTxt);

  
  u8g2.drawCircle (110, 15, 15); // contour
  u8g2.drawCircle (105, 14, 4); // left eye
  u8g2.drawCircle (115, 14, 4); // right eye
  if (SCD30_Co2 > RotSchwelle)
  {
    //u8g2.drawPixel (103,25); // mouth
    //u8g2.drawPixel (118,25); // mouth
    u8g2.drawPixel (104,24); // mouth
    u8g2.drawPixel (117,24); // mouth
    u8g2.drawPixel (105,23); // mouth
    u8g2.drawPixel (116,23); // mouth
    u8g2.drawLine (106, 22, 115,22); // mouth
    
  } else if (SCD30_Co2 < GruenSchwelle)
  {
    u8g2.drawPixel (103,22); // mouth
    u8g2.drawPixel (118,22); // mouth
    u8g2.drawPixel (104,23); // mouth
    u8g2.drawPixel (117,23); // mouth
    u8g2.drawPixel (105,24); // mouth
    u8g2.drawPixel (116,24); // mouth
    u8g2.drawLine (106, 25, 115,25); // mouth
  } else // gelb
  {
    u8g2.drawLine (103, 23, 118,23); // mouth
  }
}
#endif

void setup()
{
  Serial.begin(SERIAL_SPEED);
  Serial.println("CO2 AMPEL with NANO + SCD30");
  
  // this resets all the neopixels to an off state
  strip.Begin();
  strip.Show();

  #ifdef U8DISPLAY
    u8g2.begin();
    u8g2.setFont(u8g_font_unifont);
    u8g2.setColorIndex(1); // display draws with pixel on  
  #endif
  
  Wire.begin();


  if (SCD30_AirSensor.begin() == false)
  {
    Serial.println("SCD30 Air sensor not detected.");
    delay(5000);
  }
  SCD30_AirSensor.setMeasurementInterval(5);
  //SCD30_AirSensor.setAltitudeCompensation(200);
  SCD30_AirSensor.setAmbientPressure(976); // Aus open Sensmap

  #ifdef BME680
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
  #endif

  fRGB = green;

  
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
    ErrorCountdown = 5;
    delay(1000);
    #ifdef U8DISPLAY
    HandleDisplay(); 
    #endif
    
  }
  else
  {
    Serial.println("SCD30 w/o data..");  //The SCD30 has data ready every two seconds
    if (ErrorCountdown) ErrorCountdown--;
    else SCD30_Co2 = -1;
  } 

  #ifdef BME680
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
  #endif 

 // Ampel = 0.33- (SCD30_Co2 - 1000) / 3300.0;
 // if (Ampel > 0.33) Ampel = 0.33;
 // if (Ampel < 0) Ampel = 0;
  

  
  for (int j = 0; j < 255; j += 2) {
    for (int i = 0; i < PixelCount; i++) {
      // Neopixel:
      //fHSL = HslColor(Ampel, 1, 0.1);
      //strip.SetPixelColor(i, RgbColor(fHSL));
      if (SCD30_Co2 > RotSchwelle)
        fRGB = red;
      else if (((SCD30_Co2 > OrangeSchwelle) && (fRGB == green)) || ((SCD30_Co2 < OrangeSchwelle) && (fRGB == red)))
        fRGB = orange; 
      else if (SCD30_Co2 < 200) 
        fRGB = red;  // Fehler
      else if (SCD30_Co2 < GruenSchwelle) 
        fRGB = green; 
      strip.SetPixelColor(i, fRGB);
    }
     strip.Show();
    delay(5);
  }  

  
    //strip.SetPixelColor(0, fHSL);
    //strip.SetPixelColor(1, fHSL);
    //strip.SetPixelColor(2, fHSL);
    // strip.SetPixelColor(3, white);
    // the following line demonstrates rgbw color support
    // if the NeoPixels are rgbw types the following line will compile
    // if the NeoPixels are anything else, the following line will give an error
    //strip.SetPixelColor(3, RgbwColor(colorSaturation));
   

  delay(1000);
}
