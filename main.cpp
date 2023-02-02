//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include "TFT_eSPI.h" /* Please use the TFT library provided in the library. */
#include "logo.h"
#include "pinout.h"

#define LCD_MODULE_CMD_1
#define SEALEVELPRESSURE_HPA (1027)     // default 1013.25+

float temperature;
float pressure;
float altitude;
float humidity;

char buf_temperature[20] = {' '};
char buf_pressure[20] = {' '};
char buf_altitude[20] = {' '};
char buf_humidity[20] = {' '};

Adafruit_BME280 bme(BME_CS);    // hardware spi

//////////////////////////////////////////////////////////////////////

TFT_eSPI tft = TFT_eSPI();

#if defined(LCD_MODULE_CMD_1)
typedef struct {
  uint8_t cmd;
  uint8_t data[14];
  uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};
#endif

//////////////////////////////////////////////////////////////////////

void setup() {

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  Serial.begin(115200);
  Serial.println("Weatherstation");

  tft.begin();

#if defined(LCD_MODULE_CMD_1)
  for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
    tft.writecommand(lcd_st7789v[i].cmd);
    for (int j = 0; j < lcd_st7789v[i].len & 0x7f; j++) {
      tft.writedata(lcd_st7789v[i].data[j]);
    }

    if (lcd_st7789v[i].len & 0x80) {
      delay(120);
    }
  }
#endif

  tft.setRotation(3);
  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
  delay(2000);

  ledcSetup(0, 2000, 8);
  ledcAttachPin(PIN_LCD_BL, 0);
  ledcWrite(0, 255);

  bool status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

}

//////////////////////////////////////////////////////////////////////

void loop() {

    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();

    tft.setTextSize(4);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    //tft.setFreeFont();

    int temperature_int = (int) temperature;
    float temperature_float = (abs(temperature) - abs(temperature_int)) * 100;
    int temperature_fra = (int)temperature_float;
    sprintf (buf_temperature, "%d.%d", temperature_int, temperature_fra);

    int pressure_int = (int) pressure;
    float pressure_float = (abs(pressure) - abs(pressure_int)) * 100;
    int pressure_fra = (int)pressure_float;
    sprintf (buf_pressure, "%d.%d", pressure_int, pressure_fra);

    int altitude_int = (int) altitude;
    float altitude_float = (abs(altitude) - abs(altitude_int)) * 100;
    int altitude_fra = (int)altitude_float;
    sprintf (buf_altitude, "%d.%d", altitude_int, altitude_fra);

    int humidity_int = (int) humidity;
    float humidity_float = (abs(humidity) - abs(humidity_int)) * 100;
    int humidity_fra = (int)humidity_float;
    sprintf (buf_humidity, "%d.%d", humidity_int, humidity_fra);

    tft.drawString(buf_temperature, 20, 10, 1); tft.drawString("C", 225, 10, 1);
    tft.drawString(buf_pressure, 20, 50, 1); tft.drawString("hPa", 225, 50, 1);
    tft.drawString(buf_altitude, 20, 90, 1); tft.drawString("m", 225, 90, 1);
    tft.drawString(buf_humidity, 20, 130, 1); tft.drawString("%", 225, 130, 1);

    Serial.print("Temperature = "); Serial.print(temperature); Serial.println(" *C");
    Serial.print("Pressure = "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("Approx. Altitude = "); Serial.print(altitude); Serial.println(" m");
    Serial.print("Humidity = "); Serial.print(humidity); Serial.println(" %");
    Serial.println("");

    delay(2000);

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////