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
#define SEALEVELPRESSURE_HPA (1027)                  // default 1013.25
#define LED_PIN                        1
#define LED_COUNT                      3

float temperature;
float pressure;
float altitude;
float humidity;

char buf_temperature[20] = {' '};
char buf_pressure[20] = {' '};
char buf_altitude[20] = {' '};
char buf_humidity[20] = {' '};

Adafruit_BME280 bme(BME_CS);    // hardware spi

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int defaultBrightnessLed = 100;       // value from 1 to 255
int step = 1;

long lastReadSensor = 0;
long lastDisplayPrint = 0;
long lastDisplayPart = 0;
long lastSerialPrint = 0;

bool initSensor = HIGH;
bool initDisplay = HIGH;
bool initSerial = HIGH;

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
  tft.setRotation(3);
  tft.setSwapBytes(true);

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

  ledcSetup(0, 2000, 8);
  ledcAttachPin(PIN_LCD_BL, 0);
  ledcWrite(0, 255);

  bool status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  strip.begin();
  strip.setBrightness(defaultBrightnessLed);    
  strip.show();

}

//////////////////////////////////////////////////////////////////////

void loop() {

  if ((millis() - lastReadSensor > 250) || (initSensor == HIGH)) {
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();

    int temperatureMapRed = map(temperature, 10, 25, 0, 255);
    int temperatureMapBlue = map(temperature, 10, 25, 65, 0);

    //Serial.println(temperature);
    //Serial.println(temperatureMapRed);
    //Serial.println(temperatureMapBlue);

    strip.clear();
		
    if ((temperature >= 10) && (temperature <= 25) && (humidity < 80)) {
      strip.fill(strip.Color(temperatureMapRed, 25, temperatureMapBlue), 0, LED_COUNT);														            	
    }

    if (((temperature < 10) || (temperature > 25)) && (humidity < 80)) {
      strip.fill(strip.Color(0, 0, 0), 0, LED_COUNT);														            	
    }

    if (humidity > 80) {
      strip.fill(strip.Color(255, 0, 0), 0, LED_COUNT);														            	
    }

    strip.show();

    initSensor = LOW;
    lastReadSensor = millis();
  }


  if ((millis() - lastSerialPrint > 2000) || (initSerial == HIGH)) {

    Serial.print("Temperature = "); Serial.print(temperature); Serial.println(" *C");
    Serial.print("Pressure = "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("Approx. Altitude = "); Serial.print(altitude); Serial.println(" m");
    Serial.print("Humidity = "); Serial.print(humidity); Serial.println(" %");
    Serial.println("");

    initSerial = LOW;
    lastSerialPrint = millis();
  }
  

  if ((millis() - lastDisplayPrint < 150000) || (initDisplay == HIGH)) {

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);

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


    if ((millis() - lastDisplayPart < 3000 + random (1000)) && (step == 1)) {
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_temperature, 85, 70, 6); tft.drawString(".", 219, 53, 6); tft.drawString("C", 230, 88, 4);
      step = 2;
    }
    if (((millis() - lastDisplayPart > 4000 + random (2000)) && (millis() - lastDisplayPart < 8000)) && (step == 2)) {
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_pressure, 45, 70, 6); tft.drawString("hPa", 230, 88, 4);
      step = 3;
    }
    if (((millis() - lastDisplayPart > 8000 + random (2000)) && (millis() - lastDisplayPart < 15000)) && (step == 3)) {
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_humidity, 90, 70, 6); tft.drawString("%", 228, 87, 4);
      step = 4;
    }
    if ((millis() - lastDisplayPart > 15000) && (step == 4)) {
      lastDisplayPart = millis();
      lastDisplayPrint = millis();
      step = 1;
    }

    initDisplay = LOW;
  }

}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////