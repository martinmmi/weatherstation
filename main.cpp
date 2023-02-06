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
#define BUTTON_PIN                    14

float temperature, minTemperature, maxTemperature, lastTemperature;
float pressure;
float altitude;
float humidity, maxHumidity, minHumidity, lastHumidity;

uint32_t cpu_frequency;

char buf_temperature[20] = {' '};
char buf_minTemperature[20] = {' '};
char buf_maxTemperature[20] = {' '};
char buf_pressure[20] = {' '};
char buf_altitude[20] = {' '};
char buf_humidity[20] = {' '};
char buf_minHumidity[20] = {' '};
char buf_maxHumidity[20] = {' '};
char buf_reminingTime[20] = {' '};

Adafruit_BME280 bme(BME_CS);    // hardware spi

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int defaultBrightnessLed = 100;       // value from 1 to 255
int step = 1;
int gpio, gpioMap;
int reminingTime = 0;
int waitShort = 1500;
int waitShortRandom = 1000;
int waitLong = 2500;
int waitLongRandom = 1000;
int waitRemining = 500;
int waitReminingRandom = 250;
int debounceTime = 150;
int displayOffTime = 1200000;
int calcTime = 300000;
int serialTime = 10000;
int readSensorTime = 500;
int clearTime = 86400000;

unsigned long lastReadSensor = 0;
unsigned long lastDisplayPrint = 0;
unsigned long lastDisplayPart = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastButtonChanged = 0;
unsigned long lastTurnOff = 0;
unsigned long lastClearMinMax = 0;
unsigned long lastCalc = 0;

bool initSensor = HIGH;
bool initDisplay = HIGH;
bool initSerial = HIGH;
bool buttonState = LOW;
bool lastButtonState = LOW;
bool buttonAction = LOW; 
bool turnOff = LOW;

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

  //setCpuFrequencyMhz(80);               // Set CPU Frequenz 240, 160, 80, 40, 20, 10 Mhz
  
  //cpu_frequency = getCpuFrequencyMhz();
  //Serial.println(cpu_frequency);

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

  lastTurnOff = millis();

  bool status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  strip.begin();
  strip.setBrightness(defaultBrightnessLed);    
  strip.show();

  // Read Values for initialization Min Max
  temperature = bme.readTemperature() - 1;
  minTemperature = temperature;
  maxTemperature = temperature;

  // Read Values for initialization Min Max
  humidity = bme.readHumidity();
  minHumidity = humidity;
  maxHumidity = humidity;

  // First Calculate the remining time for clear Min Max 
  reminingTime = round(((((millis() - lastClearMinMax) - clearTime) * (-1)) / 3600000));
}

//////////////////////////////////////////////////////////////////////

void loop() {

  /////////// Read Sensor Function ///////////
  if ((millis() - lastReadSensor > readSensorTime) || (initSensor == HIGH)) {
    temperature = bme.readTemperature() - 1;
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();

    // Clear Min Max after 24h
    if (millis() - lastClearMinMax > clearTime) {
      minTemperature = temperature;
      maxTemperature = temperature;
      minHumidity = humidity;
      maxHumidity = humidity;

      lastClearMinMax = millis();
    }

    // Calculate Min Max
    if ((temperature < lastTemperature) && (temperature < minTemperature)){
      minTemperature = temperature;
    }
    if ((temperature > lastTemperature) && (temperature > maxTemperature)) {
      maxTemperature = temperature;
    }

    if ((humidity < lastHumidity) && (humidity < minHumidity)){
      minHumidity = humidity;
    }
    if ((humidity > lastHumidity) && (humidity > maxHumidity)) {
      maxHumidity = humidity;
    }

    lastTemperature = temperature;
    lastHumidity = humidity;

    // Map the Values for Neopixel
    int temperatureMapRed = map(temperature, 10, 25, 0, 255);
    int temperatureMapBlue = map(temperature, 10, 25, 65, 0);

    //Serial.println(temperature);
    //Serial.println(temperatureMapRed);
    //Serial.println(temperatureMapBlue);
    //Serial.println(lastDisplayPart);
    //Serial.println(lastDisplayPrint);
    //Serial.println(step);

    // Set Neopixel
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

  /////////// Serial Print Function ///////////

  /*
  if ((millis() - lastSerialPrint > serialTime) || (initSerial == HIGH)) {

    Serial.print("Temperature = "); Serial.print(temperature); Serial.println(" *C");
    Serial.print("TemperatureMin = "); Serial.print(minTemperature); Serial.println(" *C");
    Serial.print("TemperatureMax = "); Serial.print(maxTemperature); Serial.println(" *C");
    Serial.print("Pressure = "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("Approx. Altitude = "); Serial.print(altitude); Serial.println(" m");
    Serial.print("Humidity = "); Serial.print(humidity); Serial.println(" %");
    Serial.print("HumidityMin = "); Serial.print(minHumidity); Serial.println(" %");
    Serial.print("HumidityMax = "); Serial.print(maxHumidity); Serial.println(" %");
    Serial.println("");

    initSerial = LOW;
    lastSerialPrint = millis();
  }
  */
  
  
  /////////// Print Display Function ///////////
  if ((millis() - lastDisplayPrint > 1) || (initDisplay == HIGH)) {

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);

    // Temperature
    if ((millis() - lastDisplayPart > waitLong + random(waitLongRandom)) && (step == 1)) {
      int temperature_int = (int) temperature;
      float temperature_float = (abs(temperature) - abs(temperature_int)) * 100;
      int temperature_fra = (int)temperature_float;
      sprintf (buf_temperature, "%d.%d", temperature_int, temperature_fra);
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_temperature, 85, 70, 6); tft.drawString(".", 219, 53, 6); tft.drawString("C", 230, 88, 4);
      step = 2;
      lastDisplayPart = millis();
    }

    // Temperature Min
    if ((millis() - lastDisplayPart > waitShort + random(waitShortRandom)) && (step == 2)) {
      int minTemperature_int = (int) minTemperature;
      float minTemperature_float = (abs(minTemperature) - abs(minTemperature_int)) * 100;
      int minTemperature_fra = (int)minTemperature_float;
      sprintf (buf_minTemperature, "%d.%d", minTemperature_int, minTemperature_fra);
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_minTemperature, 85, 70, 6); tft.drawString(".", 219, 53, 6); tft.drawString("C", 230, 88, 4);
      tft.drawString("min", 110, 115, 4);
      step = 3;
      lastDisplayPart = millis();
    }

    // Temperature Max
    if ((millis() - lastDisplayPart > waitShort + random(waitShortRandom)) && (step == 3)) {
      int maxTemperature_int = (int) maxTemperature;
      float maxTemperature_float = (abs(maxTemperature) - abs(maxTemperature_int)) * 100;
      int maxTemperature_fra = (int)maxTemperature_float;
      sprintf (buf_maxTemperature, "%d.%d", maxTemperature_int, maxTemperature_fra);
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_maxTemperature, 85, 70, 6); tft.drawString(".", 219, 53, 6); tft.drawString("C", 230, 88, 4);
      tft.drawString("max", 110, 115, 4);
      step = 4;
      lastDisplayPart = millis();
    }

    // Pressure
    if ((millis() - lastDisplayPart > waitLong + random(waitLongRandom)) && (step == 4)) {
      int pressure_int = (int) pressure;
      float pressure_float = (abs(pressure) - abs(pressure_int)) * 100;
      int pressure_fra = (int)pressure_float;
      sprintf (buf_pressure, "%d.%d", pressure_int, pressure_fra);
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_pressure, 45, 70, 6); tft.drawString("hPa", 230, 88, 4);
      step = 5;
      lastDisplayPart = millis();
    }
    
    // Humidity
    if ((millis() - lastDisplayPart > waitLong + random(waitLongRandom)) && (step == 5)) {
      int humidity_int = (int) humidity;
      float humidity_float = (abs(humidity) - abs(humidity_int)) * 100;
      int humidity_fra = (int)humidity_float;
      sprintf (buf_humidity, "%d.%d", humidity_int, humidity_fra);
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_humidity, 90, 70, 6); tft.drawString("%", 228, 87, 4);
      step = 6;
      lastDisplayPart = millis();
    }

    // Humidity Min
    if ((millis() - lastDisplayPart > waitShort + random(waitShortRandom)) && (step == 6)) {
      int minHumidity_int = (int) minHumidity;
      float minHumidity_float = (abs(minHumidity) - abs(minHumidity_int)) * 100;
      int minHumidity_fra = (int)minHumidity_float;
      sprintf (buf_minHumidity, "%d.%d", minHumidity_int, minHumidity_fra);
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_minHumidity, 90, 70, 6); tft.drawString("%", 228, 87, 4);
      tft.drawString("min", 110, 115, 4);
      step = 7;
      lastDisplayPart = millis();
    }

    // Humidity Max
    if ((millis() - lastDisplayPart > waitShort + random(waitShortRandom)) && (step == 7)) {
      int maxHumidity_int = (int) maxHumidity;
      float maxHumidity_float = (abs(maxHumidity) - abs(maxHumidity_int)) * 100;
      int maxHumidity_fra = (int)maxHumidity_float;
      sprintf (buf_maxHumidity, "%d.%d", maxHumidity_int, maxHumidity_fra);
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_maxHumidity, 90, 70, 6); tft.drawString("%", 228, 87, 4);
      tft.drawString("max", 110, 115, 4);
      step = 8;
      lastDisplayPart = millis();
    }

    // Remining Time until Min Max is cleaned
    if ((millis() - lastDisplayPart > waitRemining + random(waitReminingRandom)) && (step == 8)) {
      sprintf (buf_reminingTime, "%d", reminingTime);
      tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
      tft.drawString(buf_reminingTime, 90, 70, 6); tft.drawString("h", 160, 87, 4);
      step = 9;
      lastDisplayPart = millis();
    }

    // Go back to first Step
    if ((millis() - lastDisplayPart > 1) && (step == 9)) {
      lastDisplayPart = millis();
      lastDisplayPrint = millis();
      step = 1;
    }

    initDisplay = LOW;
  }

  // Read Button and do anything
  if (millis() - lastButtonChanged > debounceTime) {
  gpio = analogRead(BUTTON_PIN);
  gpioMap = map(gpio, 4095, 0, 0, 1);
  buttonState = gpioMap; 

    if (buttonState != lastButtonState) {
      lastButtonChanged = millis();
      lastButtonState = buttonState;

      if (buttonState == LOW) {
        //Serial.println("Button released");
        buttonAction = !buttonAction;

        if (buttonAction == HIGH) {
          //Serial.println("HIGH");
          digitalWrite(PIN_POWER_ON, HIGH);
          //ledcSetup(0, 2000, 8);
          ledcAttachPin(PIN_LCD_BL, 0);
          ledcWrite(0, 255);
          turnOff = LOW;
          lastTurnOff = millis();
          strip.clear();
          strip.setBrightness(defaultBrightnessLed);   
          strip.show();
        }
        if (buttonAction == LOW) {
          //Serial.println("LOW");
          digitalWrite(PIN_POWER_ON, LOW);
          //ledcSetup(0, 2000, 8);
          ledcAttachPin(PIN_LCD_BL, 1);
          ledcWrite(0, 255);
          turnOff = HIGH;
          strip.clear();
          strip.setBrightness(0);   
          strip.show();
        }
      }
    }
  }

  // Function turn off the Display after some time
  if ((millis() - lastTurnOff > displayOffTime) && (turnOff == LOW)) {
    ledcAttachPin(PIN_LCD_BL, 1);
    ledcWrite(0, 255);
    turnOff = HIGH;
    buttonAction = !buttonAction;
    strip.clear();
    strip.setBrightness(0);   
    strip.show();
  }

  // Calculate the remining time for clear Min Max
  if (millis() - lastCalc > calcTime) {
    reminingTime = round(((((millis() - lastClearMinMax) - clearTime) * (-1)) / 3600000));
    lastCalc = millis();
  }
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////