#include <Arduino.h>
#include "LiquidCrystal_I2C.h"

#define SENSOR_PIN A0
#define SAMPLE_WINDOW 50  // Sample window width in mS (50 mS = 20Hz)

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setupLCD();
void setupSensor();
void printValues(int db);
void debug(int db, float voltage, float peakToPeak, int sample);

void setup() {
  setupLCD();
  setupSensor();
  Serial.begin(9600);
}

void loop() {
  unsigned long startMillis= millis();
  unsigned int sample;
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  float peakToPeak = 0;

  while (millis() - startMillis < SAMPLE_WINDOW) {
    sample = analogRead(SENSOR_PIN);
    if (sample < 1023) {
      if (sample > signalMax) {
        signalMax = sample;
      }
      else if (sample < signalMin) {
        signalMin = sample;
      }
    }
  }
  
  peakToPeak = signalMax - signalMin;
  float voltage = sample * (4.85 / 1023);
  int db = map(peakToPeak,5,500,40,90);

  debug(db, voltage, peakToPeak, sample);
  printValues(db);
   
  delay(100); 
}

void setupSensor() {
  pinMode(SENSOR_PIN, INPUT);
}

void setupLCD() {
  lcd.init();
  lcd.backlight();
}

void printValues(int db) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Loudness: ");
  lcd.setCursor(0, 1);
  lcd.print(db);
}

void debug(int db, float voltage, float peakToPeak, int sample) {
  Serial.println("=== DEBUG ===");

  Serial.print("Loudness: ");
  Serial.println(db);
  
  Serial.print("Peak to Peak: ");
  Serial.println(peakToPeak);

  Serial.print("Sample: ");
  Serial.println(sample);

  Serial.print("Voltage: ");
  Serial.println(voltage);
  
  Serial.println("=============");
}
