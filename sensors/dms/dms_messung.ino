#include <Wire.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>

NAU7802 nau;
unsigned long startTime;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();

  if (!nau.begin()) {
    Serial.println("FEHLER: NAU7802 nicht gefunden!");
    while (1);
  }

  nau.setGain(NAU7802_GAIN_128);
  nau.setSampleRate(NAU7802_SPS_80);
  nau.calibrateAFE();

  // CSV Header
  Serial.println("zeit_ms,raw,spannung_mV");

  startTime = millis();
}

void loop() {
  if (nau.available()) {
    long rawValue = nau.getReading();
    float voltage_mV = (rawValue * 2400.0) / (8388607.0 * 128.0);
    unsigned long zeit = millis() - startTime;

    // CSV Zeile: Zeit, Rohwert, Spannung
    Serial.print(zeit);
    Serial.print(",");
    Serial.print(rawValue);
    Serial.print(",");
    Serial.println(voltage_mV, 4);
  }
  delay(100);
}
