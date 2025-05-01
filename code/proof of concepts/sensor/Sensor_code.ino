#include <QTRSensors.h>

QTRSensors qtr;
const uint8_t sensorPins[] = {14, 12};  // GPIO14 en GPIO12
const int ledPins[] = {17, 18};         // LED voor sensor 1 en 2
const int numSensors = 2;
int threshold = 2300;

uint16_t sensorValues[numSensors];

void setup() {
  Serial.begin(115200);
  
  // LED pinnen initialiseren
  for(int i=0; i<numSensors; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, numSensors);
  qtr.setEmitterPin(255); // emitter uit
}

void loop() {
  qtr.read(sensorValues);
  
  Serial.println("\n=== Sensor Uitlezing ===");  // Duidelijke scheiding
  
  for(int i=0; i<numSensors; i++) {
    // Seriële output formatteren
    Serial.print("Sensor ");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(sensorValues[i]);
    Serial.print(" | ");
    
    if(sensorValues[i] > threshold) {
      Serial.println("Zwart  \u25CF");  // Unicode zwarte cirkel
      digitalWrite(ledPins[i], HIGH);
    } else {
      Serial.println("Wit   \u25CB");  // Unicode witte cirkel
      digitalWrite(ledPins[i], LOW);
    }
  }
  
  Serial.println("=======================");  // Onderkant van het kader
  delay(150);  // Iets langere delay voor beter leesbare output
}
