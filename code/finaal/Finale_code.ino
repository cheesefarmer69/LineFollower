#include <EEPROM.h>
#include "EEPROMAnything.h"
#include "BluetoothSerial.h"

// ======= Constanten en Pin Definities =======
#define LED_PIN 2 // Gebruikt de ingebouwde LED op de meeste ESP32-bordjes
#define BAUDRATE 115200
#define EEPROM_SIZE 128
#define CMD_BUFFER_SIZE 64

// --- Sensoren ---
const int sensorAantal = 4;
const int sensorPins[sensorAantal] = {33, 32, 35, 34}; // Volgorde: uiterst links naar uiterst rechts

// --- AANGEPAST VOOR DRV8833 ---
// Pinnen voor de DRV8833 H-Brug
const int MOTOR_L_IN1 = 16; // Verbonden met AIN1
const int MOTOR_L_IN2 = 17; // Verbonden met AIN2

// Pinnen voor Motor Rechts
const int MOTOR_R_IN1 = 18; // Verbonden met BIN1
const int MOTOR_R_IN2 = 19; // Verbonden met BIN2

// ======= Bluetooth Object =======
BluetoothSerial SerialBT;

// ======= Struct voor Parameters =======
struct param_t {
  unsigned long cycleTime;
  unsigned long knipperLichtInterval;
  bool Run;
  int black[sensorAantal];
  int white[sensorAantal];
  bool sensor;
  int power;
  float diff;
  float kp;
} params;

// ======= Globale Variabelen =======
unsigned long previousCycleTime = 0;
unsigned long ledBlinkPreviousTime = 0;
bool ledStatus = false;
char serialCmdBuffer[CMD_BUFFER_SIZE];
uint8_t serialCmdIndex = 0;
char btCmdBuffer[CMD_BUFFER_SIZE];
uint8_t btCmdIndex = 0;
int normalised[sensorAantal];
float position = 0;
unsigned long calculationTime = 0;

// ======= Functie Prototypes =======
void printToAllInterfaces(const String& message, bool addNewLine = true);
void saveParameters();
void handleStreamInput(Stream &inputStream, char* currentCmdBuffer, uint8_t &currentCmdIndex);
void processCommand(char* cmd);
void setCommand();
void debugCommand();
void calibrateCommand();
void unknownCommand();
void stuurMotor(int pin1, int pin2, int snelheid); // NIEUW prototype

// ======= Helper Functies =======
void printToAllInterfaces(const String& message, bool addNewLine) {
  if (addNewLine) {
    Serial.println(message);
    if (SerialBT.connected()) SerialBT.println(message);
  } else {
    Serial.print(message);
    if (SerialBT.connected()) SerialBT.print(message);
  }
}

void saveParameters() {
  EEPROM_writeAnything(0, params);
  EEPROM.commit();
  printToAllInterfaces("Parameters opgeslagen in EEPROM.");
}

// --- NIEUW: Functie om één motor aan te sturen met DRV8833 logica ---
void stuurMotor(int pin1, int pin2, int snelheid) {
  snelheid = constrain(snelheid, -255, 255);
  if (snelheid > 0) { // Vooruit
    analogWrite(pin1, snelheid);
    analogWrite(pin2, 0);
  } else if (snelheid < 0) { // Achteruit
    analogWrite(pin1, 0);
    analogWrite(pin2, abs(snelheid));
  } else { // Stoppen (coast)
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}

// ======= Setup Functie =======
void setup() {
  Serial.begin(BAUDRATE);
  SerialBT.begin("ESP32_LineFollower_Nic");
  EEPROM.begin(EEPROM_SIZE);

  bool newParams = false;
  if (!EEPROM_readAnything(0, params) || params.cycleTime == 0) {
    params.cycleTime = 2000;
    params.knipperLichtInterval = 500000;
    params.Run = false;
    params.sensor = false;
    params.power = 100;
    params.diff = 0.5;
    params.kp = 0.0;
    for(int i=0; i < sensorAantal; ++i) {
      params.black[i] = 0;
      params.white[i] = 4095;
    }
    newParams = true;
  }

  if (newParams) {
    saveParameters();
  } else {
    printToAllInterfaces("Parameters geladen uit EEPROM.");
  }

  pinMode(LED_PIN, OUTPUT);
  
  // --- AANGEPAST VOOR DRV8833 ---
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  printToAllInterfaces("ESP32 Lijnsensor Robot gestart.");
  printToAllInterfaces("Beschikbare commando's: set, debug, calibrate");
  printToAllInterfaces("Params: cycleTime, knipperLichtInterval, Run, sensor, power, diff, kp");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(250);
    digitalWrite(LED_PIN, LOW); delay(250);
  }
}

// ======= Hoofdlus =======
void loop() {
  handleStreamInput(Serial, serialCmdBuffer, serialCmdIndex);
  if (SerialBT.connected()) {
    handleStreamInput(SerialBT, btCmdBuffer, btCmdIndex);
  }

  unsigned long currentTime = micros();
  if (currentTime - previousCycleTime >= params.cycleTime) {
    previousCycleTime = currentTime;
    unsigned long startTime = micros();

    if(params.sensor){
      long teller = 0;
      int noemer = 0;
      for( int i=0; i < sensorAantal; i++){
        int rawValue = analogRead(sensorPins[i]);
        normalised[i] = map(rawValue, params.white[i], params.black[i], 0, 1000);
        normalised[i] = constrain(normalised[i], 0, 1000);
        teller += (long)normalised[i] * (i * 1000);
        noemer += normalised[i];
      }
      
      const int minimaalNoemer = 10;
      if(noemer > minimaalNoemer){
        position = (float)teller / noemer - 1500;
      } else {
        if (position > 0) position = 1500;
        else position = -1500;
      }
    }
    
    // --- AANGEPAST VOOR DRV8833 ---
    if (params.Run) {
      // P-Regelaar logica (blijft hetzelfde)
      float error = -position;
      float output = error * params.kp;
      output = constrain(output, -510, 510);
      
      float powerLeft, powerRight;
      if (output >= 0) {
        powerLeft = constrain(params.power + params.diff * output, -255, 255);
        powerRight = constrain(powerLeft - output, -255, 255);
        powerLeft = powerRight + output;
      } else {
        powerRight = constrain(params.power - params.diff * output, -255, 255);
        powerLeft = constrain(powerRight + output, -255, 255);
        powerRight = powerLeft - output;
      }
      
      // Aansturing via de nieuwe helper functie
      stuurMotor(MOTOR_L_IN1, MOTOR_L_IN2, powerLeft);
      stuurMotor(MOTOR_R_IN1, MOTOR_R_IN2, powerRight);

    } else { // Als Run = false, zet motoren uit
      stuurMotor(MOTOR_L_IN1, MOTOR_L_IN2, 0);
      stuurMotor(MOTOR_R_IN1, MOTOR_R_IN2, 0);
    }
    
    calculationTime = micros() - startTime;
  }
  
  unsigned long currentLoopTime = micros(); 
  if (params.Run) {
    if (currentLoopTime - ledBlinkPreviousTime >= params.knipperLichtInterval) {
      ledBlinkPreviousTime = currentLoopTime;
      ledStatus = !ledStatus;
      digitalWrite(LED_PIN, ledStatus);
    }
  } else {
    if (ledStatus) {
      digitalWrite(LED_PIN, LOW);
      ledStatus = false;
    }
  }
}

// ======= Invoer Verwerken =======
void handleStreamInput(Stream &inputStream, char* currentCmdBuffer, uint8_t &currentCmdIndex) {
  while (inputStream.available()) {
    char c = inputStream.read();
    if (c == '\n' || c == '\r') {
      if (currentCmdIndex > 0) {
        currentCmdBuffer[currentCmdIndex] = '\0';
        processCommand(currentCmdBuffer);
      }
      currentCmdIndex = 0;
    } else {
      if (currentCmdIndex < CMD_BUFFER_SIZE - 1) {
        currentCmdBuffer[currentCmdIndex++] = c;
      } else {
        currentCmdBuffer[CMD_BUFFER_SIZE - 1] = '\0';
        printToAllInterfaces("Waarschuwing: Commando buffer vol.");
        processCommand(currentCmdBuffer); 
        currentCmdIndex = 0;
        while(inputStream.available() > 0 && inputStream.read() != '\n');
      }
    }
  }
}

// ======= Commando Verwerken =======
void processCommand(char* cmd) {
  char* commandPart = strtok(cmd, " ");
  if (!commandPart) return;

  if (strcmp(commandPart, "set") == 0) setCommand();
  else if (strcmp(commandPart, "debug") == 0) debugCommand();
  else if (strcmp(commandPart, "calibrate") == 0) calibrateCommand();
  else unknownCommand();
}

// ======= SET Commando Handler =======
void setCommand() {
  char* paramName = strtok(NULL, " ");
  char* valueStr = strtok(NULL, " ");
  if (!paramName || !valueStr) {
    printToAllInterfaces("Gebruik: set <parameter> <waarde>");
    return;
  }

  bool paramUpdated = false;
  if (strcmp(paramName, "cycleTime") == 0) {
    params.cycleTime = atol(valueStr); paramUpdated = true;
  } else if (strcmp(paramName, "knipperLichtInterval") == 0) {
    params.knipperLichtInterval = atol(valueStr); paramUpdated = true;
  } else if (strcmp(paramName, "Run") == 0) {
    params.Run = (bool)atol(valueStr); paramUpdated = true;
  } else if (strcmp(paramName, "sensor") == 0) {
    params.sensor = (bool)atol(valueStr); paramUpdated = true;
  } else if (strcmp(paramName, "power") == 0) {
    params.power = atol(valueStr); paramUpdated = true;
  } else if (strcmp(paramName, "diff") == 0) {
    params.diff = atof(valueStr); paramUpdated = true;
  } else if (strcmp(paramName, "kp") == 0) {
    params.kp = atof(valueStr); paramUpdated = true;
  } else {
    printToAllInterfaces("Onbekende parameter: " + String(paramName));
  }

  if (paramUpdated) {
    printToAllInterfaces("OK: " + String(paramName) + " -> " + valueStr);
    saveParameters();
  }
}

// ======= DEBUG Commando Handler =======
void debugCommand() {
  printToAllInterfaces("--- Debug Info ---");
  printToAllInterfaces("cycle time: " + String(params.cycleTime));
  String blackValues = "black: ";
  String whiteValues = "white: ";
  String normValues = "normalised: ";
  for (int i = 0; i < sensorAantal; i++) {
    blackValues += String(params.black[i]) + " ";
    whiteValues += String(params.white[i]) + " ";
    normValues += String(normalised[i]) + " ";
  }
  printToAllInterfaces(blackValues);
  printToAllInterfaces(whiteValues);
  printToAllInterfaces(normValues);
  printToAllInterfaces("position: " + String(position, 2));
  printToAllInterfaces("power: " + String(params.power));
  printToAllInterfaces("diff: " + String(params.diff, 2));
  printToAllInterfaces("kp: " + String(params.kp, 2));
  printToAllInterfaces("calculation time: " + String(calculationTime));
  printToAllInterfaces("--- Einde Debug Info ---");
}

// ======= Sensor Kalibratie Handler =======
void calibrateCommand() {
  char* subCommand = strtok(NULL, " ");
  if (subCommand == NULL) {
    printToAllInterfaces("Gebruik: calibrate <black | white>");
    return;
  }
  int numReadings = 10; 
  long totalReading;    
  if (strcmp(subCommand, "black") == 0) {
    printToAllInterfaces("Start zwart-kalibratie... Plaats sensoren boven zwart.");
    delay(2000); 
    for (int i = 0; i < sensorAantal; i++) {
      totalReading = 0;
      for (int j = 0; j < numReadings; j++) {
        totalReading += analogRead(sensorPins[i]);
        delay(20); 
      }
      params.black[i] = totalReading / numReadings;
    }
    printToAllInterfaces("Zwart-kalibratie voltooid.");
    saveParameters();
  } else if (strcmp(subCommand, "white") == 0) {
    printToAllInterfaces("Start wit-kalibratie... Plaats sensoren boven wit.");
    delay(2000); 
    for (int i = 0; i < sensorAantal; i++) {
      totalReading = 0;
      for (int j = 0; j < numReadings; j++) {
        totalReading += analogRead(sensorPins[i]);
        delay(20); 
      }
      params.white[i] = totalReading / numReadings;
    }
    printToAllInterfaces("Wit-kalibratie voltooid.");
    saveParameters();
  } else {
    printToAllInterfaces("Onbekend kalibratie sub-commando.");
  }
}

// ======= Onbekend Commando Handler =======
void unknownCommand() {
  printToAllInterfaces("Onbekend commando. Beschikbaar: set, debug, calibrate.");
}
