// This example code is in the Public Domain (or CC0 licensed, at your option.)
// By Evandro Copercini - 2018
//
// This example creates a bridge between Serial and Classical Bluetooth (SPP)
// and also demonstrate that SerialBT have the same functionalities of a normal Serial
// Note: Pairing is authenticated automatically by this device

#include "BluetoothSerial.h"

const int potPin = 25;   // Potentiometer op GPIO34
const int ledPin = 27;   // LED op GPIO25
const int buttonPin  = 4;
const int baudrate = 115200;
bool lastButtonState = LOW;  // Vorige staat van de knop
bool currentButtonState = LOW;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000; // 5 seconden

String device_name = "ESP32-BT-Slave";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(baudrate);
  SerialBT.begin("ESP32_Nic");  //Bluetooth device name
  //SerialBT.deleteAllBondedDevices(); // Uncomment this to delete paired devices; Must be called after begin
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(buttonPin, INPUT);
}

void loop() 
{
   detectRisingEdge();
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }

  if (SerialBT.available()) {
  char cmd = SerialBT.read();        // Lees één karakter van Bluetooth

  Serial.write(cmd);                 // Stuur het karakter door naar Serial-monitor

  if (cmd == '1') {
    digitalWrite(ledPin, HIGH);      // LED aan
  } else if (cmd == '0') {
    digitalWrite(ledPin, LOW);       // LED uit
  }
  }
   if (millis() - lastSendTime > sendInterval) {
    int potValue = analogRead(potPin);
    SerialBT.println(potValue);
    lastSendTime = millis();
  }
  delay(20);
}

void detectRisingEdge() {
  currentButtonState = digitalRead(buttonPin);

  if (currentButtonState == HIGH && lastButtonState == LOW) {
    // Rising edge gedetecteerd
    SerialBT.print("Baudrate:");
    SerialBT.println(baudrate);
    // Hier kan je andere acties uitvoeren, bijv. een LED aanzetten
  }

  // Update de vorige staat voor de volgende meting
  lastButtonState = currentButtonState;
}


