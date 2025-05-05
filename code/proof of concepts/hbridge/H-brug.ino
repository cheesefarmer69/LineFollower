// Motor A
const int AIN1 = 16;
const int AIN2 = 17;
// Motor B
const int BIN1 = 18;
const int BIN2 = 19;

int speedA = 0;
bool dirA = false;
int speedB = 0;
bool dirB = false;

void setup() {
  Serial.begin(115200);
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  Serial.println("Stuur commando's in formaat:");
  Serial.println("A 128 0  -> Motor A: snelheid 128, richting vooruit");
  Serial.println("B 255 1  -> Motor B: snelheid 255, richting achteruit");
}

void setMotorSpeed(int in1, int in2, int speed, bool reverse) {
  speed = constrain(speed, 0, 255);
  
  if (speed == 0) {
    // Remmen: beide inputs LOW (of HIGH HIGH als dat je H-brug vereist)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  } else {
    analogWrite(reverse ? in2 : in1, speed);
    digitalWrite(reverse ? in1 : in2, LOW);
  }
}

void processSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    char motor = input.charAt(0);
    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);
    
    if (secondSpace != -1) {
      int speed = input.substring(firstSpace + 1, secondSpace).toInt();
      int direction = input.substring(secondSpace + 1).toInt();
      
      speed = constrain(speed, 0, 255);
      direction = constrain(direction, 0, 1);
      
      if (motor == 'A') {
        speedA = speed;
        dirA = direction;
        setMotorSpeed(AIN1, AIN2, speedA, dirA);
        Serial.print("Motor A: ");
      } else if (motor == 'B') {
        speedB = speed;
        dirB = direction;
        setMotorSpeed(BIN1, BIN2, speedB, dirB);
        Serial.print("Motor B: ");
      }
      
      Serial.print("Snelheid = ");
      Serial.print(speed);
      Serial.print(", Richting = ");
      Serial.println(direction ? "Achteruit" : "Vooruit");
    }
  }
}

void loop() {
  processSerial();
}
