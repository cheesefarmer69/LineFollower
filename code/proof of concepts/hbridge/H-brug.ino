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

// Veilige motorsturing: altijd eerst beide LOW zetten
void setMotorSpeed(int in1, int in2, int speed, bool reverse) {
  speed = constrain(speed, 0, 255);

  // Eerst beide pinnen LOW om kortsluiting te vermijden
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(10); // korte pauze voor veiligheid

  if (speed == 0) {
    // Remmen: beide inputs LOW (of beide HIGH als je H-brug dat vereist)
    // Hier LOW voor de meeste drivers
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else {
    // Eén pin PWM, andere LOW
    if (reverse) {
      analogWrite(in2, speed);
      digitalWrite(in1, LOW);
    } else {
      analogWrite(in1, speed);
      digitalWrite(in2, LOW);
    }
  }
}

// Richting veilig veranderen met ramp-down en ramp-up
void changeDirection(char motor, int newSpeed, bool newDir) {
  int curSpeed = (motor == 'A') ? speedA : speedB;
  bool curDir = (motor == 'A') ? dirA : dirB;
  int in1 = (motor == 'A') ? AIN1 : BIN1;
  int in2 = (motor == 'A') ? AIN2 : BIN2;

  // Ramp down naar 0
  for (int s = curSpeed; s > 0; s -= 10) {
    setMotorSpeed(in1, in2, s, curDir);
    delay(30);
  }
  setMotorSpeed(in1, in2, 0, curDir); // helemaal stoppen

  // Richting wisselen
  if (motor == 'A') dirA = newDir;
  else dirB = newDir;

  delay(20); // kleine pauze voor zekerheid

  // Ramp up naar nieuwe snelheid
  for (int s = 0; s <= newSpeed; s += 10) {
    setMotorSpeed(in1, in2, s, newDir);
    delay(30);
  }
  setMotorSpeed(in1, in2, newSpeed, newDir);

  // Waarden bijhouden
  if (motor == 'A') speedA = newSpeed;
  else speedB = newSpeed;
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
        changeDirection('A', speed, direction);
        Serial.print("Motor A: ");
      } else if (motor == 'B') {
        changeDirection('B', speed, direction);
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
