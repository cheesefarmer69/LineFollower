const int buttonPin = 4; // GPIO-pin voor de knop
volatile bool startStopFlag = false;  // Robot aan/uit-status
volatile bool stateChanged = false;   // Vlag voor éénmalige actie in loop
volatile unsigned long lastInterruptTime = 0;
const unsigned long debounceDelay = 500; // debounce tijd in ms

void IRAM_ATTR handleButtonInterrupt() {
  unsigned long currentTime = millis();
  if (currentTime - lastInterruptTime > debounceDelay) {
    startStopFlag = !startStopFlag;   // Toggle toestand
    stateChanged = true;              // Laat loop weten dat er iets veranderde
    lastInterruptTime = currentTime;
  }
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonInterrupt, FALLING);
  Serial.begin(115200);
}

void loop() {
  static bool systemRunning = false;  // Houdt bij of robot actief is

  if (stateChanged) {
    stateChanged = false;             // Reset vlag
    systemRunning = startStopFlag;    // Zet nieuwe status
    Serial.println(systemRunning ? "Robot gestart" : "Robot gestopt");
  }

  if (!systemRunning) {
    // Robot is gestopt, doe niets
    return;
  }

  // *** Hier je robotcode ***
  Serial.println("Robot is bezig...");
  delay(1000); // Simuleer werk
}
