const int redLED = 13;
const int yellowLED = 12;
const int greenLED = 11;

// Segment pins A to G + optional DP (ignored here)
const int segmentPins[] = {10, 9, 8, 7, 6, 5, 4}; // A-G

// 7-segment digits (0-9)
const byte digits[10][7] = {
  {0, 1, 1, 1, 1, 1, 1},  // 0
  {0, 0, 0, 1, 0, 0, 1},  // 1
  {1, 0, 1, 1, 1, 1, 0},  // 2
  {1, 0, 1, 1, 0, 1, 1},  // 3
  {1, 1, 0, 1, 0, 0, 1},  // 4
  {1, 1, 1, 0, 0, 1, 1},  // 5
  {1, 1, 1, 0, 1, 1, 1},  // 6
  {0, 0, 1, 1, 0, 0, 1},  // 7
  {1, 1, 1, 1, 1, 1, 1},  // 8
  {1, 1, 1, 1, 0, 1, 1}   // 9
};

bool isCounting = false;
unsigned long lastCountdownTime = 0;
int countdownValue = 9;
char currentSignal = 'R';

void setup() {
  Serial.begin(9600);
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
  }
  clearDisplay();
}

void loop() {
  // Read incoming signal
  if (Serial.available()) {
    char signal = Serial.read();

    if (signal == 'R') {
      currentSignal = 'R';
      digitalWrite(redLED, HIGH);
      digitalWrite(yellowLED, LOW);
      digitalWrite(greenLED, LOW);
      clearDisplay();
      isCounting = false;

    } else if (signal == 'Y') {
      if (currentSignal != 'Y') {
        currentSignal = 'Y';
        digitalWrite(redLED, LOW);
        digitalWrite(yellowLED, HIGH);
        digitalWrite(greenLED, LOW);
        countdownValue = 9;
        lastCountdownTime = millis();
        isCounting = true;
      }

    } else if (signal == 'G') {
      currentSignal = 'G';
      digitalWrite(redLED, LOW);
      digitalWrite(yellowLED, LOW);
      digitalWrite(greenLED, HIGH);
      clearDisplay();
      isCounting = false;
    }
  }

  // Countdown logic for yellow phase
  if (isCounting && (millis() - lastCountdownTime >= 1000)) {
    displayDigit(countdownValue);
    countdownValue--;
    lastCountdownTime = millis();

    if (countdownValue < 0) {
      clearDisplay();
      isCounting = false;
    }
  }
}

void displayDigit(int num) {
  if (num < 0 || num > 9) return;
  for (int i = 0; i < 7; i++) {
    digitalWrite(segmentPins[i], digits[num][i]);
  }
}

void clearDisplay() {
  for (int i = 0; i < 7; i++) {
    digitalWrite(segmentPins[i], LOW);
  }
}
