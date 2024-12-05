#include "Stepper.h"
#include "LiquidCrystal.h"
#include "DHT.h"
#include "RTClib.h"

// Water Level Sensor
const byte sensorPin = A0;
const int waterLevelThreshold = 250;

// Stepper Motor
const byte stepPin1 = 10;
const byte stepPin2 = 11;
const byte stepPin3 = 12;
const byte stepPin4 = 13;
const int stepsPerRevolution = 500;
const int stepperSpeed = 10;
Stepper myStepper = Stepper(stepsPerRevolution, stepPin1, stepPin3, stepPin2, stepPin4);

// DC Motor
const byte dcForwardPin = 38;
const byte dcBackwardPin = 40;
int dcFanState = LOW;

// LCD
const int RS = 8, EN = 9, D4 = 4, D5 = 5, D6 = 6, D7 = 7;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
const unsigned long screenUpdateTimeThreshold = 5000;
unsigned long lastScreenUpdateTime = 0;
bool lcdEmpty = true;
const char errorMessage1[] = "Water level is";
const char errorMessage2[] = "too low.";

// Humidity and Temp sensor
#define DHT11_PIN 24
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);
const float temperatureThreshold = 75.0;

// Clock Cicuit
RTC_DS1307 rtc;

// Global States
enum State {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR,
  NIL
};

// State Management
char stateNames[5][10] = {"Disabled", "Idle", "Running", "Error", "Nil"};
enum State previousState = NIL;
enum State currentState = DISABLED;
volatile enum State requestedState = NIL;
volatile bool stateTransitionRequest = false;

// All Buttons
const unsigned long debounceThreshold = 250;

// Start Button
const byte startButtonPin = 3;
volatile unsigned long prevStartInterruptTime = 0;

// Reset Button
const byte resetButtonPin = 2;
volatile unsigned long prevResetInterruptTime = 0;

// Vent Control Buttons
const byte ventRotateRightPin = 19;
const byte ventRotateLeftPin = 18;
volatile unsigned long ventControlRightInterruptTime = 0;
volatile unsigned long ventControlLeftInterruptTime = 0;
volatile bool moveVentRightFlag = false;
volatile bool moveVentLeftFlag = false;

// LEDs
const byte disabledStatusLED = 46;
const byte idleStatusLED = 48;
const byte runningStatusLED = 50;
const byte errorStatusLED = 52;

void setup() {
  Serial.begin(9600);

  myStepper.setSpeed(stepperSpeed);

  dht.begin();

  rtc.begin();

  lcd.begin(16, 2);

  pinMode(startButtonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(startButtonPin), toggleDisable, FALLING);

  pinMode(resetButtonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(resetButtonPin), toggleReset, FALLING);

  pinMode(ventRotateRightPin, INPUT_PULLUP);
  pinMode(ventRotateLeftPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ventRotateRightPin), rotateVentRightHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(ventRotateLeftPin), rotateVentLeftHandler, FALLING);

  pinMode(dcForwardPin, OUTPUT);
  pinMode(dcBackwardPin, OUTPUT);
  digitalWrite(dcForwardPin, LOW);
  digitalWrite(dcBackwardPin, LOW);

  pinMode(disabledStatusLED, OUTPUT);
  pinMode(idleStatusLED, OUTPUT);
  pinMode(runningStatusLED, OUTPUT);
  pinMode(errorStatusLED, OUTPUT);
}

void loop() {
  if (stateTransitionRequest) {
    previousState = currentState;
    currentState = requestedState;
    requestedState = NIL;
    stateTransitionRequest = false;
    logStateChange();
    cleanUp();
  }

  bool notInErrorState = (currentState != ERROR);

  if (notInErrorState) {
    if (moveVentRightFlag) {
      moveVentRightFlag = false;
      moveVent(1);
    } 
    
    if (moveVentLeftFlag) {
      moveVentLeftFlag = false;
      moveVent(-1);
    }
  }

  switch (currentState) {
    case DISABLED:
      handleDisable();
      break;
    case IDLE:
      handleIdle();
      break;
    case RUNNING:
      handleRunning();
      break;
    case ERROR:
      handleError();
      break;
  }

  previousState = currentState;
}

void toggleDisable() {
  unsigned long curStartInterruptTime = millis();

  if (curStartInterruptTime - prevStartInterruptTime > debounceThreshold) {
    requestedState = ((currentState == DISABLED) ? IDLE : DISABLED);
    stateTransitionRequest = true;
  }

  prevStartInterruptTime = curStartInterruptTime;
}

void toggleReset() {
  unsigned long curResetInterruptTime = millis();
  bool inErrorState = (currentState == ERROR);
  bool notABounce = (curResetInterruptTime - prevResetInterruptTime > debounceThreshold);

  if (inErrorState && notABounce) {
    requestedState = IDLE;
    stateTransitionRequest = true;
  }

  prevResetInterruptTime = curResetInterruptTime;
}

void handleDisable() {
  digitalWrite(disabledStatusLED, HIGH);

  if (lcdEmpty) {
    lcd.noDisplay();
    lcdEmpty = false;
  }
}

void rotateVentRightHandler() {
  unsigned long curInterruptTime = millis();
  bool notInErrorState = (currentState != ERROR);
  bool notABounce = (curInterruptTime - ventControlRightInterruptTime > debounceThreshold);

  if (notInErrorState && notABounce) {
    moveVentRightFlag = true;
  }

  ventControlRightInterruptTime = curInterruptTime;
}

void rotateVentLeftHandler() {
  unsigned long curInterruptTime = millis();
  bool notInErrorState = (currentState != ERROR);
  bool notABounce = (curInterruptTime - ventControlLeftInterruptTime > debounceThreshold);

  if (notInErrorState && notABounce) {
    moveVentLeftFlag = true;
  }

  ventControlLeftInterruptTime = curInterruptTime;
}

void handleIdle() {
  digitalWrite(idleStatusLED, HIGH);

  int waterLevel = analogRead(sensorPin);
  float curTemperature = dht.readTemperature(true);

  if (lcdEmpty) {
    updateDisplayStats();
    lcdEmpty = false;
  }

  if (waterLevel <= waterLevelThreshold) {
    requestedState = ERROR;
    stateTransitionRequest = true;
  } else if (curTemperature > temperatureThreshold) {
    requestedState = RUNNING;
    stateTransitionRequest = true;
  } else {
    unsigned long currentMillis = millis();

    if (currentMillis - lastScreenUpdateTime > screenUpdateTimeThreshold) {
      updateDisplayStats();
      lastScreenUpdateTime = currentMillis;
    }
  }
}

void handleRunning() {
  digitalWrite(runningStatusLED, HIGH);

  int waterLevel = analogRead(sensorPin);
  float curTemperature = dht.readTemperature(true);

  if (waterLevel <= waterLevelThreshold) {
    requestedState = ERROR;
    stateTransitionRequest = true;
  } else if (curTemperature <= temperatureThreshold) {
    requestedState = IDLE;
    stateTransitionRequest = true;
  } else {
    if (dcFanState == LOW) {
      toggleFan();
    }

    unsigned long currentMillis = millis();

    if (currentMillis - lastScreenUpdateTime > screenUpdateTimeThreshold) {
      updateDisplayStats();
      lastScreenUpdateTime = currentMillis;
    }
  }
}

void handleError() {
  digitalWrite(errorStatusLED, HIGH);
  if (lcdEmpty) {
    displayError();
    lcdEmpty = false;
  }
}

void logStateChange() {
    DateTime timeStamp = rtc.now();

    Serial.print("Changed from ");
    Serial.print(stateNames[previousState]);
    Serial.print(" to ");
    Serial.print(stateNames[currentState]);
    Serial.print(" at ");
    Serial.print(timeStamp.hour(), DEC);
    Serial.print(":");
    Serial.print(timeStamp.minute(), DEC);
    Serial.print(":");
    Serial.println(timeStamp.second(), DEC);
}

void toggleFan() {
  dcFanState = (dcFanState == LOW) ? HIGH : LOW;
  digitalWrite(dcForwardPin, dcFanState);
}

void cleanUp() {
  switch (previousState) {
    case DISABLED:
      digitalWrite(disabledStatusLED, LOW);
      lcd.display();
      lcdEmpty = true;
      break;
    case IDLE:
      digitalWrite(idleStatusLED, LOW);
      lcdEmpty = true;
      break;
    case RUNNING:
      digitalWrite(runningStatusLED, LOW);
      lcdEmpty = true;
      if (dcFanState == HIGH) {
        toggleFan();
      }

      break;
    case ERROR:
      digitalWrite(errorStatusLED, LOW);
      lcdEmpty = true;
      break;
  }
}

void updateDisplayStats() {
  float curTemperature = dht.readTemperature(true);
  float curHumidity = dht.readHumidity();

  lcd.clear();
  lcd.home();
  lcd.print(curTemperature);
  lcd.setCursor(0, 1);
  lcd.print(curHumidity);
}

void displayError() {
  lcd.clear();
  lcd.home();
  lcd.print(errorMessage1);
  lcd.setCursor(0, 1);
  lcd.print(errorMessage2);
}

void moveVent(int direction) {
  if (direction >= 0) {
    Serial.println("Moving Vent Right");
  } else {
    Serial.println("Moving Vent Left");
  }

  myStepper.step(direction * stepsPerRevolution);
}