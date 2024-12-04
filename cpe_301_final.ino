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
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, stepPin1, stepPin3, stepPin2, stepPin4);

// DC Motor
const byte dcForwardPin = 38;
const byte dcBackwardPin = 40;
volatile int dcFanState = LOW;

// LCD
const int RS = 8, EN = 9, D4 = 4, D5 = 5, D6 = 6, D7 = 7;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Humidity and Temp sensor
#define DHT11_PIN 24
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);
const float temperatureThreshold = 67.0;

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
volatile enum State previousState = NIL;
volatile enum State currentState = DISABLED;
volatile enum State requestedState = NIL;
volatile bool stateTransitionRequest = false;

// Start Button
const byte startButtonPin = 3;
const unsigned long debounceThreshold = 250;
volatile bool startButtonFlag = false;
volatile unsigned long prevStartInterruptTime = 0;

// LEDs
const byte disabledStatusLED = 46;
const byte idleStatusLED = 48;
const byte runningStatusLED = 50;
const byte errorStatusLED = 52;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);

  lcd.setCursor(0,0);
  lcd.print("Hello");
  lcd.setCursor(2,1);
  lcd.print("World");

  dht.begin();

  rtc.begin();

  pinMode(startButtonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(startButtonPin), toggleDisable, FALLING);

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
  // waterLevel = analogRead(sensorPin);
  // Serial.println(waterLevel);
  // delay(250);

  // myStepper.setSpeed(10);
  // myStepper.step(stepsPerRevolution);
  // delay(500);

  // Serial.println("Temperature=");
  // Serial.println(dht.readTemperature(true));
  // Serial.print("Humidity=");
  
  // Serial.println(dht.readHumidity());
  // delay(1000)

  if (stateTransitionRequest) {
    currentState = requestedState;
    requestedState = NIL;
    logStateChange();
    stateTransitionRequest = false;
    cleanUp();
  }

  // if (previousState != currentState) {
  //   clearLEDs();
  // }

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
  unsigned long curStartInterruptTime = millis(); // used only to debounce start button

  if (curStartInterruptTime - prevStartInterruptTime > debounceThreshold) {
    requestedState = ((currentState == DISABLED) ? IDLE : DISABLED);
    stateTransitionRequest = true;
  }

  prevStartInterruptTime = curStartInterruptTime;
}

void handleDisable() {
  digitalWrite(disabledStatusLED, HIGH);
}

void handleIdle() {
  digitalWrite(idleStatusLED, HIGH);

  int waterLevel = analogRead(sensorPin);
  float curTemperature = dht.readTemperature(true);
  Serial.println(curTemperature);

  if (waterLevel <= waterLevelThreshold) {
    requestedState = ERROR;
    stateTransitionRequest = true;
  } else if (curTemperature > temperatureThreshold) {
    requestedState = RUNNING;
    stateTransitionRequest = true;
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
  }
}

void handleError() {
  digitalWrite(errorStatusLED, HIGH);
}

void clearLEDs() {
  digitalWrite(disabledStatusLED, LOW);
  digitalWrite(idleStatusLED, LOW);
  digitalWrite(runningStatusLED, LOW);
  digitalWrite(errorStatusLED, LOW);
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
      break;
    case IDLE:
      digitalWrite(idleStatusLED, LOW);
      break;
    case RUNNING:
      digitalWrite(runningStatusLED, LOW);
      if (dcFanState == HIGH) {
        toggleFan();
      }

      break;
    case ERROR:
      digitalWrite(errorStatusLED, LOW);
      break;
  }
}