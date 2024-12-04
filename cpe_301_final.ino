#include "Stepper.h"
#include "LiquidCrystal.h"
#include "DHT.h"
#include "RTClib.h"

// Water Level Sensor
int sensorPin = A0;
int sensorValue = 0;

// Stepper Motor
int stepPin1 = 10;
int stepPin2 = 11;
int stepPin3 = 12;
int stepPin4 = 13;
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, stepPin1, stepPin3, stepPin2, stepPin4);

// LCD
const int RS = 8, EN = 9, D4 = 4, D5 = 5, D6 = 6, D7 = 7;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Humidity and Temp sensor
#define DHT11_PIN 24
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);

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

  pinMode(disabledStatusLED, OUTPUT);
  pinMode(idleStatusLED, OUTPUT);
  pinMode(runningStatusLED, OUTPUT);
  pinMode(errorStatusLED, OUTPUT);
}

void loop() {
  // sensorValue = analogRead(sensorPin);
  // Serial.println(sensorValue);
  // delay(250);

  // myStepper.setSpeed(10);
  // myStepper.step(stepsPerRevolution);
  // delay(500);

  // Serial.println("Temperature=");
  // Serial.println(dht.readTemperature(true));
  // Serial.print("Humidity=");
  
  // Serial.println(dht.readHumidity());
  // delay(1000)

  if (startButtonFlag) {
    previousState = currentState;
    currentState = ((currentState == DISABLED) ? IDLE : DISABLED);
    logStateChange();
    startButtonFlag = false;
  }

  if (previousState != currentState) {
    clearLEDs();
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
  unsigned long curStartInterruptTime = millis(); // used only to debounce start button

  if (curStartInterruptTime - prevStartInterruptTime > debounceThreshold) {
    startButtonFlag = true;
  }

  prevStartInterruptTime = curStartInterruptTime;
}

void handleDisable() {
  digitalWrite(disabledStatusLED, HIGH);
}

void handleIdle() {
  digitalWrite(idleStatusLED, HIGH);
}

void handleRunning() {
  digitalWrite(runningStatusLED, HIGH);
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