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
  ERROR
};

char stateNames[4][10] = {"Disabled", "Idle", "Running", "Error"};

volatile enum State previousState;
volatile enum State currentState;
volatile bool stateChangeFlag;

const byte startButtonPin = 3; 

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

  previousState = DISABLED;
  currentState = DISABLED;
  stateChangeFlag = false;
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

  if (stateChangeFlag) {
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

    stateChangeFlag = false;
  }

  switch (currentState) {
    case IDLE:
      break;
    case RUNNING:
      break;
    case ERROR:
      break;
  }
}

void toggleDisable() {
  previousState = currentState;
  currentState = (currentState == DISABLED) ? IDLE : DISABLED;
  stateChangeFlag = true;
}
