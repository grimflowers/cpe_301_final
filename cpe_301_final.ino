#include "Stepper.h"
#include "LiquidCrystal.h"
#include "DHT.h"
#include "RTClib.h"

int sensorPin = A0;
int sensorValue = 0;

int stepPin1 = 10;
int stepPin2 = 11;
int stepPin3 = 12;
int stepPin4 = 13;
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, stepPin1, stepPin3, stepPin2, stepPin4);

const int RS = 2, EN = 3, D4 = 4, D5 = 5, D6 = 6, D7 = 7;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

#define DHT11_PIN 9
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);

RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday",
"Friday", "Saturday"};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.begin(16, 2);

  lcd.setCursor(0,0);
  lcd.print("Hello");
  lcd.setCursor(2,1);
  lcd.print("World");

  dht.begin();

  rtc.begin();
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
  // delay(1000);

  DateTime now = rtc.now();
  Serial.println(now.year(), DEC);
  Serial.println(now.month(), DEC);
  Serial.println(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.println(now.hour(), DEC);
  Serial.println(now.minute(), DEC);
  Serial.println(now.second(), DEC);
  Serial.println();
  delay(3000);
}
