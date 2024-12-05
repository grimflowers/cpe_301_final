#include "Stepper.h"
#include "LiquidCrystal.h"
#include "DHT.h"
#include "RTClib.h"

// UART registers 
#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// ADC used for Water Level Sensor
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
unsigned char CHANNEL_NUM = 0;
const int waterLevelThreshold = 250;

// GPIO registers
volatile unsigned char* portB = (unsigned char*) 0x25;
volatile unsigned char* ddrB  = (unsigned char*) 0x24;
volatile unsigned char* pinB = (unsigned char*) 0x23;

volatile unsigned char* portD = (unsigned char*) 0x2B;
volatile unsigned char* ddrD = (unsigned char*) 0x2A;
volatile unsigned char* pinD = (unsigned char*) 0x29;

volatile unsigned char* portE = (unsigned char*) 0x2E;
volatile unsigned char* ddrE = (unsigned char*) 0x2D;
volatile unsigned char* pinE = (unsigned char*) 0x2C;

volatile unsigned char* portG = (unsigned char*) 0x34;
volatile unsigned char* ddrG = (unsigned char*) 0x33;
volatile unsigned char* pinG = (unsigned char*) 0x32;

volatile unsigned char* portL = (unsigned char*) 0x10B;
volatile unsigned char* ddrL = (unsigned char*) 0x10A;
volatile unsigned char* pinL = (unsigned char*) 0x109;

// Stepper Motor
const byte stepPin1 = 10;
const byte stepPin2 = 11;
const byte stepPin3 = 12;
const byte stepPin4 = 13;
const int stepsPerRevolution = 500;
const int stepperSpeed = 10;
Stepper myStepper = Stepper(stepsPerRevolution, stepPin1, stepPin3, stepPin2, stepPin4);

// DC Motor
int dcFanState = LOW;

// LCD
const int RS = 8, EN = 9, D4 = 4, D5 = 5, D6 = 6, D7 = 7;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
const unsigned long screenUpdateTimeThreshold = 60000;
unsigned long lastScreenUpdateTime = 0;
bool lcdEmpty = true;
const char errorMessage1[] = "Water level is";
const char errorMessage2[] = "too low.";

// Humidity and Temp sensor
#define DHT11_PIN 24
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);
const float temperatureThreshold = 70.0;

// Clock Cicuit
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday",
"Friday", "Saturday"};

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

void setup() {
  configureGPIO();
  U0init(9600);
  adc_init();
  dht.begin();
  rtc.begin();
  lcd.begin(16, 2);

  attachInterrupt(digitalPinToInterrupt(startButtonPin), toggleDisable, FALLING);
  attachInterrupt(digitalPinToInterrupt(resetButtonPin), toggleReset, FALLING);

  myStepper.setSpeed(stepperSpeed);
  attachInterrupt(digitalPinToInterrupt(ventRotateRightPin), rotateVentRightHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(ventRotateLeftPin), rotateVentLeftHandler, FALLING);

  // Ensure fan control pins are LOW
  *portD &= 0x7F;
  *portG &= 0xFD;
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
  unsigned long curStartInterruptTime = millis(); // used for button debounce

  if (curStartInterruptTime - prevStartInterruptTime > debounceThreshold) {
    requestedState = ((currentState == DISABLED) ? IDLE : DISABLED);
    stateTransitionRequest = true;
  }

  prevStartInterruptTime = curStartInterruptTime;
}

void toggleReset() {
  unsigned long curResetInterruptTime = millis(); // used for button debounce
  bool inErrorState = (currentState == ERROR);
  bool notABounce = (curResetInterruptTime - prevResetInterruptTime > debounceThreshold);

  if (inErrorState && notABounce) {
    requestedState = IDLE;
    stateTransitionRequest = true;
  }

  prevResetInterruptTime = curResetInterruptTime;
}

void rotateVentRightHandler() {
  unsigned long curInterruptTime = millis(); // used for button debounce
  bool notInErrorState = (currentState != ERROR);
  bool notABounce = (curInterruptTime - ventControlRightInterruptTime > debounceThreshold);

  if (notInErrorState && notABounce) {
    moveVentRightFlag = true;
  }

  ventControlRightInterruptTime = curInterruptTime;
}

void rotateVentLeftHandler() {
  unsigned long curInterruptTime = millis(); // used for button debounce
  bool notInErrorState = (currentState != ERROR);
  bool notABounce = (curInterruptTime - ventControlLeftInterruptTime > debounceThreshold);

  if (notInErrorState && notABounce) {
    moveVentLeftFlag = true;
  }

  ventControlLeftInterruptTime = curInterruptTime;
}

void handleDisable() {
  *portL |= 0x08; // turn disabled status LED on

  if (lcdEmpty) {
    lcd.noDisplay();
    lcdEmpty = false;
  }
}

void handleIdle() {
  *portL |= 0x02; // turn idle status LED on

  int waterLevel = adc_read(CHANNEL_NUM);
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
    unsigned long currentMillis = millis(); // check if enough time has passed for display update

    if (currentMillis - lastScreenUpdateTime > screenUpdateTimeThreshold) {
      updateDisplayStats();
      lastScreenUpdateTime = currentMillis;
    }
  }
}

void handleRunning() {
  *portB |= 0x08; // turn running status LED on

  int waterLevel = adc_read(CHANNEL_NUM);
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
  *portB |= 0x02; // turn error status LED on
  if (lcdEmpty) {
    displayError();
    lcdEmpty = false;
  }
}

void logTime() {
  DateTime timeStamp = rtc.now();

  myPrint(" at ");
  prettyPrintNum(timeStamp.twelveHour());
  myPrint(":");
  prettyPrintNum(timeStamp.minute());
  myPrint(":");
  prettyPrintNum(timeStamp.second());
  myPrint(" ");

  if (timeStamp.isPM()) {
    myPrint("PM");
  } else {
    myPrint("AM");
  }

  myPrint(" on ");
  myPrint(daysOfTheWeek[timeStamp.day()]);
  myPrint(" ");
  prettyPrintNum(timeStamp.day());
  myPrint("/");
  prettyPrintNum(timeStamp.month());
  myPrint("/");
  myPrintln(timeStamp.year());
}

void logStateChange() {
    myPrint("Changed from ");
    myPrint(stateNames[previousState]);
    myPrint(" to ");
    myPrint(stateNames[currentState]);
    logTime();
}

void logVentChange(int direction) {
  if (direction > 0) {
    myPrint("Moved vent right");
  } else {
    myPrint("Moved vent left");
  }

  logTime();
}

void toggleFan() {
  dcFanState = (dcFanState == LOW) ? HIGH : LOW;

  if (dcFanState == LOW) {
    *portD &= 0x7F; // turn fan off
  } else {
    *portD |= 0x80; // turn fan on
  }
}

void cleanUp() {
  switch (previousState) {
    case DISABLED:
      *portL &= 0xF7; // turn disabled status LED off
      lcd.display();
      lcdEmpty = true;
      break;
    case IDLE:
      *portL &= 0xFD; // turn idle status LED off
      lcdEmpty = true;
      break;
    case RUNNING:
      *portB &= 0xF7; // turn running status LED off
      lcdEmpty = true;
      if (dcFanState == HIGH) {
        toggleFan();
      }

      break;
    case ERROR:
      *portB &= 0xFD; // turn error status LED off
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
  myStepper.step(direction * stepsPerRevolution);
  logVentChange(direction);
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void configureGPIO() {
  *ddrB |= 0x0A; // set B1 amd B3 as output

  *ddrD |= 0x80;  // set D7 as output
  *ddrD &= 0xF3;  // set D2 and D3 and inputs
  *portD |= 0x0C; // enable pullups for D2 and D3

  *ddrE &= 0xCF; // set E4 and E5 as inputs
  *portE |= 0x30; // enable pullups for E4 and E5

  *ddrL |= 0x0A; // set L1 and L3 as output

  *ddrG |= 0x02; // set G1 as output
}

void U0init(int U0baud) {
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit() {
  return *myUCSR0A & RDA;
}

unsigned char U0getchar() {
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata) {
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void myPrint(char* printString) {
  int index = 0;

  while(printString[index] != '\0') {
    U0putchar(printString[index]);
    index++;
  }
}

void myPrintln(char* printString) {
  myPrint(printString);
  U0putchar('\n');
}

void myPrint(uint16_t printNum) {
  const int BUFF_SIZE = 5;
  char char_buff[BUFF_SIZE];

  if (printNum == 0) {
    U0putchar('0');
    U0putchar('\n');
  } else {
    int i = 0;

    // Load characters into char buffer
    while (printNum > 0 && i < BUFF_SIZE) {
      unsigned int digit = printNum % 10;
      char_buff[i] = digit + '0';
      i += 1;
      printNum = printNum / 10;
    }

    // Send characters in correct order over USB
    for (int j = i - 1; j >= 0; j--) {
      U0putchar(char_buff[j]);
    }
  }
}

void myPrintln(uint16_t printNum) {
  myPrint(printNum);
  U0putchar('\n');
}

void prettyPrintNum(uint16_t printNum) {
  if (printNum < 10) {
    myPrint("0");
  }

  myPrint(printNum);
}
