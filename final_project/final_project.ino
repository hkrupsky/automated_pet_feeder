/*************************************************************
  ENGR 696/697 Final Project
  Automated Pet Feeder
  
 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <HID.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include "HX711.h"

#define MOTOR_SPEED 175
#define EspSerial Serial1
#define ESP8266_BAUD 115200 // ESP8266 baud rate
#define FOOD_BOWL_WEIGHT 0 // weight of the empty bowl

// Pin Assignments
#define STEPPER_DIR 52 // Stepper direction
#define STEPPER_STEP 50 // Stepper step
#define STEPPER_EN 23 // Stepper enable
#define BEAM 27 // Beam break sensor
#define LOAD_CELL_DATA 3 // Load cell data
#define LOAD_CELL_CLK 2 // Load cell clock

#define CALIBRATION_FACTOR -1952 // Calibration value

ESP8266 wifi(&EspSerial);

HX711 scale;

// Schedule variables
int startHour1, startHour2;
int startMin1, startMin2;
int days1[7]; 
int days2[7];

// Bowl Weight
float currWeight = 0.0; // in grams
float minWeight = 0.0;
float pastWeight = 0.0;
float deltaWeight = 0.0;

// Default feed duration
int duration = 1;

// Current time variables
int currHour, currMin, currDay;

// Low Food Variable
int sensorState = 0, lastState = 0;
unsigned long previousMillis = 0;
const long interval = 1000;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "";
char pass[] = "";

BlynkTimer timer;

WidgetRTC rtc;

BLYNK_CONNECTED() {
  // sync clock on connection
  rtc.begin();
}

void setup()
{
  // Debug console
  Serial.begin(9600);

  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);

  Blynk.begin(auth, wifi, ssid, pass);

  // Set intervals for the different timers
  timer.setInterval(60000L, updateTime);      // update time every 1 minute
  timer.setInterval(300000L, plotFood);       // update chart tracking every 5 minutes
  timer.setInterval(301000L, checkHopper);    // check hopper level every 5 minutes
  timer.setInterval(601000L, maintainWeight); // maintain set bowl weight every 10 minutes
  timer.setInterval(1800000L, updateWeight);  // update bowl weight every 30 minutes

  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_EN, OUTPUT);

  digitalWrite(STEPPER_EN, HIGH); //Disables the motor, so it can rest until it is called upon

  // setup data line on BEAM
  pinMode(BEAM, INPUT);
  digitalWrite(BEAM, HIGH); // turn on the pullup

  // setup load cell scale
  scale.begin(LOAD_CELL_DATA, LOAD_CELL_CLK);
  scale.set_scale(CALIBRATION_FACTOR); // Setting load cell scale
  scale.tare(); // Assuming there is no weight on the scale at start up, reset the scale to 0

  // sync clock
  setSyncInterval(60 * 10); // every 10 minutes

  // updating app with initial values
  Blynk.virtualWrite(V7, duration);
  Blynk.virtualWrite(V8, 0, 0, NULL); // reset schedule1 after system reboot
  Blynk.virtualWrite(V9, 0, 0, NULL); // reset schedule2 after system reboot
  Blynk.virtualWrite(V10, minWeight);
  maintainWeight();
  updateTime();
}

boolean checkSchedule(int schedule) {
  switch(schedule) {
    case 1:
      if(currHour == startHour1 && currMin == startMin1 && days1[currDay-1])
        return true;
      else
        return false;
    case 2:
      if(currHour == startHour2 && currMin == startMin2 && days2[currDay-1])
        return true;
      else
        return false;
  }
}

void updateTime() {
  // function to update the time on the arduino
  currHour = hour();
  currMin = minute();
  currDay = weekday();
  Serial.println(String("Time updated: Day ") + currDay + String(" ") + currHour + String(":") + ((currMin < 10) ? String("0") + currMin : currMin));

  if(checkSchedule(1) || checkSchedule(2)) {
    rotate(duration);
  }
}

void maintainWeight() {
  if (currWeight < minWeight) {
    rotate(5);
  }
  Serial.println("Bowl weight maintained");
}

void updateWeight() {
  // updates the bowl weight to the app
  currWeight = scale.get_units(10);
  (currWeight < 0) ? currWeight = 0 : currWeight = currWeight; // if weight is negative, set equal to 0
  Serial.println(String("Current Bowl Weight: ") + currWeight);
}

void plotFood() {
  Blynk.virtualWrite(V12, deltaWeight);
  Serial.println("Data sent to food dispensed chart");
  deltaWeight = 0.0;
}

void checkHopper() {
  sensorState = digitalRead(BEAM);
  
  if (sensorState && !lastState) {
      Serial.println("Low Food");
      Blynk.notify("Low Food! Refill the hopper");
  }
  lastState = sensorState;
}

void rotate(int cycles) {
  pastWeight = currWeight;
  digitalWrite(STEPPER_EN, LOW); //Enabling the motor, so it will move when asked to
  digitalWrite(STEPPER_DIR, HIGH); //Setting the forward direction

  for(int i = 0; i < cycles; i++) {
    //Cycle through forward direction
    for (int j = 0; j < 1200; j++) {
      digitalWrite(STEPPER_STEP, HIGH);
      delayMicroseconds(MOTOR_SPEED);
      digitalWrite(STEPPER_STEP, LOW);
      delayMicroseconds(MOTOR_SPEED);
    }
  
    //change direction
    digitalWrite(STEPPER_DIR, LOW);
    
    //cycle through anti-jam
    for (int k = 0; k < 500; k++) {
      digitalWrite(STEPPER_STEP, HIGH);
      delayMicroseconds(MOTOR_SPEED*4);
      digitalWrite(STEPPER_STEP, LOW);
      delayMicroseconds(MOTOR_SPEED*4);
    }
  }
  digitalWrite(STEPPER_EN, HIGH); //Disables the motor, so it can rest until the next time it is called upon
  updateWeight();
  deltaWeight += currWeight - pastWeight;
}

BLYNK_WRITE(V6) // V6 is the Virtual Pin for Dispensing
{
  int pinValue = param.asInt();
  Serial.println(String("Dispensing: ") + pinValue);
  
  if (pinValue) {
    rotate(duration);
  }
}

BLYNK_WRITE(V7) // V7 is the Virtual Pin for Feeding Duration
{
  int pinValue = param.asInt();
  
  duration = pinValue;
  Serial.println(String("Feeding Duration: ") + duration);
}

BLYNK_WRITE(V8) // V8 is the Virtual Pin for Scheduling1
{
  TimeInputParam t(param);

  // Process start time

  if (t.hasStartTime())
  {
    startHour1 = t.getStartHour();
    startMin1 = t.getStartMinute();
    
    Serial.println(String("Dispense at: ") + startHour1 + ":" + ((startMin1 < 10) ? String("0") + startMin1 : startMin1));
  }

  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)

  for (int i = 1; i <= 7; i++) {
    if (t.isWeekdaySelected(i)) {
      // Store sunday in index 0
      days1[((i+1)%7)-1] = 1;
      Serial.println(String("Day ") + i + " is selected");
    } else {
      days1[((i+1)%7)-1] = 0;
    }
  }
}

BLYNK_WRITE(V9) // V9 is the Virtual Pin for Scheduling2
{
  TimeInputParam t(param);

  // Process start time

  if (t.hasStartTime())
  {
    startHour2 = t.getStartHour();
    startMin2 = t.getStartMinute();
    
    Serial.println(String("Dispense at: ") + startHour2 + ":" + ((startMin2 < 10) ? String("0") + startMin2 : startMin2));
  }

  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)

  for (int i = 1; i <= 7; i++) {
    if (t.isWeekdaySelected(i)) {
      // Store sunday in index 0
      days2[((i+1)%7)-1] = 1;
      Serial.println(String("Day ") + i + " is selected");
    } else {
      days2[((i+1)%7)-1] = 0;
    }
  }
}

BLYNK_WRITE(V10) // V10 is the virtual pin for selecting minimum bowl weight
{
  double pinValue = param.asDouble();
  Serial.println(String("Min food weight: ") + pinValue);

  // setting minimum weight of food in bowl
  minWeight = pinValue;
}

BLYNK_READ(V11) // V11 gets the command from the app to check the current bowl weight
{
  updateWeight();
  Blynk.virtualWrite(V11, currWeight);
}

BLYNK_WRITE(V13) // V13 gets command to reset the scale to 0
{
  int pinValue = param.asInt();
  
  if (pinValue) {
    scale.tare();
    updateWeight();
    Serial.println("Bowl Weight Reset");
  }
}

void loop()
{
  Blynk.run();
  timer.run(); // Initiates BlynkTimer
}
