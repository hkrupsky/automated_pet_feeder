/*************************************************************
  SFSU ENGR 696/697 Final Project
  Automated Pet Feeder
  Howard Krupsky 2021
 *************************************************************/
#include <HID.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include "HX711.h"
#include <TinyStepper_28BYJ_48.h>
#include <movingAvg.h>

// Program Constants
#define MOTOR_SPEED 175          // Auger stepper speed
#define EspSerial Serial1        // ESP output
#define ESP8266_BAUD 115200      // ESP8266 baud rate
#define FOOD_BOWL_WEIGHT 0       // weight of the empty bowl
#define BLYNK_PRINT Serial       // Print to console

// Pin Assignments
#define STEPPER_DIR 52           // Stepper direction
#define STEPPER_STEP 50          // Stepper step
#define STEPPER_EN 23            // Stepper enable
#define BEAM 27                  // Beam break sensor
#define LOAD_CELL_DATA 3         // Load cell data
#define LOAD_CELL_CLK 2          // Load cell clock
#define AGITATOR_A  8            // Agitator Coil A
#define AGITATOR_B  9            // Agitator Coil B
#define AGITATOR_C  10           // Agitator Coil C
#define AGITATOR_D  11           // Agitator Coil D
#define CALIBRATION_FACTOR -1952 // Loadcell calibration value

// Defining various objects
ESP8266 wifi(&EspSerial);
HX711 scale;
BlynkTimer timer;
WidgetRTC rtc;
TinyStepper_28BYJ_48 agitator;
movingAvg avgDailyAmt(5); // average amount over 5 days

// WiFi credentials.
char ssid[] = "";
char pass[] = "";

// Blynk Authentication Token
char auth[] = "";

// Schedule variables
int startHour1, startHour2;
int startMin1, startMin2;
int days1[7]; 
int days2[7];

// Bowl Weight
float currWeight = 0.0; // in grams
float minWeight = 0.0;

// Default feed duration
int duration = 1;

// Agitator settings
int steps = 0;
const int STEPS_PER_REVOLUTION = 2048;

// Current time variables
int currHour, currMin, currDay;

// Low food variables
int sensorState = 0, lastState = 0;
unsigned long previousMillis = 0;
const long interval = 1000;

// Food Totals
int dailyTotal;
int tempWeight;

// Sync clock on connection
BLYNK_CONNECTED() {
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
  timer.setInterval(300000L, plotFood);       // update chart every 5 minutes
  timer.setInterval(601000L, maintainWeight); // maintain set bowl weight every 10 minutes
  timer.setInterval(1800000L, updateWeight);  // update bowl weight every 30 minutes
  timer.setInterval(3601000L, checkHopper);   // check hopper level every 60 minutes

  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, HIGH); // Disables the motor, so it can rest until it is called upon

  // connect and configure the agitator stepper to its IO pins
  agitator.connectToPins(AGITATOR_A, AGITATOR_B, AGITATOR_C, AGITATOR_D);

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

  // initialize total variables
  avgDailyAmt.begin();
  dailyTotal = 0.0f;
  dailyAvg = 0;
}

// Check if scheduled feeding time has been reached and return true or false
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

// Update the time, provide daily food info, and check feeding schedule
void updateTime() {
  Blynk.virtualWrite(V15, 0); // Tell app the motor is not rotating
  
  // if new day, calculate rolling average and display daily amount on historic graph
  if (currHour > hour()) {
    Blynk.virtualWrite(V14, avgDailyAmt(dailyTotal));
    Blynk.virtualWrite(V17, dailyTotal);
    dailyTotal = 0;    // reset daily total
  }

  // update stored time and date
  currHour = hour();
  currMin = minute();
  currDay = weekday();
  Serial.println(String("Time updated: Day ") + currDay + String(" ") + currHour + String(":") + ((currMin < 10) ? String("0") + currMin : currMin));

  // checking if scheduled feeding time is reached
  if(checkSchedule(1)) {
    // if schedule has been reached and min weight selected
    if(minWeight != 0) {
      maintainWeight();
    } // otherwise, dispense default amount
    else {
      Serial.println(String("Scheduled Feeding 1: ") + startHour1 + String(":") + ((startMin1 < 10) ? String("0") + startMin1 : startMin1));
      rotate(duration);
    }
  }
  else if(checkSchedule(2)) {
    // if schedule has been reached and min weight selected
    if(minWeight != 0) {
      maintainWeight();
    } // otherwise, dispense default amount
    else {
      Serial.println(String("Scheduled Feeding 2: ") + startHour2 + String(":") + ((startMin1 < 10) ? String("0") + startMin2 : startMin2));
      rotate(duration);
    }
  }
}

// Maintaining weight in bowl specified by user
void maintainWeight() {
  if (currWeight < minWeight) {
    rotate(5);
  }
  Serial.println("Bowl weight maintained");
}

// Update stored weight value
void updateWeight() {
  // updates the bowl weight to the app
  currWeight = scale.get_units(10);
  (currWeight < 0) ? currWeight = 0 : currWeight = currWeight; // if weight is negative, set equal to 0
  Serial.println(String("Current Bowl Weight: ") + currWeight);
}

// Plot current bowl weight on historic graph
void plotFood() {
  Blynk.virtualWrite(V12, currWeight);
  Serial.println(String("Data sent to food chart: ") + currWeight);
}

// Alert user to low food event
void checkHopper() {
  sensorState = digitalRead(BEAM);
  
  if (sensorState && !lastState) {
      Serial.println("Low Food");
      Blynk.notify("Low Food! Refill the hopper");
  }
  lastState = sensorState;
}

// Rotate the auger stepper motor and dispense food
void rotate(int cycles) {
  digitalWrite(STEPPER_EN, LOW); //Enabling the motor, so it will move when asked to
  digitalWrite(STEPPER_DIR, HIGH); //Setting the forward direction
  Blynk.virtualWrite(V15, 1); //Tell app the motor is rotating
  tempWeight = static_cast<int>(currWeight);

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

    //change direction
    digitalWrite(STEPPER_DIR, HIGH);
  }
  digitalWrite(STEPPER_EN, HIGH); //Disables the motor, so it can rest until the next time it is called upon
  updateWeight();
  dailyTotal += (static_cast<int>(currWeight) - tempWeight);
  agitator();
}

// Activate hopper agitator to prevent infeed jams
void agitator() {
  Serial.println("Agitator activated");
  agitator.setSpeedInStepsPerSecond(500);
  agitator.setAccelerationInStepsPerSecondPerSecond(1000);
  agitator.moveRelativeInSteps(STEPS_PER_REVOLUTION / 2);
  delay(500);
}

// V6 is the Virtual Pin for Dispensing
BLYNK_WRITE(V6) {
  int pinValue = param.asInt();
  Serial.println(String("Dispensing: ") + pinValue);
  
  if (pinValue) {
    rotate(duration);
  }
}

// V7 is the Virtual Pin for Feeding Duration
BLYNK_WRITE(V7) {
  int pinValue = param.asInt();
  
  duration = pinValue;
  Serial.println(String("Feeding Duration: ") + duration);
}

// V8 is the Virtual Pin for Scheduling1
BLYNK_WRITE(V8) {
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

// V9 is the Virtual Pin for Scheduling2
BLYNK_WRITE(V9) {
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

// V10 is the virtual pin for selecting minimum bowl weight
BLYNK_WRITE(V10) {
  double pinValue = param.asDouble();
  Serial.println(String("Min food weight: ") + pinValue);

  // setting minimum weight of food in bowl
  minWeight = pinValue;
}

// V11 gets the command from the app to check the current bowl weight
BLYNK_READ(V11) {
  updateWeight();
  Blynk.virtualWrite(V11, currWeight);
}

// V13 gets command to reset the scale to 0
BLYNK_WRITE(V13) {
  int pinValue = param.asInt();
  
  if (pinValue) {
    scale.tare();
    updateWeight();
    Serial.println("Bowl Weight Reset");
  }
}

void loop()
{
  Blynk.run(); // Initiates Blynk
  timer.run(); // Initiates BlynkTimer
}
