#include <Servo.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_APDS9960.h>

#include "faces.h"

Servo bottomServo;
Servo topServo;

Adafruit_LIS3DH accelerometer = Adafruit_LIS3DH();

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

Adafruit_APDS9960 gesture_sensor;

typedef enum State {
  WAITING = 1,
  FLIPPING,
  TURNING,
  ANGRY,
  DRIVING,
  TANTRUM,
  CONFUSED
} states;

const int bottomServoPin = 2;
const int topServoPin = 13;
const int switchPins[5] = {8, 9, 10, 11, 12};
const int switchAngles[5] = {0, 47, 93, 139, 180};
const int armRetractedAngle = 0;
const int armExtendedAngle = 180;
const int motorPins[4] = {4, 5, 6, 7};
const int interruptPin = 3;

bool* switchFlipped;
int timesFlipped;
int prevAccelReading;
unsigned long prev_ms;
unsigned long prev_display_ms; 
unsigned long time_to_wait;
State state;
State prevState;

void setup() {

  // Program initializations
  Serial.begin(9600);
  bottomServo.attach(bottomServoPin, 475, 2415);
  bottomServo.write(switchAngles[2]);
  topServo.attach(topServoPin);
  topServo.write(0);
  for (int i = 0; i < 5; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);
  }
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
//  pinMode(interruptPin, INPUT);
  
  stopDriving();
  switchFlipped = (bool*) calloc(5, sizeof(bool));
  timesFlipped = 0;
  prevAccelReading = 0;
  prev_ms = millis();
  prev_display_ms = millis();
  time_to_wait = random(100, 4000);
  state = WAITING;
  prevState = WAITING;

  // Accelerometer initialization
  if (!accelerometer.begin(0x18)) {
    Serial.println("Could not initialize accelerometer.");
  }
  accelerometer.setRange(LIS3DH_RANGE_4_G);
  
  // OLED initialization
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  // Gesture sensor initialization
  if (!gesture_sensor.begin()) {
    Serial.println("Could not initialize gesture sensor.");
  }
  gesture_sensor.enableProximity(true);
//  gesture_sensor.setProximityInterruptThreshold(0, 255);
//  gesture_sensor.enableProximityInterrupt();
  gesture_sensor.enableGesture(true);

  Serial.println("Initialization finished.");
}

void loop() {

//  Serial.println(state);
//  Serial.println(prev_display_ms);
//  Serial.println(millis());
  
  // if box shaken, become confused
  if (isBoxShaken()) {
    prev_display_ms = millis();
    if (state == DRIVING) {
      stopDriving();
      state = WAITING;
    }
    prevState = state;
    state = CONFUSED;
  }

  // if switches were flipped too much, become angry
  if (timesFlipped > 5) {
    timesFlipped = 0;
    prev_ms = millis();
    prev_display_ms = millis();
    state = ANGRY;
  }

  switch(state) {
    case WAITING: {
      drawOWO();
      stopDriving();
      checkAndHandleGesture();
      if (checkSwitches()) {
        state = FLIPPING;
      }
      break;
    }
    case FLIPPING: {
      drawClosedEyes();
      flipSwitch();
      if (!checkSwitches()) {
        state = WAITING;
      }
      break;
    }
    case TURNING: {
      drawMouthOpen();
      checkTime(2000);
      break;
    }
    case ANGRY: {
      drawAngry();
//      checkAndHandleProximity();
      checkTime(5000);
      break;
    }
    case DRIVING: {
      drawAngry();
      checkTime(1000);
      break;
    }
    case TANTRUM: {
      checkTime(5000);
      break;
    }
    case CONFUSED: {
      drawConfused();
      checkTime(5000);
      break;
    }
    default: {
      Serial.println("Invalid state");
      break;
    }
  }
}

// check if box has been shaken
bool isBoxShaken() {
  accelerometer.read();
  int prev = prevAccelReading;
  prevAccelReading = accelerometer.y;
  return accelerometer.y > 0 && prev < 0 || accelerometer.y < 0 && prev > 0;
}

// check the values of the switches, move to FLIPPED state if any are flipped
bool checkSwitches() {
  bool anyFlipped = false;
  for (int i = 0; i < 5; i++) {
    bool isFlipped = !digitalRead(switchPins[i]);
    if (isFlipped) {
      anyFlipped = true;
      if (!switchFlipped[i]) {
        // if it wasn't previously flipped, increment the total number of times flipped
        timesFlipped++;
      }
    }
    switchFlipped[i] = isFlipped;
  }
  return anyFlipped;
}

// change state to WAITING if the specified amount of time (in milliseconds) has passed
void checkTime(long threshold) {
  long curr_ms = millis();
  if (curr_ms - prev_ms > threshold) {
    stopDriving();
    prev_ms = curr_ms;
    prev_display_ms = curr_ms;
    state = WAITING;
  }
}

// if LEFT or RIGHT gesture detected, turn CLOCKWISE or COUNTERCLOCKWISE respectively
void checkAndHandleGesture() {
  uint8_t gesture = gesture_sensor.readGesture();
  Serial.println(gesture);
  if (gesture == APDS9960_LEFT) {
    turnCW();
    prev_ms = millis();
    prev_display_ms = millis();
    state = TURNING;
  } else if (gesture == APDS9960_RIGHT) {
    turnCCW();
    prev_ms = millis();
    prev_display_ms = millis();
    state = TURNING;
  }
}

// drive backwards if a hand is detected
void checkAndHandleProximity() {
  if (gesture_sensor.readProximity() > 0) {
    driveBackwards();
    state = DRIVING;
    prev_ms = millis();
  }
}

// flip switches
void flipSwitch() {
  for (int i = 0; i < 5; i++) {
//    if (switchFlipped[i]) {
//      bottomServo.write(switchAngles[i]);
//      delay(500);
//    }
//    while (switchFlipped[i]) {
//      topServo.write(armExtendedAngle);
//      delay(500);
//      switchFlipped = digitalRead(switchPins[i]);
//    }
//    topServo.write(armRetractedAngle);
//    delay(500);
  }
}

// draw OWO face
void drawOWO() {
  long curr_ms = millis();
  display.clearDisplay();
  if (curr_ms - prev_display_ms > time_to_wait) {
    if (curr_ms - prev_display_ms > time_to_wait + 100) {
      prev_display_ms = curr_ms;
      time_to_wait = random(100, 4000);
    }
    display.drawBitmap(0, 16, owo_blink, 128, 48, WHITE);
  } else {
    display.drawBitmap(0, 16, owo, 128, 48, WHITE);
  }
  display.display();
}

void drawClosedEyes() {
  long curr_ms = millis();
  display.clearDisplay();
  display.drawBitmap(0, 16, closed_eyes, 128, 48, WHITE);
  if (curr_ms - prev_display_ms > 200) {
    if (curr_ms - prev_display_ms > 400) {
      prev_display_ms = curr_ms;
    }
    for (int i = 0; i < 3; i++) {
      display.drawBitmap(12+i*9, 51+i%2, blush, 8, 8, WHITE);
      display.drawBitmap(90+i*9, 51+i%2, blush, 8, 8, WHITE);
    }
  } else {
    for (int i = 0; i < 3; i++) {
      display.drawBitmap(13+i*9, 51+i%2, blush, 8, 8, WHITE);
      display.drawBitmap(91+i*9, 51+i%2, blush, 8, 8, WHITE);
    }
  }
  display.display();
}

void drawMouthOpen() {
  long curr_ms = millis();
  display.clearDisplay();
  display.drawBitmap(0, 16, mouth_open, 128, 48, WHITE);
  if (curr_ms - prev_display_ms > 200) {
    if (curr_ms - prev_display_ms > 400) {
      prev_display_ms = curr_ms;
    }
    for (int i = 0; i < 3; i++) {
      display.drawBitmap(12+i*9, 51+i%2, blush, 8, 8, WHITE);
      display.drawBitmap(90+i*9, 51+i%2, blush, 8, 8, WHITE);
    }
  } else {
    for (int i = 0; i < 3; i++) {
      display.drawBitmap(13+i*9, 51+i%2, blush, 8, 8, WHITE);
      display.drawBitmap(91+i*9, 51+i%2, blush, 8, 8, WHITE);
    }
  }
  display.display();
}

void drawAngry() {
  long curr_ms = millis();
  display.clearDisplay();
  display.drawBitmap(0, 16, angry, 128, 48, WHITE);
  if (curr_ms - prev_display_ms > 200) {
    if (curr_ms - prev_display_ms > 400) {
      prev_display_ms = curr_ms;
    }
    display.drawBitmap(0, 0, vein, 16, 16, WHITE);
    for (int i = 0; i < 3; i++) {
      display.drawBitmap(12+i*9, 51+i%2, blush, 8, 8, WHITE);
      display.drawBitmap(90+i*9, 51+i%2, blush, 8, 8, WHITE);
    }
  } else {
    display.drawBitmap(2, 2, vein, 16, 16, WHITE);
    for (int i = 0; i < 3; i++) {
      display.drawBitmap(13+i*9, 51+i%2, blush, 8, 8, WHITE);
      display.drawBitmap(91+i*9, 51+i%2, blush, 8, 8, WHITE);
    }
  }
  display.display();
}

long period = 100;
void drawConfused() {
  long curr_ms = millis();
  display.clearDisplay();
  if (curr_ms - prev_display_ms > period*4) {
    prev_display_ms = curr_ms;
  }
  if (curr_ms - prev_display_ms < period) {
    display.drawBitmap(7, 23, confused_eye, 32, 32, WHITE);
    display.drawBitmap(87, 21, confused_eye, 32, 32, WHITE);
    display.drawBitmap(39, 46, confused_mouth, 48, 11, WHITE);
  } else if (curr_ms - prev_display_ms < period*2) {
    display.drawBitmap(7, 23, confused_eye_90, 32, 32, WHITE);
    display.drawBitmap(87, 21, confused_eye_90, 32, 32, WHITE);
    display.drawBitmap(39, 46, confused_mouth, 48, 11, WHITE);
  } else if (curr_ms - prev_display_ms < period*3) {
    display.drawBitmap(7, 23, confused_eye_180, 32, 32, WHITE);
    display.drawBitmap(87, 21, confused_eye_180, 32, 32, WHITE);
    display.drawBitmap(40, 46, confused_mouth, 48, 11, WHITE);
  } else {
    display.drawBitmap(7, 23, confused_eye_270, 32, 32, WHITE);
    display.drawBitmap(87, 21, confused_eye_270, 32, 32, WHITE);
    display.drawBitmap(40, 46, confused_mouth, 48, 11, WHITE);
  }
  display.display();
}

void stopDriving() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], LOW);
  }
}
void turnCW() {
  digitalWrite(motorPins[0], LOW);
  digitalWrite(motorPins[1], HIGH);
  digitalWrite(motorPins[2], HIGH);
  digitalWrite(motorPins[3], LOW);
}
void turnCCW() {
  digitalWrite(motorPins[0], HIGH);
  digitalWrite(motorPins[1], LOW);
  digitalWrite(motorPins[2], LOW);
  digitalWrite(motorPins[3], HIGH);
}
void driveBackwards() {
  digitalWrite(motorPins[0], HIGH);
  digitalWrite(motorPins[1], LOW);
  digitalWrite(motorPins[2], HIGH);
  digitalWrite(motorPins[3], LOW);
}
