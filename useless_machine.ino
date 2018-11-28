#include <Adafruit_LIS3DH.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_APDS9960.h>

#include "faces.h"

Adafruit_LIS3DH accelerometer = Adafruit_LIS3DH();

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

Adafruit_APDS9960 gesture_sensor;

typedef enum State {
  WAITING = 1,
  FLIPPING,
  DRIVING,
  ANGRY,
  TANTRUM,
  CONFUSED
} states;

const int switchPins[5] = {4, 5, 6, 7, 8};

bool* switchFlipped;
int timesFlipped;
int prevAccelReading;
long prev_ms;
long prev_display_ms;
long time_to_wait;
State state;
State prevState;

void setup() {

  // Program initializations
  Serial.begin(9600);
  for (int i = 0; i < 5; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);
  }
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
  gesture_sensor.enableGesture(true);
}

void loop() {

  Serial.println(state);
  
  // if box shaken, become confused
  if (isBoxShaken()) {
    if (state == DRIVING) {
      stopDriving();
      state = WAITING;
    }
    prevState = state;
    state = CONFUSED;
  }

  // if switches were flipped too much, become angry
  if (timesFlipped > 10) {
    timesFlipped = 0;
    state = ANGRY;
  }

  switch(state) {
    case WAITING: {
      drawOWO();
      checkAndHandleGesture();
      if (checkSwitches()) {
        state = FLIPPING;
      }
      break;
    }
    case FLIPPING: {
      flipSwitch();
      if (!checkSwitches()) {
        state = WAITING;
      }
      break;
    }
    case DRIVING: {
      checkTime(1000);
      break;
    }
    case ANGRY: {
      drawAngry();
      checkAndHandleProximity();
      checkTime(5000);
      break;
    }
    case TANTRUM: {
      checkTime(5000);
      break;
    }
    case CONFUSED: {
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
void checkTime(int threshold) {
  long curr_ms = millis();
  if (curr_ms - prev_ms > threshold) {
    stopDriving();
    prev_ms = curr_ms;
    state = WAITING;
  }
}

// if LEFT or RIGHT gesture detected, turn CLOCKWISE or COUNTERCLOCKWISE respectively
void checkAndHandleGesture() {
  uint8_t gesture = gesture_sensor.readGesture();
  if (gesture == APDS9960_LEFT) {
    turnCW();
    prev_ms = millis();
    state = DRIVING;
  } else if (gesture == APDS9960_RIGHT) {
    turnCCW();
    prev_ms = millis();
    state = DRIVING;
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

// flip a switch
void flipSwitch() {
  for (int i = 0; i < 5; i++) {
    if (switchFlipped[i]) {

      break;
    }
  }
}

// draw OWO face
void drawOWO() {
  long curr_ms = millis();
  if (curr_ms - prev_display_ms > time_to_wait) {
    if (curr_ms - prev_display_ms > time_to_wait + 100) {
      prev_display_ms = curr_ms;
      time_to_wait = random(100, 4000);
    }
    display.clearDisplay();
    display.drawBitmap(0, 16, owo_blink, 128, 48, WHITE);
    display.display();
  } else {
    display.clearDisplay();
    display.drawBitmap(0, 16, owo, 128, 48, WHITE);
    display.display();
  }
}

void drawAngry() {
  long curr_ms = millis();
  if (curr_ms - prev_display_ms > 200) {
    prev_display_ms = curr_ms;
    display.clearDisplay();
    display.drawBitmap(0, 16, angry, 128, 48, WHITE);
    display.drawBitmap(0, 0, vein, 16, 16, WHITE);
    for (int i = 0; i < 3; i++) {
      display.drawBitmap(12+i*9, 51+i%2, blush, 8, 8, WHITE);
      display.drawBitmap(90+i*9, 51+i%2, blush, 8, 8, WHITE);
    }
    display.display();
  } else {
    display.clearDisplay();
    display.drawBitmap(0, 16, angry, 128, 48, WHITE);
    display.drawBitmap(2, 2, vein, 16, 16, WHITE);
    for (int i = 0; i < 3; i++) {
      display.drawBitmap(13+i*9, 51+i%2, blush, 8, 8, WHITE);
      display.drawBitmap(91+i*9, 51+i%2, blush, 8, 8, WHITE);
    }
    display.display();
  }
}

void stopDriving() {
  
}
void turnCW() {
  
}
void turnCCW() {
  
}
void driveBackwards() {
  
}
