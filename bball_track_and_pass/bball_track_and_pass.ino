#include "HUSKYLENS.h"
#include "TimerEvent.h"

#include "PWMServo.h"
// The PWMServo uses PWM hardware to control the Servo on pin 12. This will ensure smooth control over
// the servo. It was been determined for my physical 'lego' stand, that 90 degrees is centered. The
// the servo can do 0 to 180 degrees.

HUSKYLENS huskylens;

int savedObjectID = 1; // Change this to the ID of the saved object you want to track

#define STOP_SPEED 90
#define START_STEP 10
#define SPEED_STEP 2
#define MANUAL_SPEED_STEP 10
int currentSpeed = STOP_SPEED;
int targetSpeed = STOP_SPEED;
int lastTarget = -1;

#define SERVO_MIN 30
#define SERVO_MAX 150

#define DIR_SERVO_PIN SERVO_PIN_B // Servo signal pin
#define DIR_SERVO_DELAY_TIME 50
TimerEvent directionServoTimer;
PWMServo directionServo;
bool servoMoving = false; // Flag to track object detection
#define CCW 1
#define CW -1
#define STOP 0
int direction = STOP;
int last_direction = STOP;
int hTrackingRestrict = STOP;

#define STATIONARY_WAIT_TIME 1500
TimerEvent stationaryTimer;
#define RESET_BALL_WAIT_TIME 1000
TimerEvent resetBallTimer;

#define BALL_GATE_SERVO_PIN SERVO_PIN_A // Servo signal pin
#define CHANGE_BALL_GATE_WAIT_TIME 10
#define GATE_MIN 0
#define GATE_MAX 50
#define GATE_STEP 2
TimerEvent changeBallGateTimer;
PWMServo ballGateServo;
int gatePosition = GATE_MIN;
int ballGateStep = GATE_STEP;

#define TRIG_PIN 9 // Define the trig pin of the ultrasonic sensor
#define ECHO_PIN 10 // Define the echo pin of the ultrasonic sensor
#define DISTANCE_THRESHOLD 4 // Define the distance threshold in centimeters
#define TRIG_TIME 100
#define TRIG_OFF_TIME 10
TimerEvent trigTimer;
TimerEvent trigOffTimer;

#define LED_PIN 13
#define LED_OFF_TIME 1000
#define LED_ON_TIME 1000
TimerEvent ledTimer;
bool ledOn = 0;
unsigned long ledOffWaitTime = 0;

#define PRINT_OFF_TIME 5000
//#define DEBUG_PIN 8  // debug signal pin
unsigned long offPrintWaitTime = 0;

bool hRequest = false;  // Assume that serial comm is not okay
bool hLearned = true;  // Assume that learning has been done
bool hAvailable = false;  // Assume nothing is available
bool hBlock = true;
bool hThresholdOn = true;
bool hThresholdOff = false;

#define SERIAL_INPUT_ENABLE_PIN 8
int serialInputEnabled = true; // on start serialInputData is used

void printResult(HUSKYLENSResult result);
void smoothMoveServo(int targetPosition);
void moveServo(int angle);
void sendCommandToHuskyLens(String command, String parameter);
void ledOff();
void adjustDirectionServo();
void trigOn();
void trigOff();
void stationaryCheck();
void resetBallGate();

void setup() {
  Serial.begin(115200);

  Serial2.begin(9600);
  while (!huskylens.begin(Serial2)) {
      Serial.println(F("Begin failed!"));
      Serial.println(F("1. Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings >> Protocol Type >> Serial 9600)"));
      Serial.println(F("2. Please recheck the connection."));
      delay(100);
      hRequest = 1;
  }

  directionServo.attach(DIR_SERVO_PIN, 500, 2500);
  pinMode(DIR_SERVO_PIN, OUTPUT);
  directionServo.write(STOP_SPEED);

  ballGateServo.attach(BALL_GATE_SERVO_PIN, 500, 2500);
  pinMode(BALL_GATE_SERVO_PIN, OUTPUT);
  ballGateServo.write(gatePosition);

  pinMode(TRIG_PIN, OUTPUT); // Set the trig pin as an output
  pinMode(ECHO_PIN, INPUT); // Set the echo pin as an input
  digitalWrite(TRIG_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure the LED is initially off

  //pinMode(DEBUG_PIN, OUTPUT);
  //digitalWrite(DEBUG_PIN, LOW);

  trigTimer.set(TRIG_TIME, &trigOn);
  trigTimer.enable();
  trigOffTimer.set(TRIG_OFF_TIME, &trigOff);
  trigOffTimer.disable();

  // onboard LED/GPIO 13 timer
  ledTimer.set(LED_ON_TIME, &ledOff);
  ledTimer.disable();

  stationaryTimer.set(STATIONARY_WAIT_TIME, &stationaryCheck);
  stationaryTimer.disable();

  resetBallTimer.set(RESET_BALL_WAIT_TIME, &resetBallGate);
  resetBallTimer.disable();

  changeBallGateTimer.set(CHANGE_BALL_GATE_WAIT_TIME, &changeBallGate);
  changeBallGateTimer.disable();

  directionServoTimer.set(DIR_SERVO_DELAY_TIME, &adjustDirectionServo);
  directionServoTimer.disable();

  // Configure HuskyLens to recognize a specific object
  sendCommandToHuskyLens("SET_RECOGNITION_MODE", "LEARNED_OBJECT_1");

  Serial.println("Setup Done!");

  pinMode(SERIAL_INPUT_ENABLE_PIN, INPUT);
  serialInputEnabled = digitalRead(SERIAL_INPUT_ENABLE_PIN);
  if (serialInputEnabled) {
    Serial.println("Serial Input Data:");
    Serial.println("   'l' - CCW");
    Serial.println("   'r' - CW");
    Serial.println("   's' - stop");
    Serial.println("   'q' - quit");
  }
}

void loop() {
  // placing these before getting the next targetSpeed above because
  // trigOffTimer can restrict targetSpeed.
  trigTimer.update();
  trigOffTimer.update();
  ledTimer.update();
  stationaryTimer.update();
  resetBallTimer.update();
  changeBallGateTimer.update();
  directionServoTimer.update();

  if (serialInputEnabled)
    targetSpeed = getSerialInputData();
  else
    targetSpeed = getHuskyLensData();

  if (offPrintWaitTime < millis()) {
    Serial.println("t: "+String(targetSpeed)+" c: "+String(currentSpeed)+" d: "+String(direction)+" r: "+String(hTrackingRestrict));
    offPrintWaitTime = millis() + PRINT_OFF_TIME;
  }

  if (targetSpeed != currentSpeed) {
    if (!servoMoving) {
      directionServoTimer.reset();
      directionServoTimer.enable();
      Serial.println("Bt: "+String(targetSpeed)+" c: "+String(currentSpeed)+" d: "+String(direction)+" r: "+String(hTrackingRestrict));
      servoMoving = true;
    }
  }
  else if (currentSpeed == STOP_SPEED) {
    if (servoMoving) {
      directionServoTimer.disable();
      servoMoving = false;
      Serial.println("Et: "+String(targetSpeed)+" c: "+String(currentSpeed)+" d: "+String(direction)+" r: "+String(hTrackingRestrict));
      stationaryTimer.reset();
      stationaryTimer.enable();
    }
  }
}

// HuskyLens will give us the current position of the target.
// It will be between 0 and 320. If < 160 we must turn left;
// if > 160 we must turn right; if 160 we are centered and so stop.
int getHuskyLensData() {
  int tSpeed = STOP_SPEED;
  if (!huskylens.request()) {
    if (hRequest) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    hRequest = false;
  }
  else if (!huskylens.isLearned()) {
    if (hLearned) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    hLearned = false;
  }
  else if (!huskylens.available()) {
    if (hAvailable) Serial.println(F("No block"));
    hAvailable = false;
    hBlock = true;
  }
  else {
    if (hBlock) Serial.println(F("Blue Block"));
    hBlock = false;
    hAvailable = true;

    tSpeed = targetSpeed;
    HUSKYLENSResult result = huskylens.read();
    printHuskyLensResult(result);

    if (result.command == COMMAND_RETURN_BLOCK && result.ID == savedObjectID) {
      int posX = result.xCenter; // Get X position of the object
      //Serial.println(posX);
      if (posX < 150) {
        // CCW
        if (tSpeed > STOP_SPEED) {
          if (tSpeed < SERVO_MAX) tSpeed += SPEED_STEP;
        }
        else
          tSpeed = STOP_SPEED + START_STEP;
      } else if (posX > 170) {
        // CW
        if (tSpeed < STOP_SPEED) {
          if (tSpeed > SERVO_MIN) tSpeed -= SPEED_STEP;
        }
        else
          tSpeed = STOP_SPEED - START_STEP;
      }
      else {
        //STOP
        tSpeed = STOP_SPEED;
      }
    }
  }
  return tSpeed;
}

// Unless the user inputs data, currentSpeed will be returned
// if 'l' then left; if 'r' then right; if 's' then stop; if 'q' disable serialInput
int getSerialInputData() {
  int tSpeed = targetSpeed;
  if (Serial.available() > 0) {
    int input = 0;
    input = Serial.read();
    if (input == 108) {
      // 'l' CCW
      if (tSpeed > STOP_SPEED) {
        if (tSpeed < SERVO_MAX) tSpeed += MANUAL_SPEED_STEP;
      }
      else
        tSpeed = STOP_SPEED + START_STEP;
    }
    else if (input == 114) {
      // 'r' CW
      if (tSpeed < STOP_SPEED) {
        if (tSpeed > SERVO_MIN) tSpeed -= MANUAL_SPEED_STEP;
      }
      else
        tSpeed = STOP_SPEED - START_STEP;
    }
    else if (input == 115) {
      // 's' STOP
      tSpeed = STOP_SPEED;
    }
    else if (input == 113) {
      tSpeed = STOP_SPEED;
      serialInputEnabled = false;
      hAvailable = true;
    }
  }
  return tSpeed;
}

void printHuskyLensResult(HUSKYLENSResult result) {
  // Your printResult function remains the same
}

void sendCommandToHuskyLens(String command, String parameter) {
  // Craft and send the command to HuskyLens
  String fullCommand = command + " " + parameter + "\n";
  Serial2.print(fullCommand);
  delay(100);  // Give time for the HuskyLens to process the command
}

// directionServoTimer runs every DIR_SERVO_DELAY_TIME and when it runs it will
// change directionServoPosition by increment and then set new directionServoPosition.
void adjustDirectionServo() {
  if (targetSpeed < currentSpeed) {
    currentSpeed -= SPEED_STEP;
    if (currentSpeed < SERVO_MIN) currentSpeed = SERVO_MIN;
  }
  else if (targetSpeed > currentSpeed) {
    currentSpeed += SPEED_STEP;
    if (currentSpeed > SERVO_MAX) currentSpeed = SERVO_MAX;
  }
  else
    return;   // no change

  if (currentSpeed < STOP_SPEED)
    direction = CW;
  else if (currentSpeed > STOP_SPEED)
    direction = CCW;
  else
    direction = STOP;

  // if tracking is restricted in current direction then stop
  if (hTrackingRestrict == direction)
    currentSpeed = STOP_SPEED;

  if (last_direction != direction) {
    Serial.println("Dt: "+String(targetSpeed)+" c: "+String(currentSpeed)+" d: "+String(direction)+" r: "+String(hTrackingRestrict));
    last_direction = direction;
  }
//  else
//    Serial.println("Nt: "+String(targetSpeed)+" c: "+String(currentSpeed)+" d: "+String(direction)+" r: "+String(hTrackingRestrict));

  // change speed
  directionServo.write(currentSpeed);
}

// Send ultrasonic trigger pulse
void trigOn() {
  digitalWrite(TRIG_PIN, HIGH); // Turn on trigger signal
  trigOffTimer.reset();
  trigOffTimer.enable();
}

void trigOff() {
  long duration, distance; // Variables to store ultrasonic sensor readings

  // Turn off trigger signal and read the echo pulse duration
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);

  trigOffTimer.disable();
  // Calculate the distance in centimeters
  distance = duration * 0.034 / 2;

  if (distance < DISTANCE_THRESHOLD) {
    // if not already restricted, restrict tracking in the current direction
    if (hTrackingRestrict == STOP)
      hTrackingRestrict = direction;

    hThresholdOff = true;

    if (hThresholdOn) {
      Serial.println("d on: "+String(distance));
      Serial.println("Tt: "+String(targetSpeed)+" c: "+String(currentSpeed)+" d: "+String(direction)+" r: "+String(hTrackingRestrict));
      hThresholdOn = false;
    }
  }
  else {
    hTrackingRestrict = STOP;
    hThresholdOn = true;

    if (hThresholdOff) {
      Serial.println("d off: "+String(distance));
      Serial.println("Tt: "+String(targetSpeed)+" c: "+String(currentSpeed)+" d: "+String(direction)+" r: "+String(hTrackingRestrict));
      hThresholdOff = false;
    }
  }
}

void ledOff()
{
  digitalWrite(LED_PIN, LOW); // Turn off the LED
  ledOn = 0;
  Serial.println("OFF");
  ledTimer.disable();
  ledOffWaitTime = millis() + LED_OFF_TIME;
}

void stationaryCheck()
{
  stationaryTimer.disable();

  // Restrict releasing the ball only if hAvailable and still STOP
  if (!hAvailable || (currentSpeed != STOP_SPEED))
    return;

  Serial.println("Release Ball");
  ballGateStep = GATE_STEP;
  changeBallGateTimer.reset();
  changeBallGateTimer.enable();
}

void resetBallGate()
{
  resetBallTimer.disable();

  Serial.println("Reset Ball Gate");
  ballGateStep = -GATE_STEP;
  changeBallGateTimer.reset();
  changeBallGateTimer.enable();
}

void changeBallGate()
{
  gatePosition += ballGateStep;
  if ((gatePosition >= GATE_MIN) && (gatePosition <= GATE_MAX))
    ballGateServo.write(gatePosition);
  else
    changeBallGateTimer.disable();

  if (gatePosition > GATE_MAX) {
    resetBallTimer.reset();
    resetBallTimer.enable();
  }
}
