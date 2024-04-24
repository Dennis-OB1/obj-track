#include "HUSKYLENS.h"
#include "TimerEvent.h"

#include "PWMServo.h"
// The PWMServo uses PWM hardware to control the Servo on pin 12. This will ensure smooth control over
// the servo. It was been determined for my physical 'lego' stand, that 90 degrees is centered. The
// the servo can do 0 to 180 degrees.

HUSKYLENS huskylens;
//#define HUSKYLENS

int savedObjectID = 1; // Change this to the ID of the saved object you want to track

#define STOP_SPEED 90
#define SPEED_STEP 5
int curSpeed = STOP_SPEED;
int targetSpeed = STOP_SPEED;
int lastTarget = -1;

#define SERVO_MIN 0
#define SERVO_MAX 180

#define DIR_SERVO_PIN SERVO_PIN_B // Servo signal pin
#define DIR_SERVO_DELAY_TIME 20
TimerEvent directionServoTimer;
PWMServo directionServo;
bool servoMoving = false; // Flag to track object detection

#define STATIONARY_WAIT_TIME 2000
TimerEvent stationaryTimer;

#define BALL_GATE_SERVO_PIN SERVO_PIN_A // Servo signal pin
#define RESET_BALL_WAIT_TIME 1000
TimerEvent resetBallGateTimer;
PWMServo ballGateServo;

#define TRIG_PIN 9 // Define the trig pin of the ultrasonic sensor
#define ECHO_PIN 10 // Define the echo pin of the ultrasonic sensor
#define DISTANCE_THRESHOLD 0 // Define the distance threshold in centimeters
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
#define DEBUG_PIN 8  // debug signal pin
unsigned long offPrintWaitTime = 0;

bool hRequest = 0;  // Assume that serial comm is not okay
bool hLearned = 1;  // Assume that learning has been done
bool hAvailable = 1;  // Assume nothing is available
bool trackingStop = 0;

void printResult(HUSKYLENSResult result);
void smoothMoveServo(int targetPosition);
void moveServo(int angle);
void sendCommandToHuskyLens(String command, String parameter);
void ledOff();
void adjustDirectionServo();

void setup() {
  Serial.begin(115200);

#ifdef HUSKYLENS
  Serial2.begin(9600);
  while (!huskylens.begin(Serial2)) {
      Serial.println(F("Begin failed!"));
      Serial.println(F("1. Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings >> Protocol Type >> Serial 9600)"));
      Serial.println(F("2. Please recheck the connection."));
      delay(100);
      hRequest = 1;
  }
#endif

  directionServo.attach(DIR_SERVO_PIN, 500, 2500);
  pinMode(DIR_SERVO_PIN, OUTPUT);
  directionServo.write(STOP_SPEED);

  ballGateServo.attach(BALL_GATE_SERVO_PIN, 500, 2500);
  pinMode(BALL_GATE_SERVO_PIN, OUTPUT);
  ballGateServo.write(SERVO_MIN);

  pinMode(TRIG_PIN, OUTPUT); // Set the trig pin as an output
  pinMode(ECHO_PIN, INPUT); // Set the echo pin as an input
  digitalWrite(TRIG_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure the LED is initially off

  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, LOW);

  trigTimer.set(TRIG_TIME, &trigOn);
  trigTimer.enable();
  trigOffTimer.set(TRIG_OFF_TIME, &trigOff);
  trigOffTimer.disable();

  // onboard LED/GPIO 13 timer
  ledTimer.set(LED_ON_TIME, &ledOff);
  ledTimer.disable();

  stationaryTimer.set(STATIONARY_WAIT_TIME, &releaseBall);
  stationaryTimer.disable();

  resetBallGateTimer.set(RESET_BALL_WAIT_TIME, &resetBallGate);
  resetBallGateTimer.disable();

  directionServoTimer.set(DIR_SERVO_DELAY_TIME, &adjustDirectionServo);
  directionServoTimer.disable();

#ifdef HUSKYLENS
  // Configure HuskyLens to recognize a specific object
  sendCommandToHuskyLens("SET_RECOGNITION_MODE", "LEARNED_OBJECT_1");
#endif

  Serial.println("Setup Done!");
}

void loop() {
  trigTimer.update();
  trigOffTimer.update();
  ledTimer.update();
  stationaryTimer.update();
  resetBallGateTimer.update();
  directionServoTimer.update();

#ifdef HUSKYLENS
  targetSpeed = getHuskyLensData();
#else
  targetSpeed = getSerialInputData();
#endif

  if (trackingStop)
    targetSpeed = STOP_SPEED;

  if (offPrintWaitTime < millis()) {
    Serial.println("t: "+String(targetSpeed)+" c: "+String(curSpeed));
    offPrintWaitTime = millis() + PRINT_OFF_TIME;
  }

  if (targetSpeed != curSpeed) {
    if (!servoMoving) {
      directionServoTimer.reset();
      directionServoTimer.enable();
      Serial.println("Bt: "+String(targetSpeed)+" c: "+String(curSpeed));
      servoMoving = true;
      lastTarget = targetSpeed;
    }
    else if (lastTarget != targetSpeed) {
      Serial.println("Ct: "+String(targetSpeed)+" c: "+String(curSpeed));
      lastTarget = targetSpeed;
    }
  }

#if 0
  else {
    // If target is close (within 10?) to servo then disable servoTimer and start
    // 2 second timer. On expiration of timer, if no further movement, then swing
    // second servo.
    if (servoMoving) {
      directionServoTimer.disable();
      servoMoving = false;
      Serial.println("Et: "+String(targetPosition)+" s: "+String(directionServoPosition)+" i: "+String(increment));
      //directionServo.detach();
//      stationaryTimer.reset();
//      stationaryTimer.enable();
    }
  }
#endif
}

// HuskyLens will give us the current position of the target.
// It will be between 0 and 320. If < 160 we must turn left;
// if > 160 we must turn right; if 160 we are centered and so stop.
int getHuskyLensData() {
  int tSpeed = STOP_SPEED;
  if (!huskylens.request()) {
    if (hRequest) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    hRequest = 0;
  }
  else if (!huskylens.isLearned()) {
    if (hLearned) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    hLearned = 0;
  }
  else if (!huskylens.available()) {
    if (hAvailable) Serial.println(F("No block"));
    hAvailable = 0;
  }
  else {
    tSpeed = targetSpeed;
    hAvailable = 1;
    HUSKYLENSResult result = huskylens.read();
    printHuskyLensResult(result);

    if (result.command == COMMAND_RETURN_BLOCK && result.ID == savedObjectID) {
      int posX = result.xCenter; // Get X position of the object
      if (posX < 160) {
        if (tSpeed > STOP_SPEED) {
          if (tSpeed < SERVO_MAX) tSpeed += SPEED_STEP;
        }
        else
          tSpeed = STOP_SPEED + (2*SPEED_STEP);
      } else if (posX > 160) {
        if (tSpeed < STOP_SPEED) {
          if (tSpeed > SERVO_MIN) tSpeed -= SPEED_STEP;
        }
        else
          tSpeed = STOP_SPEED - (2*SPEED_STEP);
      }
      else {
        tSpeed = STOP_SPEED;
      }
    }
  }
  return tSpeed;
}

// Unless the user inputs data, curSpeed will be returned
// if 'l' then left; if 'r' then right; if 's' then stop
int getSerialInputData() {
  int tSpeed = targetSpeed;
  if (Serial.available() > 0) {
    int input = 0;
    input = Serial.read();
    if (input == 108) {
      if (tSpeed > STOP_SPEED) {
        if (tSpeed < SERVO_MAX) tSpeed += SPEED_STEP;
      }
      else
        tSpeed = STOP_SPEED + (2*SPEED_STEP);
    }
    else if (input == 114) {
      if (tSpeed < STOP_SPEED) {
        if (tSpeed > SERVO_MIN) tSpeed -= SPEED_STEP;
      }
      else
        tSpeed = STOP_SPEED - (2*SPEED_STEP);
    }
    else if (input == 115) {
      tSpeed = STOP_SPEED;
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
  if (targetSpeed < curSpeed)
    curSpeed -= SPEED_STEP;
  else if (targetSpeed > curSpeed)
    curSpeed += SPEED_STEP;
  else
    return;
  if ((curSpeed <= SERVO_MAX) && (curSpeed  >= SERVO_MIN))
    directionServo.write(curSpeed);
}

// Send ultrasonic trigger pulse
void trigOn() {
  digitalWrite(TRIG_PIN, HIGH); // Turn on trigger signal
  trigOffTimer.reset();
  trigOffTimer.enable();
}

void trigOff() {
  long duration, distance; // Variables to store ultrasonic sensor readings

  digitalWrite(TRIG_PIN, LOW); // Turn off trigger signal
  trigOffTimer.disable();

  // Read the echo pulse duration
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculate the distance in centimeters
  distance = duration * 0.034 / 2;

  if (distance < DISTANCE_THRESHOLD) {
    if (!trackingStop)
      trackingStop = 1;
    else
      trackingStop = 0;
    Serial.println(distance);
  }

#if 0
  // Check if the object is within 5 centimeters
  if (distance < DISTANCE_THRESHOLD) {
    // Turn on the LED
    digitalWrite(LED_PIN, HIGH);
  } else {
    // Turn off the LED
    digitalWrite(LED_PIN, LOW);
  }
#endif
}

void ledOff()
{
  digitalWrite(LED_PIN, LOW); // Turn off the LED
  ledOn = 0;
  Serial.println("OFF");
  ledTimer.disable();
  ledOffWaitTime = millis() + LED_OFF_TIME;
}

void releaseBall()
{
  stationaryTimer.disable();
  // run second servo full swing, wait, and then swing back.
  ballGateServo.write(SERVO_MAX);
  resetBallGateTimer.reset();
  resetBallGateTimer.enable();
}

void resetBallGate()
{
  resetBallGateTimer.disable();
  ballGateServo.write(SERVO_MIN);
}
