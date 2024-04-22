#include "HUSKYLENS.h"
#include "TimerEvent.h"

#include "PWMServo.h"
// The PWMServo uses PWM hardware to control the Servo on pin 12. This will ensure smooth control over
// the servo. It was been determined for my physical 'lego' stand, that 90 degrees is centered. The
// the servo can do 0 to 180 degrees.

HUSKYLENS huskylens;
#define HUSKYLENS

int savedObjectID = 1; // Change this to the ID of the saved object you want to track

int stopSpeed = 90;
int speedStep = 5;
int curSpeed = stopSpeed;
int targetSpeed = stopSpeed;
int lastTarget = -1;

int servoMin = 0;
int servoMax = 180;

TimerEvent directionServoTimer;
PWMServo directionServo;
const int directionServoPin = SERVO_PIN_B; // Servo signal pin
bool servoMoving = false; // Flag to track object detection
unsigned long directionServoDelayTime = 20;

TimerEvent stationaryTimer;
unsigned long stationaryWaitTime = 2000;

TimerEvent resetBallGateTimer;
PWMServo ballGateServo;
const int ballGateServoPin = SERVO_PIN_A; // Servo signal pin
unsigned long resetBallWaitTime = 1000;

const int ledPin = 13; // LED control pin
TimerEvent ledTimer;
bool ledOn = 0;
unsigned long ledOffWaitTime = 0;
unsigned long ledOffTime = 1000;
unsigned long ledOnTime = 1000;

unsigned long offPrintTime = 0;
unsigned long printOffTime = 5000;
const int debugPin = 8; // debug signal pin

bool hRequest = 0;  // Assume that serial comm is not okay
bool hLearned = 1;  // Assume that learning has been done
bool hAvailable = 1;  // Assume nothing is available

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

  directionServo.attach(directionServoPin, 500, 2500);
  pinMode(directionServoPin, OUTPUT);
  directionServo.write(stopSpeed);

  ballGateServo.attach(ballGateServoPin, 500, 2500);
  pinMode(ballGateServoPin, OUTPUT);
  ballGateServo.write(servoMin);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Ensure the LED is initially off
  pinMode(debugPin, OUTPUT);
  digitalWrite(debugPin, LOW);

  // onboard LED/GPIO 13 timer
  ledTimer.set(ledOnTime, &ledOff);
  ledTimer.disable();

  stationaryTimer.set(stationaryWaitTime, &releaseBall);
  stationaryTimer.disable();

  resetBallGateTimer.set(resetBallWaitTime, &resetBallGate);
  resetBallGateTimer.disable();

  directionServoTimer.set(directionServoDelayTime, &adjustDirectionServo);
  directionServoTimer.disable();

#ifdef HUSKYLENS
  // Configure HuskyLens to recognize a specific object
  sendCommandToHuskyLens("SET_RECOGNITION_MODE", "LEARNED_OBJECT_1");
#endif

  Serial.println("Setup Done!");
}

void loop() {
  ledTimer.update();
  stationaryTimer.update();
  resetBallGateTimer.update();
  directionServoTimer.update();

#ifdef HUSKYLENS
  targetSpeed = getHuskyLensData();
#else
  targetSpeed = getSerialInputData();
#endif

  if (offPrintTime < millis()) {
    Serial.println("t: "+String(targetSpeed)+" c: "+String(curSpeed));
    offPrintTime = millis() + printOffTime;
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
return;

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
  int tSpeed = stopSpeed;
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
        if (tSpeed > stopSpeed) {
          if (tSpeed < servoMax) tSpeed += speedStep;
        }
        else
          tSpeed = stopSpeed + (2*speedStep);
      } else if (posX > 160) {
        if (tSpeed < stopSpeed) {
          if (tSpeed > servoMin) tSpeed -= speedStep;
        }
        else
          tSpeed = stopSpeed - (2*speedStep);
      }
      else {
        tSpeed = stopSpeed;
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
      if (tSpeed > stopSpeed)
        if (tSpeed < servoMax) tSpeed += speedStep;
      else
        tSpeed = stopSpeed + (2*speedStep);
    }
    else if (input == 114) {
      if (tSpeed < stopSpeed)
        if (tSpeed > servoMin) tSpeed -= speedStep;
      else
        tSpeed = stopSpeed - (2*speedStep);
    }
    else if (input == 115) {
      tSpeed = stopSpeed;
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

// directionServoTimer runs every directionServoDelayTime and when it runs it will
// change directionServoPosition by increment and then set new directionServoPosition.
void adjustDirectionServo() {
  if (targetSpeed < curSpeed)
    curSpeed -= speedStep;
  else if (targetSpeed > curSpeed)
    curSpeed += speedStep;
  else
    return;
  if ((curSpeed <= servoMax) && (curSpeed  >= servoMin))
    directionServo.write(curSpeed);
}

void ledOff()
{
  digitalWrite(ledPin, LOW); // Turn off the LED
  ledOn = 0;
  Serial.println("OFF");
  ledTimer.disable();
  ledOffWaitTime = millis() + ledOffTime;
}

void releaseBall()
{
  stationaryTimer.disable();
  // run second servo full swing, wait, and then swing back.
  ballGateServo.write(servoMax);
  resetBallGateTimer.reset();
  resetBallGateTimer.enable();
}

void resetBallGate()
{
  resetBallGateTimer.disable();
  ballGateServo.write(servoMin);
}
