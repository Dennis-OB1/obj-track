#include "HUSKYLENS.h"
#include "TimerEvent.h"

#include "PWMServo.h"
// The PWMServo uses PWM hardware to control the Servo on pin 12. This will ensure smooth control over
// the servo. It was been determined for my physical 'lego' stand, that 90 degrees is centered. The
// the servo can do 0 to 180 degrees.

HUSKYLENS huskylens;
int savedObjectID = 1; // Change this to the ID of the saved object you want to track

int servoMin = 0;
int servoMax = 180;

TimerEvent directionServoTimer;
PWMServo directionServo;
const int directionServoPin = SERVO_PIN_B; // Servo signal pin
int directionServoPosition = (servoMax-servoMin)/2; // Initial servo position
int targetPosition = directionServoPosition;
int servoStep = 1;
int increment = 0;
int lastInc = 0;
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
unsigned long printOffTime = 3000;
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
  
  directionServo.attach(directionServoPin, 500, 2500);
  directionServo.write(directionServoPosition);
  delay(1000);
  directionServo.detach();

  ballGateServo.attach(ballGateServoPin, 500, 2500);
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

  Serial.println("Setup Done!");
}

void loop() {
  ledTimer.update();
  stationaryTimer.update();
  resetBallGateTimer.update();
  directionServoTimer.update();

  if (Serial.available() > 0) {
    String input = "";
    input = Serial.readStringUntil("/n");
    targetPosition = input.toInt();
    if ((targetPosition >= servoMin) && (targetPosition <= servoMax)) {
      Serial.print("I received: ");
      Serial.println(targetPosition);
    }
    else {
      Serial.println("invalid number");
      targetPosition = directionServoPosition;
      return;
    }
  }

  if (abs(targetPosition - directionServoPosition) != 0) {
    if (targetPosition < directionServoPosition)
      increment = -servoStep;
    else
      increment = servoStep;

    if (!servoMoving) {
      directionServo.attach(directionServoPin, 500, 2500);
      directionServoTimer.reset();
      directionServoTimer.enable();
      Serial.println("Bt: "+String(targetPosition)+" s: "+String(directionServoPosition)+" i: "+String(increment));
      servoMoving = true;
    }
    else {
      if (lastInc != increment) {
        Serial.println("t: "+String(targetPosition)+" s: "+String(directionServoPosition)+" i: "+String(increment));
        lastInc = increment;
      }
    }
  }
  else {
    // If target is close (within 10?) to servo then disable servoTimer and start
    // 2 second timer. On expiration of timer, if no further movement, then swing
    // second servo.
    if (servoMoving) {
      directionServoTimer.disable();
      servoMoving = false;
      Serial.println("Et: "+String(targetPosition)+" s: "+String(directionServoPosition)+" i: "+String(increment));
      directionServo.detach();
//      stationaryTimer.reset();
//      stationaryTimer.enable();
    }
  }
}

void printResult(HUSKYLENSResult result) {
  // Your printResult function remains the same
}

// directionServoTimer runs every directionServoDelayTime and when it runs it will
// change directionServoPosition by increment and then set new directionServoPosition.
void adjustDirectionServo() {
  directionServoPosition += increment;
  if ((directionServoPosition <= servoMax) && (directionServoPosition >= servoMin))
    directionServo.write(directionServoPosition);
}

void sendCommandToHuskyLens(String command, String parameter) {
  // Craft and send the command to HuskyLens
  String fullCommand = command + " " + parameter + "\n";
  Serial2.print(fullCommand);
  delay(100);  // Give time for the HuskyLens to process the command
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
