#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "TimerEvent.h"

#include "PWMServo.h"
// The PWMServo uses PWM hardware to control the Servo on pin 12. This will ensure smooth control over
// the servo. It was been determined for my physical 'lego' stand, that 95 degrees is centered. Since
// the servo can do 0 to 180 degrees, we will limit the left/right movement to from 5 to 180 for a total
// of 175 degrees of swing.

HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX
TimerEvent myLedTimer;
TimerEvent myServoTimer;
PWMServo myservo;

const int debugPin = 8; // debug signal pin
const int servoPin = 12; // Servo signal pin
const int ledPin = 13; // LED control pin
const int relayPin = 9; // Relay control pin
const int detectionPauseDuration = 5000; // Pause duration in milliseconds

int servoPosition = 95; // Initial servo position
int servoMin = 5;
int servoMax = 180;
float servoSlope = 11.1112;
int servoOffset = 500;

int targetPosition = 0;
bool objectDetected = false; // Flag to track object detection

int savedObjectID = 1; // Change this to the ID of the saved object you want to track

bool hRequest = 0;  // Assume that serial comm is not okay
bool hLearned = 1;  // Assume that learning has been done
bool hAvailable = 0;  // Assume nothing is available
bool ledOn = 0;
unsigned long offWaitTime = 0;
unsigned long now = 0;
unsigned long ledOffTime = 1000;
unsigned long ledOnTime = 1000;
unsigned long servoDelayTime = 100;
int servoStep = 1;
int pos = 0;

void printResult(HUSKYLENSResult result);
void smoothMoveServo(int targetPosition);
void moveServo(int angle);
void sendCommandToHuskyLens(String command, String parameter);
void ledOff();
void adjustServo();

void setup() {
  Serial.begin(115200);

  myservo.attach(servoPin, servoMin*servoSlope+servoOffset, servoMax*servoSlope+servoOffset);
  myservo.write(servoPosition); // move to a centered position

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Ensure the LED is initially off
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Ensure the relay is initially off
  pinMode(debugPin, OUTPUT);
  digitalWrite(debugPin, LOW);

  myLedTimer.set(ledOnTime, &ledOff);
  myLedTimer.disable();

  myServoTimer.set(servoDelayTime, &adjustServo);
  myServoTimer.disable();

  Serial.println("Setup Done!");
}

void loop() {
  myLedTimer.update();
  int posX = 0;

  if (Serial.available() > 0) {
    String input = "";
    input = Serial.readStringUntil("/n");
    posX = input.toInt();
    if ((posX >= servoMin) && (posX <= servoMax))
    {
      Serial.print("I received: ");
      Serial.println(posX);
      hAvailable = 1;
    }
    else
      Serial.println("invalid number");
  }

  if (!hAvailable) {
  }
  else {
    hAvailable = 0;
    // Invert the mapping to align servo movement with object position
    //targetPosition = map(posX, 0, 320, 180, 0);
    targetPosition = posX;
    Serial.println(targetPosition);
    myservo.write(targetPosition);

    if (!ledOn && (offWaitTime < millis())) {
      digitalWrite(ledPin, HIGH); // Turn on the LED
      digitalWrite(relayPin, HIGH); // Turn on the relay to activate the external light
      Serial.println("Both HIGH!");
      ledOn = 1;
      offWaitTime = 0;
      myLedTimer.reset();
      myLedTimer.enable();
    }
  }
}

void printResult(HUSKYLENSResult result) {
  // Your printResult function remains the same
}

void adjustServo() {
  int increment = (targetPosition - servoPosition > 0) ? servoStep : -servoStep;
  if (increment == servoStep)
    digitalWrite(debugPin, HIGH);
  if (servoPosition != targetPosition) {
    servoPosition += increment;
    moveServo(servoPosition);
  }
  else {
    digitalWrite(debugPin, HIGH);
    myServoTimer.disable();
  }
}

void moveServo(int angle) {
  int pulseWidth = map(angle, 0, 180, 1000, 2000); // Convert angle to pulse width
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(servoPin, LOW);
}

void sendCommandToHuskyLens(String command, String parameter) {
  // Craft and send the command to HuskyLens
  String fullCommand = command + " " + parameter + "\n";
  mySerial.print(fullCommand);
  delay(100);  // Give time for the HuskyLens to process the command
}

void ledOff()
{
  digitalWrite(ledPin, LOW); // Turn off the LED
  digitalWrite(relayPin, LOW); // Turn off the relay to deactivate the external light
  ledOn = 0;
  Serial.println("Both LOW!");
  myLedTimer.disable();
  offWaitTime = millis() + ledOffTime;
}
