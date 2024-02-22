#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "TimerEvent.h"

HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX
TimerEvent myLedTimer;
TimerEvent myServoTimer;

const int servoPin = 9; // Servo signal pin
const int ledPin = 13; // LED control pin
const int relayPin = 12; // Relay control pin
const int detectionPauseDuration = 5000; // Pause duration in milliseconds

int servoPosition = 90; // Initial servo position
int targetPosition = 0;
bool objectDetected = false; // Flag to track object detection

int savedObjectID = 1; // Change this to the ID of the saved object you want to track

bool hRequest = 0;  // Assume that serial comm is not okay
bool hLearned = 1;  // Assume that learning has been done
bool hAvailable = 1;  // Assume nothing is available
bool ledOn = 0;
unsigned long offWaitTime = 0;
unsigned long now = 0;
unsigned long ledOffTime = 1000;
unsigned long ledOnTime = 1000;
unsigned long servoDelayTime = 10;

void printResult(HUSKYLENSResult result);
void smoothMoveServo(int targetPosition);
void moveServo(int angle);
void sendCommandToHuskyLens(String command, String parameter);
void ledOff();
void adjustServo();

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);

  while (!huskylens.begin(mySerial)) {
      Serial.println(F("Begin failed!"));
      Serial.println(F("1. Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings >> Protocol Type >> Serial 9600)"));
      Serial.println(F("2. Please recheck the connection."));
      delay(100);
      hRequest = 1;
  }
  pinMode(servoPin, OUTPUT);

  digitalWrite(ledPin, LOW); // Ensure the LED is initially off
  pinMode(ledPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Ensure the relay is initially off

  myLedTimer.set(ledOnTime, &ledOff);
  myLedTimer.disable();

  myServoTimer.set(servoDelayTime, &adjustServo);
  myServoTimer.disable();

  // Configure HuskyLens to recognize a specific object
  sendCommandToHuskyLens("SET_RECOGNITION_MODE", "LEARNED_OBJECT_1");
  Serial.println("Setup Done!");
}

void loop() {
  myLedTimer.update();
  myServoTimer.update();

  if (!huskylens.request()) {
    if (hRequest) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    hRequest = 0;
    return;
  }
  else if (!huskylens.isLearned()) {
    if (hLearned) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    hLearned = 0;
    return;
  }
  else if (!huskylens.available()) {
    if (hAvailable) Serial.println(F("No block or arrow appears on the screen!"));
    hAvailable = 0;
    return;
  }
  else {
    hAvailable = 1;
    do {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);

      if (result.command == COMMAND_RETURN_BLOCK && result.ID == savedObjectID) {
        objectDetected = true; // Set object detection flag
        int posX = result.xCenter; // Get X position of the object

        // Invert the mapping to align servo movement with object position
        targetPosition = map(posX, 0, 320, 180, 0);
        myServoTimer.enable();

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
    while (huskylens.available());
  }
}

void printResult(HUSKYLENSResult result) {
  // Your printResult function remains the same
}

void adjustServo() {
  int increment = (targetPosition - servoPosition > 0) ? 1 : -1;
  if (servoPosition != targetPosition) {
    servoPosition += increment;
    moveServo(servoPosition);
  }
  else
    myServoTimer.disable();
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
