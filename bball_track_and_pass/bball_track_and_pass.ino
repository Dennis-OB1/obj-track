#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "TimerEvent.h"

#include "PWMServo.h"
// The PWMServo uses PWM hardware to control the Servo on pin 12. This will ensure smooth control over
// the servo. It was been determined for my physical 'lego' stand, that 90 degrees is centered. The
// the servo can do 0 to 180 degrees.

HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX
TimerEvent myLedTimer;
TimerEvent myStationaryTimer;
TimerEvent myDirServoTimer;
PWMServo myservo;

const int debugPin = 8; // debug signal pin
const int dirServoPin = SERVO_PIN_B; // Servo signal pin
const int ledPin = 13; // LED control pin
const int detectionPauseDuration = 5000; // Pause duration in milliseconds

int dirServoPosition = 90; // Initial servo position
int servoMin = 5;
int servoMax = 180;
float servoSlope = 11.1112;
int servoOffset = 500;

int targetPosition = 0;
int lastTarget = 0;
int increment = 0;
int lastInc = 0;
bool servoMoving = false; // Flag to track object detection

int savedObjectID = 1; // Change this to the ID of the saved object you want to track

bool hRequest = 0;  // Assume that serial comm is not okay
bool hLearned = 1;  // Assume that learning has been done
bool hAvailable = 1;  // Assume nothing is available
bool ledOn = 0;
unsigned long offWaitTime = 0;
unsigned long offPrintTime = 0;
unsigned long printOffTime = 3000;
unsigned long now = 0;
unsigned long ledOffTime = 1000;
unsigned long ledOnTime = 1000;
unsigned long stationaryWaitTime = 2000;
unsigned long dirServoDelayTime = 15;
int servoStep = 1;

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
  myservo.attach(dirServoPin, 500, 2500);
  pinMode(dirServoPin, OUTPUT);
  myservo.write(dirServoPosition);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); // Ensure the LED is initially off
  pinMode(debugPin, OUTPUT);
  digitalWrite(debugPin, LOW);

  // onboard LED/GPIO 13 timer
  myLedTimer.set(ledOnTime, &ledOff);
  myLedTimer.disable();

  myStationaryTimer.set(stationaryWaitTime, &throwBall);
  myStationaryTimer.disable();

  myDirServoTimer.set(dirServoDelayTime, &adjustServo);
  myDirServoTimer.disable();

  // Configure HuskyLens to recognize a specific object
  sendCommandToHuskyLens("SET_RECOGNITION_MODE", "LEARNED_OBJECT_1");

  Serial.println("Setup Done!");
}

void loop() {
  myLedTimer.update();
  myStationaryTimer.update();
  myDirServoTimer.update();

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
    if (hAvailable) Serial.println(F("No block"));
    hAvailable = 0;
    if (servoMoving)
      myDirServoTimer.disable();
    servoMoving = false;
    lastTarget = 0;
    return;
  }
  else {
    hAvailable = 1;
    do {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);

      if (result.command == COMMAND_RETURN_BLOCK && result.ID == savedObjectID) {
        int posX = result.xCenter; // Get X position of the object
        
        // Invert the mapping to align servo movement with object position
        targetPosition = map(posX, 0, 320, 180, 0);

        if (offPrintTime < millis()) {
          Serial.println("t: "+String(targetPosition)+" c: "+String(dirServoPosition));
          offPrintTime = millis() + printOffTime;
        }

        // The below will run every loop time if a object is detected.
        if (abs(targetPosition - dirServoPosition) >= 10) {
          // Start moving the servo toward to targetPosition
          if (targetPosition < dirServoPosition)
            increment = -servoStep;
          else
            increment = servoStep;

          if (!servoMoving) {
            myDirServoTimer.reset();
            myDirServoTimer.enable();
            Serial.println("Bt: "+String(targetPosition)+" c: "+String(dirServoPosition)+" i: "+String(increment));
            servoMoving = true;
          }
          else {
            if (lastInc != increment) {
              Serial.println("t: "+String(targetPosition)+" c: "+String(dirServoPosition)+" i: "+String(increment));
              lastInc = increment;
            }
          }
        }
        else {
          // If target is close (within 10?) to servo then disable servoTimer and start
          // 2 second timer. On expiration of timer, if no further movement, then swing
          // second servo.
          if (servoMoving) {
            myDirServoTimer.disable();
            servoMoving = false;
            Serial.println("Et: "+String(targetPosition)+" c: "+String(dirServoPosition)+" i: "+String(increment));
            myStationaryTimer.reset();
            myStationaryTimer.enable();
          }
        }
        
        if (!ledOn && (offWaitTime < millis())) {
          digitalWrite(ledPin, HIGH); // Turn on the LED
          Serial.println("ON");
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

// myDirServoTimer runs every dirServoDelayTime and when it runs it will
// change dirServoPosition by increment and then set new dirServoPosition.
void adjustServo() {
  dirServoPosition += increment;
  myservo.write(dirServoPosition);
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
  ledOn = 0;
  Serial.println("OFF");
  myLedTimer.disable();
  offWaitTime = millis() + ledOffTime;
}

void throwBall()
{
  myStationaryTimer.disable();
  // run second servo full swing, wait, and then swing back.
  //myThrowTimer.reset();
  //myThrowTimer.enable();
}

void unthrowBall()
{
  //myThrowTimer.disable();
}
