#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX

const int servoPin = 9; // Servo signal pin
int servoPosition = 90; // Initial servo position
bool objectDetected = false; // Flag to track object detection

int savedObjectID = 2; // Change this to the ID of the saved object you want to track

void printResult(HUSKYLENSResult result);
void smoothMoveServo(int targetPosition);
void moveServo(int angle);
void sendCommandToHuskyLens(String command, String parameter);

void setup() {
    Serial.begin(115200);
    mySerial.begin(9600);
    while (!huskylens.begin(mySerial)) {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1. Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings >> Protocol Type >> Serial 9600)"));
        Serial.println(F("2. Please recheck the connection."));
        delay(100);
    }
    pinMode(servoPin, OUTPUT);

    // Configure HuskyLens to recognize a specific object
    sendCommandToHuskyLens("SET_RECOGNITION_MODE", "LEARNED_OBJECT_1");
}

void loop() {
    if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    else if (!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    else if (!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else {
        while (huskylens.available()) {
            HUSKYLENSResult result = huskylens.read();
            printResult(result);

            if (result.command == COMMAND_RETURN_BLOCK && result.ID == savedObjectID) {
                objectDetected = true;
                int posX = result.xCenter; // Get X position of the object

                // Invert the mapping to align servo movement with object position
                int targetPosition = map(posX, 0, 320, 180, 0);
                smoothMoveServo(targetPosition);
            }
        }
    }
}

void printResult(HUSKYLENSResult result) {
    // Your printResult function remains the same
}

void smoothMoveServo(int targetPosition) {
    int increment = (targetPosition - servoPosition > 0) ? 1 : -1;
    while (servoPosition != targetPosition) {
        servoPosition += increment;
        moveServo(servoPosition);
        delay(10); // Adjust this delay for smoother movement
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
    Serial.print(fullCommand);
    delay(100);  // Give time for the HuskyLens to process the command
}
