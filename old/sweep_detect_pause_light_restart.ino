#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX

const int servoPin = 9; // Servo signal pin
const int ledPin = 13; // LED control pin
const int relayPin = 12; // Relay control pin

bool objectDetected = false; // Flag to track object detection
int savedObjectID = 1; // Change this to the ID of the saved object you want to track
int servoPosition = 0; // Current position of the servo
int servoDelay = 10; // Increased delay between steps for slower sweep
unsigned long lastDetectionTime = 0; // Variable to store the last time an object was detected
const int detectionPauseDuration = 5000; // Pause duration in milliseconds

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
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); // Ensure the LED is initially off
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW); // Ensure the relay is initially off
}

void loop() {
    // Perform the servo sweep forward
    for (servoPosition = 0; servoPosition <= 180; servoPosition++) {
        moveServo(servoPosition);
        delay(servoDelay);

        // Check for object detection during sweep
        if (detectObject()) {
            lastDetectionTime = millis(); // Record the time of the last detection
            objectDetected = true; // Set object detection flag
            digitalWrite(ledPin, HIGH); // Turn on the LED
            digitalWrite(relayPin, HIGH); // Turn on the relay to activate the external light
            delay(1000); // Keep the LED and external light on for 1 second
            digitalWrite(ledPin, LOW); // Turn off the LED
            digitalWrite(relayPin, LOW); // Turn off the relay to deactivate the external light
            delay(detectionPauseDuration - 1000); // Wait for the remaining duration after turning off the LED and light
            break; // Break the sweep if an object is detected
        }
    }

    // Perform the servo sweep backward
    for (servoPosition = 180; servoPosition >= 0; servoPosition--) {
        moveServo(servoPosition);
        delay(servoDelay);

        // Check for object detection during sweep
        if (detectObject()) {
            lastDetectionTime = millis(); // Record the time of the last detection
            objectDetected = true; // Set object detection flag
            digitalWrite(ledPin, HIGH); // Turn on the LED
            digitalWrite(relayPin, HIGH); // Turn on the relay to activate the external light
            delay(1000); // Keep the LED and external light on for 1 second
            digitalWrite(ledPin, LOW); // Turn off the LED
            digitalWrite(relayPin, LOW); // Turn off the relay to deactivate the external light
            delay(detectionPauseDuration - 1000); // Wait for the remaining duration after turning off the LED and light
            break; // Break the sweep if an object is detected
        }
    }

    if (objectDetected && millis() - lastDetectionTime > 2000) {
        objectDetected = false; // Reset object detection flag after 2 seconds without detection
    }
}

void moveServo(int angle) {
    int pulseWidth = map(angle, 0, 180, 1000, 2000); // Convert angle to pulse width
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servoPin, LOW);
}

bool detectObject() {
    HUSKYLENSResult result;

    // Request HuskyLens data
    if (!huskylens.request()) {
        Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
        return false;
    }

    // Check for available data
    if (!huskylens.available()) {
        Serial.println(F("No block or arrow appears on the screen!"));
        return false;
    }

    // Read HuskyLens data
    result = huskylens.read();

    if (result.command == COMMAND_RETURN_BLOCK && result.ID == savedObjectID) {
        return true;
    } else {
        return false;
    }
}
