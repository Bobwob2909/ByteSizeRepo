/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

// Bare minimum code for spinning motors triggered by controller input

// Assumes servo is connected to pin 15

// Assumes motor controller IN1 and IN2 are connected to pins 14 and 12

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <bits/stdc++.h>
#include <Arduino_APDS9960.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>


// DC Motor Vars
#define LEFTMOTOR1 12 //GPIO Pin 
#define LEFTMOTOR2 14 //GPIO Pin 
#define LEFTMOTORPOWER 16
#define RIGHTMOTOR1 27 //GPIO Pin 
#define RIGHTMOTOR2 26 //GPIO Pin 
#define RIGHTMOTORPOWER 17

// Color Sensor Vars
#define APDS9960_INT 0
#define I2C_SDA 22 //GPIO Pin 
#define I2C_SLC 21 //GPIO Pin 
#define I2C_FREQ 100000

// LED Vars
#define LED1 0 // GPIO Pin
#define LED2 15 // GPIO Pin

// OBJECTS //
// Servo
Servo servo;

// Controller
GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// Color Sensor
TwoWire I2C_0 = TwoWire(0);
APDS9960 colorSensor = APDS9960(I2C_0, APDS9960_INT);

// IR Sensor (Distance Sensor)
ESP32SharpIR frontSensor(ESP32SharpIR::GP2Y0A21YK0F, 13); //front sensor
ESP32SharpIR leftSensor(ESP32SharpIR::GP2Y0A21YK0F, 15); //left sensor sensor
ESP32SharpIR rightSensor(ESP32SharpIR::GP2Y0A21YK0F, 4); // right sensor


// Line Follower
QTRSensors qtr;
uint16_t sensors[6];

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}


void setup() {
    // Controller Setup
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    // Servo Setup
     servo.attach(25);

    // Motor Setup
    pinMode(LEFTMOTOR1, OUTPUT);
    pinMode(LEFTMOTOR2, OUTPUT);
    pinMode(LEFTMOTORPOWER, OUTPUT);
    pinMode(RIGHTMOTOR1, OUTPUT);
    pinMode(RIGHTMOTOR2, OUTPUT);
    pinMode(RIGHTMOTORPOWER, OUTPUT);

    // ESP32 Light Setup
    pinMode(2, OUTPUT);
    // pinMode(LED1, OUTPUT);
    // pinMode(LED2, OUTPUT);

    // Color Sensor Setup
    I2C_0.begin(I2C_SDA, I2C_SLC, I2C_FREQ);
    colorSensor.setInterruptPin(APDS9960_INT);
    colorSensor.begin();

    // IR Sensor (Distance Sensor) Setup
    // frontSensor.setFilterRate(1.0f);
    // leftSensor.setFilterRate(1.0f);
    // rightSensor.setFilterRate(1.0f);

    // Line Follower Setup
    qtr.setTypeAnalog(); // or setTypeAnalog()
    qtr.setSensorPins((const uint8_t[]) {36, 39, 34, 35, 32, 33}, 6); // pin numbers go in the curly brackets {}, and number of pins goes after
    // calibration sequence
    for (uint8_t i = 0; i < 250; i++) { 
        Serial.println("calibrating");
        qtr.calibrate(); 
        delay(20);
    }
    
   
    Serial.begin(115200);
}

void motorTest(){
    Serial.println("Forward");
    digitalWrite(LEFTMOTOR1, HIGH);
    digitalWrite(LEFTMOTOR2, LOW);
    digitalWrite(RIGHTMOTOR1, LOW);
    digitalWrite(RIGHTMOTOR2, HIGH);

    delay(2000);

    Serial.println(" DC motor stop");
    digitalWrite(LEFTMOTOR1, LOW);
    digitalWrite(LEFTMOTOR2, LOW);
    digitalWrite(RIGHTMOTOR1, LOW);
    digitalWrite(RIGHTMOTOR2, LOW);

    delay(2000);
}

void servoTest(){
    
}

void colorSenseTest(){
    digitalWrite(2, HIGH);
    while(!colorSensor.colorAvailable()) {
        delay(5);
    }

    int r, g, b, a;
    colorSensor.readColor(r, g, b, a);

    // Print Colors
    
    // Serial.print("r = ");
    // Serial.println(r);
    // Serial.print("g = ");
    // Serial.println(g);
    // Serial.print("b = ");
    // Serial.println(b);
    

    // Detect Colors
    if (g < 10 && b < 10) {
        Serial.print("RED is DETECTED");
    } else if (r < 10 && b < 15) {
        Serial.print("GREEN is DETECTED");
    } else if (r < 15 && g < 25) {
        Serial.print("BLUE is DETECTED");
    } else {
        Serial.print("detecting....");
    }
}

void irSensorTest() {
    Serial.print("Front Sensor: ");
    Serial.println(frontSensor.getDistanceFloat()); 
    Serial.print("Left Sensor: ");
    Serial.println(leftSensor.getDistanceFloat()); 
    Serial.print("Right Sensor: ");
    Serial.println(rightSensor.getDistanceFloat()); 
    delay(100);
}

void lineFollowerTest() {
    qtr.readLineBlack(sensors); // Get calibrated sensor values returned in the sensors array
    for (int i = 0; i < 6; i++) {
       Serial.print(sensors[i]);
       Serial.print(" "); 
    }
    delay(250);
}

void colorMode() {
    int r, g, b, a;

    // Start motors with initial power
    analogWrite(LEFTMOTORPOWER, 100);
    analogWrite(RIGHTMOTORPOWER, 100);

    // Wait until the color sensor detects an initial color
    while (!colorSensor.colorAvailable()) {
        delay(5);
    }

    // Read the initial color
    colorSensor.readColor(r, g, b, a);

    // Save the initial detected RGB values
    int rInitial = r, gInitial = g, bInitial = b;
    Serial.print("Initial color detected: R = ");
    Serial.print(rInitial);
    Serial.print(", G = ");
    Serial.print(gInitial);
    Serial.print(", B = ");
    Serial.println(bInitial);

    // Define a tolerance range for detection
    int tolerance = 15; // Adjust based on your sensor and environment

    // Start moving forward
    Serial.println("Moving forward...");
    digitalWrite(LEFTMOTOR1, HIGH);
    digitalWrite(LEFTMOTOR2, LOW);
    digitalWrite(RIGHTMOTOR1, LOW);
    digitalWrite(RIGHTMOTOR2, HIGH);

    bool colorLost = false; // Flag to track when the color is no longer detected

    while (true) {
        // Continuously read the color sensor
        while (!colorSensor.colorAvailable()) {
            delay(5);
        }
        colorSensor.readColor(r, g, b, a);

        // Debug: Print current color values
        Serial.print("Current color: R = ");
        Serial.print(r);
        Serial.print(", G = ");
        Serial.print(g);
        Serial.print(", B = ");
        Serial.println(b);

        // Check if the current color is outside the tolerance of the initial color
        if (abs(r - rInitial) > tolerance || abs(g - gInitial) > tolerance || abs(b - bInitial) > tolerance) {
            colorLost = true; // Mark that the robot has left the paper
        }

        // If the same color is detected again after leaving, stop the robot
        if (colorLost) {
            if (abs(r - rInitial) <= tolerance && abs(g - gInitial) <= tolerance && abs(b - bInitial) <= tolerance) {
                Serial.println("Same color detected again. Stopping...");
                // Stop the motors
                digitalWrite(LEFTMOTOR1, LOW);
                digitalWrite(LEFTMOTOR2, LOW);
                digitalWrite(RIGHTMOTOR1, LOW);
                digitalWrite(RIGHTMOTOR2, LOW);
                break; // Exit the loop
            }
        }

        delay(5); // Small delay to stabilize readings
    }
}

void lineFollowMode() {
    Serial.println("Entering Line Follow Mode...");

    analogWrite(LEFTMOTORPOWER, 50);
    analogWrite(RIGHTMOTORPOWER, 50);

    int lineThreshold = 800; // Target value for the middle sensors (adjust as needed)
    int leftSensor, rightSensor; // Variables for key sensor readings

    while (true) {
        // Read calibrated sensor values
        qtr.readCalibrated(sensors);

        // Get readings from the left and right middle sensors
        leftSensor = sensors[2];  // Assuming sensors[2] is the left middle sensor
        rightSensor = sensors[3]; // Assuming sensors[3] is the right middle sensor

        // Debug: Print sensor values
        Serial.print("Left Sensor: ");
        Serial.print(leftSensor);
        Serial.print(" | Right Sensor: ");
        Serial.println(rightSensor);

        // Check if no line is detected (all sensor values are below a minimum threshold)
        bool lineDetected = false;
        for (int i = 0; i < 6; i++) {
            if (sensors[i] > 200) { // Minimum threshold for detecting the line
                lineDetected = true;
                break;
            }
        }

        if (!lineDetected) {
            Serial.println("No line detected. Exiting Line Follow Mode...");
            // Stop the motors
            digitalWrite(LEFTMOTOR1, LOW);
            digitalWrite(LEFTMOTOR2, LOW);
            digitalWrite(RIGHTMOTOR1, LOW);
            digitalWrite(RIGHTMOTOR2, LOW);
            break; // Exit the loop
        }

        // Adjust direction based on middle sensors
        if (leftSensor > lineThreshold && rightSensor < lineThreshold) {
            // Line is more on the left side -> turn right
            Serial.println("Turning right...");
            digitalWrite(LEFTMOTOR1, HIGH);
            digitalWrite(LEFTMOTOR2, LOW);
            digitalWrite(RIGHTMOTOR1, LOW);
            digitalWrite(RIGHTMOTOR2, HIGH);
        } else if (rightSensor > lineThreshold && leftSensor < lineThreshold) {
            // Line is more on the right side -> turn left
            Serial.println("Turning left...");
            digitalWrite(LEFTMOTOR1, LOW);
            digitalWrite(LEFTMOTOR2, HIGH);
            digitalWrite(RIGHTMOTOR1, HIGH);
            digitalWrite(RIGHTMOTOR2, LOW);
        } else {
            // Line is centered -> move straight
            Serial.println("Moving straight...");
            digitalWrite(LEFTMOTOR1, HIGH);
            digitalWrite(LEFTMOTOR2, LOW);
            digitalWrite(RIGHTMOTOR1, HIGH);
            digitalWrite(RIGHTMOTOR2, LOW);
        }

        delay(10); // Short delay for stability
    }
}

void loop() {
    // Controller
    BP32.update();

    // Turn on LEDS
    // digitalWrite(2, HIGH);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);

    // Set Motor Power
    analogWrite(LEFTMOTORPOWER, 150);
    analogWrite(RIGHTMOTORPOWER, 150);

    //colorSenseTest();

    // irSensorTest();
    // lineFollowerTest();

    // Controller Code
    
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr controller = myGamepads[i];
        if (controller && controller->isConnected()) {
            // Serial.println("Connected");
            digitalWrite(2, HIGH);
            // delay(2000);
            // digitalWrite(2, LOW);
            

            //Move forward
            if (controller->axisY() < 0) { // negative y is upward on stick
                Serial.println(" DC motor move");
                digitalWrite(LEFTMOTOR1, HIGH);
                digitalWrite(LEFTMOTOR2, LOW);
                digitalWrite(RIGHTMOTOR1, LOW);
                digitalWrite(RIGHTMOTOR2, HIGH);
            
            }

            //Move backward
            if (controller->axisY() > 0) {
                Serial.println(" DC motor move");
            
                digitalWrite(LEFTMOTOR1, LOW);
                digitalWrite(LEFTMOTOR2, HIGH);
                digitalWrite(RIGHTMOTOR1, HIGH);
                digitalWrite(RIGHTMOTOR2, LOW);
            }

            // Turn right
            if (controller->axisRX() > 0) { 
                Serial.println(" DC motor move");
                digitalWrite(LEFTMOTOR1, LOW);
                digitalWrite(LEFTMOTOR2, HIGH);
                digitalWrite(RIGHTMOTOR1, LOW);
                digitalWrite(RIGHTMOTOR2, HIGH);
            }
            
            // Turn left
            if (controller->axisRX() < 0) { // stop motor 1
                digitalWrite(LEFTMOTOR1, HIGH);
                digitalWrite(LEFTMOTOR2, LOW);
                digitalWrite(RIGHTMOTOR1, HIGH);
                digitalWrite(RIGHTMOTOR2, LOW);
            }

            //Stop moving
            if (controller->axisRX() == 0 && controller->axisY() == 0) { // stop motor 1
                digitalWrite(LEFTMOTOR1, LOW);
                digitalWrite(LEFTMOTOR2, LOW);
                digitalWrite(RIGHTMOTOR1, LOW);
                digitalWrite(RIGHTMOTOR2, LOW);
            }

            // Controller Position Test
            // Serial.print("Joystick 1 X: ");
            // Serial.println(controller->axisX());
            // Serial.print("Joystick 1 Y: ");
            // Serial.println(controller->axisY());
            // Serial.print("Joystick 2 X: ");
            // Serial.println(controller->axisRX());
            // Serial.print("Joystick 2 Y: ");
            // Serial.println(controller->axisRY());
            // Serial.println();       

            // PHYSICAL BUTTONS
            // Button A - Color Picker Mode
            if (controller->b()) {
                Serial.println("button A pressed");
                colorMode();
            }

            // Button B - Line Mode
            if (controller->a()) {
                Serial.println("button B pressed");
                lineFollowMode();
            }

            if (controller->y()) {
                Serial.println("button X pressed");
            }

            if (controller->x()) {
                Serial.println("button Y pressed");
            }

            if (controller->l1()) {
                Serial.println("button L1 pressed");
            }

            if (controller->l2() == 1 && controller->r2() == 0) {
                Serial.println("button L2 pressed");
                servo.write(1400);
            }

            if (controller->r1()) {
                Serial.println("button R1 pressed");
            }


            if (controller->r2() == 1 && controller->l2() == 0) {
                Serial.println("button R2 pressed");
                servo.write(1600);
            }

            if (controller->r2() == 0 && controller->l2() == 0) {
                servo.write(1500);
            }
        }
    }
    
    
    vTaskDelay(1);
}

