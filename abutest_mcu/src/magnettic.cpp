#include <Arduino.h>
#include "MagneticEn.h"
#include <ESP32Servo.h>
  
#define SERVO_A_PIN 15   
#define SERVO_B_PIN 16

MagneticEn mag;
Servo servo_wheel1, servo_wheel2;

float current_angle_wheel = 90;
float angle_wheel = 90;
float speed_wheel1 = 100;
float speed_wheel2 = 100;

float current_pwm_left_wheel = 1500, current_pwm_right_wheel = 1500; 


float normaliseDegree(float degree) {
    return degree > 360 ? degree - 360 : degree;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.begin(21, 22);

    pinMode(current_pwm_left_wheel, OUTPUT);
    pinMode(current_pwm_right_wheel, OUTPUT);
    pinMode(speed_wheel1, OUTPUT);
    pinMode(speed_wheel2, OUTPUT);
  
    // Initialize servo motors
    servo_wheel1.attach(SERVO_A_PIN);
    servo_wheel2.attach(SERVO_B_PIN);

    // Set initial servo positions
    servo_wheel1.write(angle_wheel);
    servo_wheel2.write(current_angle_wheel);
}

void loop() {

    servo_wheel1.writeMicroseconds(1000); // Rotate clockwise (adjust as needed)
    delay(5000); // Run for 2 seconds

    servo_wheel1.writeMicroseconds(1500); // Stop (neutral)
    delay(2000); // Wait for 2 seconds

    servo_wheel1.writeMicroseconds(1800);  // Adjust for smoother rotation
    delay(1000);

    servo_wheel1.writeMicroseconds(2000); // Rotate counterclockwise (adjust as needed)
    delay(5000); // Run for 2 seconds

    
    // float angle_base = mag.getAngle();
    // Serial.println("Base Angle: " + String(angle_base));
    
}
