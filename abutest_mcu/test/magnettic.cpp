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
    // Initialize servo motors
    servo_wheel1.attach(SERVO_A_PIN);

    servo_wheel1.writeMicroseconds(1400);
}

void loop() {

    float angle_base = mag.getAngle();
    Serial.println("Base Angle: " + String(angle_base));
    
}
