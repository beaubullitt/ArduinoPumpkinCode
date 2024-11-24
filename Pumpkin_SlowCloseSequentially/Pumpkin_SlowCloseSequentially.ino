#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the Adafruit_PWMServoDriver class
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Variables to store current positions of servos
int servoPos0 = 0;
int servoPos3 = 0;
int servoPos11 = 0;
int servoPos15 = 0;

void setup() {
  // Initialize the servo driver with the I2C address
  pwm.begin();
  pwm.setPWMFreq(60);  // Set the PWM frequency to 60Hz (standard for servos)
}

// Function to set the position of a servo on a specified channel
void setServoPosition(uint8_t channel, uint16_t angle) {
  uint16_t pwmValue = map(angle, 0, 270, 150, 600);  // Mapping to 150-600 range for PWM control
  pwm.setPWM(channel, 0, pwmValue);
}

// Function to move all servos at the same time
void moveAllServosSmoothly(int targetPos0, int targetPos3, int targetPos11, int targetPos15, int stepDelay) {
  // Move the servos in small increments
  while (servoPos0 != targetPos0 || servoPos3 != targetPos3 || servoPos11 != targetPos11 || servoPos15 != targetPos15) {
    
    // Increment/Decrement servoPos0
    if (servoPos0 < targetPos0) servoPos0++;
    else if (servoPos0 > targetPos0) servoPos0--;

    // Increment/Decrement servoPos3
    if (servoPos3 < targetPos3) servoPos3++;
    else if (servoPos3 > targetPos3) servoPos3--;

    // Increment/Decrement servoPos11
    if (servoPos11 < targetPos11) servoPos11++;
    else if (servoPos11 > targetPos11) servoPos11--;

    // Increment/Decrement servoPos15
    if (servoPos15 < targetPos15) servoPos15++;
    else if (servoPos15 > targetPos15) servoPos15--;

    // Update the servos
    setServoPosition(0, servoPos0);
    setServoPosition(3, servoPos3);
    setServoPosition(11, servoPos11);
    setServoPosition(15, servoPos15);

    // Delay between each step
    delay(stepDelay);
  }
}

void loop() {
  // Move all servos to 180 degrees
  moveAllServosSmoothly(180, 180, 180, 180, 10);  // Move all servos to 180 degrees with stepDelay of 10ms
  delay(2000);  // Wait for 2 seconds

  // Move all servos back to 0 degrees
  moveAllServosSmoothly(0, 0, 0, 0, 10);  // Move all servos back to 0 degrees
  delay(2000);  // Wait for 2 seconds
}
