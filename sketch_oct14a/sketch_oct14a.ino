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
  // Calculate the PWM value corresponding to the desired angle (from 0 to 180 degrees)
  uint16_t pwmValue = map(angle, 0, 270, 150, 600);  // Mapping to 150-600 range for PWM control
  pwm.setPWM(channel, 0, pwmValue);
}

// Function to move servo gradually
void moveServoSmoothly(uint8_t channel, int &currentPos, int targetPos, int stepDelay) {
  // Check if we need to move up or down
  if (currentPos < targetPos) {
    // Move incrementally up
    for (int pos = currentPos; pos <= targetPos; pos++) {
      setServoPosition(channel, pos);
      delay(stepDelay);  // Delay between steps to slow down the movement
    }
  } else {
    // Move incrementally down
    for (int pos = currentPos; pos >= targetPos; pos--) {
      setServoPosition(channel, pos);
      delay(stepDelay);  // Delay between steps to slow down the movement
    }
  }
  currentPos = targetPos;  // Update current position
}

void loop() {
  // Move servos gradually to a target position
  moveServoSmoothly(0, servoPos0, 180, 10);  // Move to 180 degrees with stepDelay of 10ms
  moveServoSmoothly(3, servoPos3, 180, 10);
  moveServoSmoothly(11, servoPos11, 180, 10);
  moveServoSmoothly(15, servoPos15, 180, 10);

  delay(2000);  // Wait for 2 seconds

  // Move servos back to initial position
  moveServoSmoothly(0, servoPos0, 0, 10);   // Move back to 0 degrees with stepDelay of 10ms
  moveServoSmoothly(3, servoPos3, 0, 10);
  moveServoSmoothly(11, servoPos11, 0, 10);
  moveServoSmoothly(15, servoPos15, 0, 10);

  delay(2000);  // Wait for 2 seconds
}
