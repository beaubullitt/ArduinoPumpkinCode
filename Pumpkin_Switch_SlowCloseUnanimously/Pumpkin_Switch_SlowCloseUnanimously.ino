#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define two angles for each servo
int servo0_angle1 = 180;  // Servo 0: First angle
int servo0_angle2 = 60; // Servo 0: Second angle

int servo3_angle1 = 180;  // Servo 3: First angle
int servo3_angle2 = 60; // Servo 3: Second angle

int servo11_angle1 = 170;  // Servo 11: First angle
int servo11_angle2 = 270;  // Servo 11: Second angle

int servo15_angle1 = 170;  // Servo 15: First angle
int servo15_angle2 = 270;  // Servo 15: Second angle

// Define pin for switch
const int switchPin = 6; // Pin for switch input
int switchState = 0;     // Variable to store switch state
int lastSwitchState = HIGH; // Store the last switch state
bool moveToPosition = false; // Flag to control position switching
int stepDelay = 10;  // Delay between each step for smooth movement

// Variables to store current positions of servos
int servoPos0 = 30;  
int servoPos3 = 60;
int servoPos11 = 90;
int servoPos15 = 45;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pwm.begin();
  pwm.setPWMFreq(60);  // Set the PWM frequency

  // Set the switch pin as input
  pinMode(switchPin, INPUT_PULLUP);  // Use internal pull-up resistor
  delay(500);  // Give the servos some time to reach the neutral position
}

// Function to set the position of a servo on a specified channel
void setServoPosition(uint8_t channel, uint16_t angle) {
  uint16_t pwmValue = map(angle, 0, 270, 150, 600);  // Map angle to PWM
  pwm.setPWM(channel, 0, pwmValue);
}

// Function to move the servos in small increments to the new target positions
void moveServosSmoothly(int targetPos0, int targetPos3, int targetPos11, int targetPos15) {
  // Move the servos incrementally to the new target positions
  while (servoPos0 != targetPos0 || servoPos3 != targetPos3 || servoPos11 != targetPos11 || servoPos15 != targetPos15) {

    if (servoPos0 < targetPos0) servoPos0++;
    else if (servoPos0 > targetPos0) servoPos0--;

    if (servoPos3 < targetPos3) servoPos3++;
    else if (servoPos3 > targetPos3) servoPos3--;

    if (servoPos11 < targetPos11) servoPos11++;
    else if (servoPos11 > targetPos11) servoPos11--;

    if (servoPos15 < targetPos15) servoPos15++;
    else if (servoPos15 > targetPos15) servoPos15--;

    // Update the servo positions
    setServoPosition(0, servoPos0);
    setServoPosition(3, servoPos3);
    setServoPosition(11, servoPos11);
    setServoPosition(15, servoPos15);

    // Delay between each step to create smooth movement
    delay(stepDelay);
  }

  // Print updated positions after movement
  Serial.print("Servo on channel 0 moved to: ");
  Serial.println(servoPos0);
  Serial.print("Servo on channel 3 moved to: ");
  Serial.println(servoPos3);
  Serial.print("Servo on channel 11 moved to: ");
  Serial.println(servoPos11);
  Serial.print("Servo on channel 15 moved to: ");
  Serial.println(servoPos15);
}

void loop() {
  // Read the state of the switch
  switchState = digitalRead(switchPin);

  // Check if the switch has been pressed (state change from HIGH to LOW)
  if (switchState == LOW && lastSwitchState == HIGH) {
    // Toggle between two predefined positions for each servo
    if (!moveToPosition) {
      moveServosSmoothly(servo0_angle1, servo3_angle1, servo11_angle1, servo15_angle1);  // Move to first set of angles smoothly
      moveToPosition = true;  // Set flag to indicate move to first position
    } else {
      moveServosSmoothly(servo0_angle2, servo3_angle2, servo11_angle2, servo15_angle2);  // Move to second set of angles smoothly
      moveToPosition = false;  // Reset flag to alternate back to first position on next press
    }

    // Add a small delay for debounce
    delay(50);
  }

  // Update the last switch state
  lastSwitchState = switchState;
}
