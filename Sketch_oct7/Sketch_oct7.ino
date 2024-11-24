
#include <Adafruit_PWMServoDriver.h>

// Create an instance of the Adafruit_PWMServoDriver class
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define the servo parameters
int servoMIN = 150000;  // Min pulse length out of 4096
int servoMAX = 600000;  // Max pulse length out of 4096

int servoPin = 0;  // Use the appropriate PWM pin on the Adafruit board

void setup() {
  // Initialize the PWM board
  Serial.begin(9600);
  Serial.println("16 channel PWM test!");

  pwm.begin();
  pwm.setPWMFreq(1600);  // Set the PWM frequency (you can adjust this)

  // You can set the initial position here if needed
  pwm.setPWM(servoPin, 0, servoMIN);  // Set servo to minimum position
  delay(10000);
}

void loop() {
  // Rotate the servo to 90 degrees
  Serial.println("In the loop!");
  Serial.println("Going to max!");
  pwm.setPWM(servoPin, 0, servoMAX);
  Serial.println(servoPin);
  delay(10000);  // Wait for 1 second

  // Rotate the servo back to 0 degrees
  Serial.println("Going to min!");
  pwm.setPWM(servoPin, 0, servoMIN);
  delay(10000);  // Wait for 1 second
}



// #include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>

// // Create an instance of the Adafruit_PWMServoDriver class
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// // Define the servo parameters
// #define servoMIN 0  // Min pulse length out of 4096
// #define servoMAX 600  // Max pulse length out of 4096
// // int servoPin = 3;    // Use the appropriate PWM pin on the Adafruit board

// void setup() {
//   Serial.begin(9600);
//   // Initialize the PWM board
//   pwm.begin();
//   pwm.setPWMFreq(60);  // Set the PWM frequency (you can adjust this)

//   // You can set the initial position here if needed
//   // pwm.setPWM(servoPin, 0, servoMin); // Set servo to minimum position
// }

// void loop() {

//   for (int servo = 0; servo < 4; servo++) {
//     pwm.setPWM(servo, 0, servoMIN);
//     Serial.println(servo);
//     delay(2000);
//   }
// Rotate the servo to 90 degrees
// pwm.setPWM(servoPin, 0, servoMax);
// delay(1000);  // Wait for 1 second

// // Rotate the servo back to 0 degrees
// pwm.setPWM(servoPin, 0, servoMin);
// delay(1000);  // Wait for 1 second
// }