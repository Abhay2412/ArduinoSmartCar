/*
 * Ultrasonic Sensor and Servo Reference Code V1
 * 
 * Use the following to get accustomed operating the ultrasonic sensor
 * and the servo of the ELEGOO Car Kit in all directions
 * 
 * The following code will do the following in order:
 *    1. Rotate servo to the right
 *    2. Print out distance measured in centimeter
 *    3. Change servo to mid point after 5 seconds of operation
 *    4. Change servo direction to left side after another 5 seconds of operation
 *    
 * Require Components:
 *    - ELEGOO Car Kit
 *    - USB-A to USB-B cable (Arduino USB cable)
 *    - Ultrasonic sensor (on the ELEGOO Car Kit)
 *    - Servo motor (on the LEGOO Car Kit)
 *    
 * Author: Khaled Elmalawany
 */
 
#include <Servo.h>                          // Include servo library

Servo ultrasonicSensorServo;                // Declare a servo object from the Servo class
const int servoPin = 3;                     // Servo Pin

const int trigPin = A5;                     // Trigger Pin of Ultrasonic Sensor
const int echoPin = A4;                     // Echo Pin of Ultrasonic Sensor

void setup() {
  Serial.begin(9600);                       // Begin Serial communication
  
  // Initialize the Servo
  ultrasonicSensorServo.attach(servoPin);    // Declare that servoPin is attached to a servo motor

  // Initialize the Ultrasonic Sensor
  pinMode(trigPin, OUTPUT);                  // Intitialize the Trigger as an Output
  pinMode(echoPin, INPUT);                   // Initialize the Echo pin as an Input

  // Set the servo to angle of 0 (right side)
  // Servo angles are degrees froma minimum of 0 to a maximum of 180
  ultrasonicSensorServo.write(0);
}

void loop() {
  if (millis() > 10000)
    ultrasonicSensorServo.write(180);
  else if (millis() > 5000)
    ultrasonicSensorServo.write(90);

  Serial.println(ultrasonicDistanceInCM());
}

long ultrasonicDistanceInCM() {               // Returns the distance in centimeters using the Ultrasonic sensor
  // The above sends a pulse of ultrasonic wave
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);                       // Delay for 2 Microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);                      // Delay for 10 Microseconds
  digitalWrite(trigPin, LOW);
  // pulseIn waits for the pin to go HIGH, starts timing, then waits for the pin to go LOW and stops timing
  // pulseIn() return is of type long
  long duration = pulseIn(echoPin, HIGH);
  return duration / 29 / 2;
}
