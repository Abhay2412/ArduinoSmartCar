/**
 * Ultrasonic Sensor and Servo Code for the Smart Car
 * 
 * Author: Ben, Mutasem, Abhay
 */

#include <Servo.h>     //The servo library gets imported in 
Servo ultraSonicSensorServo;       //Creating the servo object from the imported library

const int servoPin = 2;            //Not sure about the pin number 

const int trigPin = A5;           //Trigger Pin of the Ultrasonic Sensor 
const int echoPin = A4;           //Echo Pin of the Ultrasonic Sensor

long distance;                   //Distance which is measured in cm
long duration;                   //Duration of the echoPin how long it takes for pulse to travel

void setup() {
Serial.begin(9600);            //BaudRate of 9600, and this opens the serial communication 

ultraSonicSensorServo.attach(servoPin);
pinMode(trigPin, OUTPUT);       //Setting the trigger pin to output
pinMode(echoPin, INPUT);       //The echo pin is the input
}

void loop() {
  // put your main code here, to run repeatedly:

}
