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
Serial.println(ultraSonicDistance());  //Calling the method of the distance in cm 
 if (millis() > 10000)
    ultraSonicSensorServo.write(180); //Turns left at full speed 
  else if (millis() > 2000)
    ultraSonicSensorServo.write(0); //Turns right at full speed after 2 seconds

}
long ultraSonicDistance(){  //Method will return the distance from the ultrasonic sensor 
  digitalWrite(trigPin, LOW); //Sets it to low so off the trigger pin is at 0V
  delayMicroseconds(2);                       // Delay for 2 Microseconds
  digitalWrite(trigPin, HIGH); //Sets it to high so the trigger pin is at maximum voltagr
  delayMicroseconds(10);                      // Delay for 10 Microseconds
  digitalWrite(trigPin, LOW); //Sets it to low so off the trigger pin is at 0V
  long duration = pulseIn(echoPin, HIGH); 
  distance = duration*0.034/ 2; //Using the data sheet
}
