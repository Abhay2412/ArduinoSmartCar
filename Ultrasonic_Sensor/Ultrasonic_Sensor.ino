/**
 * Ultrasonic Sensor and Servo Code for the Smart Car
 * 
 * Author: Ben, Mutasem, Abhay
 */

#include <Servo.h>                  //The servo library gets imported in  
Servo ultraSonicSensorServo;       //Creating the servo object from the imported library
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11


const int servoPin = 3;            //Not sure about the pin number 

const int trigPin = A5;           //Trigger Pin of the Ultrasonic Sensor 
const int echoPin = A4;           //Echo Pin of the Ultrasonic Sensor

long distance;                   //Distance which is measured in cm
long duration;                   //Duration of the echoPin how long it takes for pulse to travel

void setup() {
Serial.begin(9600);            //BaudRate of 9600, and this opens the serial communication 

ultraSonicSensorServo.attach(servoPin);
pinMode(IN1, OUTPUT); //Left wheels turn forward
pinMode(IN3, OUTPUT); //Right wheels turn forward 

pinMode(ENA, OUTPUT); // Enables left wheels to move
pinMode(ENB, OUTPUT); // Enables right wheels to move

pinMode(trigPin, OUTPUT);       //Setting the trigger pin to output
pinMode(echoPin, INPUT);       //The echo pin is the input
ultraSonicSensorServo.write(0);
ultrasonicDistanceInCM();
Serial.println(ultrasonicDistanceInCM()+ "is from setup");
delay(2000);
setMotorSpeed(100);
moveForward();

}

void loop() {
 /*if (millis() > 7000) //calculate the distance 
    ultraSonicSensorServo.write(90); //Turns right at full speed 
    */
 if(ultrasonicDistanceInCM() < 13){                            //If robot is closer than 13cm than turn left slightly
  wideTurnLeft();  
  delay(300);
  moveForward();
  Serial.println(ultrasonicDistanceInCM()+ "is from data 13");
 }

    else if(ultrasonicDistanceInCM() > 20){ 
     wideTurnRight(); 
      delay(300);   
      moveForward();
      Serial.println(ultrasonicDistanceInCM()+ "is from data 20");
    }
}
void turnRight()                             // Right turn function
{
  digitalWrite(IN1, HIGH);                   // Left forward motion activated
  digitalWrite(IN2, LOW);                    // Left backward motion not activated
  digitalWrite(IN3, HIGH);                   // Right backward motion activated
  digitalWrite(IN4, LOW);                    // Right forward motion not activated
}

void turnLeft()                              // Left turn function
{
  digitalWrite(IN1, LOW);                    // Left forward motion not activated
  digitalWrite(IN2, HIGH);                   // Left backward motion activated
  digitalWrite(IN3, LOW);                    // Right backward motion not activated
  digitalWrite(IN4, HIGH);                   // Right forward motion activated

}

void wideTurnRight()                          // Wide right turn function
{
  digitalWrite(IN1, HIGH);                   // Left forward motion not activated
  digitalWrite(IN2, LOW);                    // Left backward motion activated
  digitalWrite(IN3, LOW);                    // Right backward motion not activated
  digitalWrite(IN4, LOW);                    // Right forward motion activated

}

void wideTurnLeft()                          // Wide left turn function
{
  digitalWrite(IN1, LOW);                    // Left forward motion not activated
  digitalWrite(IN2, LOW);                    // Left backward motion activated
  digitalWrite(IN3, LOW);                    // Right backward motion not activated
  digitalWrite(IN4, HIGH);                   // Right forward motion activated

}
void moveForward() {  //Forward function 
  digitalWrite(IN1, HIGH);  //Left forward motion activated 
  digitalWrite(IN2, LOW);  //Left backward motion not activated 
  digitalWrite(IN3, LOW); //Right backward motion not activated 
  digitalWrite(IN4, HIGH); //Right forward motion activated 
  
}

void setMotorSpeed(int newSpeed) {
  // Avoid setting the motors to 2 different speed as it tends to stop working
  analogWrite(ENA, newSpeed);               // Enable left motor at set speed
  analogWrite(ENB, newSpeed);               // Enable right motor at set speed
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
