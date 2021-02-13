
/**
 * Ultrasonic Sensor and Servo Code for the Smart Car
 * 
 * Author: Ben, Mutasem, Abhay
 */
#include <Wire.h>
#include <I2Cdev.h>  
#include <MPU6050.h>
#include <Servo.h>                  //The servo library gets imported in  
Servo ultraSonicSensorServo;       //Creating the servo object from the imported library
MPU6050 gyroAccelTemp;

/*Arduino pin is connected to the Motor drive module*/
#define ENA 5                                // Enable input for right motors
#define ENB 6                                // Enable input for left motors
#define IN1 7                                // Left forward motion
#define IN2 8                                // Left backward motion
#define IN3 9                                // Right backward motion
#define IN4 11                               // Right forward motion


#define GYRO_Z_OFFSET 60                     // Determined using IMU_Zero under File > Examples > MPU6050 from the provided file



const int servoPin = 3;            //Not sure about the pin number 

const int trigPin = A5;           //Trigger Pin of the Ultrasonic Sensor 
const int echoPin = A4;           //Echo Pin of the Ultrasonic Sensor

long distance;                   //Distance which is measured in cm
long duration;                   //Duration of the echoPin how long it takes for pulse to travel

  float gyroZ = 0; //Gyro current value 
  float gyroBefore = 0; //Gyro value before 
  
unsigned long timer = 0;
float timeStep = 0.01;

int currentLap = 0;    //The current lap of the SmartCar 
int currentLapRev = 0;
int numberOfLap1 = 2;

bool forward = true;  //Default it will function forward 
bool reverse = false; //Afterward it will switch over to reverse

float gyroDegree;  //The angle of the gyroscope in degrees

bool isFirstLoopComplete;                  // Declaring the isFirstLoopComplete boolean flag
  float previousTime;                        // Declaring the value to hold the time


void setup() {
Serial.begin(38400);            //BaudRate of 38400, and this opens the serial communication 

Wire.begin();
  Serial.println("Initializing I2C devices...");
  gyroAccelTemp.initialize();

  // Verify IMU connection
  Serial.println("Testing device connections...");
  Serial.println(gyroAccelTemp.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  gyroAccelTemp.CalibrateGyro();                  // Fully calibrate gyro in about 15 loops
  Serial.println("Calibration complete");         // Notify when calibration is complete
  /* The following will NOT get rid of the noise produce by the IMU but they will ensure that
      the average values are at 0 to a certain extent
      (Refer to the IMU Error Determination code and the MPU6050 tutorial video for more details)
  */
  // Set Gyroscope Offsets

  gyroAccelTemp.setZGyroOffset(GYRO_Z_OFFSET);    // Set the Z gyroscope offset


  // IMPORTANT: If you do not calibrate after setting your offset, there will be error
  gyroAccelTemp.CalibrateGyro(6);                 // Fine Tuning purposes of gyroscope
  Serial.println("Fine Tuning Complete");         // Notify when fine tuning is complete

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
setMotorSpeed(170);                                               //Set motor speed 100 out of 255
moveForward();

}

void loop() {

 gyroZ = gyroAccelTemp.getRotationZ() / 131.0;
                      
                      if (isFirstLoopComplete) {
                          float timeForOneLoop = micros() - previousTime;
                          //Serial.println(timeForOneLoop);
                          gyroDegree += gyroZ * timeForOneLoop / 1000000.0;
                         }
                         // NOTE: Try and keep the following code close to the above if statement
                         previousTime = micros();
                         
                         // Change the boolean flag to true to enable collection of gyroscope data
                         if (!isFirstLoopComplete) {
                          isFirstLoopComplete = true;
                         }
                      Serial.println();
                      // Print out Gyroscope data to the serial monitor 
                      Serial.print("Gyroscope (degrees)\t\tZ: ");
                      Serial.println(gyroDegree);
  
      while(forward){
      //This is the making the car move forward around the object in the inital direction 
             if(ultrasonicDistanceInCM() < 13){                            //If robot is closer than 13cm then turn left wide
              wideTurnLeft();  //Does a wide turn left around the object 
              delay(300); //Delays for 3ms 
              moveForward(); //This makes the car keep on moving forward once it completes the left turn 
             }
            //If that is not true it continues to this statement in which it does a right turn wide similary to the left turn instead 
                else if(ultrasonicDistanceInCM() > 20){ //Greater than 20cm instead 
                 wideTurnRight(); 
                  delay(300);   
                  moveForward();
                }
                //This if statement here checks for one our car completes a lap which icnrements the currentLap we are on
                if(gyroDegree <-360*currentLap){
                  currentLap++;
                  
                }
                //Checks once we have completed enough laps to change the boolean value of forward to false and retirves the gyroBefore value from currently it is at 
                //Then it breaks out of this if statement momentartily to proceeding what is outside of it
                if(currentLap==7){
                  gyroBefore = gyroDegree;
                  forward = false;
                  break;
                }
            
                      //Implemented from the Data sheets 
                      gyroZ = gyroAccelTemp.getRotationZ() / 131.0;
                      //Using the reference code for the calculations determining the time it takes for completeting it loop 
                      if (isFirstLoopComplete) {
                          float timeForOneLoop = micros() - previousTime;
                          //Serial.println(timeForOneLoop);
                          gyroDegree += gyroZ * timeForOneLoop / 1000000.0;
                         }
                         // NOTE: Try and keep the following code close to the above if statement
                         previousTime = micros();
                         
                         // Change the boolean flag to true to enable collection of gyroscope data
                         if (!isFirstLoopComplete) {
                          isFirstLoopComplete = true;
                         }
                      Serial.println();
                      // Print out Gyroscope data
                      Serial.print("Gyroscope (degrees)\t\tZ: ");
                      Serial.println(gyroDegree);
                          
      }
//This if statement starts the reverse car functionality in our smartcar checks for the current lap in the inital direction 
if(currentLap==7){
  setMaxSpeed();
  TurnAround();
  delay(1);
  stopMovingForever();
}                                                    

//Using the similar method as forward instead using the reverse boolean 
    while(reverse){
          setMotorSpeed(170);                                               //Set motor speed 100 out of 255
          moveForward();
           delay(1);
                 if(ultrasonicDistanceInCM() < 13){                            //If robot is closer than 13cm than turn left slightly
                  wideTurnRight();  
                  delay(300);
                  moveForward();
                 }
            
                else if(ultrasonicDistanceInCM() > 20){ 
                 wideTurnLeft(); 
                  delay(300);   
                  moveForward();
                }
                if(gyroDegree >(-360*currentLapRev+110)){      //+180 //Keep in mind the gyroscope starts at -1800+180 degrees for the reverse
                  currentLapRev++;               
                  
                }
                if(currentLapRev==6){
                  reverse = false;
                  break;
                }
            
              
                        gyroZ = gyroAccelTemp.getRotationZ() / 131.0;
                        
                        if (isFirstLoopComplete) {
                            float timeForOneLoop = micros() - previousTime;
                            //Serial.println(timeForOneLoop);
                            gyroDegree += gyroZ * timeForOneLoop / 1000000.0;
                           }
                           // NOTE: Try and keep the following code close to the above if statement
                           previousTime = micros();
                           
                           // Change the boolean flag to true to enable collection of gyroscope data
                           if (!isFirstLoopComplete) {
                            isFirstLoopComplete = true;
                           }
                        Serial.println();
                        // Print out Gyroscope data
                        Serial.print("Gyroscope (degrees)\t\tZ: ");
                        Serial.println(gyroDegree);
                            }
//This is checking for reverse laps now if the car has completed the correct amount 
if(currentLapRev==6){
  moveForward();
  delay(5);
  if(ultrasonicDistanceInCM() > 40){
    stopMovingForever();
    
  }
}


}

        /////////////
        /////////////
        //Functions//
        /////////////
        /////////////
        
void TurnAround(){                          //Function to turn car around based on gyro data.
float z = 180;
      if(gyroDegree < (gyroBefore +180)){
      turnLeft();
      delay(5);
      ultraSonicSensorServo.write(180);
      }
      
      if(gyroDegree> (gyroBefore +180)){
      
        reverse = true;
        currentLap = 8;                     //Set to 7 so turn around function is ensured to be turned off.
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

void setMaxSpeed()                           // Set motors to max speed function
{
  digitalWrite(ENA, HIGH);                   // Disable right motor
  digitalWrite(ENB, HIGH);                   // Disable left motor
}


void stopMoving()                            // Stopping function
{
  digitalWrite(IN1, LOW);                    // Left forward motion not activated
  digitalWrite(IN2, LOW);                    // Left backward motion not activated
  digitalWrite(IN3, LOW);                    // Right backward motion not activated
  digitalWrite(IN4, LOW);                    // Right forward motion not activated
}

void stopMovingForever()                     // Stopping function
{
  digitalWrite(ENA, LOW);                    // Disable right motor
  digitalWrite(ENB, LOW);                    // Disable left motor
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
