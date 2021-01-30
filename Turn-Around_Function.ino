float accelX, accelY, accelZ;
  
#include <Wire.h>                            // Wire library require for I2C protocol
#include <MPU6050.h>                         // Include MPU-6050 sensor

  float timeStep = 0.01;

  unsigned long timer = 0;
  float gyroX, gyroY, gyroZ;
  float StartZ;
  float temp;

#define ENA 5                                // Enable input for right motors
#define ENB 6                                // Enable input for left motors
#define IN1 7                                // Left forward motion
#define IN2 8                                // Left backward motion
#define IN3 9                                // Right backward motion
#define IN4 11                               // Right forward motion

MPU6050 mpu;



void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);


   // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
                                              // Calibrate gyroscope. The calibration must be at rest.
                                              // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

                                              // Set threshold sensivty. Default 3.
                                              // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);


pinMode(ENA, OUTPUT);                     // Enables left wheels to move
  pinMode(ENB, OUTPUT);                     // Enables right wheels to move
  // An analog input (instead of a digital) can also be used to set the speed (instead of using 0 and max)
  pinMode(IN1, OUTPUT);                     // Allows left wheels to turn forward when HIGH
  pinMode(IN2, OUTPUT);                     // Allows left wheels to turn backwards when HIGH
  pinMode(IN3, OUTPUT);                     // Allows right wheels to turn forward when HIGH
  pinMode(IN4, OUTPUT);                     // Allows right wheels to turn backwards when HIGH


  setMotorSpeed(100);
  //turnLeft();
  //delay(1000);
  TurnAround();
  delay(1);
  stopMovingForever();


}

void loop() {
  // put your main code here, to run repeatedly:

timer = millis();

  // Read normalized values
  Vector GyroTotal = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  gyroY = gyroY + GyroTotal.YAxis * timeStep;
  gyroX = gyroX + GyroTotal.XAxis * timeStep;
  gyroZ = gyroZ + GyroTotal.ZAxis * timeStep;
  Serial.print(" Pitch = ");
  Serial.print(gyroY);
  Serial.print(" Roll = ");
  Serial.print(gyroX);  
  Serial.print(" Yaw = ");
  Serial.println(gyroZ);
  delay((timeStep*1000) - (millis() - timer));

  /*  setMaxSpeed();
  //turnLeft();
  //delay(1000);
  TurnAround();
  delay(1);
  stopMovingForever();
  */
}


//Functions
//
//

void TurnAround(){                          //Function to turn car around based on gyro data.
float z = 180;
while (gyroZ<z){
turnLeft();
delay(10);
}

}


void moveForward()                           // Forward function
{
  digitalWrite(IN1, HIGH);                   // Left forward motion activated
  digitalWrite(IN2, LOW);                    // Left backward motion not activated
  digitalWrite(IN3, LOW);                    // Right backward motion not activated
  digitalWrite(IN4, HIGH);                   // Right forward motion activated
}

void moveBackwards()                         // Backward function
{
  digitalWrite(IN1, LOW);                    // Left forward motion activated
  digitalWrite(IN2, HIGH);                   // Left backward motion not activated
  digitalWrite(IN3, HIGH);                   // Right backward motion not activated
  digitalWrite(IN4, LOW);                    // Right forward motion activated
}

void turnLeft()                              // Left turn function
{
  digitalWrite(IN1, LOW);                    // Left forward motion not activated
  digitalWrite(IN2, HIGH);                   // Left backward motion activated
  digitalWrite(IN3, LOW);                    // Right backward motion not activated
  digitalWrite(IN4, HIGH);                   // Right forward motion activated

}

void wideTurnLeft()                          // Wide left turn function
{
  digitalWrite(IN1, LOW);                    // Left forward motion not activated
  digitalWrite(IN2, LOW);                    // Left backward motion activated
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

void turnRight()                             // Right turn function
{
  digitalWrite(IN1, HIGH);                   // Left forward motion activated
  digitalWrite(IN2, LOW);                    // Left backward motion not activated
  digitalWrite(IN3, HIGH);                   // Right backward motion activated
  digitalWrite(IN4, LOW);                    // Right forward motion not activated
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

void setMaxSpeed()                           // Set motors to max speed function
{
  digitalWrite(ENA, HIGH);                   // Disable right motor
  digitalWrite(ENB, HIGH);                   // Disable left motor
}

void setMotorSpeed(int newSpeed) //0-255 (off to high)
{
  // Avoid setting the motors to 2 different speed as it tends to stop working
  analogWrite(ENA, newSpeed);               // Enable left motor at set speed
  analogWrite(ENB, newSpeed);               // Enable right motor at set speed
}
