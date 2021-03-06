/*
   MPU6050 Reference Code V1

   Use the following to get accustomed operating the GY-521 MPU6050 IMU using the ELEGOO Car Kit
   Ensure that the GY-521 MPU6050 IMU module is properly connected to the Arduino
   You may use this code to have a stronger understanding of the orientations of the IMU
   You may also refer to the GY-521 MPU6050 tutorial video for a better understanding

   The following code will print out the following in order for every void loop() iteration:
      - X rotation using the gyroscope in degrees per second
      - Y Rotation using the gyroscope in degrees per second
      - Z Rotation using the gyroscope in degrees per second
      - Temperature in Celsius
      - X Acceleration using the accelerometer in meters per second squared
      - Y Acceleration using the accelerometer in meters per second squared
      - Z Acceleration using the accelerometer in meters per second squared

   Require Components:
      - ELEGOO Car Kit
      - USB-A to USB-B cable (Arduino USB cable)
      - GY-521 MPU6050 IMU module
      - Wires
      - Breadboard

   Pinout:
      - SDA pin of MPU6050 --> Pin 20
      - SCL pin of MPU6050 --> Pin 21

   NOTE: The Arduino UNO and the Arduino MEGA 2560 have 2 SDA and SCL pins
      - Both have an SDA and SCL pin near the AREF pin
      - UNO has an additional SDA at pin A4 and SCL at pin A5
      - MEGA 2560 has an additional SDA at pin 20 and SCL at pin 21
*/

// I2Cdev and MPU6050 must be installed as libraries in Arduino
// Libraries required for the MPU6050
#include <Wire.h>                            // Wire library require for I2C protocol
//#include <I2Cdev.h>                          // Include I2C protocol
#include <MPU6050.h>                         // Include MPU-6050 sensor

MPU6050 gyroAccelTemp;                       // Declare a MPU6050 object instance

//Servo ultrasonicSensorServo;                // Declare a servo object from the Servo class
const int servoPin = 3;                     // Servo Pin

const int trigPin = A5;                     // Trigger Pin of Ultrasonic Sensor
const int echoPin = A4;                     // Echo Pin of Ultrasonic Sensor

//Define the properties of the MPU-6050 (IMU)
#define SAMPLE_RATE 8000                     // This is the default sampling rate in Hz of the MPU 6050
/*If you have any Serial commands at baud rate < 98400, then the above value has to decrease*/
#define ACCEL_X_OFFSET -909                  // Determined using IMU_Zero under File > Examples > MPU6050
#define ACCEL_Y_OFFSET 2057                  // Determined using IMU_Zero under File > Examples > MPU6050
#define ACCEL_Z_OFFSET 1705                  // Determined using IMU_Zero under File > Examples > MPU6050
#define GYRO_X_OFFSET 14                     // Determined using IMU_Zero under File > Examples > MPU6050
#define GYRO_Y_OFFSET -72                    // Determined using IMU_Zero under File > Examples > MPU6050
#define GYRO_Z_OFFSET 60                     // Determined using IMU_Zero under File > Examples > MPU6050



#define ENA 5                                // Enable input for right motors
#define ENB 6                                // Enable input for left motors
#define IN1 7                                // Left forward motion
#define IN2 8                                // Left backward motion
#define IN3 9                                // Right backward motion
#define IN4 11                               // Right forward motion


  float gyroX =0;
  float gyroY =0;
  float gyroZ =0;
unsigned long timer = 0;
float timeStep = 0.01;
  
void setup() {
  /* 38400 chosen because it works well at 8MHz, you may choose anything higher
      However, avoid using anything lower (see note under GYRO_SAMPLE_RATE)
  */


  
  Serial.begin(38400);                       // Initialize Serial Communication

  // Initialize the IMU
  Wire.begin();
  Serial.println("Initializing I2C devices...");
  gyroAccelTemp.initialize();

  // Verify IMU connection
  Serial.println("Testing device connections...");
  Serial.println(gyroAccelTemp.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  gyroAccelTemp.CalibrateGyro();                  // Fully calibrate gyro in about 15 loops
  gyroAccelTemp.CalibrateAccel();                 // Fully calibrate gyro in about 15 loops
  Serial.println("Calibration complete");         // Notify when calibration is complete
  /* The following will NOT get rid of the noise produce by the IMU but they will ensure that
      the average values are at 0 to a certain extent
      (Refer to the IMU Error Determination code and the MPU6050 tutorial video for more details)
  */
  // Set Gyroscope Offsets
  gyroAccelTemp.setXGyroOffset(GYRO_X_OFFSET);    // Set the X gyroscope offset
  gyroAccelTemp.setYGyroOffset(GYRO_Y_OFFSET);    // Set the Y gyroscope offset
  gyroAccelTemp.setZGyroOffset(GYRO_Z_OFFSET);    // Set the Z gyroscope offset

  // Set Acceleration Offsets
  gyroAccelTemp.setXAccelOffset(ACCEL_X_OFFSET);  // Set the X accelerometer offset
  gyroAccelTemp.setXAccelOffset(ACCEL_Y_OFFSET);  // Set the Y accelerometer offset
  gyroAccelTemp.setXAccelOffset(ACCEL_Z_OFFSET);  // Set the Z accelerometer offset

  // IMPORTANT: If you do not calibrate after setting your offset, there will be error
  gyroAccelTemp.CalibrateGyro(6);                 // Fine Tuning purposes of gyroscope
  gyroAccelTemp.CalibrateAccel(6);                // Fine Tuning purposes of accelerometer
  Serial.println("Fine Tuning Complete");         // Notify when fine tuning is complete

  //Pin mode set up
  pinMode(ENA, OUTPUT);                     // Enables left wheels to move
  pinMode(ENB, OUTPUT);                     // Enables right wheels to move
  // An analog input (instead of a digital) can also be used to set the speed (instead of using 0 and max)
  pinMode(IN1, OUTPUT);                     // Allows left wheels to turn forward when HIGH
  pinMode(IN2, OUTPUT);                     // Allows left wheels to turn backwards when HIGH
  pinMode(IN3, OUTPUT);                     // Allows right wheels to turn forward when HIGH
  pinMode(IN4, OUTPUT);                     // Allows right wheels to turn backwards when HIGH


}

void loop() {
  float accelX, accelY, accelZ;
  float temp;

timer  =  millis();
  // Get the data from the on-board thermometer
  /*
   * According to the MPU 6050 datasheet, the raw value of the temperature should be divided
   * by 340.0 and adding 36.53 to output the temperateure in units of Celsius
   *    - The 340 accounts for the conversion of the raw data to proper units
   *    - the 36.53 accounts for the offset
   */
  temp = gyroAccelTemp.getTemperature() / 340.0 + 36.53;

  // Get the data from the Accelerometer
  /*
   * According to the MPU 6050 datasheet, the raw value of the acceleration should be divided
   * by 16384.0 to output acceleration in units of g, which is about 9.8 m/s^2
   *    - The 16384.0 accounts conversion of the raw data to proper units
   * Resolution: +- 2g
   */
  accelX = gyroAccelTemp.getAccelerationX() / 16384.0 * 4;  // Get current X axis acceleration in meters per second^2
  accelY = gyroAccelTemp.getAccelerationY() / 16384.0 * 4;  // Get current Y axis acceleration in meters per second^2
  accelZ = gyroAccelTemp.getAccelerationZ() / 16384.0 * 4;  // Get current Z axis acceleration in meters per second^2

  // Get the data from the Gyroscope
  /*
   * According to the MPU 6050 datasheet, the raw value of the gyroscope should be divided
   * by 131.0 to output rotation in units of degrees/s
   *    - The 131.0 accounts conversion of the raw data to proper units
   * Resolution: +- 250 degrees/s
   */
  gyroX = gyroX + gyroAccelTemp.getRotationX() / 131.0 *2.8 *timeStep;         // Get current X axis orientation in degress per second
  gyroY = gyroY + gyroAccelTemp.getRotationY() / 131.0  *2.8* timeStep;         // Get current Y axis orientation in degress per second
  gyroZ = gyroZ + gyroAccelTemp.getRotationZ() / 131.0 *3.22 * timeStep;         // Get current Z axis orientation in degress (conversion from degrees per second

delay((timeStep*1000) - (millis() - timer));
  /* The following functions may also be used to extract data from the Gyroscgope and the Accelerometer:
      - gyroAccelTemp.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
          --> Receive all of the Accelerometer and Gyroscope data using 1 function
      - gyroAccelTemp.getAcceleration(&accelX, &accelY, &accelZ);
          --> Receive all of the Accelerometer data using 1 function
      - gyroAccelTemp.getRotation(&gyroX, &gyroY, &gyroZ)
          --> Receive all of the Gyroscope data using 1 function
  */

  Serial.println();
  // Print out temperature data
  Serial.print("Temperature (Celsius): ");
  Serial.println(temp);
  // Print out Acceleromter data
  Serial.print("Accelerometer (g)\tX: ");
  Serial.print(accelX);
  Serial.print("\tY: ");
  Serial.print(accelY);
  Serial.print("\tZ: ");
  Serial.println(accelZ);
  // Print out Gyroscope data
  Serial.print("Gyroscope (degrees)\t\tX: ");
  Serial.print(gyroX);
  Serial.print("\tY: ");
  Serial.print(gyroY);
  Serial.print("\tZ: ");
  Serial.println(gyroZ);

  setMaxSpeed();
  //turnLeft();
  //delay(1000);
  TurnAround();
  delay(1);
  stopMovingForever();
}


//Functions
//
//

void TurnAround(){                          //Function to turn car around based on gyro data.
float z = 180;
if(gyroZ<z){
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
