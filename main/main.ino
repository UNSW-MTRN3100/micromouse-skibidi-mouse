// Easy search: COMPONENTS, ACTION, TODO
#include "libs.hpp"
#include <MPU6050_light.h>
#include "Wire.h"
#include <VL6180X.h>
#include <Wire.h>
// ACTION: Intialise the sensors
VL6180X sensor1;
VL6180X sensor2;
VL6180X sensor3;
int sensor1_pin = A0; // ENABLE PIN FOR SENSOR 1
int sensor2_pin = A1; // ENABLE PIN FOR SENSOR 2
int sensor3_pin = A2; // ENABLE PIN FOR SENSOR 3
// ACTION: Intialise variables for the yaw MPU6050
unsigned long timer = 0;
float gyroZ = 0;
float yaw = 0;
MPU6050 mpu(Wire);
// COMPONENTS: Encoders pin
#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder
// COMPONENTS: Motor pin
#define MOT_1_PWM 11 //These are pins for the motors
#define MOT_1_DIR 12 //These are pins for the motors
#define MOT_2_PWM 9 //These are pins for the motors
#define MOT_2_DIR 10 //These are pins for the motors
// ACTION: Initialise dual encoder, IMU
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
// TODO: Change this dimensions (mm)
mtrn3100::EncoderOdometry encoder_odometry(31.3/2, 105/2); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;
// ACTION: Motor initialise
mtrn3100::Motor motor1(MOT_1_PWM, MOT_1_DIR);
mtrn3100::Motor motor2(MOT_2_PWM, MOT_2_DIR);
// ACTION: Bangbang controller initialise
// TODO: Tune the value
mtrn3100::BangBangController controller(125, 1); // in PWM
mtrn3100::BangBangController controllerR(125/3, 3); // in deg
mtrn3100::BangBangController controllerL(125/3, 3); // in deg
// ACTION: PIDController initialise
// TODO: Tune the value
// mtrn3100::PIDController controller(500, 0, 0); // in PWM
// mtrn3100::PIDController controllerR(500, 0, 0); // in rad
// mtrn3100::PIDController controllerL(500, 0, 0); // in rad
void setup() {
  // ACTION: Setup the LiDar
  lidarSetup();
  
  // ACTION: Setup the terminal
  serialSetup();
  // ACTION: Set up the IMU
   mpuSetup();
  // ACTION: Setup the BangbangController
  controllerSetup();
  Serial.println("Setup function end");
}
void loop() {
  delay(50);
  // ACTION: Test Motor, Odometry, IMU and Bang Bang
  // TODO: Test this 
//  testMPU();
  // ACTION: Read in commands and processing
  // TODO: Test this
  processCommands("flflflfl");
}
void controllerSetup() {
  controller.zeroAndSetTarget(0, 150); // in mm
  controllerL.zeroAndSetTarget(0, 90); // in mm
  controllerR.zeroAndSetTarget(0, -90); // in mm
}
void serialSetup() {
    Serial.begin(115200);  
}
void lidarSetup() {
  Wire.begin();
  // SET UP ENABLE PINS AND DISABLE SENSORS
  pinMode(sensor1_pin, OUTPUT);
  pinMode(sensor2_pin, OUTPUT);
  pinMode(sensor3_pin, OUTPUT);
  digitalWrite(sensor1_pin, LOW);
  digitalWrite(sensor2_pin, LOW);
  digitalWrite(sensor3_pin, LOW);
  // ENABLE FIRST SENSOR AND CHANGE THE ADDRESS 
  digitalWrite(sensor1_pin, HIGH);
  delay(50);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setTimeout(250);
  sensor1.setAddress(0x54);
  delay(50);
  
  // ENABLE SECOND SENSOR AND CHANGE THE ADDRESS 
  // NOTE: WE DO NOT HAVE TO DISABLE THE FIRST SENSOR AS IT IS NOW ON A DIFFERENT ADDRESS 
  digitalWrite(sensor2_pin, HIGH);
  delay(50);
  sensor2.init();
  sensor2.configureDefault();
  sensor2.setTimeout(250);
  sensor2.setAddress(0x62);
  delay(50);
  // ENABLE THIRD SENSOR AND CHANGE THE ADDRESS 
  digitalWrite(sensor3_pin, HIGH);
  delay(50);
  sensor3.init();
  sensor3.configureDefault();
  sensor3.setTimeout(250);
  sensor3.setAddress(0x58);
  delay(50);
}
void mpuSetup() {
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(500);
  mpu.calcOffsets(true,true);
  Serial.println("Done!\n");
}
void processCommands(String commands) {
    for (char command : commands) {
    Serial.print("Processing command: ");
    Serial.println(command);
    // TODO: change the smapling time?
    switch (command) {
      case 'f':
        Serial.println("Move forward");
        driveStraight();
        driveStop();
        break;
      case 'l':
        Serial.println("Turn left");
        turnLeft(90, 3);
        driveStop();  
        break;
      case 'r':
        Serial.println("Turn right"); 
        turnRight(-90, 3);
        driveStop();  
        break;
    }
    delay(100);
  }
  while(true) {
    Serial.println("Finishing command");
    driveStop();
    delay(1000);
    
  }
}
void driveStraight() {
  Serial.println("Driving Straight");
  float startingYaw = getYawMPU();
  // ACTION: Set checkpoint
  controller.zeroAndSetTarget(encoder_odometry.getX(), 50); // in mm
  // ACTION: Computing current state
  controller.compute(encoder_odometry.getX());
  encoder.reset();
  // ACTION: Zero and Set Target
  controller.zeroAndSetTarget(encoder_odometry.getX(), 50); // in mm
  // TODO: Tune the error
  // ACTION: Check if it's adjusted
  while (fabs(controller.getError()) > 1) {
    Serial.print("The error is ");
    Serial.println(controller.getError());
    // ACTION: Update postition
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    displayEncoderOdom(encoder_odometry.getX(), encoder_odometry.getH());
    
    // ACTION: Comput the control signal
    int controlSignal = controller.compute(encoder_odometry.getX());
    // Bangbang controller
    // TODO: Test this
    Serial.print("The calculated speed is: ");
    Serial.println(controlSignal);
    
//    // ACTION: Detect left wall using LiDar
//    // TODO: Adjust the value (distance leftWall)
//    int leftWall = sensor1.readRangeSingleMillimeters();
//    int frontWall = sensor2.readRangeSingleMillimeters();
//    int rightWall = sensor3.readRangeSingleMillimeters();
//
//    if (leftWall < 43 || rightWall < 43 || frontWall < 10) {
//      if (leftWall < 43) {
//        motor1.setPWM(-controlSignal - 20);
//        motor2.setPWM(controlSignal + 20);
//        continue;
//      } else if (rightWall < 43) {
//        motor1.setPWM(-controlSignal + 20);
//        motor2.setPWM(controlSignal + 20);
//        continue;
//      } else if (frontWall < 10) {
//        driveStop();
//        break;
//      }     
//    }
//    // ACTION: Detect drifitng using MPU
//    if (getYawMPU() - startingYaw > 2) {
//      turnRight(2, 2);  
//    } 
//    if (getYawMPU() - startingYaw < -2) {
//      turnLeft(2, 2);  
//    } 
    
    // ACTION: Use the Control Signal to calculate
    straight(controlSignal);
  }
  
}
void straight(int pwm) {
  motor1.setPWM(-pwm);
  motor2.setPWM(pwm);
}
void turnLeft(float degree, float error){
  // ACTION: Set checkpoint
  controllerL.zeroAndSetTarget(getYawMPU(), degree); // in mm
  // ACTION: Computing current state
  controllerL.compute(getYawMPU());
  // ACTION: Zero and Set Target
  controllerL.zeroAndSetTarget(getYawMPU(), degree); // in mm
  Serial.print("The error is ");
  Serial.println(controllerL.getError());
  // TODO: Tune the error
  while (fabs(controllerL.getError()) > error) {
    Serial.print("The error is ");
    Serial.println(controllerL.getError());
    // ACTION: Update postition
    float currYaw = getYawMPU();
    controllerL.compute(currYaw);
    
    // ACTION: Comput the control signal
    int controlSignal = controllerL.compute(currYaw);
    // Bangbang controller
    // TODO: Test this
    Serial.print("The calculated speed is: ");
    Serial.println(controlSignal);
    // ACTION: Use the Control Signal to calculate
    motor1.setPWM(controlSignal);
    motor2.setPWM(controlSignal);
  }
}
void turnRight(float degree, float error) {
  // ACTION: Set checkpoint
  controllerR.zeroAndSetTarget(getYawMPU(), degree); // in mm
  // ACTION: Computing current state
  controllerR.compute(getYawMPU());
  // ACTION: Zero and Set Target
  controllerR.zeroAndSetTarget(getYawMPU(), degree); // in mm
  Serial.print("The error is ");
  Serial.println(controllerR.getError());
  // TODO: Tune the error
  while (fabs(controllerR.getError()) > error) {
    Serial.print("The error is ");
    Serial.println(controllerR.getError());
    // ACTION: Update postition
    float currYaw = getYawMPU();
    // ACTION: Comput the control signal
    int controlSignal = controllerR.compute(currYaw);
    // Bangbang controller
    // TODO: Test this
    Serial.print("The calculated speed is: ");
    Serial.println(controlSignal);
    // ACTION: Use the Control Signal to calculate
    motor1.setPWM(controlSignal);
    motor2.setPWM(controlSignal);
  }
}
void driveStop() {
  motor1.setPWM(0);
  motor2.setPWM(0);
}
void test() {
  // ACTION: Get rotation from the IMU 
  // TODO: It works if we do it slow/fast enough
  // TODO: Tune the alpha
  testMPU();
  Serial.println("Test MPU ending");
  // ACTION: Run the motor
  // TODO: Test this 
  testMotor();
  // ACTION: Test Encoder Odometry and Bang Bang
  testEncoderAndController();
  // ACTION: Test LiDar
  testLiDar();
}
void testLiDar() {
  Serial.println("LiDar Reading.....");
  Serial.print("Left: ");
  Serial.print(sensor1.readRangeSingleMillimeters());
  Serial.print(" | ");
  Serial.print("Front: ");
  Serial.print(sensor2.readRangeSingleMillimeters());
  Serial.print(" | ");
  Serial.print("Right: ");
  Serial.print(sensor3.readRangeSingleMillimeters());
  Serial.println();
  if (sensor1.timeoutOccurred()) { Serial.println("Sensor 1 TIMEOUT"); }
  if (sensor2.timeoutOccurred()) { Serial.println("Sensor 2 TIMEOUT"); }
  if (sensor3.timeoutOccurred()) { Serial.println("Sensor 3 TIMEOUT"); }
  delay(50);
}
void testMPU() {
  getYawMPU();
}
float getYawMPU() {
  mpu.update();
  gyroZ = mpu.getGyroZ();
  float dt = (millis() - timer) / 1000.0;
  yaw += kalmanFilter(gyroZ) * dt; 
  timer = millis();
  Serial.print("Yaw: ");
  Serial.println(yaw);
  return yaw;
}
float kalmanFilter(float U) {
  static const double R = 500;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P*H/(H*P*H + R);
  U_hat = U_hat + K*(U - H*U_hat);
  P = (1-K*H)*P + Q;
  return U_hat;
}
void testMotor() {
  Serial.println("Both wheels move the same time");
  motor1.setPWM(255);
  motor2.setPWM(-255);
  delay(2000);
  Serial.println("Both wheels move opposite of each other");
  motor1.setPWM(255);
  motor2.setPWM(255);
  delay(2000);
  Serial.println("Both wheels move opposite of each other (opposite)");
  motor1.setPWM(-255);
  motor2.setPWM(-255);
  delay(2000);
  Serial.println("Both wheels move the same as each other (opposite)");
  motor1.setPWM(-255);
  motor2.setPWM(255);
  delay(2000);
  
}
void testEncoderAndController() {
  // Bangbang controller
  // TODO: Test this
  Serial.print("The calculated speed is: ");
  Serial.println(controller.compute(encoder_odometry.getX()));
  
  // Calculate Odom through enocders
  // TODO: change the smapling time?
  encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
  displayEncoderOdom(encoder_odometry.getX(), encoder_odometry.getH());
}
void displayEncoderOdom(float x, float h) {
  Serial.print("ODOM:\t\t x: ");
  Serial.print(x);
  Serial.print(",\t\t h: ");
  Serial.print(h);
  Serial.println();
}
