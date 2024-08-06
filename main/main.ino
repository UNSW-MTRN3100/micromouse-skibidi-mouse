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
mtrn3100::EncoderOdometry encoder_odometry(31.73/2, 104/2); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;

// ACTION: Motor initialise
mtrn3100::Motor motor1(MOT_1_PWM, MOT_1_DIR);
mtrn3100::Motor motor2(MOT_2_PWM, MOT_2_DIR);

// ACTION: Bangbang controller initialise
// TODO: Tune the value
mtrn3100::BangBangController controller(125, 4); // in PWM
mtrn3100::BangBangController controllerR(125/3, 3); // in deg
mtrn3100::BangBangController controllerL(125/3, 3); // in deg

// ACTION: PIDController initialise
// TODO: Tune the value
//mtrn3100::PIDController controller(100, 0, 30); // in PWM
//mtrn3100::PIDController controllerR(50, 0, 0); // in rad
//mtrn3100::PIDController controllerL(50, 0, 0); // in rad


////////////////////////////////////

enum ActionType {
    ACTION_TASK2_1,
    ACTION_TASK3_1,
    ACTION_TASK3_2,
    ACTION_UNKNOWN
};

String Start_direction;
int start_x; 
int start_y; 
String Stop_direction;
int stop_x; 
int stop_y;


float mazeCompletionPercentage;
float speedRank;

int flag = 0; 
/////////////////////////////////////


int firstLeftWall = 0;
int firstRightWall = 0;

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
//   Serial.println(encoder.getLeftRotation());
//   Serial.println(encoder.getRightRotation());

  // ACTION: Read in commands and processing
  // TODO: Test this
 // String commands = "ffff";
//String commands = "ffrrfrfl";
 // processCommands(commands);

  if (flag == 0 ){
    //If continuous looping is desired, the flag can be removed, but I think the flag is necessary for reuse. Perform a reset on the card.

    performActionBasedOnString("Task2_1");  // for task 2.1 
 //   performActionBasedOnString("Task3_1"); 
  }
  
}

void controllerSetup() {
  controller.zeroAndSetTarget(0, 250); // in mm
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

  firstLeftWall = 80;
  firstRightWall = 80;
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
        turnLeft(88, 3);
        driveStop();  
        encoder_odometry.reset();
        break;
      case 'r':
        Serial.println("Turn right"); 
        turnRight(-88, 3);
        driveStop();  
        encoder_odometry.reset();
        break;
    }
    delay(50);
  }

  while(true) {
    Serial.println("Finishing command");
    driveStop();
    delay(1000);
    
  }
}

void driveStraight() {
  Serial.println("Driving Straight");

  // ACTION: Get the currYaw
  float startingYaw = getYawMPU();

  
  // ACTION: Set checkpoint
  controller.zeroAndSetTarget(encoder_odometry.getX(), 250); // in mm
  // ACTION: Computing current state
  controller.compute(encoder_odometry.getX());

  // TODO: Tune the error
  // ACTION: Check if it's adjusted
  while (fabs(controller.getError()) > 5) {
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

    float leftWall = sensor1.readRangeSingleMillimeters();
    float rightWall = sensor3.readRangeSingleMillimeters();
    float frontWall = sensor2.readRangeSingleMillimeters();

    if (frontWall < 85) {
      break;
    }
    // difference of 2 LiDars
    if (leftWall < 79) {
      // Turn left
      motor1.setPWM(-controlSignal);
      motor2.setPWM(controlSignal - 10);
      continue;
    } 

    if (rightWall < 79) {
      // Turn Right
      motor1.setPWM(-controlSignal + 10);
      motor2.setPWM(controlSignal);
      continue;
    }

//    if(rightWall > 81 && leftWall > 81) {
//      float yaw = getYawMPU();
//      if (yaw - startingYaw > 0.04) {
//        motor1.setPWM(-controlSignal + 10);
//        motor2.setPWM(controlSignal);
//        continue;
//      } else if (yaw - startingYaw < -.04) {
//        motor1.setPWM(-controlSignal);
//        motor2.setPWM(controlSignal - 10);
//        continue;
//      }
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
   encoder.reset();
  
}

void turnRight(float degree, float error) {
  // ACTION: Set checkpoint
  controllerR.zeroAndSetTarget(getYawMPU(), degree); // in mm
  // ACTION: Computing current state
  controllerR.compute(getYawMPU());

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
  encoder.reset();
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

//float data[100] = {0};
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

///////////////////////////////////////////////////////////////////////



ActionType stringToActionType(const String& action) {
    if (action == "Task2_1") {
        return ACTION_TASK2_1;
    } 
    else if (action == "Task3_1") {
        return ACTION_TASK3_1;
    }
    else if (action == "Task3_2") {
        return ACTION_TASK3_2;
    }
    else {
        return ACTION_UNKNOWN;
    }
}

void performActionBasedOnString(const String& action) {
    ActionType actionType = stringToActionType(action);

    switch (actionType) {
        case ACTION_TASK2_1:
            Task2_1();
            break;
        case ACTION_TASK3_1:
            Task3_1(Start_direction, start_x, start_y, Stop_direction, stop_x, stop_y);
            break;
        case ACTION_TASK3_2:
            Task_3_2(mazeCompletionPercentage, speedRank) ;
            break;
        default:
            Serial.println("Unknown action type!");
            break;
    }
}


void Task2_1() {
    Serial.println("Driving Straight");

    // ACTION: Get the currYaw
    float startingYaw = getYawMPU();

    // ACTION: Set checkpoint
    controller.zeroAndSetTarget(encoder_odometry.getX(), 250); // in mm

    // ACTION: Initialize variables
    float checkpointDistance = 250; // Distance to reach per checkpoint
    float currentDistance = encoder_odometry.getX();
    bool isAligned = false;
    int numCellsReached = 0;
    
    while (fabs(controller.getError()) > 5) {
        Serial.print("The error is ");
        Serial.println(controller.getError());
        
        // ACTION: Update position
        encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
        displayEncoderOdom(encoder_odometry.getX(), encoder_odometry.getH());

        // ACTION: Compute control signal
        int controlSignal = controller.compute(encoder_odometry.getX());
        Serial.print("The calculated speed is: ");
        Serial.println(controlSignal);

        float leftWall = sensor1.readRangeSingleMillimeters();
        float rightWall = sensor3.readRangeSingleMillimeters();
        float frontWall = sensor2.readRangeSingleMillimeters();

        // ACTION: Check if robot should stop
        if (frontWall < 85) {
            Serial.println("Collision detected at front wall!");
            break;
        }

        // ACTION: Adjust robot's direction based on wall proximity
        if (leftWall < 79) {
            // Turn left
            motor1.setPWM(-controlSignal);
            motor2.setPWM(controlSignal - 10);
        } else if (rightWall < 79) {
            // Turn right
            motor1.setPWM(-controlSignal + 10);
            motor2.setPWM(controlSignal);
        } else {
            // Move straight
            straight(controlSignal);
        }

        // ACTION: Check if a cell is reached
        if (encoder_odometry.getX() >= (checkpointDistance * (numCellsReached + 1))) {
            numCellsReached++;
            Serial.print("Reached cell ");
            Serial.println(numCellsReached);
            
            // Check if final cell is reached and align robot
            if (numCellsReached == 5) {
                Serial.println("Final cell reached.");
                float finalPosition = encoder_odometry.getX();
                float distanceToCenter = fabs(finalPosition - (checkpointDistance * 5));
                if (distanceToCenter <= 25) {
                    isAligned = true;
                    Serial.println("Robot aligned within 25mm of center.");
                } else {
                    Serial.println("Robot not aligned within 25mm of center.");
                }
                break;
            }
        }
    }

    // ACTION: Final status
    if (isAligned) {
        Serial.println("Task completed successfully.");
    } else {
        Serial.println("Task failed.");
    }
    flag = 1;
}

void Task3_1(String Start_direction, int start_x, int start_y, String Stop_direction, int stop_x, int stop_y) {
    Serial.println("Starting Autonomous Labyrinth Navigation");

    // Labyrinth start and end coordinates
    const float startX = start_x;
    const float startY = start_y;
    const float endX = stop_x;
    const float endY = stop_y;

    // Target coordinates
    float currentX = encoder_odometry.getX(); // Current X position
    float currentY = encoder_odometry.getH(); // Current Y position
    float targetX = startX;
    float targetY = startY;

    bool navigating = true;
    bool reachedTarget = false;

    // Main navigation loop
    while (navigating) {
        // Update current position
        currentX = encoder_odometry.getX();
        currentY = encoder_odometry.getH();

        // Calculate distance to the target
        float distanceToTarget = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));

        // Check if the target has been reached
        if (distanceToTarget < 0.5) { // Distance threshold (e.g., 0.5 units)
            Serial.println("Reached target position!");
            reachedTarget = true;
            // If target reached, now navigate to the final destination
            targetX = endX;
            targetY = endY;
        }

        if (reachedTarget && fabs(currentX - endX) < 0.5 && fabs(currentY - endY) < 0.5) {
            Serial.println("Reached final destination.");
            navigating = false; // Stop navigation
            break; // Exit the loop
        }

        // Detect the environment (using Lidar and IMU data)
        float lidarLeft = sensor1.readRangeSingleMillimeters();
        float lidarFront = sensor2.readRangeSingleMillimeters();
        float lidarRight = sensor3.readRangeSingleMillimeters();

        // Lidar timeout checks
        if (sensor1.timeoutOccurred()) { Serial.println("Sensor 1 TIMEOUT"); }
        if (sensor2.timeoutOccurred()) { Serial.println("Sensor 2 TIMEOUT"); }
        if (sensor3.timeoutOccurred()) { Serial.println("Sensor 3 TIMEOUT"); }

        // Target alignment
        float angleToTarget = atan2(targetY - currentY, targetX - currentX) * 180 / PI;
        float angleError = angleToTarget - getYawMPU(); // Read IMU yaw using your function

        // Align to target
        if (fabs(angleError) > 5) {
            if (angleError > 0) {
                turnLeft(fabs(angleError), 5);
            } else {
                turnRight(fabs(angleError), 5);
            }
        } else {
            // Move based on Lidar distances
            if (lidarFront < 1000) { // Obstacle detected (e.g., 1000 mm threshold)
                avoidObstacle(lidarLeft, lidarFront, lidarRight); // Avoid obstacle
            } else {
                int controlSignal = controller.compute(distanceToTarget);
                straight(controlSignal);
            }
        }

        // Update position and control signals
        encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
        displayEncoderOdom(currentX, currentY);

        delay(50); // Delay time, adjustable based on need
    }

    // Stop motors when task is complete
    driveStop();
    flag = 1;
}


void avoidObstacle(float lidarLeft, float lidarFront, float lidarRight) {
    Serial.println("Obstacle detected. Avoiding.");
    
    if (lidarFront < 1000) {
        // Avoid obstacle in front
        if (lidarLeft > lidarRight) {
            turnLeft(90, 5); // Turn right
        } else {
            turnRight(90, 5); // Turn left
        }
        straight(-10); // Move backward
    } else {
        // Avoid obstacle from the sides
        if (lidarLeft < lidarRight) {
            turnRight(90, 5); // Turn left
        } else {
            turnLeft(90, 5); // Turn right
        }
        straight(-10); // Move backward
    }
}

void Task_3_2(float mazeCompletionPercentage, float speedRank) {
    int mazeCompletionMark = 0;
    int speedMark = 0;
    int totalMarks = 0;

    // Table 1: Maze Completion Marks
    if (mazeCompletionPercentage >= 100.0) {
        mazeCompletionMark = 5;
    } else if (mazeCompletionPercentage >= 80.0) {
        mazeCompletionMark = 4;
    } else if (mazeCompletionPercentage >= 60.0) {
        mazeCompletionMark = 3;
    } else if (mazeCompletionPercentage >= 40.0) {
        mazeCompletionMark = 2;
    } else if (mazeCompletionPercentage >= 20.0) {
        mazeCompletionMark = 1;
    } else {
        mazeCompletionMark = 0;
    }

    // Table 2: Speed Marks
    if (speedRank <= 10.0) {  // Top 10% of teams
        speedMark = 3;
    } else if (speedRank <= 25.0) {  // Top 25% of teams
        speedMark = 2;
    } else if (speedRank <= 50.0) {  // Top 50% of teams
        speedMark = 1;
    } else {
        speedMark = 0;
    }

    // Total marks are the sum of maze completion marks and speed marks
    totalMarks = mazeCompletionMark + speedMark;

    // Print results
    Serial.print("Maze Completion Mark: ");
    Serial.println(mazeCompletionMark);
    Serial.print("Speed Mark: ");
    Serial.println(speedMark);
    Serial.print("Total Marks: ");
    Serial.println(totalMarks);
}