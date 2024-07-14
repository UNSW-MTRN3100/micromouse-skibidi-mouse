// Easy search: COMPONENTS, ACTION, TODO
#include "libs.hpp"
#include <MPU6050_light.h>
#include "Wire.h"

// ACTION: Intialise variables for the yaw MPU
unsigned long timer = 0;
float yaw = 0.0;
// ACTION: Tune this value for filtering
float alpha = 0.72;
float gyroZ;
float gyroZFiltered = 0.0;

MPU6050 mpu(Wire);

// COMPONENTS: Encoders pin
#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

// COMPONENTS: Motor pin
#define MOT_1_PWM 11 //These are pins for the motors
#define MOT_2_PWM 12 //These are pins for the motors
#define MOT_1_DIR 9 //These are pins for the motors
#define MOT_2_DIR 10 //These are pins for the motors

// ACTION: Initialise dual encoder, IMU
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
// TODO: Change this dimensions (mm)
mtrn3100::EncoderOdometry encoder_odometry(31.3/2,127.5); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;

// ACTION: Motor initialise
mtrn3100::Motor motor1(MOT_1_PWM, MOT_1_DIR);
mtrn3100::Motor motor2(MOT_2_PWM, MOT_2_DIR);

// ACTION: Bangbang controller initialise
// TODO: Tune the value
mtrn3100::BangBangController controller(125, 10); // in PWM
mtrn3100::BangBangController controllerR(125, 5 * M_PI /180); // in rad
mtrn3100::BangBangController controllerL(125, 5 * M_PI /180); // in rad

// ACTION: PIDController initialise
// TODO: Tune the value
// mtrn3100::PIDController controller(500, 0, 0); // in PWM
// mtrn3100::PIDController controllerR(500, 0, 0); // in rad
// mtrn3100::PIDController controllerL(500, 0, 0); // in rad

String commands = "fflfrflf";
void setup() {
  // ACTION: Setup the terminal
  Serial.begin(115200);

  // ACTION: Set up the IMU
  // mpuSetup();

  // ACTION: Setup the BangbangController
  controller.zeroAndSetTarget(encoder.getLeftRotation(), 150); // in mm
}

void loop() {
  delay(50);

  // ACTION: Test Motor, Odometry, IMU and Bang Bang
  // TODO: Test this 
  // test();

  // ACTION: Read in commands and processing
  // TODO: Test this
  for (char command : commands) {
    Serial.print("Processing command: ");
    Serial.println(command);
    switch (command) {
      case 'f':
        Serial.println("Move forward");
        
        // ACTION: Move Straight
        // TODO: change the smapling time?
        driveStraight();     
        driveStop();   
        break;
      case 'l':
        Serial.println("Turn left");
        turnLeft(90 * M_PI / 180);
        driveStop();  
        break;
      case 'r':
        Serial.println("Turn right"); 
        turnRight(-90 * M_PI / 180);
        driveStop();  
        break;
    }
    delay(1000);
  }

  while(true) {
    Serial.println("Finishing command");
    delay(1000);
    
  }

}

void mpuSetup() {
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true);
  Serial.println("Done!\n");
}

void driveStraight() {
  // ACTION: Zero and Set Target
  controller.zeroAndSetTarget(0, 250); // in mm

  // TODO: Tune the error
  while (controller.getError() < 25) {
    // ACTION: Update postition
    encoder_odometry.update(encoder.getRightRotation(),encoder.getLeftRotation());
    displayEncoderOdom(encoder_odometry.getX(), encoder_odometry.getH());
    
    // ACTION: Comput the control signal
    int controlSignal = controller.compute(encoder_odometry.getX());
  
    // Bangbang controller
    // TODO: Test this
    Serial.print("The calculated speed is: ");
    Serial.println(controlSignal);

    // ACTION: Use the Control Signal to calculate
    motor1.setPWM(controlSignal);
    motor2.setPWM(-controlSignal);
  }
}

void turnLeft(float rad){
  // ACTION: Zero and Set Target
  controllerL.zeroAndSetTarget(0, rad); // in rad

  // TODO: Tune the error
  while (controller.getError() < 10 * M_PI/180) {
    // ACTION: Update postition
    encoder_odometry.update(encoder.getRightRotation(),encoder.getLeftRotation());
    displayEncoderOdom(0, encoder_odometry.getH());
    
    // ACTION: Comput the control signal
    int controlSignal = controller.compute(encoder_odometry.getH());
  
    // Bangbang controller
    // TODO: Test this
    Serial.print("The calculated speed is: ");
    Serial.println(controlSignal);

    // ACTION: Use the Control Signal to calculate
    motor1.setPWM(controlSignal);
    motor2.setPWM(controlSignal);
  }
}

void turnRight(float rad) {
  // ACTION: Zero and Set Target
  controllerR.zeroAndSetTarget(0, rad); // in rad

  // TODO: Tune the error
  while (controller.getError() < 10 * M_PI/180) {
    // ACTION: Update postition
    encoder_odometry.update(encoder.getRightRotation(),encoder.getLeftRotation());
    displayEncoderOdom(0, encoder_odometry.getH());
    
    // ACTION: Comput the control signal
    int controlSignal = controller.compute(encoder_odometry.getH());
  
    // Bangbang controller
    // TODO: Test this
    Serial.print("The calculated speed is: ");
    Serial.println(controlSignal);

    // ACTION: Use the Control Signal to calculate
    motor1.setPWM(-controlSignal);
    motor2.setPWM(-controlSignal);
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

  // Run the motor
  // TODO: Test this 
  testMotor();

  // Encoder Odometry and Bang Bang
  testEncoderAndController(); 
}

void testMPU() {
   mpuGetYaw();
}

void mpuGetYaw() {
  mpu.update();
  gyroZ = mpu.getGyroZ();
  float dt = (millis() - timer) / 1000.0;
  gyroZFiltered = alpha * gyroZFiltered + (1 - alpha) * gyroZ;
  yaw += gyroZFiltered * dt; // integrate the gyroZ to get yaw
  timer = millis();
  Serial.print("Yaw: ");
  Serial.println(yaw);
}

void testMotor() {
  Serial.println("Both wheels move the same time");
  motor1.setPWM(255);
  motor2.setPWM(255);
  delay(500);

  Serial.println("Both wheels move opposite of each other");
  motor1.setPWM(255);
  motor2.setPWM(-255);
  delay(500);

  Serial.println("Both wheels move opposite of each other (opposite)");
  motor1.setPWM(-255);
  motor2.setPWM(255);
  delay(500);

  Serial.println("Both wheels move the same as each other (opposite)");
  motor1.setPWM(-255);
  motor2.setPWM(-255);
  delay(500);
  
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
