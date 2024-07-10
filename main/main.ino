#include "libs.h"
#include <MPU6050_light.h>
#include "Wire.h"

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

#define MOT_1_PWM 11 //These are pins for the motors
#define MOT_2_PWM 12 //These are pins for the motors
#define MOT_1_DIR 9 //These are pins for the motors
#define MOT_2_DIR 10 //These are pins for the motors



mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(33/2, 102/2); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;

mtrn3100::Motor motor1(MOT_1_PWM, MOT_1_DIR);
mtrn3100::Motor motor2(MOT_2_PWM, MOT_2_DIR);


void setup() {
  Serial.begin(9600);
  Wire.begin();
  /*
  //Set up the IMU
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true);
  Serial.println("Done!\n");
  */

}

void loop() {
  // put your main code here, to run repeatedly:
    delay(50);
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

}
