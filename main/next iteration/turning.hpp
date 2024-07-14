//#pragma once
//
//#include <Arduino.h>
//#include "Motor.hpp"
//#include "PIDController.hpp"
//#include "IMU.hpp"
//
//namespace mtrn3100 {
//
//class Turn90Degrees {
//public:
//    Turn90Degrees(Motor& leftMotor, Motor& rightMotor, IMU& imu, PIDController& pidController)
//        : leftMotor(leftMotor), rightMotor(rightMotor), imu(imu), pidController(pidController) {}
//
//    // Function to turn the robot 90 degrees to the right
//    void turnRight() {
//        // Reset PID controller and IMU
//        pidController.zeroAndSetTarget(0, 90);  // Target 90 degrees
//        imu.resetYaw();
//
//        while (true) {
//            float currentYaw = imu.getYaw();
//            float error = 90 - currentYaw;  // Calculate error to reach 90 degrees
//
//            // Check if the target angle is reached
//            if (abs(error) < 1.0) {
//                // Stop motors if target angle is reached
//                leftMotor.setPWM(0);
//                rightMotor.setPWM(0);
//                break;
//            }
//
//            // Compute control signal using PID controller
//            float controlSignal = pidController.compute(currentYaw);
//
//            // Set motor speeds based on control signal (adjust direction as needed)
//            leftMotor.setPWM(-controlSignal);  // Reverse direction to turn right
//            rightMotor.setPWM(controlSignal);
//        }
//    }
//
//    // Function to turn the robot 90 degrees to the left
//    void turnLeft() {
//        // Reset PID controller and IMU
//        pidController.zeroAndSetTarget(0, -90);  // Target -90 degrees (left turn)
//        imu.resetYaw();
//
//        while (true) {
//            float currentYaw = imu.getYaw();
//            float error = -90 - currentYaw;  // Calculate error to reach -90 degrees (left turn)
//
//            // Check if the target angle is reached
//            if (abs(error) < 1.0) {
//                // Stop motors if target angle is reached
//                leftMotor.setPWM(0);
//                rightMotor.setPWM(0);
//                break;
//            }
//
//            // Compute control signal using PID controller
//            float controlSignal = pidController.compute(currentYaw);
//
//            // Set motor speeds based on control signal (adjust direction as needed)
//            leftMotor.setPWM(controlSignal);  // No need to reverse for left turn
//            rightMotor.setPWM(-controlSignal);
//        }
//    }
//
//private:
//    Motor& leftMotor;
//    Motor& rightMotor;
//    IMU& imu;
//    PIDController& pidController;
//};
//
//}  // namespace mtrn3100#pragma once
//
//#include <Arduino.h>
//#include "Motor.hpp"
//#include "PIDController.hpp"
//#include "IMU.hpp"
//
//namespace mtrn3100 {
//
//class Turn90Degrees {
//public:
//    Turn90Degrees(Motor& leftMotor, Motor& rightMotor, IMU& imu, PIDController& pidController)
//        : leftMotor(leftMotor), rightMotor(rightMotor), imu(imu), pidController(pidController) {}
//
//    // Function to turn the robot 90 degrees to the right
//    void turnRight() {
//        // Reset PID controller and IMU
//        pidController.zeroAndSetTarget(0, 90);  // Target 90 degrees
//        imu.resetYaw();
//
//        while (true) {
//            float currentYaw = imu.getYaw();
//            float error = 90 - currentYaw;  // Calculate error to reach 90 degrees
//
//            // Check if the target angle is reached
//            if (abs(error) < 1.0) {
//                // Stop motors if target angle is reached
//                leftMotor.setPWM(0);
//                rightMotor.setPWM(0);
//                break;
//            }
//
//            // Compute control signal using PID controller
//            float controlSignal = pidController.compute(currentYaw);
//
//            // Set motor speeds based on control signal (adjust direction as needed)
//            leftMotor.setPWM(-controlSignal);  // Reverse direction to turn right
//            rightMotor.setPWM(controlSignal);
//        }
//    }
//
//    // Function to turn the robot 90 degrees to the left
//    void turnLeft() {
//        // Reset PID controller and IMU
//        pidController.zeroAndSetTarget(0, -90);  // Target -90 degrees (left turn)
//        imu.resetYaw();
//
//        while (true) {
//            float currentYaw = imu.getYaw();
//            float error = -90 - currentYaw;  // Calculate error to reach -90 degrees (left turn)
//
//            // Check if the target angle is reached
//            if (abs(error) < 1.0) {
//                // Stop motors if target angle is reached
//                leftMotor.setPWM(0);
//                rightMotor.setPWM(0);
//                break;
//            }
//
//            // Compute control signal using PID controller
//            float controlSignal = pidController.compute(currentYaw);
//
//            // Set motor speeds based on control signal (adjust direction as needed)
//            leftMotor.setPWM(controlSignal);  // No need to reverse for left turn
//            rightMotor.setPWM(-controlSignal);
//        }
//    }
//
//private:
//    Motor& leftMotor;
//    Motor& rightMotor;
//    IMU& imu;
//    PIDController& pidController;
//};
//
//}  // namespace mtrn3100
