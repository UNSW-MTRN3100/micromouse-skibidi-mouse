#pragma once

#include <math.h>

namespace mtrn3100 {

class Move {

public:
    Move(mtrn3100::Motor& motorL, mtrn3100::Motor& motorR, EncoderOdemetry& odemetry, PIDController& pidController, DualEncoder& dualEncoder)
    : motorL(motorL), motorR(motorR), odometry(odometry), pidController(pidController), dualEncoder(dualEncoder) {}

    void forward(float distance) {
            // Reset the odometry and PID controller
            odometry.update(0, 0);
            pidController.zeroAndSetTarget(0, distance);  // Set PID target to the distance
            float currentPosition = odometry.getY();

        while (true) {
            // Update the odometry with current encoder values
            odometry.update(dualEncoder.getLeftRotation(), dualEncoder.getRightRotation());

            // Compute the control signal using the PID controller
            float controlSignal = pidController.compute(currentPosition);

            // Set the motor speeds based on the control signal
            leftMotor.setPWM(controlSignal);
            rightMotor.setPWM(controlSignal);

            // Compute the error
            float error = odometry.getError();

            // Check if the target distance is reached
            if (abs(error) < 0.01) {
                // Stop the motors if the target distance is reached
                leftMotor.setPWM(0);
                rightMotor.setPWM(0);
                break;
            }

            currentPosition = odometry.getY();
        }        
    }


private:
    mtrn3100::Motor& motorL;
    mtrn3100::Motor& motorR;
    EncoderOdometry& odometry;
    PIDController& pidController;
    DualEncoder& dualEncoder;
}

}