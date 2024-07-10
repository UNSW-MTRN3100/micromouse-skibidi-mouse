#pragma once
#include <math.h>
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "PID.hpp"
#include "DualEncoder"

namespace mtrn3100{

Class DriveForward {
public:
    DriveForward(Motor& motor1, Motor& motor2, EncoderOdometry& odometry, DualEncoder& encoder)
      : motor1(motor1), motor2(motor2), odometry(odometry), encoder(encoder) {}

    void drive(int distance_mm);

private:
    Motor& motor1, motor2;
    EncoderOdometry& odometry;
    DualEncoder& encoder;

};
}