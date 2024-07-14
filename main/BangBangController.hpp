#pragma once

#include <math.h>

namespace mtrn3100 {

class BangBangController {
public:
    BangBangController(float speed, float deadband) : speed(speed), deadband(deadband) {}

    float compute(float input) {
        error = setpoint - (input - zero_ref);

        if (fabs(error) > deadband) {
            if (error > 0) {
                output = speed;
            } else {
                output = -speed;
            }
        } else {
            output = 0;  
        }

        return output;
    }

    float getError() {
      return error;
    }

    void tune(float speed, float deadband) {
      speed = speed;
      deadband = deadband;
    }

    void zeroAndSetTarget(float zero, float target) {
        zero_ref = zero;
        setpoint = target;
    }

private:
    float speed, deadband;
    float error, output;
    float setpoint = 0;
    float zero_ref = 0;
};

}  // namespace mtrn3100