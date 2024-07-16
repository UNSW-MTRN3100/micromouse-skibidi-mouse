#pragma once

#include <Arduino.h>
#include "math.h"

namespace mtrn3100 {

// The motor class is a simple interface designed to assist in motor control
// You may choose to implement additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t in2) : pwm_pin(pwm_pin), dir_pin(in2) {
        // Set both pins as output
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }

    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.
    void setPWM(int16_t pwm) {
      // Clamp the pwm value to be within 0 to 255
        uint8_t pwm_value = abs(pwm);
        if (pwm_value > 255) {
            pwm_value = 255;
        }

        // Set direction pin based on the sign of the pwm value
        if (pwm > 0) {
            digitalWrite(dir_pin, HIGH);  // Forward direction
        } else {
            digitalWrite(dir_pin, LOW);   // Reverse direction
        }

        // Output the clamped PWM value
        analogWrite(pwm_pin, pwm_value);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100
