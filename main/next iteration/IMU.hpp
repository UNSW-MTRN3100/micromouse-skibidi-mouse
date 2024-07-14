#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

namespace mtrn3100 {

class IMU {
public:
    IMU() : mpu() {}

    bool begin() {
        // Initialize the IMU
        if (!mpu.begin()) {
            Serial.println("Failed to find MPU6050 chip");
            return false;
        }
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        return true;
    }

    void update() {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        unsigned long current_time = millis();
        float dt = (current_time - prev_time) / 1000.0; // Convert to seconds
        prev_time = current_time;
        
        // Update yaw using gyroscope data
        yaw += g.gyro.z * dt; // g.gyro.z is the yaw rate in radians/second
    }

    float getYaw() const {
        return yaw;
    }

private:
    Adafruit_MPU6050 mpu;
    unsigned long prev_time = 0;
    float yaw = 0.0;
};
}