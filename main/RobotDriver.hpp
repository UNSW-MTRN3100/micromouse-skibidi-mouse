// Abishek
#ifndef ROBOTDRIVER_HPP
#define ROBOTDRIVER_HPP

#include <Arduino.h>

class RobotDriver {
public:
    RobotDriver(int leftMotorPin, int rightMotorPin, int leftSensorPin, int rightSensorPin, int frontSensorPin);
    void driveStraight();

    void turnInPlace(int degrees);
    void executeCommands(const String& commands);
    bool checkCollision();
    bool checkEndOfPath();
    void stop();

private:
    int leftMotorPin;
    int rightMotorPin;
    int leftSensorPin;
    int rightSensorPin;
    int frontSensorPin;
    const int targetDistance = 25; // mm to center of the cell
    const int maxDistance = 50;    // distance to consider as collision

    void driveForward();
    void correctCourse();
    void turn(int degrees);
    void moveOneCell();
};

#endif // ROBOTDRIVER_HPP