#include "RobotDriver.hpp"

RobotDriver::RobotDriver(int leftMotorPin, int rightMotorPin, int leftSensorPin, int rightSensorPin, int frontSensorPin)
    : leftMotorPin(leftMotorPin), rightMotorPin(rightMotorPin), leftSensorPin(leftSensorPin), rightSensorPin(rightSensorPin), frontSensorPin(frontSensorPin) {
    pinMode(leftMotorPin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);
    pinMode(leftSensorPin, INPUT);
    pinMode(rightSensorPin, INPUT);
    pinMode(frontSensorPin, INPUT);
}

void RobotDriver::driveStraight() {
    for (int cell = 0; cell < 5; ++cell) {
        while (true) {
            if (checkCollision()) {
                stop();
                return;
            }
            if (checkEndOfPath()) {
                stop();
                break;
            }
            driveForward();
            correctCourse();
        }
    }
}

bool RobotDriver::checkCollision() {
    int frontDistance = analogRead(frontSensorPin);
    return frontDistance <= maxDistance;
}

bool RobotDriver::checkEndOfPath() {
    int frontDistance = analogRead(frontSensorPin);
    return frontDistance <= targetDistance;
}

void RobotDriver::stop() {
    digitalWrite(leftMotorPin, LOW);
    digitalWrite(rightMotorPin, LOW);
}

void RobotDriver::driveForward() {
    digitalWrite(leftMotorPin, HIGH);
    digitalWrite(rightMotorPin, HIGH);
}

void RobotDriver::correctCourse() {
    int leftDistance = analogRead(leftSensorPin);
    int rightDistance = analogRead(rightSensorPin);

    if (leftDistance < rightDistance) {
        // Correct to the right
        digitalWrite(leftMotorPin, HIGH);
        digitalWrite(rightMotorPin, LOW);
    } else if (rightDistance < leftDistance) {
        // Correct to the left
        digitalWrite(leftMotorPin, LOW);
        digitalWrite(rightMotorPin, HIGH);
    } else {
        // Move forward
        digitalWrite(leftMotorPin, HIGH);
        digitalWrite(rightMotorPin, HIGH);
    }
}

void RobotDriver::turnInPlace(int degrees) {
    turn(degrees);
}

void RobotDriver::turn(int degrees) {
    // Implement the turning logic
    // Assuming we have a function to measure the angle turned
    int targetAngle = degrees; // Target angle to turn

    // Assuming we have a function to read the current angle from a gyroscope or encoder
    int currentAngle = 0; // Read the current angle

    // Simple proportional control loop to turn to the target angle
    while (abs(currentAngle - targetAngle) > 5) {
        if (currentAngle < targetAngle) {
            digitalWrite(leftMotorPin, HIGH);
            digitalWrite(rightMotorPin, LOW);
        } else {
            digitalWrite(leftMotorPin, LOW);
            digitalWrite(rightMotorPin, HIGH);
        }
        currentAngle = 0; // Update with actual sensor reading
    }

    stop();
}

void RobotDriver::moveOneCell() {
    // Implement the logic to move one cell forward
    while (!checkEndOfPath()) {
        driveForward();
        correctCourse();
        if (checkCollision()) {
            stop();
            return;
        }
    }
    stop();
}

void RobotDriver::executeCommands(const String& commands) {
    for (char command : commands) {
        switch (command) {
            case 'f':
                moveOneCell();
                break;
            case 'l':
                turnInPlace(-90); // Turn left 90 degrees
                break;
            case 'r':
                turnInPlace(90); // Turn right 90 degrees
                break;
        }
        delay(1000); // Small delay between commands
    }
}
