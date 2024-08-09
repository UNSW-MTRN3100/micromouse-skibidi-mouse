#include "libs.hpp"
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Sensor initialization
VL6180X sensor1;
VL6180X sensor2;
VL6180X sensor3;

int sensor1_pin = A0;
int sensor2_pin = A1;
int sensor3_pin = A2;

// MPU6050 initialization
unsigned long timer = 0;
float gyroZ = 0;
float yaw = 0;
MPU6050 mpu(Wire);

// Encoder and motor pin definitions
#define EN_1_A 2
#define EN_1_B 7
#define EN_2_A 3
#define EN_2_B 8

#define MOT_1_PWM 11
#define MOT_1_DIR 12
#define MOT_2_PWM 9
#define MOT_2_DIR 10

// Encoder and odometry setup
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(31.73/2, 104/2);
mtrn3100::IMUOdometry IMU_odometry;

// Motor and controller setup
mtrn3100::Motor motor1(MOT_1_PWM, MOT_1_DIR);
mtrn3100::Motor motor2(MOT_2_PWM, MOT_2_DIR);

mtrn3100::BangBangController controller(125, 4);
mtrn3100::BangBangController controllerR(125/3, 3);
mtrn3100::BangBangController controllerL(125/3, 3);

// Maze dimensions
const int rows = 5;
const int cols = 9;
int maze[rows][cols] = {0};
bool visited[rows][cols] = {false};

// OLED display setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Task variables
int startX = 0;
int startY = 0;
int endX = 4;
int endY = 8;

int posX = 0;
int posY = 0;

int totalCells = rows * cols;
int visitedCells = 0;

int dirX[] = {-1, 1, 0, 0};
int dirY[] = {0, 0, -1, 1};

int parentX[rows][cols];
int parentY[rows][cols];

// BFS Queue structure
struct Queue {
    int x[rows * cols];
    int y[rows * cols];
    int front, rear;

    Queue() {
        front = -1;
        rear = -1;
    }

    bool isEmpty() {
        return front == -1;
    }

    void enqueue(int x_val, int y_val) {
        if (rear == rows * cols - 1)
            return;
        if (isEmpty()) {
            front = 0;
            rear = 0;
        } else {
            rear++;
        }
        x[rear] = x_val;
        y[rear] = y_val;
    }

    void dequeue(int &x_val, int &y_val) {
        if (isEmpty())
            return;
        x_val = x[front];
        y_val = y[front];
        if (front == rear) {
            front = -1;
            rear = -1;
        } else {
            front++;
        }
    }
};

int flag = 0;

void setup() {
    serialSetup();
    Oled_setup();
    lidarSetup();
    mpuSetup();
    controllerSetup();
    Serial.println("Setup function end");
}

void loop() {
    delay(50);

    if (flag == 0) {
        Task3_4();
    }
}

void controllerSetup() {
    controller.zeroAndSetTarget(0, 250);
    controllerL.zeroAndSetTarget(0, 90);
    controllerR.zeroAndSetTarget(0, -90);
}

void serialSetup() {
    Serial.begin(115200);
}

void lidarSetup() {
    Wire.begin();
    pinMode(sensor1_pin, OUTPUT);
    pinMode(sensor2_pin, OUTPUT);
    pinMode(sensor3_pin, OUTPUT);
    digitalWrite(sensor1_pin, LOW);
    digitalWrite(sensor2_pin, LOW);
    digitalWrite(sensor3_pin, LOW);

    digitalWrite(sensor1_pin, HIGH);
    delay(50);
    sensor1.init();
    sensor1.configureDefault();
    sensor1.setTimeout(250);
    sensor1.setAddress(0x54);
    delay(50);

    digitalWrite(sensor2_pin, HIGH);
    delay(50);
    sensor2.init();
    sensor2.configureDefault();
    sensor2.setTimeout(250);
    sensor2.setAddress(0x62);
    delay(50);

    digitalWrite(sensor3_pin, HIGH);
    delay(50);
    sensor3.init();
    sensor3.configureDefault();
    sensor3.setTimeout(250);
    sensor3.setAddress(0x58);
    delay(50);
}

void mpuSetup() {
    Wire.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) { }
  
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(500);
    mpu.calcOffsets(true, true);
    Serial.println("Done!\n");
}

void Task3_4() {
    // Step 1: Explore the maze and build the map
    exploreMaze();

    // Step 2: Return to start position
    bfs(posX, posY, startX, startY);
    followPath(startX, startY);

    // Step 3: Run BFS to find the shortest path from start to goal
    bfs(startX, startY, endX, endY);
    followPath(endX, endY);

    flag = 1; // Mark task as complete
}

void exploreMaze() {
    posX = startX;
    posY = startY;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // Mark the cell as visited
            if (!visited[posX][posY]) {
                visited[posX][posY] = true;
                visitedCells++;
            }

            // Display the mapping progress
            float completionPercentage = (visitedCells / (float)totalCells) * 100;
            String mapProgress = "Map: " + String(completionPercentage) + "%";
            writeTextToOLED(mapProgress, 0, 0);

            // Measure distances
            int frontDistance = sensor2.readRangeSingleMillimeters();
            int rightDistance = sensor3.readRangeSingleMillimeters();
            int leftDistance = sensor1.readRangeSingleMillimeters();

            // Update the maze map
            if (frontDistance < 150) { maze[posX][posY] = 1; }
            if (rightDistance < 150) { if (posY < cols - 1) maze[posX][posY + 1] = 1; }
            if (leftDistance < 150) { if (posY > 0) maze[posX][posY - 1] = 1; }

            // Move forward and update position
            moveForward();
            delay(500);
            stopMotors();

            if (i % 2 == 0) { posY++; } else { posY--; }
        }

        // Handle obstacle at the end of the row
        if (sensor2.readRangeSingleMillimeters() < 150) {
            turnRight();
            turnRight();
            moveForward();
            delay(500);
            stopMotors();
        }

        // Change direction
        if (i % 2 == 0) {
            turnRight();
            posX++;
            turnRight();
        } else {
            turnLeft();
            posX++;
            turnLeft();
        }
    }

    Serial.println("Maze exploration completed!");
}

void returnToStart(int currentX, int currentY) {
    bfs(currentX, currentY, startX, startY);
    followPath(startX, startY);
    Serial.println("Returned to start position!");
}

void followPath(int targetX, int targetY) {
    int pathX = targetX;
    int pathY = targetY;

    while (pathX != startX || pathY != startY) {
        int prevX = parentX[pathX][pathY];
        int prevY = parentY[pathX][pathY];

        if (prevX == pathX - 1) {
            moveForward();
            delay(500);
        } else if (prevY == pathY - 1) {
            turnLeft();
            moveForward();
            delay(500);
        } else if (prevY == pathY + 1) {
            turnRight();
            moveForward();
            delay(500);
        }

        stopMotors();
        pathX = prevX;
        pathY = prevY;
    }

    Serial.println("Path to goal completed!");
}

void moveForward() {
    driveStraight();
}


void stopMotors() {
    driveStop();
}

void turnRight() {
    turnRight(-88, 3);
    driveStop();  
    encoder_odometry.reset();
}

void turnLeft() {
    turnLeft(88, 3);
    driveStop();  
    encoder_odometry.reset();
}

void bfs(int startX, int startY, int goalX, int goalY) {
    int dist[rows][cols];
    memset(dist, -1, sizeof(dist));
    memset(parentX, -1, sizeof(parentX));
    memset(parentY, -1, sizeof(parentY));

    Queue queue;
    queue.enqueue(startX, startY);
    dist[startX][startY] = 0;

    while (!queue.isEmpty()) {
        int x, y;
        queue.dequeue(x, y);

        if (x == goalX && y == goalY) break;

        for (int i = 0; i < 4; i++) {
            int newX = x + dirX[i];
            int newY = y + dirY[i];

            if (newX >= 0 && newX < rows && newY >= 0 && newY < cols && maze[newX][newY] == 0 && dist[newX][newY] == -1) {
                queue.enqueue(newX, newY);
                dist[newX][newY] = dist[x][y] + 1;
                parentX[newX][newY] = x;
                parentY[newX][newY] = y;
            }
        }
    }
}

void Oled_setup() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 ekran başlatılamadı!"));
        for (;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
}

void writeTextToOLED(const String& text, int x, int y) {
    display.setCursor(x, y);
    display.print(text);
    display.display();
}

void driveStraight() {
    Serial.println("Driving Straight");

    float startingYaw = getYawMPU();
    controller.zeroAndSetTarget(encoder_odometry.getX(), 250);
    controller.compute(encoder_odometry.getX());

    while (fabs(controller.getError()) > 5) {
        encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
        displayEncoderOdom(encoder_odometry.getX(), encoder_odometry.getH());

        int controlSignal = controller.compute(encoder_odometry.getX());

        float leftWall = sensor1.readRangeSingleMillimeters();
        float rightWall = sensor3.readRangeSingleMillimeters();
        float frontWall = sensor2.readRangeSingleMillimeters();

        if (frontWall < 85) break;
        if (leftWall < 79) {
            motor1.setPWM(-controlSignal);
            motor2.setPWM(controlSignal - 10);
            continue;
        } 
        if (rightWall < 79) {
            motor1.setPWM(-controlSignal + 10);
            motor2.setPWM(controlSignal);
            continue;
        }

        straight(controlSignal);
    }
}

void displayEncoderOdom(float x, float h) {
    Serial.print("ODOM:\t\t x: ");
    Serial.print(x);
    Serial.print(",\t\t h: ");
    Serial.print(h);
    Serial.println();
}

float getYawMPU() {
    mpu.update();
    gyroZ = mpu.getGyroZ();
    float dt = (millis() - timer) / 1000.0;
    yaw += kalmanFilter(gyroZ) * dt;
    timer = millis();
    Serial.print("Yaw: ");
    Serial.println(yaw);
    return yaw;
}

float kalmanFilter(float U) {
    static const double R = 500;
    static const double H = 1.00;
    static double Q = 10;
    static double P = 0;
    static double U_hat = 0;
    static double K = 0;

    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);
    P = (1 - K * H) * P + Q;
    return U_hat;
}

void straight(int pwm) {
    motor1.setPWM(-pwm);
    motor2.setPWM(pwm);
}

void turnLeft(float degree, float error) {
    controllerL.zeroAndSetTarget(getYawMPU(), degree);
    controllerL.compute(getYawMPU());

    while (fabs(controllerL.getError()) > error) {
        float currYaw = getYawMPU();
        controllerL.compute(currYaw);
        int controlSignal = controllerL.compute(currYaw);
        motor1.setPWM(controlSignal);
        motor2.setPWM(controlSignal);
    }
    encoder.reset();
}

void turnRight(float degree, float error) {
    controllerR.zeroAndSetTarget(getYawMPU(), degree);
    controllerR.compute(getYawMPU());

    while (fabs(controllerR.getError()) > error) {
        float currYaw = getYawMPU();
        int controlSignal = controllerR.compute(currYaw);
        motor1.setPWM(controlSignal);
        motor2.setPWM(controlSignal);
    }
    encoder.reset();
}

void driveStop() {
    motor1.setPWM(0);
    motor2.setPWM(0);
}
