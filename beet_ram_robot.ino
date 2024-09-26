// Pins for motors and TB6612FNG driver
#define MOTOR_LEFT_FORWARD_PIN 9   // AIN1
#define MOTOR_LEFT_BACKWARD_PIN 8  // AIN2
#define MOTOR_RIGHT_FORWARD_PIN 12 // BIN1
#define MOTOR_RIGHT_BACKWARD_PIN 11 // BIN2

#define MOTOR_LEFT_ENABLE_PIN 6    // PWMA (Speed control for left motor)
#define MOTOR_RIGHT_ENABLE_PIN 5   // PWMB (Speed control for right motor)

#define STBY 10 // Standby pin for TB6612FNG

// Pins for sensors
#define TRIG_PIN 2 // Trigger pin for ultrasonic sensor
#define ECHO_PIN 3 // Echo pin for ultrasonic sensor
#define LEFT_IR_PIN A4 // Left IR sensor
#define RIGHT_IR_PIN A5 // Right IR sensor

// Constants for ultrasonic sensor
#define FORWARD_DISTANCE 150 // Maximum distance for object detection
#define ACCELERATION_THRESHOLD 75 // Distance for acceleration

// Threshold for IR sensors
#define IR_THRESHOLD 512

// Declaring variables
int leftIRValue;
int rightIRValue;
bool isLeftDetected = false;
bool isRightDetected = false;
int speed = 85;  // Initial speed

unsigned long obstacleStartTime = 0;
bool isObstacleClose = false;

void setup() {
  // Setup pins for motor control
  pinMode(MOTOR_LEFT_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_ENABLE_PIN, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Setup pins for sensors
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  // Setup pins for the ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Enable driver
  digitalWrite(STBY, HIGH);

  Serial.begin(115200);
}

void loop() {
  // Reading values from IR sensors
  leftIRValue = analogRead(LEFT_IR_PIN);
  rightIRValue = analogRead(RIGHT_IR_PIN);

  isLeftDetected = (leftIRValue < IR_THRESHOLD);
  isRightDetected = (rightIRValue < IR_THRESHOLD);

  Serial.print("Left IR Value: ");
  Serial.print(leftIRValue);
  Serial.print(" Right IR Value: ");
  Serial.println(rightIRValue);

  // Turning logic based on IR sensors (independent of the ultrasonic sensor)
  if (isLeftDetected && !isRightDetected) {
    Serial.println("Turning Left");
    turnLeft();
  } else if (!isLeftDetected && isRightDetected) {
    Serial.println("Turning Right");
    turnRight();
  } else {
    // If both sensors are inactive or active simultaneously, use the ultrasonic sensor
    long distance = measureDistance();
    Serial.print("Distance: ");
    Serial.println(distance);

    // If the object is within range (0-80 cm), move forward
    if (distance > 0 && distance <= FORWARD_DISTANCE) {
      if (distance <= ACCELERATION_THRESHOLD) {
        // Accelerate when the object is closer than 35 cm
        speed = map(distance, 0, ACCELERATION_THRESHOLD, 160, 85); // The closer the object, the higher the speed
      } else {
        speed = 85;  // Normal speed
      }
      moveForward(speed);  // Move forward at the current speed

      // New condition to check the distance
      if (distance <= 15) {
        if (!isObstacleClose) {
          obstacleStartTime = millis(); // Save the start time
          isObstacleClose = true;
        } else if (millis() - obstacleStartTime > 300) {
          // If the distance is less than 15 cm and more than 300 ms have passed
          moveBackward(150); // Move backward at speed 150
          delay(1000); // Wait 1 second
          stopMotors(); // Stop the motors
          isObstacleClose = false; // Reset state
        }
      } else {
        isObstacleClose = false; // If the distance is greater than 15 cm, reset the state
      }
    } else {
      stopMotors();  // If the object is not found, stop
    }
  }
  delay(50);  // Delay for data update
}

// Function for moving forward
void moveForward(int speed) {
  Serial.println("Moving Forward");
  moveLeftForward(speed);
  moveRightForward(speed);
}

// Function for moving backward
void moveBackward(int speed) {
  Serial.println("Moving Backward");
  moveLeftBackward(speed);
  moveRightBackward(speed);
}

// Function to stop the motors
void stopMotors() {
  Serial.println("Stop");
  analogWrite(MOTOR_LEFT_ENABLE_PIN, 0);
  analogWrite(MOTOR_RIGHT_ENABLE_PIN, 0);
  digitalWrite(MOTOR_LEFT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, LOW);
}

// Function to turn left
void turnLeft() {
  Serial.println("Turning Left");
  moveLeftForward(speed);
  moveRightBackward(speed);
}

// Function to turn right
void turnRight() {
  Serial.println("Turning Right");
  moveLeftBackward(speed);
  moveRightForward(speed);
}

// New motor control functions

// Move the left wheel forward
void moveLeftForward(int speed) {
  digitalWrite(MOTOR_LEFT_FORWARD_PIN, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD_PIN, LOW);
  analogWrite(MOTOR_LEFT_ENABLE_PIN, speed);
}

// Move the left wheel backward
void moveLeftBackward(int speed) {
  digitalWrite(MOTOR_LEFT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD_PIN, HIGH);
  analogWrite(MOTOR_LEFT_ENABLE_PIN, speed);
}

// Move the right wheel forward
void moveRightForward(int speed) {
  digitalWrite(MOTOR_RIGHT_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, HIGH);
  analogWrite(MOTOR_RIGHT_ENABLE_PIN, speed);
}

// Move the right wheel backward
void moveRightBackward(int speed) {
  digitalWrite(MOTOR_RIGHT_FORWARD_PIN, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD_PIN, LOW);
  analogWrite(MOTOR_RIGHT_ENABLE_PIN, speed);
}

// Function to measure distance using the ultrasonic sensor
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;  // Convert to cm
  return distance;
}
