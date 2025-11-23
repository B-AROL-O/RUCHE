/**
* Bluetooth driven robot car
* The commands can be sent through BLE communication on
* characteristic with UUID "0000ffe1-0000-1000-8000-00805f9b34fb".
* An example of python script to control it is provided in the current directory.
*
* Commands available in this example:
* - 'f': go forward
* - 'b': go backwards
* - 'l': turn left
* - 'r': turn right
* 
* Every command is just one byte ascii character.
* When 'f' or 'b' are sent, the robot starts moving in that direction until:
* - an obstacle is encountered (only if going forwards)
* - a timeout interval (default 3 seconds - defined as COMMAND_TIMEOUT_MS) is passed
*
* Obstacle detection is done with ultrasonic sensor.
* When obstacle is too near (distance is OBSTACLE_CM variable) the car stops and
* won't go forwards anymore, even if the command is sent again.
* Also the led will start blinking to warn you of this.
* Note that no detection can be done on the back of the robot, so when reversing there
* won't be any obstacle detection and the robot may go into an obstacle.
*
* The car speed is controlled through PWM, doing an ANALOG (and not digital) write on
* the pins that enable the motors (ENA and ENB).
* To go full speed, just a digital write with HIGH is enough.
* 
*/

#include <Servo.h>

// -------------------- Pin Definitions --------------------
#define LED_PIN 13

// Motor pins (L298N)
#define ENA 5
#define A1 7
#define A2 8
#define ENB 6
#define B1 9
#define B2 11

// Servo and ultrasonic
#define SERVO_PIN 3
#define SONAR_TRIG A5
#define SONAR_ECHO A4

// -------------------- Constants --------------------
#define DIR_STOP 0
#define DIR_FWD  1
#define DIR_BWD  2

#define ANGLE_CENTER 90
#define ANGLE_LEFT   180
#define ANGLE_RIGHT  0

#define TURN_DELAY_MS 200
#define OBSTACLE_CM  15
#define COMMAND_TIMEOUT_MS 3000

#define BLINK_DURATION_MS 1000
#define BLINK_INTERVAL_MS 200

#define MIN_SPEED 120
#define MAX_SPEED 255

#define CMD_MOTOR_BOTH = 0
#define CMD_MOTOR_A = 1
#define CMD_MOTOR_B = 2

// -------------------- Globals --------------------
Servo servoMotor;

uint8_t motorSpeed = 150;
uint8_t direction = DIR_STOP;

unsigned long lastCmdTime = 0;
char cmd = 0;
int lastDistance = 999; // Last measured forward distance

bool ledBlinking = false;
unsigned long blinkStartTime = 0;

// -------------------- Helper Functions --------------------
void initMotors() {
  pinMode(ENA, OUTPUT); pinMode(A1, OUTPUT); pinMode(A2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(B1, OUTPUT); pinMode(B2, OUTPUT);
}

/**
* Speed of the car is between 0 and 255, but under 120 the car
* doesn't actually move, so the input speed coming from the
* commands (0-255) are re-mapped from 120 to 255.
*/
uint8_t mapMotorSpeed(uint8_t inputSpeed) {
  return MIN_SPEED + inputSpeed / (MAX_SPEED-MIN_SPEED);
}

void setMotorSpeed(uint8_t motor, uint8_t speed) {
  if (motor == CMD_MOTOR_BOTH) {
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
  } else if (motor == CMD_MOTOR_A) {
    analogWrite(ENA, speed);
  } else if (motor == CMD_MOTOR_B) {
    analogWrite(ENB, speed);
  }
}

void singleMotorForward(uint8_t motor) {
  if (motor == CMD_MOTOR_BOTH) {
    digitalWrite(A1, HIGH);
    digitalWrite(A2, LOW);
    digitalWrite(B1, LOW);
    digitalWrite(B2, HIGH);
  } else if (motor == CMD_MOTOR_A) {
    digitalWrite(A1, HIGH);
    digitalWrite(A2, LOW);
  } else if (motor == CMD_MOTOR_B) {
    digitalWrite(B1, LOW);
    digitalWrite(B2, HIGH);
  }
}

void singleMotorBackward(uint8_t motor) {
  if (motor == CMD_MOTOR_BOTH) {
    digitalWrite(A1, LOW);
    digitalWrite(A2, HIGH);
    digitalWrite(B1, HIGH);
    digitalWrite(B2, LOW);
  } else if (motor == CMD_MOTOR_A) {
    digitalWrite(A1, LOW);
    digitalWrite(A2, HIGH);
  } else if (motor == CMD_MOTOR_B) {
    digitalWrite(B1, HIGH);
    digitalWrite(B2, LOW);
  }
}

void singleMotorStop(uint8_t motor) {
  if (motor == CMD_MOTOR_BOTH) {
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(B1, HIGH);
    digitalWrite(B2, HIGH);
  } else if (motor == CMD_MOTOR_A) {
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
  } else if (motor == CMD_MOTOR_B) {
    digitalWrite(B1, HIGH);
    digitalWrite(B2, HIGH);
  }
}

void motorForward() {
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);
  digitalWrite(B1, LOW);  digitalWrite(B2, HIGH);
}

void motorBackward() {
  digitalWrite(A1, LOW); digitalWrite(A2, HIGH);
  digitalWrite(B1, HIGH); digitalWrite(B2, LOW);
}

void motorStop() {
  digitalWrite(A1, HIGH); digitalWrite(A2, HIGH);
  digitalWrite(B1, HIGH); digitalWrite(B2, HIGH);
}

void turnLeft() {
  digitalWrite(A1, LOW); digitalWrite(A2, HIGH);
  digitalWrite(B1, LOW);  digitalWrite(B2, HIGH);
}

void turnRight() {
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);
  digitalWrite(B1, HIGH); digitalWrite(B2, LOW);
}

int getDistanceCm() {
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);
  unsigned long duration = pulseIn(SONAR_ECHO, HIGH, 25000UL);
  if (duration == 0) return 999;
  return duration / 58;
}

void startBlinking() {
  ledBlinking = true;
  blinkStartTime = millis();
}

void updateLedBlink() {
    static bool state = false;
    static unsigned long prevToggle = 0;
    unsigned long now = millis();

    if (!ledBlinking) return;

    if (now - blinkStartTime > BLINK_DURATION_MS) { // total duration
        ledBlinking = false;
        digitalWrite(LED_PIN, LOW);
        return;
    }

    if (now - prevToggle >= BLINK_INTERVAL_MS) { // interval
        state = !state;
        digitalWrite(LED_PIN, state);
        prevToggle = now;
    }
}

// -------------------- Command Handling --------------------
void handleCommand(char c) {
  switch (c) {
    case 'f':
      direction = DIR_FWD;
      if (lastDistance <= OBSTACLE_CM) {
        motorStop();
        startBlinking();
        direction = DIR_STOP;
      } else {
        motorForward();
      }
      break;

    case 'b':
      direction = DIR_BWD;
      motorBackward();
      break;

    case 's':
      direction = DIR_STOP;
      motorStop();
      break;

    case 'l':
      turnLeft();
      delay(TURN_DELAY_MS);
      if (direction == DIR_FWD) motorForward();
      else if (direction == DIR_BWD) motorBackward();
      else motorStop();
      break;

    case 'r':
      turnRight();
      delay(TURN_DELAY_MS);
      if (direction == DIR_FWD) motorForward();
      else if (direction == DIR_BWD) motorBackward();
      else motorStop();
      break;

    default:
      break;
  }
  lastCmdTime = millis();
}

// -------------------- Setup --------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  initMotors();
  setMotorSpeed(motorSpeed);

  servoMotor.attach(SERVO_PIN);
  servoMotor.write(ANGLE_CENTER);

  pinMode(SONAR_ECHO, INPUT);
  pinMode(SONAR_TRIG, OUTPUT);

  Serial.begin(9600);
}

// -------------------- Loop --------------------
void loop() {
  // Read distance continuously
  int dist = getDistanceCm(); 
  if (dist != lastDistance) {
    lastDistance = dist;
    Serial.print("f ");
    Serial.println(lastDistance);
  }

  // Stop if obstacle too close
  if (direction == DIR_FWD && lastDistance <= OBSTACLE_CM) {
    motorStop();
    direction = DIR_STOP;
    startBlinking();
  }

  // Read incoming command
  if (Serial.available()) {
    cmd = Serial.read();
    handleCommand(cmd);
  }

  // Timeout safety: stop motors if no command received
  if (millis() - lastCmdTime > COMMAND_TIMEOUT_MS) {
    motorStop();
    direction = DIR_STOP;
  }

  updateLedBlink();
}

