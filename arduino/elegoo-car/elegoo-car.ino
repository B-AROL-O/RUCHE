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

#include <string.h>

#include <Servo.h>
#include <SoftwareSerial.h>

#define DEBUG 0

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

#define MIN_SPEED_TRESH 127

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

void setLeftMotorSpeed(uint8_t speed) {
  analogWrite(ENA, speed);
}

void setRightMotorSpeed(uint8_t speed) {
  analogWrite(ENB, speed);
}

void setMotorSpeed(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void leftMotorForward() {
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);
}

void leftMotorBackward() {
  digitalWrite(A1, LOW); digitalWrite(A2, HIGH);
}

void rightMotorForward() {
  digitalWrite(B1, LOW);  digitalWrite(B2, HIGH);
}

void rightMotorBackward() {
  digitalWrite(B1, HIGH); digitalWrite(B2, LOW);
}

void motorForward() {
  digitalWrite(A1, HIGH); digitalWrite(A2, LOW);
  digitalWrite(B1, LOW);  digitalWrite(B2, HIGH);
}

void motorBackward() {
  digitalWrite(A1, LOW); digitalWrite(A2, HIGH);
  digitalWrite(B1, HIGH); digitalWrite(B2, LOW);
}

void leftMotorStop() {
  digitalWrite(A1, HIGH); digitalWrite(A2, HIGH);
}

void rightMotorStop() {
  digitalWrite(B1, HIGH); digitalWrite(B2, HIGH);
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

void sweepScan() {
  // NOTE: can be inconsistent when distance is over 90-100cm 
  servoMotor.write(0);
  delay(100); // just in case it was on the complete opposite direction
  for (int angle = 0; angle <= 180; angle += 30) {
    servoMotor.write(angle);
    delay(100);    
    int d = getDistanceCm();
    Serial.print("{angle:");
    Serial.print(angle);
    Serial.print(",dist:");
    Serial.print(d);
    Serial.println("}");
  }
  servoMotor.write(90);
  delay(2);
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

uint8_t mapSpeedToByte(float s) {
  // remap [-10, +10] to [128, 255] (+/- is just direction of rotation),
  // which avoids dead zone [0, 127] where wheels barely move.

  if (s < 0) {
    s = -s;
  }
  // remap [0, 10] to [128, 255]
  // int range = 255 - 128; // = 127
  // float factor = (float)range / 10.0; // = 12.7
  // uint8_t mapped = 12.7 * s + 128;

  // let's use [100, 255] interval
  uint8_t mapped = 15.5*s + 100;
  return mapped;
}

/**
 * Parses command like:
 * vl:<float>;vr:<float>
 * where vr and vl can also be swapped.
 * Both vr and vl labels (and values) are mandatory.
 * Floats can be negative or positive and they can be expressed
 * just as any float in C.
 */
bool parseSpeedCommand(char *cmd, float *vl, float *vr) {  
  char *delim = strchr(cmd, ';');
  if (delim == NULL) {
    return false;
  }

  *delim = '\0';

  uint8_t cnt = 0;
  bool vlSet = false;
  bool vrSet = false;
  
  while (cnt < 2) {
    cnt++;
    
    char *colPtr = strchr(cmd, ':');
    if (colPtr == NULL) {
      Serial.println("No colon found!");
      return false;
    }
    
    *colPtr = '\0';
    
    if (strcmp(cmd, "vl") == 0) {
      if (vlSet == true) {
        return false;
      }
      *vl = atof(colPtr+1);
      vlSet = true;
    } else if (strcmp(cmd, "vr") == 0) {
      if (vrSet == true) {
        return false;
      }
      *vr = atof(colPtr+1);
      vrSet = true;
    } else {
      Serial.print("Invalid label: ");
      Serial.println(cmd);
      return false;
    }
    
    cmd = delim+1;
  }
  
  return vlSet && vrSet;
}

// -------------------- Command Handling --------------------
void handleCommand(String cmd) {
  #if DEBUG
  Serial.print("Received String: '");
  Serial.print(cmd);
  Serial.println("'");
  #endif

  char c = cmd[0];
  switch (c) {
    case 'v':
      float vl = 0.0;
      float vr = 0.0;

      bool ok = parseSpeedCommand(cmd.begin(), &vl, &vr);

      #if DEBUG
      Serial.print("Parse result: ");
      Serial.println(ok);
      Serial.print("vl: ");
      Serial.println(vl);
      Serial.print("vr: ");
      Serial.println(vr);
      #endif

      if (!ok) {
        Serial.println("INV_SPEED_CMD");
        break;
      }

      if (vl < -10 || vl > 10 || vr < -10 || vr > 10) {
        Serial.println("OUT_OF_RANGE");
        break;
      }

      if (vl > -0.01 && vl < 0.01) {
        // stop motor
        Serial.println("Stop left");
        leftMotorStop();
      } else {
        uint8_t b_vl = mapSpeedToByte(vl);
        Serial.print("b_vl: ");
        Serial.println(b_vl);
        setLeftMotorSpeed(b_vl);
        if (vl < 0) {
          Serial.println("negative left");
          leftMotorBackward();
        } else {
          Serial.println("positive left");
          leftMotorForward();
        }
      }

      if (vr > -0.01 && vr < 0.01) {
        // stop motor
        Serial.println("Stop right");
        rightMotorStop();
      } else {
        uint8_t b_vr = mapSpeedToByte(vr);
        Serial.print("b_vr: ");
        Serial.println(b_vr);
        setRightMotorSpeed(b_vr);
        if (vr < 0) {
          Serial.println("negative right");
          rightMotorBackward();
        } else {
          Serial.println("positive right");
          rightMotorForward();
        }
      }
      break;

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

    case 'S':
      direction = DIR_STOP;
      motorStop();
      sweepScan();
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
  // if (dist != lastDistance) {
  //   lastDistance = dist;
  //   // Serial.print("f ");
  //   // Serial.println(lastDistance);
  // }

  // Stop if obstacle too close
  if (direction == DIR_FWD && lastDistance <= OBSTACLE_CM) {
    motorStop();
    direction = DIR_STOP;
    startBlinking();
  }

  // Read incoming command
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // Timeout safety: stop motors if no command received
  if (millis() - lastCmdTime > COMMAND_TIMEOUT_MS) {
    motorStop();
    direction = DIR_STOP;
  }

  updateLedBlink();
}
