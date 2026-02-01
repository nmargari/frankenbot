#include <Servo.h>

// ---------------- PIN DEFINITIONS ----------------
const uint8_t SERVO_PIN   = 3;
const uint8_t TRIGGER_PIN = 12;
const uint8_t ECHO_PIN    = 11;

const uint8_t LEFT_ENABLE  = 10;   // PWM
const uint8_t RIGHT_ENABLE = 9;    // PWM

const uint8_t LEFT_MOTOR_PIN1  = 7;
const uint8_t LEFT_MOTOR_PIN2  = 6;
const uint8_t RIGHT_MOTOR_PIN1 = 5;
const uint8_t RIGHT_MOTOR_PIN2 = 4;

// ---------------- TUNING ----------------
const uint16_t DISTANCE_LIMIT_CM = 30;
const uint16_t TURN_TIME_MS      = 700;   
const uint16_t NO_ECHO_CM        = 999;

const uint16_t PING_TIMEOUT_US   = 30000; // Timeout for pulseIn
const uint8_t  PING_SAMPLES      = 5;     // Median of 5
const uint16_t PING_GAP_MS       = 40;    // Gap between pings

const uint16_t SERVO_SETTLE_MS   = 500;   // Time for servo to settle

// Speeds (0..255). Tune these to reduce overshoot / slipping.
const uint8_t SPEED_FWD  = 210;
const uint8_t SPEED_TURN = 180;

// ---------------- GLOBALS ----------------
Servo servo_motor;

// ---------------- MOTOR CONTROL ----------------
void setSpeed(uint8_t left, uint8_t right) 
{
  analogWrite(LEFT_ENABLE, left);
  analogWrite(RIGHT_ENABLE, right);
}

void driveLeft(int8_t dir) 
{
  // Direction: +1 forward, -1 backward, 0 stop
  if (dir > 0) 
  {
    digitalWrite(LEFT_MOTOR_PIN1, HIGH);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
  } 
  else if (dir < 0) 
  {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
  } 
  else 
  {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
  }
}

void driveRight(int8_t dir) 
{
  if (dir > 0) 
  {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
  } 
  else if (dir < 0) 
  {
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  } 
  else 
  {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  }
}

void moveForward() 
{
  driveLeft(+1);
  driveRight(+1);
}

void stopCar() 
{
  driveLeft(0);
  driveRight(0);
}

void turnLeftSpin() 
{
  // Spin-in-place left: left backward, right forward
  driveLeft(-1);
  driveRight(+1);
}

void turnRightSpin() 
{
  // Spin-in-place right: left forward, right backward
  driveLeft(+1);
  driveRight(-1);
}

// ---------------- ULTRASONIC ----------------
uint16_t pingOnceCm() 
{
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, PING_TIMEOUT_US);
  if (duration == 0) 
  {
    return NO_ECHO_CM;
  }

  return (uint16_t)((duration * 343UL) / 20000UL);
}

uint16_t getDistanceCm() 
{
  uint16_t v[PING_SAMPLES];

  // Take PING_SAMPLES times ping values
  for (uint8_t i = 0; i < PING_SAMPLES; i++) 
  {
    v[i] = pingOnceCm();
    delay(PING_GAP_MS);
  }

  // Sort the array with ping values 
  for (uint8_t i = 0; i < PING_SAMPLES - 1; i++) 
  {
    for (uint8_t j = i + 1; j < PING_SAMPLES; j++) 
    {
      if (v[j] < v[i]) 
      {
        uint16_t t = v[i]; 
        v[i] = v[j]; 
        v[j] = t;
      }
    }
  }

  // Return the median value in the center of the array
  return v[PING_SAMPLES / 2]; 
}

// ---------------- SERVO LOOK ----------------
void lookTo(uint8_t angle) 
{
  // Do not let angles over 180 degrees
  if (angle > 180) 
  {
    angle = 180;
  }

  servo_motor.write(angle);
  delay(SERVO_SETTLE_MS);
}

// ---------------- SETUP / LOOP ----------------
void setup() 
{
  // Start serial monitor for debugging purposes
  Serial.begin(9600);

  // Set the enables for the L293D ICs
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);

  // Set the motor pins
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

  // Set the ultrasonic
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set the servo motor
  servo_motor.attach(SERVO_PIN);
  lookTo(90);

  // Start with controlled speed
  setSpeed(SPEED_FWD, SPEED_FWD);

  // Let everything finish with a small delay
  delay(300);
}

void loop() 
{
  uint16_t front = getDistanceCm();

  Serial.print("Front cm: ");
  Serial.println(front);

  // Check distance to the front if smaller than the limit
  if (front != NO_ECHO_CM && front < DISTANCE_LIMIT_CM) 
  {
    // Stop the car
    stopCar();
    delay(200);

    // Start looking right
    Serial.println("Looking RIGHT...");
    lookTo(0);

    // Measure the distance on the right in cm
    uint16_t rightDist = getDistanceCm();

    Serial.print("Right cm: ");
    Serial.println(rightDist);

    // Start looking left
    Serial.println("Looking LEFT...");
    lookTo(180);

    // Measure the distance on the left in cm
    uint16_t leftDist = getDistanceCm();

    Serial.print("Left cm: ");
    Serial.println(leftDist);

    // Look forward
    Serial.println("Center...");
    lookTo(90);

    // Set the turn speed. It is set lower to be more precise
    setSpeed(SPEED_TURN, SPEED_TURN);

    // Check where there is more space to travel
    if (rightDist > leftDist) 
    {
      Serial.println("Turning RIGHT (spin)...");
      turnRightSpin();
    } 
    else 
    {
      Serial.println("Turning LEFT (spin)...");
      turnLeftSpin();
    }

    delay(TURN_TIME_MS);

    // Stop the car
    stopCar();
    delay(100);

    // Set the speed to moving
    setSpeed(SPEED_FWD, SPEED_FWD);

    // Move forward to the new direction
    moveForward();
  } 
  else 
  {
    // Set the speed to moving
    setSpeed(SPEED_FWD, SPEED_FWD);

    // Move forward to the same direction
    moveForward();
  }

  // A small delay before every loop to finish everything
  delay(50);
}
