#include <Servo.h>
#include <QuadratureEncoder.h>
#include <PID_v1.h>

// Constants
#define MOTOR_PIN_1 3
#define MOTOR_PIN_2 5

#define GEAR_RATIO 4
#define ENCODER_DISCRETE_POSITIONS 8192
#define ONE_TURN (ENCODER_DISCRETE_POSITIONS * GEAR_RATIO)
#define ANGLE_MULTIPLIER (ONE_TURN / 360)

#define KP1 4.0
#define KI1 2.0
#define KD1 1.0

#define KP2 5.0
#define KI2 3.0
#define KD2 1.0

#define INITIAL_SETPOINT 90.0
#define FORWARD_SETPOINT 135.0
#define BACKWARD_SETPOINT 45.0

#define SETTLE_ANGLE_THRESHOLD 5.0
#define SETTLE_DELAY 10
#define SETTLE_COUNT_MAX 5

// MOTOR
Servo motor1;
Servo motor2;

// ENCODER
Encoders encoder1(11, 12);
Encoders encoder2(8, 9);

// PID 1
double input1 = 0, output1 = 0, setpoint1 = INITIAL_SETPOINT;
PID myPID1(&input1, &output1, &setpoint1, KP1, KI1, KD1, DIRECT);

// PID 2
double input2 = 0, output2 = 0, setpoint2 = INITIAL_SETPOINT;
PID myPID2(&input2, &output2, &setpoint2, KP2, KI2, KD2, DIRECT);

// Flag to ensure startUp() runs only once
bool startUpComplete = false;

void setup() {
  motor1.attach(MOTOR_PIN_1);
  motor2.attach(MOTOR_PIN_2);

  myPID1.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(100);
  myPID1.SetOutputLimits(1000, 2000);

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetSampleTime(100);
  myPID2.SetOutputLimits(1000, 2000);

  delay(5000);
}

void loop() {
  if (!startUpComplete) {
    stabilizeAtStartPosition();
    startUp();
    startUpComplete = true;
  } else {
    walk();
  }
}

void stabilizeAtStartPosition() {
  setpoint1 = INITIAL_SETPOINT;
  setpoint2 = INITIAL_SETPOINT;

  // Stabilize outer leg
  waitForSettle(input1, setpoint1, motor1, myPID1, encoder1);

  // Stabilize inner leg
  waitForSettle(input2, setpoint2, motor2, myPID2, encoder2);

  delay(1000); // Allow both legs to fully stabilize before starting
}

static void startUp() {
  if (setpoint1 >= INITIAL_SETPOINT && setpoint2 >= INITIAL_SETPOINT) { 
    setpoint1 = FORWARD_SETPOINT;
    waitForSettle(input1, setpoint1, motor1, myPID1, encoder1);

    delay(1000); // Ensure the first leg finishes moving

    setpoint2 = BACKWARD_SETPOINT;
    waitForSettle(input2, setpoint2, motor2, myPID2, encoder2);
  }
}

static void walk() {
  if (setpoint1 >= FORWARD_SETPOINT && setpoint2 <= BACKWARD_SETPOINT) { 
    setpoint1 = INITIAL_SETPOINT - SETTLE_ANGLE_THRESHOLD;
    waitForSettle(input1, setpoint1, motor1, myPID1, encoder1);

    delay(1000); // Ensure the outer leg finishes moving

    setpoint2 = FORWARD_SETPOINT;
    waitForSettle(input2, setpoint2, motor2, myPID2, encoder2);
  }

  if (setpoint1 <= INITIAL_SETPOINT - SETTLE_ANGLE_THRESHOLD && setpoint2 >= FORWARD_SETPOINT) {
    setpoint1 = FORWARD_SETPOINT;
    waitForSettle(input1, setpoint1, motor1, myPID1, encoder1);

    delay(1000); // Ensure the outer leg finishes moving

    setpoint2 = INITIAL_SETPOINT - SETTLE_ANGLE_THRESHOLD;
    waitForSettle(input2, setpoint2, motor2, myPID2, encoder2);
  }
}

void waitForSettle(double &input, double setpoint, Servo &motor, PID &pid, Encoders &encoder) {
    int unsettled = SETTLE_COUNT_MAX;

    while (unsettled) {
        long currentEncoderCount = encoder.getEncoderCount();
        double angle = currentEncoderCount / ANGLE_MULTIPLIER;
        input = -angle;
        pid.Compute();
        motor.writeMicroseconds(output1);

        if (abs(input - setpoint) <= SETTLE_ANGLE_THRESHOLD) {
            unsettled--;
        } else {
            unsettled = SETTLE_COUNT_MAX;
        }

        delay(SETTLE_DELAY);
    }
}

