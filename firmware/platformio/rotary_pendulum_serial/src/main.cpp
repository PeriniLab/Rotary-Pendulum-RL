#include <ESP32Encoder.h>
#include <Arduino.h>
#include "utils.h"

// Define the pins used by the sensors
#define ENCODER_A 14
#define ENCODER_B 27
#define HALL_SENSOR 26

// Define the pins used by the stepper driver
#define STEP_PIN 18
#define DIR_PIN 19
#define ENABLE_PIN 23
// 1ULL shifts 1 to the left by STEP_PIN positions
#define STEP_BIT (1ULL << STEP_PIN)
#define DIR_BIT  (1ULL << DIR_PIN)
#define ENABLE_BIT (1ULL << ENABLE_PIN)
#define HALL_BIT (1ULL << HALL_SENSOR)

// Define the number of networks you have
#define NUM_NETWORKS 3

// Create a list of SSID and passwords
const char* ssids[NUM_NETWORKS] = {"FASTWEB-FC20AB", "ees-lab", "CasaCamilla_3"};
const char* passwords[NUM_NETWORKS] = {"PP2NEEGH67", "cipiacelafft", "GranseolaCheSiole9102"};

const long encoderSteps = 1200; // Encoder steps per half rotation
ESP32Encoder encoder(true, NULL, NULL);

const int steps_per_rev = 3200;
// steps per second
double maxSpeed = 4000.0; // Max speed in steps per second
volatile double currentAngle = 0.0; // Current angle of the motor in degrees
volatile double rangeAngle = 200.0; // Range of motion of the motor in degrees
volatile const double degreesPerStep = 360.0 / steps_per_rev;
volatile double speed = 0.0;
volatile double homingSpeed = 2000.0;
bool dir = false; // false = counter-clockwise, true = clockwise
volatile long stepCount = 0;
volatile bool homing = false;
volatile bool episodeDone = false;
volatile int hallSensorDebounce = 0;
volatile bool hallSensorCentering = false;
// millis variables for timing
double timestep = 5.0; // ms
double theta_dot = 0.0;
double oldTheta = 0.0;
unsigned long oldTime = 0.0;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void sendSerialOutput(double angle, double angularVelocity, double currentAngle, bool episodeDone) {
  // Format the output string
  String output = String(angle) + "," + String(angularVelocity) + "," + String(currentAngle) + "," + String(episodeDone);
  Serial.println(output);
}

// Function to set direction
void setDirection(bool direction) {
  dir = direction;
  if (dir) {
    GPIO.out |= DIR_BIT; // Set direction pin to HIGH
  } else {
    GPIO.out &= ~DIR_BIT; // Set direction pin to LOW
  }
}

// Function to set speed
void setSpeed(double newSpeed) {
  if ((int)newSpeed == 0) {
    portENTER_CRITICAL(&timerMux);
    speed = newSpeed;
    timerAlarmDisable(timer);  // Disable the timer
    portEXIT_CRITICAL(&timerMux);
    // STEP pin to LOW to ensure the motor is not in mid-step
    GPIO.out &= ~STEP_BIT;
  } else {
    portENTER_CRITICAL(&timerMux);
    speed = newSpeed;
    timerAlarmWrite(timer, 1000000 / speed, true); // Update the timer interval
    timerAlarmEnable(timer);  // Re-enable the timer
    portEXIT_CRITICAL(&timerMux);
  }
}

// Function to zero the position
void zeroPosition() {
  portENTER_CRITICAL(&timerMux);
  stepCount = 0; // Reset step count
  portEXIT_CRITICAL(&timerMux);
}

// Function to set speed based on percentage
void setSpeedPercentage(int percentage) {
  speed = maxSpeed * (percentage / 100.0);
  setSpeed(speed);
}

// Function to move to home position
void moveToHome() {
  homing = true;
  if (currentAngle > 0.0) {
    dir = false;
  }
  else if (currentAngle < 0.0) {
    dir = true;
  }
  else {
    homing = false;
    episodeDone = false;
    zeroPosition();
    setSpeed(0);
    return;
  }
  setDirection(dir);
  setSpeed(homingSpeed);
}

void IRAM_ATTR onTimer() {
  // If the speed is set to zero, exit the ISR without toggling the STEP pin
  if ((int)speed == 0) {
    return;
  }
  portENTER_CRITICAL_ISR(&timerMux);
  
  // Toggle the step pin for moving the motor
  if (GPIO.out & STEP_BIT) {
    GPIO.out &= ~STEP_BIT; // Set step pin to LOW
  } else {
    GPIO.out |= STEP_BIT; // Set step pin to HIGH
  }

  if (GPIO.in & HALL_BIT) {
    zeroPosition();
  }

  // Change steps only when step pin goes HIGH
  if (GPIO.out & STEP_BIT) {
    // update the steps the motor has moved from the beginning
    stepCount += dir ? 1 : -1;
    // convert steps to degree angle
    currentAngle = stepCount * degreesPerStep;
    // if the pendulum is not homing
    if (!homing){
      // if it is not inside the permitted range, bring pendulum back to home position
      if ((currentAngle >= rangeAngle/2 && !episodeDone) || (currentAngle <= -rangeAngle/2 && !episodeDone)) {
        episodeDone = true;
        moveToHome();
      }
      else {
        episodeDone = false;
      }
    }
    // if it is homing
    else {
      // check if the hall sensor gets activated and reset the position and speed
      if (GPIO.in & HALL_BIT && !hallSensorCentering) {
        hallSensorCentering = true;
        hallSensorDebounce = 0;
      }
      else if (GPIO.in & HALL_BIT && hallSensorCentering) {
        hallSensorDebounce++;
      }
      else if (!(GPIO.in & HALL_BIT) && hallSensorCentering && hallSensorDebounce > 5) {
        hallSensorCentering = false;
        hallSensorDebounce = 0;
        homing = false;
        episodeDone = false;
        zeroPosition();
        setSpeed(0);
        
      }
    }
  }
  
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(HALL_SENSOR, INPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable the motor

  // Connect to WiFi
  setupWiFi(ssids, passwords, NUM_NETWORKS);

  // to get accurate readings, the encoder ISR should be serviced by a core
  ESP32Encoder::isrServiceCpuCore=1;
  encoder.attachFullQuad(ENCODER_A, ENCODER_B);
  encoder.clearCount();
  encoder.setFilter(1023);

  Serial.begin(115200);

  // Set up the timer
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, counting up
  timerAttachInterrupt(timer, &onTimer, true); // Attach onTimer function
  timerAlarmEnable(timer); // Enable the timer
}

void loop() {
  // get encoder count
  long currentCount = encoder.getCount();
  // map it in range [-3.14, 3.14]
  double theta = getTheta(currentCount, encoderSteps);

  // Check for incoming serial data if episode is not done
  if (Serial.available() > 0 && !episodeDone) {
    String command = Serial.readStringUntil('\n'); // Read the incoming data until newline
    command.trim(); // Remove any whitespace

    // Parse the command and execute it
    if (command.length() > 0) {
      // parse the command, for example "-20,0" to set the speed
      int commaIndex = command.indexOf(',');
      if (commaIndex != -1) {
        String speedStr = command.substring(0, commaIndex);
        String episodeStr = command.substring(commaIndex + 1);

        // clip speed percentage to [-100, 100]
        int speedPercentage = clip(speedStr.toInt(), -100, 100);
        bool episode = episodeStr.toInt(); // Convert episode to boolean (0 or 1)

        // if episode is not done
        if (!episode) {
          // Update speed and direction based on the command
          if (speedPercentage < 0) {
            dir = false;
            setDirection(dir); // Set direction counterclockwise
            speedPercentage = -speedPercentage; // Make the percentage positive
          } else if (speedPercentage > 0) {
            dir = true;
            setDirection(dir); // Set direction clockwise
          }
          else {
            speedPercentage = 0;
          }

          setSpeedPercentage(speedPercentage); // Set speed as a percentage of max speed
          episodeDone = false;
        }
        else {
          // If episode is done, bring pendulum back to home position
          episodeDone = true;
          moveToHome();
        }
      }
    }
  }

  // send data to serial
  if (millis() - oldTime >= timestep) {
    theta_dot = (theta - oldTheta) / (timestep/1000.0);
    // clip theta_dot to [-10, 10]
    theta_dot = clip(theta_dot, -10.0, 10.0);
    sendSerialOutput(theta, theta_dot, currentAngle, episodeDone);
    oldTime = millis();
    oldTheta = theta;    
  }
  
}
