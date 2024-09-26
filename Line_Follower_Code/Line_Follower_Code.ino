#include <Arduino.h>

// Defining Variables 
  // Motor control pins
    const byte leftMotorEnablePin = 3;   // Enable pin for the left motor 3
    const byte leftMotorInput1 = 4;     // Input pin 1 for the left motor 4
    const byte leftMotorInput2 = 5;     // Input pin 2 for the left motor 5
    const byte rightMotorEnablePin = 6;  // Enable pin for the right motor 6
    const byte rightMotorInput1 = 8;    // Input pin 1 for the right motor 8 
    const byte rightMotorInput2 = 7;    // Input pin 2 for the right motor 7

  // Indicating LED
    const int LED = 13;

  // IR sensor Pins
    const byte sensorPins[] = {A0, A1, A2, A3, A4, A5};
    const int numSensors = sizeof(sensorPins) / sizeof(sensorPins[0]);  // Number of sensors
    float arena_threshold;

  // Motor regulation
    int motorSpeed = 120;

/**
 * @brief Blinks the LED according to the specified number of times.
 * 
 * @param blinkCount Number of times the LED should blink.
 * @param pin The pin number to which the LED is connected.
 */
void blinkLED(int blinkCount, int pin) {
  for (int i = 0; i < blinkCount; i++) {
    digitalWrite(pin, HIGH);  // Turn the LED on
    delay(500);               // Wait for 500 milliseconds
    digitalWrite(pin, LOW);   // Turn the LED off
    delay(500);               // Wait for 500 milliseconds
  }
}

/**
 * @brief Collects the IR data from the surface and calculates the average value.
 * 
 * @param sensorPins Array of sensor pin numbers.
 * @param numSensors Number of sensors.
 * @param readings Number of readings to average.
 * @return The average value of the sensor readings.
 */
float surfaceData(const byte sensorPins[], int numSensors, int readings) {
  float sum = 0.0;

  for (int j = 0; j < readings; j++) {
    for (int i = 0; i < numSensors; i++) {
      sum += analogRead(sensorPins[i]);
      delay(100);  // Small delay between sensor readings
    }
  }
  
  float average = sum / (numSensors * readings);
  return average;
}

/**
 * @brief Calculates the threshold value for IR sensor readings by measuring 
 *        the average IR values of white and black surfaces.
 * 
 * @param sensorPins Array of sensor pin numbers.
 * @param numSensors Number of sensors.
 * @param readings Number of readings to average.
 * @return The calculated threshold value.
 */
float calculateThreshold(const byte sensorPins[], int numSensors, int readings) {
  // Blink LED to indicate starting white surface measurement
  blinkLED(2, LED);
  delay(500);
  
  // Measure average IR value for white surface
  float white = surfaceData(sensorPins, numSensors, readings);

  // Blink LED to indicate starting black surface measurement
  blinkLED(2, LED);
  delay(500);
  
  // Measure average IR value for black surface
  blinkLED(4,LED);
  float black = surfaceData(sensorPins, numSensors, readings);

  // Calculate and return the threshold value
  float threshold = (white + black) / 2;

  return threshold;
}

//float continuousThreshold(float sensorValue)
  
  //}
   


/**
 * @brief Reads sensor values and updates a byte representing sensor data based on a threshold.
 * 
 * This function reads the values from multiple sensors, compares each sensor's value to a given threshold,
 * and updates a byte where each bit represents whether the corresponding sensor value exceeds the threshold.
 * 
 * @param sensorPins Array of sensor pin numbers.
 * @param numSensors Number of sensors.
 * @param threshold Threshold value for sensor comparison.
 * @return A byte where each bit is set if the corresponding sensor's value exceeds the threshold.
 */
byte sensorRead(const byte sensorPins[], int numSensors, float threshold) {
  byte sensorData = 0;  // Initialize the byte to store sensor data
  
  float white[2] = {0.0f, 0.0f};
  float black[2] = {0.0f, 0.0f};
  
  // Iterate over each sensor
  for (int i = 0; i < numSensors; i++) {
    float sensorValue = analogRead(sensorPins[i]);  // Read the analog value from the sensor

    // Check if the sensor value exceeds the threshold
    if (sensorValue > threshold) {
      // Set the corresponding bit in sensorData to 1 if sensor value is above threshold
      sensorData |= (1 << i);
      black[0] += sensorValue;
      if (black[1] > sensorValue || black[1] == 0){
        black[1] = sensorValue;
      }
    } else {
      white[0] += sensorValue;
      if (white[1] < sensorValue || white[1] == 0){
        white[1] = sensorValue;
      }
    }
  }
  
  

  return sensorData;  // Return the byte with updated sensor data
}

/**
 * @brief Maps sensor data byte to error for line condition.
 * 
 * @param Case The sensor data byte.
 * @return The error corresponding to the sensor data byte.
 */
float lineCondition(int Case) {
  float error;

  switch (Case) {
    case 32:
      error = 2;
      break;
    case 48:
      error = 1.25;
      break;
    case 56:
      error = 0.75;
      break;
    case 24:
      error = 0.3;
      break;
    case 12:
      error = 0;
      break;
    case 6:
      error = -0.3;
      break;
    case 7:
      error = -0.75;
      break;
    case 3:
      error = -1.25;
      break;
    case 1:
      error = -2;
      break;
    default:
      error = 0;
      break;
  }

  return error;
}

/**
 * @brief Controls the motors based on error.
 * 
 * @param error The error for motor control.
 * @param motorSpeed The base motor speed.
 * @param difference The speed difference for turning.
 */
void motorControl(float correction, int motorSpeed, int difference) {
  int leftspeed = motorSpeed + correction * difference;
  int rightspeed = motorSpeed - correction * difference;

  leftspeed = constrain(leftspeed, 100, 255);
  rightspeed = constrain(rightspeed, 100, 255);
  
  if (leftspeed == rightspeed) {
    digitalWrite(leftMotorInput1, HIGH);
    digitalWrite(leftMotorInput2, LOW);

    digitalWrite(rightMotorInput1, HIGH);
    digitalWrite(rightMotorInput2, LOW);
  } else if (leftspeed < rightspeed) {
    digitalWrite(leftMotorInput1, HIGH);
    digitalWrite(leftMotorInput2, LOW);

    digitalWrite(rightMotorInput1, LOW);
    digitalWrite(rightMotorInput2, HIGH);
  } else {
    digitalWrite(leftMotorInput1, LOW);
    digitalWrite(leftMotorInput2, HIGH);

    digitalWrite(rightMotorInput1, HIGH);
    digitalWrite(rightMotorInput2, LOW);
  }

  // Set motor speeds using PWM
  analogWrite(leftMotorEnablePin, leftspeed);
  analogWrite(rightMotorEnablePin, rightspeed);
}

/**
 * @brief Calculates the PID correction value.
 * 
 * @param error The current error value.
 * @param Kp The proportional gain.
 * @param Ki The integral gain.
 * @param Kd The derivative gain.
 * @return The calculated correction value.
 */
float errorCalculation(float error, float K1, float K2, float K3){
  // PID Calculation
  static float previousError = 0;
  static signed long previousDerivative = 0;

  // To damp the oscillation speed 
  signed long derivative1 = (error - previousError);
  signed long derivative2 = (derivative1 - previousDerivative);

  float correction = K1 * error + K2 * derivative1 + K3 * derivative2;
  
  previousError = error;
  previousDerivative = derivative1;
  
  return correction;
}

void setup() {
  Serial.begin(9600);
  // Indicator LED
  pinMode(LED, OUTPUT);

  // Left motor pins setup
  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorInput1, OUTPUT);
  pinMode(leftMotorInput2, OUTPUT);

  digitalWrite(leftMotorEnablePin, LOW);
  digitalWrite(leftMotorInput1, LOW);
  digitalWrite(leftMotorInput2, LOW);

  // Right motor pins setup
  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorInput1, OUTPUT);
  pinMode(rightMotorInput2, OUTPUT);

  digitalWrite(rightMotorEnablePin, LOW);
  digitalWrite(rightMotorInput1, LOW);
  digitalWrite(rightMotorInput2, LOW);

  // Sensor pins setup
  for (byte i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Calculate the threshold
  arena_threshold = calculateThreshold(sensorPins, numSensors, 20); // Readings = 20

  Serial.begin(9600);
}

void loop() {
  // Read sensor data and get the condition of the line
  byte Case = sensorRead(sensorPins, numSensors, arena_threshold);
  float error = lineCondition(Case);
  float correction = errorCalculation(error, 4, 4, 1); //K1, K2, K3 order of entry
  motorControl(correction, motorSpeed, 12); // Difference = 12
  
}
