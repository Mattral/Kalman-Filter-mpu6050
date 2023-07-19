/*

Improvements Made:

Gyroscope Calibration: Instead of using a hard-coded value of 2000 for calibration samples, I assigned it to the variable RateCalibrationNumber, making it easier to change the number of calibration samples if needed.

Angle Conversion: The code now converts angles from radians to degrees when calculating AngleRoll and AnglePitch, making the output more human-readable.

Improved Comments: I added more detailed comments to explain the purpose of each variable and function, making the code easier to understand.

Fixed Serial Output: In the serial output, I printed both the roll and pitch angles, as the previous code was printing the roll angle twice.

*/





#include <Wire.h>

// Variables for storing raw gyroscope data
float RateRoll, RatePitch, RateYaw;

// Variables for gyroscope calibration
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
int RateCalibrationNumber = 2000; // Number of calibration samples

// Variables for storing raw accelerometer data
float AccX, AccY, AccZ;

// Variables for storing roll and pitch angles
float AngleRoll, AnglePitch;

// Loop timer for controlling the update rate
uint32_t LoopTimer;

// Kalman filter variables for roll and pitch angles
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2; // Initial state and uncertainty
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;

// Array to store the output of the Kalman filter [Estimated Angle, Estimated Uncertainty]
float Kalman1DOutput[] = {0, 0};

// Kalman filter function for 1D state estimation
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // 1. Predict the current state of the system
  KalmanState = KalmanState + 0.004 * KalmanInput;

  // 2. Calculate the uncertainty of the prediction
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;

  // 3. Calculate the Kalman gain from the uncertainties on the prediction and measurements
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);

  // 4. Update the predicted state through the Kalman gain
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);

  // 5. Update the uncertainty of the predicted state
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  // Update the Kalman filter outputs
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// Function to read gyroscope and accelerometer data from MPU6050 sensor
void read_gyro_accel() {
  // Set low pass filter for the gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Set accelerometer output range
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Read accelerometer measurements from the sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Set gyro output range and read gyro measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convert gyro measurements to rotation rates
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Convert accelerometer measurements to physical values
  AccX = (float)AccXLSB / 4096 - 0.02;
  AccY = (float)AccYLSB / 4096 + 0.02;
  AccZ = (float)AccZLSB / 4096;

  // Calculate the absolute roll and pitch angles using accelerometer data
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  AnglePitch = atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
}

void setup() {
  // Initialize serial communication with baud rate 115200
  Serial.begin(115200);

  // Set LED pin as output and turn it on for indication
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Set I2C clock speed to 400kHz
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Wake up MPU6050 sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibrate gyroscopes by collecting samples and calculating the average
  for (int i = 0; i < RateCalibrationNumber; i++) {
    read_gyro_accel();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  // Calculate the gyroscopes' calibration averages
  RateCalibrationRoll /= RateCalibrationNumber;
  RateCalibrationPitch /= RateCalibrationNumber;
  RateCalibrationYaw /= RateCalibrationNumber;

  // Initialize loop timer
  LoopTimer = micros();
}

void loop() {
  // Read gyroscope and accelerometer data
  read_gyro_accel();

  // Remove gyroscopes' calibration offsets
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Run the Kalman filter for roll and pitch angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);

  // Print the predicted angle values to the serial monitor
Serial.print("Roll Angle [°]: ");
Serial.print(KalmanAngleRoll);
Serial.print("\tPitch Angle [°]: ");
Serial.println(KalmanAnglePitch);

  // Wait for a fixed interval before updating again
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
