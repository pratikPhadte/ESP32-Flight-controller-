#include <Wire.h>

// Raw sensor data variables
int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroX, GyroY, GyroZ;

// Processed data variables
volatile float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

// Calibration variables
volatile float RateCalibrationPitch = 0, RateCalibrationRoll = 0, RateCalibrationYaw = 0;
float AccXCalibration = 0, AccYCalibration = 0, AccZCalibration = 0;

// Number of samples for calibration
const int CalibrationSamples = 2000;

void gyro_signals(void)
{
  // Configure the low-pass filter (DLPF - Digital Low Pass Filter)
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // Write to the configuration register (0x1A)
  Wire.write(0x05); // Set DLPF_CFG to 5, which configures a low-pass filter at ~10 Hz
  Wire.endTransmission();

  // Configure accelerometer sensitivity
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Write to accelerometer configuration register (0x1C)
  Wire.write(0x10); // Set sensitivity to ±8g
  Wire.endTransmission();

  // Begin reading accelerometer data from register 0x3B
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Address of the first accelerometer data register (X-axis high byte)
  Wire.endTransmission();

  // Request 6 bytes of accelerometer data (X, Y, Z axes, each 16 bits)
  Wire.requestFrom(0x68, 6);

  // Combine the high and low bytes to form 16-bit signed integers
  AccXLSB = Wire.read() << 8 | Wire.read(); // X-axis acceleration
  AccYLSB = Wire.read() << 8 | Wire.read(); // Y-axis acceleration
  AccZLSB = Wire.read() << 8 | Wire.read(); // Z-axis acceleration

  // Set gyro sensitivity scale factor
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Write to gyro configuration register (0x1B)
  Wire.write(0x08); // Set sensitivity to ±500 degrees/sec (scale factor: 65.5 LSB/°/sec)
  Wire.endTransmission();

  // Begin reading gyro data from register 0x43
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Address of the first gyro data register (X-axis high byte)
  Wire.endTransmission();

  // Request 6 bytes of gyro data (X, Y, Z axes, each 16 bits)
  Wire.requestFrom(0x68, 6);

  // Combine the high and low bytes to form 16-bit signed integers
  GyroX = Wire.read() << 8 | Wire.read(); // X-axis gyro rate
  GyroY = Wire.read() << 8 | Wire.read(); // Y-axis gyro rate
  GyroZ = Wire.read() << 8 | Wire.read(); // Z-axis gyro rate

  // Convert gyro raw data (LSB) to angular velocity in degrees/sec
  RateRoll = (float)GyroX / 65.5;  // Roll rate (X-axis rotation)
  RatePitch = (float)GyroY / 65.5; // Pitch rate (Y-axis rotation)
  RateYaw = (float)GyroZ / 65.5;   // Yaw rate (Z-axis rotation)

  // Convert accelerometer raw data (LSB) to acceleration in g's
  AccX = (float)AccXLSB / 4096; // X-axis acceleration
  AccY = (float)AccYLSB / 4096; // Y-axis acceleration
  AccZ = (float)AccZLSB / 4096; // Z-axis acceleration
}

void calibrate_sensors()
{
  float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  float sumGyroRoll = 0, sumGyroPitch = 0, sumGyroYaw = 0;

  // Collect multiple samples to calculate average values
  for (int i = 0; i < CalibrationSamples; i++)
  {
    gyro_signals();

    sumAccX += AccX;
    sumAccY += AccY;
    sumAccZ += AccZ;

    sumGyroRoll += RateRoll;
    sumGyroPitch += RatePitch;
    sumGyroYaw += RateYaw;

    delay(1); // Short delay to avoid overwhelming the sensor
  }

  // Calculate average offset values
  AccXCalibration = sumAccX / CalibrationSamples;
  AccYCalibration = sumAccY / CalibrationSamples;
  AccZCalibration = (sumAccZ / CalibrationSamples) - 1.0; // Subtract 1g for gravity

  RateCalibrationRoll = sumGyroRoll / CalibrationSamples;
  RateCalibrationPitch = sumGyroPitch / CalibrationSamples;
  RateCalibrationYaw = sumGyroYaw / CalibrationSamples;

  // Print calibrated values
  Serial.println("Calibration Completed:");
  Serial.print("AccX Calibration: ");
  Serial.println(AccXCalibration);
  Serial.print("AccY Calibration: ");
  Serial.println(AccYCalibration);
  Serial.print("AccZ Calibration: ");
  Serial.println(AccZCalibration);
  Serial.print("Gyro Roll Calibration: ");
  Serial.println(RateCalibrationRoll);
  Serial.print("Gyro Pitch Calibration: ");
  Serial.println(RateCalibrationPitch);
  Serial.print("Gyro Yaw Calibration: ");
  Serial.println(RateCalibrationYaw);
}

void setup()
{
  Serial.begin(115200);

  // Set the clock speed of I2C to 400kHz
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Start the gyro in power mode.
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Wake up MPU6050
  Wire.endTransmission(true);

  delay(100);

  // Calibration instructions
  Serial.println("Welcome to quadcopter IMU calibration");
  Serial.println("Place the quadcopter flat on a surface and press 'Space bar' and then 'Enter' to begin Gyro and AccZ calibration.");
  while (!Serial.available()); // Wait for the user to press 'Enter'
  Serial.read(); // Clear the input buffer

  // Perform sensor calibration
  Serial.println("Calibrating sensors...");
  calibrate_sensors();
}

void loop()
{
  gyro_signals();

  // Correct the measured values using calibration offsets
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  // Print the accelerometer and gyroscope values
  Serial.print("AccX: ");
  Serial.print(AccX);
  Serial.print(", AccY: ");
  Serial.print(AccY);
  Serial.print(", AccZ: ");
  Serial.print(AccZ);

  Serial.print(" | GyroX: ");
  Serial.print(RateRoll);
  Serial.print(", GyroY: ");
  Serial.print(RatePitch);
  Serial.print(", GyroZ: ");
  Serial.println(RateYaw);

  delay(1); // Delay for 100ms before reading again
}