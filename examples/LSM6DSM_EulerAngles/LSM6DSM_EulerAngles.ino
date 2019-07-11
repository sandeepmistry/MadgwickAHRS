/*
  Arduino LSM6DSM - Euler Angles

  This example reads the accelerometer and gyroscope values 
  from the LSM6DSM sensor and converts them to Euler angles
  using an Madgwick AHRS filer, then continuously prints the
  values to the Serial Monitor or Serial Plotter.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT

  This example code is in the public domain.
*/

#include <Arduino_LSM6DSM.h>
#include <MadgwickAHRS.h>

Madgwick filter;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // start the IMU and filter
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  float accelerationSampleRate = IMU.accelerationSampleRate();
  float gyroscopeSampleRate = IMU.gyroscopeSampleRate();

  Serial.print("Accelerometer sample rate = ");
  Serial.print(accelerationSampleRate);
  Serial.println(" Hz");
  Serial.print("Gyroscop sample rate = ");
  Serial.print(gyroscopeSampleRate);
  Serial.println(" Hz");
  Serial.println();

  filter.begin(min(accelerationSampleRate, gyroscopeSampleRate));

  Serial.println("Heading\tPitch\tRoll");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float heading, pitch, roll;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    // read data from the IMU
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    heading = filter.getYaw();
    pitch = filter.getPitch();
    roll = filter.getRoll();

    Serial.print(heading);
    Serial.print('\t');
    Serial.print(pitch);
    Serial.print('\t');
    Serial.println(roll);
  }
}
