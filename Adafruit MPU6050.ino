#include <hardware/regs/m0plus.h>
#include <hardware/regs/addressmap.h>

#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

#define SAMPLERATE_DELAY_MS 100
#define G 9.81 

bool ledState = LOW;
unsigned long millisOld;
float dt;
float phi[] = {.0, .0, .0, .0, .0};
float theta[]   = {.0, .0, .0, .0, .0};
float gyroOffsets[] = {0, 0, 0};  // Offsets for X, Y, Z

const float noiseFilter = .1;
const int calibrationSamples = 100;  // Number of samples for calibration

inline void rebootNaive() {
    auto & AIRCR_register = *(volatile uint32_t*)(PPB_BASE + M0PLUS_AIRCR_OFFSET);
    AIRCR_register = (0x05FA << M0PLUS_AIRCR_VECTKEY_LSB) | M0PLUS_AIRCR_SYSRESETREQ_BITS;
}

void setup(void) {
  Serial.begin(115200);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate the gyroscope
  for (int i = 0; i < calibrationSamples; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);  // Read sensor data
      gyroOffsets[0] += gyroX(g);
      gyroOffsets[1] += gyroY(g);
      gyroOffsets[2] += g.gyro.z;
      delay(10);  // Small delay between samples
  }

  // Calculate average offsets
  gyroOffsets[0] /= calibrationSamples;
  gyroOffsets[1] /= calibrationSamples;
  gyroOffsets[2] /= calibrationSamples;

  millisOld = millis();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  delay(100);
}

float accelX(sensors_event_t a) {
  return -a.acceleration.x; // MPU6050 in 90 degrees  
}

float accelY(sensors_event_t a) {
  return -a.acceleration.y; // MPU6050 in 90 degrees  
}

float accelZ(sensors_event_t a) {
  return a.acceleration.z; // MPU6050 in 90 degrees  
}

float gyroX(sensors_event_t g) {
  return g.gyro.y; // MPU6050 in 90 degrees  
}

float gyroY(sensors_event_t g) {
  return -g.gyro.x; // MPU6050 in 90 degrees  
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float gx = gyroX(g) - gyroOffsets[0];
  float gy = gyroY(g) - gyroOffsets[1];
  float x = accelX(a);
  float y = accelY(a);
  float z = accelZ(a) / G;

  phi[0] = atan2(y, z) * (180 / PI) / G;
  theta[0] = atan2(x, z) * (180 / PI) / G;

  /* Low Pass Filters */
  float noice = (1 - noiseFilter);
  phi[1] = noice * phi[2] + noiseFilter * phi[0];
  theta[1] = noice * theta[2] + noiseFilter * theta[0];
  phi[2] = phi[1];
  theta[2] = theta[1];

  /* Using Gyros for Measuring */
  dt = (millis() - millisOld) / 1000.;
  millisOld = millis();
  phi[3] += gy * dt;
  theta[3] += gx * dt;

  /* Accurate and Stable Tilt */
  phi[4] = (phi[4] - gy * dt) * noice + phi[0] * noiseFilter;
  theta[4] = (theta[4] - gy * dt) * noice + theta[0] * noiseFilter;

  /* Print out the values */
  Serial.print(x);  
  Serial.print(",");
  Serial.print(y);  
  Serial.print(",");
  Serial.print(z);
  Serial.print(",");

  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(g.gyro.z - gyroOffsets[2]);
  Serial.print(",");
  Serial.print(theta[0]);
  Serial.print(",");
  Serial.print(phi[0]);
  Serial.print(",");
  Serial.print(theta[1]);
  Serial.print(",");
  Serial.print(phi[1]);
  Serial.print(",");
  Serial.print(theta[3]);
  Serial.print(",");
  Serial.print(phi[3]);
  Serial.print(",");
  Serial.print(theta[4]);
  Serial.print(",");
  Serial.print(phi[4]);
  Serial.println(); 
  
  char command = Serial.read();
  if (command == 's') {
      Serial.println("Stopping...");
      rebootNaive();
  } else if (command == 'i') {
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
      case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
      case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
      case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
      case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }

    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
      case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
      case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
      case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
      case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
      case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
      case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
      case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
      case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
      case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
      case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
      case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }
    delay(5000);
  }

  digitalWrite(LED_BUILTIN, ledState = !ledState);
  delay(SAMPLERATE_DELAY_MS);
}
