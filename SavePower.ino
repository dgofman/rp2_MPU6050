#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include "pico/stdlib.h"

#define DEBUG true
#define INTERRUPT_PIN 15 // Interrupt pin connected to MPU6050 INT

Adafruit_MPU6050 mpu;

unsigned long startMillis = 0;  // Time of the last motion detection
const unsigned long cooldownTime = 5000;  // Cooldown period in milliseconds
volatile bool motionDetected = false;

void setup() {
  if (DEBUG) {
    Serial.begin(115200);

    startMillis = millis();  // Store the current time
    while (!Serial) {
      if (millis() - startMillis >= 3000) {  // Check if 3 seconds have passed
        break;  // Exit the loop after 3 seconds
      }
      delay(10);  // Wait for a short period to avoid freezing
    }
  } else {
    Serial.end();
  }

  // Initialize the MPU6050 sensor
  if (!mpu.begin()) {
    println("Failed to initialize MPU6050!");
    while (1);  // Stop the program if initialization fails
  }

  // Configure MPU6050 motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);  // Set high-pass filter
  mpu.setMotionDetectionThreshold(1);  // Set motion detection threshold (1g)
  mpu.setMotionDetectionDuration(20);  // Set motion detection duration (20ms)
  mpu.setInterruptPinLatch(true);  // Latch interrupt pin (keep the interrupt active until read)
  mpu.setInterruptPinPolarity(true);  // Active HIGH polarity for interrupt pin
  mpu.setMotionInterrupt(true);  // Enable motion interrupt on INT pin

  // Configure the interrupt pin (GPIO 15)
  pinMode(INTERRUPT_PIN, INPUT);  // Set the INT pin as an input

  // Attach the interrupt to the motion detection handler
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), motionInterruptHandler, FALLING);

  savePower();
}

void loop() {
  // If motion is detected, handle it and then sleep again
  if (motionDetected) {
    if (millis() - startMillis >= cooldownTime) {
      startMillis = millis();
      motionDetected = false;  // Reset the motion flag
      motionHandler();
      if (mpu.getMotionInterruptStatus()) { // Enable motionInterruptHandler 
        println("Going to deep sleep...");
        // mpu.enableCycle(false);
        mpu.enableSleep(true);

        sleep_ms(5000); // enter deep sleep 

        // mpu.enableCycle(true);
        mpu.enableSleep(false);
      }
    }
  }
  Serial.print('.');
  delay(500);
}

void println(String msg) {
  if (DEBUG) {
    Serial.println(msg);
  }
}

// https://youtu.be/GqmnV_T4yAU?t=62
void savePower() {
  // Disable WiFi to save power
  WiFi.disconnect(true);  // Disconnect any existing WiFi connections
  WiFi.mode(WIFI_OFF);    // Turn off WiFi module

  pinMode(LED_BUILTIN, OUTPUT);  // Set the onboard LED pin as output
  digitalWrite(LED_BUILTIN, LOW);  // Turn off the onboard LED

  pinMode(26, INPUT);  // ADC pin to measure the voltage drop across sense resistor

  // Turn off External Modules (WiFi, Bluetooth, etc.) Using GPIO
  pinMode(23, OUTPUT);      // Set GPIO 23 as output for external module
  digitalWrite(23, LOW);    // Turn off the external module (assuming LOW powers it off)

  println("Going to sleep...");
  sleep_until(digitalPinToInterrupt(INTERRUPT_PIN) == LOW);  // Wait until interrupt pin is triggered
}

void motionHandler() {
  println("Motion detected!");
}

// Interrupt service routine (ISR) for motion detection
void motionInterruptHandler() {
  println(".");
  motionDetected = true;
}