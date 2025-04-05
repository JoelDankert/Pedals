#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   in radians

// Define button pins
const int buttonPin1 = 18; // Button 1 on GPIO18
const int buttonPin2 = 17; // Button 2 on GPIO17
const int buttonPin3 = 4;  // Button 3 on GPIO4

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C with the specified SDA and SCL pins
  Wire.begin(21, 22);  // Replace with your actual SDA/SCL pins
  delay(1000);

  // Set up button pins with internal pull-ups
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    while (1);
  }
}

void loop() {
  if (!dmpReady) return;

  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    mpu.resetFIFO(); // flush the buffer
    Serial.println("FIFO overflow!");
  }
  else if (fifoCount >= packetSize) {
    while (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    // Decode the latest valid packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Print orientation and button states on one line.
    // Orientation values are divided by PI to convert radians to multiples of PI.
    Serial.print("[");
    Serial.print(ypr[0] / M_PI);
    Serial.print(",");
    Serial.print(ypr[1] / M_PI);
    Serial.print(",");
    Serial.print(ypr[2] / M_PI);
    Serial.print("],[");

    // For buttons: since they are connected to ground and use pull-ups,
    // a pressed button returns LOW. We output 1 when pressed, 0 otherwise.
    Serial.print((digitalRead(buttonPin1) == LOW) ? 1 : 0);
    Serial.print(",");
    Serial.print((digitalRead(buttonPin2) == LOW) ? 1 : 0);
    Serial.print(",");
    Serial.print((digitalRead(buttonPin3) == LOW) ? 1 : 0);
    Serial.println("]");
  }

  delay(10);
}
