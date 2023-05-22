/*
 * SenseGlove
 * (C) 2023 UnexomWid
 */
#include "uwtypes.h"
#include "FastIMU.h"

#define PERFORM_CALIBRATION
#define IMU_ADDRESS 0x68

enum class RecvOpcode : uint8 {
  POLL = 0,
  CALIBRATE = 1,
  SYNC = 0xFF
};

enum class SendOpcode : uint8 {
  SYNC = 0xFF
};

MPU6500 imu;

calData calib = { 0 };  //Calibration data
RawAccelData accelData; //Sensor data
RawGyroData gyroData;

void led(bool state) {
  digitalWrite(LED_BUILTIN, state ? HIGH : LOW);
}

void init_imu() {
  imu.init(calib, IMU_ADDRESS);
  imu.setGyroRange(250);
  imu.setAccelRange(2);
}

void calibrate() {
  imu.calibrateAccelGyro(&calib);
  init_imu();
}

void wait_for_sync() {
  while (true) {
    if (!Serial.available()) {
      continue;
    }

    RecvOpcode op = (RecvOpcode) Serial.read();
    if (op == RecvOpcode::SYNC) {
      Serial.write((uint8) SendOpcode::SYNC);
      return;
    }
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial);
  
  init_imu();

  wait_for_sync();
  led(true);
}

void loop() {
  if (!Serial) {
    led(false);
    wait_for_sync();
    led(true);
    return;
  }
  
  while (Serial.available() > 0) {
    RecvOpcode op = (RecvOpcode) Serial.read();

    switch (op) {
      case RecvOpcode::SYNC:
        Serial.write((uint8) SendOpcode::SYNC);
        return;
      case RecvOpcode::POLL: {
        imu.update();
        imu.getRawAccel(&accelData);
        imu.getRawGyro(&gyroData);

        uint8 button = BOOTSEL ? 1 : 0; // Ensure that it's either 0 or 1

        Serial.write(reinterpret_cast<uint8*>(&accelData.accelX), 2);
        Serial.write(reinterpret_cast<uint8*>(&accelData.accelY), 2);
        Serial.write(reinterpret_cast<uint8*>(&accelData.accelZ), 2);
        Serial.write(reinterpret_cast<uint8*>(&gyroData.gyroX), 2);
        Serial.write(reinterpret_cast<uint8*>(&gyroData.gyroY), 2);
        Serial.write(reinterpret_cast<uint8*>(&gyroData.gyroZ), 2);
        Serial.write(&button, 1);
        break;
      }
      case RecvOpcode::CALIBRATE:
        calibrate();
        break;
    }
  }
}

void loop1() {
  // uncomment if you want to make the button calibrate the sensor
  //if (BOOTSEL) {
  //  delay(2000);
  //  calibrate();
  //}

  delay(10);
}
