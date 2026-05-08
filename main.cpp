#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

#define UNIT_ID 4   // change to 1, 2, 3, or 4

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

#define RS485_TX 17
#define RS485_RX 16
#define RS485_EN 4

VL53L1X lidar;

void sendRS485(String msg) {
  digitalWrite(RS485_EN, HIGH);   // transmit mode
  delay(2);

  Serial2.println(msg);
  Serial2.flush();

  delay(2);
  digitalWrite(RS485_EN, LOW);    // receive mode
}

void setup() {
  Serial.begin(115200);  // USB debug
  Serial2.begin(115200, SERIAL_8N1, RS485_RX, RS485_TX);

  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(500);

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Start LiDAR
  lidar.setTimeout(500);

  if (lidar.init()) {
    lidar.setDistanceMode(VL53L1X::Long);
    lidar.setMeasurementTimingBudget(50000);
    lidar.startContinuous(100);
    Serial.println("LIDAR OK");
  } else {
    Serial.println("LIDAR INIT FAILED");
  }

  sendRS485("UNIT:" + String(UNIT_ID) + ",BOOT:OK");
}

void loop() {
  uint16_t mm = lidar.read();

  String lidarText;
  if (!lidar.timeoutOccurred()) {
    lidarText = String(mm / 10);
  } else {
    lidarText = "TIMEOUT";
  }

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  byte error = Wire.endTransmission(false);

  if (error != 0) {
    sendRS485("UNIT:" + String(UNIT_ID) + ",LIDAR_CM:" + lidarText + ",MPU:I2C_FAIL");
    delay(150);
    return;
  }

  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() < 14) {
    sendRS485("UNIT:" + String(UNIT_ID) + ",LIDAR_CM:" + lidarText + ",MPU:READ_FAIL");
    delay(150);
    return;
  }

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read(); // skip temp

  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  String msg = "UNIT:" + String(UNIT_ID) +
               ",LIDAR_CM:" + lidarText +
               ",AX:" + String(ax / 16384.0, 3) +
               ",AY:" + String(ay / 16384.0, 3) +
               ",AZ:" + String(az / 16384.0, 3) +
               ",GX:" + String(gx / 131.0, 3) +
               ",GY:" + String(gy / 131.0, 3) +
               ",GZ:" + String(gz / 131.0, 3);

  Serial.println(msg);   // USB debug
  sendRS485(msg);        // RS485 to Pi

  delay(150);
}