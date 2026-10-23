#include <Wire.h>

const int FSR_PIN = 1;          // 보드 A0 = GPIO1
const uint8_t MPU_ADDR = 0x68;

const unsigned long SAMPLE_INTERVAL = 20;  // 20ms = 50Hz
unsigned long lastSample = 0;
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);

  // IMU: SDA=A4(GPIO11), SCL=A5(GPIO12)
  Wire.begin(11, 12);

  // IMU 깨우기
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(500);

  startTime = millis();

  // CSV 헤더
  Serial.println("time_ms,fsr,ax,ay,az,gx,gy,gz");
}

void loop() {

  unsigned long now = millis();

  if (now - lastSample >= SAMPLE_INTERVAL) {
    lastSample += SAMPLE_INTERVAL;

    // FSR 읽기
    int fsr = analogRead(FSR_PIN);

    // IMU 읽기
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, (uint8_t)14);

    if (Wire.available() == 14) {

      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();

      // 온도값 2바이트 건너뛰기
      Wire.read();
      Wire.read();

      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      // CSV 형식으로 출력
      Serial.print(now - startTime);
      Serial.print(",");
      Serial.print(fsr);
      Serial.print(",");
      Serial.print(ax);
      Serial.print(",");
      Serial.print(ay);
      Serial.print(",");
      Serial.print(az);
      Serial.print(",");
      Serial.print(gx);
      Serial.print(",");
      Serial.print(gy);
      Serial.print(",");
      Serial.println(gz);
    }
  }
}