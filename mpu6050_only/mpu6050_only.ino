#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

#define FILTER_N 10
int16_t ax_buf[FILTER_N], ay_buf[FILTER_N], az_buf[FILTER_N];
int16_t gx_buf[FILTER_N], gy_buf[FILTER_N], gz_buf[FILTER_N];
int filter_idx = 0;
bool filter_full = false;

float ax_f = 0, ay_f = 0, az_f = 0, gx_f = 0, gy_f = 0, gz_f = 0;
float alpha = 0.2; // Filtro exponencial

void addToBuffer(int16_t* buf, int16_t val) {
  buf[filter_idx] = val;
}

int16_t avgBuffer(int16_t* buf) {
  int32_t sum = 0;
  int n = filter_full ? FILTER_N : filter_idx;
  if (n == 0) return 0;
  for (int i = 0; i < n; i++) sum += buf[i];
  return sum / n;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(6, 7);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 no encontrado!");
    while (1);
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Media móvil
  addToBuffer(ax_buf, ax);
  addToBuffer(ay_buf, ay);
  addToBuffer(az_buf, az);
  addToBuffer(gx_buf, gx);
  addToBuffer(gy_buf, gy);
  addToBuffer(gz_buf, gz);
  filter_idx++;
  if (filter_idx >= FILTER_N) {
    filter_idx = 0;
    filter_full = true;
  }
  int16_t ax_avg = avgBuffer(ax_buf);
  int16_t ay_avg = avgBuffer(ay_buf);
  int16_t az_avg = avgBuffer(az_buf);
  int16_t gx_avg = avgBuffer(gx_buf);
  int16_t gy_avg = avgBuffer(gy_buf);
  int16_t gz_avg = avgBuffer(gz_buf);

  // Filtro exponencial
  ax_f = alpha * ax_avg + (1 - alpha) * ax_f;
  ay_f = alpha * ay_avg + (1 - alpha) * ay_f;
  az_f = alpha * az_avg + (1 - alpha) * az_f;
  gx_f = alpha * gx_avg + (1 - alpha) * gx_f;
  gy_f = alpha * gy_avg + (1 - alpha) * gy_f;
  gz_f = alpha * gz_avg + (1 - alpha) * gz_f;

  Serial.print("ACC:");
  Serial.print((int)ax_f); Serial.print(",");
  Serial.print((int)ay_f); Serial.print(",");
  Serial.print((int)az_f); Serial.print(";");
  Serial.print("GYRO:");
  Serial.print((int)gx_f); Serial.print(",");
  Serial.print((int)gy_f); Serial.print(",");
  Serial.print((int)gz_f); Serial.println(";");

  delay(50);
} 