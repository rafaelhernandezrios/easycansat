#include "esp_camera.h"
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Pines cámara XIAO ESP32S3 Sense
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13
// Por defecto Serial es el USB (Serial0)
#define ACP_RX_PIN 44   // GPIO44
#define ACP_TX_PIN 43   // GPIO43

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
  Serial1.begin(115200, SERIAL_8N1, ACP_RX_PIN, ACP_TX_PIN);
  // Inicializa cámara
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 12;
  config.fb_count = 1;
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Error al inicializar la cámara");
    while (1);
  }
  // Inicializa MPU6050
  Wire.begin(); // Usa los pines correctos para tu placa
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

  // Envía los datos filtrados
  Serial.print("ACC:");
  Serial.print((int)ax_f); Serial.print(",");
  Serial.print((int)ay_f); Serial.print(",");
  Serial.print((int)az_f); Serial.print(";");
  Serial.print("GYRO:");
  Serial.print((int)gx_f); Serial.print(",");
  Serial.print((int)gy_f); Serial.print(",");
  Serial.print((int)gz_f); Serial.println(";");
  Serial1.print("ACC:");
  Serial1.print((int)ax_f); Serial1.print(",");
  Serial1.print((int)ay_f); Serial1.print(",");
  Serial1.print((int)az_f); Serial1.print(";");
  Serial1.print("GYRO:");
  Serial1.print((int)gx_f); Serial1.print(",");
  Serial1.print((int)gy_f); Serial1.print(",");
  Serial1.print((int)gz_f); Serial1.println(";");
  // Envía la imagen
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Error al capturar imagen");
    return;
  }
  Serial.write(0xAA);
  Serial.write((fb->len >> 8) & 0xFF);
  Serial.write(fb->len & 0xFF);
  Serial.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  delay(100); // Ajusta para la tasa de cuadros
} 