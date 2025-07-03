#include "esp_camera.h"
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// ... configuración de pines de la cámara ...
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

#define FILTER_N 10

int16_t ax_buf[FILTER_N], ay_buf[FILTER_N], az_buf[FILTER_N];
int16_t gx_buf[FILTER_N], gy_buf[FILTER_N], gz_buf[FILTER_N];
int filter_idx = 0;
bool filter_full = false;

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
  // Inicializa la cámara (igual que antes)
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

  // Inicializa la cámara
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Error al inicializar la cámara");
    return;
  }
  // Inicializa el MPU6050
  Wire.begin(6, 7); // Cambia los pines si usas otros
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 no encontrado!");
    while (1);
  }
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Añade a los buffers
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

  // Calcula el promedio
  int16_t ax_f = avgBuffer(ax_buf);
  int16_t ay_f = avgBuffer(ay_buf);
  int16_t az_f = avgBuffer(az_buf);
  int16_t gx_f = avgBuffer(gx_buf);
  int16_t gy_f = avgBuffer(gy_buf);
  int16_t gz_f = avgBuffer(gz_buf);

  // Envía los datos filtrados
  Serial.print("ACC:");
  Serial.print(ax_f); Serial.print(",");
  Serial.print(ay_f); Serial.print(",");
  Serial.print(az_f); Serial.print(";");
  Serial.print("GYRO:");
  Serial.print(gx_f); Serial.print(",");
  Serial.print(gy_f); Serial.print(",");
  Serial.print(gz_f); Serial.println(";");

  // Envía la imagen como antes
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

  delay(50); // Ajusta para la tasa de cuadros
}