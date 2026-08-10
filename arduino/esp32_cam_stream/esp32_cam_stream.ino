#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"           // Required to disable brownout detector
#include "soc/rtc_cntl_reg.h"  // Required to disable brownout detector

// camera model
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// WiFi credentials
const char* ssid = "SLT-Fiber-Uqh95-2.4G";
const char* password = "JeRT7Wa7";

void startCameraServer();
void setupLedFlash(int pin);

void setup() {
  // Disable brownout detector to prevent power-drop resets and black screens
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(1000);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("Starting ESP32-CAM...");

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

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // Camera settings
  config.xclk_freq_hz = 20000000; // Increased to standard 20MHz for OV2640 sync
  config.pixel_format = PIXFORMAT_JPEG;

  // Optimized buffer allocation dynamically based on hardware
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;     
    config.jpeg_quality = 10; // Lower number = higher quality
    config.fb_count = 2;      // Double buffering for smooth streaming
    config.grab_mode = CAMERA_GRAB_LATEST; 
    config.fb_location = CAMERA_FB_IN_PSRAM;
    Serial.println("PSRAM found. Optimized for smooth streaming.");
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_DRAM;
    Serial.println("PSRAM not found.");
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // Camera init
  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  Serial.println("Camera initialized");

  sensor_t *s = esp_camera_sensor_get();

  if (s == NULL) {
    Serial.println("Camera sensor not found");
    return;
  }

  // Sensor adjustments
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_quality(s, 10);
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  
  // ADD THESE TWO LINES TO FLIP THE IMAGE
  s->set_vflip(s, 1);    // Flips the image vertically
  s->set_hmirror(s, 1);  // Mirrors the image so left and right stay correct

#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  delay(10000);
}