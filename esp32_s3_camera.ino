#include <Arduino_GFX_Library.h>
#include "esp_camera.h"

// ---- Display pins ----
#define TFT_SCK 21
#define TFT_MOSI 47
#define TFT_DC 42
#define TFT_CS 41
#define TFT_RST 14
#define TFT_BLK -1      // BLK wired to 3V3

// ---- Encoder pins ----
#define ENC_A 1
#define ENC_B 2
#define ENC_SW 3

// ---- Freenove ESP32-S3 WROOM camera pins ----
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED);
// rotation 1 = landscape (320x240). Change panel size / IPS flag if colors or offsets look wrong.
Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 1 /*rotation*/, false /*IPS*/, 240, 320);

int brightness = 0;
volatile int encDelta = 0;
int lastA;

void IRAM_ATTR onEnc() {
  bool a = digitalRead(ENC_A), b = digitalRead(ENC_B);
  if (a != lastA) { encDelta += (a != b) ? 1 : -1; lastA = a; }
}

bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;   // raw pixels, no JPEG decode needed
  c.frame_size = FRAMESIZE_QVGA;       // 320x240 matches the landscape screen
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c) == ESP_OK;
}

void setup() {
  Serial.begin(115200);
	//  pinMode(TFT_BLK, OUTPUT); digitalWrite(TFT_BLK, HIGH);
  pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP); pinMode(ENC_SW, INPUT_PULLUP);
  lastA = digitalRead(ENC_A);
  attachInterrupt(ENC_A, onEnc, CHANGE);

  gfx->begin(40000000);
  gfx->fillScreen(RGB565_BLACK);

  if (!initCamera()) {
    gfx->setCursor(10, 10); gfx->setTextColor(RGB565_RED); gfx->println("Camera init failed");
    while (true) delay(1000);
  }
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);      // flip these if the image is upside down / mirrored
  s->set_hmirror(s, 0);
}

void loop() {
  // Encoder -> camera brightness (-2..2), push -> reset
  int d = encDelta; encDelta = 0;
  if (d) {
    brightness = constrain(brightness + (d > 0 ? 1 : -1), -2, 2);
    esp_camera_sensor_get()->set_brightness(esp_camera_sensor_get(), brightness);
  }
  if (digitalRead(ENC_SW) == LOW) {
    brightness = 0;
    esp_camera_sensor_get()->set_brightness(esp_camera_sensor_get(), 0);
    delay(200);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)fb->buf, fb->width, fb->height);
  esp_camera_fb_return(fb);
}
