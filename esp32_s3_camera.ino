#include <Arduino_GFX_Library.h>
#include <SD_MMC.h>
#include <esp_camera.h>
#include <vector>
#include <algorithm>
#include "img_converters.h"

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

// ---- SD card (Freenove 1-bit SD_MMC) ----
#define SD_CLK 39
#define SD_CMD 38
#define SD_D0  40

bool sdOk = false;
// bool lastSw = HIGH;
uint32_t msgUntil = 0;
const char *msg = "";

#define IMG_W 320
#define IMG_H 240
#define LONG_MS 700
#define ENC_COUNTS_PER_STEP 2   // set to 1 if one click skips a row, 4 if it takes several clicks

enum Mode { LIVE, LIST, VIEW };
Mode mode = LIVE;

std::vector<String> files;
int sel = 0;
int encAccum = 0;
uint16_t *viewBuf = nullptr;

bool swDown = false, longFired = false;
uint32_t swSince = 0;

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

void saveSnapshot(camera_fb_t *fb) {
  if (!sdOk) { msg = "No SD card"; msgUntil = millis() + 1500; return; }

  // Random name, retried if it already exists
  char path[24];
  do {
    snprintf(path, sizeof(path), "/IMG_%08X.jpg", (unsigned)esp_random());
  } while (SD_MMC.exists(path));

  uint8_t *jpg = nullptr;
  size_t jpgLen = 0;
  bool ok = frame2jpg(fb, 85, &jpg, &jpgLen);   // quality 0-100
  if (ok) {
    File f = SD_MMC.open(path, FILE_WRITE);
    if (f) {
      ok = (f.write(jpg, jpgLen) == jpgLen);
      f.close();
    } else {
      ok = false;
    }
  }
  free(jpg);

  if (ok) Serial.printf("Saved %s (%u bytes)\n", path, (unsigned)jpgLen);
  msg = ok ? "Saved!" : "Save failed";
  msgUntil = millis() + 1500;
}

// 0 = nothing, 1 = short press (on release), 2 = long press (fires while held)
int readButton() {
  bool down = digitalRead(ENC_SW) == LOW;
  uint32_t now = millis();
  int ev = 0;
  if (down && !swDown) {
    swDown = true; swSince = now; longFired = false;
  } else if (down && swDown && !longFired && now - swSince >= LONG_MS) {
    longFired = true; ev = 2;
  } else if (!down && swDown) {
    swDown = false;
    if (!longFired && now - swSince > 30) ev = 1;
  }
  return ev;
}

void loadList() {
  files.clear();
  File root = SD_MMC.open("/");
  if (root) {
    File f;
    while ((f = root.openNextFile())) {
      String n = f.name();
      if (!f.isDirectory() && n.startsWith("IMG_") && n.endsWith(".jpg")) files.push_back(n);
      f.close();
    }
    root.close();
  }
  std::sort(files.begin(), files.end());
  if (sel >= (int)files.size()) sel = max(0, (int)files.size() - 1);
}

void drawList() {
  const int rowH = 24, rows = 9;
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_CYAN, RGB565_BLACK);
  gfx->setCursor(4, 4);
  gfx->printf("Images (%d)", (int)files.size());
  if (files.empty()) {
    gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
    gfx->setCursor(8, 40);
    gfx->print(sdOk ? "No images yet" : "No SD card");
    return;
  }
  int top = constrain(sel - rows / 2, 0, max(0, (int)files.size() - rows));
  for (int i = 0; i < rows && top + i < (int)files.size(); i++) {
    int idx = top + i, y = 24 + i * rowH;
    uint16_t bg = (idx == sel) ? RGB565_BLUE : RGB565_BLACK;
    gfx->fillRect(0, y, 320, rowH, bg);
    gfx->setTextColor(RGB565_WHITE, bg);
    gfx->setCursor(8, y + 4);
    gfx->print(files[idx]);
  }
}

bool showImage(const String &name) {
  File f = SD_MMC.open("/" + name, FILE_READ);
  if (!f) return false;
  size_t len = f.size();
  uint8_t *jpg = (uint8_t *)ps_malloc(len);
  if (!jpg) { f.close(); return false; }
  f.read(jpg, len);
  f.close();
  bool ok = jpg2rgb565(jpg, len, (uint8_t *)viewBuf, JPG_SCALE_NONE);
  free(jpg);
  if (ok) {
		//    gfx->draw16bitBeRGBBitmap(0, 0, viewBuf, IMG_W, IMG_H);
		gfx->draw16bitRGBBitmap(0, 0, viewBuf, IMG_W, IMG_H);
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
    gfx->setCursor(2, 230);
    gfx->print(name);
  } else {
    gfx->fillScreen(RGB565_BLACK);
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565_RED, RGB565_BLACK);
    gfx->setCursor(8, 8);
    gfx->print("Can't open image");
  }
  return ok;
}

void setup() {

	Serial.begin(115200);

	SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
	sdOk = SD_MMC.begin("/sdcard", true);   // true = 1-bit mode
	Serial.println(sdOk ? "SD mounted" : "SD mount failed");

	viewBuf = (uint16_t *)ps_malloc(IMG_W * IMG_H * 2);
	
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
  int ev = readButton();

  noInterrupts(); int d = encDelta; encDelta = 0; interrupts();

  switch (mode) {

    case LIVE: {
      if (ev == 2) {
        loadList(); encAccum = 0; mode = LIST; drawList();
        break;
      }
      if (d) {
        brightness = constrain(brightness + (d > 0 ? 1 : -1), -2, 2);
        esp_camera_sensor_get()->set_brightness(esp_camera_sensor_get(), brightness);
      }
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) return;
      if (ev == 1) saveSnapshot(fb);
      gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)fb->buf, fb->width, fb->height);
      esp_camera_fb_return(fb);

      if (millis() < msgUntil) {
        gfx->setTextSize(2);
        gfx->setTextColor(RGB565_GREEN, RGB565_BLACK);
        gfx->setCursor(8, 8);
        gfx->print(msg);
      }
      break;
    }

    case LIST: {
      if (ev == 2) { mode = LIVE; break; }
      int n = files.size();
      encAccum += d;
      int steps = encAccum / ENC_COUNTS_PER_STEP;
      encAccum -= steps * ENC_COUNTS_PER_STEP;
      if (steps && n) {
        sel = ((sel + steps) % n + n) % n;
        drawList();
      }
      if (ev == 1 && n) {
        mode = VIEW; encAccum = 0;
        showImage(files[sel]);
      }
      break;
    }

    case VIEW: {
      if (ev == 2) { mode = LIVE; break; }
      int n = files.size();
      encAccum += d;
      int steps = encAccum / ENC_COUNTS_PER_STEP;
      encAccum -= steps * ENC_COUNTS_PER_STEP;
      if (steps && n) {
        sel = ((sel + steps) % n + n) % n;
        showImage(files[sel]);
      }
      if (ev == 1) { mode = LIST; encAccum = 0; drawList(); }
      break;
    }
  }
}
