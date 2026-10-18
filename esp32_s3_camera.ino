// ESP32-S3 CAM + ST7789 + rotary encoder
// Live view, snapshots, MJPEG/AVI video recording, gallery, effects menu.
//
// Board: ESP32S3 Dev Module, PSRAM: OPI PSRAM
// Library: GFX Library for Arduino

#include <Arduino_GFX_Library.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "SD_MMC.h"
#include <vector>
#include <algorithm>

// ======================= Display (tweak here if colors/orientation are off) =======================
#define TFT_SCK          21
#define TFT_MOSI         47
#define TFT_DC           42
#define TFT_CS           41
#define TFT_RST          14
#define DISPLAY_ROTATION 1       // 1 = landscape 320x240
#define DISPLAY_IPS      false   // inversion flag (the one that fixed the inverted colors)
#define CAM_VFLIP        1
#define CAM_HMIRROR      0
#define FRAME_BE         false    // byte order of decoded frames. If colors look swapped, set false.

// ======================= Encoder / buttons =======================
#define ENC_A   1
#define ENC_B   2
#define ENC_SW  3
#define REC_BTN 0                // K0 (and the onboard BOOT button, same pin)

// ======================= SD card (Freenove 1-bit SD_MMC) =======================
#define SD_CLK 39
#define SD_CMD 38
#define SD_D0  40

// ======================= Camera pins (Freenove ESP32-S3 WROOM) =======================
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// ======================= Settings =======================
#define IMG_W 320
#define IMG_H 240
#define JPEG_QUALITY 12          // 0-63, lower = better quality / bigger files
#define LONG_MS 700
#define ENC_COUNTS_PER_STEP 2    // 1 if one click skips a row, 4 if it takes several clicks
#define JPG_BUF_SZ (96 * 1024)   // max size of one JPEG (image or video frame)
#define AVI_HDR 224              // fixed AVI header size

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, DISPLAY_ROTATION, DISPLAY_IPS, 240, 320);

// ======================= State =======================
enum Mode { LIVE, LIST, VIEW, PLAYBACK, EFFECTS };
Mode mode = LIVE;

struct Btn {
  uint8_t pin;
  bool down;
  bool longFired;
  uint32_t since;
};
Btn encBtn = {ENC_SW, false, false, 0};
Btn kBtn = {REC_BTN, false, false, 0};

int brightness = 0;
volatile int encDelta = 0;
int lastA;
int encAccum = 0;

bool sdOk = false;
char msg[32] = "";
uint32_t msgUntil = 0;

uint16_t *frameBuf = nullptr;   // one decoded RGB565 frame, shared by live / viewer / player
uint8_t *jpgBuf = nullptr;      // one JPEG, shared by viewer / player

std::vector<String> files;
int sel = 0;

// recording
struct IdxEntry { uint32_t off, size; };
std::vector<IdxEntry> recIdx;
File recFile;
bool rec = false;
uint32_t recStartMs = 0, recMoviBytes = 0, recMaxFrame = 0;

// playback
File playFile;
uint32_t playUs = 66667, playNextUs = 0;

// ======================= Helpers =======================
void IRAM_ATTR onEnc() {
  bool a = digitalRead(ENC_A), b = digitalRead(ENC_B);
  if (a != lastA) { encDelta += (a != b) ? 1 : -1; lastA = a; }
}

void notify(const char *m) { strlcpy(msg, m, sizeof(msg)); msgUntil = millis() + 1500; }

void fatal(const char *m) {
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_RED);
  gfx->setCursor(10, 10);
  gfx->println(m);
  Serial.println(m);
  while (true) delay(1000);
}

inline void blit(uint16_t *buf) {
  if (FRAME_BE) gfx->draw16bitBeRGBBitmap(0, 0, buf, IMG_W, IMG_H);
  else gfx->draw16bitRGBBitmap(0, 0, buf, IMG_W, IMG_H);
}

// Pixel value in native byte order (for effects that need to look at the color channels).
inline uint16_t framePx(uint16_t v) { return FRAME_BE ? __builtin_bswap16(v) : v; }

inline bool decodeToFrame(const uint8_t *jpg, size_t len) {
  return jpg2rgb565(jpg, len, (uint8_t *)frameBuf, JPG_SCALE_NONE);
}

bool isVideo(const String &n) { return n.endsWith(".avi"); }

// 0 = nothing, 1 = short press (on release), 2 = long press (fires while held)
int readButton(Btn &b) {
  bool down = digitalRead(b.pin) == LOW;
  uint32_t now = millis();
  int ev = 0;
  if (down && !b.down) {
    b.down = true; b.since = now; b.longFired = false;
  } else if (down && b.down && !b.longFired && now - b.since >= LONG_MS) {
    b.longFired = true; ev = 2;
  } else if (!down && b.down) {
    b.down = false;
    if (!b.longFired && now - b.since > 30) ev = 1;
  }
  return ev;
}

int encSteps(int d) {
  encAccum += d;
  int s = encAccum / ENC_COUNTS_PER_STEP;
  encAccum -= s * ENC_COUNTS_PER_STEP;
  return s;
}

// ======================= Camera =======================
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
  c.pixel_format = PIXFORMAT_JPEG;     // sensor-side JPEG: free to record, decoded for the screen
  c.frame_size = FRAMESIZE_QVGA;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c) == ESP_OK;
}

// ======================= Snapshot =======================
void saveSnapshot(camera_fb_t *fb) {
  if (!sdOk) { notify("No SD card"); return; }
  char path[24];
  do {
    snprintf(path, sizeof(path), "/IMG_%08X.jpg", (unsigned)esp_random());
  } while (SD_MMC.exists(path));

  File f = SD_MMC.open(path, FILE_WRITE);
  bool ok = f && f.write(fb->buf, fb->len) == fb->len;
  if (f) f.close();
  if (ok) Serial.printf("Saved %s (%u bytes)\n", path, (unsigned)fb->len);
  notify(ok ? "Saved!" : "Save failed");
}

// ======================= AVI writing =======================
static void put32(uint8_t *&p, uint32_t v) { memcpy(p, &v, 4); p += 4; }
static void put16(uint8_t *&p, uint16_t v) { memcpy(p, &v, 2); p += 2; }
static void putTag(uint8_t *&p, const char *t) { memcpy(p, t, 4); p += 4; }

// Builds the fixed 224-byte AVI header (RIFF + hdrl + strl + start of movi list).
void buildAviHeader(uint8_t *buf, uint32_t frames, uint32_t rate100, uint32_t moviBytes,
                    uint32_t fileSize, uint32_t maxFrame) {
  uint32_t usPerFrame = 100000000UL / rate100;   // rate100 = fps * 100
  uint8_t *p = buf;
  putTag(p, "RIFF"); put32(p, fileSize - 8); putTag(p, "AVI ");
  putTag(p, "LIST"); put32(p, 192); putTag(p, "hdrl");
  putTag(p, "avih"); put32(p, 56);
  put32(p, usPerFrame); put32(p, maxFrame * rate100 / 100); put32(p, 0); put32(p, 0x10);
  put32(p, frames); put32(p, 0); put32(p, 1); put32(p, maxFrame);
  put32(p, IMG_W); put32(p, IMG_H);
  put32(p, 0); put32(p, 0); put32(p, 0); put32(p, 0);
  putTag(p, "LIST"); put32(p, 116); putTag(p, "strl");
  putTag(p, "strh"); put32(p, 56);
  putTag(p, "vids"); putTag(p, "MJPG"); put32(p, 0);
  put16(p, 0); put16(p, 0); put32(p, 0);
  put32(p, 100); put32(p, rate100); put32(p, 0); put32(p, frames);
  put32(p, maxFrame); put32(p, 0xFFFFFFFF); put32(p, 0);
  put16(p, 0); put16(p, 0); put16(p, IMG_W); put16(p, IMG_H);
  putTag(p, "strf"); put32(p, 40);
  put32(p, 40); put32(p, IMG_W); put32(p, IMG_H); put16(p, 1); put16(p, 24);
  putTag(p, "MJPG"); put32(p, IMG_W * IMG_H * 3);
  put32(p, 0); put32(p, 0); put32(p, 0); put32(p, 0);
  putTag(p, "LIST"); put32(p, 4 + moviBytes); putTag(p, "movi");
}

bool recStart() {
  if (!sdOk) { notify("No SD card"); return false; }
  char path[24];
  do {
    snprintf(path, sizeof(path), "/VID_%08X.avi", (unsigned)esp_random());
  } while (SD_MMC.exists(path));

  recFile = SD_MMC.open(path, FILE_WRITE);
  if (!recFile) { notify("SD open failed"); return false; }
  uint8_t zeros[AVI_HDR] = {0};
  recFile.write(zeros, AVI_HDR);          // placeholder, rewritten when recording stops
  recIdx.clear();
  recIdx.reserve(2048);
  recMoviBytes = 0; recMaxFrame = 0; recStartMs = millis();
  rec = true;
  Serial.printf("Recording %s\n", path);
  return true;
}

bool recWrite(camera_fb_t *fb) {
  uint32_t len = fb->len, pad = len & 1, pos = recFile.position();
  uint8_t h[8];
  memcpy(h, "00dc", 4);
  memcpy(h + 4, &len, 4);
  static const uint8_t zero = 0;
  bool ok = recFile.write(h, 8) == 8 &&
            recFile.write(fb->buf, len) == len &&
            (!pad || recFile.write(&zero, 1) == 1);
  if (!ok) return false;
  recIdx.push_back({pos - (AVI_HDR - 4), len});   // idx1 offsets are relative to the 'movi' tag
  recMoviBytes += 8 + len + pad;
  if (len > recMaxFrame) recMaxFrame = len;
  return true;
}

void recStop() {
  if (!rec) return;
  rec = false;

  uint32_t frames = recIdx.size();
  uint32_t idxBytes = frames * 16;
  uint8_t h[8];
  memcpy(h, "idx1", 4);
  memcpy(h + 4, &idxBytes, 4);
  recFile.write(h, 8);
  for (auto &e : recIdx) {
    uint8_t r[16];
    uint32_t flags = 0x10;                // AVIIF_KEYFRAME
    memcpy(r, "00dc", 4);
    memcpy(r + 4, &flags, 4);
    memcpy(r + 8, &e.off, 4);
    memcpy(r + 12, &e.size, 4);
    recFile.write(r, 16);
  }

  uint32_t fileSize = recFile.position();
  uint32_t ms = millis() - recStartMs;
  if (ms == 0) ms = 1;
  uint32_t rate100 = frames ? (uint32_t)((uint64_t)frames * 100000ULL / ms) : 1500;
  if (rate100 == 0) rate100 = 100;

  uint8_t hdr[AVI_HDR];
  buildAviHeader(hdr, frames, rate100, recMoviBytes, fileSize, recMaxFrame);
  recFile.seek(0);
  recFile.write(hdr, AVI_HDR);
  recFile.close();

  Serial.printf("Video done: %u frames, %.1f fps, %u bytes\n",
                (unsigned)frames, rate100 / 100.0f, (unsigned)fileSize);
  recIdx.clear();
  recIdx.shrink_to_fit();
  notify("Video saved");
}

// ======================= Playback =======================
bool startPlay(const String &name) {
  playFile = SD_MMC.open("/" + name, FILE_READ);
  if (!playFile) return false;
  uint8_t h[AVI_HDR];
  if (playFile.read(h, AVI_HDR) != AVI_HDR || memcmp(h, "RIFF", 4) || memcmp(h + 8, "AVI ", 4)) {
    playFile.close();
    return false;
  }
  uint32_t us;
  memcpy(&us, h + 32, 4);                 // dwMicroSecPerFrame
  if (us < 1000 || us > 1000000) us = 66667;
  playUs = us;
  playNextUs = micros();
  return true;
}

void stopPlay() { if (playFile) playFile.close(); }

// Returns false when the video has ended (or a read fails).
bool playStep() {
  if ((int32_t)(micros() - playNextUs) < 0) return true;   // not time for the next frame yet

  uint8_t h[8];
  if (playFile.read(h, 8) != 8 || memcmp(h, "00dc", 4)) return false;
  uint32_t len;
  memcpy(&len, h + 4, 4);
  if (len == 0 || len > JPG_BUF_SZ) return false;
  if (playFile.read(jpgBuf, len) != len) return false;
  if (len & 1) playFile.read();                            // skip pad byte

  bool late = (int32_t)(micros() - playNextUs) > (int32_t)playUs;
  if (!late && decodeToFrame(jpgBuf, len)) blit(frameBuf); // if behind, skip drawing to stay in real time

  playNextUs += playUs;
  if ((int32_t)(micros() - playNextUs) > 500000) playNextUs = micros();   // far behind: resync
  return true;
}

// ======================= Effects =======================
// Effects run on the decoded frame in the live view only. Snapshots and videos are saved
// unmodified because they come straight from the camera's JPEG.
// To add an effect: write a function like the two below and add a row to effects[].
typedef void (*EffectFn)(uint16_t *px, size_t count);
struct Effect { const char *name; EffectFn fn; };

void fxInvert(uint16_t *px, size_t count) {
  uint32_t *p = (uint32_t *)px;                // two pixels per 32-bit word
  for (size_t i = 0; i < count / 2; i++) p[i] = ~p[i];
}

void fxSwapColors(uint16_t *px, size_t count) {   // red <-> blue
  for (size_t i = 0; i < count; i++) {
    uint16_t v = framePx(px[i]);
    v = ((v & 0x1F) << 11) | (v & 0x07E0) | (v >> 11);
    px[i] = framePx(v);
  }
}

const Effect effects[] = {
  {"None", nullptr},
  {"Invert", fxInvert},
  {"Swap colors", fxSwapColors},
};
const int NUM_EFFECTS = sizeof(effects) / sizeof(effects[0]);
int curEffect = 0;   // active effect
int fxSel = 0;       // highlighted row in the menu

void applyEffect() {
  if (effects[curEffect].fn) effects[curEffect].fn(frameBuf, IMG_W * IMG_H);
}

void drawEffects() {
  const int rowH = 24, rows = 8;
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_CYAN, RGB565_BLACK);
  gfx->setCursor(4, 4);
  gfx->print("Effects");
  int top = constrain(fxSel - rows / 2, 0, max(0, NUM_EFFECTS - rows));
  for (int i = 0; i < rows && top + i < NUM_EFFECTS; i++) {
    int idx = top + i, y = 28 + i * rowH;
    uint16_t bg = (idx == fxSel) ? RGB565_BLUE : RGB565_BLACK;
    gfx->fillRect(0, y, 320, rowH, bg);
    gfx->setTextColor(RGB565_WHITE, bg);
    gfx->setCursor(8, y + 4);
    gfx->print(effects[idx].name);
    if (idx == curEffect) {                     // mark the active effect
      gfx->setTextColor(RGB565_GREEN, bg);
      gfx->setCursor(290, y + 4);
      gfx->print("*");
    }
  }
}

// ======================= Gallery =======================
void loadList() {
  files.clear();
  File root = SD_MMC.open("/");
  if (root) {
    File f;
    while ((f = root.openNextFile())) {
      String n = f.name();
      if (!f.isDirectory() &&
          ((n.startsWith("IMG_") && n.endsWith(".jpg")) || (n.startsWith("VID_") && n.endsWith(".avi"))))
        files.push_back(n);
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
  gfx->printf("Files (%d)", (int)files.size());
  if (files.empty()) {
    gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
    gfx->setCursor(8, 40);
    gfx->print(sdOk ? "Nothing saved yet" : "No SD card");
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
  bool ok = false;
  size_t len = 0;
  if (f) {
    len = f.size();
    ok = len > 0 && len <= JPG_BUF_SZ && f.read(jpgBuf, len) == len;
    f.close();
  }
  ok = ok && decodeToFrame(jpgBuf, len);
  if (ok) {
    blit(frameBuf);
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

// Next still image in the list (skips videos), wrapping around.
int nextImage(int from, int dir) {
  int n = files.size();
  for (int i = 1; i <= n; i++) {
    int k = ((from + dir * i) % n + n) % n;
    if (!isVideo(files[k])) return k;
  }
  return from;
}

void openSelected() {
  if (files.empty()) return;
  encAccum = 0;
  if (isVideo(files[sel])) {
    if (startPlay(files[sel])) {
      mode = PLAYBACK;
      gfx->fillScreen(RGB565_BLACK);
    } else {
      gfx->setTextSize(2);
      gfx->setTextColor(RGB565_RED, RGB565_BLACK);
      gfx->setCursor(8, 226);
      gfx->print("Can't play video");
    }
  } else {
    mode = VIEW;
    showImage(files[sel]);
  }
}

// ======================= Live view =======================
void drawOverlay() {
  if (rec) {
    uint32_t s = (millis() - recStartMs) / 1000;
    gfx->fillCircle(308, 12, 6, RGB565_RED);
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
    gfx->setCursor(236, 5);
    gfx->printf("%02u:%02u", (unsigned)(s / 60), (unsigned)(s % 60));
  }
  if (millis() < msgUntil) {
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565_GREEN, RGB565_BLACK);
    gfx->setCursor(8, 8);
    gfx->print(msg);
  }
  if (curEffect) {                                 // small label while an effect is active
    gfx->setTextSize(1);
    gfx->setTextColor(RGB565_YELLOW, RGB565_BLACK);
    gfx->setCursor(2, 230);
    gfx->print(effects[curEffect].name);
  }
}

// ev = encoder button event, kev = K0 button event, d = encoder rotation
void liveStep(int ev, int kev, int d) {
  if (ev == 2) {                                   // encoder long press -> gallery
    recStop();
    loadList(); encAccum = 0; mode = LIST; drawList();
    return;
  }
  if (kev == 2) {                                  // K0 long press -> effects menu
    recStop();
    fxSel = curEffect; encAccum = 0; mode = EFFECTS; drawEffects();
    return;
  }
  if (kev == 1) { if (rec) recStop(); else recStart(); }   // K0 short press -> record on/off

  if (d) {
    brightness = constrain(brightness + (d > 0 ? 1 : -1), -2, 2);
    esp_camera_sensor_get()->set_brightness(esp_camera_sensor_get(), brightness);
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  if (rec && !recWrite(fb)) { recStop(); notify("Write failed"); }
  if (ev == 1) saveSnapshot(fb);                   // encoder short press -> snapshot
  bool ok = decodeToFrame(fb->buf, fb->len);
  esp_camera_fb_return(fb);
  if (ok) {
    applyEffect();
    blit(frameBuf);
  }
  drawOverlay();
}

// ======================= Arduino entry points =======================
void setup() {
  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(REC_BTN, INPUT_PULLUP);
  lastA = digitalRead(ENC_A);
  attachInterrupt(ENC_A, onEnc, CHANGE);

  gfx->begin(40000000);
  gfx->fillScreen(RGB565_BLACK);

  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
  sdOk = SD_MMC.begin("/sdcard", true);            // true = 1-bit mode
  Serial.println(sdOk ? "SD mounted" : "SD mount failed");

  frameBuf = (uint16_t *)ps_malloc(IMG_W * IMG_H * 2);
  jpgBuf = (uint8_t *)ps_malloc(JPG_BUF_SZ);
  if (!frameBuf || !jpgBuf) fatal("Out of PSRAM");

  if (!initCamera()) fatal("Camera init failed");
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, CAM_VFLIP);
  s->set_hmirror(s, CAM_HMIRROR);
}

void loop() {
  int ev = readButton(encBtn);
  int kev = readButton(kBtn);
  noInterrupts(); int d = encDelta; encDelta = 0; interrupts();

  switch (mode) {

    case LIVE:
      liveStep(ev, kev, d);
      break;

    case LIST: {
      if (ev == 2) { mode = LIVE; break; }
      int n = files.size();
      int steps = encSteps(d);
      if (steps && n) {
        sel = ((sel + steps) % n + n) % n;
        drawList();
      }
      if (ev == 1) openSelected();
      break;
    }

    case VIEW: {
      if (ev == 2) { mode = LIVE; break; }
      int steps = encSteps(d);
      if (steps && !files.empty()) {
        int old = sel;
        for (int i = 0; i < abs(steps); i++) sel = nextImage(sel, steps > 0 ? 1 : -1);
        if (sel != old) showImage(files[sel]);
      }
      if (ev == 1) { mode = LIST; encAccum = 0; drawList(); }
      break;
    }

    case PLAYBACK:
      if (ev == 2) { stopPlay(); mode = LIVE; break; }
      if (ev == 1) { stopPlay(); mode = LIST; drawList(); break; }
      if (!playStep()) { stopPlay(); mode = LIST; drawList(); }
      break;

    case EFFECTS: {
      if (ev == 2 || kev == 2) { mode = LIVE; break; }          // cancel, keep current effect
      int steps = encSteps(d);
      if (steps) {
        fxSel = ((fxSel + steps) % NUM_EFFECTS + NUM_EFFECTS) % NUM_EFFECTS;
        drawEffects();
      }
      if (ev == 1) { curEffect = fxSel; mode = LIVE; }          // select, return to live view
      break;
    }
  }
}
