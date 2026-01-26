#include <LovyanGFX.hpp>
#include <SD.h>

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;

public:
  LGFX(void)
  {
    { // SPI
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 80000000;
      cfg.freq_read  = 20000000;
      cfg.spi_3wire  = false;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc   = 9;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    { // Painel ST7796
      auto cfg = _panel_instance.config();
      cfg.pin_cs  = 10;
      cfg.pin_rst = 11;
      cfg.pin_busy = -1;

      cfg.memory_width  = 320;
      cfg.memory_height = 480;
      cfg.panel_width   = 320;
      cfg.panel_height  = 480;

      cfg.offset_x = 0;
      cfg.offset_y = 0;

      cfg.readable   = false;
      cfg.invert     = true;    // ⚠️ deixar true
      cfg.rgb_order  = false;   // ⚠️ força modo BGR (a maioria dessas telas usa)
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;

      _panel_instance.config(cfg);
    }

    { // Backlight
      auto cfg = _light_instance.config();
      cfg.pin_bl = 38;
      cfg.invert = false;
      cfg.freq   = 10000;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    setPanel(&_panel_instance);
  }
};

LGFX tft;
#define SD_CS 4
LGFX_Sprite frame(&tft);

struct Sprite {
  LGFX_Sprite spr;

  Sprite() : spr(&tft) {
    spr.setPsram(true);
    spr.fillSprite(TFT_TRANSPARENT);
    spr.setColorDepth(16);
  }

  bool load(const char* caminho, int largura, int altura) {
    if (!spr.createSprite(largura, altura)) {
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1.5);
      tft.setCursor(5, 5);
      tft.printf("ERR 0x003 - Sprite creation failed at %s", caminho);
      while (1);
    }

    File f = SD.open(caminho);
    if (!f) {
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1.5);
      tft.setCursor(5, 5);
      tft.printf("ERR 0x004 - Couldn't open file at %s", caminho);
      while (1);
    }

    size_t tamanho = f.size();
    uint8_t* buffer = (uint8_t*)malloc(tamanho);
    if (!buffer) {
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1.5);
      tft.setCursor(5, 5);
      tft.printf("ERR 0x005 - Malloc failed at adress %s", *buffer);
      f.close();
      while (1);
    }

    f.read(buffer, tamanho);
    f.close();
    spr.drawPng(buffer, tamanho, 0, 0);
    free(buffer);
    return true;
  }

  void draw(int x, int y) {
    spr.pushSprite(&frame, x, y, 0x0000);
  }
};

int didix = 10;
int didiy = 0;
Sprite* bg;
Sprite* didi;

unsigned long lastMillis = 0;
unsigned long lastFrameTime = 0;
int frames = 0;
float fps = 0.0f;
float deltaTime = 0.0f;

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setColorDepth(16);
  tft.setBrightness(255);
  tft.setSwapBytes(true);

  if (!SD.begin(SD_CS)) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x001 - Couldn't initiate SD Card module.");
    while (1);
  }

  frame.setPsram(true);
  frame.setColorDepth(16);
  if (!frame.createSprite(tft.width(), tft.height())) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x002 - Couldn't create drawing frame.");
    while (1);
  }

  // teste de cores puras
  tft.fillScreen(tft.color565(255, 0, 0));
  delay(1000);
  tft.fillScreen(tft.color565(0, 255, 0));
  delay(1000);
  tft.fillScreen(tft.color565(0, 0, 255));
  delay(1000);
  tft.fillScreen(TFT_BLACK);

  bg = new Sprite();
  didi = new Sprite();
  bg->load("/bg.png", 480, 320);
  didi->load("/didi.png", 259, 194);

  lastFrameTime = millis();
  lastMillis = lastFrameTime;
}

void loop() {
  unsigned long now = millis();
  deltaTime = (now - lastFrameTime) / 1000.0f;
  lastFrameTime = now;

  if (didix > tft.width()) didix = -259;
  if (didiy > tft.height()) didiy = -194;

  // DRAW EVERYTHING HERE
  bg->draw(0, 0);
  didi->draw(didix, didiy);

  int analogx = analogRead(19);
  int analogy = analogRead(20);
  
  int deadzone = 60;
  int center = 2048;
  int maxSpeed = 15;

  int dx = analogx - center;
  if (abs(dx) > deadzone) {
    float intensity = (float)(abs(dx) - deadzone) / (float)(2048 - deadzone);
    intensity = pow(intensity, 1.5);
    int speed = intensity * maxSpeed;
    didix += (dx > 0 ? speed : -speed);
  }

  int dy = analogy - center;
  if (abs(dy) > deadzone) {
    float intensity = (float)(abs(dy) - deadzone) / (float)(2048 - deadzone);
    intensity = pow(intensity, 1.5);  // 1.0 = linear, >1 = mais suave
    int speed = intensity * maxSpeed;
    didiy += (dy > 0 ? speed : -speed);
  }

  // FPS
  frames++;
  if (now - lastMillis >= 1000) {
    fps = frames * 1000.0 / (now - lastMillis);
    frames = 0;
    lastMillis = now;
  }

  // TEXT
  frame.setTextColor(TFT_WHITE, TFT_BLACK);
  frame.setTextSize(1);
  frame.setCursor(5, 5);
  frame.printf("FPS: %.1f", fps);
  frame.setCursor(5, 20);
  frame.printf("deltaTime: %.3f s", deltaTime);

  frame.pushSprite(0, 0);
}