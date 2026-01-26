#include <LovyanGFX.hpp>
#include <SD.h>
#include "Geometos.h"

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
      cfg.freq_read  = 16000000;
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
      cfg.invert     = true;
      cfg.rgb_order  = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;

      _panel_instance.config(cfg);
    }

    { // Backlight
      auto cfg = _light_instance.config();
      cfg.pin_bl = 38;
      cfg.invert = false;
      cfg.freq   = 5000;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    setPanel(&_panel_instance);
  }
};

LGFX tft;
LGFX_Sprite frame(&tft); // TUDO DEVE SER DESENHADO AQUI DENTRO
#define SD_CS 4

void setup()
{
    tft.init();
    tft.setRotation(1);
    tft.setBrightness(255);

    if (!SD.begin(SD_CS)) {
      Serial.println("Falha no SD!");
      while (1);
    }
    Serial.println("SD OK!");

    frame.setPsram(true);
    if (!frame.createSprite(tft.width(), tft.height())) {
      Serial.println("Falha ao criar frame!");
      while (1);
    }

    frame.setFont(&Geometos14pt);
}

void loop(){
  frame.fillScreen(TFT_BLACK);
  frame.setTextColor(TFT_RED);
  frame.setCursor(10, 50);
  frame.print("OLÁ LOVYANGFX!");
  frame.pushSprite(0, 0);
}