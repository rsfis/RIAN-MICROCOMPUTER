#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7796     _panel_instance;
  lgfx::Bus_SPI          _bus_instance;
  lgfx::Light_PWM        _light_instance;

public:
  LGFX(void)
  {
    { // Configuração SPI
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
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

    { // Configuração do painel ST7796
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
      cfg.invert     = false;
      cfg.rgb_order  = false;   // ⚠️ BGR verdadeiro do módulo QD3525/MSP3525
      cfg.invert = true;
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

void setup()
{
  tft.init();
  tft.setRotation(1);
  tft.setBrightness(255);

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(20, 20);
  tft.println("Hello ESP32-S3 + ST7796!");
  delay(1000);

  tft.fillScreen(0xF800);  // Vermelho puro
  delay(600);
  tft.fillScreen(0x07E0);  // Verde puro
  delay(600);
  tft.fillScreen(0x001F);  // Azul puro
  delay(600);
  tft.fillScreen(0xFFFF);  // Branco
  delay(600);
  tft.fillScreen(0x0000);  // Preto
  tft.drawRect(10, 10, 50, 50, 0xF800);
  tft.setColor(0xFF0000U);
  tft.fillCircle ( 40, 80, 20);
  tft.fillEllipse( 80, 40, 10, 20);
  tft.fillArc    ( 80, 80, 20, 10, 0, 90);
  tft.fillTriangle(80, 80, 60, 80, 80, 60);
  tft.setColor(0x0000FFU);
  tft.drawCircle ( 40, 80, 20);
  tft.drawEllipse( 80, 40, 10, 20);
  tft.drawArc    ( 80, 80, 20, 10, 0, 90);
  tft.drawTriangle(60, 80, 80, 80, 80, 60);
  tft.setColor(0x00FF00U);
  tft.drawBezier( 60, 80, 80, 80, 80, 60);
  tft.drawBezier( 60, 80, 80, 20, 20, 80, 80, 60);
  tft.drawLine(0, 1, 39, 40, 0x001F);
}

void loop()
{
  static int x = 0;
  tft.fillCircle(160 + sin(x * 0.1) * 100, 240, 10, TFT_GREEN);
  delay(30);
  x++;
}