#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>

#include <LovyanGFX.hpp>
#include <SD.h>

// ===== Lua headers =====
extern "C" {
  #include "lua.h"
  #include "lauxlib.h"
  #include "lualib.h"
}

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
LGFX_Sprite frame(&tft);

// ===== Config =====
#define LUA_HEAP_LIMIT (7 * 1024 * 1024)
#define LUA_TASK_STACK 30000

#define SD_CS 4

static size_t luaHeapUsed = 0;
volatile bool luaRequestExit = false;

// ===== Alocador Lua usando PSRAM =====
void* lua_psram_alloc(void* ud, void* ptr, size_t osize, size_t nsize) {
  (void)ud;

  if (nsize == 0) {
    if (ptr) {
      luaHeapUsed -= osize;
      heap_caps_free(ptr);
    }
    return NULL;
  }

  if (luaHeapUsed - osize + nsize > LUA_HEAP_LIMIT) {
    Serial.println("ERR: Lua HEAP is OVER");
    return NULL;
  }

  void* newptr = heap_caps_realloc(ptr, nsize, MALLOC_CAP_SPIRAM);
  if (newptr) {
    luaHeapUsed = luaHeapUsed - osize + nsize;
  }

  return newptr;
}

// ===== EXPORTED FUNCTIONS =====
//int l_teste(lua_State* L) {
//  int numero = luaL_checkinteger(L, 1);
//  Serial.printf("[Lua] Teste: %d (core %d)\n", numero, xPortGetCoreID());
//  return 0;
//}

int l_endProgram(lua_State* L) {
  Serial.println("[Lua] Requesting VM shutdown");
  luaRequestExit = true;
  return 0;
}

// ===== RESGISTER EXPORTED FUNCTIONS =====
void registerApi(lua_State* L) {
  //lua_register(L, "teste", l_teste);
  lua_register(L, "endProgram", l_endProgram);
}

// ===== Task Lua (Core 1) =====
void taskLuaApp(void* arg) {
  Serial.printf("== Lua Task Started at Core: %d ==\n", xPortGetCoreID());

  luaHeapUsed = 0;

  lua_State* L = lua_newstate(lua_psram_alloc, NULL, esp_random());
  if (!L) {
    Serial.println("ERR: Couldn't create LUA VM");
    Serial.println("SOL: Deleting Task");
    vTaskDelete(NULL);
    return;
  }

  luaL_openlibs(L);
  registerApi(L);

  //const char* script =
  //  "print('Lua OK no Core 1')\n"
  //  "for i=1,5 do\n"
  //  "  print('[LUA] Teste: ', i)\n"
  //  "end\n"
  //  "endProgram()";

  File f = SD.open("/app.lua");
  if (!f) {
    Serial.println("ERR: app.lua not found");
    lua_close(L);
    vTaskDelete(NULL);
    return;
  }
  size_t size = f.size();
  char* buffer = (char*)malloc(size + 1);
  if (!buffer) {
    Serial.println("ERR: No RAM avaliable");
    f.close();
    return;
  }else{
    f.readBytes(buffer, size);
    buffer[size] = '\0';

    f.close();

    const char* script = buffer;
    free(buffer);

    if (luaL_dostring(L, script) != LUA_OK) {
      Serial.printf("=== LUA VM INTERPRETER ERROR: %s\n", lua_tostring(L, -1));
    }

    //LOOP
    while (!luaRequestExit) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    luaRequestExit = false;
    lua_close(L);
    Serial.printf("[LUA] VM Shutdown\n");
    vTaskDelete(NULL);
  }
}

// ===== Kernel (Core 0) =====
void setup() {
  esp_task_wdt_deinit();

  Serial.begin(115200);

  Serial.printf("====== FiscionOS ======\n");

  Serial.printf("Starting Kernel at core %d\n", xPortGetCoreID());

  if (!psramFound()) {
    Serial.println("PSRAM NOT FOUND!");
    return;
  }

  Serial.printf("======= MEMORY =======\n");
  Serial.printf("PSRAM total: %u bytes\n", ESP.getPsramSize());
  Serial.printf("PSRAM free: %u bytes\n", ESP.getFreePsram());

  Serial.printf("======================\n");

  Serial.printf("=== Starting Screen ===\n");
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setColorDepth(16);
  tft.setBrightness(255);
  tft.setSwapBytes(true);

  Serial.printf("===== SD Start =====\n");
  if (!SD.begin(SD_CS)) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x001 - Couldn't initiate SD Card module.");
    while (1);
  }

  Serial.printf("== Creating Frame ==\n");
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

  // ONLY IF APP PRESENT:
  Serial.printf("== Starting LUA VM ==\n");

  xTaskCreatePinnedToCore(
    taskLuaApp,
    "LUA_APP",
    LUA_TASK_STACK,
    NULL,
    2,
    NULL,
    1
  );
  delay(3000);
  xTaskCreatePinnedToCore(
    taskLuaApp,
    "LUA_APP2",
    LUA_TASK_STACK,
    NULL,
    2,
    NULL,
    1
  );
}

void loop() {
  // KERNEL
}