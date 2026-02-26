#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>

#include <LovyanGFX.hpp>
#include <SD.h>

#define LUA_SPRITE_DEFINITION "Display.Sprite"
#define LUA_CLEAR_SCREEN_DEFINITION "Display_ClearScreen"
#define LUA_UPDATE_SCREEN_DEFINITION "Display_UpdateScreen"
struct Sprite;

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

// KERNEL FUNCTIONS
struct Sprite {
  LGFX_Sprite spr;

  Sprite() : spr(&tft) {
    spr.fillSprite(TFT_TRANSPARENT);
    spr.setColorDepth(16);
  }

  bool load(const char* caminho, int largura, int altura) {
    spr.setPsram(true);
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

int l_clearScreen(lua_State* L) {
  frame.fillScreen(TFT_BLACK);
  return 0;
}

int l_updateScreen(lua_State* L){
  frame.pushSprite(0, 0);
  return 0;
}

// Sprites
static Sprite* checkSprite(lua_State* L, int index) {
  return (Sprite*)luaL_checkudata(L, index, LUA_SPRITE_DEFINITION);
}

int l_sprite_free(lua_State* L) {
  Sprite* spr = checkSprite(L, 1);
  spr->spr.deleteSprite();   // libera sprite interno
  return 0;
}

int l_sprite_constructor(lua_State* L) {
  const char* path = luaL_checkstring(L, 2);
  int w = luaL_checkinteger(L, 3);
  int h = luaL_checkinteger(L, 4);

  Sprite* spr = (Sprite*)lua_newuserdata(L, sizeof(Sprite));
  new (spr) Sprite();

  if (!spr->load(path, w, h)) {
    return luaL_error(L, "Sprite load failed");
  }

  luaL_getmetatable(L, LUA_SPRITE_DEFINITION);
  lua_setmetatable(L, -2);

  return 1;
}

int l_sprite_draw(lua_State* L) {
  Sprite* spr = checkSprite(L, 1);
  int x = luaL_checkinteger(L, 2);
  int y = luaL_checkinteger(L, 3);

  spr->draw(x, y);
  return 0;
}

int l_sprite_gc(lua_State* L) {
  Sprite* spr = checkSprite(L, 1);
  spr->~Sprite();
  return 0;
}

void registerSprite(lua_State* L) {

  // cria metatable do userdata
  luaL_newmetatable(L, LUA_SPRITE_DEFINITION);

  // __gc
  lua_pushcfunction(L, l_sprite_gc);
  lua_setfield(L, -2, "__gc");

  // métodos
  lua_newtable(L);

  lua_pushcfunction(L, l_sprite_draw);
  lua_setfield(L, -2, "draw");

  lua_pushcfunction(L, l_sprite_free);
  lua_setfield(L, -2, "free");

  lua_setfield(L, -2, "__index");

  lua_pop(L, 1);

  // ===== Tabela global Sprite =====
  lua_newtable(L);

  // metatable da tabela para suportar __call
  lua_newtable(L);
  lua_pushcfunction(L, l_sprite_constructor);
  lua_setfield(L, -2, "__call");
  lua_setmetatable(L, -2);

  lua_setglobal(L, "Sprite");
}

// ===== RESGISTER EXPORTED FUNCTIONS =====
void registerApis(lua_State* L) {
  //lua_register(L, "teste", l_teste);
  lua_register(L, "endProgram", l_endProgram);
  lua_register(L, LUA_CLEAR_SCREEN_DEFINITION, l_clearScreen);
  lua_register(L, LUA_UPDATE_SCREEN_DEFINITION, l_updateScreen);
  registerSprite(L);
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
  registerApis(L);

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
  tft.fillScreen(tft.color565(255, 0, 255));
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
}

void loop() {
  // KERNEL
}