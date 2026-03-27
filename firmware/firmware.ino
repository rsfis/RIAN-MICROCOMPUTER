#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <LovyanGFX.hpp>
#include <SD.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "time.h"

#define KERNEL_VERSION "ALPHA 0.2.6"

#define LUA_HEAP_LIMIT (7 * 1024 * 1024)
#define LUA_TASK_STACK 20000

#define SD_CS 4

#define LUA_SPRITE_DEFINITION "Display.Sprite"
#define LUA_CLEAR_SCREEN_DEFINITION "Display_ClearScreen"
#define LUA_UPDATE_SCREEN_DEFINITION "Display_UpdateScreen"
#define LUA_GET_FREE_MEMORY_DEFINITION "OS_GetFreeMemory"
#define LUA_GET_MEMORY_SIZE_DEFINITION "OS_GetMemorySize"
#define LUA_GET_FREE_HEAP_MEMORY_DEFINITION "OS_GetFreeHeapMemory"
#define LUA_GET_HEAP_MEMORY_SIZE_DEFINITION "OS_GetHeapMemorySize"
#define LUA_GET_CPU_TEMPERATURE_DEFINITION "OS_GetCPUTemperature"
#define LUA_GET_TIME_DEFINITION "OS_GetTime"
#define LUA_GET_LUA_HEAP_USAGE_DEFINITION "OS_GetLuaVMHeapMemoryUsage"
#define LUA_GET_USER_USERNAME_DEFINITION "OS_GetUserUsername"
#define LUA_GET_USER_PASSWORD_DEFINITION "OS_GetUserPassword"
#define LUA_OS_SLEEP_DEFINITION "OS_Sleep"
#define LUA_OS_RESTART_DEFINITION "OS_Restart"
#define LUA_OS_GET_KERNEL_VERSION_DEFINITION "OS_GetKernelVersion"
#define LUA_OS_SAVE_BIOS_CONFIGURATION_DEFINITION "OS_SaveBiosConfig"
#define LUA_OS_SAVE_SYSTEM_CONFIGURATION_DEFINITION "OS_SaveSystemConfig"
#define LUA_OS_SAVE_USER_CONFIGURATION_DEFINITION "OS_SaveUserConfig"
#define LUA_OS_SET_BIOS_CONFIG_INTEGER_KEY "OS_SetBiosConfigIntegerKey"
#define LUA_OS_SET_BIOS_CONFIG_INTEGER_KEY "OS_SetBiosConfigStringKey"
#define LUA_OS_SET_SYSTEM_CONFIG_INTEGER_KEY "OS_SetSystemConfigIntegerKey"
#define LUA_OS_SET_SYSTEM_CONFIG_INTEGER_KEY "OS_SetSystemConfigStringKey"
#define LUA_OS_SET_USER_CONFIG_INTEGER_KEY "OS_SetUserConfigIntegerKey"
#define LUA_OS_SET_USER_CONFIG_INTEGER_KEY "OS_SetUserConfigStringKey"

const char* ntpServer = "pool.ntp.org";
long gmtOffset_sec = -3 * 3600;  // UTC-3
const int daylightOffset_sec = 0;

StaticJsonDocument<1024> BiosConfiguration;
StaticJsonDocument<1024> SystemConfiguration;
StaticJsonDocument<1024> UserConfiguration;

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

// TIME SYNC
void syncTime(const char* ssid, const char* password) {

  if (ssid != nullptr && password != nullptr &&
      strlen(ssid) > 0 && strlen(password) > 0) {

    WiFi.begin(ssid, password);
    Serial.println("Wifi Begin");

    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED) {
      if (millis() - startAttemptTime > 60000) {
        Serial.println("ERR 0x005 - WiFi timeout");
        goto WIFI_FAIL;
      }
      delay(300);
    }

    Serial.println("Wifi Connected");

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    struct tm timeinfo;
    startAttemptTime = millis();

    while (!getLocalTime(&timeinfo)) {
      if (millis() - startAttemptTime > 60000) {
        Serial.println("ERR 0x006 - NTP timeout");
        goto WIFI_FAIL;
      }
      delay(100);
    }

    Serial.println("Time Synchronized");

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

WIFI_FAIL:

  tft.print("ERR 0x004 - Couldn't start WIFI and configure time. Using 1/1/1970 - 00:00:00");
  Serial.println("ERR 0x004 - Couldn't start WIFI and configure time. Using 1/1/1970 - 00:00:00");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(3000);
}

static size_t luaHeapUsed = 0;
volatile bool luaRequestExit = false;

// ===== Alocador Lua usando PSRAM =====
void* lua_psram_alloc(void* ud, void* ptr, size_t osize, size_t nsize) {
  (void)ud;
  (void)osize; // NÃO confiamos mais nisso

  const size_t HEADER_SIZE = sizeof(size_t);

  // =========================
  // FREE
  // =========================
  if (nsize == 0) {
    if (ptr) {
      uint8_t* realPtr = (uint8_t*)ptr - HEADER_SIZE;
      size_t storedSize = *((size_t*)realPtr);

      if (luaHeapUsed >= storedSize)
        luaHeapUsed -= storedSize;
      else
        luaHeapUsed = 0;

      heap_caps_free(realPtr);
    }
    return NULL;
  }

  // =========================
  // REALLOC
  // =========================
  if (ptr) {
    uint8_t* realPtr = (uint8_t*)ptr - HEADER_SIZE;
    size_t oldSize = *((size_t*)realPtr);

    size_t futureUsed = luaHeapUsed - oldSize + nsize;

    if (futureUsed > LUA_HEAP_LIMIT) {
      Serial.println("ERR: Lua HEAP LIMIT REACHED");
      return NULL;
    }

    uint8_t* newRealPtr = (uint8_t*)heap_caps_realloc(realPtr, nsize + HEADER_SIZE,
                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!newRealPtr) return NULL;

    *((size_t*)newRealPtr) = nsize;
    luaHeapUsed = futureUsed;

    return newRealPtr + HEADER_SIZE;
  }

  // =========================
  // MALLOC
  // =========================
  if (luaHeapUsed + nsize > LUA_HEAP_LIMIT) {
    Serial.println("ERR: Lua HEAP LIMIT REACHED");
    return NULL;
  }

  uint8_t* realPtr = (uint8_t*)heap_caps_malloc(
    nsize + HEADER_SIZE,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );

  if (!realPtr) return NULL;

  *((size_t*)realPtr) = nsize;
  luaHeapUsed += nsize;

  return realPtr + HEADER_SIZE;
}

// KERNEL FUNCTIONS
bool saveBiosPreferences() {

  if (SD.exists("/boot/boot.json")) {
    SD.remove("/boot/boot.json");
  }

  File file = SD.open("/boot/boot.json", FILE_WRITE);
  if (!file) {
    Serial.println("ERR 0x101 - Couldn't open boot.json for writing.");
    return false;
  }

  if (serializeJson(BiosConfiguration, file) == 0) {
    Serial.println("ERR 0x102 - Failed to write boot.json.");
    file.close();
    return false;
  }

  file.close();
  Serial.println("Saved boot.json");
  return true;
}

bool saveSystemPreferences() {

  if (SD.exists("/usr/sys/general.json")) {
    SD.remove("/usr/sys/general.json");
  }

  File file = SD.open("/usr/sys/general.json", FILE_WRITE);
  if (!file) {
    Serial.println("ERR 0x201 - Couldn't open general.json for writing.");
    return false;
  }

  if (serializeJson(SystemConfiguration, file) == 0) {
    Serial.println("ERR 0x202 - Failed to write general.json.");
    file.close();
    return false;
  }

  file.close();
  Serial.println("Saved system.json");
  return true;
}

bool saveUserPreferences() {

  if (SD.exists("/usr/sys/user.json")) {
    SD.remove("/usr/sys/user.json");
  }

  File file = SD.open("/usr/sys/user.json", FILE_WRITE);
  if (!file) {
    Serial.println("ERR 0x301 - Couldn't open user.json for writing.");
    return false;
  }

  if (serializeJson(UserConfiguration, file) == 0) {
    Serial.println("ERR 0x302 - Failed to write user.json.");
    file.close();
    return false;
  }

  file.close();
  Serial.println("Saved user.json");
  return true;
}

struct Sprite {
  LGFX_Sprite spr;

  float rotation = 0.0f;   // graus
  int width = 0;
  int height = 0;

  Sprite() : spr(&tft) {
    spr.setColorDepth(16);
  }

  bool load(const char* caminho, int largura, int altura) {
    width = largura;
    height = altura;

    spr.setPsram(true);

    if (!spr.createSprite(largura, altura)) {
      tft.setTextColor(TFT_WHITE);
      tft.setCursor(5, 5);
      tft.printf("ERR 0x007 - Sprite creation failed at %s", caminho);
      while (1);
    }

    File f = SD.open(caminho);
    if (!f) {
      tft.setTextColor(TFT_WHITE);
      tft.setCursor(5, 5);
      tft.printf("ERR 0x008 - Couldn't open file at %s", caminho);
      while (1);
    }

    size_t tamanho = f.size();
    uint8_t* buffer = (uint8_t*)malloc(tamanho);
    if (!buffer) {
      tft.setTextColor(TFT_WHITE);
      tft.setCursor(5, 5);
      tft.printf("ERR 0x009 - Malloc failed");
      f.close();
      while (1);
    }

    f.read(buffer, tamanho);
    f.close();

    spr.drawPng(buffer, tamanho, 0, 0);
    free(buffer);

    return true;
  }

  void setRotation(float graus) {
    rotation = graus;
  }

  void draw(int x, int y) {
    if (rotation == 0.0f) {
      spr.pushSprite(&frame, x, y, 0x0000);
      return;
    }

    spr.setPivot(width / 2, height / 2);
    frame.setPivot(x + width / 2, y + height / 2);

    spr.pushRotateZoom(&frame, rotation, 1.0f, 1.0f, 0x0000);
  }
};

// TIME
typedef struct {
    time_t timestamp;
} LuaTime;

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

// OS FUNCTIONS
int l_getFreeMemory(lua_State* L){
  lua_pushinteger(L, ESP.getFreePsram());
  return 1; // number of returned values
}

int l_getMemorySize(lua_State* L){
  lua_pushinteger(L, ESP.getPsramSize());
  return 1;
}

int l_getFreeHeapMemory(lua_State* L){
  lua_pushinteger(L, ESP.getFreeHeap());
  return 1; // number of returned values
}

int l_getHeapMemorySize(lua_State* L){
  lua_pushinteger(L, ESP.getHeapSize());
  return 1;
}

int l_getCpuTemperature(lua_State* L){
    lua_pushnumber(L, temperatureRead());
    return 1;
}

int l_getLuaHeapUsage(lua_State* L){
  lua_pushnumber(L, luaHeapUsed);
  return 1;
}

int l_getUserUsername(lua_State* L){
  const char* username = UserConfiguration["username"];
  lua_pushstring(L, username);
  return 1;
}

int l_getUserPassword(lua_State* L){
  const char* password = UserConfiguration["password"];
  lua_pushstring(L, password);
  return 1;
}

int l_OS_Sleep(lua_State* L){
  delay(luaL_checkinteger(L, 1));
  return 1;
}

int l_OS_Restart(lua_State* L){
  esp_restart();
  return 0;
}

int l_OS_GetKernelVersion(lua_State* L){
  lua_pushstring(L, KERNEL_VERSION);
  return 1;
}

int l_OS_SetBiosConfigIntegerKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    int val = luaL_checkinteger(L, 2);

    BiosConfiguration[key] = val;

    return 0;
}

int l_OS_SetBiosConfigStringKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    const char* val = luaL_checkstring(L, 2);

    BiosConfiguration[key] = val;

    return 0;
}

int l_OS_SetSystemConfigIntegerKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    int val = luaL_checkinteger(L, 2);

    SystemConfiguration[key] = val;

    return 0;
}

int l_OS_SetSystemConfigStringKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    const char* val = luaL_checkstring(L, 2);

    SystemConfiguration[key] = val;

    return 0;
}

int l_OS_SetUserConfigIntegerKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    int val = luaL_checkinteger(L, 2);

    UserConfiguration[key] = val;

    return 0;
}

int l_OS_SetUserConfigStringKey(lua_State* L){
    const char* key = luaL_checkstring(L, 1);
    const char* val = luaL_checkstring(L, 2);

    UserConfiguration[key] = val;

    return 0;
}

int l_OS_SaveBiosConfig(lua_State* L){
  saveBiosPreferences();
  return 0;
}

int l_OS_SaveSystemConfig(lua_State* L){
  saveSystemPreferences();
  return 0;
}

int l_OS_SaveUserConfig(lua_State* L){
  saveUserPreferences();
  return 0;
}

int l_time_toString(lua_State* L) {
    LuaTime* t = (LuaTime*)luaL_checkudata(L, 1, "LuaTime");
    const char* format = luaL_optstring(L, 2, "%d/%m/%Y %H:%M:%S");

    struct tm* tm_info = localtime(&t->timestamp);

    char buffer[64];
    strftime(buffer, sizeof(buffer), format, tm_info);

    lua_pushstring(L, buffer);
    return 1;
}

int l_getTime(lua_State* L) {

    LuaTime* t = (LuaTime*)lua_newuserdata(L, sizeof(LuaTime));
    t->timestamp = time(nullptr);

    luaL_getmetatable(L, "LuaTime");
    lua_setmetatable(L, -2);

    return 1;
}

void registerTime(lua_State* L) {

    luaL_newmetatable(L, "LuaTime");

    lua_pushvalue(L, -1);
    lua_setfield(L, -2, "__index");

    lua_pushcfunction(L, l_time_toString);
    lua_setfield(L, -2, "toString");

    lua_pop(L, 1);
}

struct Font {

#pragma pack(push, 1)
    struct Glyph {
        uint32_t code;
        uint16_t atlas_x;
        uint16_t atlas_y;
        uint16_t w;
        uint16_t h;
        int16_t  xOff;
        int16_t  yOff;
        uint16_t advance;
    };
#pragma pack(pop)

    uint8_t* data = nullptr;
    size_t size = 0;

    uint16_t fontSize = 0;
    uint16_t ascent = 0;
    uint16_t glyphCount = 0;
    uint16_t atlasW = 0;
    uint16_t atlasH = 0;

    Glyph* glyphs = nullptr;
    uint8_t* bitmap = nullptr;

    Font(const char* path){
        load(path);
    }

    bool load(const char* path){

        File f = SD.open(path, FILE_READ);
        if(!f){
            Serial.println("0x010 - Font open error");
            return false;
        }

        size = f.size();

        data = (uint8_t*)heap_caps_malloc(
            size,
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
        );

        if(!data){
            Serial.println("0x011 - Font alloc error");
            f.close();
            return false;
        }

        f.read(data, size);
        f.close();

        uint8_t* ptr = data;

        if(memcmp(ptr, "FNT1", 4) != 0){
            Serial.println("0x012 - Invalid font format");
            return false;
        }

        ptr += 4;

        fontSize   = *(uint16_t*)ptr; ptr += 2;
        ascent     = *(uint16_t*)ptr; ptr += 2;
        glyphCount = *(uint16_t*)ptr; ptr += 2;
        atlasW     = *(uint16_t*)ptr; ptr += 2;
        atlasH     = *(uint16_t*)ptr; ptr += 2;

        glyphs = (Glyph*)ptr;
        ptr += sizeof(Glyph) * glyphCount;

        uint32_t bitmapSize = *(uint32_t*)ptr;
        ptr += 4;

        bitmap = ptr;

        //Serial.printf("Font loaded: %d glyphs | sizeof(Glyph)=%d\n",
        //              glyphCount, sizeof(Glyph));

        return true;
    }

    Glyph* findGlyph(uint32_t code){
        for(int i = 0; i < glyphCount; i++){
            if(glyphs[i].code == code)
                return &glyphs[i];
        }
        return nullptr;
    }

    void drawString(LGFX_Sprite &spr,
                    const char* text,
                    int x,
                    int y,
                    int pts,
                    int r,
                    int g,
                    int b){

        if(!data) return;

        float scale = (float)pts / (float)fontSize;
        uint16_t color = tft.color565(r,g,b);

        int cursorX = x;

        while(*text){

            uint8_t c = *text++;
            Glyph* gptr = findGlyph(c);

            if(!gptr){
                cursorX += pts / 2;
                continue;
            }

            int drawX = cursorX + (int)(gptr->xOff * scale);
            int drawY = y + (int)((ascent + gptr->yOff) * scale);

            int scaledW = (int)(gptr->w * scale);
            int scaledH = (int)(gptr->h * scale);

            for(int sy = 0; sy < scaledH; sy++){
                for(int sx = 0; sx < scaledW; sx++){

                    int srcX = (int)(sx / scale);
                    int srcY = (int)(sy / scale);

                    int atlasIndex =
                        (gptr->atlas_y + srcY) * atlasW +
                        (gptr->atlas_x + srcX);

                    uint8_t alpha = bitmap[atlasIndex];

                    if(alpha > 128){
                        spr.drawPixel(
                            drawX + sx,
                            drawY + sy,
                            color
                        );
                    }
                }
            }

            cursorX += (int)(gptr->advance * scale);
        }
    }

    void freeMemory(){
        if(data){
            heap_caps_free(data);
            data = nullptr;
            size = 0;
        }
    }

    ~Font(){
        freeMemory();
    }
};

int l_font_constructor(lua_State* L){

    const char* path = luaL_checkstring(L, 2);

    Font* font = (Font*)lua_newuserdata(L, sizeof(Font));
    new (font) Font(path);

    if(!font->load(path)){
        return luaL_error(L, "Font load failed");
    }

    luaL_getmetatable(L, "Font");
    lua_setmetatable(L, -2);

    return 1;
}

int l_font_draw(lua_State* L){

    Font* font = (Font*)luaL_checkudata(L, 1, "Font");

    const char* text = luaL_checkstring(L, 2);
    int x = luaL_checkinteger(L, 3);
    int y = luaL_checkinteger(L, 4);
    int pts = luaL_checkinteger(L, 5);
    int r = luaL_checkinteger(L, 6);
    int g = luaL_checkinteger(L, 7);
    int b = luaL_checkinteger(L, 8);

    font->drawString(frame, text, x, y, pts, r, g, b);

    return 0;
}

int l_font_gc(lua_State* L){

    Font* font = (Font*)luaL_checkudata(L, 1, "Font");
    font->freeMemory();

    return 0;
}

void registerFont(lua_State* L){

    luaL_newmetatable(L, "Font");

    lua_pushcfunction(L, l_font_gc);
    lua_setfield(L, -2, "__gc");

    lua_newtable(L);

    lua_pushcfunction(L, l_font_draw);
    lua_setfield(L, -2, "drawString");

    lua_setfield(L, -2, "__index");

    lua_pop(L, 1);

    // Global Font constructor
    lua_newtable(L);

    lua_pushcfunction(L, l_font_constructor);
    lua_setfield(L, -2, "__call");

    lua_setmetatable(L, -2);

    lua_setglobal(L, "Font");
}

// ===== RESGISTER EXPORTED FUNCTIONS =====
void registerApis(lua_State* L) {
  //lua_register(L, "teste", l_teste);
  lua_register(L, "endProgram", l_endProgram);
  lua_register(L, LUA_CLEAR_SCREEN_DEFINITION, l_clearScreen);
  lua_register(L, LUA_UPDATE_SCREEN_DEFINITION, l_updateScreen);
  lua_register(L, LUA_GET_FREE_MEMORY_DEFINITION, l_getFreeMemory);
  lua_register(L, LUA_GET_MEMORY_SIZE_DEFINITION, l_getMemorySize);
  lua_register(L, LUA_GET_FREE_HEAP_MEMORY_DEFINITION, l_getFreeHeapMemory);
  lua_register(L, LUA_GET_HEAP_MEMORY_SIZE_DEFINITION, l_getHeapMemorySize);
  lua_register(L, LUA_GET_CPU_TEMPERATURE_DEFINITION, l_getCpuTemperature);
  lua_register(L, LUA_GET_LUA_HEAP_USAGE_DEFINITION, l_getLuaHeapUsage);
  lua_register(L, LUA_GET_TIME_DEFINITION, l_getTime);
  lua_register(L, LUA_GET_USER_USERNAME_DEFINITION, l_getUserUsername);
  lua_register(L, LUA_GET_USER_PASSWORD_DEFINITION, l_getUserPassword);
  lua_register(L, LUA_OS_SLEEP_DEFINITION, l_OS_Sleep);
  lua_register(L, LUA_OS_RESTART_DEFINITION, l_OS_Restart);
  lua_register(L, LUA_OS_GET_KERNEL_VERSION_DEFINITION, l_OS_GetKernelVersion);
  lua_register(L, LUA_OS_SAVE_BIOS_CONFIGURATION_DEFINITION, l_OS_SaveBiosConfig);
  lua_register(L, LUA_OS_SAVE_SYSTEM_CONFIGURATION_DEFINITION, l_OS_SaveSystemConfig);
  lua_register(L, LUA_OS_SAVE_USER_CONFIGURATION_DEFINITION, l_OS_SaveUserConfig);
  lua_register(L, LUA_OS_SET_BIOS_CONFIG_INTEGER_KEY, l_OS_SetBiosConfigIntegerKey);
  lua_register(L, LUA_OS_SET_BIOS_CONFIG_INTEGER_KEY, l_OS_SetBiosConfigStringKey);
  lua_register(L, LUA_OS_SET_SYSTEM_CONFIG_INTEGER_KEY, l_OS_SetSystemConfigIntegerKey);
  lua_register(L, LUA_OS_SET_SYSTEM_CONFIG_INTEGER_KEY, l_OS_SetSystemConfigStringKey);
  lua_register(L, LUA_OS_SET_USER_CONFIG_INTEGER_KEY, l_OS_SetUserConfigIntegerKey);
  lua_register(L, LUA_OS_SET_USER_CONFIG_INTEGER_KEY, l_OS_SetUserConfigStringKey);
  registerTime(L);
  registerSprite(L);
  registerFont(L);
}

// ===== Task Lua (Core 1) =====
void taskLuaApp(void* arg) {
  Serial.printf("== Lua Task Started at Core: %d ==\n", xPortGetCoreID());

  luaHeapUsed = 0;

  lua_State* L = lua_newstate(lua_psram_alloc, NULL, esp_random());
  if (!L) {
    Serial.println("ERR: Couldn't create LUA VM");
    vTaskDelete(NULL);
    return;
  }

  luaL_openlibs(L);
  registerApis(L);

  File f = SD.open("/sys/src/sys.lua");
  if (!f) {
    Serial.println("ERR: /sys/src/sys.lua not found");
    lua_close(L);
    vTaskDelete(NULL);
    return;
  }

  size_t size = f.size();
  char* buffer = (char*)malloc(size + 1);

  if (!buffer) {
    Serial.println("ERR: No RAM available");
    f.close();
    lua_close(L);
    vTaskDelete(NULL);
    return;
  }

  f.readBytes(buffer, size);
  buffer[size] = '\0';
  f.close();

  // EXECUTA ANTES DE DAR FREE
  if (luaL_dostring(L, buffer) != LUA_OK) {
    Serial.printf("LUA ERROR: %s\n", lua_tostring(L, -1));
  }

  free(buffer);

  // LOOP
  while (!luaRequestExit) {
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  luaRequestExit = false;
  lua_close(L);

  Serial.println("[LUA] VM Shutdown");
  vTaskDelete(NULL);
}

// Boot
Sprite* boot_bg = new Sprite();
Sprite* boot_loadingicon = new Sprite();

TaskHandle_t bootTaskHandle = NULL;
bool bootTaskExit = false;

void taskDrawBootImages(void* arg) {
  while (bootTaskExit != true) {
    boot_loadingicon->rotation += 8;

    boot_bg->draw(0,0);
    boot_loadingicon->draw(204,118);

    frame.pushSprite(0, 0);

    vTaskDelay(pdMS_TO_TICKS(16));
  }

  vTaskDelete(bootTaskHandle);
}

// ===== Kernel (Core 0) =====
void setup() {
  esp_task_wdt_deinit();

  Serial.begin(115200);

  Serial.printf("====== RazorOS ======\n");

  Serial.printf("Starting Kernel at core %d\n", xPortGetCoreID());

  if (!psramFound()) {
    Serial.println("PSRAM NOT FOUND!");
    return;
  }

  // SCREEN START
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setColorDepth(16);
  tft.setBrightness(255);
  tft.setSwapBytes(true);
  Serial.printf("Screen Started\n");

  // SD BEGIN
  if (!SD.begin(SD_CS)) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x001 - Couldn't initiate SD Card module.");
    while (1);
  }
  Serial.printf("Started SD module\n");

  // LOAD BIOS CONFIG
  File biosConfigFile = SD.open("/boot/boot.json");
  if (!biosConfigFile){
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x003 - Couldn't open bios settings bios.json.");
  }
  DeserializationError error = deserializeJson(BiosConfiguration, biosConfigFile);

  if (error) {
    Serial.print("Erro parse: ");
    Serial.println(error.c_str());
    biosConfigFile.close();
    return;
  }

  biosConfigFile.close();
  Serial.printf("Loaded /boot/boot.json configs\n");

  // CREATING SCREEN FRAME
  frame.setPsram(true);
  frame.setColorDepth(16);
  if (!frame.createSprite(tft.width(), tft.height())) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x002 - Couldn't create drawing frame.");
    while (1);
  }
  Serial.printf("Screen Frame created\n");

  // Colors Test
  if (BiosConfiguration["TestScreenColorsWhenStartingUp"] == true){
    Serial.printf("Testing screen colours\n");
    tft.fillScreen(tft.color565(255, 0, 255));
    delay(1000);
    tft.fillScreen(tft.color565(0, 255, 0));
    delay(1000);
    tft.fillScreen(tft.color565(0, 0, 255));
    delay(1000);
    tft.fillScreen(TFT_BLACK);
  }

  // DRAW BOOT IMAGES
  boot_bg->load("/boot/img/boot_bg.png", 480, 320);
  boot_loadingicon->load("/boot/img/boot_loading.png", 72, 72);
  xTaskCreatePinnedToCore(
    taskDrawBootImages,
    "DrawBootImages",
    LUA_TASK_STACK,
    NULL,
    2,
    &bootTaskHandle,
    1
  );

  // LOAD SYSTEM CONFIGS
  File generalConfigFile = SD.open("/usr/sys/general.json");
  if (!generalConfigFile) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x003 - Couldn't open system settings general.json.");
    while (1);
  }

  DeserializationError _error = deserializeJson(SystemConfiguration, generalConfigFile);

  if (_error) {
    Serial.print("Erro parse: ");
    Serial.println(_error.c_str());
    generalConfigFile.close();
    return;
  }

  generalConfigFile.close();
  Serial.printf("Loaded /usr/sys/general.json configs\n");

  const char* ssid = SystemConfiguration["Wifi"]["ssid"];
  const char* password = SystemConfiguration["Wifi"]["password"];

  Serial.println("Synchronizing Time");
  gmtOffset_sec = int(SystemConfiguration["Time"]["timezone"]) * 3600;
  syncTime(ssid, password);
  //struct tm timeinfo;
  //getLocalTime(&timeinfo);
  //Serial.println(&timeinfo, "%d/%m/%Y %H:%M:%S");

  //vTaskDelete(bootTaskHandle);
  //bootTaskHandle = NULL;
  bootTaskExit = true;
  delay(2000);
  bootTaskExit = false;

  // LOAD USER
  // LOAD BIOS CONFIG
  File userConfigFile = SD.open("/usr/sys/user.json");
  if (!userConfigFile){
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1.5);
    tft.setCursor(5, 5);
    tft.print("ERR 0x003 - Couldn't open user user.json.");
  }
  DeserializationError _errorUser = deserializeJson(UserConfiguration, userConfigFile);

  if (_errorUser) {
    Serial.print("Erro parse: ");
    Serial.println(_errorUser.c_str());
    userConfigFile.close();
    return;
  }

  userConfigFile.close();
  Serial.printf("Loaded /usr/sys/user.json user configs\n");

  // TEST
  //BiosConfiguration["TestScreenColorsWhenStartingUp"] = true;
  //UserConfiguration["username"] = "rsfiscina";
  //SystemConfiguration["Time"]["timezone"] = -4;
  //saveBiosPreferences();
  //saveUserPreferences();
  //saveSystemPreferences();

  // ONLY IF APP PRESENT:
  Serial.printf("Starting LUA VM\n");

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