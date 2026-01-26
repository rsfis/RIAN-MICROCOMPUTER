#include <Arduino.h>
#include <LittleFS.h>
#include <SPI.h>
#include <LovyanGFX.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <pgmspace.h>
// OTIMIZAR PERFORMANCE
// DIMINUIR O TAMANHO DOS HEADERS
#include "link.h"
#include "ganondorf.h"
#include "sky.h"

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7796 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;
public:
  LGFX(void){
    { auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST; cfg.spi_mode = 0;
      cfg.freq_write = 80000000; cfg.freq_read = 20000000;
      cfg.spi_3wire = false; cfg.use_lock = true; cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 14; cfg.pin_mosi = 13; cfg.pin_miso = 12; cfg.pin_dc = 9;
      _bus_instance.config(cfg); _panel_instance.setBus(&_bus_instance);
    }
    { auto cfg = _panel_instance.config();
      cfg.pin_cs = 10; cfg.pin_rst = 11; cfg.pin_busy = -1;
      cfg.memory_width = 320; cfg.memory_height = 480;
      cfg.panel_width = 320; cfg.panel_height = 480;
      cfg.offset_x = 0; cfg.offset_y = 0;
      cfg.readable = false; cfg.invert = true; cfg.rgb_order = false;
      cfg.dlen_16bit = false; cfg.bus_shared = true;
      _panel_instance.config(cfg);
    }
    { auto cfg = _light_instance.config();
      cfg.pin_bl = 38; cfg.invert = false; cfg.freq = 10000; cfg.pwm_channel = 7;
      _light_instance.config(cfg); _panel_instance.setLight(&_light_instance);
    }
    setPanel(&_panel_instance);
  }
};
static LGFX tft;

// ---------------- 3D math ----------------
struct Vec3 { float x,y,z; };
struct Vec4 { float x,y,z,w; };
struct Mat4 { float m[4][4]; };

inline Vec4 mul(const Mat4 &M,const Vec4 &v){
  Vec4 r;
  r.x = M.m[0][0]*v.x + M.m[0][1]*v.y + M.m[0][2]*v.z + M.m[0][3]*v.w;
  r.y = M.m[1][0]*v.x + M.m[1][1]*v.y + M.m[1][2]*v.z + M.m[1][3]*v.w;
  r.z = M.m[2][0]*v.x + M.m[2][1]*v.y + M.m[2][2]*v.z + M.m[2][3]*v.w;
  r.w = M.m[3][0]*v.x + M.m[3][1]*v.y + M.m[3][2]*v.z + M.m[3][3]*v.w;
  return r;
}

inline Mat4 matIdentity(){ Mat4 I={}; for(int i=0;i<4;i++) for(int j=0;j<4;j++) I.m[i][j]=(i==j)?1.0f:0.0f; return I; }
inline Mat4 matTranslation(float tx,float ty,float tz){ Mat4 M=matIdentity(); M.m[0][3]=tx; M.m[1][3]=ty; M.m[2][3]=tz; return M; }
inline Mat4 matScale(float sx,float sy,float sz){ Mat4 M={}; M.m[0][0]=sx; M.m[1][1]=sy; M.m[2][2]=sz; M.m[3][3]=1.0f; return M; }
inline Mat4 matRotX(float a){ Mat4 M=matIdentity(); float c=cosf(a), s=sinf(a); M.m[1][1]=c; M.m[1][2]=-s; M.m[2][1]=s; M.m[2][2]=c; return M; }
inline Mat4 matRotY(float a){ Mat4 M=matIdentity(); float c=cosf(a), s=sinf(a); M.m[0][0]=c; M.m[0][2]=s; M.m[2][0]=-s; M.m[2][2]=c; return M; }
inline Mat4 matRotZ(float a){ Mat4 M=matIdentity(); float c=cosf(a), s=sinf(a); M.m[0][0]=c; M.m[0][1]=-s; M.m[1][0]=s; M.m[1][1]=c; return M; }
inline Mat4 matMul(const Mat4 &A,const Mat4 &B){ Mat4 R={}; for(int i=0;i<4;i++) for(int j=0;j<4;j++){ float s=0; for(int k=0;k<4;k++) s+=A.m[i][k]*B.m[k][j]; R.m[i][j]=s; } return R; }

Mat4 lookAt(const Vec3 &eye,const Vec3 &center,const Vec3 &up){
  Vec3 f={center.x-eye.x, center.y-eye.y, center.z-eye.z};
  float fl=sqrtf(f.x*f.x+f.y*f.y+f.z*f.z); if(fl==0) fl=1; f.x/=fl; f.y/=fl; f.z/=fl;
  Vec3 upn=up; float ul=sqrtf(upn.x*upn.x+upn.y*upn.y+upn.z*upn.z); if(ul==0) ul=1; upn.x/=ul; upn.y/=ul; upn.z/=ul;
  Vec3 s={ f.y*upn.z - f.z*upn.y, f.z*upn.x - f.x*upn.z, f.x*upn.y - f.y*upn.x };
  float sl = sqrtf(s.x*s.x+s.y*s.y+s.z*s.z); if(sl==0) sl=1; s.x/=sl; s.y/=sl; s.z/=sl;
  Vec3 u={ s.y*f.z - s.z*f.y, s.z*f.x - s.x*f.z, s.x*f.y - s.y*f.x };
  Mat4 M = matIdentity();
  M.m[0][0]=s.x; M.m[0][1]=s.y; M.m[0][2]=s.z; M.m[0][3]=-(s.x*eye.x + s.y*eye.y + s.z*eye.z);
  M.m[1][0]=u.x; M.m[1][1]=u.y; M.m[1][2]=u.z; M.m[1][3]=-(u.x*eye.x + u.y*eye.y + u.z*eye.z);
  M.m[2][0]=-f.x; M.m[2][1]=-f.y; M.m[2][2]=-f.z; M.m[2][3]=(f.x*eye.x + f.y*eye.y + f.z*eye.z);
  M.m[3][0]=0; M.m[3][1]=0; M.m[3][2]=0; M.m[3][3]=1;
  return M;
}

Mat4 perspective(float fovY,float aspect,float zn,float zf){
  float f = 1.0f / tanf(fovY * 0.5f);
  Mat4 P={};
  P.m[0][0] = f / aspect;
  P.m[1][1] = f;
  P.m[2][2] = (zf + zn) / (zn - zf);
  P.m[2][3] = (2.0f * zf * zn) / (zn - zf);
  P.m[3][2] = -1.0f;
  return P;
}

static Vec3 cross(const Vec3& a, const Vec3& b) {
  return Vec3(
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  );
}
// ---------------- Cube (cada cube sabe desenhar-se nos buffers) ----------------
struct Triangle { int v0,v1,v2; uint16_t color; };

struct Model {
  int vertCount;
  int triCount;
  const float* vertices;       // PROGMEM: x,y,z,x,y,z,...
  const unsigned int* indices; // PROGMEM: tripletas (v0,v1,v2,...)
  const float* uvs;            // PROGMEM: u,v,u,v,...
  const unsigned char* texture;// PROGMEM: RGBA bytes
  int texWidth;
  int texHeight;

  Vec3 position;
  Vec3 rotation;
  Vec3 scale;

  Model()
    : vertCount(0), triCount(0),
      vertices(nullptr), indices(nullptr), uvs(nullptr),
      texture(nullptr), texWidth(0), texHeight(0),
      position({0,0,0}), rotation({0,0,0}), scale({1,1,1})
  {}

  Mat4 getModelMatrix() const {
    Mat4 S = matScale(scale.x, scale.y, scale.z);
    Mat4 R = matMul(matRotZ(rotation.z), matMul(matRotY(rotation.y), matRotX(rotation.x)));
    Mat4 T = matTranslation(position.x, position.y, position.z);
    return matMul(T, matMul(R, S));
  }

  // Helpers to read from PROGMEM (works on ESP32 and AVR with included pgmspace.h)
  float readProgFloat(const float* addr) const {
    float v;
    memcpy_P(&v, addr, sizeof(float));
    return v;
  }
  unsigned int readProgUInt(const unsigned int* addr) const {
    unsigned int v;
    memcpy_P(&v, addr, sizeof(unsigned int));
    return v;
  }
  uint8_t readProgByte(const uint8_t* addr) const {
    return pgm_read_byte(addr);
  }

  // Convert RGBA (0-255) to RGB565 (uint16_t)
  static inline uint16_t rgba_to_rgb565(uint8_t r, uint8_t g, uint8_t b, uint8_t /*a*/) {
    return (uint16_t)((((uint16_t)r & 0xF8) << 8) | (((uint16_t)g & 0xFC) << 3) | (((uint16_t)b & 0xF8) >> 3));
  }

  // draw_fast signature matches Sphere::draw_fast
  void draw_fast(const Mat4 &MV, const Mat4 &MVP,
                 uint16_t *hrColor, int32_t *hrZ, int HR_W, int HR_H) const
  {
    if (vertCount <= 0 || triCount <= 0 || vertices == nullptr || indices == nullptr) return;

    const int QX = 16;
    const int QZ = 16;
    const int32_t X_SCALE = 1 << QX;
    const int32_t Z_SCALE = 1 << QZ;

    // 1) projetar e calcular atributos por vértice
    std::vector<Vec4> v_view(vertCount);
    std::vector<Vec4> v_clip(vertCount);
    std::vector<float> ndc_x(vertCount), ndc_y(vertCount), ndc_z(vertCount), wvals(vertCount), view_z(vertCount);
    std::vector<int32_t> px_fp(vertCount), pz_fp(vertCount);
    std::vector<int> py_i(vertCount);

    // ler vértices da PROGMEM e projetar
    for (int i = 0; i < vertCount; ++i) {
      // vertex components are at vertices[3*i + 0..2] in PROGMEM
      float vx, vy, vz;
      memcpy_P(&vx, vertices + (size_t)3*i + 0, sizeof(float));
      memcpy_P(&vy, vertices + (size_t)3*i + 1, sizeof(float));
      memcpy_P(&vz, vertices + (size_t)3*i + 2, sizeof(float));

      // model space -> view space and clip space
      v_view[i] = mul(MV, {vx, vy, vz, 1.0f});
      v_clip[i] = mul(MVP, {vx, vy, vz, 1.0f});

      float w = v_clip[i].w;
      wvals[i] = w;
      // NDC (if w != 0)
      if (w != 0.0f) {
        ndc_x[i] = v_clip[i].x / w;
        ndc_y[i] = v_clip[i].y / w;
        ndc_z[i] = v_clip[i].z / w;
      } else {
        ndc_x[i] = ndc_y[i] = ndc_z[i] = 0.0f;
      }

      float sx = (ndc_x[i] * 0.5f + 0.5f) * (float)HR_W;
      float sy = (-ndc_y[i] * 0.5f + 0.5f) * (float)HR_H;

      px_fp[i] = (int32_t)(sx * (float)X_SCALE + 0.5f);
      py_i[i]  = (int32_t)(sy + 0.5f);
      // depth from view-space z: camera looks towards -Z, front => v_view.z < 0
      pz_fp[i] = (int32_t)((-v_view[i].z) * (float)Z_SCALE + 0.5f);
      view_z[i] = v_view[i].z;
    }

    // 2) Iterate triangles (indices are in PROGMEM)
    for (int ti = 0; ti < triCount; ++ti) {
      // read triangle indices: each triangle uses 3 unsigned ints starting at indices[3*ti + k]
      unsigned int i0, i1, i2;
      memcpy_P(&i0, indices + (size_t)3*ti + 0, sizeof(unsigned int));
      memcpy_P(&i1, indices + (size_t)3*ti + 1, sizeof(unsigned int));
      memcpy_P(&i2, indices + (size_t)3*ti + 2, sizeof(unsigned int));

      if (i0 >= (unsigned)vertCount || i1 >= (unsigned)vertCount || i2 >= (unsigned)vertCount) continue;

      // quick reject if all clip w <= 0
      if (wvals[i0] <= 0.0f && wvals[i1] <= 0.0f && wvals[i2] <= 0.0f) continue;

      // reject if any vertex is behind camera (simple)
      if (view_z[i0] >= 0.0f || view_z[i1] >= 0.0f || view_z[i2] >= 0.0f) continue;

      // Backface culling
      Vec3 A = { v_view[i0].x, v_view[i0].y, v_view[i0].z };
      Vec3 B = { v_view[i1].x, v_view[i1].y, v_view[i1].z };
      Vec3 C = { v_view[i2].x, v_view[i2].y, v_view[i2].z };
      Vec3 AB = { B.x - A.x, B.y - A.y, B.z - A.z };
      Vec3 AC = { C.x - A.x, C.y - A.y, C.z - A.z };
      Vec3 N = cross(AB, AC);
      // camera in view space looks along -Z; if Nz is >0 maybe backface
      //if (N.z >= 0.0f) continue;

      // Screen coords and depths in fixed
      int32_t ax = px_fp[i0], ay = py_i[i0], za = pz_fp[i0];
      int32_t bx = px_fp[i1], by = py_i[i1], zb = pz_fp[i1];
      int32_t cx = px_fp[i2], cy = py_i[i2], zc = pz_fp[i2];

      // sort vertices by y (ay <= by <= cy)
      if (ay > by) { std::swap(ay,by); std::swap(ax,bx); std::swap(za,zb); std::swap(i0,i1); }
      if (by > cy) { std::swap(by,cy); std::swap(bx,cx); std::swap(zb,zc); std::swap(i1,i2); }
      if (ay > by) { std::swap(ay,by); std::swap(ax,bx); std::swap(za,zb); std::swap(i0,i1); }

      if (ay == cy) continue; // degenerate

      // clamp y range
      int yStart = std::max<int>(0, ay);
      int yEnd   = std::min<int>(HR_H-1, cy);

      // compute edge steps dx/dy and dz/dy in fixed point
      auto edge_step = [&](int32_t xA, int32_t yA, int32_t zA, int32_t xB, int32_t yB, int32_t zB)->std::pair<int64_t,int64_t> {
        int dy = yB - yA;
        if (dy == 0) return {0,0};
        int64_t dx_dy = ((int64_t)xB - (int64_t)xA) / dy;
        int64_t dz_dy = ((int64_t)zB - (int64_t)zA) / dy;
        return {dx_dy, dz_dy};
      };

      auto step01 = edge_step(ax,ay,za, bx,by,zb);
      auto step02 = edge_step(ax,ay,za, cx,cy,zc);
      auto step12 = edge_step(bx,by,zb, cx,cy,zc);

      auto edge_x_at = [&](int32_t xA, int32_t yA, int64_t dx_dy, int y)->int32_t {
        return xA + (int32_t)(dx_dy * (int64_t)(y - yA));
      };
      auto edge_z_at = [&](int32_t zA, int32_t yA, int64_t dz_dy, int y)->int32_t {
        return zA + (int32_t)(dz_dy * (int64_t)(y - yA));
      };

      // Precompute vertex UVs (read from PROGMEM)
      float u0=0,v0=0,u1=0,v1=0,u2=0,v2=0;
      bool haveUVs = (uvs != nullptr && texWidth>0 && texHeight>0);
      if (haveUVs) {
        memcpy_P(&u0, uvs + (size_t)2*i0 + 0, sizeof(float));
        memcpy_P(&v0, uvs + (size_t)2*i0 + 1, sizeof(float));
        memcpy_P(&u1, uvs + (size_t)2*i1 + 0, sizeof(float));
        memcpy_P(&v1, uvs + (size_t)2*i1 + 1, sizeof(float));
        memcpy_P(&u2, uvs + (size_t)2*i2 + 0, sizeof(float));
        memcpy_P(&v2, uvs + (size_t)2*i2 + 1, sizeof(float));
      }

      // For each scanline
      for (int y = yStart; y <= yEnd; ++y) {
        // determine left and right edges x and z
        int32_t xL_fp, xR_fp, zL_fp, zR_fp;

        if (y < by) {
          // top part: interp between 0-1 and 0-2
          xL_fp = edge_x_at(ax, ay, step01.first, y);
          zL_fp = edge_z_at(za, ay, step01.second, y);
          xR_fp = edge_x_at(ax, ay, step02.first, y);
          zR_fp = edge_z_at(za, ay, step02.second, y);
        } else {
          // bottom part: interp between 1-2 and 0-2
          xL_fp = edge_x_at(bx, by, step12.first, y);
          zL_fp = edge_z_at(zb, by, step12.second, y);
          xR_fp = edge_x_at(ax, ay, step02.first, y);
          zR_fp = edge_z_at(za, ay, step02.second, y);
        }

        // ensure left <= right
        if (xL_fp > xR_fp) { std::swap(xL_fp, xR_fp); std::swap(zL_fp, zR_fp); }

        // convert to pixel integer x range (x in QX)
        int xStart = (int) ((xL_fp + (1<< (QX-1))) >> QX); // rounded
        int xEnd   = (int) ((xR_fp + (1<< (QX-1))) >> QX);

        if (xEnd < 0 || xStart >= HR_W) continue;
        xStart = std::max(0, xStart);
        xEnd   = std::min(HR_W - 1, xEnd);

        // step in z across scanline: dz/dx in QZ per pixel
        int len = xEnd - xStart + 1;
        if (len <= 0) continue;

        // compute dz/dx
        int64_t dx_line = xR_fp - xL_fp; // in QX units
        int64_t dz_line = (int64_t)zR_fp - (int64_t)zL_fp; // in QZ units
        // dz per pixel (QZ)
        int64_t dz_dx_qz;
        if (dx_line == 0) dz_dx_qz = 0;
        else dz_dx_qz = (dz_line << QX) / dx_line; // careful scaling

        // compute starting z at xStart (QZ)
        // xL_fp is in QX; xStart * X_SCALE equals xStart << QX
        int64_t xStart_fp = ((int64_t)xStart) * (int64_t)X_SCALE;
        int64_t z_at_xStart = (int64_t)zL_fp + (dz_line * (xStart_fp - xL_fp)) / (dx_line==0?1:dx_line);

        // For textured interpolation we need barycentric. We'll compute barycentric in screen space (float) per pixel.
        // Precompute triangle area in screen (in float) using pixel coordinates (not fixed)
        float sx0 = (float)px_fp[i0] / (float)X_SCALE;
        float sy0 = (float)py_i[i0];
        float sx1 = (float)px_fp[i1] / (float)X_SCALE;
        float sy1 = (float)py_i[i1];
        float sx2 = (float)px_fp[i2] / (float)X_SCALE;
        float sy2 = (float)py_i[i2];

        float triArea = (sx1 - sx0)*(sy2 - sy0) - (sx2 - sx0)*(sy1 - sy0);
        if (fabsf(triArea) < 1e-6f) continue;

        // scan across pixels
        int64_t z_qz = z_at_xStart; // QZ fixed
        for (int x = xStart; x <= xEnd; ++x) {
          int pixIndex = y * HR_W + x;
          // depth test
          int32_t z_test = (int32_t)z_qz;
          if (z_test < 0) { z_qz += dz_dx_qz; continue; } // ignore negative depths
          if (z_test <= hrZ[pixIndex]) {
            // compute barycentric weights (in screen space) to interpolate UVs
            float pxf = (float)x + 0.5f;
            float pyf = (float)y + 0.5f;
            // barycentric coordinates:
            float w0 = ( (sx1 - pxf)*(sy2 - pyf) - (sx2 - pxf)*(sy1 - pyf) ) / triArea;
            float w1 = ( (sx2 - pxf)*(sy0 - pyf) - (sx0 - pxf)*(sy2 - pyf) ) / triArea;
            float w2 = 1.0f - w0 - w1;

            uint16_t color565 = 0xFFFF; // default white if no texture
            if (haveUVs && texture != nullptr && texWidth>0 && texHeight>0) {
              // interpolate UV
              float u = w0*u0 + w1*u1 + w2*u2;
              float v = w0*v0 + w1*v1 + w2*v2;
              // wrap/clamp
              if (u < 0) u = 0; if (u > 1) u = 1;
              if (v < 0) v = 0; if (v > 1) v = 1;
              int tx = (int)(u * (texWidth - 1) + 0.5f);
              int ty = (int)(v * (texHeight - 1) + 0.5f);
              int tIndex = (ty * texWidth + tx) * 4; // RGBA

              // read bytes from PROGMEM
              uint8_t r = pgm_read_byte(texture + tIndex + 0);
              uint8_t g = pgm_read_byte(texture + tIndex + 1);
              uint8_t b = pgm_read_byte(texture + tIndex + 2);
              uint8_t a = pgm_read_byte(texture + tIndex + 3);
              //(void)a; // alpha ignored for now

              //color565 = rgba_to_rgb565(r, b, g, a);
              //color565 = rgba_to_rgb565(b, r, g, a);
              color565 = rgba_to_rgb565(r, g, b, a);
            } else {
              // No texture => use white or could use flat shading (based on normal)
              color565 = 0xFFFF;
            }

            // write pixel and depth (remember hrColor is uint16_t, hrZ is int32_t)
            hrColor[pixIndex] = color565;
            hrZ[pixIndex] = z_test;
          }

          z_qz += dz_dx_qz;
        } // for x
      } // for y
    } // for tri
  } // draw_fast
}; // struct Model

struct Camera {
  Vec3 position;
  Vec3 target;
  Vec3 up;
  float fov;
  float nearZ;
  float farZ;

  float yaw;   // horizontal rotation (rad)
  float pitch; // vertical rotation (rad)

  Camera() {
    position = {0.0f, 0.0f, -3.0f};
    target   = {0.0f, 0.0f, 0.0f};
    up       = {0.0f, 1.0f, 0.0f};
    fov      = 60.0f * (3.14159265f/180.0f);
    nearZ    = 0.1f;
    farZ     = 100.0f;
    yaw      = 0.0f;
    pitch    = 0.0f;
  }

  void orbitate(float radius) {
    position.x = target.x + radius * cosf(pitch) * sinf(yaw);
    position.y = target.y + radius * sinf(pitch);
    position.z = target.z + radius * cosf(pitch) * cosf(yaw);
  }

  void freecam(){
    Vec3 front;
    front.x = cosf(pitch) * sinf(yaw);
    front.y = sinf(pitch);
    front.z = cosf(pitch) * cosf(yaw);

    target.x = position.x + front.x;
    target.y = position.y + front.y;
    target.z = position.z + front.z;
  }

  void move(const Vec3 &delta) {
    position.x += delta.x;
    position.y += delta.y;
    position.z += delta.z;
  }

  void rotate(float deltaYaw, float deltaPitch) {
    yaw   += deltaYaw;
    pitch += deltaPitch;
    if(pitch >  1.57f) pitch =  1.57f;
    if(pitch < -1.57f) pitch = -1.57f;
  }

  Mat4 getViewMatrix() const {
    return lookAt(position, target, up);
  }
};

// ---------------- Globals & buffers ----------------
int SCR_W, SCR_H;
int HR_W, HR_H;
uint16_t *hrColor = nullptr;
int32_t *hrZ = nullptr;
uint16_t *rowTmp = nullptr;
LGFX_Sprite frame(&tft);
Camera camera;
Model m;
Model m2;
Model sky;

// FPS
unsigned long lastTime = 0;
float dt = 0;
int frames = 0;
float fpsVal = 0.0f;
unsigned long fpsTimer = 0;

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.setBrightness(255);

  SCR_W = tft.width();
  SCR_H = tft.height();
  HR_W = SCR_W / 2;
  HR_H = SCR_H / 2;

  // allocate PSRAM buffers
  hrColor = (uint16_t*)ps_malloc(sizeof(uint16_t) * HR_W * HR_H);
  hrZ     = (int32_t*)ps_malloc(sizeof(int32_t) * HR_W * HR_H);
  rowTmp  = (uint16_t*)ps_malloc(sizeof(uint16_t) * SCR_W);
  if (!hrColor || !hrZ || !rowTmp) {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(5,5); tft.setTextSize(2);
    tft.print("ERR alloc");
    while (1) delay(1000);
  }

  frame.setPsram(true);
  frame.setColorDepth(16);
  if (!frame.createSprite(SCR_W, SCR_H)) {
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(5,5); tft.setTextSize(2);
    tft.print("ERR frame");
    while (1) delay(1000);
  }

  // camera
  camera.position = {0.0f, 0.0f, -3.0f};
  camera.target   = {0.0f, 0.0f, 0.0f};
  camera.up       = {0.0f, 1.0f, 0.0f};
  camera.fov      = 60.0f * (3.14159265f/180.0f);
  camera.nearZ    = 0.1f;
  camera.farZ     = 100.0f;
  camera.yaw      = 0.0f;
  camera.pitch    = 0.0f;

  m.vertCount = (sizeof(ganondorf_0_vertices)/sizeof(float))/3;
  m.triCount  = (sizeof(ganondorf_0_indices)/sizeof(unsigned int))/3;
  m.vertices  = ganondorf_0_vertices;
  m.indices   = ganondorf_0_indices;
  m.uvs       = ganondorf_0_uvs;
  m.texture   = ganondorf_0_texture;
  m.texWidth  = ganondorf_0_texWidth;
  m.texHeight = ganondorf_0_texHeight;
  m.position = {2.0f, -1.8f, 0.0f};
  m.rotation = {-1.5f, 0.0f, 0.0f};
  m.scale    = {0.028f, 0.028f, 0.028f};

  m2.vertCount = (sizeof(link_0_vertices)/sizeof(float))/3;
  m2.triCount  = (sizeof(link_0_indices)/sizeof(unsigned int))/3;
  m2.vertices  = link_0_vertices;
  m2.indices   = link_0_indices;
  m2.uvs       = link_0_uvs;
  m2.texture   = link_0_texture;
  m2.texWidth  = link_0_texWidth;
  m2.texHeight = link_0_texHeight;
  m2.position = {0.0f, -1.4f, 0.0f};
  m2.rotation = {-1.5f, 0.0f, 0.0f};
  m2.scale    = {0.015f, 0.015f, 0.015f};

  sky.vertCount = (sizeof(sky_0_vertices)/sizeof(float))/3;
  sky.triCount  = (sizeof(sky_0_indices)/sizeof(unsigned int))/3;
  sky.vertices  = sky_0_vertices;
  sky.indices   = sky_0_indices;
  sky.uvs       = sky_0_uvs;
  sky.texture   = sky_0_texture;
  sky.texWidth  = sky_0_texWidth;
  sky.texHeight = sky_0_texHeight;
  sky.position = {0.0f, 0.0f, 0.0f};
  sky.rotation = {0.0f, 0.0f, 0.0f};
  sky.scale    = {15.0f, 15.0f, 15.0f};

  tft.fillScreen(TFT_BLACK);

  lastTime = millis();
  fpsTimer = millis();

  Serial.println("========= SYSTEM INFO =========");

  float heapLivreMB = ESP.getFreeHeap() / (1024.0 * 1024.0);
  float heapTotalMB = ESP.getHeapSize() / (1024.0 * 1024.0);
  Serial.printf("Heap RAM: %.2f MB livre / %.2f MB total\n", heapLivreMB, heapTotalMB);

  if (psramFound()) {
    float psramLivreMB = ESP.getFreePsram() / (1024.0 * 1024.0);
    float psramTotalMB = ESP.getPsramSize() / (1024.0 * 1024.0);
    Serial.printf("PSRAM: %.2f MB livre / %.2f MB total\n", psramLivreMB, psramTotalMB);
  } else {
    Serial.println("⚠️ PSRAM não detectada!");
  }
  
  Serial.printf("Flash Total: %.2f MB\n", ESP.getFlashChipSize() / 1048576.0);
  Serial.printf("Flash APP Disponível: %.2f MB\n", ESP.getFreeSketchSpace() / 1048576.0);
  Serial.printf("CPU freq: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf("XTAL freq: %d MHz\n", getXtalFrequencyMhz());
  Serial.printf("APB freq: %d MHz\n", getApbFrequency() / 1000000);
  Serial.println("================================");
}

void loop() {
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0) dt = 0.0001f;
  lastTime = now;

  // ORBITATE
  camera.move({dt*0.5f, 0.0f, 0.0f});
  camera.rotate(dt*1.0f, 0);
  camera.orbitate(3);

  // clear half-res buffers
  const int total = HR_W * HR_H;
  const int32_t ZINF = INT32_MAX;
  for (int i=0;i<total;i++) { hrZ[i] = ZINF; hrColor[i] = TFT_BLACK; } // define here the background color

  Mat4 view = camera.getViewMatrix();
  Mat4 proj = perspective(camera.fov, (float)HR_W/(float)HR_H, camera.nearZ, camera.farZ);

  Mat4 M = m.getModelMatrix();
  Mat4 MV = matMul(view, M);
  Mat4 MVP = matMul(proj, MV);

  m.draw_fast(MV, MVP, hrColor, hrZ, HR_W, HR_H);

  Mat4 M2 = m2.getModelMatrix();
  Mat4 MV2 = matMul(view, M2);
  Mat4 MVP2 = matMul(proj, MV2);

  m2.draw_fast(MV2, MVP2, hrColor, hrZ, HR_W, HR_H);

  //Mat4 M3 = sky.getModelMatrix();
  //Mat4 MV3 = matMul(view, M3);
  //Mat4 MVP3 = matMul(proj, MV3);

  //sky.draw_fast(MV3, MVP3, hrColor, hrZ, HR_W, HR_H);

  // 4) upscale half-res -> frame sprite (simple nearest)
  for (int hy=0; hy<HR_H; ++hy) {
    uint16_t *src = hrColor + hy * HR_W;
    for (int hx=0; hx<HR_W; ++hx) {
      uint16_t v = src[hx];
      rowTmp[2*hx] = v;
      rowTmp[2*hx + 1] = v;
    }
    int dy = hy * 2;
    frame.pushImage(0, dy, HR_W*2, 1, (lgfx::rgb565_t*)rowTmp);
    frame.pushImage(0, dy+1, HR_W*2, 1, (lgfx::rgb565_t*)rowTmp);
  }

  frames++;
  if (now - fpsTimer >= 1000000) { // 1 segundo
    fpsVal = frames * 1000000.0f / (now - fpsTimer);
    fpsTimer = now;
    frames = 0;
  }
  frame.setTextColor(TFT_WHITE, TFT_BLACK);
  frame.setTextSize(1);
  frame.setCursor(4,4);
  frame.printf("FPS: %.1f", fpsVal);

  frame.pushSprite(0,0);
}
