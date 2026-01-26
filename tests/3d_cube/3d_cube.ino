#include <Arduino.h>
#include <SPI.h>
#include <LovyanGFX.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <cstdint>
#include "esp_sleep.h"
#include "driver/gpio.h"

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
      cfg.pin_bl = 45; cfg.invert = false; cfg.freq = 10000; cfg.pwm_channel = 7;
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

// ---------------- Cube (cada cube sabe desenhar-se nos buffers) ----------------
struct Triangle { int v0,v1,v2; uint16_t color; };

struct Cube {
  Vec3 vertices[8];
  Triangle tris[12];
  Vec3 position;
  Vec3 rotation;
  Vec3 scale;

  Cube() {
    vertices[0] = {-0.5f,-0.5f,-0.5f}; vertices[1] = {0.5f,-0.5f,-0.5f};
    vertices[2] = {0.5f,0.5f,-0.5f};   vertices[3] = {-0.5f,0.5f,-0.5f};
    vertices[4] = {-0.5f,-0.5f,0.5f};  vertices[5] = {0.5f,-0.5f,0.5f};
    vertices[6] = {0.5f,0.5f,0.5f};    vertices[7] = {-0.5f,0.5f,0.5f};
    tris[0] = {0,1,2, TFT_RED}; tris[1] = {0,2,3, TFT_ORANGE};
    tris[2] = {4,6,5, TFT_YELLOW}; tris[3] = {4,7,6, TFT_GREEN};
    tris[4] = {0,5,1, TFT_CYAN}; tris[5] = {0,4,5, TFT_WHITE};
    tris[6] = {3,2,6, TFT_BLUE}; tris[7] = {3,6,7, TFT_PURPLE};
    tris[8] = {0,3,7, TFT_MAGENTA}; tris[9] = {0,7,4, TFT_PINK};
    tris[10] = {1,5,6, TFT_BROWN}; tris[11] = {1,6,2, 0x7BEF};
    position = {0,0,0}; rotation = {0,0,0}; scale = {1,1,1};
  }

  Mat4 getModelMatrix() const {
    Mat4 S = matScale(scale.x, scale.y, scale.z);
    Mat4 R = matMul(matRotZ(rotation.z), matMul(matRotY(rotation.y), matRotX(rotation.x)));
    Mat4 T = matTranslation(position.x, position.y, position.z);
    return matMul(T, matMul(R, S));
  }

  // optimized draw: recebe MV e MVP pré-computadas & parametros de tela
    // optimized draw: recebe MV e MVP pré-computadas & parametros de tela
  void draw_fast(const Mat4 &MV, const Mat4 &MVP,
            uint16_t *hrColor, int32_t *hrZ, int HR_W, int HR_H) const
  {
    const int QX = 16;
    const int QZ = 16;
    const int32_t X_SCALE = 1 << QX;
    const int32_t Z_SCALE = 1 << QZ;

    // 1) calcular vertices em view space e projetados (uma vez)
    Vec4 v_view[8];
    Vec4 v_clip[8];
    float ndc_x[8], ndc_y[8], ndc_z[8];
    int32_t px_fp[8]; // x in QX fixed
    int32_t py_i[8];  // integer y
    int32_t pz_fp[8]; // z in QZ fixed (usaremos view-space z)
    float wvals[8];
    float view_z[8];  // z in view space (positive = behind camera, negative = in front)

    for (int i=0;i<8;i++){
      Vec3 V = vertices[i];
      v_view[i] = mul(MV, {V.x, V.y, V.z, 1.0f});
      v_clip[i] = mul(MVP, {V.x, V.y, V.z, 1.0f});
      float w = v_clip[i].w;
      wvals[i] = w;

      // store view-space z (usaremos para depth & culling)
      view_z[i] = v_view[i].z;

      // if w == 0 will be handled later
      if (w != 0.0f) {
        ndc_x[i] = v_clip[i].x / w;
        ndc_y[i] = v_clip[i].y / w;
        ndc_z[i] = v_clip[i].z / w;
      } else {
        ndc_x[i] = ndc_y[i] = ndc_z[i] = 2.0f; // far off-screen
      }
      // to screen floats (use ndc for screen coords)
      float sx = (ndc_x[i] * 0.5f + 0.5f) * (float)HR_W;
      float sy = (-ndc_y[i] * 0.5f + 0.5f) * (float)HR_H;
      // fast conversion to fixed:
      px_fp[i] = (int32_t) (sx * (float)X_SCALE + 0.5f);
      py_i[i]  = (int32_t) (sy + 0.5f);
      // depth from view-space z: assume camera looks towards negative Z (front => v_view.z < 0)
      // map to positive integer where smaller = closer:
      // if vertex is in front v_view.z < 0 => -v_view.z > 0
      pz_fp[i] = (int32_t) ((-v_view[i].z) * (float)Z_SCALE + 0.5f);
    }

    // 2) iterate triangles
    for (int ti=0; ti<12; ++ti) {
      const Triangle &tr = tris[ti];
      int i0 = tr.v0, i1 = tr.v1, i2 = tr.v2;

      // quick reject if all clip w<=0 (comportamento anterior)
      if (wvals[i0] <= 0.0f && wvals[i1] <= 0.0f && wvals[i2] <= 0.0f) continue;

      // **Reject triangles with any vertex behind the camera**
      // (um tratamento simples que evita artefatos; clipping seria solução completa)
      if (view_z[i0] >= 0.0f || view_z[i1] >= 0.0f || view_z[i2] >= 0.0f) continue;

      // Backface culling using view-space full normal (robusto)
      Vec3 A = { v_view[i0].x, v_view[i0].y, v_view[i0].z };
      Vec3 B = { v_view[i1].x, v_view[i1].y, v_view[i1].z };
      Vec3 C = { v_view[i2].x, v_view[i2].y, v_view[i2].z };
      Vec3 AB = { B.x - A.x, B.y - A.y, B.z - A.z };
      Vec3 AC = { C.x - A.x, C.y - A.y, C.z - A.z };
      // full cross product
      Vec3 normal = {
        AB.y*AC.z - AB.z*AC.y,
        AB.z*AC.x - AB.x*AC.z,
        AB.x*AC.y - AB.y*AC.x
      };
      // In view space the camera looks along -Z; reject if normal.z >= 0 (backface)
      if (normal.z >= 0.1f) continue; // tolerância angular (~5.7°)

      // screen-space integer area test (keeps previous logic but now with earlier culling fixed)
      int sx0 = px_fp[i0], sx1 = px_fp[i1], sx2 = px_fp[i2];
      int sy0 = py_i[i0], sy1 = py_i[i1], sy2 = py_i[i2];
      long area2_qx = (long)(sx1 - sx0) * (long)(sy2 - sy0) - (long)(sy1 - sy0) * (long)(sx2 - sx0);
      if (llabs(area2_qx) < (1LL << (QX+1))) continue; // area threshold scaled
      if (area2_qx < 0) continue; // clockwise/backface in screen space

      // prepare vertex ints (ax,ay in same spaces as before)
      int32_t ax = sx0;
      int32_t ay = sy0;
      int32_t bx = sx1;
      int32_t by = sy1;
      int32_t cx = sx2;
      int32_t cy = sy2;
      int32_t za = pz_fp[i0];
      int32_t zb = pz_fp[i1];
      int32_t zc = pz_fp[i2];

      // Bounding-box clipping in integer screen coords: quick reject if totally outside
      int minX = (int) ((std::min({ax,bx,cx}) + (1<<QX)-1) >> QX); // convert QX->pixel
      int maxX = (int) ((std::max({ax,bx,cx}) + (1<<QX)-1) >> QX);
      int minY = std::min({ay,by,cy});
      int maxY = std::max({ay,by,cy});

      if (maxX < 0 || minX >= HR_W || maxY < 0 || minY >= HR_H) continue;

      // sort vertices by y (ay <= by <= cy)
      if (ay > by) { std::swap(ay,by); std::swap(ax,bx); std::swap(za,zb); }
      if (by > cy) { std::swap(by,cy); std::swap(bx,cx); std::swap(zb,zc); }
      if (ay > by) { std::swap(ay,by); std::swap(ax,bx); std::swap(za,zb); }
      if (ay == cy) continue;

      // clamp y range to screen
      int yStart = std::max<int>(0, ay);
      int yEnd   = std::min<int>(HR_H-1, cy);

      // Precompute edge dx/dy and dz/dy in fixed units (x in QX, z in QZ)
      auto edge_step = [&](int32_t xA, int32_t yA, int32_t zA, int32_t xB, int32_t yB, int32_t zB)->std::pair<int64_t,int64_t> {
        int dy = yB - yA;
        if (dy == 0) {
          return {0,0};
        }
        int64_t dx_dy = ((int64_t)xB - (int64_t)xA) / dy; // QX per scanline
        int64_t dz_dy = ((int64_t)zB - (int64_t)zA) / dy; // QZ per scanline
        return {dx_dy, dz_dy};
      };

      int32_t x0_fp = ax, y0_i = ay, z0_fp = za;
      int32_t x1_fp = bx, y1_i = by, z1_fp = zb;
      int32_t x2_fp = cx, y2_i = cy, z2_fp = zc;

      auto step01 = edge_step(x0_fp, y0_i, z0_fp, x1_fp, y1_i, z1_fp);
      auto step02 = edge_step(x0_fp, y0_i, z0_fp, x2_fp, y2_i, z2_fp);
      auto step12 = edge_step(x1_fp, y1_i, z1_fp, x2_fp, y2_i, z2_fp);

      auto edge_x_at = [&](int32_t xA, int32_t yA, int64_t dx_dy, int y)->int32_t {
        return xA + (int32_t)(dx_dy * (int64_t)(y - yA));
      };
      auto edge_z_at = [&](int32_t zA, int32_t yA, int64_t dz_dy, int y)->int32_t {
        return zA + (int32_t)(dz_dy * (int64_t)(y - yA));
      };

      for (int y = yStart; y <= yEnd; ++y) {
        int32_t xl_fp, xr_fp, zl_fp, zr_fp;
        if (y <= y1_i) {
          xl_fp = edge_x_at(x0_fp, y0_i, step01.first, y);
          zl_fp = edge_z_at(z0_fp, y0_i, step01.second, y);
          xr_fp = edge_x_at(x0_fp, y0_i, step02.first, y);
          zr_fp = edge_z_at(z0_fp, y0_i, step02.second, y);
        } else {
          xl_fp = edge_x_at(x1_fp, y1_i, step12.first, y);
          zl_fp = edge_z_at(z1_fp, y1_i, step12.second, y);
          xr_fp = edge_x_at(x0_fp, y0_i, step02.first, y);
          zr_fp = edge_z_at(z0_fp, y0_i, step02.second, y);
        }

        if (xr_fp < xl_fp) { std::swap(xl_fp, xr_fp); std::swap(zl_fp, zr_fp); }

        // convert fixed x range to integer pixel columns
        int xStart = (int) ((xl_fp + (X_SCALE-1)) >> QX);
        int xEnd   = (int) ((xr_fp) >> QX);

        if (xEnd < 0 || xStart >= HR_W) continue;
        if (xStart < 0) xStart = 0;
        if (xEnd > HR_W-1) xEnd = HR_W-1;
        if (xEnd < xStart) continue;

        int64_t span_fp = (int64_t)xr_fp - (int64_t)xl_fp;
        int idx = y * HR_W + xStart;

        if (span_fp == 0) {
          int64_t depth_fp = ((int64_t)zl_fp + (int64_t)zr_fp) / 2;
          if (depth_fp < hrZ[idx]) { hrZ[idx] = (int32_t)depth_fp; hrColor[idx] = tr.color; }
          continue;
        }

        int64_t numer = (int64_t)(xStart << QX) - (int64_t)xl_fp;
        int64_t depth_fp = (int64_t)zl_fp + ( numer * ( (int64_t)zr_fp - (int64_t)zl_fp ) ) / span_fp;
        int64_t dz_per_pixel_fp = ( ( (int64_t)zr_fp - (int64_t)zl_fp ) << QX ) / span_fp;

        for (int x = xStart; x <= xEnd; ++x, ++idx) {
          if (depth_fp < hrZ[idx]) {
            hrZ[idx] = (int32_t)depth_fp;
            hrColor[idx] = tr.color;
          }
          depth_fp += dz_per_pixel_fp;
        }
      } // scanline
    } // tri
  } // draw_fast
};

struct Sphere {
  static const int VERT_COUNT = 45; //81
  static const int TRI_COUNT  = 64; //128
  Vec3 vertices[VERT_COUNT];
  Triangle tris[TRI_COUNT];
  Vec3 position;
  Vec3 rotation;
  Vec3 scale;

  Sphere() {
    const int stacks = 4;   // latitude // 8
    const int slices = 8;   // longitude
    int v = 0;

    // ----- Gerar vértices -----
    for (int i = 0; i <= stacks; i++) {
      float phi = M_PI * i / stacks;  // 0 → π
      for (int j = 0; j <= slices; j++) {
        float theta = 2 * M_PI * j / slices;  // 0 → 2π
        float x = 0.5f * sin(phi) * cos(theta);
        float y = 0.5f * cos(phi);
        float z = 0.5f * sin(phi) * sin(theta);
        vertices[v++] = {x, y, z};
      }
    }

    // ----- Gerar triângulos -----
    int t = 0;
    for (int i = 0; i < stacks; i++) {
      for (int j = 0; j < slices; j++) {
        int a = i * (slices + 1) + j;
        int b = a + slices + 1;
        int c = b + 1;
        int d = a + 1;

        uint16_t color = 0xFFFF; // branco por padrão
        // Colorir por latitude (só pra ver melhor)
        switch (i % 6) {
          case 0: color = TFT_RED; break;
          case 1: color = TFT_ORANGE; break;
          case 2: color = TFT_YELLOW; break;
          case 3: color = TFT_GREEN; break;
          case 4: color = TFT_BLUE; break;
          case 5: color = TFT_PURPLE; break;
        }

        tris[t++] = {a, b, c, color};
        tris[t++] = {a, c, d, color};
      }
    }

    position = {0,0,0};
    rotation = {0,0,0};
    scale = {1,1,1};
  }

  Mat4 getModelMatrix() const {
    Mat4 S = matScale(scale.x, scale.y, scale.z);
    Mat4 R = matMul(matRotZ(rotation.z), matMul(matRotY(rotation.y), matRotX(rotation.x)));
    Mat4 T = matTranslation(position.x, position.y, position.z);
    return matMul(T, matMul(R, S));
  }

  void draw_fast(const Mat4 &MV, const Mat4 &MVP,
            uint16_t *hrColor, int32_t *hrZ, int HR_W, int HR_H) const
  {
    const int QX = 16;
    const int QZ = 16;
    const int32_t X_SCALE = 1 << QX;
    const int32_t Z_SCALE = 1 << QZ;

    // 1) calcular vertices em view space e projetados (uma vez)
    Vec4 v_view[VERT_COUNT];
    Vec4 v_clip[VERT_COUNT];
    float ndc_x[VERT_COUNT], ndc_y[VERT_COUNT], ndc_z[VERT_COUNT];
    int32_t px_fp[VERT_COUNT]; // x in QX fixed
    int32_t py_i[VERT_COUNT];  // integer y
    int32_t pz_fp[VERT_COUNT]; // z in QZ fixed (usaremos view-space z)
    float wvals[VERT_COUNT];
    float view_z[VERT_COUNT];  // z in view space (positive = behind camera, negative = in front)

    for (int i=0;i<VERT_COUNT;i++){
      Vec3 V = vertices[i];
      v_view[i] = mul(MV, {V.x, V.y, V.z, 1.0f});
      v_clip[i] = mul(MVP, {V.x, V.y, V.z, 1.0f});
      float w = v_clip[i].w;
      wvals[i] = w;

      // store view-space z (usaremos para depth & culling)
      view_z[i] = v_view[i].z;

      // if w == 0 will be handled later
      if (w != 0.0f) {
        ndc_x[i] = v_clip[i].x / w;
        ndc_y[i] = v_clip[i].y / w;
        ndc_z[i] = v_clip[i].z / w;
      } else {
        ndc_x[i] = ndc_y[i] = ndc_z[i] = 2.0f; // far off-screen
      }
      // to screen floats (use ndc for screen coords)
      float sx = (ndc_x[i] * 0.5f + 0.5f) * (float)HR_W;
      float sy = (-ndc_y[i] * 0.5f + 0.5f) * (float)HR_H;
      // fast conversion to fixed:
      px_fp[i] = (int32_t) (sx * (float)X_SCALE + 0.5f);
      py_i[i]  = (int32_t) (sy + 0.5f);
      // depth from view-space z: assume camera looks towards negative Z (front => v_view.z < 0)
      // map to positive integer where smaller = closer:
      // if vertex is in front v_view.z < 0 => -v_view.z > 0
      pz_fp[i] = (int32_t) ((-v_view[i].z) * (float)Z_SCALE + 0.5f);
    }

    // 2) iterate triangles
    for (int ti=0; ti<TRI_COUNT; ++ti) {
      const Triangle &tr = tris[ti];
      int i0 = tr.v0, i1 = tr.v1, i2 = tr.v2;

      // quick reject if all clip w<=0 (comportamento anterior)
      if (wvals[i0] <= 0.0f && wvals[i1] <= 0.0f && wvals[i2] <= 0.0f) continue;

      // **Reject triangles with any vertex behind the camera**
      // (um tratamento simples que evita artefatos; clipping seria solução completa)
      if (view_z[i0] >= 0.0f || view_z[i1] >= 0.0f || view_z[i2] >= 0.0f) continue;

      // Backface culling using view-space full normal (robusto)
      Vec3 A = { v_view[i0].x, v_view[i0].y, v_view[i0].z };
      Vec3 B = { v_view[i1].x, v_view[i1].y, v_view[i1].z };
      Vec3 C = { v_view[i2].x, v_view[i2].y, v_view[i2].z };
      Vec3 AB = { B.x - A.x, B.y - A.y, B.z - A.z };
      Vec3 AC = { C.x - A.x, C.y - A.y, C.z - A.z };
      // full cross product
      Vec3 normal = {
        AB.y*AC.z - AB.z*AC.y,
        AB.z*AC.x - AB.x*AC.z,
        AB.x*AC.y - AB.y*AC.x
      };
      // In view space the camera looks along -Z; reject if normal.z >= 0 (backface)
      //if (normal.z >= 0.1f) continue; // tolerância angular (~5.7°)

      // screen-space integer area test (keeps previous logic but now with earlier culling fixed)
      int sx0 = px_fp[i0], sx1 = px_fp[i1], sx2 = px_fp[i2];
      int sy0 = py_i[i0], sy1 = py_i[i1], sy2 = py_i[i2];
      long area2_qx = (long)(sx1 - sx0) * (long)(sy2 - sy0) - (long)(sy1 - sy0) * (long)(sx2 - sx0);
      if (llabs(area2_qx) < (1LL << (QX+1))) continue; // area threshold scaled
      if (area2_qx < 0) continue; // clockwise/backface in screen space

      // prepare vertex ints (ax,ay in same spaces as before)
      int32_t ax = sx0;
      int32_t ay = sy0;
      int32_t bx = sx1;
      int32_t by = sy1;
      int32_t cx = sx2;
      int32_t cy = sy2;
      int32_t za = pz_fp[i0];
      int32_t zb = pz_fp[i1];
      int32_t zc = pz_fp[i2];

      // Bounding-box clipping in integer screen coords: quick reject if totally outside
      int minX = (int) ((std::min({ax,bx,cx}) + (1<<QX)-1) >> QX); // convert QX->pixel
      int maxX = (int) ((std::max({ax,bx,cx}) + (1<<QX)-1) >> QX);
      int minY = std::min({ay,by,cy});
      int maxY = std::max({ay,by,cy});

      if (maxX < 0 || minX >= HR_W || maxY < 0 || minY >= HR_H) continue;

      // sort vertices by y (ay <= by <= cy)
      if (ay > by) { std::swap(ay,by); std::swap(ax,bx); std::swap(za,zb); }
      if (by > cy) { std::swap(by,cy); std::swap(bx,cx); std::swap(zb,zc); }
      if (ay > by) { std::swap(ay,by); std::swap(ax,bx); std::swap(za,zb); }
      if (ay == cy) continue;

      // clamp y range to screen
      int yStart = std::max<int>(0, ay);
      int yEnd   = std::min<int>(HR_H-1, cy);

      // Precompute edge dx/dy and dz/dy in fixed units (x in QX, z in QZ)
      auto edge_step = [&](int32_t xA, int32_t yA, int32_t zA, int32_t xB, int32_t yB, int32_t zB)->std::pair<int64_t,int64_t> {
        int dy = yB - yA;
        if (dy == 0) {
          return {0,0};
        }
        int64_t dx_dy = ((int64_t)xB - (int64_t)xA) / dy; // QX per scanline
        int64_t dz_dy = ((int64_t)zB - (int64_t)zA) / dy; // QZ per scanline
        return {dx_dy, dz_dy};
      };

      int32_t x0_fp = ax, y0_i = ay, z0_fp = za;
      int32_t x1_fp = bx, y1_i = by, z1_fp = zb;
      int32_t x2_fp = cx, y2_i = cy, z2_fp = zc;

      auto step01 = edge_step(x0_fp, y0_i, z0_fp, x1_fp, y1_i, z1_fp);
      auto step02 = edge_step(x0_fp, y0_i, z0_fp, x2_fp, y2_i, z2_fp);
      auto step12 = edge_step(x1_fp, y1_i, z1_fp, x2_fp, y2_i, z2_fp);

      auto edge_x_at = [&](int32_t xA, int32_t yA, int64_t dx_dy, int y)->int32_t {
        return xA + (int32_t)(dx_dy * (int64_t)(y - yA));
      };
      auto edge_z_at = [&](int32_t zA, int32_t yA, int64_t dz_dy, int y)->int32_t {
        return zA + (int32_t)(dz_dy * (int64_t)(y - yA));
      };

      for (int y = yStart; y <= yEnd; ++y) {
        int32_t xl_fp, xr_fp, zl_fp, zr_fp;
        if (y <= y1_i) {
          xl_fp = edge_x_at(x0_fp, y0_i, step01.first, y);
          zl_fp = edge_z_at(z0_fp, y0_i, step01.second, y);
          xr_fp = edge_x_at(x0_fp, y0_i, step02.first, y);
          zr_fp = edge_z_at(z0_fp, y0_i, step02.second, y);
        } else {
          xl_fp = edge_x_at(x1_fp, y1_i, step12.first, y);
          zl_fp = edge_z_at(z1_fp, y1_i, step12.second, y);
          xr_fp = edge_x_at(x0_fp, y0_i, step02.first, y);
          zr_fp = edge_z_at(z0_fp, y0_i, step02.second, y);
        }

        if (xr_fp < xl_fp) { std::swap(xl_fp, xr_fp); std::swap(zl_fp, zr_fp); }

        // convert fixed x range to integer pixel columns
        int xStart = (int) ((xl_fp + (X_SCALE-1)) >> QX);
        int xEnd   = (int) ((xr_fp) >> QX);

        if (xEnd < 0 || xStart >= HR_W) continue;
        if (xStart < 0) xStart = 0;
        if (xEnd > HR_W-1) xEnd = HR_W-1;
        if (xEnd < xStart) continue;

        int64_t span_fp = (int64_t)xr_fp - (int64_t)xl_fp;
        int idx = y * HR_W + xStart;

        if (span_fp == 0) {
          int64_t depth_fp = ((int64_t)zl_fp + (int64_t)zr_fp) / 2;
          if (depth_fp < hrZ[idx]) { hrZ[idx] = (int32_t)depth_fp; hrColor[idx] = tr.color; }
          continue;
        }

        int64_t numer = (int64_t)(xStart << QX) - (int64_t)xl_fp;
        int64_t depth_fp = (int64_t)zl_fp + ( numer * ( (int64_t)zr_fp - (int64_t)zl_fp ) ) / span_fp;
        int64_t dz_per_pixel_fp = ( ( (int64_t)zr_fp - (int64_t)zl_fp ) << QX ) / span_fp;

        for (int x = xStart; x <= xEnd; ++x, ++idx) {
          if (depth_fp < hrZ[idx]) {
            hrZ[idx] = (int32_t)depth_fp;
            hrColor[idx] = tr.color;
          }
          depth_fp += dz_per_pixel_fp;
        }
      } // scanline
    } // tri
  } // draw_fast
};

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

Camera camera;

// ---------------- Globals & buffers ----------------
int SCR_W, SCR_H;
int HR_W, HR_H;
uint16_t *hrColor = nullptr;
int32_t *hrZ = nullptr;
uint16_t *rowTmp = nullptr;
LGFX_Sprite frame(&tft);
std::vector<Cube> sceneCubes;
std::vector<Sphere> sceneSpheres;

// FPS
unsigned long lastTime = 0;
float dt = 0;
int frames = 0;
float fpsVal = 0.0f;
unsigned long fpsTimer = 0;
#define AWAKEN GPIO_NUM_0
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

  // scene
  sceneCubes.clear();
  Cube c;
  c.position = {0,0,0}; c.rotation = {0.0f,0.0f,0}; c.scale = {1,1,1};
  sceneCubes.push_back(c);
  Cube c2 = c; c2.position = {1.0f, 0.0f, 0.0f}; c2.scale = {0.6f,0.6f,0.6f};
  sceneCubes.push_back(c2);
  Cube c3 = c; c3.position = {-1.0f, 0.0f, 0.4f}; c3.scale = {0.7f,0.7f,0.7f};
  sceneCubes.push_back(c3);
  Cube c4 = c; c4.position = {0.0f, 1.0f, 0.0f}; c4.scale = {0.3f,0.3f,0.9f};
  sceneCubes.push_back(c4);

  Sphere s;
  s.position = {0, 0, -1}; c.rotation = {0.0f,0.0f,0}; c.scale = {0.5f,0.5f,0.5f};
  sceneSpheres.push_back(s);

  tft.fillScreen(TFT_BLACK);

  lastTime = millis();
  fpsTimer = millis();

  Serial.println("===== MEMÓRIA DISPONÍVEL =====");

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


  pinMode(AWAKEN, INPUT_PULLUP);
  esp_sleep_enable_ext0_wakeup(AWAKEN, 0); // LOW wakes

}
int cuframes = 0;
bool podeResetar = false;
void loop() {
  if (cuframes == 137){
    cuframes = 0;
    tft.sleep();
    podeResetar = true;
    delay(100);
    esp_light_sleep_start();
    esp_restart();
  }

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0f;
  if (dt <= 0) dt = 0.0001f;
  lastTime = now;

  // ÓRBITA
  camera.move({dt*0.5f, 0.0f, 0.0f});
  camera.rotate(dt*0.5f, 0);
  camera.orbitate(3);

  // 1) clear half-res buffers
  const int total = HR_W * HR_H;
  const int32_t ZINF = INT32_MAX;
  for (int i=0;i<total;i++) { hrZ[i] = ZINF; hrColor[i] = 0x0000; }

  // 2) clear sprite background
  frame.fillSprite(TFT_BLACK);

  // PRE-COMPUTE view & proj once
  Mat4 view = camera.getViewMatrix();
  Mat4 proj = perspective(camera.fov, (float)HR_W/(float)HR_H, camera.nearZ, camera.farZ);

  // 3) update cubes & draw
  for (Cube &c : sceneCubes) {
    // update local rotation if desired (kept commented)
    //c.rotation.x += dt * 0.7f;
    //c.rotation.y += dt * 0.9f;
    //c.rotation.z += dt * 0.4f;

    Mat4 model = c.getModelMatrix();
    Mat4 MV = matMul(view, model);
    Mat4 MVP = matMul(proj, MV);

    c.draw_fast(MV, MVP, hrColor, hrZ, HR_W, HR_H);
  }

  for (Sphere &s : sceneSpheres) {
    // update local rotation if desired (kept commented)
    s.rotation.x += dt * 0.7f;
    s.rotation.y += dt * 0.9f;
    s.rotation.z += dt * 0.4f;

    Mat4 model = s.getModelMatrix();
    Mat4 MV = matMul(view, model);
    Mat4 MVP = matMul(proj, MV);

    s.draw_fast(MV, MVP, hrColor, hrZ, HR_W, HR_H);
  }

  // 4) upscale half-res -> frame sprite (simple nearest)
  for (int hy=0; hy<HR_H; ++hy) {
    uint16_t *src = hrColor + hy * HR_W;
    for (int hx=0; hx<HR_W; ++hx) {
      uint16_t v = src[hx];
      rowTmp[2*hx] = v;
      rowTmp[2*hx + 1] = v;
    }
    int dy = hy * 2;
    frame.pushImage(0, dy, HR_W*2, 1, rowTmp);
    frame.pushImage(0, dy+1, HR_W*2, 1, rowTmp);
  }

  // 5) overlay FPS
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

  // 6) push sprite to display
  frame.pushSprite(0,0);

  cuframes ++;
}
