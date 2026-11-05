// Painel de rota GPS para bike (M5StickC Plus)
// O celular manda posicao e rota por BLE; o Stick desenha a trilha e os numeros.
//
// Protocolo (little endian, uma escrita por mensagem):
//   'P' i32 lat*1e7, i32 lon*1e7, u16 vel km/h*10, i16 rumo graus*10 (-1 = sem),
//       u16 precisao m, u32 distancia m, u32 tempo em movimento s
//   'R' u16 n            inicia uma rota de n pontos
//   'C' u16 inicio, n x (i32 lat*1e7, i32 lon*1e7)
//   'E'                  fim da rota
//   'X'                  apaga a rota
//   'M' i32 lat*1e7, i32 lon*1e7   inicia um mapa de ruas centrado ai
//   'W' varias ruas: [u8 tipo, u8 n, n x (i16 x m, i16 y m)]... (relativas ao centro)
//   'F'                  fim do mapa: passa a ser o mostrado
//
// Botao A (frente): troca de tela; segurado: 2D/3D. Botao B (lado): zoom (auto primeiro); segurado: brilho.

#include <M5Unified.h>
#include <NimBLEDevice.h>
#include <math.h>

#define SERVICE_UUID "6b1e0001-5a1c-4b7e-9d3a-b1ce0a7e0001"
#define CHAR_UUID    "6b1e0002-5a1c-4b7e-9d3a-b1ce0a7e0001"

static const int MAX_ROUTE = 2000;
static const int MAX_TRAIL = 600;
static const float ZOOMS[] = {60, 120, 250, 500, 1000, 2500};  // metros da meia largura
static const int NZOOM = sizeof(ZOOMS) / sizeof(ZOOMS[0]);
static const uint8_t BRIGHT[] = {60, 120, 200, 255};
static const int MAP_PTS = 6000, MAP_WAYS = 2000;

M5Canvas canvas(&M5.Display);

// mapa de ruas, em dois buffers: um mostrado, outro recebendo
struct MapBuf {
  int16_t *x, *y;
  uint16_t *start;
  uint8_t *len, *cls;
  int8_t *bx0, *bx1, *by0, *by1;  // caixa de cada rua, em blocos de 16 m
  int nPts, nWays;
  double lat, lon, mPerDegLon;
};
static MapBuf maps[2];
static volatile int mapFront = -1;
static int mapBack = 0;
static float mx = 0, my = 0;  // posicao atual relativa ao centro do mapa
// suavizacao: entre leituras do GPS (1/s) a tela preve o movimento
static float offX = 0, offY = 0, dispHead = 0;
static uint32_t fixLocalMs = 0;

// ---- dados vindos do celular ----
struct Fix {
  double lat, lon;
  float speed;     // km/h
  float heading;   // graus, <0 sem rumo
  uint16_t acc;
  uint32_t dist;   // m
  uint32_t moving; // s
};
static portMUX_TYPE fixMux = portMUX_INITIALIZER_UNLOCKED;
static Fix fix;
static volatile bool hasFix = false;
static volatile uint32_t lastFixMs = 0;
static volatile bool connected = false;

// rota em metros, relativa ao primeiro ponto
static float rx[MAX_ROUTE], ry[MAX_ROUTE], rcum[MAX_ROUTE];
static int32_t rawLat[MAX_ROUTE], rawLon[MAX_ROUTE];
static volatile int routeN = 0;
static volatile bool routeReady = false;
static volatile bool routeDirty = false;
static int routeExpected = 0;
static double originLat = 0, originLon = 0, mPerDegLon = 111320;

// trilha percorrida
static float tx[MAX_TRAIL], ty[MAX_TRAIL];
static int trailHead = 0, trailCount = 0;

// estado da tela
static int screen = 0, zoom = 0, bright = 1;
// vista: 0 = 3D, 1 = 3D nitido (mais perto, ruas largas), 2 = 2D. Botao A segurado troca
static int viewMode = 0;
static bool view3d = true;
// favoritos vindos do celular (icone 0 = estrela, 1 = casa, 2 = trabalho)
struct Fav { double lat, lon; uint8_t icon; };
static Fav favs[16];
static volatile int favN = 0;
// proxima manobra da rota, limite da via e hora de chegada (minutos do dia)
static volatile uint8_t turnId = 0, turnKind = 0, speedLimit = 0;
static volatile uint16_t turnDist = 0, etaMin = 0xFFFF;
static volatile uint32_t turnMs = 0;
static int nearIdx = 0;
static float remaining = -1, offRoute = 0, lastHeading = 0;
static float px = 0, py = 0;  // posicao atual em metros

static int32_t rd32(const uint8_t* p) { return (int32_t)(p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24); }
static uint16_t rd16(const uint8_t* p) { return p[0] | p[1] << 8; }

static void setOrigin(double lat, double lon) {
  originLat = lat;
  originLon = lon;
  mPerDegLon = 111320.0 * cos(lat * M_PI / 180.0);
}
static void project(double lat, double lon, float& x, float& y) {
  x = (lon - originLon) * mPerDegLon;
  y = (lat - originLat) * 110540.0;
}

class CharCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    NimBLEAttValue v = c->getValue();
    const uint8_t* d = v.data();
    size_t n = v.size();
    if (n < 1) return;
    switch (d[0]) {
      case 'P':
        if (n >= 23) {
          Fix f;
          f.lat = rd32(d + 1) / 1e7;
          f.lon = rd32(d + 5) / 1e7;
          f.speed = rd16(d + 9) / 10.0f;
          f.heading = (int16_t)rd16(d + 11) / 10.0f;
          f.acc = rd16(d + 13);
          f.dist = (uint32_t)rd32(d + 15);
          f.moving = (uint32_t)rd32(d + 19);
          portENTER_CRITICAL(&fixMux);
          fix = f;
          hasFix = true;
          lastFixMs = millis();
          portEXIT_CRITICAL(&fixMux);
        }
        break;
      case 'R':
        if (n >= 3) {
          routeReady = false;
          routeExpected = min((int)rd16(d + 1), MAX_ROUTE);
          routeN = 0;
        }
        break;
      case 'C':
        if (n >= 3) {
          int start = rd16(d + 1);
          int cnt = (n - 3) / 8;
          for (int i = 0; i < cnt && start + i < routeExpected; i++) {
            rawLat[start + i] = rd32(d + 3 + i * 8);
            rawLon[start + i] = rd32(d + 7 + i * 8);
            if (start + i + 1 > routeN) routeN = start + i + 1;
          }
        }
        break;
      case 'E':
        routeDirty = true;  // processa no loop, fora da tarefa do BLE
        break;
      case 'X':
        routeReady = false;
        routeN = 0;
        remaining = -1;
        break;
      case 'M':
        if (n >= 9) {
          MapBuf& m = maps[mapBack];
          m.nPts = m.nWays = 0;
          m.lat = rd32(d + 1) / 1e7;
          m.lon = rd32(d + 5) / 1e7;
          m.mPerDegLon = 111320.0 * cos(m.lat * M_PI / 180.0);
        }
        break;
      case 'W': {
        MapBuf& m = maps[mapBack];
        size_t o = 1;
        while (o + 2 <= n) {
          uint8_t cls = d[o], cnt = d[o + 1];
          o += 2;
          if (o + cnt * 4 > n) break;
          if (cnt >= 2 && m.nWays < MAP_WAYS && m.nPts + cnt <= MAP_PTS) {
            m.start[m.nWays] = m.nPts;
            m.len[m.nWays] = cnt;
            m.cls[m.nWays] = cls;
            int16_t x0 = 32767, x1 = -32768, y0 = 32767, y1 = -32768;
            for (int i = 0; i < cnt; i++) {
              int16_t x = (int16_t)rd16(d + o + i * 4), y = (int16_t)rd16(d + o + 2 + i * 4);
              m.x[m.nPts + i] = x;
              m.y[m.nPts + i] = y;
              x0 = min(x0, x); x1 = max(x1, x); y0 = min(y0, y); y1 = max(y1, y);
            }
            m.bx0[m.nWays] = constrain(x0 >> 4, -128, 127); m.bx1[m.nWays] = constrain((x1 >> 4) + 1, -128, 127);
            m.by0[m.nWays] = constrain(y0 >> 4, -128, 127); m.by1[m.nWays] = constrain((y1 >> 4) + 1, -128, 127);
            m.nPts += cnt;
            m.nWays++;
          }
          o += cnt * 4;
        }
        break;
      }
      case 'F':
        mapFront = mapBack;
        mapBack ^= 1;
        break;
      case 'V': {
        int cnt = n >= 2 ? min((int)d[1], 16) : 0, k = 0;
        for (int i = 0; i < cnt && 2 + i * 9 + 9 <= (int)n; i++, k++) {
          const uint8_t* q = d + 2 + i * 9;
          favs[k] = {rd32(q + 1) / 1e7, rd32(q + 5) / 1e7, q[0]};
        }
        favN = k;
        break;
      }
      case 'T':
        if (n >= 7) {
          turnId = d[1];
          turnKind = d[2];
          turnDist = rd16(d + 3);
          etaMin = rd16(d + 5);
          turnMs = millis();
        }
        break;
      case 'S':
        if (n >= 2) speedLimit = d[1];
        break;
    }
  }
};

class ServerCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override { connected = true; }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    connected = false;
    NimBLEDevice::startAdvertising();
  }
};

static void buildRoute() {
  int n = routeN;
  if (n < 2) { routeReady = false; return; }
  setOrigin(rawLat[0] / 1e7, rawLon[0] / 1e7);
  rcum[0] = 0;
  for (int i = 0; i < n; i++) {
    project(rawLat[i] / 1e7, rawLon[i] / 1e7, rx[i], ry[i]);
    if (i) rcum[i] = rcum[i - 1] + hypotf(rx[i] - rx[i - 1], ry[i] - ry[i - 1]);
  }
  trailCount = 0;  // trilha usava a origem antiga
  nearIdx = 0;
  routeReady = true;
}

// ponto mais proximo da rota: atualiza nearIdx, remaining e offRoute
static void matchRoute() {
  int n = routeN;
  if (!routeReady || n < 2) { remaining = -1; return; }
  auto scan = [&](int a, int b, float& best, int& bi, float& bt) {
    for (int i = max(a, 0); i < min(b, n - 1); i++) {
      float ax = rx[i], ay = ry[i], bx = rx[i + 1], by = ry[i + 1];
      float dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
      float t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
      t = constrain(t, 0.0f, 1.0f);
      float d = hypotf(ax + t * dx - px, ay + t * dy - py);
      if (d < best) { best = d; bi = i; bt = t; }
    }
  };
  float best = 1e9, bt = 0;
  int bi = nearIdx;
  scan(nearIdx - 20, nearIdx + 80, best, bi, bt);  // perto de onde estava, evita pular em lacos
  if (best > 60) scan(0, n, best, bi, bt);
  nearIdx = bi;
  offRoute = best;
  float seg = rcum[bi + 1] - rcum[bi];
  remaining = rcum[n - 1] - rcum[bi] - seg * bt;
}

static void addTrail(float x, float y) {
  int last = (trailHead - 1 + MAX_TRAIL) % MAX_TRAIL;
  if (trailCount && hypotf(x - tx[last], y - ty[last]) < 5) return;
  tx[trailHead] = x;
  ty[trailHead] = y;
  trailHead = (trailHead + 1) % MAX_TRAIL;
  if (trailCount < MAX_TRAIL) trailCount++;
}

static String fmtDist(float m) {
  if (m < 1000) return String((int)m) + " m";
  return String(m / 1000.0f, m < 10000 ? 2 : 1) + " km";
}
static String fmtTime(uint32_t s) {
  char b[12];
  snprintf(b, sizeof b, "%u:%02u:%02u", s / 3600, s / 60 % 60, s % 60);
  return b;
}

static String fmtClock(uint16_t min) {
  char b[8];
  snprintf(b, sizeof b, "%02u:%02u", min / 60, min % 60);
  return b;
}

static void drawBattery(int x, int y) {
  int lvl = M5.Power.getBatteryLevel();
  uint16_t col = lvl > 30 ? TFT_GREEN : lvl > 15 ? TFT_ORANGE : TFT_RED;
  canvas.drawRect(x, y, 20, 9, TFT_LIGHTGREY);
  canvas.fillRect(x + 20, y + 2, 2, 5, TFT_LIGHTGREY);
  canvas.fillRect(x + 2, y + 2, max(0, lvl) * 16 / 100, 5, col);
}

static void drawStatus(bool stale) {
  canvas.fillCircle(232, 6, 3, !connected ? TFT_RED : stale ? TFT_ORANGE : TFT_GREEN);
  drawBattery(206, 2);
}

// seta da manobra: haste de baixo e ponta virada no angulo da curva
static void drawTurnIcon(int kind, int x, int y, uint16_t col) {
  if (kind == 9) {  // chegada
    canvas.drawWideLine(x - 6, y + 11, x - 6, y - 11, 2, col);
    canvas.fillTriangle(x - 6, y - 11, x + 9, y - 6, x - 6, y - 1, TFT_RED);
    return;
  }
  if (kind == 10) {  // rotatoria
    canvas.drawCircle(x, y - 2, 6, col);
    canvas.drawWideLine(x, y + 12, x, y + 4, 2.5f, col);
    canvas.fillTriangle(x + 6, y - 8, x + 11, y - 2, x + 2, y - 2, col);
    return;
  }
  static const int16_t ANG[] = {0, 0, 45, 90, 135, 180, -45, -90, -135};
  float a = ANG[kind < 9 ? kind : 1] * M_PI / 180, sa = sinf(a), ca = cosf(a);
  int ex = x + (int)(sa * 10), ey = y - (int)(ca * 10);
  canvas.drawWideLine(x, y + 12, x, y, 2.5f, col);
  canvas.drawWideLine(x, y, ex, ey, 2.5f, col);
  float px2 = -ca, py2 = -sa;  // perpendicular para a ponta
  canvas.fillTriangle(ex + (int)(sa * 6), ey - (int)(ca * 6), ex + (int)(px2 * 5), ey + (int)(py2 * 5),
                      ex - (int)(px2 * 5), ey - (int)(py2 * 5), col);
}

static void drawMap(const Fix& f, bool stale) {
  const int cx = 120;
  // zoom 0 = automatico: abre conforme a velocidade
  float half = zoom == 0 ? constrain(150 + f.speed * 10, 150.0f, 1200.0f) : ZOOMS[zoom - 1];
  if (viewMode == 1) half *= 0.55f;          // nitido: mais perto
  const float wk = viewMode == 1 ? 2.0f : 1.0f;  // nitido: ruas mais largas
  const float vx = px + offX, vy = py + offY, mvx = mx + offX, mvy = my + offY;
  float h = dispHead * M_PI / 180.0f, ch = cosf(h), sh = sinf(h);
  // 2D: vista de cima. 3D: camera atras e acima de voce, olhando para frente
  const int HOR = 16, ME3D = 112, ME2D = 82;
  const int cy = view3d ? ME3D : ME2D;
  const float s = 120.0f / half;                              // 2D: pixels por metro
  const float back = half * 0.45f;                            // 3D: camera atras de voce (m)
  const float focal = 120.0f * back / half;                   // meia largura = half no seu ponto
  const float camH = (ME3D - HOR) * back / focal;             // altura que poe voce em ME3D
  const float nearZ = back * 0.2f;
  const float farZ = half * 4.5f;                             // 3D: nao desenha alem disso

  if (view3d) {  // ceu e chao com degrade
    canvas.fillGradientRect(0, 0, 240, HOR, 0x10A6, 0x0000, lgfx::VLINEAR);
    canvas.drawFastHLine(0, HOR, 240, 0x2945);
  }

  // projeta um ponto (metros, relativo a voce, leste/norte); falso se atras da camera
  auto proj = [&](float dx, float dy, int& sx, int& sy, float& z) {
    float r = dx * ch - dy * sh, q = dx * sh + dy * ch;
    if (!view3d) { sx = cx + (int)(r * s); sy = ME2D - (int)(q * s); z = back; return true; }
    z = q + back;
    if (z < nearZ) return false;
    sx = cx + (int)(r * focal / z);
    sy = HOR + (int)(focal * camH / z);
    return true;
  };
  auto seg = [&](float ax, float ay, float bx, float by, float w, uint16_t col) {
    float ar = ax * ch - ay * sh, aq = ax * sh + ay * ch;
    float br = bx * ch - by * sh, bq = bx * sh + by * ch;
    float az = back, bz = back;
    int x0, y0, x1, y1;
    if (view3d) {
      az = aq + back; bz = bq + back;
      if ((az < nearZ && bz < nearZ) || (az > farZ && bz > farZ)) return;
      if (az < nearZ) { float t = (nearZ - az) / (bz - az); ar += (br - ar) * t; az = nearZ; }
      else if (bz < nearZ) { float t = (nearZ - bz) / (az - bz); br += (ar - br) * t; bz = nearZ; }
      x0 = cx + (int)(ar * focal / az); y0 = HOR + (int)(focal * camH / az);
      x1 = cx + (int)(br * focal / bz); y1 = HOR + (int)(focal * camH / bz);
      if (w > 0) w *= constrain(back * 1.2f / ((az + bz) * 0.5f), 0.5f, 2.5f);  // perto = mais grosso
    } else {
      x0 = cx + (int)(ar * s); y0 = ME2D - (int)(aq * s);
      x1 = cx + (int)(br * s); y1 = ME2D - (int)(bq * s);
    }
    if ((x0 < 0 && x1 < 0) || (x0 > 240 && x1 > 240) || (y0 < 0 && y1 < 0) || (y0 > 135 && y1 > 135)) return;
    if (w > 1.3f) canvas.drawWideLine(x0, y0, x1, y1, w, col);
    else canvas.drawLine(x0, y0, x1, y1, col);
  };

  // ruas: das menores para as maiores, para as grandes ficarem por cima
  int mf = mapFront;
  if (mf >= 0) {
    const MapBuf& m = maps[mf];
    static const uint16_t COLS[] = {0xFD20, 0xFFFF, 0xAD55, 0x528A};
    static const float WIDTHS[] = {2.6f, 1.8f, 1.0f, 0};
    const float R = view3d ? farZ : half * 1.3f;  // ruas fora deste raio nem sao olhadas
    for (int c = 3; c >= 0; c--) {
      for (int w = 0; w < m.nWays; w++) {
        if (m.cls[w] != c) continue;
        if (mvx + R < m.bx0[w] * 16 || mvx - R > m.bx1[w] * 16 || mvy + R < m.by0[w] * 16 || mvy - R > m.by1[w] * 16) continue;
        int p = m.start[w];
        for (int k = 1; k < m.len[w]; k++)
          seg(m.x[p + k - 1] - mvx, m.y[p + k - 1] - mvy, m.x[p + k] - mvx, m.y[p + k] - mvy,
              viewMode == 1 ? max(WIDTHS[c], 0.8f) * wk : view3d ? WIDTHS[c] : (c < 2 ? WIDTHS[c] : 0), COLS[c]);
      }
    }
  }

  // trilha percorrida
  if (trailCount > 1) {
    int start = (trailHead - trailCount + MAX_TRAIL) % MAX_TRAIL;
    for (int k = 1; k < trailCount; k++) {
      int a = (start + k - 1) % MAX_TRAIL, b = (start + k) % MAX_TRAIL;
      seg(tx[a] - vx, ty[a] - vy, tx[b] - vx, ty[b] - vy, 1.5f, 0x3D7F);
    }
  }

  // rota: ja feita em cinza, o resto em amarelo grosso
  if (routeReady) {
    int n = routeN;
    for (int i = 1; i < n; i++) {
      if (i <= nearIdx) seg(rx[i - 1] - vx, ry[i - 1] - vy, rx[i] - vx, ry[i] - vy, 0, TFT_DARKGREY);
      else seg(rx[i - 1] - vx, ry[i - 1] - vy, rx[i] - vx, ry[i] - vy, 3.0f * wk, TFT_YELLOW);
    }
    int ex, ey; float ez;
    if (proj(rx[n - 1] - vx, ry[n - 1] - vy, ex, ey, ez)) {  // chegada: bandeira
      canvas.drawFastVLine(ex, ey - 12, 12, TFT_WHITE);
      canvas.fillTriangle(ex, ey - 12, ex + 8, ey - 9, ex, ey - 6, TFT_RED);
    }
  }

  // favoritos
  const float kxF = 111320.0f * cosf(f.lat * M_PI / 180.0f);
  for (int i = 0; i < favN; i++) {
    float dx = (favs[i].lon - f.lon) * kxF - offX, dy = (favs[i].lat - f.lat) * 110540.0f - offY;
    int fx, fy; float fz;
    if (hypotf(dx, dy) > (view3d ? farZ : half * 1.5f) || !proj(dx, dy, fx, fy, fz)) continue;
    if (fx < -8 || fx > 248 || fy < -8 || fy > 143) continue;
    if (favs[i].icon == 1) {         // casa
      canvas.fillRect(fx - 5, fy - 6, 11, 7, TFT_GREEN);
      canvas.fillTriangle(fx - 7, fy - 6, fx + 7, fy - 6, fx, fy - 13, TFT_GREEN);
      canvas.fillRect(fx - 1, fy - 4, 3, 5, TFT_BLACK);
    } else if (favs[i].icon == 2) {  // trabalho
      canvas.fillRoundRect(fx - 6, fy - 10, 13, 10, 2, 0x3D7F);
      canvas.drawRect(fx - 2, fy - 13, 5, 3, 0x3D7F);
    } else {                         // estrela
      canvas.fillCircle(fx, fy - 6, 6, TFT_MAGENTA);
      canvas.drawCircle(fx, fy - 6, 6, TFT_WHITE);
      canvas.fillCircle(fx, fy - 6, 2, TFT_WHITE);
    }
    canvas.drawFastVLine(fx, fy - 1, 2, TFT_WHITE);
  }

  // voce: seta branca, sempre para a frente (vermelha fora da rota)
  uint16_t me = offRoute > 40 && routeReady ? TFT_RED : TFT_WHITE;
  int tipY = view3d ? cy - 8 : cy - 9;
  canvas.fillTriangle(cx, tipY, cx - 7, cy + 6, cx + 7, cy + 6, me);
  canvas.fillTriangle(cx - 7, cy + 6, cx + 7, cy + 6, cx, cy + 1, TFT_BLACK);

  // norte
  int nx = 226 + (int)(-sh * 9), ny = 125 - (int)(ch * 9);
  canvas.drawCircle(226, 125, 9, TFT_DARKGREY);
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(TFT_RED);
  canvas.drawString("N", nx, ny);

  // velocidade
  canvas.setTextDatum(top_left);
  canvas.fillRoundRect(0, 0, 74, 34, 4, TFT_BLACK);
  canvas.setFont(&fonts::Font4);
  bool over = speedLimit && f.speed > speedLimit + 3;
  canvas.setTextColor(stale ? TFT_DARKGREY : over ? TFT_RED : TFT_WHITE);
  canvas.drawString(String(f.speed, 0), 4, 3);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.drawString("km/h", 4 + canvas.textWidth("000", &fonts::Font4) - 4, 20);
  if (speedLimit) {  // placa de limite
    canvas.fillCircle(14, 50, 12, TFT_RED);
    canvas.fillCircle(14, 50, 9, TFT_WHITE);
    canvas.setTextDatum(middle_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(TFT_BLACK);
    canvas.drawString(String(speedLimit), 14, 51);
  }

  // proxima manobra
  bool turnOn = turnKind && millis() - turnMs < 6000;
  if (turnOn) {
    bool near = turnDist <= 150;
    canvas.fillRoundRect(80, 0, 92, 34, 5, near ? 0x8400 : 0x18E3);
    drawTurnIcon(turnKind, 96, 15, TFT_WHITE);
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString(turnDist < 1000 ? String(turnDist) : String(turnDist / 1000.0f, 1), 112, 15);
    canvas.setFont(&fonts::Font0);
    canvas.drawString(turnDist < 1000 ? "m" : "km", 112 + canvas.textWidth(turnDist < 1000 ? String(turnDist) : String(turnDist / 1000.0f, 1), &fonts::Font4) + 2, 20);
  }

  // escala e restante
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.setTextDatum(bottom_left);
  canvas.drawString((zoom == 0 ? "A " : "") + fmtDist(half) + (viewMode == 1 ? " +" : ""), 3, 133);
  if (!routeReady && mf < 0) {
    canvas.setTextDatum(bottom_center);
    canvas.setFont(&fonts::Font2);
    canvas.setTextColor(TFT_DARKGREY);
    canvas.drawString("baixando ruas...", 125, 133);
  }
  if (remaining >= 0) {
    canvas.setFont(&fonts::Font2);
    canvas.setTextDatum(bottom_center);
    if (offRoute > 40) {
      canvas.setTextColor(TFT_RED);
      canvas.drawString("fora da rota " + fmtDist(offRoute), 125, 133);
    } else {
      canvas.setTextColor(TFT_YELLOW);
      String t = "faltam " + fmtDist(remaining);
      if (etaMin != 0xFFFF) { char b[8]; snprintf(b, sizeof b, " %02u:%02u", etaMin / 60, etaMin % 60); t += b; }
      canvas.drawString(t, 125, 133);
    }
  }
}

static void drawStats(const Fix& f, bool stale) {
  canvas.setTextDatum(top_left);
  canvas.setFont(&fonts::Font7);
  canvas.setTextColor(stale ? TFT_DARKGREY : TFT_WHITE);
  canvas.drawString(String(f.speed, 1), 4, 4);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.drawString("km/h", 6 + canvas.textWidth(String(f.speed, 1), &fonts::Font7), 36);

  float avg = f.moving > 0 ? f.dist / (float)f.moving * 3.6f : 0;
  struct { const char* k; String v; } rows[] = {
    {"dist", fmtDist(f.dist)},
    {"tempo", fmtTime(f.moving)},
    {etaMin != 0xFFFF ? "chegada" : "media", etaMin != 0xFFFF ? fmtClock(etaMin) : String(avg, 1) + " km/h"},
    {"faltam", remaining >= 0 ? fmtDist(remaining) : String("-")},
  };
  for (int i = 0; i < 4; i++) {
    int x = (i % 2) * 120 + 4, y = 62 + (i / 2) * 36;
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(TFT_DARKGREY);
    canvas.drawString(rows[i].k, x, y);
    canvas.setFont(&fonts::Font4);
    canvas.setTextColor(i == 3 ? TFT_YELLOW : TFT_WHITE);
    canvas.drawString(rows[i].v, x, y + 9);
  }
}

static void drawWaiting() {
  canvas.setTextDatum(middle_center);
  canvas.setFont(&fonts::Font4);
  canvas.setTextColor(TFT_CYAN);
  canvas.drawString("BikeNav", 120, 45);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.drawString(connected ? "conectado, esperando GPS..." : "abra a pagina no Bluefy", 120, 80);
  canvas.drawString(connected ? "" : "e toque em Conectar", 120, 98);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Speaker.setVolume(200);
  M5.Display.setRotation(1);
  Serial.begin(115200);
  M5.Display.setBrightness(BRIGHT[bright]);
  canvas.setColorDepth(16);
  canvas.createSprite(240, 135);
  NimBLEDevice::init("BikeNav");
  NimBLEServer* srv = NimBLEDevice::createServer();
  srv->setCallbacks(new ServerCb());
  NimBLEService* svc = srv->createService(SERVICE_UUID);
  NimBLECharacteristic* ch = svc->createCharacteristic(CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  ch->setCallbacks(new CharCb());
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setName("BikeNav");
  adv->enableScanResponse(true);
  bool advOk = adv->start();
  Serial.printf("ble adv %s heap %u\n", advOk ? "ok" : "FALHOU", ESP.getFreeHeap());

  // mapa por ultimo: o Bluetooth precisa da memoria primeiro
  for (auto& m : maps) {
    m.x = (int16_t*)malloc(MAP_PTS * 2);
    m.y = (int16_t*)malloc(MAP_PTS * 2);
    m.start = (uint16_t*)malloc(MAP_WAYS * 2);
    m.len = (uint8_t*)malloc(MAP_WAYS);
    m.cls = (uint8_t*)malloc(MAP_WAYS);
    for (int8_t** b : {&m.bx0, &m.bx1, &m.by0, &m.by1}) *b = (int8_t*)malloc(MAP_WAYS);
    m.nPts = m.nWays = 0;
  }

#ifdef BENCH
  {  // mapa falso (grade de 100 m) e moto andando, so para medir FPS
    MapBuf& m = maps[0];
    m.lat = -23.56; m.lon = -46.65; m.mPerDegLon = 111320.0 * cos(m.lat * M_PI / 180.0);
    for (int dir = 0; dir < 2; dir++)
      for (int l = -15; l <= 15; l++)
        for (int k = -15; k < 15 && m.nWays < MAP_WAYS; k++) {
          int w = m.nWays, p = m.nPts;
          int16_t a = l * 100, b0 = k * 100, b1 = k * 100 + 100;
          m.x[p] = dir ? a : b0; m.y[p] = dir ? b0 : a;
          m.x[p + 1] = dir ? a : b1; m.y[p + 1] = dir ? b1 : a;
          m.start[w] = p; m.len[w] = 2; m.cls[w] = (l % 5 == 0) ? 1 : 2;
          m.bx0[w] = min(m.x[p], m.x[p + 1]) >> 4; m.bx1[w] = (max(m.x[p], m.x[p + 1]) >> 4) + 1;
          m.by0[w] = min(m.y[p], m.y[p + 1]) >> 4; m.by1[w] = (max(m.y[p], m.y[p + 1]) >> 4) + 1;
          m.nPts += 2; m.nWays++;
        }
    mapFront = 0;
    fix = {m.lat, m.lon, 40, 45, 5, 0, 0};
    hasFix = true; lastFixMs = millis();
    Serial.printf("bench: %d ruas\n", m.nWays);
  }
#endif
  Serial.printf("mapa alocado, heap %u\n",ESP.getFreeHeap());
}

#ifdef BENCH
static uint32_t benchT = 0;
#endif
void loop() {
#ifdef BENCH
  uint32_t now0 = millis();
#endif
  M5.update();
  if (M5.BtnA.wasHold()) { viewMode = (viewMode + 1) % 3; view3d = viewMode != 2; }
  else if (M5.BtnA.wasClicked()) screen = (screen + 1) % 2;
  if (M5.BtnB.wasHold()) {
    bright = (bright + 1) % sizeof(BRIGHT);
    M5.Display.setBrightness(BRIGHT[bright]);
  } else if (M5.BtnB.wasClicked()) {
    zoom = (zoom + 1) % (NZOOM + 1);
  }

  if (routeDirty) { routeDirty = false; buildRoute(); }

  Fix f;
  bool have;
  uint32_t age;
  portENTER_CRITICAL(&fixMux);
  f = fix;
  have = hasFix;
  age = millis() - lastFixMs;
  portEXIT_CRITICAL(&fixMux);
  bool stale = age > 5000;
#ifdef BENCH
  stale = false;
  if (now0 - benchT > 1000) {  // nova leitura falsa a cada 1 s, girando devagar
    benchT = now0;
    portENTER_CRITICAL(&fixMux);
    fix.heading = fmodf(fix.heading + 10, 360); lastFixMs = now0;
    portEXIT_CRITICAL(&fixMux);
  }
  if (M5.BtnB.wasClicked()) {}
#endif

  static uint32_t lastProcessed = 0;
  if (have && lastFixMs != lastProcessed) {
    lastProcessed = lastFixMs;
    if (!routeReady && trailCount == 0) setOrigin(f.lat, f.lon);
    float ox = px, oy = py, shownX = px + offX, shownY = py + offY;
    project(f.lat, f.lon, px, py);
    // continua de onde a tela estava, sem pulo; salto grande (ou origem nova) zera
    offX = shownX - px; offY = shownY - py;
    if (hypotf(offX, offY) > 150) offX = offY = 0;
    fixLocalMs = millis();
    if (f.heading >= 0) lastHeading = f.heading;
    else if (hypotf(px - ox, py - oy) > 3) lastHeading = atan2f(px - ox, py - oy) * 180 / M_PI;
    addTrail(px, py);
    matchRoute();
  }
  int mf = mapFront;
  if (have && mf >= 0) {
    mx = (f.lon - maps[mf].lon) * maps[mf].mPerDegLon;
    my = (f.lat - maps[mf].lat) * 110540.0;
  }

  // previsao entre leituras: anda na direcao atual com a velocidade atual
  static uint32_t lastFrame = millis();
  uint32_t now = millis();
  float frameDt = min((now - lastFrame) / 1000.0f, 0.2f);
  lastFrame = now;
  if (have) {
    float dt = min((now - fixLocalMs) / 1000.0f, 1.5f), v = stale ? 0 : f.speed / 3.6f, hr = lastHeading * M_PI / 180.0f;
    float k = 1 - expf(-frameDt * 8);
    offX += (sinf(hr) * v * dt - offX) * k;
    offY += (cosf(hr) * v * dt - offY) * k;
    float dh = fmodf(lastHeading - dispHead + 540.0f, 360.0f) - 180.0f;  // menor giro
    dispHead = fmodf(dispHead + dh * (1 - expf(-frameDt * 6)) + 360.0f, 360.0f);
  }

  // bipe 150 m antes da curva (dois bipes na chegada)
  static uint8_t beepedId = 0;
  static uint32_t secondBeepAt = 0;
  bool turnFresh = turnKind && now - turnMs < 6000;
  if (turnFresh && turnId != beepedId && turnDist <= 150) {
    beepedId = turnId;
    M5.Speaker.tone(2700, 120);
    if (turnKind == 9) secondBeepAt = now + 220;
  }
  if (secondBeepAt && now >= secondBeepAt) { M5.Speaker.tone(3300, 160); secondBeepAt = 0; }

  // tela apaga depois de 3 min parado; acende ao andar, num botao ou perto de curva
  static uint32_t stillSince = 0;
  static bool asleep = false;
  if (!have || f.speed > 5 || M5.BtnA.wasPressed() || M5.BtnB.wasPressed() || (turnFresh && turnDist < 300)) stillSince = now;
  bool sleepNow = have && now - stillSince > 180000;
  if (sleepNow != asleep) {
    asleep = sleepNow;
    M5.Display.setBrightness(asleep ? 0 : BRIGHT[bright]);
  }
  if (asleep) { delay(50); return; }

  canvas.fillSprite(TFT_BLACK);
  if (!have) drawWaiting();
  else if (screen == 0) drawMap(f, stale);
  else drawStats(f, stale);
  drawStatus(stale);
  canvas.pushSprite(0, 0);

  static uint32_t frames = 0, fpsT = 0;
  frames++;
  if (now - fpsT > 2000) {
    Serial.printf("fps %.1f heap %u\n", frames * 1000.0f / (now - fpsT), ESP.getFreeHeap());
    frames = 0; fpsT = now;
  }
  delay(1);
}
