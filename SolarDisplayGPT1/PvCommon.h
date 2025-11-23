#pragma once
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <time.h>

// ------------------------- Anzeige-Konstanten -------------------------
#define tagesAnzeige  1
#define monatsAnzeige 2
#define expPreis 0.138
#define t1Preis 0.344
#define t2Preis 0.2597

// ================= Multicast (UDP) =================
static const IPAddress MCAST_GRP(239, 12, 12, 12);
static const uint16_t  MCAST_PORT = 55221;

// ================= CRC16 (Modbus) =================
static inline uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

// ------------------------- Frame V4 -------------------------
#define PV_MAGIC   0xBEEF
#define PV_VERSION 4

typedef struct __attribute__((packed)) {
  uint16_t magic;         // PV_MAGIC
  uint8_t  version;       // PV_VERSION (=4)
  uint32_t seq;           // laufende Nummer
  uint32_t ts;            // UNIX time (s)

  // Hauptwerte
  int32_t  pvW;
  int32_t  gridW;
  int32_t  battW;
  int32_t  loadW;

  int16_t  temp10;        // 0.1°C
  uint16_t socx10;        // 0.1%

  float    pvTodayKWh;
  float    gridExpToday;
  float    gridImpToday;
  float    loadTodayKWh;

  int32_t  eta20s;        // Sekunden bis 20% (oder -1 wenn unbekannt)

  // Zusatz (Skalen s. Namen)
  int16_t  pv1Voltage_x10_V;
  int16_t  pv1Current_x10_A;
  int16_t  pv2Voltage_x10_V;
  int16_t  pv2Current_x10_A;

  int32_t  gridVoltageA_x10_V;
  int32_t  gridVoltageB_x10_V;
  int32_t  gridVoltageC_x10_V;

  int32_t  gridCurrentA_x100_A;
  int32_t  gridCurrentB_x100_A;
  int32_t  gridCurrentC_x100_A;

  uint16_t crc;           // CRC-16 (Modbus) über alles bis vor 'crc'
} PvFrameV4;

// ------------------------- Layout (CYD 320x240 landscape) -------------------------
static constexpr int W=320, H=240;
static constexpr int PAD_X=8, PAD_Y=6;
static constexpr int STATUS_H=32;
static constexpr int headerLineY=STATUS_H+2;
static constexpr int startY = headerLineY + 6;

// ------------------------- Optionale Provider-Hooks (weak) -------------------------
#if defined(__GNUC__)
extern bool pvGetTodaySplits(float& t1_kWh, float& t2_kWh) __attribute__((weak));
extern bool pvGetTodayExport(float& exp_kWh) __attribute__((weak));
extern bool pvGetTodayPV(float& pv_kWh)      __attribute__((weak));
extern bool pvGetTodayLoad(float& load_kWh)  __attribute__((weak));
#else
extern bool pvGetTodaySplits(float& t1_kWh, float& t2_kWh);
extern bool pvGetTodayExport(float& exp_kWh);
extern bool pvGetTodayPV(float& pv_kWh);
extern bool pvGetTodayLoad(float& load_kWh);
#endif

// ================= Sichtbare Anzeige-Funktionen ===================

// Header
static inline void drawStatusHeader(TFT_eSPI& tft, const PvFrameV4& f){
  // lokale Lambdas
  auto nowHHMM = []() -> String {
    time_t n; struct tm ti; time(&n); localtime_r(&n,&ti);
    char b[6]; snprintf(b,sizeof(b),"%02d:%02d",ti.tm_hour,ti.tm_min);
    return String(b);
  };
  auto fmtETA = [](int32_t s)->String{
    if(s<=0) return "--:--";
    time_t now; time(&now); time_t eta = now + s;
    struct tm ti; localtime_r(&eta,&ti);
    char b[6]; snprintf(b,sizeof(b),"%02d:%02d",ti.tm_hour,ti.tm_min);
    return String(b);
  };

  bool ok20 = (f.socx10>=200);
  uint16_t bg = ok20? TFT_DARKGREEN : TFT_MAROON;
  uint16_t fg = 0xA554; // MidGrey

  tft.fillRect(0,0,W,STATUS_H,bg);

  // Zeit links
  tft.setTextDatum(ML_DATUM); tft.setTextFont(2); tft.setTextSize(2); tft.setTextColor(fg,bg);
  tft.drawString(nowHHMM(), PAD_X, STATUS_H/2);

  // Batterie Mitte
  const int iconW=60, iconH=24;
  const int iconX=W/2-iconW/2-25, iconY=STATUS_H/2-iconH/2;
  tft.drawRect(iconX,iconY,iconW,iconH,fg);
  tft.fillRect(iconX+iconW,iconY+iconH/4,4,iconH/2,fg);
  int soc = f.socx10/10; int fillW=((iconW-4)*constrain(soc,0,100)/100);
  tft.fillRect(iconX+2,iconY+2,fillW,iconH-4,fg);
  tft.setTextDatum(MC_DATUM); tft.setTextFont(1); tft.setTextSize(2); tft.setTextColor(TFT_GREEN);
  tft.drawString(String(soc)+"%", iconX+iconW/2, iconY+iconH/2);

  // ETA rechts
  tft.setTextDatum(MR_DATUM); tft.setTextFont(1); tft.setTextSize(2); tft.setTextColor(fg,bg);
  tft.drawString(String("ETA: ")+fmtETA(f.eta20s), W-PAD_X, STATUS_H/2);

  // Linie
  tft.drawLine(PAD_X, headerLineY, W-PAD_X, headerLineY, TFT_DARKGREY);
}

// Seite 2 – String-Leistungen (PV1/PV2) als Balken + V/A-Anzeige
static inline void drawPage2Content(TFT_eSPI& tft, const PvFrameV4& f){
  // --- lokale Helfer ---
  auto drawHBar = [&](int x, int y, int w, int h, int32_t valueW, int32_t maxW, uint16_t colFill){
    if (maxW <= 0) maxW = 1;
    tft.drawRect(x, y, w, h, TFT_DARKGREY);
    int32_t v = valueW; if (v < 0) v = 0; if (v > maxW) v = maxW;
    int fillW = (int)((int64_t)v * (w-2) / maxW);
    if (fillW > 0) tft.fillRect(x+1, y+1, fillW, h-2, colFill);
  };
  auto fmtWatt = [](int32_t w)->String{
    return (abs(w)>=1000) ? String(w/1000.0,2)+" kW" : String(w)+" W";
  };

  // Korrekte String-Leistung: Vx10 * Ax100 / 1000 -> Watt (mit +500 für Rundung)
  const int32_t pv1W = ((int32_t)f.pv1Voltage_x10_V * (int32_t)f.pv1Current_x10_A + 500) / 1000;
  const int32_t pv2W = ((int32_t)f.pv2Voltage_x10_V * (int32_t)f.pv2Current_x10_A + 500) / 1000;
  // Gesamtleistung (Reg 32064) kommt bereits als Watt in f.pvW
  const int32_t pvTotalW = f.pvW;

  // Echte Einheiten für Anzeige (Strings)
  const float pv1V = f.pv1Voltage_x10_V / 10.0f;
  const float pv1A = f.pv1Current_x10_A / 100.0f;
  const float pv2V = f.pv2Voltage_x10_V / 10.0f;
  const float pv2A = f.pv2Current_x10_A / 100.0f;

  // Layout
  const int barMarginX = PAD_X;
  const int barWidth   = W - 2*barMarginX;
  const int barHeight  = 28;
  const int gapY       = 34;

  const int yTitle = startY + 10;
  const int y1     = startY + 14 + 20;
  const int y2     = y1 + barHeight + gapY;
  const int y3     = y2 + barHeight + gapY;  // Gesamt

  // Skalen
  const int32_t MAX_W_STR  = 6000; // 6 kW für PV1/PV2
  const int32_t MAX_W_TOT  = 9000; // 9 kW für Gesamt

  // Titel + Hinweis für String-Balken
  tft.setTextDatum(ML_DATUM);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("PV-String Leistung", PAD_X, yTitle);

  tft.setTextDatum(MR_DATUM);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("PV1/PV2 max 6.0 kW", W-PAD_X, yTitle);

  // ---- PV1 ----
  drawHBar(barMarginX, y1, barWidth, barHeight, pv1W, MAX_W_STR, TFT_YELLOW);
  // Label links
  tft.setTextDatum(ML_DATUM);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("PV1", barMarginX, y1 - 4);
  // Wert rechts
  tft.setTextDatum(MR_DATUM);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtWatt(pv1W), barMarginX + barWidth, y1 + barHeight/2);
  // Unterzeile V/A
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString(String(pv1V,1)+" V  ·  "+String(pv1A,2)+" A", barMarginX + barWidth, y1 + barHeight + 12);

  // ---- PV2 ----
  drawHBar(barMarginX, y2, barWidth, barHeight, pv2W, MAX_W_STR, TFT_CYAN);
  // Label links
  tft.setTextDatum(ML_DATUM);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("PV2", barMarginX, y2 - 4);
  // Wert rechts
  tft.setTextDatum(MR_DATUM);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtWatt(pv2W), barMarginX + barWidth, y2 + barHeight/2);
  // Unterzeile V/A
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString(String(pv2V,1)+" V  ·  "+String(pv2A,2)+" A", barMarginX + barWidth, y2 + barHeight + 12);

  // ---- Gesamt (Reg 32064) ----
  drawHBar(barMarginX, y3, barWidth, barHeight, pvTotalW, MAX_W_TOT, TFT_ORANGE);
  // Label links
  tft.setTextDatum(ML_DATUM);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Total", barMarginX, y3 - 4);
  // Wert rechts
  tft.setTextDatum(MR_DATUM);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtWatt(pvTotalW), barMarginX + barWidth, y3 + barHeight/2);
  // kleine Skalen-Notiz für Total rechts oben vom Balken
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("max 9.0 kW", barMarginX + barWidth, y3 - 4);
}

// Seite 3 – Uhrzeit + Datum
static inline void drawPage3Content(TFT_eSPI& tft, const PvFrameV4&){
  // lokale Lambdas
  auto nowHHMM = []() -> String {
    time_t n; struct tm ti; time(&n); localtime_r(&n,&ti);
    char b[6]; snprintf(b,sizeof(b),"%02d:%02d",ti.tm_hour,ti.tm_min);
    return String(b);
  };
  auto fmtDateDDMMYYYY = []() -> String {
    time_t n; struct tm ti; time(&n); localtime_r(&n,&ti);
    static const char* wd[7] = {"So","Mo","Di","Mi","Do","Fr","Sa"};
    char b[24];
    snprintf(b, sizeof(b), "%s %02d.%02d.%04d",
            wd[ti.tm_wday], ti.tm_mday, ti.tm_mon+1, ti.tm_year+1900);
    return String(b);
  };

  tft.setTextDatum(MC_DATUM);

  // Zeit groß
  tft.setTextFont(4);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(nowHHMM(), W/2, headerLineY + (H - headerLineY)/2 - 16);

  // Datum darunter
  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString(fmtDateDDMMYYYY(), W/2, headerLineY + (H - headerLineY)/2 + 16);
}

// Seite 5 – Drei Zeiger-Gauges: PV (Reg 32064), Batterie (±), Grid (±)
static void drawPage5Content(TFT_eSPI& tft, const PvFrameV4& f){
  // ---------- Geometrie ----------
  const int gaugesY = 42;               // Oberkante der 3 Meter
  const int gW = 100, gH = 160;         // Größe pro Meter
  const int xPV   =   5;
  const int xBATT = 110;
  const int xGRID = 215;

  // ---------- Werte aus Frame ----------
  const float pvW   = (float)f.pvW;
  const float battW = (float)f.battW; // +C/-D
  const float gridW = (float)f.gridW; // +Export/-Import (laut Kommentar)
  const float loadW = (float)((int32_t)f.pvW - (int32_t)f.gridW - (int32_t)f.battW);
  float pvToday = f.pvTodayKWh;        // Fallback, falls Hook nicht da
  const float tempC   = (f.temp10 != INT16_MIN) ? (f.temp10 / 10.0f) : NAN;

  // integrierte Last (heute) via Hook (Fallback: "--")
  float loadToday = NAN;
  if (&pvGetTodayLoad) { float v; if (pvGetTodayLoad(v)) loadToday = v; }
  // integrierte PV (heute) via Hook (Fallback: Frame)
  if (&pvGetTodayPV) { float v; if (pvGetTodayPV(v)) pvToday = v; }

  // ---------- Ranges für Gauges ----------
  const float pvMin=0,      pvMax=9000;   // PV 0..9 kW
  const float batMin=-5000, batMax=+5000;
  const float gridMin=-9000,gridMax=+9000;

  // Deadbands
  const float GRID_DB = 25.0f;
  const float BATT_DB = 25.0f;
  const float PV_DB   = 50.0f;

  // ---------- lokale Helfer ----------
  auto fmtKWh = [&](float v)->String{
    if (isnan(v)) return String("--");
    char b[24];
    dtostrf(v, 0, (v<10.f?2:1), b);
    String s(b);
    if (s.indexOf('.')>=0){
      while (s.endsWith("0")) s.remove(s.length()-1);
      if (s.endsWith(".")) s.remove(s.length()-1);
    }
    return s + " kWh";
  };
  auto fmtW = [&](float v)->String{
    char b[24];
    float av=fabsf(v);
    if (av>=10000.f){ dtostrf(v/1000.f,0,1,b); return String(b)+" kW"; }
    if (av>=1000.f){  dtostrf(v/1000.f,0,2,b); return String(b)+" kW"; }
    dtostrf(v,0,0,b); return String(b)+" W";
  };
  auto fmtTemp = [&](float t)->String{
    if (isnan(t)) return String("--");
    char b[24]; dtostrf(t,0,1,b); return String(b)+" \xB0""C";
  };
  auto drawLabel = [&](int xLeft, int y, const char* label, const String& val, uint16_t col=TFT_LIGHTGREY){
    tft.setTextDatum(TL_DATUM);
    tft.setTextFont(2); tft.setTextSize(1);
    tft.setTextColor(col, TFT_BLACK);
    tft.drawString(label, xLeft, y);
    int lw = tft.textWidth(label);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(val, xLeft + lw + 6, y);
  };
  auto drawCentered = [&](int xCenter, int y, const String& val, uint16_t col){
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(2); tft.setTextSize(1);
    tft.setTextColor(col, TFT_BLACK);
    tft.drawString(val, xCenter, y+10);
  };
  auto fmtCHF = [&](float v)->String{ char b[24]; dtostrf(v,0,2,b); return String(b)+" CHF"; };

  // ---------- Gauge-Zeichner (Sprite, 270°-Skala, dynamische Farben) ----------
  auto drawGauge = [&](int x, int y, int w, int h,
                       float value, float vMin, float vMax,
                       const char* name,
                       bool bipolar, float deadband, bool positiveIsGreen){
    if (w < 60) w = 60;
    if (h < 80) h = 80;
    if (vMax == vMin) vMax = vMin + 1.0f;

    // Sprite pro Gauge – simpel & flackerfrei (wird jedes Mal erstellt)
    TFT_eSprite spr(&tft);
    spr.setColorDepth(16);
    spr.createSprite(w, h);
    spr.fillSprite(TFT_BLACK);

    // Farben
    const uint16_t COL_GRAY     = TFT_DARKGREY;
    const uint16_t COL_POS      = TFT_GREEN;
    const uint16_t COL_NEG      = TFT_RED;
    const uint16_t COL_TICK     = TFT_LIGHTGREY;
    const uint16_t COL_ZERO     = TFT_YELLOW;
    const uint16_t COL_LABEL    = TFT_WHITE;
    const uint16_t COL_VALUE    = TFT_WHITE;
    const uint16_t COL_NEEDLE   = TFT_WHITE;
    const uint16_t COL_HUB      = TFT_LIGHTGREY;

    // Geometrie
    const int pad = 4;
    const int cx = w / 2;
    const int cy = (int)(h * 0.44f);            // etwas höher -> unten mehr Platz
    int rOuter = (min(w, h) / 2) - pad; if (rOuter < 22) rOuter = 22;
    const int arcThick  = max(8, min(14, h / 12));
    const int rTickBase = rOuter;

    // Winkel
    auto deg2rad = [](float d){ return d * 3.14159265358979323846f / 180.0f; };
    const float aMin  = 225.0f;  // vMin
    const float aMax  = -45.0f;  // vMax
    const float aZero = 90.0f;   // 12 Uhr

    // dicker Bogen (unten offen)
    auto sprDrawThickArc = [&](float aStartDeg, float aEndDeg, uint16_t color){
      int thickness = arcThick;
      int rInner = rOuter - thickness; if (rInner < 1) rInner = 1;
      const float step = 3.0f;
      const int dir = (aEndDeg < aStartDeg) ? -1 : +1;
      float a = aStartDeg;
      while (true) {
        float aNext = a + dir * step;
        bool last = (dir < 0) ? (aNext <= aEndDeg) : (aNext >= aEndDeg);
        if (last) aNext = aEndDeg;

        float r0 = deg2rad(a), r1 = deg2rad(aNext);
        int x0o = (int)(cx + cosf(r0) * rOuter);
        int y0o = (int)(cy - sinf(r0) * rOuter);
        int x1o = (int)(cx + cosf(r1) * rOuter);
        int y1o = (int)(cy - sinf(r1) * rOuter);
        int x0i = (int)(cx + cosf(r0) * (rOuter - thickness));
        int y0i = (int)(cy - sinf(r0) * (rOuter - thickness));
        int x1i = (int)(cx + cosf(r1) * (rOuter - thickness));
        int y1i = (int)(cy - sinf(r1) * (rOuter - thickness));

        spr.fillTriangle(x0o, y0o, x1o, y1o, x0i, y0i, color);
        spr.fillTriangle(x1o, y1o, x1i, y1i, x0i, y0i, color);

        if (last) break;
        a = aNext;
      }
    };
    auto sprTick = [&](float angDeg, int len, uint16_t col){
      float r = deg2rad(angDeg);
      int x0 = (int)(cx + cosf(r) * (rTickBase - len));
      int y0 = (int)(cy - sinf(r) * (rTickBase - len));
      int x1 = (int)(cx + cosf(r) * rTickBase);
      int y1 = (int)(cy - sinf(r) * rTickBase);
      spr.drawLine(x0, y0, x1, y1, col);
    };

    // Basisskala: grau
    if (bipolar) {
      sprDrawThickArc(aMin,  aZero, COL_GRAY); // negativ
      sprDrawThickArc(aZero, aMax,  COL_GRAY); // positiv
    } else {
      sprDrawThickArc(aMin, aMax, COL_GRAY);
    }

    // Farbakzent abhängig vom Wert / Deadband
    auto clamp01 = [](float x){ return x<0.f?0.f:(x>1.f?1.f:x); };

    if (!bipolar) {
      // Unipolar (PV): grün wenn > DB, ansonsten bleibt grau
      if (value > deadband) {
        // komplette Skala grün einfärben (ästhetisch)
        sprDrawThickArc(aMin, aMax, COL_POS);
      }
    } else {
      // Bipolar (Batt/Grid), Deadband ±deadband bleibt grau
      if (value >  deadband) {
        // positive Seite farblich
        sprDrawThickArc(aZero, aMax, positiveIsGreen ? COL_POS : COL_NEG);
      } else if (value < -deadband) {
        // negative Seite farblich
        sprDrawThickArc(aMin,  aZero, positiveIsGreen ? COL_NEG : COL_POS);
      }
    }

    // Ticks
    sprTick(aMin,  arcThick + 4, COL_TICK);
    sprTick(aMax,  arcThick + 4, COL_TICK);
    if (bipolar) sprTick(aZero, arcThick + 6, COL_ZERO);

    // Label oben
    spr.setTextFont(2);
    spr.setTextSize(1);
    spr.setTextDatum(TC_DATUM);
    spr.setTextColor(COL_LABEL);
    int fh = spr.fontHeight();
    spr.fillRect(0, 0, w, fh + 2, TFT_BLACK);
    spr.drawString(String(name), cx, 1);

    // Wert groß, ~50 px über Unterkante
    auto formatWithK = [](float v)->String{
      char buf[24];
      float av = fabsf(v);
      if (av >= 1000.0f) {
        uint8_t dec = (av < 10000.0f) ? 1 : 0;
        dtostrf(v / 1000.0f, 0, dec, buf);
        return String(buf) + "k";
      } else {
        uint8_t dec = (av < 10.0f) ? 1 : (av < 100.0f ? 1 : 0);
        dtostrf(v, 0, dec, buf);
        String s(buf);
        if (s.indexOf('.') >= 0) {
          while (s.endsWith("0")) s.remove(s.length() - 1);
          if (s.endsWith(".")) s.remove(s.length() - 1);
        }
        return s;
      }
    };
    String vStr = formatWithK(value);
    spr.setTextFont(4);
    spr.setTextSize(1);
    spr.setTextDatum(MC_DATUM);
    spr.setTextColor(COL_VALUE);
    const int valueYOffset = 50;
    int yVal = h - valueYOffset;
    if (yVal < fh + 16) yVal = fh + 16;
    spr.drawString(vStr, cx, yVal);

    // Zeigerwinkel
    if (value < vMin) value = vMin;
    if (value > vMax) value = vMax;

    float angDeg;
    if (bipolar) {
      if (value >= 0.0f) {
        float t = (vMax > 0) ? clamp01(value / vMax) : 0.f;
        angDeg = 90.0f - t * 135.0f;        // 90 -> -45
      } else {
        float t = (vMin < 0) ? clamp01(value / vMin) : 0.f;
        angDeg = 90.0f + t * 135.0f;        // 90 -> 225
      }
    } else {
      float t = clamp01((value - vMin) / (vMax - vMin));
      angDeg = 225.0f - t * 270.0f;         // 225 -> -45
    }

    // Zeiger (Dreieck)
    float ar = deg2rad(angDeg);
    int tipR   = rOuter - 4;
    int tailR  = max(8, (rOuter - arcThick) / 2);
    int halfW  = max(3, arcThick / 3);

    int xTip = (int)(cx + cosf(ar) * tipR);
    int yTip = (int)(cy - sinf(ar) * tipR);
    int xBaseC = (int)(cx - cosf(ar) * tailR);
    int yBaseC = (int)(cy + sinf(ar) * tailR);

    float apr = ar + 3.14159265358979323846f / 2.0f;
    int xB1 = (int)(xBaseC + cosf(apr) * halfW);
    int yB1 = (int)(yBaseC - sinf(apr) * halfW);
    int xB2 = (int)(xBaseC - cosf(apr) * halfW);
    int yB2 = (int)(yBaseC + sinf(apr) * halfW);

    spr.fillTriangle(xTip, yTip, xB1, yB1, xB2, yB2, COL_NEEDLE);
    spr.fillCircle(cx, cy, max(arcThick / 3, 5), COL_HUB);

    spr.pushSprite(x, y);
    spr.deleteSprite();
  };

  // ---------- 3 Meter ----------
  // PV (unipolar, grün wenn >PV_DB)
  drawGauge(xPV,   gaugesY, gW, gH, pvW,   pvMin,  pvMax,  "PV",   /*bipolar=*/false, PV_DB,   /*positiveIsGreen=*/true);
  // Batt (bipolar, ±25 W Deadband, +C grün / -D rot)
  drawGauge(xBATT, gaugesY, gW, gH, battW, batMin, batMax, "Batt", /*bipolar=*/true,  BATT_DB, /*positiveIsGreen=*/true);
  // Grid (bipolar, ±25 W Deadband, +Export grün / -Import rot)
  drawGauge(xGRID, gaugesY, gW, gH, gridW, gridMin, gridMax,"Grid", /*bipolar=*/true,  GRID_DB, /*positiveIsGreen=*/true);

  // ---------- Untertexte ----------
  // PV heute (integriert) – zentriert unter dem PV-Zeiger
  drawCentered(xPV + gW/2, gaugesY + gH + 2 - 30, fmtKWh(pvToday), TFT_YELLOW);

  // Batt: WR-Temperatur, Load (W)
  drawLabel(xBATT, gaugesY + gH +  2 - 30, "Temp",   fmtTemp(tempC),   TFT_LIGHTGREY);
  drawLabel(xBATT, gaugesY + gH + 16 - 30, "Load",   fmtW(loadW),      TFT_LIGHTGREY);
  // Batt Σ Load heute (integriert) – zentriert unter dem Batt-Zeiger (gleiche Zeile wie Exp)
  drawCentered(xBATT + gW/2, gaugesY + gH, fmtKWh(loadToday), TFT_CYAN);

  // Grid: T1 (rot), T2 (blau), Export (grün)
  float t1=-1.f, t2=-1.f, expK=-1.f;
  if (&pvGetTodaySplits){ float a,b; if (pvGetTodaySplits(a,b)) { t1=a; t2=b; } }
  if (&pvGetTodayExport){ float e;   if (pvGetTodayExport(e))   { expK=e; } }
  if (expK<0.f) expK = f.gridExpToday; // Fallback: Export-Tag aus Frame

  int gy = gaugesY + gH + 2 - 30;
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(2); tft.setTextSize(1);

  tft.setTextColor(TFT_RED,   TFT_BLACK);
  tft.drawString("T1", xGRID, gy);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString((t1>=0.f)? fmtKWh(t1) : String("--"), xGRID + 28, gy); gy += 14;

  tft.setTextColor(TFT_BLUE,  TFT_BLACK);
  tft.drawString("T2", xGRID, gy);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString((t2>=0.f)? fmtKWh(t2) : String("--"), xGRID + 28, gy); gy += 14;

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Exp", xGRID, gy);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtKWh(expK), xGRID + 28, gy);

  // PV: Tages-Verlust/Gewinn in CHF (Gewinn = negativ) – zentriert, gleiche Linie wie Load Σ & Exp
  bool chfValid = (t1>=0.f && t2>=0.f && !isnan(expK));
  if (chfValid){
    float chf = t1*t1Preis + t2*t2Preis - expK*expPreis; // Gewinn < 0
    uint16_t col = (chf < 0.f) ? TFT_GREEN : TFT_RED;
    drawCentered(xPV + gW/2, gaugesY + gH, fmtCHF(chf), col);
  } else {
    drawCentered(xPV + gW/2, gaugesY + gH, String("-- CHF"), TFT_DARKGREY);
  }
}

// ------------------- Seite 6 --------------------------------------------------------------
// === Seite 6: 30-Tage- oder Monats-Balken (Export oben grün; Bezug unten T1 rot + T2 blau) ===
// NVS: Namespace "pvstats", Keys je Tag: DYYYYMMDD  (DayAgg-Blob)
//      bzw. je Monat: MYYYYMM    (DayAgg-Blob als Monatsaggregation)
static void drawPage6Content(TFT_eSPI& tft, const PvFrameV4& , int kind) {
  #include <Preferences.h>
  #include <time.h>

  auto ymdFromTime = [](time_t tt)->uint32_t {
    struct tm tmv; localtime_r(&tt, &tmv);
    return (uint32_t)((tmv.tm_year+1900)*10000 + (tmv.tm_mon+1)*100 + tmv.tm_mday);
  };
  auto prevDay = [](time_t tt)->time_t { return tt - 86400; };

  struct DayAgg { // Tageswerte
    float gen_kWh;
    float load_kWh;
    float impT1_kWh;
    float impT2_kWh;
    float exp_kWh;
  };

  auto decMonth = [&](int &y, int &m){
    m--; if (m < 1) { m = 12; y--; }
  };

  auto keyDay = [](int y,int m,int d)->String {
    char b[16];
    snprintf(b,sizeof(b),"D%04d%02d%02d", y,m,d);
    return String(b);
  };

  auto keyMon =  [](int y,int m)->String { 
    char b[16]; 
    snprintf(b,sizeof(b),"M%04d%02d",y,m); 
    return String(b); 
  };

  // ---- Geometrie / Layout ----
  const int W = 320, H = 240;
  const int PAD_X = 6;
  const int AXIS_W = 46;          // linke Achsenspalte (mehr Platz für größere Y-Beschriftung)
  const int topY  = 42;           // unterhalb Statusheader
  const int botPad = 30;          // Platz für Legende/Marks
  const int left = PAD_X + AXIS_W, right = W - PAD_X;
  const int top = topY, bottom = H - botPad;
  const int plotW = right - left;
  const int plotH = bottom - top;
  int zeroY = (top + bottom) / 2; 

  // Hintergrund & Rahmen
  tft.fillRect(0, top, W, bottom - top, TFT_BLACK);
  tft.drawRect(left-1, top-1, plotW+2, plotH+2, TFT_DARKGREY);

  // ---- Datencontainer ----
  float t1v[30], t2v[30], expv[30];
  for (int i=0;i<30;++i){ t1v[i]=t2v[i]=expv[i]=0.f; }
  int n = 0;
  int idxToday = -1;
  float kosten;

  // ---- NVS lesen: 30 Tage bzw. 12 Monate rückwärts, dann alt->neu sortieren ----
  bool have = false;
  {
    Preferences prefs;
    if (prefs.begin("pvstats", true)) {
      struct DayRec { float t1,t2,exp; bool ok; } tmp[30];
      for (int i=0;i<30;++i){ tmp[i]={0,0,0,false}; }

      time_t nowT; time(&nowT);
      time_t cur = nowT;
      int got = 0;
      // --- Zeit / aktueller Monat ---
      time_t nowTs; time(&nowTs);
      struct tm lt; localtime_r(&nowTs, &lt);
      int cy = lt.tm_year + 1900;
      int cm = lt.tm_mon + 1;              // 1..12

      for (int i=0;i<30; ++i) {
        uint32_t ymd = ymdFromTime(cur);
        int y = ymd / 10000;
        int m = (ymd / 100) % 100;
        int d = ymd % 100;
        
        String key;
        // === Schlüssel wie beim Schreiben verwenden ===
        if (kind==tagesAnzeige) key = keyDay(y, m, d);   // gleicher Key wie in saveDayToNVS()
        if (kind==monatsAnzeige) key = keyMon(cy, cm);   // gleicher Key wie in saveMonToNVS()

        DayAgg a{}; 
        bool ok = false;
        // ganzen DayAgg-Blob lesen und Felder übernehmen
        if (prefs.getBytesLength(key.c_str()) == sizeof(DayAgg) &&
            prefs.getBytes(key.c_str(), &a, sizeof(DayAgg)) == sizeof(DayAgg)) {
          tmp[i].t1  = a.impT1_kWh;
          tmp[i].t2  = a.impT2_kWh;
          tmp[i].exp = a.exp_kWh;
          ok = true;
        }
        tmp[i].ok = ok;
        ++got; // gezählt, auch wenn nicht ok (Platzhalter)

        if (kind==tagesAnzeige) cur = prevDay(cur);
        if (kind==monatsAnzeige) decMonth(cy,cm);
      }
      prefs.end();

      if (got>0) {
        int k=0;
        int ruz;
        if (kind==tagesAnzeige) ruz = 29;
        if (kind==monatsAnzeige) ruz = 11;
        kosten = 0;
        for (int i=ruz;i>=0;--i) {
          if(true) {
            t1v[k]=tmp[i].t1; t2v[k]=tmp[i].t2; expv[k]=tmp[i].exp; 
            kosten = kosten + t1v[k]*t1Preis + t2v[k]*t2Preis - expv[k]*expPreis;
            ++k;
          }
        }
        n = k;
        if (n>0) { if (n>30) n=30; idxToday = n-1; have = true; }
      }
    }
  }
  if (kind==tagesAnzeige) n=30;
  if (kind==monatsAnzeige) n=12; 

  if (!have || n<=0) {
    // Platzhalter
    tft.setTextDatum(MC_DATUM);
    tft.setTextFont(2); tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("Keine 30-Tage-Historie vorhanden", W/2, (top+bottom)/2);

    int legY = bottom + 6;
    tft.setTextDatum(TL_DATUM);
    tft.setTextFont(2); tft.setTextSize(1);
    tft.setTextColor(TFT_RED,   TFT_BLACK); tft.drawString("T1",  left,      legY);
    tft.setTextColor(TFT_BLUE,  TFT_BLACK); tft.drawString("T2",  left+34,   legY);
    tft.setTextColor(TFT_GREEN, TFT_BLACK); tft.drawString("Exp", left+68,   legY);
    return;
  }

  // ---- Skala bestimmen ----
  float maxExp=0.f, maxImp=0.f;
  for (int i=0;i<n;++i) {
    if (expv[i] > maxExp) maxExp = expv[i];
    float imp = t1v[i] + t2v[i];
    if (imp > maxImp) maxImp = imp;
  }
  if (maxExp < 0.001f) maxExp = 0.001f;
  if (maxImp < 0.001f) maxImp = 0.001f;
  maxExp *= 1.10f;  // Headroom
  maxImp *= 1.10f;

  // ---- Balkenbreite / Abstände ----
  const int gap = 1;                                // kleiner Abstand
  int barW = ( (right - left) - (n-1)*gap ) / n;    // gleichmäßig aufteilen
  if (barW < 3) barW = 3;                           // mind. 3 Pixel

  // ---- Farben ----
  const uint16_t COL_EXP     = TFT_GREEN;
  const uint16_t COL_T1      = TFT_RED;
  const uint16_t COL_T2      = TFT_BLUE;
  const uint16_t COL_OUTLINE = TFT_WHITE;

  // Null-Linie (gewichtet durch Maxima oben/unten)
  zeroY = top + (int)(plotH * (maxExp / (maxExp + maxImp)) + 0.5f);
  tft.drawLine(left, zeroY, right, zeroY, TFT_LIGHTGREY);

  // ---- Y-Mapping ----
  auto yFromPos = [&](float kwh)->int {
    float t = kwh / maxExp; if (t<0) t=0; if (t>1) t=1;
    int hPix = (int)(t * (zeroY - top));
    return zeroY - hPix;
  };
  auto yFromNeg = [&](float kwh)->int {
    float t = kwh / maxImp; if (t<0) t=0; if (t>1) t=1;
    int hPix = (int)(t * (bottom - zeroY));
    return zeroY + hPix;
  };

  // ---- Balken zeichnen ----
  int x = left;
  for (int i=0;i<n;++i) {
    // Export (oben, grün)
    if (expv[i] > 0.0f) {
      int yTop = yFromPos(expv[i]);
      int hPix = zeroY - yTop;
      if (hPix > 0) tft.fillRect(x, yTop, barW, hPix, COL_EXP);
    }
    // Import (unten): T2 (blau) unten, darauf T1 (rot)
    float t2k = t2v[i] > 0 ? t2v[i] : 0.f;
    float t1k = t1v[i] > 0 ? t1v[i] : 0.f;

    if (t2k > 0.0f) {
      int yBotT2 = yFromNeg(t2k);
      int hT2    = yBotT2 - zeroY;
      if (hT2 > 0) tft.fillRect(x, zeroY+1, barW, hT2-1, COL_T2);
    }
    if (t1k > 0.0f) {
      float sum = t2k + t1k;
      int yBotSum = yFromNeg(sum);
      int yBotT2  = yFromNeg(t2k);
      int hT1     = yBotSum - yBotT2;
      if (hT1 > 0) tft.fillRect(x, yBotT2, barW, hT1, COL_T1);
    }

    // Heute markieren
    if (i == idxToday) {
      if (expv[i] > 0.0f) {
        int yTop = yFromPos(expv[i]);
        int hPix = zeroY - yTop;
        if (hPix > 0) tft.drawRect(x, yTop, barW, hPix, COL_OUTLINE);
      }
      float sumImp = t1k + t2k;
      if (sumImp > 0.0f) {
        int yBot = yFromNeg(sumImp);
        int hPix = yBot - zeroY;
        if (hPix > 0) tft.drawRect(x, zeroY+1, barW, hPix-1, COL_OUTLINE);
      }
    }

    x += barW + gap;
  }

  // ---- Y-Labels (größer) ----
  tft.setTextDatum(MR_DATUM);
  tft.setTextFont(2); tft.setTextSize(2);                 // größer als zuvor
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  { char b[20]; dtostrf(maxExp,0,(maxExp<10.f?1:0),b); tft.drawString(String(b), left-AXIS_W/2 +15, top,    1); }
  tft.drawString("kWh", left-AXIS_W/2+15, zeroY, 1);
  { char b[20]; dtostrf(maxImp,0,(maxImp<10.f?1:0),b); tft.drawString(String(b), left-AXIS_W/2 +15, bottom, 1); }

  // ---- X-Marks alle 5 Balken ----
  for (int i=0, xx=left; i<n; ++i, xx += (barW+gap)) {
    if (i % 5 == 0) tft.drawLine(xx, bottom+1, xx, bottom+5, TFT_DARKGREY);
  }

  // ---- Legende ----
  int legY = bottom + 6;
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(2); tft.setTextSize(1);
  tft.setTextColor(TFT_RED,   TFT_BLACK); tft.drawString("T1",  left,      legY);
  tft.setTextColor(TFT_BLUE,  TFT_BLACK); tft.drawString("T2",  left+34,   legY);
  tft.setTextColor(TFT_GREEN, TFT_BLACK); tft.drawString("Exp", left+68,   legY);
  tft.setTextColor(TFT_YELLOW,TFT_BLACK); tft.drawString(String(kosten), left+130,   legY);
}

// ------------------------- Seitensteuerung -------------------------
static constexpr int PV_MAX_PAGES = 5; // 0..4 sichtbar
static inline int pvMaxPages(){ return PV_MAX_PAGES; }

// Öffentliche API: rendert Header, löscht Inhalt, rendert Seite
static inline void drawPvPage(TFT_eSPI& tft, const PvFrameV4& f, int page){
  // lokales clearContent (gekapselt)
  auto clearContentArea = [&](TFT_eSPI& t){
    const int y = headerLineY + 1;
    t.fillRect(0, y, W, H - y, TFT_BLACK);
  };

  // 1) Header
  drawStatusHeader(tft, f);
  // 2) Inhaltsbereich freiräumen
  clearContentArea(tft);
  // 3) Seite rendern (nur Inhalt)
  switch(page){
    default:
    case 0: drawPage5Content(tft,f); break; 
    case 1: drawPage2Content(tft,f); break; 
    case 2: drawPage6Content(tft,f,tagesAnzeige); break; 
    case 3: drawPage6Content(tft,f,monatsAnzeige); break; 
    case 4: drawPage3Content(tft,f); break; 
  }
}
