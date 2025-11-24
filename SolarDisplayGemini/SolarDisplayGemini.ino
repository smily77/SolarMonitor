// ===================================================================================
// SOLAR DISPLAY COMPLETE (Huawei Sun 2000) - FINAL v4 (Poller + Layout Fixes)
// Hardware: CYD (ESP32-2432S028)
// ===================================================================================

#include <Arduino.h>

/************* ROLLE AUSWÄHLEN *************/
//#define ROLE_POLLER    // <--- EINKOMMENTIEREN = Poller (Sender); AUSKOMMENTIEREN = Client (Empfänger)
/*******************************************/

// ================= KONFIGURATION & DEFINES =================
#define PV_MAGIC   0xBEEF
#define PV_VERSION 4

// Preise
#define EXP_PREIS 0.138f
#define T1_PREIS  0.344f
#define T2_PREIS  0.2597f

// Anzeige-Modi
#define VIEW_DAY   1
#define VIEW_MONTH 2

// WLAN Credentials laden (oder unten im setup hardcoden)
#include <Credentials.h> 

// TFT & Hardware Setup
#define USER_SETUP_LOADED
#include "User_Setup.h" 

#ifndef TFT_BL
  #define TFT_BL 21
#endif

#include <TFT_eSPI.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <time.h>
#include <Preferences.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#ifdef ROLE_POLLER
  // Modbus nur für Poller Rolle benötigt
  #include <ModbusIP_ESP8266.h> 
#endif

// Touch Pins (CYD Standard)
#define XPT_IRQ   36
#define XPT_MOSI  32
#define XPT_MISO  39
#define XPT_CLK   25
#define XPT_CS    33

// Instanzen
TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI(VSPI);
XPT2046_Touchscreen touchscreen(XPT_CS, XPT_IRQ);

// ================= PROTOKOLL DEFINITIONEN (NICHT ÄNDERN) =================

// Wichtig: Inverter-Daten müssen in diesem Frame gesammelt werden
typedef struct __attribute__((packed)) {
  uint16_t magic; uint8_t version; uint32_t seq; uint32_t ts;
  int32_t  pvW; int32_t gridW; int32_t battW; int32_t loadW;
  int16_t  temp10; uint16_t socx10;
  float    pvTodayKWh; float gridExpToday; float gridImpToday; float loadTodayKWh;
  int32_t  eta20s;
  int16_t  pv1Voltage_x10_V; int16_t pv1Current_x10_A;
  int16_t  pv2Voltage_x10_V; int16_t pv2Current_x10_A;
  int32_t  gridVoltageA_x10_V; int32_t gridVoltageB_x10_V; int32_t gridVoltageC_x10_V;
  int32_t  gridCurrentA_x100_A; int32_t gridCurrentB_x100_A; int32_t gridCurrentC_x100_A;
  uint16_t crc;
} PvFrameV4;

// Datenspeicher Strukturen
struct DayAgg   { float gen_kWh; float load_kWh; float impT1_kWh; float impT2_kWh; float exp_kWh; };
struct MonthAgg { float gen_kWh; float load_kWh; float impT1_kWh; float impT2_kWh; float exp_kWh; };

// Netzwerk Adressen
static const IPAddress MCAST_GRP(239, 12, 12, 12);
static const uint16_t  MCAST_PORT = 55221;

// ================= GLOBALE VARIABLEN =================
static PvFrameV4 lastF{};
static bool      haveFrame = false;
static uint32_t  lastSeq = 0;
static int       pageIndex = 0;

// Speicher & Logik
static DayAgg    dayAgg = {0,0,0,0,0};
static MonthAgg  monthAgg = {0,0,0,0,0};
static int       curY=0, curM=0, curD=0;

AsyncUDP udpFrame;
Preferences prefs;

// Layout Konstanten
const int W=320, H=240;
const int STATUS_H=32;
const int HEADER_LINE_Y = STATUS_H + 2;

#ifdef ROLE_POLLER
  ModbusIP mb;
  // Hier bitte die IP-Adresse des Huawei Inverters anpassen
  IPAddress inverterIP(192,168,0,10); 
  const uint16_t modbusPort = 502;
#endif

// ================= HILFSFUNKTIONEN =================

// CRC16 Modbus
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

// Datum & Zeit String
String getStrTime() {
  time_t n; time(&n); struct tm ti; localtime_r(&n, &ti);
  char b[8]; snprintf(b, sizeof(b), "%02d:%02d", ti.tm_hour, ti.tm_min);
  return String(b);
}

String getStrDate() {
  time_t n; time(&n); struct tm ti; localtime_r(&n, &ti);
  static const char* wd[7] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};
  char b[24];
  snprintf(b, sizeof(b), "%s %02d.%02d.%04d", wd[ti.tm_wday], ti.tm_mday, ti.tm_mon + 1, ti.tm_year + 1900);
  return String(b);
}

// Formatter
String fmtWatt(float w) {
  if (fabsf(w) >= 1000.0f) return String(w / 1000.0f, (fabsf(w)<10000)?2:1) + " kW";
  return String((int)w) + " W";
}

String fmtKWh(float v) {
  if (isnan(v)) return "--";
  char b[16]; dtostrf(v, 0, (v < 10.0f ? 2 : 1), b);
  String s(b); s.trim();
  return s + " kWh";
}

String fmtCHF(float v) {
  char b[16]; dtostrf(v, 0, 2, b);
  return String(b) + " CHF";
}

String fmtETA(int32_t seconds) {
  if (seconds <= 0 || seconds > 172800) return "--:--"; // Plausibilitätscheck
  time_t now; time(&now);
  time_t et = now + seconds;
  struct tm ti; localtime_r(&et, &ti);
  char b[8]; snprintf(b, sizeof(b), "%02d:%02d", ti.tm_hour, ti.tm_min);
  return String(b);
}

// Datum Helfer für Historie
uint32_t ymdFromTime(time_t tt) {
  struct tm tmv; localtime_r(&tt, &tmv);
  return (uint32_t)((tmv.tm_year+1900)*10000 + (tmv.tm_mon+1)*100 + tmv.tm_mday);
}
time_t prevDay(time_t tt) { return tt - 86400; }
void decMonth(int &y, int &m){ m--; if (m < 1) { m = 12; y--; } }

String keyDay(int y,int m,int d){ char b[16]; snprintf(b,sizeof(b),"D%04d%02d%02d", y,m,d); return String(b); }
String keyMon(int y,int m){ char b[16]; snprintf(b,sizeof(b),"M%04d%02d",y,m); return String(b); }

// ================= DISPLAY FUNKTIONEN =================

void drawStatusHeader(const PvFrameV4& f) {
  bool ok20 = (f.socx10 >= 200);
  uint16_t bg = ok20 ? TFT_DARKGREEN : TFT_MAROON;
  uint16_t fg = 0xA554; // MidGrey

  tft.fillRect(0, 0, W, STATUS_H, bg);
  
  // Zeit
  tft.setTextDatum(ML_DATUM); tft.setTextFont(2); tft.setTextSize(2); tft.setTextColor(fg, bg);
  tft.drawString(getStrTime(), 8, STATUS_H / 2);

  // Batterie Icon
  const int iconW = 60, iconH = 24;
  const int iconX = W/2 - iconW/2 - 25, iconY = STATUS_H/2 - iconH/2;
  tft.drawRect(iconX, iconY, iconW, iconH, fg);
  tft.fillRect(iconX + iconW, iconY + iconH/4, 4, iconH/2, fg);
  int soc = f.socx10 / 10;
  int fillW = ((iconW - 4) * constrain(soc, 0, 100) / 100);
  tft.fillRect(iconX + 2, iconY + 2, fillW, iconH - 4, fg);
  tft.setTextDatum(MC_DATUM); tft.setTextFont(1); tft.setTextSize(2); tft.setTextColor(TFT_GREEN);
  tft.drawString(String(soc) + "%", iconX + iconW/2, iconY + iconH/2);

  // ETA
  tft.setTextDatum(MR_DATUM); tft.setTextFont(1); tft.setTextSize(2); tft.setTextColor(fg, bg);
  tft.drawString("ETA: " + fmtETA(f.eta20s), W - 8, STATUS_H / 2);
  tft.drawLine(8, HEADER_LINE_Y, W - 8, HEADER_LINE_Y, TFT_DARKGREY);
}

// Helper für Seite 2
void drawBarLine(int y, int w, int h, int32_t val, int32_t maxVal, uint16_t color, String label, String subL, String subR) {
  int x = 8;
  if (maxVal <= 0) maxVal = 1;
  tft.drawRect(x, y, w, h, TFT_DARKGREY);
  int32_t v = constrain(val, 0, maxVal);
  int fillW = (int)((int64_t)v * (w - 2) / maxVal);
  if (fillW > 0) tft.fillRect(x + 1, y + 1, fillW, h - 2, color);

  tft.setTextFont(2); tft.setTextSize(1);
  tft.setTextDatum(ML_DATUM); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString(label, x, y - 4);
  tft.setTextDatum(MR_DATUM); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtWatt(val), x + w, y + h/2);

  if (subL.length() > 0) {
     tft.setTextDatum(MR_DATUM); tft.setTextColor(TFT_CYAN, TFT_BLACK);
     tft.drawString(subL + "  " + subR, x + w, y + h + 12);
  }
}

// SEITE 2: Balken
void drawPage2Content(const PvFrameV4& f) {
  int32_t pv1W = ((int32_t)f.pv1Voltage_x10_V * (int32_t)f.pv1Current_x10_A + 500) / 1000;
  int32_t pv2W = ((int32_t)f.pv2Voltage_x10_V * (int32_t)f.pv2Current_x10_A + 500) / 1000;
  String s1L = String(f.pv1Voltage_x10_V/10.0, 1) + "V"; String s1R = String(f.pv1Current_x10_A/100.0, 1) + "A";
  String s2L = String(f.pv2Voltage_x10_V/10.0, 1) + "V"; String s2R = String(f.pv2Current_x10_A/100.0, 1) + "A";

  tft.setTextDatum(ML_DATUM); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("PV-String Leistung", 8, HEADER_LINE_Y + 16);
  
  int yBase = HEADER_LINE_Y + 45;
  drawBarLine(yBase, W-16, 28, pv1W, 6000, TFT_YELLOW, "PV1", s1L, s1R);
  drawBarLine(yBase + 62, W-16, 28, pv2W, 6000, TFT_CYAN, "PV2", s2L, s2R);
  drawBarLine(yBase + 124, W-16, 28, f.pvW, 9000, TFT_ORANGE, "Total", "", "max 9kW");
}

// SEITE 3: Uhr
void drawPage3Content() {
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(4); tft.setTextSize(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(getStrTime(), W / 2, HEADER_LINE_Y + (H - HEADER_LINE_Y) / 2 - 16);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString(getStrDate(), W / 2, HEADER_LINE_Y + (H - HEADER_LINE_Y) / 2 + 16);
}

// SEITE 5: Gauges (Optimiert)
void drawGaugeNew(int x, int y, int w, int h, float val, float minV, float maxV, const char* label, bool bipolar, float deadband, bool posIsGreen) {
  static TFT_eSprite spr(&tft);
  static bool sprInit = false;
  if (!sprInit) { spr.setColorDepth(16); spr.createSprite(100, 160); sprInit = true; }
  
  spr.fillSprite(TFT_BLACK);

  int cx = w / 2; int cy = (int)(h * 0.44f);
  int rOuter = (min(w, h) / 2) - 4; int arcThick = 12;
  
  if (maxV == minV) maxV++;
  float range = maxV - minV;
  float vClamped = constrain(val, minV, maxV);
  float pct = (vClamped - minV) / range;
  float angle = 225.0f - (pct * 270.0f); // 225 -> -45
  
  uint16_t colArc = TFT_DARKGREY;
  if (!bipolar) {
    if (val > deadband) colArc = TFT_GREEN;
  } else {
    if (val > deadband) colArc = posIsGreen ? TFT_GREEN : TFT_RED;
    else if (val < -deadband) colArc = posIsGreen ? TFT_RED : TFT_GREEN;
  }

  // Hintergrund und Bogen
  spr.drawSmoothArc(cx, cy, rOuter, rOuter-arcThick, 45, 315, TFT_DARKGREY, TFT_BLACK, true);
  if (colArc != TFT_DARKGREY) spr.drawSmoothArc(cx, cy, rOuter, rOuter-arcThick, 45, 315, colArc, TFT_BLACK, true);

  // Ticks
  auto drawTick = [&](float angDeg, uint16_t c, int l) {
     float rad = angDeg * DEG_TO_RAD;
     int x0 = cx + cos(rad) * (rOuter - l); int y0 = cy - sin(rad) * (rOuter - l);
     int x1 = cx + cos(rad) * rOuter;       int y1 = cy - sin(rad) * rOuter;
     spr.drawLine(x0, y0, x1, y1, c);
  };
  drawTick(225, TFT_LIGHTGREY, arcThick+4);
  drawTick(-45, TFT_LIGHTGREY, arcThick+4);
  if (bipolar) drawTick(90, TFT_YELLOW, arcThick+6);

  spr.setTextDatum(TC_DATUM); spr.setTextFont(2); spr.setTextColor(TFT_WHITE); spr.drawString(label, cx, 2);
  spr.setTextDatum(MC_DATUM); spr.setTextFont(4);
  String sVal = (fabs(val) >= 1000) ? String(val/1000.0, (fabs(val)<10000)?1:0) + "k" : String((int)val);
  spr.drawString(sVal, cx, h - 45);

  float ar = angle * DEG_TO_RAD;
  int rTip = rOuter - 5; int xTip = cx + cos(ar) * rTip; int yTip = cy - sin(ar) * rTip;
  spr.drawWideLine(cx, cy, xTip, yTip, 3, TFT_WHITE, TFT_BLACK);
  spr.fillCircle(cx, cy, 4, TFT_LIGHTGREY);

  spr.pushSprite(x, y);
}

void drawPage5Content(const PvFrameV4& f) {
  int yG = HEADER_LINE_Y + 6;
  float loadW = f.pvW - f.gridW - f.battW;
  
  drawGaugeNew(5,   yG, 100, 160, f.pvW,   0,     9000, "PV",   false, 50, true);
  drawGaugeNew(110, yG, 100, 160, f.battW, -5000, 5000, "Batt", true,  25, true);
  drawGaugeNew(215, yG, 100, 160, f.gridW, -9000, 9000, "Grid", true,  25, true);

  // --- KORRIGIERTES LAYOUT ---
  // Start der Zeilen wird 5 Pixel unter dem Gauge-Ende (yG + 160) platziert und dann weiter nach unten verschoben.
  // yG + 160 = 200. Wir starten bei 205 und ziehen die Reihen zusammen.
  // yRow3 (Bottom Line) muss bei H - 8 = 232 sein (MC_DATUM)
  int yRow3 = H - 8; // 232
  int yRow2 = yRow3 - 18; // 214
  int yRow1 = yRow2 - 18; // 196 (Text Startpunkt. MC-Datum für die Gauge-Texte unten muss auf dieser Linie liegen.)

  tft.setTextFont(2); tft.setTextSize(1);

  // Links (PV)
  tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString(fmtKWh(dayAgg.gen_kWh), 55, yRow3);
  
  // Mitte (Batt/Load)
  int xMid = 110; 
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_LIGHTGREY); tft.drawString("Temp", xMid, yRow1);
  tft.setTextColor(TFT_WHITE);     tft.drawString(String(f.temp10/10.0, 1)+" C", xMid+38, yRow1);
  
  tft.setTextColor(TFT_LIGHTGREY); tft.drawString("Load", xMid, yRow2);
  tft.setTextColor(TFT_WHITE);     tft.drawString(fmtWatt(loadW), xMid+38, yRow2);
  
  tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString(fmtKWh(dayAgg.load_kWh), xMid + 50, yRow3);

  // Rechts (Grid)
  int xRight = 215;
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_RED);   tft.drawString("T1", xRight, yRow1);
  tft.setTextColor(TFT_WHITE); tft.drawString(fmtKWh(dayAgg.impT1_kWh), xRight+25, yRow1);
  
  tft.setTextColor(TFT_BLUE);  tft.drawString("T2", xRight, yRow2);
  tft.setTextColor(TFT_WHITE); tft.drawString(fmtKWh(dayAgg.impT2_kWh), xRight+25, yRow2);
  
  tft.setTextColor(TFT_GREEN); tft.drawString("Exp", xRight, yRow3);
  tft.setTextColor(TFT_WHITE); tft.drawString(fmtKWh(dayAgg.exp_kWh), xRight+25, yRow3);
}

// SEITE 6: Historie
void drawPage6Content(const PvFrameV4&, int kind) {
  const int top = HEADER_LINE_Y + 15;
  const int bottom = H - 25;
  const int left = 35;
  const int right = W - 5;
  const int pW = right - left;
  const int pH = bottom - top;
  
  tft.fillRect(0, top-10, W, H-(top-10), TFT_BLACK);
  tft.drawRect(left, top, pW, pH, TFT_DARKGREY);

  tft.setTextDatum(MC_DATUM); tft.setTextFont(2); tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString(kind == VIEW_DAY ? "30 Tage Verlauf" : "Jahresverlauf", W/2, top - 12);

  // Daten Laden
  float vExp[30], vImpT1[30], vImpT2[30];
  for(int i=0; i<30; i++) { vExp[i]=0; vImpT1[i]=0; vImpT2[i]=0; }
  
  int n = (kind == VIEW_DAY) ? 30 : 12;
  time_t curT; time(&curT);
  struct tm lt; localtime_r(&curT, &lt);
  int cy = lt.tm_year + 1900; int cm = lt.tm_mon + 1;

  struct Rec { float exp; float t1; float t2; } tmp[30];
  prefs.begin("pvstats", true); 
  for (int i=0; i<n; ++i) {
    String k;
    if (kind == VIEW_DAY) {
      uint32_t ymd = ymdFromTime(curT); k = keyDay(ymd/10000, (ymd/100)%100, ymd%100); curT = prevDay(curT);
    } else {
      k = keyMon(cy, cm); decMonth(cy, cm);
    }
    tmp[i] = {0,0,0};
    if (kind == VIEW_DAY) {
      DayAgg d; if (prefs.getBytes(k.c_str(), &d, sizeof(DayAgg)) == sizeof(DayAgg)) {
        tmp[i].exp = d.exp_kWh; tmp[i].t1 = d.impT1_kWh; tmp[i].t2 = d.impT2_kWh;
      }
    } else {
      MonthAgg m; if (prefs.getBytes(k.c_str(), &m, sizeof(MonthAgg)) == sizeof(MonthAgg)) {
        tmp[i].exp = m.exp_kWh; tmp[i].t1 = m.impT1_kWh; tmp[i].t2 = m.impT2_kWh;
      }
    }
  }
  prefs.end();

  // Arrays umdrehen (damit heute rechts ist) und Maxima finden
  float maxE=0.1, maxI=0.1;
  for(int i=0; i<n; i++) {
    vExp[i] = tmp[n-1-i].exp; 
    vImpT1[i] = tmp[n-1-i].t1;
    vImpT2[i] = tmp[n-1-i].t2;
    
    if(vExp[i] > maxE) maxE = vExp[i];
    float sumImp = vImpT1[i] + vImpT2[i];
    if(sumImp > maxI) maxI = sumImp;
  }
  
  // Nulllinie und Balken
  float totalRange = maxE + maxI; if(totalRange < 0.1) totalRange = 1.0;
  int zeroY = top + (int)(pH * (maxE / totalRange));
  tft.drawLine(left, zeroY, right, zeroY, TFT_LIGHTGREY);

  int barW = (pW / n) - 2; if (barW < 2) barW = 2;
  for (int i=0; i<n; i++) {
    int x = left + 1 + i * (pW / n);
    
    // Export (Grün, nach oben)
    if (vExp[i] > 0) {
      int hBar = (int)((vExp[i] / maxE) * (zeroY - top));
      if(hBar > 0) tft.fillRect(x, zeroY - hBar, barW, hBar, TFT_GREEN);
    }
    
    // Import (Nach unten gestapelt: Erst T2 (Blau), dann T1 (Rot) darüber)
    if (vImpT2[i] > 0 || vImpT1[i] > 0) {
        int hT2 = 0;
        // T2 (Blau)
        if (vImpT2[i] > 0) {
            hT2 = (int)((vImpT2[i] / maxI) * (bottom - zeroY));
            if(hT2 > 0) tft.fillRect(x, zeroY + 1, barW, hT2, TFT_BLUE);
        }
        // T1 (Rot) - beginnt unter T2
        if (vImpT1[i] > 0) {
            // Die Höhe der T1-Bar muss relativ zu T2 beginnen
            int hT1 = (int)((vImpT1[i] / maxI) * (bottom - zeroY));
            if (hT1 > 0) tft.fillRect(x, zeroY + 1 + hT2, barW, hT1, TFT_RED);
        }
    }
  }

  // Achsen Labels (Klein!)
  tft.setTextFont(1); tft.setTextSize(1); tft.setTextDatum(MR_DATUM); tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString(String((int)maxE)+" kWh", left - 2, top);
  tft.drawString(String((int)maxI)+" kWh", left - 2, bottom);

  // Legende
  tft.setTextDatum(TL_DATUM); tft.setTextFont(2);
  int yLeg = bottom + 4;
  tft.setTextColor(TFT_RED);   tft.drawString("T1", 10, yLeg);
  tft.setTextColor(TFT_BLUE);  tft.drawString("T2", 45, yLeg);
  tft.setTextColor(TFT_GREEN); tft.drawString("Exp", 80, yLeg);
}

// ZENTRALER DISPATCHER
void drawPvPage(int page, const PvFrameV4& f) {
  drawStatusHeader(f);
  tft.fillRect(0, HEADER_LINE_Y + 1, W, H - (HEADER_LINE_Y + 1), TFT_BLACK);
  
  switch(page) {
    case 0: drawPage5Content(f); break; 
    case 1: drawPage2Content(f); break; 
    case 2: drawPage6Content(f, VIEW_DAY); break;
    case 3: drawPage6Content(f, VIEW_MONTH); break;
    case 4: drawPage3Content(); break; 
  }
}

// ================= LOGIK KERN =================

bool isT1_now() {
  struct tm ti; time_t n; time(&n); localtime_r(&n, &ti);
  // Hochtarif: Mo-Fr, 7:00 - 19:59
  if (ti.tm_wday == 0 || ti.tm_wday == 6) return false; 
  return (ti.tm_hour >= 7 && ti.tm_hour < 20);
}

void integrateTick(int32_t pv, int32_t grid, int32_t batt) {
  static uint32_t lastMs = 0;
  if (lastMs == 0) { lastMs = millis(); return; }
  uint32_t now = millis(); uint32_t dt = now - lastMs;
  if (dt < 100) return; 
  lastMs = now;
  float hours = dt / 3600000.0f;
  
  if (pv > 0) dayAgg.gen_kWh += (pv * hours) / 1000.0f;
  if (grid > 0) dayAgg.exp_kWh += (grid * hours) / 1000.0f;
  else          {
      float imp = (-grid * hours) / 1000.0f;
      if (isT1_now()) dayAgg.impT1_kWh += imp; else dayAgg.impT2_kWh += imp;
  }
  float load = pv - grid - batt;
  if (load > 0) dayAgg.load_kWh += (load * hours) / 1000.0f;
}

// Speichern (vereinfacht)
void checkDayMonthRollover() {
  time_t now; time(&now); struct tm ti; localtime_r(&now, &ti);
  int y=ti.tm_year+1900, m=ti.tm_mon+1, d=ti.tm_mday;
  
  if (curD != d) { // Neuer Tag
    prefs.begin("pvstats", false);
    String kd = keyDay(curY, curM, curD);
    prefs.putBytes(kd.c_str(), &dayAgg, sizeof(DayAgg));
    
    // Aggregiere Tageswerte in Monats-Aggregat
    monthAgg.gen_kWh += dayAgg.gen_kWh;
    monthAgg.load_kWh += dayAgg.load_kWh;
    monthAgg.impT1_kWh += dayAgg.impT1_kWh;
    monthAgg.impT2_kWh += dayAgg.impT2_kWh;
    monthAgg.exp_kWh += dayAgg.exp_kWh;

    if (curM != m) { // Neuer Monat
        String km = keyMon(curY, curM);
        prefs.putBytes(km.c_str(), &monthAgg, sizeof(MonthAgg));
        monthAgg = {0,0,0,0,0};
    }
    prefs.end();
    
    curY=y; curM=m; curD=d;
    dayAgg = {0,0,0,0,0};
  }
}

// Touch Handling
void handleTouch() {
  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    static uint32_t lastT = 0;
    if (millis() - lastT > 300) {
       pageIndex++; if (pageIndex > 4) pageIndex = 0;
       drawPvPage(pageIndex, lastF);
       lastT = millis();
    }
  }
}

// ================= POLLER FUNKTIONEN (NUR WENN ROLE_POLLER DEFINIERT) =================

#ifdef ROLE_POLLER

// Modbus Callback zur Verarbeitung der gelesenen Register
void cb(Modbus::ResultCode rc, uint16_t* regs, uint16_t numRegs, int32_t reqID) {
    if (rc != Modbus::EX_SUCCESS) {
        Serial.printf("MB Error ID %d: %s\n", reqID, mb.errToString(rc));
        return;
    }

    if (reqID == 100 && numRegs >= 10) { // Hauptwerte (z.B. 37001)
        // Huawei Daten sind oft 32bit (2x 16bit Register) und big-endian.
        // Die genauen Adressen und Skalierungen müssen evtl. angepasst werden, 
        // hier werden Standard-Offsets verwendet.

        // Power Werte (W) - oft Register 37001-37004 (2x 32bit: pvW, gridW)
        // Wir nehmen an, dass die Polling-Software die Werte aufbereitet hat
        // und speichern sie direkt in lastF
        
        // Simuliert: Daten aus den Modbus-Registern in lastF schreiben
        lastF.pvW = (int32_t)regs[0];      // PV Power
        lastF.gridW = (int32_t)regs[2];    // Grid Power (Import negativ, Export positiv)
        lastF.battW = (int32_t)regs[4];    // Battery Power (Charge negativ, Discharge positiv)
        lastF.temp10 = (int16_t)regs[8];   // Temp x 10
        lastF.socx10 = (uint16_t)regs[9];  // SOC x 10
        lastF.eta20s = 0;                  // ETA kann separat gelesen werden, hier Platzhalter
    } 
    else if (reqID == 101 && numRegs >= 10) { // Tageswerte (z.B. 37015)
        // Tages-kWh Werte
        lastF.pvTodayKWh   = ((uint32_t)regs[1] << 16 | regs[0]) / 1000.0f; // PV Today kWh (x100)
        lastF.gridImpToday = ((uint32_t)regs[3] << 16 | regs[2]) / 1000.0f; // Grid Imp Today kWh
        lastF.gridExpToday = ((uint32_t)regs[5] << 16 | regs[4]) / 1000.0f; // Grid Exp Today kWh
        // loadTodayKWh muss oft berechnet werden, hier Platzhalter
        lastF.loadTodayKWh = lastF.pvTodayKWh + lastF.gridImpToday - lastF.gridExpToday; 
    }
    
    // Frame fertig stellen und senden, wenn alle Daten da sind
    static bool req100_ok = false;
    static bool req101_ok = false;
    if (reqID == 100) req100_ok = true;
    if (reqID == 101) req101_ok = true;

    if (req100_ok && req101_ok) {
        req100_ok = false; req101_ok = false; // Reset für nächsten Poll
        
        lastF.magic = PV_MAGIC;
        lastF.version = PV_VERSION;
        lastF.seq++;
        time(&lastF.ts);
        
        // LoadW berechnen (PV - Grid - Batt)
        lastF.loadW = lastF.pvW - lastF.gridW - lastF.battW;
        
        // CRC berechnen und senden
        lastF.crc = crc16_modbus((const uint8_t*)&lastF, sizeof(PvFrameV4) - sizeof(uint16_t));
        
        udpFrame.writeTo((uint8_t*)&lastF, sizeof(PvFrameV4), MCAST_GRP, MCAST_PORT);
        
        // Lokale Anzeige des Pollers aktualisieren
        haveFrame = true;
        integrateTick(lastF.pvW, lastF.gridW, lastF.battW);
        checkDayMonthRollover();
        drawPvPage(pageIndex, lastF); 
    }
}

#endif // ROLE_POLLER

// ================= SETUP & LOOP =================

void setup() {
  Serial.begin(115200);
  tft.init(); tft.setRotation(1); tft.fillScreen(TFT_BLACK);
  pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
  
  touchscreenSPI.begin(XPT_CLK, XPT_MISO, XPT_MOSI, XPT_CS);
  touchscreen.begin(touchscreenSPI); touchscreen.setRotation(1);

  tft.drawString("Verbinde WiFi...", W/2, H/2);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  
  configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3", "pool.ntp.org", "time.google.com");
  delay(1000);
  
  time_t n; time(&n); struct tm ti; localtime_r(&n, &ti);
  curY=ti.tm_year+1900; curM=ti.tm_mon+1; curD=ti.tm_mday;
  
  // Init NVS
  prefs.begin("pvstats", true);
  String k = keyDay(curY, curM, curD);
  if(prefs.getBytes(k.c_str(), &dayAgg, sizeof(DayAgg)) != sizeof(DayAgg)) dayAgg={0,0,0,0,0};
  prefs.end();

#ifdef ROLE_POLLER
  // Poller: Modbus & Multicast senden
  mb.client(); mb.onReply(cb);
  mb.connect(inverterIP, modbusPort);
  tft.fillScreen(TFT_BLACK); tft.drawString("Poller gestartet...", W/2, H/2);
#else
  // Client: Frames empfangen
  if(udpFrame.listenMulticast(MCAST_GRP, MCAST_PORT)) {
    udpFrame.onPacket([](AsyncUDPPacket packet) {
       if (packet.length() == sizeof(PvFrameV4)) {
         memcpy(&lastF, packet.data(), sizeof(PvFrameV4));
         // CRC Check (einfach, aber wichtig)
         uint16_t calcCrc = crc16_modbus((const uint8_t*)&lastF, sizeof(PvFrameV4) - sizeof(uint16_t));
         if (lastF.magic == PV_MAGIC && lastF.crc == calcCrc) {
            haveFrame = true;
            integrateTick(lastF.pvW, lastF.gridW, lastF.battW);
            checkDayMonthRollover();
            drawPvPage(pageIndex, lastF); 
         }
       }
    });
  }
  tft.fillScreen(TFT_BLACK); tft.drawString("Warte auf Daten...", W/2, H/2);
#endif
}

void loop() {
  #ifdef ROLE_POLLER
    static uint32_t lastConnTry=0;
    static uint32_t lastPollStart=0;
    const  uint32_t POLL_INTERVAL_MS=3000;
    
    if(!mb.isConnected(inverterIP)){
      if(millis()-lastConnTry>5000){
        mb.connect(inverterIP, modbusPort);
        lastConnTry=millis();
      }
    }else{
        mb.task(); // Modbus-Antworten verarbeiten
        if(millis()-lastPollStart>=POLL_INTERVAL_MS) { 
            lastPollStart=millis();
            // --- Modbus Polling (Huawei Sun2000 typische Adressen) ---
            // Unit ID: 1 (normalerweise)
            // 1. Hauptwerte (37001) - Power, Temp, SOC (10 Register)
            mb.readHreg(1, 37001, 10, 100); 
            // 2. Tageswerte (37015) - Today kWh (10 Register)
            mb.readHreg(1, 37015, 10, 101);
        }
    }
  #endif

  handleTouch();
  delay(10);
}
