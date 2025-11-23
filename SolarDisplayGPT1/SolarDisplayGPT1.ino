// SolarDisplayGPT1: vereinfachte Variante des Sketches
// Automatische Arduino-Prototypen deaktivieren, damit benutzerdefinierte
// Typen (z.B. PvFrameV4, StatsHdr) in Signaturen korrekt erkannt werden.
#define ARDUINO_NO_PROTOTYPES

// Vorwärtsdeklarationen, damit generierte Prototypen die benutzerdefinierten
// Typen erkennen, auch wenn die vollständigen Definitionen später folgen.
struct PvFrameV4;
struct StatsHdr;
struct DayAgg;
struct MonthAgg;
// Rolle zur Compile-Zeit wählen: Poller liest Werte aus und verteilt sie,
// Client zeigt lediglich empfangene Daten an.
//#define ROLE_POLLER    // einkommentieren = Poller; auskommentieren = Client

// ---- TFT_eSPI über lokalen User_Setup.h laden ----
#define USER_SETUP_LOADED
#include "User_Setup.h"

#ifndef TFT_BL
  #define TFT_BL 21
#endif
#ifndef TFT_ROTATION
  #define TFT_ROTATION 1
#endif

#include <TFT_eSPI.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <time.h>
#include <Preferences.h>
#include <Streaming.h>

#include <Credentials.h>

// Touch (XPT2046) – CYD Pins
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#define XPT2046_IRQ   36   // T_IRQ
#define XPT2046_MOSI  32   // T_DIN
#define XPT2046_MISO  39   // T_OUT
#define XPT2046_CLK   25   // T_CLK
#define XPT2046_CS    33   // T_CS
SPIClass touchscreenSPI(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

// ---- Eingebettetes PvCommon (vorher separate Header) ----
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

  const int vaY1 = y1 + barHeight + 2;
  char b1[32]; snprintf(b1,sizeof(b1),"%0.1f V   %0.2f A", pv1V, pv1A);
  tft.drawString(String(b1), barMarginX + barWidth, vaY1);

  // ---- PV2 ----
  drawHBar(barMarginX, y2, barWidth, barHeight, pv2W, MAX_W_STR, TFT_ORANGE);
  tft.setTextDatum(ML_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("PV2", barMarginX, y2 - 4);
  tft.setTextDatum(MR_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtWatt(pv2W), barMarginX + barWidth, y2 + barHeight/2);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_CYAN, TFT_BLACK);
  char b2[32]; snprintf(b2,sizeof(b2),"%0.1f V   %0.2f A", pv2V, pv2A);
  tft.drawString(String(b2), barMarginX + barWidth, y2 + barHeight + 2);

  // ---- Gesamt PV ----
  drawHBar(barMarginX, y3, barWidth, barHeight, pvTotalW, MAX_W_TOT, TFT_GREEN);
  tft.setTextDatum(ML_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("PV total", barMarginX, y3 - 4);
  tft.setTextDatum(MR_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtWatt(pvTotalW), barMarginX + barWidth, y3 + barHeight/2);

  // Linie unten
  tft.drawLine(PAD_X, y3 + barHeight + 6, W-PAD_X, y3 + barHeight + 6, TFT_DARKGREY);
}

// Seite 3 – Netzströme + Spannungen
static inline void drawPage3Content(TFT_eSPI& tft, const PvFrameV4& f){
  auto fmtA = [](int32_t a_x100)->String{
    float a = a_x100 / 100.0f;
    return String(a, 2) + " A";
  };
  auto fmtV = [](int32_t v_x10)->String{
    float v = v_x10 / 10.0f;
    return String(v, 1) + " V";
  };

  const int yTitle = startY + 8;
  tft.setTextDatum(ML_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Netz-Phasen", PAD_X, yTitle);

  const int baseY = yTitle + 16;
  const int lineH = 34;
  const int col1X = PAD_X;
  const int col2X = W/2 + 20;

  // Spalten-Labels
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Strom", col1X, baseY);
  tft.drawString("Spannung", col2X, baseY);

  auto drawRow = [&](const char* name, int32_t cur_x100, int32_t volt_x10, int y){
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString(name, PAD_X, y);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(fmtA(cur_x100), col1X, y);
    tft.drawString(fmtV(volt_x10), col2X, y);
  };

  drawRow("Phase A", f.gridCurrentA_x100_A, f.gridVoltageA_x10_V, baseY + lineH);
  drawRow("Phase B", f.gridCurrentB_x100_A, f.gridVoltageB_x10_V, baseY + 2*lineH);
  drawRow("Phase C", f.gridCurrentC_x100_A, f.gridVoltageC_x10_V, baseY + 3*lineH);

  // Linie unten
  tft.drawLine(PAD_X, baseY + 3*lineH + 8, W-PAD_X, baseY + 3*lineH + 8, TFT_DARKGREY);
}

// Anzeige der drei Haupt-Leistungen als horizontale Balken
static inline void drawMainBars(TFT_eSPI& tft, const PvFrameV4& f){
  auto drawBar = [&](int x, int y, int w, int h, int32_t valueW, int32_t maxAbsW, uint16_t colPos, uint16_t colNeg){
    tft.drawRect(x, y, w, h, TFT_DARKGREY);
    int32_t v = valueW;
    if (v > maxAbsW)  v = maxAbsW;
    if (v < -maxAbsW) v = -maxAbsW;
    int mid = x + w/2;
    if (v >= 0) {
      int fillW = (int)((int64_t)v * (w/2-2) / maxAbsW);
      if (fillW > 0) tft.fillRect(mid, y+1, fillW, h-2, colPos);
    } else {
      int fillW = (int)((int64_t)(-v) * (w/2-2) / maxAbsW);
      if (fillW > 0) tft.fillRect(mid - fillW, y+1, fillW, h-2, colNeg);
    }
  };
  auto fmtWatt = [](int32_t w)->String{
    return (abs(w)>=1000) ? String(w/1000.0,2)+" kW" : String(w)+" W";
  };

  const int barMarginX = PAD_X;
  const int barWidth   = W - 2*barMarginX;
  const int barHeight  = 28;
  const int gapY       = 34;

  const int y1 = startY + 14;
  const int y2 = y1 + barHeight + gapY;
  const int y3 = y2 + barHeight + gapY;

  const int32_t MAX_W = 6000; // +/-6 kW Skala

  // PV
  drawBar(barMarginX, y1, barWidth, barHeight, f.pvW, MAX_W, TFT_GREEN, TFT_DARKGREEN);
  tft.setTextDatum(ML_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("PV", barMarginX, y1 - 4);
  tft.setTextDatum(MR_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtWatt(f.pvW), barMarginX + barWidth, y1 + barHeight/2);

  // Grid
  drawBar(barMarginX, y2, barWidth, barHeight, f.gridW, MAX_W, TFT_GREEN, TFT_RED);
  tft.setTextDatum(ML_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Grid (+Export / -Import)", barMarginX, y2 - 4);
  tft.setTextDatum(MR_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtWatt(f.gridW), barMarginX + barWidth, y2 + barHeight/2);

  // Batt
  drawBar(barMarginX, y3, barWidth, barHeight, f.battW, MAX_W, TFT_GREEN, TFT_RED);
  tft.setTextDatum(ML_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Batterie (+Laden / -Entladen)", barMarginX, y3 - 4);
  tft.setTextDatum(MR_DATUM); tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(fmtWatt(f.battW), barMarginX + barWidth, y3 + barHeight/2);

  // Linie unten
  tft.drawLine(PAD_X, y3 + barHeight + 6, W-PAD_X, y3 + barHeight + 6, TFT_DARKGREY);
}

// Tabelle rechts oben mit den Haupt-Werten
static inline void drawMainTable(TFT_eSPI& tft, const PvFrameV4& f){
  auto drawRow = [&](int y, const char* label, const String& value){
    tft.setTextDatum(ML_DATUM);
    tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString(label, W/2 + 10, y);
    tft.setTextDatum(MR_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(value, W - PAD_X, y);
  };

  auto fmtWatt = [](int32_t w)->String{
    if (abs(w) >= 1000) return String(w/1000.0, 2) + " kW";
    return String(w) + " W";
  };

  auto fmtC = [](int16_t t10){ return String(t10/10.0,1) + " °C"; };
  auto fmtKWh = [](float v){ return String(v,2) + " kWh"; };

  int y = startY + 6;
  const int dy = 26;
  drawRow(y, "PV", fmtWatt(f.pvW));
  y += dy;
  drawRow(y, "Grid", fmtWatt(f.gridW));
  y += dy;
  drawRow(y, "Batterie", fmtWatt(f.battW));
  y += dy;
  drawRow(y, "Load", fmtWatt(f.loadW));
  y += dy;
  drawRow(y, "Temp", fmtC(f.temp10));
  y += dy;
  drawRow(y, "PV heute", fmtKWh(f.pvTodayKWh));
  y += dy;
  drawRow(y, "Grid Export", fmtKWh(f.gridExpToday));
  y += dy;
  drawRow(y, "Grid Import", fmtKWh(f.gridImpToday));
  y += dy;
  drawRow(y, "Load heute", fmtKWh(f.loadTodayKWh));
}

// Seite 1 – Hauptanzeige (Balken + Tabelle)
static inline void drawPage1(TFT_eSPI& tft, const PvFrameV4& f){
  drawStatusHeader(tft, f);
  drawMainBars(tft, f);
  drawMainTable(tft, f);
}

// Seite 2 – PV Strings
static inline void drawPage2(TFT_eSPI& tft, const PvFrameV4& f){
  drawStatusHeader(tft, f);
  drawPage2Content(tft, f);
}

// Seite 3 – Netz-Phasen
static inline void drawPage3(TFT_eSPI& tft, const PvFrameV4& f){
  drawStatusHeader(tft, f);
  drawPage3Content(tft, f);
}

// ===== Page-Dispatcher =====
static inline void drawPvPage(TFT_eSPI& tft, int page, const PvFrameV4& f){
  switch(page){
    case 0: drawPage1(tft,f); break;
    case 1: drawPage2(tft,f); break;
    case 2: drawPage3(tft,f); break;
    default: drawPage1(tft,f); break;
  }
}

static inline int pvMaxPages(){ return 3; }

// ---- Eingebettetes PvStats ----
#ifndef STATS_MCAST_GRP
  #define STATS_MCAST_GRP IPAddress(239, 0, 0, 58)
#endif
#ifndef STATS_MCAST_PORT
  #define STATS_MCAST_PORT 43210   // Discover / Offer Kanal (Multicast)
#endif
#ifndef STATS_SERVER_PORT
  #define STATS_SERVER_PORT 43211  // Unicast-Stream (Poller -> Client)
#endif

// ---- Nachrichten-Typen ----
enum : uint8_t {
  STATS_DISCOVER = 1,   // Client -> Multicast: "Wer ist Poller?"
  STATS_OFFER    = 2,   // Poller -> Client: "Ich hier, nimm Port X"
  STATS_REQ_RANGE= 3,   // Client -> Poller: "Schick mir alles (oder ab Zeit X)"
  STATS_DAY      = 4,   // Poller -> Client: Tages-Datensatz
  STATS_MON      = 5,   // Poller -> Client: Monats-Datensatz
  STATS_ACK      = 6,   // Client -> Poller: ACK für Seq (derzeit ungenutzt)
  STATS_DONE     = 7    // Poller -> Client: Ende des Streams
};

// ---- Header ----
struct StatsHdr {
  uint16_t magic;    // 0xCAFE
  uint8_t  version;  // 1
  uint8_t  type;     // STATS_*
  uint32_t seq;      // Sequenznummer
  uint16_t len;      // Payload-Länge
  uint16_t crc;      // CRC über Header (crc=0) + Payload
} __attribute__((packed));

// CRC16-CCITT (0x1021, start 0xFFFF)
inline uint16_t pvstats_crc(const StatsHdr& h, const uint8_t* payload){
  auto crc16 = [](const uint8_t* data, size_t len, uint16_t c=0xFFFF){
    for(size_t i=0;i<len;i++){
      c ^= (uint16_t)data[i] << 8;
      for(int k=0;k<8;k++) c = (c & 0x8000) ? (uint16_t)((c<<1) ^ 0x1021) : (uint16_t)(c<<1);
    }
    return c;
  };
  StatsHdr hc = h; hc.crc = 0;
  uint16_t c = crc16((const uint8_t*)&hc, sizeof(StatsHdr));
  if (h.len && payload) c = crc16(payload, h.len, c);
  return c;
}

// ---- Discover/Offer ----
struct PayloadOffer {
  uint16_t statsPort; // Unicast-Port des Pollers
  uint16_t rsv;
} __attribute__((packed));

struct PayloadReqRange {
  // 0 bedeutet "ab Beginn/alles"
  uint16_t fromY;    // ab Jahr
  uint8_t  fromM;    // ab Monat
  uint8_t  fromD;    // ab Tag
  uint16_t fromMonY; // ab Monat-Jahr für Monatsblöcke
  uint8_t  fromMonM; // ab Monat (1..12)
  uint8_t  rsv;
} __attribute__((packed));

struct PayloadAck {
  uint32_t ackSeq;
} __attribute__((packed));

// ---- WICHTIG: Payloads enthalten jetzt auch load_kWh ----
struct PayloadDay {
  uint16_t y, m, d;
  float    gen_kWh;     // PV Erzeugung (integriert)
  float    load_kWh;    // Load/Verbrauch (integriert)
  float    impT1_kWh;   // Netzbezug T1
  float    impT2_kWh;   // Netzbezug T2
  float    exp_kWh;     // Einspeisung
} __attribute__((packed));

struct PayloadMon {
  uint16_t y, m;
  float    gen_kWh;     // PV Erzeugung (Summen aller Tage im Monat)
  float    load_kWh;    // Verbrauch (Summen)
  float    impT1_kWh;
  float    impT2_kWh;
  float    exp_kWh;
} __attribute__((packed));

// ---- Zeitzone (Fallback) ----
#ifndef TZ_EU_ZURICH
  #define TZ_EU_ZURICH "CET-1CEST,M3.5.0/2,M10.5.0/3"
#endif

// ===== WLAN =====
//Comes from Credentials.h

// ===== Anzeige =====
TFT_eSPI tft;

// ===== UDP =====
AsyncUDP udpFrame;       // Multicast Frames (v4)
AsyncUDP udpStatsCtrl;   // Discover/Offer + Steuersignale (Multicast)
AsyncUDP udpStatsSrv;    // Unicast Server (Poller) oder leer (Client)

// ===== Gemeinsame Frame/State =====
static PvFrameV4 lastF{};
static bool      haveFrame=false;
static uint32_t  lastSeq=0;
static uint32_t  lastRxMs=0;

// ===== Seitensteuerung =====
static int pageIndex = 0;  // wird durch Swipe geändert

// ===== Touch/Swipe Konfiguration (Rotation 1: Landscape 320x240) =====
static const int SCREEN_W = 320;
static const int SCREEN_H = 240;
// Kalibrierung (wie getestet)
static const int TOUCH_X_MIN = 200;
static const int TOUCH_X_MAX = 3700;
static const int TOUCH_Y_MIN = 240;
static const int TOUCH_Y_MAX = 3800;

// Swipe-Erkennung
static const int SWIPE_MIN_DIST = 60;     // min. 60 px horizontal
static const int SWIPE_MAX_TIME = 1000;   // max. 1s
static const int TAP_MAX_DIST   = 15;     // kleiner Versatz -> Tap
static const int TOUCH_SAMPLES  = 3;      // Mittelung

static bool     tsDown     = false;
static int      tsStartX   = 0;
static int      tsStartY   = 0;
static uint32_t tsStartMs  = 0;
// während Touch letzte valide Pos merken (robust gegen Aussetzer)
static int      tsLastX    = 0;
static int      tsLastY    = 0;

// ===== Integrations-/Speicher-Modelle =====
struct DayAgg {   // Tageswerte
  float gen_kWh;
  float load_kWh;
  float impT1_kWh;
  float impT2_kWh;
  float exp_kWh;
};
struct MonthAgg { // Monatswerte
  float gen_kWh;
  float load_kWh;
  float impT1_kWh;
  float impT2_kWh;
  float exp_kWh;
};

static DayAgg   dayAgg = {0,0,0,0,0};
static MonthAgg monthAgg = {0,0,0,0,0};

Preferences prefs;

// Integrations-Zwischenwerte
static uint32_t lastIntMs = 0;
static int32_t  pvPrev=0, gridPrev=0, battPrev=0;

// Tages-/Monatsanker
static int curY=0, curM=0, curD=0;

// ===== Hooks für die Anzeigen =====
bool pvGetTodayPV(float& pv_kWh){ pv_kWh = dayAgg.gen_kWh; return true; }
bool pvGetTodayLoad(float& load_kWh){ load_kWh = dayAgg.load_kWh; return true; }
bool pvGetTodayExport(float& exp_kWh){ exp_kWh = dayAgg.exp_kWh; return true; }
bool pvGetTodaySplits(float& t1_kWh, float& t2_kWh){ t1_kWh = dayAgg.impT1_kWh; t2_kWh = dayAgg.impT2_kWh; return true; }
bool pvGetMonthTotals(float& pv_kWh, float& load_kWh, float& t1_kWh, float& t2_kWh, float& exp_kWh){
  pv_kWh=monthAgg.gen_kWh; load_kWh=monthAgg.load_kWh; t1_kWh=monthAgg.impT1_kWh; t2_kWh=monthAgg.impT2_kWh; exp_kWh=monthAgg.exp_kWh; return true;
}

// ===== Zeit/Helfer =====
static inline void nowLocal(struct tm& ti){ time_t n; time(&n); localtime_r(&n,&ti); }
static inline void todayYMD(int &y,int &m,int &d){ struct tm ti; nowLocal(ti); y=ti.tm_year+1900; m=ti.tm_mon+1; d=ti.tm_mday; }
static inline bool isLeap(int y){ return ((y%4==0)&&(y%100!=0)) || (y%400==0); }
static int daysInMonth(int y,int m){ static const uint8_t dm[12]={31,28,31,30,31,30,31,31,30,31,30,31}; return m==2? dm[m-1]+(isLeap(y)?1:0) : dm[m-1]; }
static inline bool isT1_now(){ struct tm ti; nowLocal(ti); return (ti.tm_wday>=1 && ti.tm_wday<=5) && (ti.tm_hour>=7 && ti.tm_hour<18); }

static void nvsBegin(){ static bool b=false; if(!b){ prefs.begin("pvstats", false); b=true; } }
static String keyDay(int y,int m,int d){ char b[16]; snprintf(b,sizeof(b),"D%04d%02d%02d",y,m,d); return String(b); }
static String keyMon(int y,int m){ char b[16]; snprintf(b,sizeof(b),"M%04d%02d",y,m); return String(b); }

static void saveDayToNVS(int y,int m,int d,const DayAgg& a){ nvsBegin(); prefs.putBytes(keyDay(y,m,d).c_str(), &a, sizeof(a)); }
static bool loadDayFromNVS(int y,int m,int d, DayAgg& a){ nvsBegin(); size_t got=prefs.getBytes(keyDay(y,m,d).c_str(), &a, sizeof(a)); return got==sizeof(a); }
static void saveMonthToNVS(int y,int m,const MonthAgg& a){ nvsBegin(); prefs.putBytes(keyMon(y,m).c_str(), &a, sizeof(a)); }
static bool loadMonthFromNVS(int y,int m, MonthAgg& a){ nvsBegin(); size_t got=prefs.getBytes(keyMon(y,m).c_str(), &a, sizeof(a)); if (got!=sizeof(a)) a={0,0,0,0,0}; return got==sizeof(a); }

// ===== Integration (trapez) =====
static void integrateTick(int32_t pvW, int32_t gridW, int32_t battW){
  uint32_t nowMs = millis();
  if (lastIntMs==0){ lastIntMs=nowMs; pvPrev=pvW; gridPrev=gridW; battPrev=battW; return; }
  uint32_t dt = nowMs - lastIntMs; if (!dt){ pvPrev=pvW; gridPrev=gridW; battPrev=battW; return; }
  lastIntMs = nowMs;

  // PV >= 0
  double pv0 = pvPrev>0 ? pvPrev : 0; double pv1 = pvW>0 ? pvW : 0;
  double ePv_Wh = ((pv0 + pv1) * 0.5) * (dt / 3600000.0);

  // Grid: Export>0, Import<0
  double g0 = gridPrev, g1 = gridW;
  double exp0 = g0>0 ? g0 : 0, exp1 = g1>0 ? g1 : 0;
  double imp0 = g0<0 ? -g0: 0, imp1 = g1<0 ? -g1: 0;
  double eExp_Wh = ((exp0 + exp1) * 0.5) * (dt / 3600000.0);
  double eImp_Wh = ((imp0 + imp1) * 0.5) * (dt / 3600000.0);

  // Load = PV - Grid - Batt (nur >=0 integrieren)
  double l0 = pvPrev - gridPrev - battPrev; if (l0<0) l0=0;
  double l1 = pvW    - gridW    - battW;    if (l1<0) l1=0;
  double eLoad_Wh = ((l0 + l1) * 0.5) * (dt / 3600000.0);

  // Tagesakkus
  dayAgg.gen_kWh  += ePv_Wh   / 1000.0;
  dayAgg.exp_kWh  += eExp_Wh  / 1000.0;
  dayAgg.load_kWh += eLoad_Wh / 1000.0;
  if (eImp_Wh>0){
    if (isT1_now()) dayAgg.impT1_kWh += eImp_Wh/1000.0;
    else            dayAgg.impT2_kWh += eImp_Wh/1000.0;
  }

  // Monatsakkus
  monthAgg.gen_kWh  += ePv_Wh   / 1000.0;
  monthAgg.exp_kWh  += eExp_Wh  / 1000.0;
  monthAgg.load_kWh += eLoad_Wh / 1000.0;
  if (eImp_Wh>0){
    if (isT1_now()) monthAgg.impT1_kWh += eImp_Wh/1000.0;
    else            monthAgg.impT2_kWh += eImp_Wh/1000.0;
  }

  pvPrev=pvW; gridPrev=gridW; battPrev=battW;
}

// ===== Tages-/Monatswechsel =====
static void handleDayMonthRollover(){
  int y,m,d; todayYMD(y,m,d);
  if (curY<=2000){ // init
    curY=y;curM=m;curD=d;
    loadMonthFromNVS(y,m, monthAgg);
    DayAgg tmp; if (loadDayFromNVS(y,m,d,tmp)) dayAgg=tmp;
    return;
  }
  if (d!=curD){
    // gestern sichern
    saveDayToNVS(curY,curM,curD, dayAgg);
    // Monatswechsel?
    if (curM!=m){
      saveMonthToNVS(curY,curM, monthAgg);
      monthAgg={0,0,0,0,0};
      loadMonthFromNVS(y,m, monthAgg); // evtl. laden (falls existiert)
    }
    // neuer Tag
    curY=y;curM=m;curD=d;
    dayAgg={0,0,0,0,0};
    DayAgg tmp; if (loadDayFromNVS(y,m,d,tmp)) dayAgg=tmp;
  }
}

// ===== Touch lesen =====
static bool readTouchAvg(int &x, int &y) {
  if (!(touchscreen.tirqTouched() && touchscreen.touched())) return false; // falls T_IRQ nicht angeschlossen -> nur touchscreen.touched()

  long sx = 0, sy = 0; int n = 0;
  for (int i=0; i<TOUCH_SAMPLES; ++i) {
    if (!(touchscreen.tirqTouched() && touchscreen.touched())) break;
    TS_Point p = touchscreen.getPoint(); // roher Wert
    int rx = map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 1, SCREEN_W);
    int ry = map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 1, SCREEN_H);
    if (rx < 1) rx = 1; if (rx > SCREEN_W) rx = SCREEN_W;
    if (ry < 1) ry = 1; if (ry > SCREEN_H) ry = SCREEN_H;
    sx += rx; sy += ry; n++;
    delay(2);
  }
  if (n == 0) return false;
  x = (int)(sx / n);
  y = (int)(sy / n);
  return true;
}

// ===== Swipe / Touch =====
static void handleTouchSwipe(){
  if (!haveFrame) return; // erst reagieren, wenn Daten da sind

  int x=0, y=0;
  bool touching = readTouchAvg(x,y);

  if (!tsDown) {
    if (touching) {
      tsDown    = true;
      tsStartX  = x;
      tsStartY  = y;
      tsLastX   = x;
      tsLastY   = y;
      tsStartMs = millis();
    }
    return;
  }

  if (touching) {
    tsLastX = x;
    tsLastY = y;
    return;
  }

  // Finger losgelassen
  tsDown = false;
  uint32_t dt = millis() - tsStartMs;

  int dx = tsLastX - tsStartX;
  int dy = tsLastY - tsStartY;

  if ((abs(dx) <= TAP_MAX_DIST) && (abs(dy) <= TAP_MAX_DIST)) return;

  if (dt <= SWIPE_MAX_TIME && abs(dx) >= SWIPE_MIN_DIST && abs(dx) > (abs(dy) * 1.2f)) {
    int maxPages = pvMaxPages();   // aus PvCommon.h
    int oldPage  = pageIndex;

    if (dx < 0) { // rechts -> links
      if (pageIndex < maxPages - 1) pageIndex++;
    } else {      // links -> rechts
      if (pageIndex > 0) pageIndex--;
    }

    if (pageIndex != oldPage) drawPvPage(tft, lastF, pageIndex);  // ganze Seite neu zeichnen
  }
}

// ====== Zeichnen ======
static void drawIfFrame(){
  if (!haveFrame) return;
  drawPvPage(tft, lastF, pageIndex);
}

#ifdef ROLE_POLLER
  #include <ModbusIP_ESP8266.h>
  ModbusIP mb;
  IPAddress inverterIP(192,168,0,10);
  const uint16_t modbusPort = 502;
  const uint8_t  unitId     = 2;

// Register
const uint16_t REG_PV_AC      = 32064; // int32 (Hi,Lo), W
const uint16_t REG_GRID_P     = 37113; // int32 (Hi,Lo), W, +E/-I
const uint16_t REG_BATT_P     = 37001; // int32 (Hi,Lo), W, +C/-D
const uint16_t REG_WR_TEMP    = 32087; // int16 (x10 °C)
const uint16_t REG_PV_TODAY   = 32114; // uint32 (kWh/100)
const uint16_t REG_GRID_EXP_T = 37119; // uint32 (kWh/100)
const uint16_t REG_GRID_IMP_T = 37121; // uint32 (kWh/100)
const uint16_t REG_BATT_SOCX  = 37004; // uint16 (x10)

// PV1/PV2 Strings
const uint16_t REG_PV1_V      = 32016; // int16 Vx10
const uint16_t REG_PV1_A      = 32017; // int16 Ax100
const uint16_t REG_PV2_V      = 32018; // int16 Vx10
const uint16_t REG_PV2_A      = 32019; // int16 Ax100

// Netz V/I Phasen
const uint16_t REG_VA         = 37101; // int32 Vx10
const uint16_t REG_VB         = 37103; // int32 Vx10
const uint16_t REG_VC         = 37105; // int32 Vx10
const uint16_t REG_IA         = 37107; // int32 Ax100 (signed)
const uint16_t REG_IB         = 37109; // int32 Ax100 (signed)
const uint16_t REG_IC         = 37111; // int32 Ax100 (signed)

  static uint32_t lastPollStart=0, lastPollTick=0;
  const  uint32_t POLL_INTERVAL_MS=30000, TIMEOUT_MS=8000;
  static volatile int  pending=0;
  static volatile bool gotAny=false, hadError=false;
  static bool printedThisRound = true;

  // Puffer
  static uint16_t bufPv[2], bufGrid[2], bufBatt[2], bufTemp[1], bufSoC[1];
  static uint16_t bufPvToday[2], bufExpTot[2], bufImpTot[2];
  static uint16_t bufPV1V[1], bufPV1A[1], bufPV2V[1], bufPV2A[1];
  static uint16_t bufVA[2], bufVB[2], bufVC[2], bufIA[2], bufIB[2], bufIC[2];

  inline int32_t  mk32_BE(uint16_t hi, uint16_t lo){ return (int32_t)(((uint32_t)hi<<16)|lo); }
  static inline uint32_t mkU32_BE(uint16_t hi, uint16_t lo){ return (((uint32_t)hi<<16)|lo); }
  static inline bool cbFinal(bool success){ if(success) gotAny=true; else hadError=true; if(pending>0) pending--; return true; }

  // ==== Snapshot/Staging für atomare Messbilder ====
  struct Snapshot {
    int32_t pvW=0, gridW=0, battW=0;
    int16_t temp10=0;
    float   pvTodayKWh=0.0f; // (falls genutzt)
    uint16_t socx10=0;
    // Strings
    int16_t pv1Voltage_x10_V=0, pv1Current_x10_A=0;
    int16_t pv2Voltage_x10_V=0, pv2Current_x10_A=0;
    // Netz V/I
    int32_t gridVoltageA_x10_V=0, gridVoltageB_x10_V=0, gridVoltageC_x10_V=0;
    int32_t gridCurrentA_x100_A=0, gridCurrentB_x100_A=0, gridCurrentC_x100_A=0;
    uint32_t expTot=0, impTot=0; // total counters (optional)
    uint32_t readyMask=0;
  };
  static Snapshot snapStage, snapLive;

  enum : uint32_t {
    RM_PV      = 1u<<0,
    RM_GRID    = 1u<<1,
    RM_BATT    = 1u<<2,
    RM_TEMP    = 1u<<3,
    RM_PVTODAY = 1u<<4,
    RM_EXP     = 1u<<5,
    RM_IMP     = 1u<<6,
    RM_SOC     = 1u<<7,
    RM_PV1V    = 1u<<8,
    RM_PV1A    = 1u<<9,
    RM_PV2V    = 1u<<10,
    RM_PV2A    = 1u<<11,
    RM_VA      = 1u<<12,
    RM_VB      = 1u<<13,
    RM_VC      = 1u<<14,
    RM_IA      = 1u<<15,
    RM_IB      = 1u<<16,
    RM_IC      = 1u<<17,
  };
  static const uint32_t RM_REQUIRED = RM_PV | RM_GRID | RM_BATT; // für konsistente Integration

  static void startPoll(){
    if (pending>0 || !mb.isConnected(inverterIP)) return;

    hadError=false; gotAny=false; printedThisRound=false;
    pending=16; lastPollStart=millis();

    // Snapshot leeren
    snapStage = Snapshot{};

    mb.readHreg(inverterIP, REG_PV_AC, bufPv, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.pvW=mk32_BE(bufPv[0],bufPv[1]); snapStage.readyMask|=RM_PV; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    mb.readHreg(inverterIP, REG_GRID_P, bufGrid, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.gridW=mk32_BE(bufGrid[0],bufGrid[1]); snapStage.readyMask|=RM_GRID; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    mb.readHreg(inverterIP, REG_BATT_P, bufBatt, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.battW=mk32_BE(bufBatt[0],bufBatt[1]); snapStage.readyMask|=RM_BATT; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    mb.readHreg(inverterIP, REG_WR_TEMP, bufTemp, 1, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.temp10=(int16_t)bufTemp[0]; snapStage.readyMask|=RM_TEMP; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    mb.readHreg(inverterIP, REG_PV_TODAY, bufPvToday, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.pvTodayKWh=mkU32_BE(bufPvToday[0],bufPvToday[1])/100.0f; snapStage.readyMask|=RM_PVTODAY; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    mb.readHreg(inverterIP, REG_GRID_EXP_T, bufExpTot, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.expTot=mkU32_BE(bufExpTot[0],bufExpTot[1]); snapStage.readyMask|=RM_EXP; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    mb.readHreg(inverterIP, REG_GRID_IMP_T, bufImpTot, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.impTot=mkU32_BE(bufImpTot[0],bufImpTot[1]); snapStage.readyMask|=RM_IMP; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    mb.readHreg(inverterIP, REG_BATT_SOCX, bufSoC, 1, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.socx10=bufSoC[0]; snapStage.readyMask|=RM_SOC; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    // Strings
    mb.readHreg(inverterIP, REG_PV1_V, bufPV1V, 1, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.pv1Voltage_x10_V=(int16_t)bufPV1V[0]; snapStage.readyMask|=RM_PV1V; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);
    mb.readHreg(inverterIP, REG_PV1_A, bufPV1A, 1, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.pv1Current_x10_A=(int16_t)bufPV1A[0]; snapStage.readyMask|=RM_PV1A; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);
    mb.readHreg(inverterIP, REG_PV2_V, bufPV2V, 1, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.pv2Voltage_x10_V=(int16_t)bufPV2V[0]; snapStage.readyMask|=RM_PV2V; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);
    mb.readHreg(inverterIP, REG_PV2_A, bufPV2A, 1, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.pv2Current_x10_A=(int16_t)bufPV2A[0]; snapStage.readyMask|=RM_PV2A; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    // Netz V/I
    mb.readHreg(inverterIP, REG_VA, bufVA, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.gridVoltageA_x10_V=mk32_BE(bufVA[0],bufVA[1]); snapStage.readyMask|=RM_VA; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);
    mb.readHreg(inverterIP, REG_VB, bufVB, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.gridVoltageB_x10_V=mk32_BE(bufVB[0],bufVB[1]); snapStage.readyMask|=RM_VB; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);
    mb.readHreg(inverterIP, REG_VC, bufVC, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.gridVoltageC_x10_V=mk32_BE(bufVC[0],bufVC[1]); snapStage.readyMask|=RM_VC; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    mb.readHreg(inverterIP, REG_IA, bufIA, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.gridCurrentA_x100_A=mk32_BE(bufIA[0],bufIA[1]); snapStage.readyMask|=RM_IA; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);
    mb.readHreg(inverterIP, REG_IB, bufIB, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.gridCurrentB_x100_A=mk32_BE(bufIB[0],bufIB[1]); snapStage.readyMask|=RM_IB; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);
    mb.readHreg(inverterIP, REG_IC, bufIC, 2, [](Modbus::ResultCode rc, uint16_t, void*)->bool{
      if(rc==Modbus::EX_SUCCESS){ snapStage.gridCurrentC_x100_A=mk32_BE(bufIC[0],bufIC[1]); snapStage.readyMask|=RM_IC; }
      return cbFinal(rc==Modbus::EX_SUCCESS);
    }, unitId);

    Serial.println("[INFO] Poll gestartet");
  }

  static void maybeFinishPoll(){
    if(pending>0 && (millis()-lastPollStart>=TIMEOUT_MS)){ pending=0; Serial.println("[POLL] timeout"); }
    if(pending>0 || printedThisRound) return;
    if(!gotAny){ printedThisRound=true; return; }

    // Nur mit konsistentem Kern übernehmen
    if ( (snapStage.readyMask & RM_REQUIRED) != RM_REQUIRED ) { printedThisRound=true; return; }

    // Atomar übernehmen
    snapLive = snapStage;

    // Frame aus Snapshot befüllen
    lastF.pvW   = snapLive.pvW;
    lastF.gridW = snapLive.gridW;
    lastF.battW = snapLive.battW;
    lastF.temp10= snapLive.temp10;
    lastF.socx10= snapLive.socx10;

    // Strings
    lastF.pv1Voltage_x10_V = snapLive.pv1Voltage_x10_V;
    lastF.pv1Current_x10_A = snapLive.pv1Current_x10_A;
    lastF.pv2Voltage_x10_V = snapLive.pv2Voltage_x10_V;
    lastF.pv2Current_x10_A = snapLive.pv2Current_x10_A;

    // Netz V/I
    lastF.gridVoltageA_x10_V = snapLive.gridVoltageA_x10_V;
    lastF.gridVoltageB_x10_V = snapLive.gridVoltageB_x10_V;
    lastF.gridVoltageC_x10_V = snapLive.gridVoltageC_x10_V;
    lastF.gridCurrentA_x100_A = snapLive.gridCurrentA_x100_A;
    lastF.gridCurrentB_x100_A = snapLive.gridCurrentB_x100_A;
    lastF.gridCurrentC_x100_A = snapLive.gridCurrentC_x100_A;

    // Meta
    lastF.magic   = PV_MAGIC;
    lastF.version = PV_VERSION;
    lastF.seq     = ++lastSeq;
    { time_t n; time(&n); lastF.ts=(uint32_t)n; }

    // Heute-Werte ins Frame (aus lokaler Integration)
    lastF.pvTodayKWh   = dayAgg.gen_kWh;
    lastF.gridExpToday = dayAgg.exp_kWh;
    lastF.gridImpToday = dayAgg.impT1_kWh + dayAgg.impT2_kWh;

    lastF.crc = 0;
    lastF.crc = crc16_modbus((const uint8_t*)&lastF, sizeof(PvFrameV4)-2);

    // Multicast senden
    udpFrame.writeTo((uint8_t*)&lastF, sizeof(PvFrameV4), MCAST_GRP, MCAST_PORT);

    haveFrame=true;
    drawIfFrame();
    printedThisRound=true;
  }
#endif // ROLE_POLLER

#ifndef ROLE_POLLER
// ---- Client: Frames empfangen ----
static void beginListenFrames(){
  if (!udpFrame.listenMulticast(MCAST_GRP, MCAST_PORT, 1, TCPIP_ADAPTER_IF_STA)){
    Serial.println("[ERR] listenMulticast failed"); return;
  }
  udpFrame.onPacket([](AsyncUDPPacket p){
    if (p.length() < sizeof(PvFrameV4)) return;
    const PvFrameV4* f = (const PvFrameV4*)p.data();
    if (f->magic!=PV_MAGIC || f->version!=PV_VERSION) return;
    uint16_t check = crc16_modbus((const uint8_t*)p.data(), sizeof(PvFrameV4)-2);
    if (check != f->crc) return;
    if (f->seq <= lastSeq) return;

    lastSeq = f->seq; lastRxMs = millis();
    lastF = *f; haveFrame=true;

    // Lokal integrieren, damit Anzeige auf Clients Werte hat
    integrateTick(lastF.pvW, lastF.gridW, lastF.battW);
    handleDayMonthRollover();

    drawIfFrame();
  });
}
#endif

// ======= Stats: Discover/Offer + Stream (mit load_kWh) =======
static uint32_t statsSeq=1;

static void statsSendTo(IPAddress ip, uint16_t port, uint8_t type, uint32_t seq, const void* pl, uint16_t len){
  StatsHdr h{0xCAFE, 1, type, seq, len, 0};
  h.crc = pvstats_crc(h, (const uint8_t*)pl);
  uint8_t buf[sizeof(StatsHdr)+128];
  memcpy(buf, &h, sizeof(h));
  if (pl && len) memcpy(buf+sizeof(h), pl, len);
  udpStatsCtrl.writeTo(buf, sizeof(h)+len, ip, port);
}

static void statsSendDiscover(){
  statsSendTo(STATS_MCAST_GRP, STATS_MCAST_PORT, STATS_DISCOVER, ++statsSeq, nullptr, 0);
}

#ifdef ROLE_POLLER
static inline int daysInMonthInline(int y,int m){ return daysInMonth(y,m); }

static void statsPollerStart(){
  // Multicast: Discover empfangen, Offer senden
  if (!udpStatsCtrl.listenMulticast(STATS_MCAST_GRP, STATS_MCAST_PORT)){
    Serial.println("[STATS] mcast listen failed");
  }
  udpStatsCtrl.onPacket([](AsyncUDPPacket p){
    if (p.length() < sizeof(StatsHdr)) return;
    const StatsHdr* h = (const StatsHdr*)p.data();
    if (h->magic!=0xCAFE || h->version!=1) return;
    const uint8_t* pl = (const uint8_t*)p.data()+sizeof(StatsHdr);
    if (pvstats_crc(*h, pl)!=h->crc) return;

    if (h->type==STATS_DISCOVER){
      PayloadOffer off{STATS_SERVER_PORT, 0};
      statsSendTo(p.remoteIP(), p.remotePort(), STATS_OFFER, ++statsSeq, &off, sizeof(off));
    } else if (h->type==STATS_REQ_RANGE){
      if (h->len < sizeof(PayloadReqRange)) return;
      // Unicast server
      if (!udpStatsSrv.listen(STATS_SERVER_PORT)){
        Serial.println("[STATS] server listen failed");
        return;
      }
      // Sende alle Tage + Monate
      PayloadReqRange r = *(const PayloadReqRange*)pl;

      // Tage streamen (ab r.from*)
      int y= (r.fromY? r.fromY:1970), m=(r.fromM? r.fromM:1), d=(r.fromD? r.fromD:1);
      int ty,tm,td; todayYMD(ty,tm,td);
      while ( (y<ty) || (y==ty && (m<tm || (m==tm && d<=td))) ){
        DayAgg a; if (!loadDayFromNVS(y,m,d,a)) a={0,0,0,0,0};
        PayloadDay pd{ (uint16_t)y,(uint16_t)m,(uint16_t)d, a.gen_kWh, a.load_kWh, a.impT1_kWh, a.impT2_kWh, a.exp_kWh };
        statsSendTo(p.remoteIP(), STATS_SERVER_PORT, STATS_DAY, ++statsSeq, &pd, sizeof(pd));
        // nächster Tag
        int dim=daysInMonthInline(y,m); d++; if (d>dim){ d=1; m++; if (m>12){ m=1; y++; } }
        delay(2);
      }

      // Monate streamen (ab r.fromMon*)
      y=(r.fromMonY? r.fromMonY:1970); m=(r.fromMonM? r.fromMonM:1);
      while ( (y<ty) || (y==ty && m<=tm) ){
        MonthAgg ma; loadMonthFromNVS(y,m,ma);
        PayloadMon pm{ (uint16_t)y,(uint16_t)m, ma.gen_kWh, ma.load_kWh, ma.impT1_kWh, ma.impT2_kWh, ma.exp_kWh };
        statsSendTo(p.remoteIP(), STATS_SERVER_PORT, STATS_MON, ++statsSeq, &pm, sizeof(pm));
        m++; if (m>12){ m=1; y++; }
        delay(2);
      }

      statsSendTo(p.remoteIP(), STATS_SERVER_PORT, STATS_DONE, ++statsSeq, nullptr, 0);
    }
  });
}
#else
static IPAddress statsServerIP;
static uint16_t  statsServerPort=0;

static void statsClientStart(){
  if (!udpStatsCtrl.listenMulticast(STATS_MCAST_GRP, STATS_MCAST_PORT)){
    Serial.println("[STATS] mcast listen failed");
  }
  udpStatsCtrl.onPacket([](AsyncUDPPacket p){
    if (p.length() < sizeof(StatsHdr)) return;
    const StatsHdr* h = (const StatsHdr*)p.data();
    if (h->magic!=0xCAFE || h->version!=1) return;
    const uint8_t* pl = (const uint8_t*)p.data()+sizeof(StatsHdr);
    if (pvstats_crc(*h, pl)!=h->crc) return;

    switch(h->type){
      case STATS_OFFER:{
        if (h->len<sizeof(PayloadOffer)) return;
        const PayloadOffer* off = (const PayloadOffer*)pl;
        statsServerIP = p.remoteIP(); statsServerPort = off->statsPort;
        // Alles ab Beginn anfordern
        PayloadReqRange r{}; r.fromY=0; r.fromM=0; r.fromD=0; r.fromMonY=0; r.fromMonM=0;
        statsSendTo(statsServerIP, statsServerPort, STATS_REQ_RANGE, ++statsSeq, &r, sizeof(r));
      }break;
      case STATS_DAY:{
        if (h->len<sizeof(PayloadDay)) return;
        const PayloadDay* d = (const PayloadDay*)pl;
        DayAgg a{ d->gen_kWh, d->load_kWh, d->impT1_kWh, d->impT2_kWh, d->exp_kWh };
        saveDayToNVS(d->y, d->m, d->d, a);
      }break;
      case STATS_MON:{
        if (h->len<sizeof(PayloadMon)) return;
        const PayloadMon* m = (const PayloadMon*)pl;
        MonthAgg a{ m->gen_kWh, m->load_kWh, m->impT1_kWh, m->impT2_kWh, m->exp_kWh };
        saveMonthToNVS(m->y, m->m, a);
      }break;
      case STATS_DONE:{
        // aktuellen Tag/Monat in RAM laden
        int y,m,d; todayYMD(y,m,d);
        DayAgg td; if (loadDayFromNVS(y,m,d,td)) dayAgg=td;
        MonthAgg tm; loadMonthFromNVS(y,m,tm); monthAgg=tm;
      }break;
      default: break;
    }
  });

  // Discover anstoßen
  statsSendDiscover();
}
#endif

// ===== Setup / Loop =====
void setup(){
  Serial.begin(115200);
  tft.init(); tft.setRotation(TFT_ROTATION);
  tft.fillScreen(TFT_BLACK);
  #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT); digitalWrite(TFT_BL, HIGH);
  #endif

  // Touch init
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(1);  // Landscape-1

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while(WiFi.status()!=WL_CONNECTED){ delay(300); Serial.print("."); }
  Serial.println(); Serial.println("[WiFi] " + WiFi.localIP().toString());

  configTzTime(TZ_EU_ZURICH, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
  delay(300);

  // Init Tages-/Monatsanker
  int y,m,d; todayYMD(y,m,d);
  curY=y;curM=m;curD=d;
  loadMonthFromNVS(y,m, monthAgg);
  DayAgg td; if (loadDayFromNVS(y,m,d, td)) dayAgg=td;

#ifdef ROLE_POLLER
  // Modbus
  mb.client();
  mb.connect(inverterIP, modbusPort);

  // Stats-Server:
  statsPollerStart();
#else
  // Client: Frames empfangen + Stats-Client
  beginListenFrames();
  statsClientStart();
#endif

  // Startscreen
  tft.setTextDatum(MC_DATUM);
  tft.setTextFont(2); tft.setTextSize(1); tft.setTextColor(TFT_WHITE, TFT_BLACK);
#ifdef ROLE_POLLER
  tft.drawString("Poller bereit…", 160, 120);
#else
  tft.drawString("Warte auf PV-Daten…", 160, 120);
#endif
}

void loop(){
#ifdef ROLE_POLLER
  static uint32_t lastConnTry=0, lastPollTick=0, lastPollStart=0;
  const  uint32_t POLL_INTERVAL_MS=30000;

  if(!mb.isConnected(inverterIP)){
    if(millis()-lastConnTry>2000){
      mb.connect(inverterIP, modbusPort);
      lastConnTry=millis();
    }
  }else{
    if (millis()-lastPollTick >= 300){
      lastPollTick = millis();
      if (millis()-lastPollStart>=POLL_INTERVAL_MS) { lastPollStart=millis(); startPoll(); }
    }
  }
  mb.task();
  maybeFinishPoll();
#else
  // Client: passiv
  delay(5);
#endif

#ifdef ROLE_POLLER
  if (haveFrame && pending==0){
    integrateTick(lastF.pvW, lastF.gridW, lastF.battW);
    handleDayMonthRollover();
  }
  handleTouchSwipe();
#else
  if (haveFrame){
    integrateTick(lastF.pvW, lastF.gridW, lastF.battW);
    handleDayMonthRollover();
    handleTouchSwipe();           // Seitenwechsel per Wisch
  } else {
    handleTouchSwipe();           // Touch initialisieren
  }
#endif
}
