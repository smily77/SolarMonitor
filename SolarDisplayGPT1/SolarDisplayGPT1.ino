// SolarDisplayGPT1: vereinfachte Variante des Sketches
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

#include "PvCommon.h"  // Frame v4, drawPvPage(...), pvMaxPages(), crc16_modbus, MCAST_GRP, MCAST_PORT
#include "PvStats.h"   // bereits übernommen (enthält load_kWh in Payloads)

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
