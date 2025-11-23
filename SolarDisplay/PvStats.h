// ===================== PvStats.h =====================
#pragma once
#include <Arduino.h>
#include <IPAddress.h>

// ---- Multicast & Ports (kannst du bei Bedarf anpassen) ----
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
