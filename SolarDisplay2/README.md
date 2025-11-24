# SolarDisplay2

Vereinfachte und aufgeräumte Version des Solar-Monitors für Huawei Sun2000 Anlagen.

## Unterschiede zum Original (SolarDisplay)

- **Alles in einer Datei**: SolarDisplay2.ino enthält den gesamten Code (außer User_Setup.h für Display-Konfiguration)
- **Kompakterer Code**: ~120 Zeilen weniger durch Vereinfachung und bessere Strukturierung
- **Klarere Organisation**: Bessere Kommentare und logische Gruppierung
- **Keine History.ino**: Nicht mehr benötigt
- **Gleiche Funktionalität**: Alle Features bleiben erhalten

## Funktionen

- **Poller/Client Architektur**: Ein Gerät (Poller) liest die Daten vom Wechselrichter, alle anderen (Clients) empfangen die Daten per UDP
- **5 Anzeigeseiten**:
  1. Gauges (PV, Batterie, Grid)
  2. PV-String Leistungen (PV1/PV2)
  3. Tages-Statistik (30 Tage)
  4. Monats-Statistik (12 Monate)
  5. Uhrzeit/Datum
- **Touch-Swipe Navigation**: Wischen zum Seitenwechsel
- **Lokale Integration**: PV, Grid und Batterie-Daten werden lokal integriert
- **NVS-Speicherung**: Tages- und Monatswerte werden im Flash gespeichert

## Konfiguration

In `SolarDisplay2.ino` Zeile 17:
```cpp
//#define ROLE_POLLER    // einkommentieren = Poller; auskommentieren = Client
```

- **Poller**: Zeile einkommentieren (`#define ROLE_POLLER`)
- **Client**: Zeile auskommentiert lassen

## Hardware

- ESP32-2432S028R (CYD - Cheap Yellow Display)
- ILI9341 Display (320x240)
- XPT2046 Touch

## Programmierung

Arduino IDE verwenden. Benötigte Bibliotheken:
- TFT_eSPI
- AsyncUDP
- XPT2046_Touchscreen
- ModbusIP_ESP8266 (nur für Poller)
- Preferences

## Credentials

Erstelle eine `Credentials.h` Datei im Bibliotheksordner oder im Sketch-Ordner:
```cpp
const char* ssid = "DEIN_WIFI";
const char* password = "DEIN_PASSWORT";
```

## Bekannte Probleme

- **ETA-Anzeige (Zeit bis 20% Batterie)**: Aktuell noch nicht korrekt implementiert (zeigt --:--)
