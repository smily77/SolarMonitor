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

## ETA-Berechnung

Die **ETA (Estimated Time of Arrival)** zeigt an, wann die Batterie voraussichtlich 20% erreichen wird:

- **Funktionsweise**: Berechnet die Zeit basierend auf der aktuellen Entladeleistung
- **Annahmen**: Standard-Batteriekapazität von 5 kWh (kann im Code angepasst werden: Zeile 1144)
- **Aktivierung**: Nur bei Entladung (battW < -50W) und SOC > 20%
- **Anzeige**: Zeit im Format HH:MM oder "--:--" wenn nicht anwendbar
- **Plausibilitätscheck**: Maximum 24 Stunden

### Anpassung der Batteriekapazität

Falls deine Batterie eine andere Kapazität hat, ändere in `SolarDisplay2.ino` Zeile 1144:

```cpp
const float BATTERY_CAPACITY_WH = 5000.0f; // Deine Kapazität in Wh (z.B. 10000.0f für 10 kWh)
```

## Code-Optimierungen

Gegenüber dem Original wurden folgende Vereinfachungen vorgenommen:

- Entfernung redundanter Funktionen (daysInMonthInline)
- Entfernung ungenutzter Bibliotheken (Streaming.h)
- Kompaktere Enum-Definitionen
- Bessere Code-Organisation
