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

## ETA-Berechnung (Adaptive Ladezeit-Vorhersage)

Die **ETA (Estimated Time of Arrival)** zeigt an, wann die Batterie voraussichtlich 20% erreichen wird:

### Funktionsweise

Die ETA-Berechnung ist **adaptiv** und benötigt **keine Batteriekapazität**:

1. **Aktivierung**: Nur bei SOC < 20% und aktivem Laden (battW > 50W)
2. **Tracking**: Bei jedem 1%-Anstieg wird die Zeit gemessen
3. **Berechnung**: `ETA = (20% - aktueller SOC) × Zeit_pro_Prozent`
4. **Anzeige**: Zeit im Format HH:MM oder "--:--" wenn nicht verfügbar

### Vorteile dieser Methode

- ✅ **Keine Batteriekapazität nötig**: Funktioniert mit jeder Batteriegröße
- ✅ **Adaptiv**: Passt sich automatisch der sich ändernden Ladeleistung an
- ✅ **Präzise bei Sonnenaufgang**: Berücksichtigt steigende PV-Leistung optimal
- ✅ **Einfach**: Nutzt nur die tatsächlich gemessene Ladegeschwindigkeit

### Beispiel

```
Aktueller SOC: 15%
Letzter 1%-Schritt: 3 Minuten (180 Sekunden)
Noch zu laden: 5%
ETA = 5 × 3 Min = 15 Minuten
Anzeige: 08:15 (wenn aktuelle Zeit 08:00)
```

### Plausibilitätschecks

- Zeitdifferenz pro %: 1 Sekunde bis 1 Stunde
- Zeit pro %: maximal 1 Stunde
- Gesamt-ETA: maximal 24 Stunden

## Code-Optimierungen

Gegenüber dem Original wurden folgende Vereinfachungen vorgenommen:

- Entfernung redundanter Funktionen (daysInMonthInline)
- Entfernung ungenutzter Bibliotheken (Streaming.h)
- Kompaktere Enum-Definitionen
- Bessere Code-Organisation
