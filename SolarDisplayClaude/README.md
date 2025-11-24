# SolarDisplayClaude - LovyanGFX Version

LovyanGFX-Version des Solar-Monitors für Huawei Sun2000 Anlagen.

## Unterschiede zu SolarDisplay2

- **LovyanGFX statt TFT_eSPI**: Bessere Hardware-Unterstützung und Performance
- **CYD_Display_Config.h**: Modulare Display-Konfiguration für verschiedene CYD-Varianten
- **Integriertes Touch-Handling**: Touch wird direkt von LovyanGFX gehandhabt
- **Gleiche Funktionalität**: Alle Features von SolarDisplay2 bleiben erhalten

## Hardware-Unterstützung

Über `CYD_Display_Config.h` unterstützt:
- ✅ ESP32-2432S028R (Standard CYD, 320x240, ILI9341) - vorkonfiguriert
- ✅ ESP32-2432S024 (Template vorhanden)
- ✅ Custom CYD-Displays (eigene Konfiguration möglich)

## Konfiguration

### Display-Typ wählen

In `CYD_Display_Config.h` das gewünschte Display auskommentieren:

```cpp
#define CYD_2432S028R    // Standard CYD
//#define CYD_2432S024     // Alternative
//#define CYD_CUSTOM       // Eigene Konfiguration
```

### Poller/Client Rolle

In `SolarDisplayClaude.ino` Zeile 17:
```cpp
//#define ROLE_POLLER    // einkommentieren = Poller; auskommentieren = Client
```

## Funktionen

- **Poller/Client Architektur**: Ein Gerät (Poller) liest die Daten vom Wechselrichter, alle anderen (Clients) empfangen die Daten per UDP
- **5 Anzeigeseiten**:
  1. Gauges (PV, Batterie, Grid)
  2. PV-String Leistungen (PV1/PV2)
  3. Tages-Statistik (30 Tage)
  4. Monats-Statistik (12 Monate)
  5. Uhrzeit/Datum
- **Touch-Swipe Navigation**: Wischen zum Seitenwechsel
- **Adaptive ETA-Berechnung**: Zeit bis 20% Batterieladung
- **Lokale Integration**: PV, Grid und Batterie-Daten werden lokal integriert
- **NVS-Speicherung**: Tages- und Monatswerte werden im Flash gespeichert

## Programmierung

### Benötigte Bibliotheken

- **LovyanGFX** (statt TFT_eSPI)
- AsyncUDP
- ModbusIP_ESP8266 (nur für Poller)
- Preferences
- Streaming

### Arduino IDE Setup

1. LovyanGFX Bibliothek installieren
2. `Credentials.h` erstellen (siehe unten)
3. Display-Typ in `CYD_Display_Config.h` wählen
4. Kompilieren und hochladen

### Credentials.h

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

### Vorteile

- ✅ Keine Batteriekapazität nötig
- ✅ Adaptiv: Passt sich an veränderte Ladeleistung an
- ✅ Präzise bei Sonnenaufgang: Berücksichtigt steigende PV-Leistung

## Vorteile von LovyanGFX

1. **Performance**: Schnelleres Rendering
2. **Flexibilität**: Einfache Anpassung für verschiedene Displays
3. **Integriert**: Display + Touch in einer Bibliothek
4. **Modern**: Aktive Entwicklung und Updates
5. **Hardware-nah**: Direkter Zugriff auf Display-Features

## Migration von SolarDisplay2

SolarDisplay2 (TFT_eSPI) bleibt als lauffähige Version erhalten!

**Hauptänderungen:**
- `TFT_eSPI tft` → `LGFX lcd`
- `User_Setup.h` → `CYD_Display_Config.h`
- Touch-Handling vereinfacht (integriert in LovyanGFX)

## Dateien

```
SolarDisplayClaude/
├── SolarDisplayClaude.ino   (Hauptprogramm mit LovyanGFX)
├── CYD_Display_Config.h     (Display-Konfiguration)
└── README.md                (Diese Datei)
```

## Credits

Basiert auf SolarDisplay2 mit folgenden Verbesserungen:
- Umstellung auf LovyanGFX
- Modulare Display-Konfiguration
- Vereinfachtes Touch-Handling
