// ===========================================================================================
// CYD_Display_Config.h - LovyanGFX Konfiguration für verschiedene CYD-Displays
// ===========================================================================================
// Diese Datei definiert die LGFX Klasse für Display und Touch
// Ermöglicht Verwendung desselben Codes für verschiedene CYD-Varianten
//
// ANLEITUNG: Kommentiere die gewünschte Display-Konfiguration ein!
// ===========================================================================================

#pragma once
#include <LovyanGFX.hpp>

// ================= Display-Typ auswählen =================
// Uncomment EINE der folgenden Optionen:

#define CYD_2432S028R    // ESP32-2432S028R (Standard CYD, 320x240, ILI9341)
//#define CYD_2432S024     // ESP32-2432S024 (Alternative)
//#define CYD_CUSTOM       // Eigene Konfiguration (siehe unten)

// ================= Konfiguration für CYD-2432S028R =================
#ifdef CYD_2432S028R

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ILI9341 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_XPT2046 _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = HSPI_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc = 2;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs = 15;
      cfg.pin_rst = -1;
      cfg.pin_busy = -1;
      cfg.panel_width = 240;
      cfg.panel_height = 320;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;
      cfg.readable = true;
      cfg.invert = false;
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;
      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = 21;
      cfg.invert = false;
      cfg.freq = 44100;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }

    {
      auto cfg = _touch_instance.config();
      cfg.x_min = 200;
      cfg.x_max = 3700;
      cfg.y_min = 240;
      cfg.y_max = 3800;
      cfg.pin_int = 36;
      cfg.bus_shared = true;
      cfg.offset_rotation = 0;
      cfg.spi_host = VSPI_HOST;
      cfg.freq = 1000000;
      cfg.pin_sclk = 25;
      cfg.pin_mosi = 32;
      cfg.pin_miso = 39;
      cfg.pin_cs = 33;
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

#endif // CYD_2432S028R

// ================= Konfiguration für CYD-2432S024 =================
#ifdef CYD_2432S024

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ILI9341 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Touch_XPT2046 _touch_instance;

public:
  LGFX(void)
  {
    // TODO: Konfiguration für CYD-2432S024
    // Anpassen nach Bedarf
  }
};

#endif // CYD_2432S024

// ================= Custom Konfiguration =================
#ifdef CYD_CUSTOM

class LGFX : public lgfx::LGFX_Device
{
  // TODO: Eigene Display-Konfiguration hier einfügen
  // Siehe LovyanGFX Dokumentation
public:
  LGFX(void)
  {
    // Custom Setup
  }
};

#endif // CYD_CUSTOM

// ================= Fallback wenn keine Konfiguration gewählt =================
#if !defined(CYD_2432S028R) && !defined(CYD_2432S024) && !defined(CYD_CUSTOM)
  #error "Bitte wähle eine Display-Konfiguration in CYD_Display_Config.h!"
#endif
