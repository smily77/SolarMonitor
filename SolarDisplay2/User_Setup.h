// ===================== User_Setup.h (lokal im Sketch-Ordner) =====================
// CYD (ESP32-2432S028[R]) + ILI9341 + HSPI-Pinout (Variante B)

// WICHTIG: Dein .ino muss VOR <TFT_eSPI.h> folgendes haben:
//   #define USER_SETUP_LOADED
//   #include "User_Setup.h"

#define ILI9341_DRIVER
#define USER_SETUP_LOADED

// Panelgröße
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// HSPI verwenden:
#define USE_HSPI_PORT

// HSPI-Pins (CYD Variante B)
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1     // RST oft fest verdrahtet
#define TFT_BL   21     // Backlight (aktiv HIGH)

// ---- Touch (XPT2046) ----
// Teilt sich MISO/MOSI/SCLK mit HSPI; eigener CS + optional IRQ:
#define TOUCH_DRIVER 0x2046
#define TOUCH_CS   33      // <— Falls Touch weiterhin tot ist, versuchsweise 16
#define TOUCH_IRQ  36

// Fonts/Optionen
#define LOAD_GLCD        // Font 1
#define LOAD_FONT2       // kleine serifenlose Schrift
#define LOAD_FONT4       // größere serifenlose Schrift (optional)
#define LOAD_GFXFF
#define SMOOTH_FONT

// SPI-Frequenzen
#define SPI_FREQUENCY        40000000
#define SPI_READ_FREQUENCY   20000000
#define SPI_TOUCH_FREQUENCY   2500000

// Landscape
#define TFT_ROTATION 1
