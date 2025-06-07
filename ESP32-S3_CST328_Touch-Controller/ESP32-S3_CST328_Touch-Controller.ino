/*
 * ESP32-S3 CST328 Touch-Controller - Vollständige Kalibrierung
 * ==========================================================
 * 
 * Features:
 * - Konfigurierbare Touch-Orientierung und Mapping
 * - Mehrere Test-Modi für perfekte Kalibrierung
 * - Visuelle Kalibrierungs-Interface
 * - Dual I2C Bus System (Wire1 für Touch, Wire für andere)
 * 
 * Hardware:
 * - Display: ST7789 320x240 (SPI)
 * - Touch: CST328 (I2C auf Wire1)
 * - Orientation: Landscape USB rechts oben
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Wire.h>

// ============================================
// KONFIGURATION - HIER ANPASSEN
// ============================================

// Display-Konfiguration
#define DISPLAY_WIDTH   320
#define DISPLAY_HEIGHT  240
#define DISPLAY_ROTATION 1    // 1 = Landscape USB rechts

// Touch-Hardware-Konfiguration
#define CST328_SDA_PIN  1
#define CST328_SCL_PIN  3
#define CST328_INT_PIN  4
#define CST328_RST_PIN  2
#define CST328_ADDR     0x1A
#define I2C_MASTER_FREQ_HZ 400000

// Touch-Mapping-Konfiguration (KORRIGIERT für Ihre Hardware)
#define TOUCH_RAW_X_MIN     0
#define TOUCH_RAW_X_MAX     240
#define TOUCH_RAW_Y_MIN     0  
#define TOUCH_RAW_Y_MAX     320   // KORRIGIERT: war 272, aber Ihre Daten zeigen 318+

// Touch-Kalibrierung (DIESE VARIABLEN ANPASSEN)
bool TOUCH_SWAP_XY = true;           // X/Y vertauschen
bool TOUCH_INVERT_X = false;         // X-Achse invertieren
bool TOUCH_INVERT_Y = true;          // Y-Achse invertieren
bool TOUCH_MIRROR_X = false;         // X spiegeln (alternative zu invert)
bool TOUCH_MIRROR_Y = false;         // Y spiegeln (alternative zu invert)

// Auto-Kalibrierung für Raw-Bereiche
uint16_t measured_x_min = 9999, measured_x_max = 0;
uint16_t measured_y_min = 9999, measured_y_max = 0;
bool auto_calibration_active = false;

// Kalibrierungs-Modi
typedef enum {
  TOUCH_MODE_0_ORIGINAL = 0,        // Ihr ursprüngliches Mapping
  TOUCH_MODE_1_XY_SWAP = 1,         // X/Y vertauscht
  TOUCH_MODE_2_XY_SWAP_INV_Y = 2,   // X/Y vertauscht + Y invertiert
  TOUCH_MODE_3_XY_SWAP_INV_X = 3,   // X/Y vertauscht + X invertiert  
  TOUCH_MODE_4_XY_SWAP_INV_BOTH = 4, // X/Y vertauscht + beide invertiert
  TOUCH_MODE_5_NORMAL_INV_Y = 5,    // Normal + Y invertiert
  TOUCH_MODE_6_NORMAL_INV_X = 6,    // Normal + X invertiert
  TOUCH_MODE_7_NORMAL_INV_BOTH = 7, // Normal + beide invertiert
  TOUCH_MODE_MAX = 8
} touch_mapping_mode_t;

// Aktuelle Konfiguration
touch_mapping_mode_t current_touch_mode = TOUCH_MODE_2_XY_SWAP_INV_Y;
bool calibration_mode = false;

// ============================================
// DISPLAY KONFIGURATION
// ============================================

class LGFX : public lgfx::LGFX_Device {
lgfx::Panel_ST7789 _panel_instance;
lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI3_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read  = 16000000;
      cfg.spi_3wire  = true;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 40;
      cfg.pin_mosi = 45;
      cfg.pin_miso = -1;
      cfg.pin_dc   = 41;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs    = 42;
      cfg.pin_rst   = 39;
      cfg.pin_busy  = -1;
      cfg.memory_width  = 240;
      cfg.memory_height = 320;
      cfg.panel_width   = 240;
      cfg.panel_height  = 320;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable = false;
      cfg.invert = true;
      cfg.rgb_order = true;
      cfg.dlen_16bit = false;
      cfg.bus_shared = true;
      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance);
  }
};

LGFX tft;

// ============================================
// I2C KONFIGURATION
// ============================================

#define I2C0_SDA_PIN    11
#define I2C0_SCL_PIN    10

// CST328 Register
#define ESP_LCD_TOUCH_CST328_READ_Number_REG    0xD005
#define ESP_LCD_TOUCH_CST328_READ_XY_REG        0xD000
#define HYN_REG_MUT_DEBUG_INFO_MODE             0xD101
#define HYN_REG_MUT_NORMAL_MODE                 0xD109
#define HYN_REG_MUT_DEBUG_INFO_BOOT_TIME        0xD1FC
#define HYN_REG_MUT_DEBUG_INFO_TP_NTX           0xD1F4
#define CST328_LCD_TOUCH_MAX_POINTS             5

// Touch-Daten
struct CST328_Touch {
  uint8_t points;
  struct {
    uint16_t x, y;
    uint16_t strength;
  } coords[CST328_LCD_TOUCH_MAX_POINTS];
} touch_data = {0};

bool touchInitialized = false;

// ============================================
// I2C FUNKTIONEN
// ============================================

void initDualI2C() {
  Serial.println("\n🔧 DUAL I2C SYSTEM INITIALISIEREN");
  Serial.println("==================================");
  
  Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, 400000);
  delay(100);
  
  Wire1.begin(CST328_SDA_PIN, CST328_SCL_PIN, I2C_MASTER_FREQ_HZ);
  delay(100);
  
  Serial.printf("I2C Bus 0 (Wire): SDA=%d, SCL=%d\n", I2C0_SDA_PIN, I2C0_SCL_PIN);
  Serial.printf("I2C Bus 1 (Wire1): SDA=%d, SCL=%d\n", CST328_SDA_PIN, CST328_SCL_PIN);
  Serial.println("✅ Dual I2C System bereit");
}

bool Touch_I2C_Read(uint8_t addr, uint16_t reg, uint8_t *data, uint32_t length) {
  Wire1.beginTransmission(addr);
  Wire1.write((uint8_t)(reg >> 8));
  Wire1.write((uint8_t)reg);
  if (Wire1.endTransmission(true) != 0) return false;
  
  Wire1.requestFrom(addr, length);
  for (uint32_t i = 0; i < length; i++) {
    if (Wire1.available()) {
      data[i] = Wire1.read();
    } else {
      return false;
    }
  }
  return true;
}

bool Touch_I2C_Write(uint8_t addr, uint16_t reg, const uint8_t *data, uint32_t length) {
  Wire1.beginTransmission(addr);
  Wire1.write((uint8_t)(reg >> 8));
  Wire1.write((uint8_t)reg);
  for (uint32_t i = 0; i < length; i++) {
    Wire1.write(data[i]);
  }
  return (Wire1.endTransmission(true) == 0);
}

// ============================================
// CST328 FUNKTIONEN
// ============================================

bool CST328_Touch_Reset() {
  Serial.println("   - CST328 Hardware-Reset...");
  
  pinMode(CST328_RST_PIN, OUTPUT);
  digitalWrite(CST328_RST_PIN, HIGH);
  delay(50);
  digitalWrite(CST328_RST_PIN, LOW);
  delay(5);
  digitalWrite(CST328_RST_PIN, HIGH);
  delay(50);
  
  return true;
}

uint16_t CST328_Read_Config() {
  Serial.println("   - CST328 Konfiguration lesen...");
  
  uint8_t buf[24];
  
  if (!Touch_I2C_Write(CST328_ADDR, HYN_REG_MUT_DEBUG_INFO_MODE, buf, 0)) {
    Serial.println("   ❌ Debug-Modus fehlgeschlagen");
    return 0;
  }
  
  if (Touch_I2C_Read(CST328_ADDR, HYN_REG_MUT_DEBUG_INFO_BOOT_TIME, buf, 4)) {
    Serial.printf("   TouchPad_ID: 0x%02X,0x%02X,0x%02X,0x%02X\n", buf[0], buf[1], buf[2], buf[3]);
  }
  
  if (Touch_I2C_Read(CST328_ADDR, HYN_REG_MUT_DEBUG_INFO_TP_NTX, buf, 24)) {
    Serial.printf("   D1F4: 0x%02X,0x%02X,0x%02X,0x%02X\n", buf[0], buf[1], buf[2], buf[3]);
    uint16_t verification = (((uint16_t)buf[11] << 8) | buf[10]);
    Serial.printf("   Verification: 0x%04X\n", verification);
    
    Touch_I2C_Write(CST328_ADDR, HYN_REG_MUT_NORMAL_MODE, buf, 0);
    return verification;
  }
  
  return 0;
}

bool initCST328() {
  Serial.println("\n👆 CST328 Touch-Controller initialisieren...");
  
  CST328_Touch_Reset();
  delay(100);
  
  uint16_t verification = CST328_Read_Config();
  
  if (verification == 0xCACA) {
    Serial.println("✅ CST328 ERFOLGREICH initialisiert!");
    touchInitialized = true;
    return true;
  } else {
    Serial.printf("❌ CST328 Verification fehlgeschlagen: 0x%04X\n", verification);
    touchInitialized = false;
    return false;
  }
}

bool Touch_Read_Data() {
  if (!touchInitialized) return false;
  
  uint8_t buf[41];
  uint8_t clear = 0;
  
  if (!Touch_I2C_Read(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_Number_REG, buf, 1)) {
    return false;
  }
  
  uint8_t touch_cnt = buf[0] & 0x0F;
  
  if (touch_cnt == 0) {
    Touch_I2C_Write(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_Number_REG, &clear, 1);
    touch_data.points = 0;
    return false;
  }
  
  if (touch_cnt > CST328_LCD_TOUCH_MAX_POINTS) {
    Touch_I2C_Write(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_Number_REG, &clear, 1);
    return false;
  }
  
  if (!Touch_I2C_Read(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_XY_REG, &buf[1], 27)) {
    return false;
  }
  
  Touch_I2C_Write(CST328_ADDR, ESP_LCD_TOUCH_CST328_READ_Number_REG, &clear, 1);
  
  touch_data.points = touch_cnt;
  
  for (int i = 0; i < touch_cnt; i++) {
    int num = (i > 0) ? 2 : 0;
    
    touch_data.coords[i].x = (uint16_t)(((uint16_t)buf[(i * 5) + 2 + num] << 4) + 
                                       ((buf[(i * 5) + 4 + num] & 0xF0) >> 4));
    touch_data.coords[i].y = (uint16_t)(((uint16_t)buf[(i * 5) + 3 + num] << 4) + 
                                       (buf[(i * 5) + 4 + num] & 0x0F));
    touch_data.coords[i].strength = ((uint16_t)buf[(i * 5) + 5 + num]);
  }
  
  return true;
}

// ============================================
// AUTO-KALIBRIERUNG FÜR RAW-BEREICHE
// ============================================

void startAutoCalibration() {
  measured_x_min = 9999; measured_x_max = 0;
  measured_y_min = 9999; measured_y_max = 0;
  auto_calibration_active = true;
  
  Serial.println("🎯 AUTO-KALIBRIERUNG GESTARTET");
  Serial.println("Berühren Sie alle vier Ecken und die Mitte mehrmals!");
}

void updateAutoCalibration(uint16_t rawX, uint16_t rawY) {
  if (!auto_calibration_active) return;
  
  if (rawX < measured_x_min) measured_x_min = rawX;
  if (rawX > measured_x_max) measured_x_max = rawX;
  if (rawY < measured_y_min) measured_y_min = rawY;
  if (rawY > measured_y_max) measured_y_max = rawY;
  
  Serial.printf("📊 Raw Bereiche: X=%d-%d, Y=%d-%d\n", 
               measured_x_min, measured_x_max, measured_y_min, measured_y_max);
}

void finishAutoCalibration() {
  auto_calibration_active = false;
  Serial.println("✅ AUTO-KALIBRIERUNG BEENDET");
  Serial.printf("📋 EMPFOHLENE WERTE:\n");
  Serial.printf("   #define TOUCH_RAW_X_MIN %d\n", measured_x_min);
  Serial.printf("   #define TOUCH_RAW_X_MAX %d\n", measured_x_max);
  Serial.printf("   #define TOUCH_RAW_Y_MIN %d\n", measured_y_min);
  Serial.printf("   #define TOUCH_RAW_Y_MAX %d\n", measured_y_max);
}

// Konfigurierbare Mapping-Funktion
bool getTouchXY_Configurable(uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num) {
  *point_num = touch_data.points;
  
  if (touch_data.points > 0) {
    uint16_t rawX = touch_data.coords[0].x;
    uint16_t rawY = touch_data.coords[0].y;
    uint16_t mappedX, mappedY;
    
    // Basis-Koordinaten basierend auf TOUCH_SWAP_XY
    if (TOUCH_SWAP_XY) {
      // X/Y vertauschen
      mappedX = map(rawY, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, 0, DISPLAY_WIDTH - 1);
      mappedY = map(rawX, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, 0, DISPLAY_HEIGHT - 1);
    } else {
      // Normal
      mappedX = map(rawX, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, 0, DISPLAY_WIDTH - 1);
      mappedY = map(rawY, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, 0, DISPLAY_HEIGHT - 1);
    }
    
    // Invertierung anwenden
    if (TOUCH_INVERT_X || TOUCH_MIRROR_X) {
      mappedX = (DISPLAY_WIDTH - 1) - mappedX;
    }
    
    if (TOUCH_INVERT_Y || TOUCH_MIRROR_Y) {
      mappedY = (DISPLAY_HEIGHT - 1) - mappedY;
    }
    
    // Koordinaten begrenzen
    *x = constrain(mappedX, 0, DISPLAY_WIDTH - 1);
    *y = constrain(mappedY, 0, DISPLAY_HEIGHT - 1);
    
    if (strength) {
      *strength = touch_data.coords[0].strength;
    }
    
    touch_data.points = 0;
    return true;
  }
  
  return false;
}

// Vordefinierte Modi für Tests
bool getTouchXY_MultiMode(uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num) {
  *point_num = touch_data.points;
  
  if (touch_data.points > 0) {
    uint16_t rawX = touch_data.coords[0].x;
    uint16_t rawY = touch_data.coords[0].y;
    
    // Auto-Kalibrierung aktualisieren
    updateAutoCalibration(rawX, rawY);
    
    // Verwende gemessene Bereiche wenn verfügbar, sonst Defaults
    uint16_t x_min = auto_calibration_active && measured_x_min < 9999 ? measured_x_min : TOUCH_RAW_X_MIN;
    uint16_t x_max = auto_calibration_active && measured_x_max > 0 ? measured_x_max : TOUCH_RAW_X_MAX;
    uint16_t y_min = auto_calibration_active && measured_y_min < 9999 ? measured_y_min : TOUCH_RAW_Y_MIN;
    uint16_t y_max = auto_calibration_active && measured_y_max > 0 ? measured_y_max : TOUCH_RAW_Y_MAX;
    
    switch (current_touch_mode) {
      case TOUCH_MODE_0_ORIGINAL:
        // Ihr ursprüngliches Mapping
        *x = map(rawX, x_min, x_max, 319, 0);
        *y = map(rawY, y_min, y_max, 0, 239);
        break;
        
      case TOUCH_MODE_1_XY_SWAP:
        // X/Y vertauscht
        *x = map(rawY, y_min, y_max, 0, 319);
        *y = map(rawX, x_min, x_max, 0, 239);
        break;
        
      case TOUCH_MODE_2_XY_SWAP_INV_Y:
        // X/Y vertauscht + Y invertiert (EMPFOHLEN für Ihre Daten)
        *x = map(rawY, y_min, y_max, 0, 319);
        *y = map(rawX, x_min, x_max, 239, 0);
        break;
        
      case TOUCH_MODE_3_XY_SWAP_INV_X:
        // X/Y vertauscht + X invertiert
        *x = map(rawY, y_min, y_max, 319, 0);
        *y = map(rawX, x_min, x_max, 0, 239);
        break;
        
      case TOUCH_MODE_4_XY_SWAP_INV_BOTH:
        // X/Y vertauscht + beide invertiert
        *x = map(rawY, y_min, y_max, 319, 0);
        *y = map(rawX, x_min, x_max, 239, 0);
        break;
        
      case TOUCH_MODE_5_NORMAL_INV_Y:
        // Normal + Y invertiert
        *x = map(rawX, x_min, x_max, 0, 319);
        *y = map(rawY, y_min, y_max, 239, 0);
        break;
        
      case TOUCH_MODE_6_NORMAL_INV_X:
        // Normal + X invertiert
        *x = map(rawX, x_min, x_max, 319, 0);
        *y = map(rawY, y_min, y_max, 0, 239);
        break;
        
      case TOUCH_MODE_7_NORMAL_INV_BOTH:
        // Normal + beide invertiert
        *x = map(rawX, x_min, x_max, 319, 0);
        *y = map(rawY, y_min, y_max, 239, 0);
        break;
        
      default:
        return false;
    }
    
    // Koordinaten begrenzen
    *x = constrain(*x, 0, 319);
    *y = constrain(*y, 0, 239);
    
    if (strength) {
      *strength = touch_data.coords[0].strength;
    }
    
    touch_data.points = 0;
    return true;
  }
  
  return false;
}

// ============================================
// KALIBRIERUNGS-INTERFACE
// ============================================

void cycleTouchMode() {
  current_touch_mode = (touch_mapping_mode_t)((current_touch_mode + 1) % TOUCH_MODE_MAX);
  
  Serial.print("🔄 Touch-Modus gewechselt zu: ");
  switch (current_touch_mode) {
    case TOUCH_MODE_0_ORIGINAL: Serial.println("0: ORIGINAL"); break;
    case TOUCH_MODE_1_XY_SWAP: Serial.println("1: XY_SWAP"); break;
    case TOUCH_MODE_2_XY_SWAP_INV_Y: Serial.println("2: XY_SWAP_INV_Y ⭐"); break;
    case TOUCH_MODE_3_XY_SWAP_INV_X: Serial.println("3: XY_SWAP_INV_X"); break;
    case TOUCH_MODE_4_XY_SWAP_INV_BOTH: Serial.println("4: XY_SWAP_INV_BOTH"); break;
    case TOUCH_MODE_5_NORMAL_INV_Y: Serial.println("5: NORMAL_INV_Y"); break;
    case TOUCH_MODE_6_NORMAL_INV_X: Serial.println("6: NORMAL_INV_X"); break;
    case TOUCH_MODE_7_NORMAL_INV_BOTH: Serial.println("7: NORMAL_INV_BOTH"); break;
    default: Serial.println("UNKNOWN"); break;
  }
}

void drawTouchTestInterface() {
  tft.fillScreen(0x0000);
  
  // Header
  tft.setTextColor(0xFFFF);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("CST328 Touch-Test");
  
  // Modus anzeigen
  tft.setTextSize(1);
  tft.setTextColor(0xFFE0);
  tft.setCursor(10, 40);
  tft.printf("Modus: %d", current_touch_mode);
  
  const char* modeNames[] = {
    "ORIGINAL", "XY_SWAP", "XY_SWAP_INV_Y", "XY_SWAP_INV_X",
    "XY_SWAP_INV_BOTH", "NORMAL_INV_Y", "NORMAL_INV_X", "NORMAL_INV_BOTH"
  };
  
  tft.setCursor(60, 40);
  if (current_touch_mode < TOUCH_MODE_MAX) {
    tft.print(modeNames[current_touch_mode]);
    if (current_touch_mode == TOUCH_MODE_2_XY_SWAP_INV_Y) {
      tft.print(" *");
    }
  }
  
  // Test-Kreuze an den vier Ecken + Mitte
  struct {
    uint16_t x, y;
    uint16_t color;
    const char* label;
  } testPoints[] = {
    {30, 30, 0xF800, "1:OL"},       // Oben Links (Rot)
    {290, 30, 0x07E0, "2:OR"},      // Oben Rechts (Grün)
    {30, 210, 0x001F, "3:UL"},      // Unten Links (Blau)
    {290, 210, 0xFFE0, "4:UR"},     // Unten Rechts (Gelb)
    {160, 120, 0xF81F, "5:M"}       // Mitte (Magenta)
  };
  
  // Zeichne Kalibrierungs-Kreuze
  for (int i = 0; i < 5; i++) {
    // Kreuz zeichnen
    tft.drawLine(testPoints[i].x - 15, testPoints[i].y, 
                 testPoints[i].x + 15, testPoints[i].y, testPoints[i].color);
    tft.drawLine(testPoints[i].x, testPoints[i].y - 15,
                 testPoints[i].x, testPoints[i].y + 15, testPoints[i].color);
    tft.drawCircle(testPoints[i].x, testPoints[i].y, 20, testPoints[i].color);
    
    // Label
    tft.setTextColor(testPoints[i].color);
    tft.setCursor(testPoints[i].x - 15, testPoints[i].y + 25);
    tft.print(testPoints[i].label);
  }
  
  // Modus-Buttons
  tft.fillRect(10, 60, 70, 25, 0x07E0);
  tft.setTextColor(0x0000);
  tft.setCursor(15, 68);
  tft.print("MODUS+");
  
  tft.fillRect(90, 60, 70, 25, 0xF800);
  tft.setTextColor(0xFFFF);
  tft.setCursor(95, 68);
  tft.print("KONFIG");
  
  tft.fillRect(170, 60, 70, 25, 0x001F);
  tft.setTextColor(0xFFFF);
  tft.setCursor(175, 68);
  tft.print("AUTO-CAL");
  
  tft.fillRect(250, 60, 65, 25, 0x8410);
  tft.setTextColor(0xFFFF);
  tft.setCursor(255, 68);
  tft.print("EXIT");
  
  // Anweisungen
  tft.setTextColor(0x8410);
  tft.setCursor(10, 95);
  tft.println("Berühren Sie die Kreuze zum Testen:");
  tft.setCursor(10, 105);
  tft.println("OL=Oben Links, OR=Oben Rechts");
  tft.setCursor(10, 115);
  tft.println("UL=Unten Links, UR=Unten Rechts, M=Mitte");
}

void showConfigInterface() {
  tft.fillScreen(0x0000);
  
  tft.setTextColor(0xFFFF);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Touch-Konfiguration");
  
  tft.setTextSize(1);
  tft.setTextColor(0xFFE0);
  
  // Aktuelle Werte anzeigen
  int y = 50;
  tft.setCursor(10, y); tft.printf("TOUCH_SWAP_XY: %s", TOUCH_SWAP_XY ? "TRUE" : "FALSE"); y += 15;
  tft.setCursor(10, y); tft.printf("TOUCH_INVERT_X: %s", TOUCH_INVERT_X ? "TRUE" : "FALSE"); y += 15;
  tft.setCursor(10, y); tft.printf("TOUCH_INVERT_Y: %s", TOUCH_INVERT_Y ? "TRUE" : "FALSE"); y += 15;
  tft.setCursor(10, y); tft.printf("TOUCH_MIRROR_X: %s", TOUCH_MIRROR_X ? "TRUE" : "FALSE"); y += 15;
  tft.setCursor(10, y); tft.printf("TOUCH_MIRROR_Y: %s", TOUCH_MIRROR_Y ? "TRUE" : "FALSE"); y += 15;
  
  y += 10;
  tft.setCursor(10, y); tft.printf("Raw Range X: %d-%d", TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX); y += 15;
  tft.setCursor(10, y); tft.printf("Raw Range Y: %d-%d", TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX); y += 15;
  tft.setCursor(10, y); tft.printf("Display: %dx%d Rot:%d", DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_ROTATION); y += 15;
  
  // Buttons für Toggle
  struct {
    int x, y, w, h;
    const char* label;
    bool* var;
    uint16_t color;
  } buttons[] = {
    {10, 160, 60, 20, "SWAP_XY", &TOUCH_SWAP_XY, 0x07E0},
    {80, 160, 60, 20, "INV_X", &TOUCH_INVERT_X, 0xF800},
    {150, 160, 60, 20, "INV_Y", &TOUCH_INVERT_Y, 0x001F},
    {220, 160, 60, 20, "MIR_X", &TOUCH_MIRROR_X, 0xFFE0},
    {10, 190, 60, 20, "MIR_Y", &TOUCH_MIRROR_Y, 0xF81F},
    {250, 190, 65, 20, "ZURÜCK", nullptr, 0x8410}
  };
  
  for (int i = 0; i < 6; i++) {
    uint16_t color = buttons[i].color;
    if (buttons[i].var && *buttons[i].var) {
      color = 0xFFFF; // Weiß wenn aktiv
    }
    
    tft.fillRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, color);
    tft.setTextColor(0x0000);
    tft.setCursor(buttons[i].x + 5, buttons[i].y + 5);
    tft.print(buttons[i].label);
  }
}

void testTouchCalibration() {
  drawTouchTestInterface();
  unsigned long lastTouch = 0;
  
  while (calibration_mode) {
    uint16_t x, y, strength;
    uint8_t points;
    
    if (Touch_Read_Data() && millis() - lastTouch > 200) {
      bool hasTouch = false;
      
      if (calibration_mode) {
        hasTouch = getTouchXY_MultiMode(&x, &y, &strength, &points);
      } else {
        hasTouch = getTouchXY_Configurable(&x, &y, &strength, &points);
      }
      
      if (hasTouch) {
        lastTouch = millis();
        
        // Touch-Punkt anzeigen
        tft.fillCircle(x, y, 8, 0xFFFF);
        tft.drawCircle(x, y, 12, 0xFFFF);
        
        // Info-Bereich clearen und neu zeichnen
        tft.fillRect(10, 220, 300, 20, 0x0000);
        tft.setTextColor(0xFFE0);
        tft.setCursor(10, 225);
        tft.printf("Touch: (%d,%d) Raw:(%d,%d) Str:%d", 
                  x, y, touch_data.coords[0].x, touch_data.coords[0].y, strength);
        
        // Button-Checks
        if (y >= 60 && y <= 85) {
          if (x >= 10 && x <= 80) {
            // MODUS+ Button
            cycleTouchMode();
            delay(300);
            drawTouchTestInterface();
          } else if (x >= 90 && x <= 160) {
            // KONFIG Button
            showConfigInterface();
            delay(300);
          } else if (x >= 170 && x <= 240) {
            // AUTO-CAL Button
            if (auto_calibration_active) {
              finishAutoCalibration();
            } else {
              startAutoCalibration();
            }
            delay(300);
            drawTouchTestInterface();
          } else if (x >= 250 && x <= 315) {
            // EXIT Button
            calibration_mode = false;
            break;
          }
        }
        
        // Config-Interface Button-Checks
        if (x >= 10 && x <= 70 && y >= 160 && y <= 180) {
          TOUCH_SWAP_XY = !TOUCH_SWAP_XY;
          showConfigInterface();
          delay(300);
        } else if (x >= 80 && x <= 140 && y >= 160 && y <= 180) {
          TOUCH_INVERT_X = !TOUCH_INVERT_X;
          showConfigInterface();
          delay(300);
        } else if (x >= 150 && x <= 210 && y >= 160 && y <= 180) {
          TOUCH_INVERT_Y = !TOUCH_INVERT_Y;
          showConfigInterface();
          delay(300);
        } else if (x >= 220 && x <= 280 && y >= 160 && y <= 180) {
          TOUCH_MIRROR_X = !TOUCH_MIRROR_X;
          showConfigInterface();
          delay(300);
        } else if (x >= 10 && x <= 70 && y >= 190 && y <= 210) {
          TOUCH_MIRROR_Y = !TOUCH_MIRROR_Y;
          showConfigInterface();
          delay(300);
        } else if (x >= 250 && x <= 315 && y >= 190 && y <= 210) {
          drawTouchTestInterface();
          delay(300);
        }
        
        Serial.printf("🎯 TOUCH: Mode=%d, Mapped=(%d,%d), Raw=(%d,%d), Strength=%d\n",
                     current_touch_mode, x, y, 
                     touch_data.coords[0].x, touch_data.coords[0].y, strength);
      }
    }
    
    delay(50);
  }
}

// ============================================
// SETUP FUNKTIONEN
// ============================================

void setupBacklight() {
  Serial.println("🔆 Backlight aktivieren...");
  for (int i = 0; i <= 100; i += 25) {
    analogWrite(5, map(i, 0, 100, 0, 255));
    delay(50);
  }
  Serial.println("✅ Backlight aktiv");
}

void drawMainInterface() {
  tft.fillScreen(0x0000);
  
  // Header
  tft.setTextColor(0xFFFF);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("CST328 Touch System");
  
  // Status
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  if (touchInitialized) {
    tft.setTextColor(0x07E0);
    tft.println("✅ CST328 Touch OK (Wire1)");
  } else {
    tft.setTextColor(0xF800);
    tft.println("❌ CST328 Touch Error");
  }
  
  // Aktuelle Konfiguration
  tft.setTextColor(0xFFE0);
  tft.setCursor(10, 60);
  tft.printf("Config: SWAP=%s INV_X=%s INV_Y=%s", 
            TOUCH_SWAP_XY ? "Y" : "N",
            TOUCH_INVERT_X ? "Y" : "N", 
            TOUCH_INVERT_Y ? "Y" : "N");
  
  // Touch-Bereich
  tft.drawRect(10, 90, 300, 120, 0x07E0);
  tft.setTextColor(0x07E0);
  tft.setCursor(15, 95);
  tft.println("Touch-Aktiv-Bereich");
  tft.setCursor(15, 110);
  tft.println("Berühren für Touch-Test");
  
  // Kalibrierungs-Button
  tft.fillRect(220, 220, 90, 15, 0xF800);
  tft.setTextColor(0xFFFF);
  tft.setCursor(225, 223);
  tft.print("KALIBRIERUNG");
  
  // Info
  tft.setTextColor(0x8410);
  tft.setCursor(10, 240);
  tft.println("Hardware: ESP32-S3, ST7789, CST328 (Wire1)");
  tft.setCursor(10, 255);
  tft.println("Touch für Test, Kalibrierung für Setup");
}

void updateMainTouchDisplay() {
  uint16_t x, y, strength;
  uint8_t points;
  
  if (Touch_Read_Data() && getTouchXY_Configurable(&x, &y, &strength, &points)) {
    // Touch-Punkt zeichnen
    tft.fillCircle(x, y, 8, 0xF800);
    tft.drawCircle(x, y, 12, 0xFFFF);
    
    // Touch-Info
    tft.fillRect(10, 270, 300, 50, 0x0000);
    tft.setTextColor(0xFFE0);
    tft.setTextSize(1);
    tft.setCursor(10, 275);
    tft.printf("TOUCH: X=%d, Y=%d, Points=%d", x, y, points);
    tft.setCursor(10, 285);
    tft.printf("Raw: X=%d, Y=%d, Strength=%d", 
              touch_data.coords[0].x, touch_data.coords[0].y, strength);
    tft.setCursor(10, 295);
    tft.printf("Config: SWAP=%s INV_X=%s INV_Y=%s", 
              TOUCH_SWAP_XY ? "Y" : "N",
              TOUCH_INVERT_X ? "Y" : "N",
              TOUCH_INVERT_Y ? "Y" : "N");
    
    // Prüfen ob Kalibrierungs-Button gedrückt
    if (x >= 220 && x <= 310 && y >= 220 && y <= 235) {
      calibration_mode = true;
      testTouchCalibration();
      drawMainInterface(); // Nach Kalibrierung zurück zum Hauptmenü
    }
    
    Serial.printf("👆 MAIN TOUCH: X=%d, Y=%d (Raw: %d,%d) Strength=%d\n", 
                 x, y, touch_data.coords[0].x, touch_data.coords[0].y, strength);
  }
}

// ============================================
// SETUP & LOOP
// ============================================

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  Serial.println("🚀 ESP32-S3 CST328 TOUCH KALIBRIERUNGS-SYSTEM");
  Serial.println("===============================================");
  Serial.printf("Display: %dx%d Rotation:%d\n", DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_ROTATION);
  Serial.printf("Touch Config: SWAP_XY=%s INV_X=%s INV_Y=%s\n",
               TOUCH_SWAP_XY ? "TRUE" : "FALSE",
               TOUCH_INVERT_X ? "TRUE" : "FALSE", 
               TOUCH_INVERT_Y ? "TRUE" : "FALSE");
  Serial.println("===============================================");
  
  // Hardware Setup
  setupBacklight();
  
  Serial.println("\n📺 Display initialisieren...");
  tft.init();
  tft.setRotation(DISPLAY_ROTATION);
  Serial.println("✅ Display OK");
  
  // Dual I2C System
  initDualI2C();
  
  // CST328 initialisieren
  bool touchOK = initCST328();
  
  // Interface anzeigen
  drawMainInterface();
  
  // Ergebnis
  Serial.println("\n🎯 TOUCH-KALIBRIERUNGS-SYSTEM BEREIT!");
  if (touchOK) {
    Serial.println("✅ CST328 erfolgreich initialisiert");
    Serial.println("👆 Berühren Sie das Display für Touch-Tests");
    Serial.println("🔧 'KALIBRIERUNG' Button für erweiterte Tests");
    Serial.println("⚙️  Konfiguration in den #define Variablen anpassbar");
  } else {
    Serial.println("❌ CST328 nicht gefunden - prüfen Sie Hardware");
  }
  
  Serial.println("\n📋 VERFÜGBARE KOMMANDOS:");
  Serial.println("   - Berühren für Touch-Test");
  Serial.println("   - Kalibrierungs-Button für Modus-Tests");
  Serial.println("   - Konfiguration über #define Variablen");
}

void loop() {
  static unsigned long lastCheck = 0;
  static unsigned long lastStatus = 0;
  
  // Touch alle 50ms prüfen
  if (touchInitialized && !calibration_mode && millis() - lastCheck > 50) {
    lastCheck = millis();
    updateMainTouchDisplay();
  }
  
  // Status alle 10 Sekunden
  if (millis() - lastStatus > 10000) {
    lastStatus = millis();
    
    Serial.printf("⏱️  Up: %lus | Heap: %uKB | CST328: %s | Mode: %s\n",
                 millis() / 1000, ESP.getFreeHeap() / 1024,
                 touchInitialized ? "OK" : "ERROR",
                 calibration_mode ? "CALIBRATION" : "NORMAL");
  }
  
  delay(10);
}

/*
 * ============================================
 * LÖSUNG FÜR IHR PROBLEM UND KONFIGURATION
 * ============================================
 * 
 * IHR PROBLEM: Touch-Punkt nur bei Hälfte der Y-Position
 * URSACHE: Raw Y-Werte (318) außerhalb des definierten Bereichs (0-272)
 * LÖSUNG: TOUCH_RAW_Y_MAX von 272 auf 320+ erhöht
 * 
 * ============================================
 * SCHNELLE LÖSUNG - DIESE WERTE VERWENDEN:
 * ============================================
 * 
 * #define TOUCH_RAW_Y_MAX 320  // Statt 272
 * bool TOUCH_SWAP_XY = true
 * bool TOUCH_INVERT_Y = true  
 * current_touch_mode = TOUCH_MODE_2_XY_SWAP_INV_Y
 * 
 * ============================================
 * AUTO-KALIBRIERUNG VERWENDEN:
 * ============================================
 * 
 * 1. Touch "KALIBRIERUNG" Button
 * 2. Touch "AUTO-CAL" Button 
 * 3. Berühren Sie alle vier Ecken mehrmals
 * 4. Touch "AUTO-CAL" wieder zum Beenden
 * 5. Serial Monitor zeigt optimale #define Werte
 * 
 * ============================================
 * MANUELLE KONFIGURATION:
 * ============================================
 * 
 * 1. GRUNDKONFIGURATION (oben im Code):
 *    - TOUCH_SWAP_XY: X/Y vertauschen
 *    - TOUCH_INVERT_X: X-Achse invertieren  
 *    - TOUCH_INVERT_Y: Y-Achse invertieren
 *    - TOUCH_RAW_X/Y_MAX: Raw-Werte-Bereiche
 * 
 * 2. KALIBRIERUNG:
 *    - "KALIBRIERUNG" Button
 *    - "MODUS+" zum Testen verschiedener Modi
 *    - "KONFIG" für Live-Änderung der Parameter
 * 
 * 3. RAW-BEREICHE ERMITTELN:
 *    - Serial Monitor beobachten während Touch-Tests
 *    - Maximale/minimale Raw-Werte notieren
 *    - #define Werte entsprechend anpassen
 * 
 * ============================================
 * DEBUGGING:
 * ============================================
 * 
 * Serial Monitor zeigt:
 * - Raw-Koordinaten (X, Y)
 * - Gemappte Koordinaten (Display X, Y)
 * - Aktuelle Modi und Konfiguration
 * - Auto-Kalibrierungs-Bereiche
 * 
 * ============================================
 */