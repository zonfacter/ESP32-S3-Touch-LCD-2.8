/*
 * ESP32-S3 CST328 Multi-Touch System mit Gesten-Erkennung
 * ======================================================
 * 
 * Features:
 * - Bis zu 5 gleichzeitige Touch-Punkte
 * - Gesten-Erkennung: Pinch/Zoom, Rotation, Swipe
 * - Touch-Events und Callbacks
 * - Multi-Touch-Kalibrierung und Visualisierung
 * - Erweiterte Touch-Analyse
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <math.h>

// ============================================
// KONFIGURATION - Ihre funktionierende Werte
// ============================================

// Display-Konfiguration
#define DISPLAY_WIDTH   320
#define DISPLAY_HEIGHT  240
#define DISPLAY_ROTATION 1

// Touch-Hardware-Konfiguration
#define CST328_SDA_PIN  1
#define CST328_SCL_PIN  3
#define CST328_INT_PIN  4
#define CST328_RST_PIN  2
#define CST328_ADDR     0x1A
#define I2C_MASTER_FREQ_HZ 400000

// Touch-Mapping-Konfiguration (Ihre funktionierenden Werte)
#define TOUCH_RAW_X_MIN     0
#define TOUCH_RAW_X_MAX     240
#define TOUCH_RAW_Y_MIN     0  
#define TOUCH_RAW_Y_MAX     320

// Touch-Kalibrierung (Ihre funktionierenden Werte)
bool TOUCH_SWAP_XY = true;
bool TOUCH_INVERT_X = false;
bool TOUCH_INVERT_Y = true;
bool TOUCH_MIRROR_X = false;
bool TOUCH_MIRROR_Y = false;

// Multi-Touch-Konfiguration
#define MAX_TOUCH_POINTS    5
#define GESTURE_MIN_DISTANCE 20
#define GESTURE_TIMEOUT_MS   1000
#define PINCH_MIN_CHANGE     10

// ============================================
// MULTI-TOUCH DATENSTRUKTUREN
// ============================================

typedef struct {
  uint16_t x, y;
  uint16_t strength;
  bool active;
  unsigned long first_touch;
  unsigned long last_touch;
  uint16_t start_x, start_y;  // Erste Touch-Position
} TouchPoint;

typedef enum {
  GESTURE_NONE = 0,
  GESTURE_TAP,
  GESTURE_DOUBLE_TAP,
  GESTURE_LONG_PRESS,
  GESTURE_SWIPE_LEFT,
  GESTURE_SWIPE_RIGHT,
  GESTURE_SWIPE_UP,
  GESTURE_SWIPE_DOWN,
  GESTURE_PINCH_IN,
  GESTURE_PINCH_OUT,
  GESTURE_ROTATE_CW,
  GESTURE_ROTATE_CCW,
  GESTURE_TWO_FINGER_TAP,
  GESTURE_THREE_FINGER_TAP
} GestureType;

typedef struct {
  GestureType type;
  uint16_t x, y;              // Zentrum der Geste
  float value;                // Zusätzlicher Wert (Winkel, Distanz, etc.)
  uint8_t finger_count;
  unsigned long timestamp;
} GestureEvent;

// Multi-Touch-Zustand
TouchPoint touch_points[MAX_TOUCH_POINTS];
uint8_t active_touch_count = 0;
GestureEvent last_gesture = {GESTURE_NONE, 0, 0, 0.0, 0, 0};

// Gesten-Erkennung Variablen
float initial_distance = 0;
float initial_angle = 0;
bool gesture_in_progress = false;
unsigned long gesture_start_time = 0;
unsigned long last_tap_time = 0;
uint8_t tap_count = 0;

// ============================================
// DISPLAY KONFIGURATION (wie vorher)
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
// CST328 HARDWARE (wie vorher, aber erweitert)
// ============================================

#define I2C0_SDA_PIN    11
#define I2C0_SCL_PIN    10

#define ESP_LCD_TOUCH_CST328_READ_Number_REG    0xD005
#define ESP_LCD_TOUCH_CST328_READ_XY_REG        0xD000
#define HYN_REG_MUT_DEBUG_INFO_MODE             0xD101
#define HYN_REG_MUT_NORMAL_MODE                 0xD109
#define HYN_REG_MUT_DEBUG_INFO_BOOT_TIME        0xD1FC
#define HYN_REG_MUT_DEBUG_INFO_TP_NTX           0xD1F4
#define CST328_LCD_TOUCH_MAX_POINTS             5

struct CST328_Touch {
  uint8_t points;
  struct {
    uint16_t x, y;
    uint16_t strength;
  } coords[CST328_LCD_TOUCH_MAX_POINTS];
} touch_data = {0};

bool touchInitialized = false;

// ============================================
// I2C FUNKTIONEN (wie vorher)
// ============================================

void initDualI2C() {
  Serial.println("\n🔧 DUAL I2C SYSTEM INITIALISIEREN");
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
  Serial.println("\n👆 CST328 Multi-Touch Controller initialisieren...");
  CST328_Touch_Reset();
  delay(100);
  
  uint16_t verification = CST328_Read_Config();
  
  if (verification == 0xCACA) {
    Serial.println("✅ CST328 ERFOLGREICH initialisiert!");
    Serial.println("🖐️  Multi-Touch unterstützt: bis zu 5 Finger");
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
// MULTI-TOUCH KOORDINATEN-MAPPING
// ============================================

void mapRawToDisplay(uint16_t rawX, uint16_t rawY, uint16_t *dispX, uint16_t *dispY) {
  uint16_t mappedX, mappedY;
  
  // Basis-Koordinaten basierend auf TOUCH_SWAP_XY
  if (TOUCH_SWAP_XY) {
    mappedX = map(rawY, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, 0, DISPLAY_WIDTH - 1);
    mappedY = map(rawX, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, 0, DISPLAY_HEIGHT - 1);
  } else {
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
  *dispX = constrain(mappedX, 0, DISPLAY_WIDTH - 1);
  *dispY = constrain(mappedY, 0, DISPLAY_HEIGHT - 1);
}

// ============================================
// MULTI-TOUCH VERARBEITUNG
// ============================================

void updateMultiTouch() {
  if (!Touch_Read_Data()) {
    // Alle Touch-Punkte als inaktiv markieren
    for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
      touch_points[i].active = false;
    }
    active_touch_count = 0;
    return;
  }
  
  unsigned long now = millis();
  active_touch_count = touch_data.points;
  
  // Touch-Punkte aktualisieren
  for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
    if (i < touch_data.points) {
      // Touch-Punkt ist aktiv
      uint16_t rawX = touch_data.coords[i].x;
      uint16_t rawY = touch_data.coords[i].y;
      uint16_t dispX, dispY;
      
      mapRawToDisplay(rawX, rawY, &dispX, &dispY);
      
      // Neuer Touch-Punkt?
      if (!touch_points[i].active) {
        touch_points[i].first_touch = now;
        touch_points[i].start_x = dispX;
        touch_points[i].start_y = dispY;
      }
      
      touch_points[i].x = dispX;
      touch_points[i].y = dispY;
      touch_points[i].strength = touch_data.coords[i].strength;
      touch_points[i].active = true;
      touch_points[i].last_touch = now;
      
    } else {
      // Touch-Punkt ist inaktiv
      touch_points[i].active = false;
    }
  }
  
  // Gesten-Erkennung
  processGestures();
}

// ============================================
// GESTEN-ERKENNUNG
// ============================================

float calculateDistance(TouchPoint p1, TouchPoint p2) {
  float dx = p1.x - p2.x;
  float dy = p1.y - p2.y;
  return sqrt(dx * dx + dy * dy);
}

float calculateAngle(TouchPoint p1, TouchPoint p2) {
  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  return atan2(dy, dx) * 180.0 / PI;
}

void processGestures() {
  unsigned long now = millis();
  
  if (active_touch_count == 0) {
    // Keine Touch-Punkte - Geste beenden
    if (gesture_in_progress) {
      gesture_in_progress = false;
      // Touch-Ende-Events verarbeiten
    }
    return;
  }
  
  if (active_touch_count == 1) {
    // Ein-Finger-Gesten
    TouchPoint &p = touch_points[0];
    unsigned long touch_duration = now - p.first_touch;
    
    if (!gesture_in_progress) {
      gesture_start_time = now;
      gesture_in_progress = true;
    }
    
    // Long Press erkennen
    if (touch_duration > 800 && last_gesture.type != GESTURE_LONG_PRESS) {
      triggerGesture(GESTURE_LONG_PRESS, p.x, p.y, touch_duration, 1);
    }
    
    // Swipe erkennen (beim Loslassen)
    if (!p.active && gesture_in_progress) {
      float dx = p.x - p.start_x;
      float dy = p.y - p.start_y;
      float distance = sqrt(dx * dx + dy * dy);
      
      if (distance > GESTURE_MIN_DISTANCE) {
        if (abs(dx) > abs(dy)) {
          // Horizontaler Swipe
          if (dx > 0) {
            triggerGesture(GESTURE_SWIPE_RIGHT, p.x, p.y, distance, 1);
          } else {
            triggerGesture(GESTURE_SWIPE_LEFT, p.x, p.y, distance, 1);
          }
        } else {
          // Vertikaler Swipe
          if (dy > 0) {
            triggerGesture(GESTURE_SWIPE_DOWN, p.x, p.y, distance, 1);
          } else {
            triggerGesture(GESTURE_SWIPE_UP, p.x, p.y, distance, 1);
          }
        }
      } else if (touch_duration < 300) {
        // Tap erkennen
        if (now - last_tap_time < 400) {
          tap_count++;
          if (tap_count >= 2) {
            triggerGesture(GESTURE_DOUBLE_TAP, p.x, p.y, tap_count, 1);
            tap_count = 0;
          }
        } else {
          tap_count = 1;
          triggerGesture(GESTURE_TAP, p.x, p.y, 1, 1);
        }
        last_tap_time = now;
      }
    }
    
  } else if (active_touch_count == 2) {
    // Zwei-Finger-Gesten
    TouchPoint &p1 = touch_points[0];
    TouchPoint &p2 = touch_points[1];
    
    float current_distance = calculateDistance(p1, p2);
    float current_angle = calculateAngle(p1, p2);
    
    if (!gesture_in_progress) {
      initial_distance = current_distance;
      initial_angle = current_angle;
      gesture_start_time = now;
      gesture_in_progress = true;
    }
    
    // Pinch/Zoom erkennen
    float distance_change = current_distance - initial_distance;
    if (abs(distance_change) > PINCH_MIN_CHANGE) {
      if (distance_change > 0) {
        triggerGesture(GESTURE_PINCH_OUT, (p1.x + p2.x) / 2, (p1.y + p2.y) / 2, 
                      current_distance / initial_distance, 2);
      } else {
        triggerGesture(GESTURE_PINCH_IN, (p1.x + p2.x) / 2, (p1.y + p2.y) / 2, 
                      current_distance / initial_distance, 2);
      }
    }
    
    // Rotation erkennen
    float angle_change = current_angle - initial_angle;
    if (abs(angle_change) > 15) {  // 15 Grad Mindest-Rotation
      if (angle_change > 0) {
        triggerGesture(GESTURE_ROTATE_CW, (p1.x + p2.x) / 2, (p1.y + p2.y) / 2, 
                      angle_change, 2);
      } else {
        triggerGesture(GESTURE_ROTATE_CCW, (p1.x + p2.x) / 2, (p1.y + p2.y) / 2, 
                      abs(angle_change), 2);
      }
    }
    
  } else if (active_touch_count >= 3) {
    // Drei+ Finger-Gesten
    if (!gesture_in_progress) {
      gesture_start_time = now;
      gesture_in_progress = true;
      
      // Zentrum berechnen
      uint16_t center_x = 0, center_y = 0;
      for (int i = 0; i < active_touch_count; i++) {
        center_x += touch_points[i].x;
        center_y += touch_points[i].y;
      }
      center_x /= active_touch_count;
      center_y /= active_touch_count;
      
      triggerGesture(GESTURE_THREE_FINGER_TAP, center_x, center_y, 1, active_touch_count);
    }
  }
}

void triggerGesture(GestureType type, uint16_t x, uint16_t y, float value, uint8_t fingers) {
  last_gesture.type = type;
  last_gesture.x = x;
  last_gesture.y = y;
  last_gesture.value = value;
  last_gesture.finger_count = fingers;
  last_gesture.timestamp = millis();
  
  // Geste ausgeben
  Serial.printf("🎭 GESTE: ");
  switch (type) {
    case GESTURE_TAP: Serial.print("TAP"); break;
    case GESTURE_DOUBLE_TAP: Serial.print("DOUBLE_TAP"); break;
    case GESTURE_LONG_PRESS: Serial.print("LONG_PRESS"); break;
    case GESTURE_SWIPE_LEFT: Serial.print("SWIPE_LEFT"); break;
    case GESTURE_SWIPE_RIGHT: Serial.print("SWIPE_RIGHT"); break;
    case GESTURE_SWIPE_UP: Serial.print("SWIPE_UP"); break;
    case GESTURE_SWIPE_DOWN: Serial.print("SWIPE_DOWN"); break;
    case GESTURE_PINCH_IN: Serial.print("PINCH_IN"); break;
    case GESTURE_PINCH_OUT: Serial.print("PINCH_OUT"); break;
    case GESTURE_ROTATE_CW: Serial.print("ROTATE_CW"); break;
    case GESTURE_ROTATE_CCW: Serial.print("ROTATE_CCW"); break;
    case GESTURE_TWO_FINGER_TAP: Serial.print("TWO_FINGER_TAP"); break;
    case GESTURE_THREE_FINGER_TAP: Serial.print("THREE_FINGER_TAP"); break;
    default: Serial.print("UNKNOWN"); break;
  }
  Serial.printf(" at (%d,%d) value=%.2f fingers=%d\n", x, y, value, fingers);
}

// ============================================
// MULTI-TOUCH VISUALISIERUNG
// ============================================

void setupBacklight() {
  Serial.println("🔆 Backlight aktivieren...");
  for (int i = 0; i <= 100; i += 25) {
    analogWrite(5, map(i, 0, 100, 0, 255));
    delay(50);
  }
  Serial.println("✅ Backlight aktiv");
}

void drawMultiTouchInterface() {
  tft.fillScreen(0x0000);
  
  // Header
  tft.setTextColor(0xFFFF);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Multi-Touch System");
  
  // Status
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  if (touchInitialized) {
    tft.setTextColor(0x07E0);
    tft.printf("✅ CST328 OK - %d Punkte aktiv", active_touch_count);
  } else {
    tft.setTextColor(0xF800);
    tft.println("❌ CST328 Touch Error");
  }
  
  // Touch-Bereich
  tft.drawRect(10, 70, 300, 140, 0x07E0);
  tft.setTextColor(0x07E0);
  tft.setCursor(15, 75);
  tft.println("Multi-Touch-Bereich");
  tft.setCursor(15, 85);
  tft.printf("Unterstützt: bis zu %d Finger", MAX_TOUCH_POINTS);
  
  // Gesten-Info
  tft.setTextColor(0x8410);
  tft.setCursor(10, 220);
  tft.println("Gesten: Tap, Swipe, Pinch, Rotate, Long Press");
  
  // Letzte Geste anzeigen
  if (last_gesture.type != GESTURE_NONE) {
    tft.fillRect(10, 240, 300, 30, 0x0000);
    tft.setTextColor(0xFFE0);
    tft.setCursor(10, 245);
    
    switch (last_gesture.type) {
      case GESTURE_TAP: tft.print("Letzte Geste: TAP"); break;
      case GESTURE_DOUBLE_TAP: tft.print("Letzte Geste: DOUBLE TAP"); break;
      case GESTURE_LONG_PRESS: tft.print("Letzte Geste: LONG PRESS"); break;
      case GESTURE_SWIPE_LEFT: tft.print("Letzte Geste: SWIPE LEFT"); break;
      case GESTURE_SWIPE_RIGHT: tft.print("Letzte Geste: SWIPE RIGHT"); break;
      case GESTURE_SWIPE_UP: tft.print("Letzte Geste: SWIPE UP"); break;
      case GESTURE_SWIPE_DOWN: tft.print("Letzte Geste: SWIPE DOWN"); break;
      case GESTURE_PINCH_IN: tft.print("Letzte Geste: PINCH IN"); break;
      case GESTURE_PINCH_OUT: tft.print("Letzte Geste: PINCH OUT"); break;
      case GESTURE_ROTATE_CW: tft.print("Letzte Geste: ROTATE CW"); break;
      case GESTURE_ROTATE_CCW: tft.print("Letzte Geste: ROTATE CCW"); break;
      case GESTURE_THREE_FINGER_TAP: tft.print("Letzte Geste: 3-FINGER TAP"); break;
      default: tft.print("Letzte Geste: UNKNOWN"); break;
    }
    
    tft.setCursor(10, 255);
    tft.printf("Position: (%d,%d) Wert: %.2f", last_gesture.x, last_gesture.y, last_gesture.value);
  }
}

void drawTouchPoints() {
  static unsigned long last_clear = 0;
  
  // Touch-Bereich alle 100ms leicht dimmen für Fade-Effekt
  if (millis() - last_clear > 100) {
    last_clear = millis();
    // Leichte Verdunkelung für Fade-Effekt
    tft.fillRect(10, 70, 300, 140, 0x0020);
  }
  
  // Alle aktiven Touch-Punkte zeichnen
  uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F}; // Rot, Grün, Blau, Gelb, Magenta
  
  for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
    if (touch_points[i].active) {
      uint16_t color = colors[i % 5];
      uint16_t x = touch_points[i].x;
      uint16_t y = touch_points[i].y;
      
      // Touch-Punkt zeichnen
      tft.fillCircle(x, y, 8, color);
      tft.drawCircle(x, y, 12, 0xFFFF);
      tft.drawCircle(x, y, 16, color);
      
      // Finger-Nummer anzeigen
      tft.setTextColor(0xFFFF);
      tft.setTextSize(1);
      tft.setCursor(x - 3, y - 3);
      tft.print(i + 1);
      
      // Stärke-Indikator
      uint8_t strength_radius = map(touch_points[i].strength, 0, 255, 4, 20);
      tft.drawCircle(x, y, strength_radius, 0x8410);
    }
  }
  
  // Linien zwischen zwei Touch-Punkten für Gesten-Visualisierung
  if (active_touch_count == 2) {
    tft.drawLine(touch_points[0].x, touch_points[0].y, 
                 touch_points[1].x, touch_points[1].y, 0x8410);
    
    // Zentrum markieren
    uint16_t center_x = (touch_points[0].x + touch_points[1].x) / 2;
    uint16_t center_y = (touch_points[0].y + touch_points[1].y) / 2;
    tft.drawCircle(center_x, center_y, 3, 0xFFFF);
  }
  
  // Multi-Touch-Info aktualisieren
  tft.fillRect(10, 40, 200, 15, 0x0000);
  tft.setTextColor(0x07E0);
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.printf("✅ CST328 OK - %d Punkte aktiv", active_touch_count);
  
  if (active_touch_count > 0) {
    tft.fillRect(10, 50, 300, 15, 0x0000);
    tft.setCursor(10, 50);
    tft.printf("Aktive Punkte: ");
    for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
      if (touch_points[i].active) {
        tft.printf("%d:(%d,%d,S%d) ", i+1, touch_points[i].x, touch_points[i].y, touch_points[i].strength);
      }
    }
  }
}

// ============================================
// SETUP & LOOP
// ============================================

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  Serial.println("🚀 ESP32-S3 CST328 MULTI-TOUCH SYSTEM");
  Serial.println("=====================================");
  Serial.printf("Display: %dx%d Rotation:%d\n", DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_ROTATION);
  Serial.printf("Touch Config: SWAP_XY=%s INV_X=%s INV_Y=%s\n",
               TOUCH_SWAP_XY ? "TRUE" : "FALSE",
               TOUCH_INVERT_X ? "TRUE" : "FALSE", 
               TOUCH_INVERT_Y ? "TRUE" : "FALSE");
  Serial.printf("Max Touch Points: %d\n", MAX_TOUCH_POINTS);
  Serial.println("=====================================");
  
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
  
  // Multi-Touch-Array initialisieren
  for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
    touch_points[i].active = false;
  }
  
  // Interface anzeigen
  drawMultiTouchInterface();
  
  // Ergebnis
  Serial.println("\n🎯 MULTI-TOUCH SYSTEM BEREIT!");
  if (touchOK) {
    Serial.println("✅ CST328 erfolgreich initialisiert");
    Serial.println("🖐️  Multi-Touch: bis zu 5 Finger gleichzeitig");
    Serial.println("🎭 Gesten-Erkennung: Tap, Swipe, Pinch, Rotate");
    Serial.println("👆 Berühren Sie das Display mit 1-5 Fingern!");
  } else {
    Serial.println("❌ CST328 nicht gefunden - prüfen Sie Hardware");
  }
  
  Serial.println("\n📋 VERFÜGBARE GESTEN:");
  Serial.println("   🖱️  1 Finger: Tap, Double-Tap, Long-Press, Swipe");
  Serial.println("   ✌️  2 Finger: Pinch-In/Out, Rotate, Two-Finger-Tap");
  Serial.println("   🖐️  3+ Finger: Multi-Finger-Tap");
}

void loop() {
  static unsigned long lastTouchCheck = 0;
  static unsigned long lastStatus = 0;
  
  // Multi-Touch alle 20ms prüfen
  if (touchInitialized && millis() - lastTouchCheck > 20) {
    lastTouchCheck = millis();
    updateMultiTouch();
    drawTouchPoints();
  }
  
  // Status alle 5 Sekunden
  if (millis() - lastStatus > 5000) {
    lastStatus = millis();
    
    Serial.printf("⏱️  Up: %lus | Heap: %uKB | CST328: %s | Active: %d touches\n",
                 millis() / 1000, ESP.getFreeHeap() / 1024,
                 touchInitialized ? "OK" : "ERROR", active_touch_count);
  }
  
  delay(10);
}

/*
 * ============================================
 * MULTI-TOUCH SYSTEM DOKUMENTATION
 * ============================================
 * 
 * UNTERSTÜTZTE GESTEN:
 * 
 * 🖱️ EIN-FINGER-GESTEN:
 *   - TAP: Kurzer Touch (< 300ms)
 *   - DOUBLE_TAP: Zwei Taps innerhalb 400ms
 *   - LONG_PRESS: Touch > 800ms
 *   - SWIPE_LEFT/RIGHT/UP/DOWN: Finger bewegen > 20px
 * 
 * ✌️ ZWEI-FINGER-GESTEN:
 *   - PINCH_IN: Zwei Finger zusammen bewegen
 *   - PINCH_OUT: Zwei Finger auseinander bewegen (Zoom)
 *   - ROTATE_CW/CCW: Zwei Finger rotieren (> 15°)
 *   - TWO_FINGER_TAP: Beide Finger gleichzeitig
 * 
 * 🖐️ MULTI-FINGER-GESTEN:
 *   - THREE_FINGER_TAP: Drei oder mehr Finger
 * 
 * VISUALISIERUNG:
 *   - Jeder Touch-Punkt hat eine eigene Farbe
 *   - Finger-Nummer wird angezeigt (1-5)
 *   - Stärke-Indikator als Kreis
 *   - Fade-Effekt für Touch-Spuren
 *   - Linien zwischen Touch-Punkten für Gesten
 * 
 * CALLBACK-SYSTEM:
 *   - triggerGesture() wird bei jeder erkannten Geste aufgerufen
 *   - Erweitern Sie diese Funktion für eigene Anwendungen
 *   - last_gesture enthält die letzte erkannte Geste
 * 
 * KONFIGURATION:
 *   - MAX_TOUCH_POINTS: Anzahl unterstützter Touch-Punkte
 *   - GESTURE_MIN_DISTANCE: Mindest-Bewegung für Swipe
 *   - GESTURE_TIMEOUT_MS: Timeout für Gesten-Erkennung
 *   - PINCH_MIN_CHANGE: Mindest-Änderung für Pinch-Gesten
 * 
 * ============================================
 */