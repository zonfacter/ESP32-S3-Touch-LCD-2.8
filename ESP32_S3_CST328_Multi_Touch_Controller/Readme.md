# ESP32-S3 CST328 Multi-Touch System

Ein vollständiges Multi-Touch-System mit Gesten-Erkennung für ESP32-S3 mit CST328 Touch-Controller und ST7789 Display.

![Multi-Touch Demo](https://img.shields.io/badge/ESP32--S3-Multi--Touch-blue) ![CST328](https://img.shields.io/badge/CST328-5%20Finger-green) ![Performance](https://img.shields.io/badge/Performance-43%20FPS-orange)

## 🎯 Features

- ✅ **Bis zu 5 gleichzeitige Touch-Punkte**
- ✅ **Konflikt-freie Gesten-Erkennung** 
- ✅ **Live-Visualisierung** aller Touch-Punkte
- ✅ **60fps Performance** (~43fps real)
- ✅ **Präzise Koordinaten-Mapping**
- ✅ **Robuste Hardware-Abstraktionsschicht**

## 🖐️ Unterstützte Gesten

### Ein-Finger-Gesten
- **TAP** - Kurzer Touch (< 250ms, < 20px Bewegung)
- **DOUBLE_TAP** - Zwei Taps innerhalb 400ms
- **LONG_PRESS** - Touch > 800ms ohne Bewegung
- **SWIPE** - Bewegung > 30px (Links/Rechts/Oben/Unten)

### Multi-Finger-Gesten
- **TWO_FINGER_TAP** - Beide Finger gleichzeitig
- **THREE_FINGER_TAP** - 3+ Finger gleichzeitig

### Geplante Erweiterungen
- **PINCH_IN/OUT** - Zoom-Gesten
- **ROTATE** - Rotations-Gesten
- **VELOCITY_SWIPE** - Schwung-basierte Gesten

## 🔧 Hardware-Anforderungen

### Hauptkomponenten
- **ESP32-S3** Development Board
- **ST7789 Display** (320x240, SPI)
- **CST328 Touch-Controller** (I2C, bis zu 5 Touch-Punkte)

### Pin-Konfiguration

```cpp
// Display (SPI)
#define DISPLAY_SCLK    40
#define DISPLAY_MOSI    45
#define DISPLAY_DC      41
#define DISPLAY_CS      42
#define DISPLAY_RST     39
#define BACKLIGHT_PIN   5

// Touch Controller (I2C Wire1)
#define CST328_SDA_PIN  1
#define CST328_SCL_PIN  3
#define CST328_INT_PIN  4
#define CST328_RST_PIN  2
#define CST328_ADDR     0x1A

// Zusätzliche I2C-Geräte (I2C Wire)
#define I2C0_SDA_PIN    11
#define I2C0_SCL_PIN    10
```

### Touch-Mapping-Parameter

```cpp
// Kalibrierte Werte für CST328
#define TOUCH_RAW_X_MIN     0
#define TOUCH_RAW_X_MAX     240
#define TOUCH_RAW_Y_MIN     0  
#define TOUCH_RAW_Y_MAX     320

// Koordinaten-Transformation
bool TOUCH_SWAP_XY = true;      // X/Y vertauschen
bool TOUCH_INVERT_X = false;    // X-Achse invertieren
bool TOUCH_INVERT_Y = true;     // Y-Achse invertieren
```

## 🚀 Quick Start

### 1. Installation

```bash
# Arduino IDE Bibliotheken installieren
# - LovyanGFX Library
# - ESP32 Board Package

# Repository klonen
git clone https://github.com/your-repo/esp32-multitouch
cd esp32-multitouch
```

### 2. Hardware-Setup

1. **Display anschließen** (SPI-Bus)
2. **Touch-Controller anschließen** (I2C Wire1)
3. **Backlight-Pin konfigurieren**
4. **Touch-Kalibrierung durchführen**

### 3. Code hochladen

```cpp
// Grundlegende Initialisierung
void setup() {
  Serial.begin(115200);
  
  setupBacklight();
  tft.init();
  tft.setRotation(DISPLAY_ROTATION);
  
  initDualI2C();
  initCST328();
  
  drawStaticInterface();
}
```

## 📊 Performance

### Benchmark-Ergebnisse

```
📊 FPS: 43.2 | Aktive Touches: 2 | Heap: 343KB
🎭 GESTE: DOUBLE_TAP at (113,142) value=2.00 fingers=1
🎭 GESTE: SWIPE_RIGHT at (199,150) value=195.58 fingers=1
```

- **Frame Rate:** 40-43 FPS konstant
- **Heap Usage:** 343KB stabil
- **Touch Latency:** ~17ms
- **Gesten-Präzision:** ±2-3 Pixel

## 🎭 Gesten-API

### Event-Handler

```cpp
void onGestureDetected(GestureEvent gesture) {
  switch (gesture.type) {
    case GESTURE_TAP:
      Serial.printf("Tap at (%d,%d)\n", gesture.x, gesture.y);
      break;
      
    case GESTURE_DOUBLE_TAP:
      Serial.printf("Double-Tap at (%d,%d)\n", gesture.x, gesture.y);
      // Zoom-In Action
      break;
      
    case GESTURE_SWIPE_LEFT:
      Serial.printf("Swipe Left, distance: %.2f\n", gesture.value);
      // Navigation zurück
      break;
      
    case GESTURE_TWO_FINGER_TAP:
      Serial.printf("Two-Finger at (%d,%d)\n", gesture.x, gesture.y);
      // Kontext-Menü öffnen
      break;
  }
}
```

### Touch-Koordinaten abrufen

```cpp
void getActiveTouches() {
  for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
    if (touch_points[i].active) {
      uint16_t x = touch_points[i].x;
      uint16_t y = touch_points[i].y;
      uint16_t strength = touch_points[i].strength;
      
      // Touch-Punkt verarbeiten
      processTouch(i, x, y, strength);
    }
  }
}
```

## 🔮 Mögliche Erweiterungen

### 1. Pinch/Zoom-Gesten

```cpp
// Implementierung für Zwei-Finger Zoom
void processPinchGesture() {
  if (active_touch_count == 2) {
    float current_distance = calculateDistance(touch_points[0], touch_points[1]);
    static float initial_distance = 0;
    
    if (initial_distance == 0) {
      initial_distance = current_distance;
    }
    
    float zoom_factor = current_distance / initial_distance;
    
    if (abs(zoom_factor - 1.0) > 0.1) {
      if (zoom_factor > 1.0) {
        triggerGesture(GESTURE_PINCH_OUT, center_x, center_y, zoom_factor, 2);
      } else {
        triggerGesture(GESTURE_PINCH_IN, center_x, center_y, zoom_factor, 2);
      }
    }
  }
}
```

### 2. Rotations-Gesten

```cpp
// Implementierung für Zwei-Finger Rotation
void processRotationGesture() {
  if (active_touch_count == 2) {
    float current_angle = calculateAngle(touch_points[0], touch_points[1]);
    static float initial_angle = 0;
    
    if (initial_angle == 0) {
      initial_angle = current_angle;
    }
    
    float angle_diff = current_angle - initial_angle;
    
    // Normalisierung für 360°-Rotation
    if (angle_diff > 180) angle_diff -= 360;
    if (angle_diff < -180) angle_diff += 360;
    
    if (abs(angle_diff) > 15) { // 15° Mindest-Rotation
      if (angle_diff > 0) {
        triggerGesture(GESTURE_ROTATE_CW, center_x, center_y, angle_diff, 2);
      } else {
        triggerGesture(GESTURE_ROTATE_CCW, center_x, center_y, abs(angle_diff), 2);
      }
    }
  }
}
```

### 3. Touch-UI-Elemente

```cpp
// Button-Klasse für Touch-Interface
class TouchButton {
  private:
    uint16_t x, y, width, height;
    String label;
    uint16_t color;
    bool pressed;
    void (*callback)();
    
  public:
    TouchButton(uint16_t x, uint16_t y, uint16_t w, uint16_t h, 
               String lbl, uint16_t col, void (*cb)()) :
      x(x), y(y), width(w), height(h), label(lbl), 
      color(col), callback(cb), pressed(false) {}
    
    bool isPressed(uint16_t touch_x, uint16_t touch_y) {
      return (touch_x >= x && touch_x <= x + width &&
              touch_y >= y && touch_y <= y + height);
    }
    
    void draw() {
      uint16_t btn_color = pressed ? 0xFFFF : color;
      tft.fillRect(x, y, width, height, btn_color);
      tft.drawRect(x, y, width, height, 0xFFFF);
      
      tft.setTextColor(pressed ? 0x0000 : 0xFFFF);
      tft.setCursor(x + 5, y + 5);
      tft.print(label);
    }
    
    void handleTouch(uint16_t touch_x, uint16_t touch_y, bool is_active) {
      bool was_pressed = pressed;
      pressed = is_active && isPressed(touch_x, touch_y);
      
      if (!was_pressed && pressed && callback) {
        callback(); // Button wurde gedrückt
      }
      
      if (was_pressed != pressed) {
        draw(); // Neu zeichnen wenn sich Status ändert
      }
    }
};

// Verwendung
TouchButton btn1(10, 220, 80, 30, "Button 1", 0x07E0, []() {
  Serial.println("Button 1 gedrückt!");
});
```

### 4. Velocity-basierte Gesten

```cpp
// Geschwindigkeits-Tracking für Schwung-Gesten
void processVelocityGestures() {
  static unsigned long last_time = 0;
  static uint16_t last_x = 0, last_y = 0;
  
  if (active_touch_count == 1 && touch_points[0].active) {
    unsigned long now = millis();
    uint16_t current_x = touch_points[0].x;
    uint16_t current_y = touch_points[0].y;
    
    if (last_time > 0) {
      float dt = (now - last_time) / 1000.0; // Sekunden
      float dx = current_x - last_x;
      float dy = current_y - last_y;
      
      float velocity = sqrt(dx*dx + dy*dy) / dt; // Pixel pro Sekunde
      
      if (velocity > 500) { // Schneller Swipe
        triggerGesture(GESTURE_VELOCITY_SWIPE, current_x, current_y, velocity, 1);
      }
    }
    
    last_time = now;
    last_x = current_x;
    last_y = current_y;
  } else {
    last_time = 0; // Reset bei Touch-Ende
  }
}
```

### 5. Multi-Touch-Zeichnen

```cpp
// Implementierung für Multi-Touch-Zeichnen
class MultiTouchDrawing {
  private:
    uint16_t trail_colors[MAX_TOUCH_POINTS] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F};
    
  public:
    void drawTouchTrails() {
      for (int i = 0; i < MAX_TOUCH_POINTS; i++) {
        if (touch_points[i].active && touch_points[i].was_active_last_frame) {
          // Linie von letzter Position zur aktuellen
          static uint16_t last_pos[MAX_TOUCH_POINTS][2];
          
          if (last_pos[i][0] != 0 || last_pos[i][1] != 0) {
            tft.drawLine(last_pos[i][0], last_pos[i][1], 
                        touch_points[i].x, touch_points[i].y, 
                        trail_colors[i]);
          }
          
          last_pos[i][0] = touch_points[i].x;
          last_pos[i][1] = touch_points[i].y;
        }
      }
    }
    
    void clearCanvas() {
      tft.fillRect(10, 70, 300, 130, 0x0000);
    }
};
```

### 6. Erweiterte Kalibrierung

```cpp
// Auto-Kalibrierung für verschiedene Hardware-Varianten
void autoCalibrateTouchMapping() {
  Serial.println("🎯 Auto-Kalibrierung gestartet...");
  Serial.println("Berühren Sie alle vier Ecken des Displays:");
  
  uint16_t measured_x_min = 9999, measured_x_max = 0;
  uint16_t measured_y_min = 9999, measured_y_max = 0;
  int calibration_points = 0;
  
  // Kalibrierungs-Interface
  tft.fillScreen(0x0000);
  tft.setTextColor(0xFFFF);
  tft.setCursor(10, 10);
  tft.println("TOUCH-KALIBRIERUNG");
  tft.setCursor(10, 30);
  tft.println("Berühren Sie alle vier Ecken:");
  
  // Kalibrierungs-Kreuze zeichnen
  drawCalibrationCrosses();
  
  while (calibration_points < 20) { // 20 Messpunkte sammeln
    if (Touch_Read_Data()) {
      for (int i = 0; i < touch_data.points; i++) {
        uint16_t rawX = touch_data.coords[i].x;
        uint16_t rawY = touch_data.coords[i].y;
        
        if (rawX < measured_x_min) measured_x_min = rawX;
        if (rawX > measured_x_max) measured_x_max = rawX;
        if (rawY < measured_y_min) measured_y_min = rawY;
        if (rawY > measured_y_max) measured_y_max = rawY;
        
        calibration_points++;
        
        tft.setCursor(10, 50);
        tft.printf("Punkte: %d/20", calibration_points);
      }
    }
    delay(100);
  }
  
  // Ergebnisse ausgeben
  Serial.println("✅ Kalibrierung abgeschlossen!");
  Serial.printf("Empfohlene Werte:\n");
  Serial.printf("#define TOUCH_RAW_X_MIN %d\n", measured_x_min);
  Serial.printf("#define TOUCH_RAW_X_MAX %d\n", measured_x_max);
  Serial.printf("#define TOUCH_RAW_Y_MIN %d\n", measured_y_min);
  Serial.printf("#define TOUCH_RAW_Y_MAX %d\n", measured_y_max);
}
```

## 🔧 Konfiguration

### Touch-Parameter anpassen

```cpp
// Gesten-Schwellenwerte
#define TAP_MAX_DURATION    250    // Max Zeit für Tap
#define TAP_MAX_MOVEMENT    20     // Max Bewegung für Tap
#define DOUBLE_TAP_INTERVAL 400    // Zeit zwischen Double-Taps
#define LONG_PRESS_DURATION 800    // Zeit für Long Press
#define SWIPE_MIN_DISTANCE  30     // Min Distanz für Swipe

// Hardware-spezifische Anpassungen
#define CST328_I2C_FREQ     400000 // I2C-Frequenz
#define TOUCH_UPDATE_RATE   17     // Update-Intervall (ms)
#define GESTURE_COOLDOWN    150    // Cooldown zwischen Gesten
```

### Display-Orientierung

```cpp
// Verschiedene Orientierungen unterstützt
#define DISPLAY_ROTATION 0  // Portrait
#define DISPLAY_ROTATION 1  // Landscape (USB rechts)
#define DISPLAY_ROTATION 2  // Portrait umgedreht
#define DISPLAY_ROTATION 3  // Landscape (USB links)
```

## 🐛 Troubleshooting

### Häufige Probleme

**Problem:** Touch funktioniert nicht
```cpp
// Debug-Ausgabe aktivieren
void debugTouchHardware() {
  Serial.println("🔍 Touch-Hardware Debug:");
  
  // I2C-Scanner
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      Serial.printf("I2C Gerät gefunden: 0x%02X\n", addr);
    }
  }
  
  // CST328 Verification
  uint16_t verification = CST328_Read_Config();
  Serial.printf("CST328 Verification: 0x%04X\n", verification);
}
```

**Problem:** Falsche Koordinaten
```cpp
// Touch-Mapping testen
void testTouchMapping() {
  Serial.println("🎯 Touch-Mapping Test:");
  Serial.println("Berühren Sie die vier Ecken:");
  
  while (true) {
    if (Touch_Read_Data()) {
      for (int i = 0; i < touch_data.points; i++) {
        uint16_t rawX = touch_data.coords[i].x;
        uint16_t rawY = touch_data.coords[i].y;
        uint16_t dispX, dispY;
        
        mapRawToDisplay(rawX, rawY, &dispX, &dispY);
        
        Serial.printf("Raw:(%d,%d) -> Display:(%d,%d)\n", 
                     rawX, rawY, dispX, dispY);
      }
    }
    delay(500);
  }
}
```

**Problem:** Niedrige Performance
```cpp
// Performance-Optimierung
void optimizePerformance() {
  // CPU-Frequenz erhöhen
  setCpuFrequencyMhz(240);
  
  // Display-Update-Rate reduzieren
  #define TOUCH_UPDATE_RATE 20  // Statt 17ms
  
  // Unnötige Serial.print() entfernen
  #define DEBUG_OUTPUT false
}
```

## 📈 Roadmap

### Version 1.0 (Aktuell)
- ✅ Basis Multi-Touch-Funktionalität
- ✅ Ein-Finger-Gesten (Tap, Double-Tap, Swipe)
- ✅ Multi-Finger-Gesten (Two/Three-Finger-Tap)
- ✅ Live-Visualisierung

### Version 1.1 (Geplant)
- 🔄 Pinch/Zoom-Gesten
- 🔄 Rotations-Gesten
- 🔄 Velocity-basierte Gesten
- 🔄 Touch-UI-Framework

### Version 1.2 (Zukunft)
- 🔄 Gesture-Recording/Playback
- 🔄 Machine Learning Gesten-Erkennung
- 🔄 Wireless Touch-Übertragung
- 🔄 Multi-Display-Support

## 🤝 Contributing

Beiträge sind willkommen! Bitte beachten Sie:

1. **Fork** das Repository
2. **Branch** für Ihr Feature erstellen
3. **Tests** für neue Funktionen hinzufügen
4. **Pull Request** mit Beschreibung erstellen

### Development Setup

```bash
# Repository klonen
git clone https://github.com/your-repo/esp32-multitouch
cd esp32-multitouch

# Development Branch
git checkout -b feature/neue-geste

# Änderungen testen
# ... Code ändern ...

# Commit und Push
git add .
git commit -m "Neue Geste: Drei-Finger-Swipe"
git push origin feature/neue-geste
```

## 📄 Lizenz

MIT License - siehe [LICENSE](LICENSE) Datei für Details.

## 🙏 Credits

- **LovyanGFX** - Display-Bibliothek
- **ESP32 Arduino Core** - Hardware-Abstraktionsschicht
- **CST328 Community** - Touch-Controller-Dokumentation

## 📞 Support

- **Issues:** [GitHub Issues](https://github.com/your-repo/esp32-multitouch/issues)
- **Diskussionen:** [GitHub Discussions](https://github.com/your-repo/esp32-multitouch/discussions)
- **Email:** your-email@example.com

---

**⭐ Wenn dieses Projekt hilfreich war, geben Sie ihm einen Stern!** ⭐
