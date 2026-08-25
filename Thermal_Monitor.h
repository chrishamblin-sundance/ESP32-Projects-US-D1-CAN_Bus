#ifndef THERMAL_MONITOR_H
#define THERMAL_MONITOR_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_GridEYE_Arduino_Library.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define THERMAL_SDA 33
#define THERMAL_SCL 32

TwoWire ThermalWire = TwoWire(1);
GridEYE grideye;

inline float thermalPixels[64];
inline float maxThermalTemp = 0.0f;
inline bool humanHeatDetected = false;
inline bool thermalSensorFound = false;

inline bool setupThermalMonitor() {
  Serial.println("[THERMAL] Initializing secondary I2C bus (Wire1) on pins 33/32...");
  
  ThermalWire.begin(THERMAL_SDA, THERMAL_SCL, 100000);
  
  // Initialize GridEYE with address 0x69 on Wire1
  grideye.begin(0x69, ThermalWire);

  // Ping the I2C bus to verify the sensor responds at 0x69
  ThermalWire.beginTransmission(0x69);
  if (ThermalWire.endTransmission() == 0) {
    thermalSensorFound = true;
    Serial.println("[THERMAL] SparkFun GridEYE initialized successfully!");
    return true;
  }

  thermalSensorFound = false;
  Serial.println("[THERMAL] GridEYE NOT found on 0x69!");
  return false;
}

inline void updateThermalMonitor() {
  if (!thermalSensorFound) return;

  static unsigned long lastThermalRead = 0;
  if (millis() - lastThermalRead < 150) return; // ~7 FPS update rate
  lastThermalRead = millis();

  float currentMax = -100.0f;
  float currentMin = 100.0f;
  float totalTemp = 0.0f;

  for (unsigned char i = 0; i < 64; i++) {
    float temp = grideye.getPixelTemperature(i);
    thermalPixels[i] = temp;

    if (temp > currentMax) currentMax = temp;
    if (temp < currentMin) currentMin = temp;
    totalTemp += temp;
  }

  maxThermalTemp = currentMax;
  float avgTemp = totalTemp / 64.0f;

  // Person detected if hottest spot is at least 3.0°C above frame average
  // AND the maximum temperature reaches at least 26.0°C
  humanHeatDetected = (maxThermalTemp >= 26.0f) && (maxThermalTemp >= avgTemp + 3.0f);
}

inline void printThermalSerialGrid() {
  if (!thermalSensorFound) return;

  Serial.println("\n--- GRIDEYE 8x8 THERMAL GRID (°C) ---");
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      Serial.printf("%4.1f ", thermalPixels[y * 8 + x]);
    }
    Serial.println();
  }
  Serial.printf("Max Temp: %.1f°C | Human Detected: %s\n", maxThermalTemp, humanHeatDetected ? "YES" : "NO");
  Serial.println("-------------------------------------");
}

// 1. Boundary-checked point fetcher
inline float get_point(float *src, uint8_t src_rows, uint8_t src_cols, int8_t x, int8_t y) {
  if (x < 0) x = 0;
  if (x >= src_cols) x = src_cols - 1;
  if (y < 0) y = 0;
  if (y >= src_rows) y = src_rows - 1;
  return src[y * src_cols + x];
}

// 2. 1D Catmull-Rom Cubic Spline Interpolation
inline float cubicInterpolate(float p[4], float x) {
  return p[1] + 0.5f * x * (p[2] - p[0] + x * (2.0f * p[0] - 5.0f * p[1] + 4.0f * p[2] - p[3] + x * (3.0f * (p[1] - p[2]) + p[3] - p[0])));
}

// 3. 2D Bicubic Interpolator
inline float bicubicInterpolate(float p[16], float x, float y) {
  float arr[4];
  arr[0] = cubicInterpolate(&p[0], x);
  arr[1] = cubicInterpolate(&p[4], x);
  arr[2] = cubicInterpolate(&p[8], x);
  arr[3] = cubicInterpolate(&p[12], x);
  return cubicInterpolate(arr, y);
}

// 4. Image Rescaler (8x8 -> 16x16)
inline void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols) {
  float mu_x = (float)src_cols / dest_cols;
  float mu_y = (float)src_rows / dest_rows;
  float adj_2d[16];

  for (uint8_t y = 0; y < dest_rows; y++) {
    float src_y = (y + 0.5f) * mu_y - 0.5f;
    int8_t y_int = (int8_t)floor(src_y);
    float weight_y = src_y - y_int;

    for (uint8_t x = 0; x < dest_cols; x++) {
      float src_x = (x + 0.5f) * mu_x - 0.5f;
      int8_t x_int = (int8_t)floor(src_x);

      for (int8_t i = -1; i < 3; i++) {
        for (int8_t j = -1; j < 3; j++) {
          adj_2d[(i + 1) * 4 + (j + 1)] = get_point(src, src_rows, src_cols, x_int + j, y_int + i);
        }
      }
      dest[y * dest_cols + x] = bicubicInterpolate(adj_2d, src_x - x_int, weight_y);
    }
  }
}

// 5. Updated Screen Renderer using the 16x16 interpolated grid
// Change startX default to 76 (or pass 76 when calling the function)
inline void drawThermalHeatmap(Adafruit_SH1107 &disp, int startX = 1, int startY = 76) {
  if (!thermalSensorFound) return;

  float grid24x24[576];
  interpolate_image(thermalPixels, 8, 8, grid24x24, 24, 24);

  float minTemp = 100.0f;
  float maxTemp = -100.0f;

  for (int i = 0; i < 576; i++) {
    float t = grid24x24[i];
    if (t < minTemp) minTemp = t;
    if (t > maxTemp) maxTemp = t;
  }

  float range = maxTemp - minTemp;
  if (range < 1.0f) range = 1.0f;

  // Draws 2x2 blocks -> Total span: 48x48 display pixels
  for (int y = 0; y < 24; y++) {
    for (int x = 0; x < 24; x++) {
      float temp = grid24x24[y * 24 + x];
      int px = startX + (x * 2);
      int py = startY + (y * 2);

      if (temp >= 28.0f) {
        disp.fillRect(px, py, 2, 2, SH110X_WHITE);
      } else {
        float normalized = (temp - minTemp) / range;
        if (normalized > 0.60f) {
          disp.fillRect(px, py, 2, 2, SH110X_WHITE);
        } else if (normalized > 0.35f) {
          disp.drawPixel(px, py, SH110X_WHITE);
        }
      }
    }
  }
}



#endif
