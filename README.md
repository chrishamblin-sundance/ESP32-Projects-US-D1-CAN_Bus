# US-D1 Radar & Thermal Sensor Fusion System

An ESP32-powered sensor fusion system that combines **TWAI (CAN Bus) mmWave Radar** data with an **AMG8833 8x8 Thermal Infrared Camera**. The system processes telemetry across dual FreeRTOS cores to classify targets (Objects vs. Verified Humans), displaying real-time diagnostics on an **SH1107 OLED** and triggering dynamic visual alerts on a **30-pixel NeoPixel LED strip**.

---

## Software & Library Dependencies

Ensure the following libraries are installed in your Arduino IDE or PlatformIO environment:

* Adafruit_GFX (Core graphics library)

* Adafruit_SH1108 / Adafruit_SH110X (Display driver)

* Adafruit_NeoPixel (LED strip controller)

* ESP32 Board Support Package (Includes native driver/twai.h)


## 🚀 System Overview & Features

* **Dual-Core Processing (FreeRTOS):** 
  * **Core 0:** Dedicated TWAI/CAN task polling radar altitude/distance and SNR telemetry.
  * **Core 1:** Main execution loop managing AMG8833 thermal reads, target classification, LED effects, and OLED rendering.
* **Sensor Fusion Engine:**
  * Cross-verifies Radar signal-to-noise ratio (SNR) and target altitude against thermal heat differential array readings to accurately detect human presence.
* **Dynamic NeoPixel Status Lighting (30 LEDs):**
  * **Aviation Nav Lights:** Solid Port (Red, LED 14) and Starboard (Green, LED 29) wingtip markers.
  * **Idle Sweeper Mode:** Dual-wing synchronized white scanner animation when no target is detected for > 3 seconds.
  * **Object Range Ramp:** Color transition from Green (Far) to Red (Close) for unverified physical targets.
  * **Human Verified Alert:** High-priority proximity-pulsed Blue strobe when Radar + Thermal cross-verify a human target.
* **Diagnostic OLED UI (128x128 SH1107):**
  * Displays real-time altitude/distance, SNR levels, target classification (`NONE`, `OBJECT`, `RADAR?`, `HUMAN`), live CPU core load percentages, and a rendered 8x8 thermal array heatmap frame.

---

## 📌 Hardware Pinout & Wiring

| Peripheral | Component / Interface | ESP32 GPIO Pin / Bus | Notes |
| :--- | :--- | :--- | :--- |
| **CAN Transceiver** | TWAI TX | `GPIO 18` | 1 Mbps CAN bus speed |
| **CAN Transceiver** | TWAI RX | `GPIO 5` | Communicates with US-D1 Radar |
| **NeoPixel Strip** | Data Input (DIN) | `GPIO 15` | 30x WS2812B LEDs, 5V power |
| **SH1107 OLED** | I2C (SDA / SCL) | Standard I2C | 128x128 pixels @ Address `0x3C` (400kHz) |
| **AMG8833 Thermal**| I2C (SDA / SCL) | Standard I2C | 8x8 Infrared sensor array |

---

## 🛩️ LED Strip Layout Architecture

The NeoPixel strip is mapped as two mirrored 15-LED wings running from the central chassis out to the wingtips:

```text
[LEFT WING / PORT]                                [RIGHT WING / STARBOARD]
Body (0) ===> (13) [LED 14: RED NAV] | Chassis | Body (15) ===> (28) [LED 29: GREEN NAV]

               +-----------------------+
               |  Radar Data (CAN Bus) |
               +-----------+-----------+
                           |
               IsValidTarget? (SNR > 13, 0.1m - 2.0m)
                           |
            +--------------+--------------+
            |                             |
          [YES]                          [NO] ---> IDLE MODE: White Sweep + Nav Lights
            |
    Is Radar Profile Human?
       (SNR 15 - 35 dB)
            |
     +------+------+
     |             |
   [YES]          [NO] ---> OBJECT DETECTED: Green-to-Red Distance LED Ramp
     |
  Thermal Heat
   Detected?
 (AMG8833 Matrix)
     |
  +--+--+
  |     |
[YES]  [NO] -------------> RADAR UNVERIFIED Target state
  |
HUMAN VERIFIED! ---------> Fast Flashing Blue Alert LEDs + Highlight Box on OLED


