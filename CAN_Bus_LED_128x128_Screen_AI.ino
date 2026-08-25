// ============================================================================
// SYSTEM HEADERS & DISPLAY SETUP
// ============================================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "logo.h"

// Define OLED screen physical pixel dimensions
#define SCREEN_WIDTH 128  
#define SCREEN_HEIGHT 128 
#define OLED_RESET -1     // Hardware reset pin (-1 if sharing microcontroller reset)

// Initialize SH1107 OLED via I2C at 400kHz fast-mode bus speed
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 400000, 100000);

// ============================================================================
// NEOPIXEL LED STRIP SETUP
// ============================================================================
#include <Adafruit_NeoPixel.h>
#define PIN 15            // ESP32 GPIO pin driving the NeoPixel strip
int Pixels = 30;          // Total number of addressable LEDs on the strip
#define BRIGHTNESS 50     // Set master brightness (~20% to manage power draw)

// Configure NeoPixel driver object (GRB color order @ 800kHz signal speed)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(Pixels, PIN, NEO_GRB + NEO_KHZ800);

// ============================================================================
// CAN BUS (TWAI) & INTER-CORE SYNCHRONIZATION SETUP
// ============================================================================
#include "driver/twai.h"  // ESP32 Two-Wire Automotive Interface (CAN controller)
#include <atomic>

#define TX_PIN GPIO_NUM_18 // CAN Transceiver TX Pin
#define RX_PIN GPIO_NUM_5  // CAN Transceiver RX Pin

// Atomic variables ensure thread-safe memory reads/writes between Core 0 (CAN Task) and Core 1 (Main Loop)
std::atomic<float> currentAltitude(0.0f); // Target distance/altitude in meters
std::atomic<uint16_t> currentSNR(0);      // Signal-to-Noise Ratio (signal confidence)

// ============================================================================
// AUXILIARY SYSTEM HEADERS
// ============================================================================
#include "CPU_Monitor.h"     // Helper routines for measuring dual-core ESP32 CPU load
#include "Thermal_Monitor.h" // Driver routines for the AMG8833 8x8 Thermal Camera grid

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Sets every LED on the NeoPixel strip to a single solid RGB color value
void LED_Full_Colour(int RED, int GREEN, int BLUE) {
  int pix_num = 0;
  while (pix_num < Pixels) {
    strip.setPixelColor(pix_num, RED, GREEN, BLUE);
    pix_num++;
  }
  strip.show(); // Push color memory buffer to actual hardware
}

// Draws the splash screen bitmap logo stored in logo.h centered on the OLED display
void testdrawbitmap(void) {
  display.clearDisplay();
  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1
  );
  display.display();
  delay(1000); // Hold splash screen briefly
}

// Configures and launches the ESP32 hardware TWAI/CAN bus driver at 1 Mbps speed
void setupTWAI() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); 
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI Driver Installed!");
  }
  if (twai_start() == ESP_OK) {
    Serial.println("TWAI Driver Started!");
  }
}

// ============================================================================
// FREERTOS BACKGROUND CAN TASK (Pinning CAN polling dedicated to Core 0)
// ============================================================================
void CAN_Task_Core0(void *pvParameters) {
  setupTWAI();
  twai_message_t rx_message;

  for (;;) {
    // Non-blocking frame check: waits up to 50ms for incoming CAN message
    if (twai_receive(&rx_message, pdMS_TO_TICKS(50)) == ESP_OK) {
      // Decode incoming standard message frame (Expects altitude and SNR data)
      if (rx_message.extd && rx_message.data_length_code >= 4) {
        uint16_t alt_cm = (rx_message.data[0] << 8) | rx_message.data[1];
        uint16_t snr    = (rx_message.data[2] << 8) | rx_message.data[3];

        // Store fetched parameters atomically for main loop evaluation
        currentAltitude.store(alt_cm / 100.0f); // Convert cm to meters
        currentSNR.store(snr);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // Yield to keep Watchdog Timer (WDT) happy
  }
}

// ============================================================================
// SETUP ROUTINE (Executes once on startup)
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(250); // Allow power supply lines and OLED hardware to stabilize

  // --- 1. Initialize Display ---
  display.begin(0x3C, true); // Primary I2C address 0x3C
  display.clearDisplay();
  display.display();

  // Draw hardware boot logo
  testdrawbitmap(); 
  delay(2000);

  display.clearDisplay();
  display.display();

  // --- 2. Initialize NeoPixel LEDs ---
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show(); // Ensure all LEDs start in OFF state

  // Visual LED power-on self test (Flash Red briefly)
  LED_Full_Colour(255, 0, 0); 
 
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Init. LEDs");
  display.display();
  delay(1000);

  LED_Full_Colour(0, 0, 0); // Turn off test pattern
  display.clearDisplay();
  display.display();

  // --- 3. Thermal Sensor Setup ---
  setupThermalMonitor();

  // --- 4. Launch FreeRTOS Task on Core 0 ---
  xTaskCreatePinnedToCore(
    CAN_Task_Core0, // Task function name
    "CAN_Task",      // Debug name
    4096,            // Stack size in words
    NULL,            // Input parameters
    1,               // Task priority
    NULL,            // Task handle pointer
    0                // Hardware Core ID (Core 0)
  );

  // --- 5. CPU Profiler Setup ---
  setupCPUMonitor();
}

// ============================================================================
// MAIN LOOP (Executes repeatedly on Core 1)
// ============================================================================
void loop() {
  // ----------------------------------------------------
  // STEP 1: FETCH LATEST SENSOR TELEMETRY
  // ----------------------------------------------------
  float alt    = currentAltitude.load(); // Retrieve radar distance (meters)
  uint16_t snr = currentSNR.load();      // Retrieve radar signal strength

  // ----------------------------------------------------
  // STEP 2: UPDATE THERMAL SENSOR DATA
  // ----------------------------------------------------
  updateThermalMonitor(); // Reads AMG8833 thermal camera array via I2C

  // Periodically output 8x8 thermal array matrix to Serial Monitor (every 500ms)
  static unsigned long lastSerialPrint = 0;
  if (millis() - lastSerialPrint >= 500) {
    lastSerialPrint = millis();
    printThermalSerialGrid();
  }

  // ----------------------------------------------------
  // STEP 3: SENSOR FUSION & TARGET CLASSIFICATION LOGIC
  // ----------------------------------------------------
  // Rule A: Target is physically valid if SNR > 13dB and distance is within 0.1m - 2.0m range
  bool isValidTarget = (snr > 13) && (alt > 0.1f) && (alt <= 2.0f);
  
  // Rule B: Radar flags a potential human based on typical cross-section return signal profile
  bool radarPerson   = isValidTarget && (snr >= 15 && snr <= 35);

  // Rule C: SENSOR FUSION CROSS-VERIFICATION
  // Target is ONLY classified as a verified human if BOTH Radar signature AND Thermal signature match!
  bool verifiedHuman = radarPerson && humanHeatDetected; 

  // ----------------------------------------------------
  // STEP 4: NON-BLOCKING LED ALERT INDICATOR LOGIC
  // ----------------------------------------------------
  static unsigned long lastTargetTime = 0;

  if (isValidTarget) {
    lastTargetTime = millis(); // Reset timeout tracker when any target is present

    if (verifiedHuman) { 
      // STATE 1: VERIFIED HUMAN DETECTED (Radar + Thermal)
      // Flash blue LEDs at a variable rate that speeds up as distance decreases
      static unsigned long lastFlashTime = 0;
      static bool flashState = false;

      // Scale flash delay between 50ms (close/fast) and 400ms (far/slow)
      int flashInterval = map(constrain(alt * 100, 30, 300), 30, 300, 50, 400);

      if (millis() - lastFlashTime >= flashInterval) {
        lastFlashTime = millis();
        flashState = !flashState; // Toggle LED on/off state
        LED_Full_Colour(0, 0, flashState ? 255 : 0); // Flash BLUE
      }
    } else {
      // STATE 2: GENERAL UNVERIFIED TARGET DETECTED (Object/Vehicle)
      // Transition LED colors continuously from Green (Far = 3.0m) to Red (Close = 0.3m)
      float clampedDist = constrain(alt, 0.3f, 3.0f);
      int redVal   = map(clampedDist * 100, 30, 300, 255, 0);   // Nearer = Stronger Red
      int greenVal = map(clampedDist * 100, 30, 300, 0, 255);   // Farther = Stronger Green
      LED_Full_Colour(redVal, greenVal, 0);
    }
 } else if (millis() - lastTargetTime >= 3000) {
    // ----------------------------------------------------
    // IDLE SCANNING MODE (With Navigation Lights on Wingtips)
    // ----------------------------------------------------
    static int wingPos = 0;
    static int scanDir = 1;
    static unsigned long lastScanUpdate = 0;

    if (millis() - lastScanUpdate >= 80) { // Frame rate for sweep animation
      lastScanUpdate = millis();
      strip.clear();

      // 1. Navigation Lights at Wingtips (Always ON during sweep)
      // Port (Left Wing Tip)  = RED
      // Starboard (Right Wing Tip) = GREEN
      strip.setPixelColor(14, strip.Color(255, 0, 0));   // LED 14: Left Wingtip (Port)
      strip.setPixelColor(29, strip.Color(0, 255, 0));   // LED 29: Right Wingtip (Starboard)

      // 2. Dual Synchronized Sweep (0->13 on Wing 1, 15->28 on Wing 2)
      const int SWEEP_LEN = 14; // 14 controllable LEDs per side (excluding tips)

      for (int i = -2; i <= 2; i++) {
        int pos = wingPos + i;
        if (pos >= 0 && pos < SWEEP_LEN) {
          int brightness = 255 - (abs(i) * 85); // Center pulse brightest, edges fade
          
          int leftPixel  = pos;        // Sweeps 0 (body) -> 13 (near tip)
          int rightPixel = 15 + pos;   // Sweeps 15 (body) -> 28 (near tip)

          strip.setPixelColor(leftPixel,  strip.Color(brightness, brightness, brightness));
          strip.setPixelColor(rightPixel, strip.Color(brightness, brightness, brightness));
        }
      }

      strip.show();

      // Bounce scanning position back and forth along the length of the wings
      wingPos += scanDir;
      if (wingPos <= 0 || wingPos >= SWEEP_LEN - 1) {
        scanDir *= -1; // Reverse direction at body and inner tip
      }
    }
  }

  // ----------------------------------------------------
  // STEP 5: CALCULATE CPU UTILIZATION
  // ----------------------------------------------------
  updateCPULoad(); // Computes execution load for both CPU cores

  // ----------------------------------------------------
  // STEP 6: NON-BLOCKING OLED GUI RENDER ROUTINE (20 FPS / 50ms)
  // ----------------------------------------------------
  static unsigned long lastOLEDUpdate = 0;

  if (millis() - lastOLEDUpdate >= 50) {
    lastOLEDUpdate = millis();

    display.clearDisplay();
    
    // --- Header Section: Title & System Performance Diagnostics ---
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("US-D1 Radar");
    display.print("C0:");
    display.print(cpuCore0.load());
    display.print("% C1:");
    display.print(cpuCore1.load());
    display.println("%");
    display.println("----------------");

    // --- Primary Metric: Distance Readout ---
    display.setTextSize(2);
    display.println("Dist:");
    display.setTextSize(5); // Large readable numbers
    display.print(alt, 2);  // Print distance rounded to 2 decimal places

    // --- Footer Section: Target Classification State Machine ---
    display.setTextSize(1);
    display.setCursor(0, 80);
    display.print("Target: ");

    if (verifiedHuman) {
      display.println("HUMAN");    // Full consensus: Radar + Thermal confirm human
    } else if (radarPerson) {
      display.println("RADAR?");   // Candidate: Radar sees human profile, thermal pending
    } else if (isValidTarget) {
      display.println("OBJECT");   // Target detected, but not human profile
    } else {
      display.println("NONE");     // Clear space / No valid target
    }

    // Display Signal-to-Noise Ratio (Radar confidence)
    display.print("SNR: ");
    display.print(snr);
    display.println(" dB");

    // --- Graphic Widget: Draw AMG8833 Thermal Heatmap Array ---
    drawThermalHeatmap(display, 74, 83); // Render 8x8 pixel grid starting at X=74, Y=83

    // Render a high-contrast white box overlay on top of thermal map when a human is confirmed
    if (verifiedHuman) {
      display.drawRect(72, 81, 52, 52, SH110X_WHITE);
    }    
    
    // Push completed frame to physical screen driver
    display.display(); 
  }

  // ----------------------------------------------------
  // STEP 7: RTOS SCHEDULER YIELD
  // ----------------------------------------------------
  vTaskDelay(pdMS_TO_TICKS(1)); // Prevents CPU Core 1 task starvation
}
