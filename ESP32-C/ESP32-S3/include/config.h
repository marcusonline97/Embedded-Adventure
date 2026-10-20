#pragma once
// Keep this file out of git: it holds your WiFi password.

// ---- WiFi (2.4 GHz only) ----
#define WIFI_SSID "your-wifi-name"
#define WIFI_PASS "your-wifi-password"

// PC address. Leave "" to broadcast on the local subnet (no PC IP needed).
// Set e.g. "192.168.1.50" if broadcast gets blocked by your router.
#define PC_IP ""

// ---- Joystick wiring ----
// Use ADC1 pins (GPIO 1-10): ADC2 doesn't work while WiFi is on.
// Power the module from 3V3, not 5V.
#define JOY_X_PIN 2
#define JOY_Y_PIN 1
#define JOY_BTN_PIN 14      // joystick push button, wired to GND, uses the internal pull-up
#define JOY_BTN_DEBOUNCE_MS 20
#define JOY_INVERT_X 0
#define JOY_INVERT_Y 0
#define JOY_DEADZONE 0.08f  // fraction of full travel ignored around the center

// ---- Timing ----
#define JOY_SEND_HZ 100
#define HELLO_INTERVAL_MS 1000
