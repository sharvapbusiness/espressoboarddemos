#include <FastLED.h>
#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 46
#define NUM_LEDS 1

CRGB leds[NUM_LEDS];

// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // Check if the received data is "Blue"
  if (strcmp((char*)incomingData, "Blue") == 0) {
    leds[0] = CRGB::Blue;
    FastLED.show();
    delay(5000);
  }
  leds[0] = CRGB::Green;
  FastLED.show();
}

void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  leds[0] = CRGB::Red;
  FastLED.show();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // put your main code here, to run repeatedly:
}
