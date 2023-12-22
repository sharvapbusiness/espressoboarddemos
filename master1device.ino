#include <FastLED.h>
#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 46
#define NUM_LEDS 1
#define CHANNEL 1
#define BUTTON_PIN 1

CRGB leds[NUM_LEDS];

// ESP-NOW parameters
esp_now_peer_info_t slave;
uint8_t slave_address[] = {0x70, 0x04, 0x1D, 0xA4, 0x69, 0x04}; // Replace with your slave MAC address

bool ack = false;

// Callback function for ESP-NOW
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 if (status == ESP_NOW_SEND_SUCCESS) {
    ack = true;
    leds[0] = CRGB::Green;
    FastLED.show();
  } else {
    ack = false;
    leds[0] = CRGB::Red;
    FastLED.show();
  }
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

  // Once ESP-NOW is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(slave.peer_addr, slave_address, 6);
  slave.channel = CHANNEL;  
  slave.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&slave) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Set the button pin as input
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  // Send a test message to check connection
  uint8_t data[] = "Test";
  esp_now_send(slave_address, data, sizeof(data));
  delay(1000); // Wait for acknowledgment

  // If no acknowledgment received, assume the slave is disconnected
  if (!ack) {
    leds[0] = CRGB::Red;
    FastLED.show();
  }

  // Check if button is pressed
  if (digitalRead(BUTTON_PIN) == LOW) {
    uint8_t data[] = "Blue";   // Send "Blue" when button is pressed
    esp_now_send(slave_address, data, sizeof(data));
    delay(10); // Small delay to ensure data is sent
    leds[0] = CRGB::Blue;
    FastLED.show();
    delay(5000);
    if (ack) {
      leds[0] = CRGB::Green;
    } else {
      leds[0] = CRGB::Red;
    }
    FastLED.show();
  }
}
