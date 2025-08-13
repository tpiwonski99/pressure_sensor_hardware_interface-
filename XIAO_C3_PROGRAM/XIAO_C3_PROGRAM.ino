#include <WiFi.h>
#include <WiFiUdp.h>

#define ANALOG_PIN      A0
#define VOUT_ENABLE_PIN D3
#define THRESHOLD_V     0.1f 

const char* ssid     = "WIFI_NAME";
const char* password = "WIFI_PASSWORD";

const char* udpAddress = "PC_IP";
const int udpPort = 5005;

WiFiUDP udp;

uint8_t last_contact = 255; 

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(VOUT_ENABLE_PIN, OUTPUT);
  digitalWrite(VOUT_ENABLE_PIN, HIGH);

  Serial.printf("Łączenie z WiFi %s...\n", ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nPołączono!");
  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  int adc = analogRead(ANALOG_PIN);
  float voltage = adc * (3.3f / 4095.0f);

  uint8_t contact = (voltage > THRESHOLD_V) ? 1 : 0;

  if (contact != last_contact) {
    last_contact = contact;

    udp.beginPacket(udpAddress, udpPort);
    udp.write(&contact, 1);
    udp.endPacket();

    Serial.printf("Wysłano: %d\n", contact);
  }

  delay(50);
}
