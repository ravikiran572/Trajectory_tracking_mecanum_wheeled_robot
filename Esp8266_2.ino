#include <ESP8266WiFi.h>

const char *ssid = "ravi";
const char *password = "ravi@123";
const char *serverIP = "192.168.4.1";
const int port = 80;

WiFiClient client;

void setup() {
  Serial.begin(115200);  // Connect to Arduino 2 via TX/RX
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(100);
}

void loop() {
  if (!client.connected()) {
    client.connect(serverIP, port);
    delay(100);
  }

  if (client.connected() && client.available()) {
    String data = client.readStringUntil('\n');
    data.trim();

    // Forward to Arduino 2
    Serial.println(data);
  }
  delay(8);
}
