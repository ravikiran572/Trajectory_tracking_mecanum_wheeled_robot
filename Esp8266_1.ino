#include <ESP8266WiFi.h>

const char *ssid = "ravi";
const char *password = "ravi@123";
const int port = 80;
WiFiServer server(port);

// Filter parameters
const float alpha = 0.2;  // Smoothing factor (0.1-0.3 recommended)
float filteredX = 0;
float filteredY = 0;

void setup() {
  Serial.begin(115200);
  
  // Start AP mode
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(ssid, password)) {
    Serial.println("Failed to start AP");
    while(1);
  }
  
  Serial.println("AP started successfully");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  
  server.begin();
  Serial.println("TCP server started");
}

void loop() {
  WiFiClient client = server.available();
  
  if (client && client.connected()) {
    while (Serial.available()) {
      String data = Serial.readStringUntil('\n');
      data.trim();
      
      // Parse and validate data
      int comma1 = data.indexOf(',');
      
      if (comma1 > 0) {
        // Extract and constrain values
        float x_in = constrain(data.substring(0, comma1).toFloat(), -2.0, 2.0);   
        float y_in = constrain(data.substring(comma1+1).toFloat(), -2.0, 2.0);
        
        
        // Apply exponential smoothing filter
        filteredX = alpha * x_in + (1 - alpha) * filteredX;
        filteredY = alpha * y_in + (1 - alpha) * filteredY;
        
        // Send filtered data to client
        client.println(String(filteredX,2) + "," + String(filteredY,2));           
      }
    }
  }
  client.stop();
  delay(10);  // Small delay to prevent watchdog reset
}
