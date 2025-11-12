#include <WiFi.h>
#include <WebServer.h>
#include "driver/mcpwm.h"
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "Irancell-TF-i60-437E_1";
const char* password = "kabiri123456";
WebServer server(80);

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", []() {
    server.send(200, "text/plain", "Hello from ESP32");
  });

  server.begin();
}

void loop() {
  server.handleClient();

  //Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  delay(2000); // print every 2 seconds
}

