#include "WiFiManager.h"
#include "../config/NetworkConfig.h"
#include "../core/RobotContext.h"
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Dang ket noi WiFi");
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(250); Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    gRobot.myIP = WiFi.localIP().toString();
    Serial.println("Da ket noi WiFi nha.");
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    gRobot.myIP = WiFi.softAPIP().toString();
    Serial.printf("Khong vao duoc WiFi nha -> phat WiFi rieng '%s' (mat khau %s)\n", AP_SSID, AP_PASS);
  }
  Serial.println("==============================================");
  Serial.println("Mo trinh duyet:  http://" + gRobot.myIP);
  Serial.println("hoac:            http://robot.local");
  Serial.println("==============================================");
  if (MDNS.begin(MDNS_HOSTNAME)) MDNS.addService("http", "tcp", 80);
}
