#include "RobotWebServer.h"
#include "ApiHandlers.h"

static WebServer server(80);

WebServer& getWebServer() {
  return server;
}

void webServerInit() {
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/scan", handleScan);
  server.on("/api/scan.bin", handleScanBin);
  server.on("/api/cmd", handleCmd);
  server.on("/api/settings", handleSettings);
  server.on("/api/plan", handlePlan);
  server.on("/api/unomode", handleUnoMode);
  server.begin();
}

void webServerHandleClient() {
  server.handleClient();
}
