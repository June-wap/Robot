#ifndef ROBOT_WEB_SERVER_H
#define ROBOT_WEB_SERVER_H

#include <WebServer.h>

WebServer& getWebServer();
void webServerInit();
void webServerHandleClient();

#endif // ROBOT_WEB_SERVER_H
