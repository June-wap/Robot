#ifndef ARDUINO_LINK_H
#define ARDUINO_LINK_H

void arduinoLinkInit();
void setUnoFlashMode(bool en);
void pollLink();
void sendLink();
void arduinoLinkSendImmediateStop();
void arduinoLinkSendOtaStop();

#endif // ARDUINO_LINK_H
