#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include "../planning/LaneTypes.h"

void loadSettings();
void saveSettings();

bool parseStrictFloat(const String& str, float& out);
bool parseStrictInt(const String& str, int& out);
bool parseDirectionArg(const String& str, CleaningDirection& out);

#endif // SETTINGS_H
