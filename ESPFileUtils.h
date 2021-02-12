#ifndef ESPFileUtils_h
#define ESPFileUtils_h

#include "LittleFS.h"

boolean mountFileSystem();
boolean fileExists(String path);
String readFile(String path);
void writeFile(String path, const String message);
boolean deleteFile(String path);

#endif
