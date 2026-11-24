#pragma once
#include "Arduino.h"
#define WIFI_AP_STA 1
#define WIFI_STA 2
struct IPStub {String toString(){return String("192.168.4.1");}};
class WifiStub {public: void persistent(bool){} void setSleep(bool){} void mode(int){} void disconnect(){} bool softAP(const char*,const char*,int,int,int){return true;} IPStub softAPIP(){return IPStub{};}};
inline WifiStub WiFi;
