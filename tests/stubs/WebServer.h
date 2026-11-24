#pragma once
#include "Arduino.h"
class WebServer {public:
 WebServer(int){} void on(const char*,int,std::function<void()>){ } void onNotFound(std::function<void()>){ }
 bool hasArg(const char*){return true;} String arg(const char*){return String("1");}
 void send(int,const char*,const String&){} void send_P(int,const char*,const char*){}
 void sendHeader(const char*,const char*){} void begin(){} void handleClient(){}
};
