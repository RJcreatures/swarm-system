#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <cstdlib>
#include <string>
#include <functional>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#define IRAM_ATTR
#define OUTPUT 1
#define INPUT 0
#define LOW 0
#define HIGH 1
#define RISING 3
#define PROGMEM
#define HTTP_GET 0
#define HTTP_POST 1
#define constrain(amt,lo,hi) ((amt)<(lo)?(lo):((amt)>(hi)?(hi):(amt)))
class String {
public:
 std::string s;
 String():s(""){} String(const char*x):s(x?x:""){} String(const std::string &x):s(x){}
 String(int x):s(std::to_string(x)){} String(unsigned int x):s(std::to_string(x)){}
 String(long x):s(std::to_string(x)){} String(unsigned long x):s(std::to_string(x)){}
 String(long long x):s(std::to_string(x)){} String(unsigned long long x):s(std::to_string(x)){}
 String(float x, int n):s(std::to_string(x).substr(0, std::to_string(x).find('.')+n+1)){}
 const char*c_str()const{return s.c_str();} size_t length()const{return s.size();}
 bool isEmpty()const{return s.empty();} char operator[](size_t i)const{return s[i];}
 long toInt()const{return std::strtol(s.c_str(),nullptr,10);} void reserve(size_t){}
 String&operator+=(const String &x){s+=x.s;return *this;} String&operator+=(const char*x){s+=x;return *this;} String&operator+=(char x){s+=x;return *this;}
 friend String operator+(const String &a,const String &b){return String(a.s+b.s);}
 friend bool operator==(const String&a,const char*b){return a.s==b;}
 friend bool operator!=(const String&a,const char*b){return a.s!=b;}
};
inline uint32_t fakeMillis = 100;
inline uint32_t millis(){return fakeMillis;}
inline void delay(uint32_t ms){fakeMillis+=ms;}
inline void pinMode(uint8_t,int){} inline void digitalWrite(uint8_t,int){}
inline void attachInterruptArg(uint8_t,void(*)(void*),void*,int){}
inline void noInterrupts(){} inline void interrupts(){}
inline bool ledcAttach(uint8_t,uint32_t,uint8_t){return true;}
inline bool ledcWrite(uint8_t,uint32_t){return true;}
class SerialStub {public: void begin(int){} void println(const char*){} void printf(const char*, ...){} };
inline SerialStub Serial;
