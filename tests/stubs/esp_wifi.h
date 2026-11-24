#pragma once
#include "Arduino.h"
#define WIFI_IF_STA 0
#define WIFI_SECOND_CHAN_NONE 0
using esp_err_t=int;
#define ESP_OK 0
inline esp_err_t esp_wifi_set_channel(int,int){return ESP_OK;}
inline esp_err_t esp_wifi_get_mac(int,uint8_t*out){for(int i=0;i<6;i++)out[i]=i;return ESP_OK;}
