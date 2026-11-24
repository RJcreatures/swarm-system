#pragma once
#include "esp_wifi.h"
#include <stdint.h>
#define ESP_ERR_ESPNOW_NO_MEM 123
struct esp_now_recv_info_t {const uint8_t*src_addr;};
struct esp_now_peer_info_t {uint8_t peer_addr[6];uint8_t channel;uint8_t ifidx;bool encrypt;};
inline esp_err_t esp_now_init(){return ESP_OK;}
inline esp_err_t esp_now_add_peer(const esp_now_peer_info_t*){return ESP_OK;}
inline bool esp_now_is_peer_exist(const uint8_t*){return false;}
inline esp_err_t esp_now_del_peer(const uint8_t*){return ESP_OK;}
inline esp_err_t esp_now_send(const uint8_t*,const uint8_t*,size_t){return ESP_OK;}
inline esp_err_t esp_now_register_recv_cb(void(*)(const esp_now_recv_info_t*,const uint8_t*,int)){return ESP_OK;}
