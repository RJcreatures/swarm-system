#pragma once
#include <stddef.h>
using QueueHandle_t = void*;
inline QueueHandle_t xQueueCreate(int,size_t){return (QueueHandle_t)1;}
inline int xQueueSend(QueueHandle_t,const void*,int){return 1;}
inline int xQueueReceive(QueueHandle_t,void*,int){return 0;}
