#ifndef PLATFORM_H
#define PLATFORM_H

#include <esp_platform.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
//#include "esp_timer.h"
typedef struct RtosConnType RtosConnType;
typedef RtosConnType* ConnTypePtr;
typedef TimerHandle_t HttpdPlatTimerHandle;
//#define httpd_printf(fmt, ...) printf(fmt, ##__VA_ARGS__)
//#define httpd_printf(fmt, ...) ESP_LOGI(__func__,fmt, ##__VA_ARGS__)
#define httpd_printf(fmt, ...)
#define ICACHE_FLASH_ATTR
#endif
