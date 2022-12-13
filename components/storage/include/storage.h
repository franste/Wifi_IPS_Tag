#ifndef STORAGE_H
#define STORAGE_H

#include "cJSON.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

typedef struct control_s{
    cJSON *settings_ptr;
    SemaphoreHandle_t settings_mutex;
    bool reboot;
    TaskHandle_t main_task_handle;
    EventGroupHandle_t task_event_group;
    const int SETTINGS_NOT_UPDATING_BIT;
    const int TASK_COMPLETED_BIT;
    int interval;
} control_t;

esp_err_t storageInit(void);
esp_err_t saveSettings(const cJSON *object);
cJSON * readSettings(void);

#endif