#ifndef settings_H
#define settings_H

#include "esp_err.h"

enum DEFAULT_SETTINGS {
    DEFAULT_WIFI_SCAN_TIME_ACTIVE_MIN = 30,
    DEFAULT_WIFI_SCAN_TIME_ACTIVE_MAX = 40,     
};

typedef struct {
    int scan_time_active_min;
    int scan_time_active_max;   
} scan_settings_t;   

typedef struct {
    scan_settings_t scan;
} settings_t;

settings_t* getSettings();
esp_err_t saveSettings(settings_t *settings);

#endif