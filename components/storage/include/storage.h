#ifndef STORAGE_H
#define STORAGE_H

#include "cJSON.h"
#include "esp_err.h"

esp_err_t storageInit(void);
esp_err_t saveSettings(const cJSON *object);
cJSON * readSettings(void);

#endif