#ifndef JSONUTIL_H
#define JSONUTIL_H

#include "cJSON.h"
#include "esp_err.h"
#include "wifiSTA.h"

char* result2JsonStr(result_t result);

#endif