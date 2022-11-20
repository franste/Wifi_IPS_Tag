#ifndef JSONUTIL_H
#define JSONUTIL_H

#include "cJSON.h"
#include "esp_err.h"
#include "wifiSTA.h"

typedef struct {
    float temperature_c;
    float pressure_pa;
    bool valid;
} sensor_data_t;

char* result2JsonStr(result_t result, csi_result_list_t csi_result_list, sensor_data_t* sensor_data);

#endif