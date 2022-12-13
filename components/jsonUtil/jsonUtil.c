#include <stdio.h>
#include "stdlib.h"
#include "jsonUtil.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include <string.h>
#include <math.h>


#define MAC_ADDRESS_LENGTH 6


char* result2JsonStr(result_t results, csi_result_list_t csi_results, sensor_data_t* sensor_data)
{
    uint8_t mac[MAC_ADDRESS_LENGTH];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    char macStr[MAC_ADDRESS_LENGTH * 2 + 1];
    sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON *results_json = cJSON_CreateObject();
    cJSON_AddStringToObject(results_json, "device", macStr);

    if (sensor_data->valid) {
        cJSON_AddNumberToObject(results_json, "temp", roundf(sensor_data->temperature_c));
        cJSON_AddNumberToObject(results_json, "pa", roundf(sensor_data->pressure_pa));
    }
    //cJSON_AddStringToObject(results_json, "Timestamp", esp_log_system_timestamp());
    cJSON *result_array_json = cJSON_CreateArray();

    for (int i = 0; i < results.numOfResults; i++)
    {
        memcpy(mac, results.ftmResultsList[i].bssid, MAC_ADDRESS_LENGTH);
        sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        cJSON *result_json = cJSON_CreateObject();
        cJSON_AddStringToObject(result_json, "bssid", macStr);
        cJSON_AddNumberToObject(result_json, "rssi", results.ftmResultsList[i].rssi);
        if (results.ftmResultsList[i].avg_rtt_raw != 0) {
            cJSON *ftm_json = cJSON_CreateObject();
            cJSON_AddNumberToObject(ftm_json, "avg_RTT", results.ftmResultsList[i].avg_rtt_raw);
            cJSON_AddNumberToObject(ftm_json, "min_RTT", results.ftmResultsList[i].min_rtt_raw);
            cJSON_AddNumberToObject(ftm_json, "avg_RSSI", results.ftmResultsList[i].avg_RSSI);
            cJSON_AddNumberToObject(ftm_json, "rtt_est", results.ftmResultsList[i].rtt_est);
            cJSON_AddNumberToObject(ftm_json, "dist_est", results.ftmResultsList[i].dist_est);
            cJSON_AddItemToObject(result_json, "ftm", ftm_json);
        }
        cJSON_AddItemToArray(result_array_json, result_json);
        for (int j = 0; j < csi_results.len; j++) {
            if (memcmp(csi_results.list[j].bssid, mac, MAC_ADDRESS_LENGTH) == 0) {
                cJSON *csi_json = cJSON_CreateObject();
                cJSON_AddNumberToObject(csi_json, "noise_floor", csi_results.list[j].noise_floor);
                cJSON_AddNumberToObject(csi_json, "rssi", csi_results.list[j].rssi);
                cJSON_AddNumberToObject(csi_json, "amplitude_min", csi_results.list[j].amplitude_min);
                cJSON_AddNumberToObject(csi_json, "amplitude_max", csi_results.list[j].amplitude_max);
                cJSON_AddNumberToObject(csi_json, "amplitude_avg", csi_results.list[j].amplitude_avg);
                cJSON_AddNumberToObject(csi_json, "phase_min", csi_results.list[j].phase_min);
                cJSON_AddNumberToObject(csi_json, "phase_max", csi_results.list[j].phase_max);
                cJSON_AddNumberToObject(csi_json, "phase_avg", csi_results.list[j].phase_avg);
                cJSON_AddItemToObject(result_json, "csi", csi_json);
            }
        }
    }
    cJSON_AddItemToObject(results_json, "results", result_array_json);
    cJSON_AddStringToObject(results_json, "type", "DATA");
    char *results_json_str = cJSON_PrintUnformatted(results_json);
    cJSON_Delete(results_json);
    return results_json_str;
}
