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
    cJSON_AddStringToObject(results_json, "Device", macStr);

    if (sensor_data->valid) {
        cJSON_AddNumberToObject(results_json, "Temp", roundf(sensor_data->temperature_c));
        cJSON_AddNumberToObject(results_json, "Pa", roundf(sensor_data->pressure_pa));
    }
    //cJSON_AddStringToObject(results_json, "Timestamp", esp_log_system_timestamp());
    cJSON *result_array_json = cJSON_CreateArray();

    for (int i = 0; i < results.numOfResults; i++)
    {
        memcpy(mac, results.ftmResultsList[i].bssid, MAC_ADDRESS_LENGTH);
        sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        cJSON *result_json = cJSON_CreateObject();
        cJSON_AddStringToObject(result_json, "BSSID", macStr);
        cJSON_AddNumberToObject(result_json, "RSSI", results.ftmResultsList[i].rssi);
        if (results.ftmResultsList[i].avg_rtt_raw != 0) {
            cJSON *ftm_json = cJSON_CreateObject();
            cJSON_AddNumberToObject(ftm_json, "Avg_RTT", results.ftmResultsList[i].avg_rtt_raw);
            cJSON_AddNumberToObject(ftm_json, "Min_RTT", results.ftmResultsList[i].min_rtt_raw);
            cJSON_AddNumberToObject(ftm_json, "Avg_RSSI", results.ftmResultsList[i].avg_RSSI);
            cJSON_AddNumberToObject(ftm_json, "RTT_est", results.ftmResultsList[i].rtt_est);
            cJSON_AddNumberToObject(ftm_json, "Dist_est", results.ftmResultsList[i].dist_est);
            cJSON_AddItemToObject(result_json, "Ftm", ftm_json);
        }
        cJSON_AddItemToArray(result_array_json, result_json);
        for (int j = 0; j < csi_results.len; j++) {
            if (memcmp(csi_results.list[j].bssid, mac, MAC_ADDRESS_LENGTH) == 0) {
                cJSON *csi_json = cJSON_CreateObject();
                cJSON_AddNumberToObject(csi_json, "Noise_floor", csi_results.list[j].noise_floor);
                cJSON_AddNumberToObject(csi_json, "RSSI", csi_results.list[j].rssi);
                cJSON_AddNumberToObject(csi_json, "Amplitude_min", csi_results.list[j].amplitude_min);
                cJSON_AddNumberToObject(csi_json, "Amplitude_max", csi_results.list[j].amplitude_max);
                cJSON_AddNumberToObject(csi_json, "Amplitude_avg", csi_results.list[j].amplitude_avg);
                cJSON_AddNumberToObject(csi_json, "Phase_min", csi_results.list[j].phase_min);
                cJSON_AddNumberToObject(csi_json, "Phase_max", csi_results.list[j].phase_max);
                cJSON_AddNumberToObject(csi_json, "Phase_avg", csi_results.list[j].phase_avg);
                cJSON_AddItemToObject(result_json, "CSI", csi_json);
            }
        }
    }
    cJSON_AddItemToObject(results_json, "Results", result_array_json);
    char *results_json_str = cJSON_PrintUnformatted(results_json);
    cJSON_Delete(results_json);
    return results_json_str;
}
