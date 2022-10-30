#include <stdio.h>
#include "stdlib.h"
#include "jsonUtil.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include <string.h>


#define MAC_ADDRESS_LENGTH 6


char* result2JsonStr(result_t results)
{
    uint8_t mac[MAC_ADDRESS_LENGTH];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    char macStr[MAC_ADDRESS_LENGTH * 2 + 1];
    sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON *results_json = cJSON_CreateObject();
    cJSON_AddStringToObject(results_json, "Device", macStr);
    cJSON_AddStringToObject(results_json, "Timestamp", esp_log_system_timestamp());
    cJSON *result_array_json = cJSON_CreateArray();

    for (int i = 0; i < results.numOfResults; i++)
    {
        memcpy(mac, results.ftmResultsList[i].bssid, MAC_ADDRESS_LENGTH);
        sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        cJSON *result_json = cJSON_CreateObject();
        cJSON_AddStringToObject(result_json, "BSSID", macStr);
        cJSON_AddNumberToObject(result_json, "RSSI", results.ftmResultsList[i].rssi);
        if (results.ftmResultsList[i].avg_rtt_raw != 0)
            cJSON_AddNumberToObject(result_json, "Avg_RTT", results.ftmResultsList[i].avg_rtt_raw);
        if (results.ftmResultsList[i].min_rtt_raw != 0)
            cJSON_AddNumberToObject(result_json, "Min_RTT", results.ftmResultsList[i].min_rtt_raw);
        if (results.ftmResultsList[i].avg_RSSI != 0)
            cJSON_AddNumberToObject(result_json, "Avg_RSSI", results.ftmResultsList[i].avg_RSSI);
        if (results.ftmResultsList[i].rtt_est != 0)
            cJSON_AddNumberToObject(result_json, "RTT_est", results.ftmResultsList[i].rtt_est);
        if (results.ftmResultsList[i].dist_est != 0)
            cJSON_AddNumberToObject(result_json, "Dist_est", results.ftmResultsList[i].dist_est);
        cJSON_AddItemToArray(result_array_json, result_json);
    }
    cJSON_AddItemToObject(results_json, "Results", result_array_json);
    char *results_json_str = cJSON_PrintUnformatted(results_json);
    cJSON_Delete(results_json);
    return results_json_str;
}
