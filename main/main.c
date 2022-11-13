#include <stdio.h>
#include "storage.h"
#include "repl.h"
#include "cJSON.h"
#include "esp_system.h"
#include "wifiSTA.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "jsonUtil.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"



#define PRODUCTION true
#define MAC_ADDRESS_LENGTH 6
#define MIN_FTM_RESULTS 0

// Default settings for the device
static cJSON* useDefaultSettings(void) {
    cJSON *settings_json = cJSON_CreateObject();
    cJSON_AddStringToObject(settings_json, "server_url", "http://192.168.1.3:8080/post/");
    cJSON_AddStringToObject(settings_json, "connect_to_ap_ssid", "NETGEARREX");
    cJSON_AddStringToObject(settings_json, "connect_to_ap_password", "Investor001");
    return settings_json;
}

void app_main(void)
{
    // Initialize NVS.
    esp_err_t err = storageInit();
    if (err != ESP_OK) {
        printf("Failed to initialize NVS");
    }
    // load settings from NVS
    cJSON *p_settings;
    p_settings = readSettings();
    if (p_settings == NULL)
    {
        printf("No settings file found, using default settings\n");
        p_settings = useDefaultSettings();
    }

    // Initialize the Wifi
    ESP_ERROR_CHECK(wifiStaInit());

    // Initialize the CSI
    wifi_csi_init();

    // Set default Logging level. Can be changed via REPL
    esp_log_level_set("*", ESP_LOG_NONE);

    // Start the REPL
    repl(p_settings);

    // Join Ap
    joinAP(CONFIG_DEV_WIFI_SSID, CONFIG_DEV_WIFI_PASSWORD);
    

    /* Scanning all channels*/
    while (true)
    {
        /* Scanning all channels*/
        scanResult_t scanResult = wifiScanAllChannels();
        if (scanResult.numOfScannedAP > 0) {
            result_t result = performFTM(scanResult); // Initiate FTM to all FTM responders in the scan result
            csi_result_list_t csi_result_list = get_csi_results();
            char* results_json_str = result2JsonStr(result, csi_result_list); // Convert the results to a JSON string
            
            // HTTP POST to server
            vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
            send_http_post(cJSON_GetObjectItemCaseSensitive(p_settings, "server_url")->valuestring, results_json_str);
            free(results_json_str);
            free(csi_result_list.list);

            /* Scanning only channels found in all scan*/
            while (result.numOfFtmResponders >=  MIN_FTM_RESULTS) {
                //vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
                int start = esp_timer_get_time();
                if (result.ftmResultsList != NULL) {
                    free(result.ftmResultsList);
                    result.ftmResultsList = NULL;
                    result.numOfResults = 0;
                    result.numOfFtmResponders = 0;
                }
                scanResult = wifiScanActiveChannels(scanResult);
                if (scanResult.numOfScannedAP > 0) {
                    //vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
                    result = performFTM(scanResult); // Initiate FTM to all FTM responders in the scan result
                    csi_result_list = get_csi_results();
                    char* results_json_str = result2JsonStr(result, csi_result_list); // Convert the results to a JSON string
                    
                    //HTTP POST to server
                    //vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
                    send_http_post(cJSON_GetObjectItemCaseSensitive(p_settings, "server_url")->valuestring, results_json_str);
                    free(results_json_str);
                    free(csi_result_list.list);
                    int end = esp_timer_get_time();
                    int heap = esp_get_free_heap_size();
                    ESP_LOGI("main", "Loop took %d ms heapsize: %d", (end - start) / 1000, heap);
                } else {
                    break;
                }
            }
            ESP_LOGI("main", "No more FTM results, WIFI scan ALL channels now");
        } else {
            ESP_LOGI("main", "No APs found\n");
        }
        if (scanResult.scannedApList != NULL) {
            free(scanResult.scannedApList);
            scanResult.scannedApList = NULL;
            scanResult.numOfScannedAP = 0;
        }
        if (scanResult.uniqueChannels != NULL) {
            free(scanResult.uniqueChannels);
            scanResult.uniqueChannels = NULL;
            scanResult.uniqueChannelCount = 0;
        }
    } 
//         ESP_LOGI("main", "End of main");



//     /*Repl startup menu*/
//     //repl(pSettings);

//     /* Load settings from anchor*/

//     // Scan known channels

//     // Ftm to anchors found on scan

//     // Upload result to anchor

//     // get settings version from anchor

//     // Update settings if needed
}