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
#include "espNowMaster.h"

//#include "ftm.h"
//#include "espnow.h"
//#include "espnow_example.h"

// Default settings for the device
static cJSON* useDefaultSettings(void) {
    cJSON *settings_json = cJSON_CreateObject();
    cJSON_AddStringToObject(settings_json, "deviceName", "Tag");
    cJSON_AddStringToObject(settings_json, "Wifi_username", "admin");
    cJSON_AddStringToObject(settings_json, "Wifi_password", "password");
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
    cJSON *pSettings;
    pSettings = readSettings();
    if (pSettings == NULL)
    {
        printf("No settings file found, using default settings\n");
        pSettings = useDefaultSettings();
    }

    // Initialize the Wifi
    //wifiSettings(pSettings);
    ESP_ERROR_CHECK(wifiStaInit());

    // Initialize the ESP-NOW
    espNowInit();

    // Sync the time with network WIP
    syncTime();


    //ESP_ERROR_CHECK(ftmInit());

    //wifi_csi_init();
    while (true)
    {
        int start = esp_timer_get_time();
        scanResult_t scanResult = wifiScanAllChannels();
        if (scanResult.numOfScannedAP > 0) {
            result_t result = performFTM(scanResult); // Initiate FTM to all FTM responders in the scan result
            char* results_json_str = result2JsonStr(result); // Convert the results to a JSON string
            //send espnow to only FTM responders ak. Esp32S3
            uint8_t channel = 0;
            int8_t rssi = -127;
            for (int i = 0; i < result.numOfResults; i++) {
                // Find the channel of ESP-NOW peer with highest RSSI
                if ( result.ftmResultsList[i].min_rtt_raw > 1 ) {  // If the peer is a FTM responder
                    if (result.ftmResultsList[i].rssi > rssi) {
                        rssi = result.ftmResultsList[i].rssi;
                        channel = result.ftmResultsList[i].channel;
                    }
                }
            }
            // Broadcast to get closest ESP-NOW peer (If there is one)
            if (channel != 0) espNowSendData(channel, (uint8_t*)results_json_str, strlen(results_json_str));
            free(results_json_str);
            free(result.ftmResultsList);
            break;

        } else {
            ESP_LOGI("main", "No APs found\n");
        }
        scanResult.numOfScannedAP = 0;
        free(scanResult.scannedApList);
        free(scanResult.uniqueChannels);
        scanResult.scannedApList = NULL;
        scanResult.uniqueChannels = NULL;

        int end = esp_timer_get_time();
        int heap = esp_get_free_heap_size();
        ESP_LOGI("main", "Loop took %d ms heapsize: %d", (end - start) / 1000, heap);
    } 
        ESP_LOGI("main", "End of main");
    //wifiScanAP();
    //err = espNowInit();
    //if (err != ESP_OK) printf("Espnow failed to initialize");
    //espNowSendBroadcast();



    /*Repl startup menu*/
    //repl(pSettings);

    /* Load settings from anchor*/

    // Scan known channels

    // Ftm to anchors found on scan

    // Upload result to anchor

    // get settings version from anchor

    // Update settings if needed
}