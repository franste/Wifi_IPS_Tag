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

//#include "ftm.h"
//#include "espnow.h"
//#include "espnow_example.h"

#define PRODUCTION false
#define MAC_ADDRESS_LENGTH 6


// Default settings for the device
static cJSON* useDefaultSettings(void) {
    cJSON *settings_json = cJSON_CreateObject();
    cJSON_AddStringToObject(settings_json, "SSID", "factorySSID");
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
    //espNowInit();

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
            
            //send_result_param_t *send_param = malloc(sizeof(send_result_param_t) + strlen(results_json_str));
            //memcpy(send_param->payload, results_json_str, strlen(results_json_str));

            if (PRODUCTION) { 
                uint8_t bestRSSI = 0;
                int8_t rssi = -127;
                // Find the channel of ESP-NOW peer with highest RSSI
                for (int i = 0; i < result.numOfResults; i++) {
                    if (result.ftmResultsList[i].rssi > rssi) {
                        rssi = result.ftmResultsList[i].rssi;
                        bestRSSI = i;
                    }
                }
                // wifi_config_t send_config = {
                //     .sta = {
                //         .ssid = pSettings->SSID,
                //         .password = "password",
                //         .bssid_set = true,
                //         .bssid = {result.ftmResultsList[bestRSSI].bssid[0], result.ftmResultsList[bestRSSI].bssid[1], result.ftmResultsList[bestRSSI].bssid[2], result.ftmResultsList[bestRSSI].bssid[3], result.ftmResultsList[bestRSSI].bssid[4], result.ftmResultsList[bestRSSI].bssid[5]},
                //         .channel = result.ftmResultsList[bestRSSI].channel,
                //     },
                // };    
                //memcpy(send_param->bssid, result.ftmResultsList[bestRSSI].bssid, MAC_ADDRESS_LENGTH);
                //send_param->channel = result.ftmResultsList[bestRSSI].channel;
                //sendToServer(send_param);
            }
            // Broadcast to get closest ESP-NOW peer (If there is one)
            else {
                char mac_str[] = CONFIG_UPLOAD_TO_DEV_AP_MAC;
                uint8_t mac_addr[MAC_ADDRESS_LENGTH];
                sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac_addr[0], &mac_addr[1], &mac_addr[2], &mac_addr[3], &mac_addr[4], &mac_addr[5]);

                wifi_config_t send_config = {
                    .sta = {
                        .ssid = CONFIG_UPLOAD_TO_DEV_AP_SSID,
                        .password = CONFIG_UPLOAD_TO_DEV_AP_PASSWORD,
                        .bssid_set = true,
                        .bssid = {mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]},
                        .channel = CONFIG_UPLOAD_TO_DEV_AP_Channel
                    },
                };
                sendToServer(send_config, results_json_str);

                // char mac_str[] = CONFIG_UPLOAD_TO_DEV_AP_MAC;
                // uint8_t mac_addr[MAC_ADDRESS_LENGTH];
                // sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac_addr[0], &mac_addr[1], &mac_addr[2], &mac_addr[3], &mac_addr[4], &mac_addr[5]);
                // memcpy(send_param->bssid, mac_addr, MAC_ADDRESS_LENGTH);
                // send_param->ssid = CONFIG_UPLOAD_TO_DEV_AP_SSID;
                // send_param->channel = (uint8_t) CONFIG_UPLOAD_TO_DEV_AP_Channel;
                // send_param->password = CONFIG_UPLOAD_TO_DEV_AP_PASSWORD;
                // sendToServer(send_param);
            }
            //if (channel != 0) espNowSendData(channel, (uint8_t*)results_json_str, strlen(results_json_str));
            //printf(results_json_str);
            //free(results_json_str);
            //free(result.ftmResultsList);
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