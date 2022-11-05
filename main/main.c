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


#define PRODUCTION true
#define MAC_ADDRESS_LENGTH 6

static const char *server_url = "http://192.168.1.3:8080/post/";
static const char *server_ap_ssid = "NETGEARREX";
static const char *server_ap_password = "Investor001";




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
    //syncTime();


    //ESP_ERROR_CHECK(ftmInit());

    //wifi_csi_init();
    while (true)
    {
        int start = esp_timer_get_time();
        scanResult_t scanResult = wifiScanAllChannels();
        if (scanResult.numOfScannedAP > 0) {
            result_t result = performFTM(scanResult); // Initiate FTM to all FTM responders in the scan result
            char* results_json_str = result2JsonStr(result); // Convert the results to a JSON string
            
            // Send the results to the server
            uint8_t bestRSSI = 0;
            int8_t rssi = -127;
            // Find the channel of AP with highest RSSI that has the same SSID as the server network.
            for (int i = 0; i < result.numOfResults; i++) {
                if (result.ftmResultsList[i].rssi > rssi && result.ftmResultsList[i].ssid[0] == *server_ap_ssid  ) {
                    rssi = result.ftmResultsList[i].rssi;
                    bestRSSI = i;
                }
            }

            // WIFI config for sending to server
            wifi_config_t send_config = {
                .sta = {
                    .bssid_set = true,
                    .channel = result.ftmResultsList[bestRSSI].channel,
                },
            };    
            memcpy(send_config.sta.ssid, result.ftmResultsList[bestRSSI].ssid, 32);
            memcpy(send_config.sta.password, server_ap_password, strlen(server_ap_password));
            memcpy(send_config.sta.bssid, result.ftmResultsList[bestRSSI].bssid, MAC_ADDRESS_LENGTH);
            free(result.ftmResultsList);

            
            sendToServer(send_config, server_url, results_json_str);
            free(results_json_str);
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