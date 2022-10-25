#include <stdio.h>
#include "storage.h"
#include "repl.h"
#include "cJSON.h"
#include "esp_system.h"
#include "wifiSTA.h"
#include "ftm.h"
#include "espnow.h"
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
    wifiStaInit();
    ESP_ERROR_CHECK(ftmInit());

    //wifi_csi_init();
    err = wifiScanAllChannels();
    err = wifiScanActiveChannels();
    printf("wifiScanActiveChannelscode: %d\n", err);
    
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