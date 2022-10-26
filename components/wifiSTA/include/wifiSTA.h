#ifndef WIFISTA_H
#define WIFISTA_H

#include <stdbool.h>
#include "esp_err.h"
#include "esp_wifi_types.h"

typedef struct {
    uint16_t numOfScannedAP;
    wifi_ap_record_t *scannedApList;
    uint8_t *uniqueChannels;
    uint16_t uniqueChannelCount;
} scanResult_t;


esp_err_t wifiStaInit();
esp_err_t wifiScanAllChannels();
esp_err_t wifiScanActiveChannels();
void wifi_csi_init();

//esp_err_t wifi_apsta(int timeout_ms);

#endif
