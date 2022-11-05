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

typedef struct {
    int avg_rtt_raw;   /**< Raw average Round-Trip-Time with peer in Nano-Seconds */
    int min_rtt_raw;   /**< Raw average Round-Trip-Time with peer in Nano-Seconds */
    int avg_RSSI;      /**< Average RSSI during FTM */
    int rtt_est;       /**< Estimated Round-Trip-Time with peer in Nano-Seconds */
    int dist_est;
    uint8_t channel;
    uint8_t bssid[6];                     /**< MAC address of AP */
    int8_t  rssi;                         /**< signal strength of AP */ 
    uint8_t ssid[32];                     /**< SSID of AP */
} ftmResult_t;

typedef struct {
    ftmResult_t *ftmResultsList;
    uint16_t numOfResults;
} result_t;


esp_err_t wifiStaInit();
scanResult_t wifiScanAllChannels();
scanResult_t wifiScanActiveChannels(scanResult_t scanResult);
result_t performFTM(scanResult_t scanResult);
void sendToServer(wifi_config_t send_config, const char *url, char *payload);
void wifi_csi_init();
void syncTime();


//esp_err_t wifi_apsta(int timeout_ms);

#endif
