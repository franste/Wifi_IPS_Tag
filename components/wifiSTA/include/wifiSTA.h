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
    uint16_t numOfFtmResponders;
} result_t;

typedef struct {
    signed int noise_floor;
    signed int rssi;
    uint8_t bssid[6];
    int amplitude_min;
    int amplitude_max;
    int amplitude_avg;
    int phase_min;
    int phase_max;
    int phase_avg;
} csi_result_t;

typedef struct {
    csi_result_t *list;
    uint16_t len;
    uint16_t max_len;
} csi_result_list_t;

esp_err_t wifiStaInit();
scanResult_t wifiScanAllChannels();
scanResult_t wifiScanActiveChannels(scanResult_t scanResult);
result_t performFTM(scanResult_t scanResult);
void send_http_post(const char *url, char *payload);
void wifi_csi_init();
csi_result_list_t get_csi_results();
esp_err_t joinAP( char *ssid, char *password );

#endif
