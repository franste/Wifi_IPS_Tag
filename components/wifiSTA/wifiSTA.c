#include <stdio.h>
#include "wifiSTA.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdbool.h>
#include "esp_timer.h"
#include "esp_mac.h"

#include <string.h>
#include <stdlib.h>
#include "rom/ets_sys.h"
#include "ftm.h"

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_recv";

static const char *TAG_STA = "Wifi_STA";
scanResult_t scanResult;

static const wifi_scan_config_t scanALl_config = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time.active.min = 10, // milliseconds
    .scan_time.active.max = 40  // milliseconds
};

esp_err_t wifiScanActiveChannels() {
    wifi_scan_config_t scan_config = scanALl_config;
    uint16_t numAP = 0;
    scanResult.numOfScannedAP = 0;
    if (scanResult.scannedApList != NULL) {
        free(scanResult.scannedApList);
        scanResult.scannedApList = NULL;
    }
    for (int i = 0; i < scanResult.uniqueChannelCount; i++) {
        scan_config.channel = scanResult.uniqueChannels[i];
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&numAP));

        if (scanResult.scannedApList == NULL) {
            //ESP_LOGI(TAG_STA, "Allocating memory malloc");
            scanResult.scannedApList = malloc(numAP * sizeof(wifi_ap_record_t));
        } else {
            //ESP_LOGI(TAG_STA, "Allocating memory realloc");
            scanResult.scannedApList = realloc(
                scanResult.scannedApList,
                (scanResult.numOfScannedAP + numAP) * sizeof(wifi_ap_record_t)
            );
        }
        if (!scanResult.scannedApList) {
            ESP_LOGE(TAG_STA, "Failed to allocate memory for scanned AP list");
            return ESP_ERR_NO_MEM;
        }
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&numAP, (wifi_ap_record_t *)(scanResult.scannedApList + scanResult.numOfScannedAP)));
        scanResult.numOfScannedAP += numAP;
        //ESP_LOGI(TAG_STA, "Found %d APs on channel %d\n", numAP, scan_config.channel);
    }
    int ftm_responders = 0;
    for (int i = 0; i < scanResult.numOfScannedAP; i++) {
        if (scanResult.scannedApList[i].ftm_responder == 1) {
            ftm_responders++;
            wifi_ap_record_t *pFtmAP;
            pFtmAP = (scanResult.scannedApList +i);
            ftmResult_t *pFtmResult;
            pFtmResult= ftm(pFtmAP);

            // Scan log for FTM Responders
            ESP_LOGI(TAG_STA, "SSID: %s, RSSI: %d, MAC: "MACSTR", Channel: %d, FTM: %d",
                scanResult.scannedApList[i].ssid,
                scanResult.scannedApList[i].rssi,
                MAC2STR(scanResult.scannedApList[i].bssid),
                scanResult.scannedApList[i].primary,
                scanResult.scannedApList[i].ftm_responder
            );

            ESP_LOGI(TAG_STA, "FTM avg_RSSI: %d, avg_rtt_raw: %d, min_rtt_raw: %d, rtt_est: %d, dist_est: %d cm ", 
                pFtmResult->avg_RSSI, pFtmResult->avg_rtt_raw, pFtmResult->min_rtt_raw, pFtmResult->rtt_est, pFtmResult->dist_est
            );

            vTaskDelay(10 / portTICK_PERIOD_MS);
            
            //free(pFtmAP);
            //free(pFtmResult);
        } 
    }      
    if (ftm_responders < 3) {
        ESP_LOGE(TAG_STA, "Not enough FTM responders found, found only %d responders", ftm_responders);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;

}


esp_err_t wifiScanAllChannels()
{
    //int64_t start = esp_timer_get_time();
    wifi_scan_config_t scan_config = scanALl_config;
    if (ESP_OK != esp_wifi_scan_start(&scan_config, true)) {
        ESP_LOGI(TAG_STA, "Failed to perform scan");
        return ESP_FAIL;
    }
    esp_wifi_scan_get_ap_num(&scanResult.numOfScannedAP);
    if (scanResult.numOfScannedAP == 0) {
        ESP_LOGI(TAG_STA, "No matching AP found");
        return ESP_FAIL;
    }

    if (scanResult.scannedApList != NULL) {
        free(scanResult.scannedApList);
    }
    if (scanResult.uniqueChannels != NULL) {
        free(scanResult.uniqueChannels);
    }

    scanResult.scannedApList = malloc(scanResult.numOfScannedAP * sizeof(wifi_ap_record_t));
    scanResult.uniqueChannels = malloc(scanResult.numOfScannedAP * sizeof(uint16_t));
    if (scanResult.scannedApList == NULL || scanResult.uniqueChannels == NULL) {
        ESP_LOGE(TAG_STA, "Failed to malloc buffer for scan results");
        return ESP_ERR_NO_MEM;
    }

    if (esp_wifi_scan_get_ap_records(
        &scanResult.numOfScannedAP, (wifi_ap_record_t *)scanResult.scannedApList)
        == ESP_OK) {
        int counter = 0;
        for (int i=0; i < scanResult.numOfScannedAP; i++) {
            // Store unique channels
            int j;
            for (j = 0; j < i; j++) {
                if (scanResult.scannedApList[i].primary == scanResult.scannedApList[j].primary)
                    break;
            }
            if (i == j) {
                scanResult.uniqueChannels[counter] = scanResult.scannedApList[i].primary;
                counter++;
            }
        }
        scanResult.uniqueChannelCount = counter;

        // Print log for how long time it took to scan all channels.
        // ESP_LOGI(
        //     TAG_STA, 
        //     "Scan All completed, found %d APs on %d channels, scantime %lld ms",
        //     scanResult.numOfScannedAP, scanResult.uniqueChannelCount, (esp_timer_get_time() - start) / 1000);
        
        return ESP_OK;
    }
    return ESP_FAIL;

}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || !info->mac) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6)) {
        return;
    }

    static uint32_t s_count = 0;
    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;

    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
            s_count++, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
            rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
            rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
            rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
            rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);

    ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);

    for (int i = 1; i < info->len; i++) {
        ets_printf(",%d", info->buf[i]);
    }

    ets_printf("]\"\n");
}

void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(g_wifi_radar_config->wifi_sniffer_cb));

    /**< default config */
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}


esp_err_t wifiStaInit(void)
{
    const char *country = "SE";
    esp_log_level_set("wifi", ESP_LOG_WARN);
	static bool initialized = false;
	if (initialized) {
		return ESP_OK;
	}
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_LR) );
    ESP_ERROR_CHECK( esp_wifi_set_country_code(country, true) );
    //ESP_ERROR_CHECK( esp_wifi_set_bandwidth(WIFI_MODE_STA, ) );
    ESP_ERROR_CHECK( esp_wifi_start());

//#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
//    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
//#endif
    initialized = true;
    return ESP_OK;
}

