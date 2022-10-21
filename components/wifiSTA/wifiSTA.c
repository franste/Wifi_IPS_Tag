#include <stdio.h>
#include "wifiSTA.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdbool.h>
#include "esp_timer.h"

static const char *TAG_STA = "Wifi_STA";
uint16_t g_scan_ap_num;
wifi_ap_record_t *g_ap_list_buffer;

static const wifi_scan_config_t scanALl_config = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time.active.min = 10, // milliseconds
    .scan_time.active.max = 50  // milliseconds
};

bool wifiScanAP(uint8_t *bssid ,uint8_t channel)
{
    uint64_t start = esp_timer_get_time();

    wifi_scan_config_t scan_config = scanALl_config;
    if (bssid != NULL) scan_config.bssid = bssid;
    if (channel != 0) scan_config.channel = channel;    

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    if (ESP_OK != esp_wifi_scan_start(&scan_config, true)) {
        ESP_LOGI(TAG_STA, "Failed to perform scan");
        return NULL;
    }
    uint64_t end = esp_timer_get_time();
    esp_wifi_scan_get_ap_num(&g_scan_ap_num);
    if (g_scan_ap_num == 0) {
        ESP_LOGI(TAG_STA, "No matching AP found");
        return NULL;
    }

    if (g_ap_list_buffer) {
        free(g_ap_list_buffer);
    }
    g_ap_list_buffer = malloc(g_scan_ap_num * sizeof(wifi_ap_record_t));
    if (g_ap_list_buffer == NULL) {
        ESP_LOGE(TAG_STA, "Failed to malloc buffer to print scan results");
        return NULL;
    }
    if (esp_wifi_scan_get_ap_records(&g_scan_ap_num, (wifi_ap_record_t *)g_ap_list_buffer) == ESP_OK) {

        for (int i=0; i < g_scan_ap_num; i++) {
            printf("SSID: %s, RSSI: %d, Channel: %d, BSSID: %s, AuthMode: %d\n",
                g_ap_list_buffer[i].ssid,
                g_ap_list_buffer[i].rssi,
                g_ap_list_buffer[i].primary,
                MAC2STR(g_ap_list_buffer[i].bssid),
                g_ap_list_buffer[i].authmode);
        }
        printf("ftm_scan took: %llu milliseconds\n", (end - start)/1000);


        return true;
    }
    return NULL;

}


void wifiStaInit(void)
{
    esp_log_level_set("wifi", ESP_LOG_WARN);
	static bool initialized = false;
	if (initialized) {
		return;
	}
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
    
    initialized = true;
}

