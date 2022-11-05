#include <stdio.h>
#include "wifiSTA.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdbool.h>
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include <string.h>
#include <stdlib.h>
#include "rom/ets_sys.h"
#include "cJSON.h"
#include "esp_sntp.h"
#include "esp_http_client.h"
#include "esp_tls.h"


#define MAC_ADDRESS_LENGTH 6
#define FTM_FRM_COUNT 16
#define FTM_BURST_PERIOD 2

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 1024

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};

// Tags for logging
static const char *TAG_CSI = "csi_recv";
static const char *TAG_STA = "Wifi";
static const char *TAG_HTTP = "HTTP_POST";

// static const char *TAG_FTM = "FTM";

static esp_netif_t *sta_netif = NULL;

static EventGroupHandle_t s_wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int DISCONNECTED_BIT = BIT1;
static const int GOT_IP_BIT = BIT2;

static EventGroupHandle_t s_ftm_event_group;
static const int FTM_REPORT_BIT = BIT0;
static const int FTM_FAILURE_BIT = BIT1;
static wifi_ftm_report_entry_t *s_ftm_report;
static uint8_t s_ftm_report_num_entries;
static uint32_t s_rtt_est, s_dist_est;


static const wifi_scan_config_t scanALl_config = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time.active.min = 10, // milliseconds
    .scan_time.active.max = 40  // milliseconds
};

static wifi_config_t wifi_config = {
    .sta = {
        .ssid = CONFIG_STA_WIFI_SSID,
        .password = CONFIG_STA_WIFI_PASSWORD,
        .rm_enabled = 1,
        .pmf_cfg = {
            .capable = true,
            .required = false},
        .btm_enabled = 1,
        .mbo_enabled = 1,
        .ft_enabled = 1,
    },
};

static ftmResult_t ftm_process_report(void)
{
    ftmResult_t ftmResult;
    int avg_rssi = 0;
    int avg_rtt_raw = 0;
    int min_rtt_raw = 0;
    for (int i = 0; i < s_ftm_report_num_entries; i++)
    {
        avg_rssi += s_ftm_report[i].rssi;
        avg_rtt_raw += s_ftm_report[i].rtt;
        if (min_rtt_raw > s_ftm_report[i].rtt || min_rtt_raw == 0)
            min_rtt_raw = s_ftm_report[i].rtt;
    }
    avg_rssi = avg_rssi / s_ftm_report_num_entries;
    avg_rtt_raw = avg_rtt_raw / s_ftm_report_num_entries;

    // Set ftmReport
    ftmResult.dist_est = s_dist_est;
    ftmResult.rtt_est = s_rtt_est;
    ftmResult.avg_RSSI = avg_rssi;
    ftmResult.avg_rtt_raw = avg_rtt_raw;
    ftmResult.min_rtt_raw = min_rtt_raw;
    return ftmResult;
}

static ftmResult_t startFtmSession(u_int8_t *bssid, u_int8_t channel)
{
    ftmResult_t ftmResult = {0};
    EventBits_t bits;
    wifi_ftm_initiator_cfg_t ftmi_cfg = {
        .frm_count = FTM_FRM_COUNT,
        .burst_period = FTM_BURST_PERIOD,
    };
    bits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, 0, 1, 0);
    memcpy(ftmi_cfg.resp_mac, bssid, MAC_ADDRESS_LENGTH);
    ftmi_cfg.channel = channel;

    ESP_LOGI(TAG_STA, "Requesting FTM session with Frm Count - %d, Burst Period - %dmSec (0: No Preference)",
             ftmi_cfg.frm_count, ftmi_cfg.burst_period * 100);

    if (ESP_OK != esp_wifi_ftm_initiate_session(&ftmi_cfg))
    {
        ESP_LOGE(TAG_STA, "Failed to start FTM session");
        return ftmResult;
    }

    bits = xEventGroupWaitBits(s_ftm_event_group, FTM_REPORT_BIT | FTM_FAILURE_BIT,
                               pdTRUE, pdFALSE, portMAX_DELAY);
    /* Processing data from FTM session */
    if (bits & FTM_REPORT_BIT)
    {
        ftmResult = ftm_process_report();
        free(s_ftm_report);
        s_ftm_report = NULL;
        s_ftm_report_num_entries = 0;
        return ftmResult;
    }
    else
    {
        /* Failure case */
    }
    return ftmResult;
}

static void eventHandler(void *arg, esp_event_base_t event_base,
                         int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;

        ESP_LOGI(TAG_STA, "Connected to %s (BSSID: " MACSTR ", Channel: %d)", event->ssid,
                 MAC2STR(event->bssid), event->channel);
    
        xEventGroupClearBits(s_wifi_event_group, DISCONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGI(TAG_STA, "Disconnected from %s (BSSID: " MACSTR ", Reason: %d)", disconn->ssid,
                 MAC2STR(disconn->bssid), disconn->reason);
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, GOT_IP_BIT);
        xEventGroupSetBits(s_wifi_event_group, DISCONNECTED_BIT);
    }
    else if (event_id == WIFI_EVENT_FTM_REPORT)
    {
        wifi_event_ftm_report_t *event = (wifi_event_ftm_report_t *)event_data;

        if (event->status == FTM_STATUS_SUCCESS)
        {
            s_rtt_est = event->rtt_est;
            s_dist_est = event->dist_est;
            s_ftm_report = event->ftm_report_data;
            s_ftm_report_num_entries = event->ftm_report_num_entries;
            xEventGroupSetBits(s_ftm_event_group, FTM_REPORT_BIT);
        }
        else
        {
            ESP_LOGI(TAG_STA, "FTM procedure with Peer(" MACSTR ") failed! (Status - %d)",
                     MAC2STR(event->peer_mac), event->status);
            xEventGroupSetBits(s_ftm_event_group, FTM_FAILURE_BIT);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, GOT_IP_BIT);
    }
}

static void send_http_post(const char *url, char *payload) {
    char output_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};   // Buffer to store response of http request
    int content_length = 0;
    esp_http_client_config_t config = {
        .url = url,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_err_t err = esp_http_client_open(client, strlen(payload));
    if (err != ESP_OK) {
        ESP_LOGE(TAG_HTTP, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    } else {
        int wlen = esp_http_client_write(client, payload, strlen(payload));
        if (wlen < 0) {
            ESP_LOGE(TAG_HTTP, "Write failed");
        }
        content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0) {
            ESP_LOGE(TAG_HTTP, "HTTP client fetch headers failed");
        } else {
            int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
            if (data_read >= 0) {
                ESP_LOGI(TAG_HTTP, "HTTP POST Status = %d, content_length = %lld",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
                ESP_LOG_BUFFER_HEX(TAG_HTTP, output_buffer, strlen(output_buffer));
            } else {
                ESP_LOGE(TAG_HTTP, "Failed to read response");
            }
        }
    }
    esp_http_client_cleanup(client);     
}

void sendToServer(wifi_config_t send_config, const char *url, char *payload)
{
    esp_wifi_set_config(ESP_IF_WIFI_STA, &send_config);
    //esp_wifi_connect();
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_STA, "esp_wifi_connect failed: %s", esp_err_to_name(err)); 
    } else {
        // Waiting for the connection to be established and IP address to be assigned
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            GOT_IP_BIT | DISCONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

        // If IP address is assigned
        if (bits & GOT_IP_BIT) {
            /* Send data to server */
            send_http_post(url, payload);
        }
    }
    esp_wifi_disconnect();
    xEventGroupWaitBits(s_wifi_event_group, DISCONNECTED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    //esp_wifi_disconnect();
    ESP_LOGI(TAG_STA, "GOT IP Disconnect");
}

scanResult_t wifiScanActiveChannels(scanResult_t scanResult)
{
    wifi_scan_config_t scan_config = scanALl_config;
    uint16_t numAP = 0;
    scanResult.numOfScannedAP = 0;
    if (scanResult.scannedApList != NULL)
    {
        free(scanResult.scannedApList);
        scanResult.scannedApList = NULL;
    }
    for (int i = 0; i < scanResult.uniqueChannelCount; i++)
    {
        scan_config.channel = scanResult.uniqueChannels[i];
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&numAP));
        int i = 0;
        if (scanResult.scannedApList == NULL)
        {
            do
            {
                scanResult.scannedApList = malloc(numAP * sizeof(wifi_ap_record_t));
                i++;
            } while (scanResult.scannedApList == NULL || i > 5);
        }
        else
        {
            wifi_ap_record_t *temp = NULL;
            i = 0;
            do
            {
                temp = realloc(
                    scanResult.scannedApList,
                    (scanResult.numOfScannedAP + numAP) * sizeof(wifi_ap_record_t));
                i++;
            } while (temp == NULL || i > 5);
            scanResult.scannedApList = temp;
        }
        if (!scanResult.scannedApList)
        {
            ESP_LOGE(TAG_STA, "Failed to allocate memory for scanned AP list");
            scanResult.numOfScannedAP = 0;
            free(scanResult.scannedApList);
            free(scanResult.uniqueChannels);
            scanResult.scannedApList = NULL;
            scanResult.uniqueChannels = NULL;
            return scanResult;
        }
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&numAP, (wifi_ap_record_t *)(scanResult.scannedApList + scanResult.numOfScannedAP)));
        scanResult.numOfScannedAP += numAP;
        
        // ESP_LOGI(TAG_STA, "Found %d APs on channel %d\n", numAP, scan_config.channel);
    }
    return scanResult;
}

result_t performFTM(scanResult_t scanResult) {
    int ftm_responders = 0;
    result_t result;
    result.numOfResults = 0;
    result.ftmResultsList = NULL;
    int i = 0;
    do
    {
        result.ftmResultsList = malloc(scanResult.numOfScannedAP * sizeof(ftmResult_t));
        i++;
    } while (result.ftmResultsList == NULL || i > 5);
    if (!result.ftmResultsList)
    {
        ESP_LOGE(TAG_STA, "Failed to allocate memory for FTM result list");
        free(result.ftmResultsList);
        result.ftmResultsList = NULL;
        result.numOfResults = 0;
        return result;        
    }
    for (int i = 0; i < scanResult.numOfScannedAP; i++)
    {
        ftmResult_t ftmResult;
        if (scanResult.scannedApList[i].ftm_responder == 1)
        {
            ftm_responders++;
            ftmResult = startFtmSession(scanResult.scannedApList[i].bssid, scanResult.scannedApList[i].primary);

            // ESP_LOGI(TAG_STA, "FTM avg_RSSI: %d, avg_rtt_raw: %d, min_rtt_raw: %d, rtt_est: %d, dist_est: %d cm ",
            //     pFtmResult->avg_RSSI, pFtmResult->avg_rtt_raw, pFtmResult->min_rtt_raw, pFtmResult->rtt_est, pFtmResult->dist_est
            //);
        }
        else
        {
            ftmResult.avg_RSSI = 0;
            ftmResult.avg_rtt_raw = 0;
            ftmResult.min_rtt_raw = 0;
            ftmResult.rtt_est = 0;
            ftmResult.dist_est = 0;
        }
        ftmResult.rssi = scanResult.scannedApList[i].rssi;
        ftmResult.channel = scanResult.scannedApList[i].primary;
        memcpy(ftmResult.bssid, scanResult.scannedApList[i].bssid, MAC_ADDRESS_LENGTH);
        memcpy(ftmResult.ssid, scanResult.scannedApList[i].ssid, 32);
        memcpy(result.ftmResultsList + i, &ftmResult, sizeof(ftmResult_t));
    }
    result.numOfResults = scanResult.numOfScannedAP;

    // Send results to the server
    return result;
    
}

scanResult_t wifiScanAllChannels()
{
    // int64_t start = esp_timer_get_time();
    wifi_scan_config_t scan_config = scanALl_config;
    scanResult_t scanResult;

    if (ESP_OK == esp_wifi_scan_start(&scan_config, true)) {
        esp_wifi_scan_get_ap_num(&scanResult.numOfScannedAP);
        if (scanResult.numOfScannedAP > 0) {
            scanResult.scannedApList = malloc(scanResult.numOfScannedAP * sizeof(wifi_ap_record_t));
            scanResult.uniqueChannels = malloc(scanResult.numOfScannedAP * sizeof(uint16_t));
            if (scanResult.scannedApList != NULL || scanResult.uniqueChannels != NULL) {
                if (esp_wifi_scan_get_ap_records(
                        &scanResult.numOfScannedAP,
                        (wifi_ap_record_t *)scanResult.scannedApList
                ) == ESP_OK) {
                    int counter = 0;
                    for (int i = 0; i < scanResult.numOfScannedAP; i++)
                    {
                        // Store unique channels
                        int j;
                        for (j = 0; j < i; j++)
                        {
                            if (scanResult.scannedApList[i].primary == scanResult.scannedApList[j].primary)
                                break;
                        }
                        if (i == j)
                        {
                            scanResult.uniqueChannels[counter] = scanResult.scannedApList[i].primary;
                            counter++;
                        }
                    }
                    scanResult.uniqueChannelCount = counter;
                    return scanResult;
                }
            }
        }
    }
    scanResult.numOfScannedAP = 0;
    free(scanResult.scannedApList);
    free(scanResult.uniqueChannels);
    scanResult.scannedApList = NULL;
    scanResult.uniqueChannels = NULL;
    return scanResult;
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || !info->mac)
    {
        ESP_LOGW(TAG_CSI, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6))
    {
        return;
    }

    static uint32_t s_count = 0;
    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;

    if (!s_count)
    {
        ESP_LOGI(TAG_CSI, "================ CSI RECV ================");
        ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               s_count++, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
               rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
               rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
               rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
               rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);

    ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);

    for (int i = 1; i < info->len; i++)
    {
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
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = true,
        .manu_scale = false,
        .shift = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void syncTime()
{
    int64_t time = 0;
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_STA, "esp_wifi_connect failed: %s", esp_err_to_name(err)); 
    } else {
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            CONNECTED_BIT | DISCONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

        if (bits & CONNECTED_BIT) {
            ESP_LOGI(TAG_STA, "Connected to AP and syncing time");
            time = esp_wifi_get_tsf_time(ESP_IF_WIFI_STA);
            sntp_sync_time((struct timeval*) &time);        
        }
    }
    if (time == 0) ESP_LOGE(TAG_STA, "Failed to get time");

    esp_wifi_disconnect();
    xEventGroupWaitBits(s_wifi_event_group, DISCONNECTED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG_STA, "Disconnected from AP");    
}

esp_err_t wifiStaInit(void)
{
    const char *country = "SE";
    esp_log_level_set("wifi", ESP_LOG_WARN);
    static bool initialized = false;
    if (initialized)
    {
        return ESP_OK;
    }
    ESP_ERROR_CHECK(esp_netif_init());

    s_wifi_event_group = xEventGroupCreate();
    s_ftm_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &eventHandler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &eventHandler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_country_code(country, true));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_start());
    initialized = true;
    return ESP_OK;
}
