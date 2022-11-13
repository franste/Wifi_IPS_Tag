#include <stdio.h>
#include "wifiSTA.h"
#include "esp_wifi.h"
#include "esp_wnm.h"
#include "esp_rrm.h"
#include "esp_mbo.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdbool.h>
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

#include <string.h>
#include <stdlib.h>
#include "rom/ets_sys.h"
#include "cJSON.h"
#include "esp_sntp.h"
#include "esp_http_client.h"
//#include "esp_tls.h"
#include "esp_timer.h"
#include "wifi_packets.h"
#include "math.h"

/* Length of mac adress*/
#define MAC_ADDRESS_LENGTH 6

/* Settings for Fine Time Measurement*/
#define FTM_FRM_COUNT 16
#define FTM_BURST_PERIOD 2
static EventGroupHandle_t s_ftm_event_group;
static const int FTM_REPORT_BIT = BIT0;
static const int FTM_FAILURE_BIT = BIT1;
static wifi_ftm_report_entry_t *s_ftm_report;
static uint8_t s_ftm_report_num_entries;
static uint32_t s_rtt_est, s_dist_est;

/* Settings for HTTP*/
#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 1024

/* Settings for Roaming*/
#define WLAN_EID_NEIGHBOR_REPORT 52
#define MAX_NEIGHBOR_LEN 512
#define WLAN_EID_MEASURE_REPORT 39
#define MEASURE_TYPE_LCI 9
#define MEASURE_TYPE_LOCATION_CIVIC 11
int rrm_ctx = 0;   // Context for RRM

/* Settings for connecting*/
static esp_netif_t *sta_netif = NULL;
static EventGroupHandle_t s_wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int DISCONNECTED_BIT = BIT1;
static const int GOT_IP_BIT = BIT2;

// Tags for logging
static const char *TAG_CSI = "csi_recv";
static const char *TAG_STA = "Wifi";
static const char *TAG_HTTP = "HTTP_POST";

static const wifi_scan_config_t scanALl_config = {
    .ssid = NULL,
    .bssid = NULL,
    .channel = 0,
    .show_hidden = true,
    .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    .scan_time.active.min = 50, // milliseconds
    .scan_time.active.max = 50,  // milliseconds
};

static wifi_config_t wifi_config = {
    .sta = {
        .ssid = CONFIG_DEV_WIFI_SSID,
        .password = CONFIG_DEV_WIFI_PASSWORD,
        .rm_enabled = 1,
        .pmf_cfg = {
            .capable = true,
            .required = false},
        .btm_enabled = 1,
        .mbo_enabled = 1,
        .ft_enabled = 1,
    },
};

//CSI
#define INITIAL_CSI_RESULT_LIST_SIZE 10
#define MAX_CSI_RESULT_LIST_SIZE 30
static int csi_result_list_size = INITIAL_CSI_RESULT_LIST_SIZE;
static SemaphoreHandle_t mutex = NULL; // mutex for csi
static csi_result_list_t csi_results; // csi result list 

static void eventHandler(void *arg, esp_event_base_t event_base,
                         int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;

        ESP_LOGI(TAG_STA, "Connected to %s (BSSID: " MACSTR ", Channel: %d)", event->ssid,
                 MAC2STR(event->bssid), event->channel);

        // Sync time using SNTP
        int64_t time = 0;
        time = esp_wifi_get_tsf_time(ESP_IF_WIFI_STA);
        if (time > 0) sntp_sync_time((struct timeval*) &time);
    
        xEventGroupClearBits(s_wifi_event_group, DISCONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGI(TAG_STA, "Disconnected from %s (BSSID: " MACSTR ", Reason: %d)", disconn->ssid,
                 MAC2STR(disconn->bssid), disconn->reason);
        if (disconn->reason == WIFI_REASON_ROAMING)
        {
            // Do not clear BITS

        } else {
            xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
            xEventGroupClearBits(s_wifi_event_group, GOT_IP_BIT);
            xEventGroupSetBits(s_wifi_event_group, DISCONNECTED_BIT);
            esp_wifi_connect();
            ESP_LOGI(TAG_STA, "retry to connect to the AP");
        }
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

static inline uint32_t WPA_GET_LE32(const uint8_t *a)
{
	return ((uint32_t) a[3] << 24) | (a[2] << 16) | (a[1] << 8) | a[0];
}

static char * get_btm_neighbor_list(uint8_t *report, size_t report_len)
{
	size_t len = 0;
	const uint8_t *data;
	int ret = 0;

	/*
	 * Neighbor Report element (IEEE P802.11-REVmc/D5.0)
	 * BSSID[6]
	 * BSSID Information[4]
	 * Operating Class[1]
	 * Channel Number[1]
	 * PHY Type[1]
	 * Optional Subelements[variable]
	 */
#define NR_IE_MIN_LEN (MAC_ADDRESS_LENGTH + 4 + 1 + 1 + 1)

	if (!report || report_len == 0) {
		ESP_LOGI(TAG_STA, "RRM neighbor report is not valid");
		return NULL;
	}

	char *buf = calloc(1, MAX_NEIGHBOR_LEN);
	data = report;

	while (report_len >= 2 + NR_IE_MIN_LEN) {
		const uint8_t *nr;
		char lci[256 * 2 + 1];
		char civic[256 * 2 + 1];
		uint8_t nr_len = data[1];
		const uint8_t *pos = data, *end;

		if (pos[0] != WLAN_EID_NEIGHBOR_REPORT ||
		    nr_len < NR_IE_MIN_LEN) {
			ESP_LOGI(TAG_STA, "CTRL: Invalid Neighbor Report element: id=%u len=%u",
					data[0], nr_len);
			ret = -1;
			goto cleanup;
		}

		if (2U + nr_len > report_len) {
			ESP_LOGI(TAG_STA, "CTRL: Invalid Neighbor Report element: id=%u len=%zu nr_len=%u",
					data[0], report_len, nr_len);
			ret = -1;
			goto cleanup;
		}
		pos += 2;
		end = pos + nr_len;

		nr = pos;
		pos += NR_IE_MIN_LEN;

		lci[0] = '\0';
		civic[0] = '\0';
		while (end - pos > 2) {
			uint8_t s_id, s_len;

			s_id = *pos++;
			s_len = *pos++;
			if (s_len > end - pos) {
				ret = -1;
				goto cleanup;
			}
			if (s_id == WLAN_EID_MEASURE_REPORT && s_len > 3) {
				/* Measurement Token[1] */
				/* Measurement Report Mode[1] */
				/* Measurement Type[1] */
				/* Measurement Report[variable] */
				switch (pos[2]) {
					case MEASURE_TYPE_LCI:
						if (lci[0])
							break;
						memcpy(lci, pos, s_len);
						break;
					case MEASURE_TYPE_LOCATION_CIVIC:
						if (civic[0])
							break;
						memcpy(civic, pos, s_len);
						break;
				}
			}

			pos += s_len;
		}

		ESP_LOGI(TAG_STA, "RMM neigbor report bssid=" MACSTR
				" info=0x%x op_class=%u chan=%u phy_type=%u%s%s%s%s",
				MAC2STR(nr), WPA_GET_LE32(nr + MAC_ADDRESS_LENGTH),
				nr[MAC_ADDRESS_LENGTH + 4], nr[MAC_ADDRESS_LENGTH + 5],
				nr[MAC_ADDRESS_LENGTH + 6],
				lci[0] ? " lci=" : "", lci,
				civic[0] ? " civic=" : "", civic);

		/* neighbor start */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, " neighbor=");
		/* bssid */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, MACSTR, MAC2STR(nr));
		/* , */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, ",");
		/* bssid info */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, "0x%04x", WPA_GET_LE32(nr + MAC_ADDRESS_LENGTH));
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, ",");
		/* operating class */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, "%u", nr[MAC_ADDRESS_LENGTH + 4]);
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, ",");
		/* channel number */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, "%u", nr[MAC_ADDRESS_LENGTH + 5]);
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, ",");
		/* phy type */
		len += snprintf(buf + len, MAX_NEIGHBOR_LEN - len, "%u", nr[MAC_ADDRESS_LENGTH + 6]);
		/* optional elements, skip */

		data = end;
		report_len -= 2 + nr_len;
	}

cleanup:
	if (ret < 0) {
		free(buf);
		buf = NULL;
	}
	return buf;
}

void neighbor_report_recv_cb(void *ctx, const uint8_t *report, size_t report_len)
{
	int *val = ctx;
	uint8_t *pos = (uint8_t *)report;
	int cand_list = 0;

	if (!report) {
		ESP_LOGE(TAG_STA, "report is null");
		return;
	}
	if (*val != rrm_ctx) {
		ESP_LOGE(TAG_STA, "rrm_ctx value didn't match, not initiated by us");
		return;
	}
	/* dump report info */
	ESP_LOGI(TAG_STA, "rrm: neighbor report len=%d", report_len);
	ESP_LOG_BUFFER_HEXDUMP(TAG_STA, pos, report_len, ESP_LOG_INFO);

	/* create neighbor list */
	char *neighbor_list = get_btm_neighbor_list(pos + 1, report_len - 1);

	/* In case neighbor list is not present issue a scan and get the list from that */
	if (!neighbor_list) return; // Not done any scan yet so there is no candidates in supplicant cache
    else {
		cand_list = 1;
	    /* send AP btm query, this will cause STA to roam as well */
	    esp_wnm_send_bss_transition_mgmt_query(REASON_FRAME_LOSS, neighbor_list, cand_list);
        free(neighbor_list);
    }
}

static void esp_bss_rssi_low_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	wifi_event_bss_rssi_low_t *event = event_data;

	ESP_LOGI(TAG_STA, "%s:bss rssi is=%d", __func__, event->rssi);
	/* Lets check channel conditions */
	rrm_ctx++;
	if (esp_rrm_send_neighbor_rep_request(neighbor_report_recv_cb, &rrm_ctx) < 0) {
		/* failed to send neighbor report request */
		ESP_LOGI(TAG_STA, "failed to send neighbor report request");
		if (esp_wnm_send_bss_transition_mgmt_query(REASON_FRAME_LOSS, NULL, 0) < 0) {
			ESP_LOGI(TAG_STA, "failed to send btm query");
		}
	}
}

void send_http_post(const char *url, char *payload) {
    //char output_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};   // Buffer to store response of http request
    //int content_length = 0;
    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 1000,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
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
        // content_length = esp_http_client_fetch_headers(client);
        // if (content_length < 0) {
        //     ESP_LOGE(TAG_HTTP, "HTTP client fetch headers failed");
        // } else {
        //     int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
        //     if (data_read >= 0) {
        //         ESP_LOGI(TAG_HTTP, "HTTP POST Status = %d, content_length = %lld",
        //         esp_http_client_get_status_code(client),
        //         esp_http_client_get_content_length(client));
        //         ESP_LOG_BUFFER_HEX(TAG_HTTP, output_buffer, strlen(output_buffer));
        //     } else {
        //         ESP_LOGE(TAG_HTTP, "Failed to read response");
        //     }
        // }
    }
    esp_http_client_cleanup(client);
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

    //esp_wifi_set_csi(true);

    for (int i = 0; i < scanResult.uniqueChannelCount; i++)
    {
        scan_config.channel = scanResult.uniqueChannels[i];
        // esp_err_t err = esp_wifi_set_channel(scan_config.channel, WIFI_SECOND_CHAN_NONE);
        // if (err != ESP_OK) {
        //     ESP_LOGE("TEST", "esp_wifi_set_channel failed: %s", esp_err_to_name(err));
        // }
        // ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT40));
        // //create_probe_request_packet();
        
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&numAP));

        if (scanResult.scannedApList == NULL)
        {
            int j = 0;
            do
            {
                vTaskDelay(10 / portTICK_PERIOD_MS);
                scanResult.scannedApList = malloc(numAP * sizeof(wifi_ap_record_t));
                j++;
            } while (scanResult.scannedApList == NULL && j < 5);
        }
        else
        {
            wifi_ap_record_t *temp = NULL;
            int j = 0;
            do
            {   
                vTaskDelay(10 / portTICK_PERIOD_MS);
                temp = realloc(
                    scanResult.scannedApList,
                    (scanResult.numOfScannedAP + numAP) * sizeof(wifi_ap_record_t));
                j++;
            } while (temp == NULL && j < 5);
            scanResult.scannedApList = temp;
        }
        if (!scanResult.scannedApList)
        {
            ESP_LOGE(TAG_STA, "Failed to allocate memory for scanned AP list");
            scanResult.numOfScannedAP = 0;
            scanResult.uniqueChannelCount = 0;
            free(scanResult.scannedApList);
            free(scanResult.uniqueChannels);
            scanResult.scannedApList = NULL;
            scanResult.uniqueChannels = NULL;
            //vTaskDelay(1000 / portTICK_PERIOD_MS);
            //ssesp_wifi_set_csi(false);
            return scanResult;
        }
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&numAP, (wifi_ap_record_t *)(scanResult.scannedApList + scanResult.numOfScannedAP)));
        scanResult.numOfScannedAP += numAP;        
    }
    //esp_wifi_set_csi(false);
    return scanResult;
}

scanResult_t wifiScanAllChannels()
{
    // int64_t start = esp_timer_get_time();
    wifi_scan_config_t scan_config = scanALl_config;
    scanResult_t scanResult;
    //esp_wifi_set_csi(true);
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
                    //esp_wifi_set_csi(false);
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
    esp_wifi_set_csi(false);
    return scanResult;
}

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

    //esp_wifi_set_channel(ftmi_cfg.channel, WIFI_SECOND_CHAN_ABOVE);
    esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT40);
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
        result.numOfFtmResponders = 0;
        return result;        
    }
    int ftm_responders_count = 0;
    for (int i = 0; i < scanResult.numOfScannedAP; i++)
    {
        ftmResult_t ftmResult;
        if (scanResult.scannedApList[i].ftm_responder == 1)
        {
            ftm_responders++;
            esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT40);
            ftmResult = startFtmSession(scanResult.scannedApList[i].bssid, scanResult.scannedApList[i].primary);
            if (ftmResult.avg_rtt_raw > 0) ftm_responders_count++;
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
    result.numOfFtmResponders = ftm_responders_count;

    // Send results to the server
    return result;
    
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    
    //xSemaphoreTake(mutex, portMAX_DELAY);
    uint8_t mac[MAC_ADDRESS_LENGTH];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    if (memcmp(info->mac, mac, MAC_ADDRESS_LENGTH) == 0) {
        ESP_LOGE(TAG_CSI, "CSI data received channel: %d same mac: "MACSTR" - "MACSTR"", info->rx_ctrl.channel, MAC2STR(info->mac), MAC2STR(mac));
        //Scource MAC is the same as the device MAC, Do Nothing.
        return;
    } else {
        ESP_LOGE(TAG_CSI, "CSI data received channel: %d different mac: "MACSTR" - "MACSTR" bandwith: %d", info->rx_ctrl.channel, MAC2STR(info->mac), MAC2STR(mac), info->rx_ctrl.cwb);
        csi_result_t csiResult;
        memcpy(csiResult.bssid, info->mac, MAC_ADDRESS_LENGTH);
        csiResult.noise_floor = info->rx_ctrl.noise_floor;
        csiResult.rssi = info->rx_ctrl.rssi;

        // amplitude
        int8_t *csi_ptr;
        csi_ptr = (int8_t *)info->buf;
        int amplitude_min = -1;
        int amplitude_max = -1;
        int amplitude_avg = 0;
        int phase_min = -1;
        int phase_max = -1;
        int phase_avg = 0;

        for (int i = 0; i < info->len / 2; i++) {
            int a = (int) sqrt(pow(csi_ptr[i * 2], 2) + pow(csi_ptr[(i * 2) + 1], 2));
            if (a > amplitude_max) amplitude_max = a;
            if (amplitude_min == -1 || a < amplitude_min) amplitude_min = a;
            amplitude_avg += a;
            
            int p = (int) atan2(csi_ptr[(i * 2) + 1], csi_ptr[i * 2]);
            if (p > phase_max) phase_max = a;
            if (phase_min == -1 || a < phase_min) phase_min = a;
            phase_avg += a;
        }
        amplitude_avg /= (info->len / 2);
        csiResult.amplitude_min = amplitude_min;
        csiResult.amplitude_max = amplitude_max;
        csiResult.amplitude_avg = amplitude_avg;
        phase_avg /= (info->len / 2);
        csiResult.phase_min = phase_min;
        csiResult.phase_max = phase_max;
        csiResult.phase_avg = phase_avg;

        xSemaphoreTake(mutex, portMAX_DELAY);
        if (csi_results.list == NULL) {
            csi_results.list = malloc(sizeof(csi_result_t) * csi_result_list_size);
            csi_results.list[0] = csiResult;
            csi_results.len = 1;
            csi_results.max_len = csi_result_list_size;
        } else if (csi_results.len < csi_results.max_len) {
            for (int i = 0; i < csi_results.len; i++) {
                if (memcmp(csi_results.list[i].bssid, csiResult.bssid, MAC_ADDRESS_LENGTH) == 0) {
                    // Already in the list, update the values
                    csi_results.list[i] = csiResult;
                    xSemaphoreGive(mutex);
                    return;
                }
            }
            csi_results.list[csi_results.len] = csiResult;
            csi_results.len++;
        } else if (csi_results.len == csi_results.max_len && csi_results.max_len < MAX_CSI_RESULT_LIST_SIZE) {;
            csi_results.list = realloc(csi_results.list, (csi_results.max_len + 1) * sizeof(csi_result_t));
            csi_results.list[csi_results.len] = csiResult;
            csi_results.len++;
            csi_results.max_len++;
            csi_result_list_size = csi_results.max_len;
        } else {
            ESP_LOGE(TAG_CSI, "CSI result list is full");
        }
        xSemaphoreGive(mutex);
        return;
    }

    // Check if the AP is already in the list
    




    // if (!info || !info->buf || !info->mac)
    // {
    //     ESP_LOGW(TAG_CSI, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
    //     return;
    // }

    // if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6))
    // {
    //     return;
    // }

    // static uint32_t s_count = 0;
    // const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;

    // if (!s_count)
    // {
    //     ESP_LOGI(TAG_CSI, "================ CSI RECV ================");
    //     ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,len,first_word,data\n");
    // }

    // ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    //            s_count++, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
    //            rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
    //            rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
    //            rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
    //            rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);

    // ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);

    // for (int i = 1; i < info->len; i++)
    // {
    //     ets_printf(",%d", info->buf[i]);
    // }

    // ets_printf("]\"\n");
}

csi_result_list_t get_csi_results() {
    csi_result_list_t results;
    xSemaphoreTake(mutex, portMAX_DELAY);
    results.len = csi_results.len;
    results.max_len = csi_results.max_len;
    results.list = malloc(sizeof(csi_result_t) * csi_results.max_len);
    memcpy(results.list, csi_results.list, sizeof(csi_result_t) * csi_results.max_len);
    
    // clear the list
    csi_results.len = 0;
    xSemaphoreGive(mutex);
    return results;
}
void wifi_csi_init()
{
    

    /**< default config */
    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = false,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = false,
    };

    wifi_promiscuous_filter_t filer_promi_ctrl = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_CTRL | WIFI_PROMIS_FILTER_MASK_DATA,
    };
    esp_wifi_set_promiscuous_filter(&filer_promi_ctrl);
    //esp_wifi_set_promiscuous_rx_cb(&wifi_csi_rx_cb);

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    mutex = xSemaphoreCreateMutex();
}

esp_err_t joinAP( char *ssid, char *password )
{
    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = CONFIG_DEV_WIFI_SSID,
            .password = CONFIG_DEV_WIFI_PASSWORD,
            .scan_method = WIFI_FAST_SCAN,
            .rm_enabled = 1,
            .pmf_cfg = {
                .capable = true,
                .required = false},
            .btm_enabled = 1,
            .mbo_enabled = 1,
            .ft_enabled = 1,
        },
    };
    strcpy((char *)wifi_config.sta.ssid, ssid);
    strcpy((char *)wifi_config.sta.password, password);
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_ABOVE));   // Dosent seem to be able to force bandwidth to 40mhz, so FTM is only 20mhz
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT40));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_MAC_WIFI_STA, &wifi_cfg));
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_STA, "Error connecting to AP: %s", esp_err_to_name(err));
        return ESP_FAIL;
    } else {
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            GOT_IP_BIT | DISCONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

        // If IP address is assigned
        if (bits & GOT_IP_BIT) {
            ESP_LOGI(TAG_STA, "Connected to AP");
            return ESP_OK;
        } else if (bits & DISCONNECTED_BIT) {
            ESP_LOGE(TAG_STA, "Failed to connect to AP");
            return ESP_FAIL;
        } else {
            ESP_LOGE(TAG_STA, "UNEXPECTED EVENT");
            return ESP_FAIL;
        }
    }
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

    cfg.nvs_enable = true;
    //cfg.nvs_enable = false;

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
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_BSS_RSSI_LOW,
				&esp_bss_rssi_low_handler, NULL));                                                    

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
