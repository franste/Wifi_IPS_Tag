#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
//#include "nvs_flash.h"
//#include "cmd_system.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_console.h"
#include "esp_mac.h"
#include "ftm.h"
#include "sdkconfig.h"

#define ETH_ALEN 6
#define MAX_CONNECT_RETRY_ATTEMPTS  5

//static bool s_reconnect = true;
//static int s_retry_num = 0;
static const char *TAG_STA = "ftm_station";
//static const char *TAG_AP = "ftm_ap";

static EventGroupHandle_t s_wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int DISCONNECTED_BIT = BIT1;

static EventGroupHandle_t s_ftm_event_group;
static const int FTM_REPORT_BIT = BIT0;
static const int FTM_FAILURE_BIT = BIT1;
static wifi_ftm_report_entry_t *s_ftm_report;
static uint8_t s_ftm_report_num_entries;
static uint32_t s_rtt_est, s_dist_est;
//static bool s_ap_started;
//static uint8_t s_ap_channel;
//static uint8_t s_ap_bssid[ETH_ALEN];

const int g_report_lvl =
#ifdef CONFIG_ESP_FTM_REPORT_SHOW_DIAG
    BIT0 |
#endif
#ifdef CONFIG_ESP_FTM_REPORT_SHOW_RTT
    BIT1 |
#endif
#ifdef CONFIG_ESP_FTM_REPORT_SHOW_T1T2T3T4
    BIT2 |
#endif
#ifdef CONFIG_ESP_FTM_REPORT_SHOW_RSSI
    BIT3 |
#endif
0;

static void ftmEventHandler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
	if (event_id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;

        ESP_LOGI(TAG_STA, "Connected to %s (BSSID: "MACSTR", Channel: %d)", event->ssid,
                 MAC2STR(event->bssid), event->channel);

        //memcpy(s_ap_bssid, event->bssid, ETH_ALEN);
        //s_ap_channel = event->channel;
        xEventGroupClearBits(s_wifi_event_group, DISCONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // if (s_reconnect && ++s_retry_num < MAX_CONNECT_RETRY_ATTEMPTS) {
        //     ESP_LOGI(TAG_STA, "sta disconnect, retry attempt %d...", s_retry_num);
        //     esp_wifi_connect();
        // } else {
        //     ESP_LOGI(TAG_STA, "ftmEventHndler - sta disconnected");
        // }
        ESP_LOGI(TAG_STA, "ftmEventHndler - sta disconnected");
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, DISCONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_FTM_REPORT) {
        wifi_event_ftm_report_t *event = (wifi_event_ftm_report_t *) event_data;

        if (event->status == FTM_STATUS_SUCCESS) {
            s_rtt_est = event->rtt_est;
            s_dist_est = event->dist_est;
            s_ftm_report = event->ftm_report_data;
            s_ftm_report_num_entries = event->ftm_report_num_entries;
            xEventGroupSetBits(s_ftm_event_group, FTM_REPORT_BIT);
        } else {
            ESP_LOGI(TAG_STA, "FTM procedure with Peer("MACSTR") failed! (Status - %d)",
                     MAC2STR(event->peer_mac), event->status);
            xEventGroupSetBits(s_ftm_event_group, FTM_FAILURE_BIT);
        }
    // } else if (event_id == WIFI_EVENT_AP_START) {
    //     s_ap_started = true;
    // } else if (event_id == WIFI_EVENT_AP_STOP) {
    //     s_ap_started = false;
     }
}

static void ftm_process_report(void)
{
    int i;
    char *log = NULL;

    if (!g_report_lvl)
        return;

    log = malloc(200);
    if (!log) {
        ESP_LOGE(TAG_STA, "Failed to alloc buffer for FTM report");
        return;
    }

    bzero(log, 200);
    sprintf(log, "%s%s%s%s", g_report_lvl & BIT0 ? " Diag |":"", g_report_lvl & BIT1 ? "   RTT   |":"",
                 g_report_lvl & BIT2 ? "       T1       |       T2       |       T3       |       T4       |":"",
                 g_report_lvl & BIT3 ? "  RSSI  |":"");
    ESP_LOGI(TAG_STA, "FTM Report:");
    ESP_LOGI(TAG_STA, "|%s", log);
    for (i = 0; i < s_ftm_report_num_entries; i++) {
        char *log_ptr = log;

        bzero(log, 200);
        if (g_report_lvl & BIT0) {
            log_ptr += sprintf(log_ptr, "%6d|", s_ftm_report[i].dlog_token);
        }
        if (g_report_lvl & BIT1) {
            log_ptr += sprintf(log_ptr, "%7lu  |", s_ftm_report[i].rtt);
        }
        if (g_report_lvl & BIT2) {
            log_ptr += sprintf(log_ptr, "%14llu  |%14llu  |%14llu  |%14llu  |", s_ftm_report[i].t1,
                                        s_ftm_report[i].t2, s_ftm_report[i].t3, s_ftm_report[i].t4);
        }
        if (g_report_lvl & BIT3) {
            log_ptr += sprintf(log_ptr, "%6d  |", s_ftm_report[i].rssi);
        }
        ESP_LOGI(TAG_STA, "|%s", log);
    }
    free(log);
}
void ftmTest(wifi_ap_record_t * ftmAP){
    printf("FTM Test %s \n", ftmAP->ssid);
    fflush(stdout);
}

int ftm(wifi_ap_record_t * ftmAP)
{   
    EventBits_t bits;
    wifi_ftm_initiator_cfg_t ftmi_cfg = {
        .frm_count = 32,
        .burst_period = 2,
    };
    //ESP_LOGE(TAG_STA, "========Before crash============");
    bits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, 0, 1, 0);
    //ESP_LOGE(TAG_STA, "========Before crash============");
    memcpy(ftmi_cfg.resp_mac, ftmAP->bssid, ETH_ALEN);
    //ESP_LOGE(TAG_STA, "==========Did it Crash ===========");
    ftmi_cfg.channel = ftmAP->primary;

    ESP_LOGI(TAG_STA, "Requesting FTM session with Frm Count - %d, Burst Period - %dmSec (0: No Preference)",
             ftmi_cfg.frm_count, ftmi_cfg.burst_period*100);

    if (ESP_OK != esp_wifi_ftm_initiate_session(&ftmi_cfg)) {
        ESP_LOGE(TAG_STA, "Failed to start FTM session");
        return 0;
    }

    bits = xEventGroupWaitBits(s_ftm_event_group, FTM_REPORT_BIT | FTM_FAILURE_BIT,
                                           pdTRUE, pdFALSE, portMAX_DELAY);
    /* Processing data from FTM session */
    if (bits & FTM_REPORT_BIT) {
        ftm_process_report();
        free(s_ftm_report);
        s_ftm_report = NULL;
        s_ftm_report_num_entries = 0;
        ESP_LOGI(TAG_STA, "Estimated RTT - %d nSec, Estimated Distance - %d.%02d meters",
                          s_rtt_est, s_dist_est / 100, s_dist_est % 100);
    } else {
        /* Failure case */
    }

    return 0;
}
esp_err_t ftmInit() {
    s_wifi_event_group = xEventGroupCreate();
    s_ftm_event_group = xEventGroupCreate();
    esp_event_handler_instance_t instance_any_id;
    return esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &ftmEventHandler, NULL,&instance_any_id);
}
