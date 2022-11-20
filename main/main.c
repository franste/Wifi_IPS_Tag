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
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "spi_bmp390.h"
//#include "spi_bmp390_test.h"

//#define PRODUCTION true
#define SENSOR_BMP390 true
#define MAC_ADDRESS_LENGTH 6
#define MIN_FTM_RESULTS 1
#define FACTOR_us_TO_s 1000000

static cJSON *p_settings;
static bool run_once = false;
static TaskHandle_t main_task_handle = NULL;

static EventGroupHandle_t s_task_event_group;
static const int TASK_COMPLETED_BIT = BIT0;

static bool sensor_bmp390 = SENSOR_BMP390;

// Default settings for the device
static cJSON* useDefaultSettings(void) {
    cJSON *settings_json = cJSON_CreateObject();
    cJSON_AddStringToObject(settings_json, "server_url", CONFIG_DEV_SERVER_URL);
    cJSON_AddStringToObject(settings_json, "connect_to_ap_ssid", CONFIG_DEV_WIFI_SSID);
    cJSON_AddStringToObject(settings_json, "connect_to_ap_password", CONFIG_DEV_WIFI_PASSWORD);
    cJSON_AddNumberToObject(settings_json, "interval", 0); // 0 = send data as soon as possible (seconds)
    return settings_json;
}



void main_task(void *pvParameter)
{
    xEventGroupClearBits(s_task_event_group, TASK_COMPLETED_BIT);

    do {
        /* Scanning all channels*/
        scanResult_t scanResult = wifiScanAllChannels();
        if (scanResult.numOfScannedAP > 0) {
            result_t result = performFTM(scanResult); // Initiate FTM to all FTM responders in the scan result
            csi_result_list_t csi_result_list = get_csi_results();

            sensor_data_t sensor_data = {0};
            if (sensor_bmp390) {
                bmp390_temperature_and_pressure_shot_read(&sensor_data.temperature_c, &sensor_data.pressure_pa);
                sensor_data.valid = true;
            }

            char* results_json_str = result2JsonStr(result, csi_result_list, &sensor_data); // Convert the results to a JSON string
            
            // HTTP POST to server
            vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
            send_http_post(cJSON_GetObjectItemCaseSensitive(p_settings, "server_url")->valuestring, results_json_str);
            free(results_json_str);
            free(csi_result_list.list);

            /* Scanning only channels found in all scan*/
            while (result.numOfFtmResponders >=  MIN_FTM_RESULTS && !run_once) {
                vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
                int start = esp_timer_get_time();
                if (result.ftmResultsList != NULL) {
                    free(result.ftmResultsList);
                    result.ftmResultsList = NULL;
                    result.numOfResults = 0;
                    result.numOfFtmResponders = 0;
                }
                scanResult = wifiScanActiveChannels(scanResult);
                if (scanResult.numOfScannedAP > 0) {
                    //vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
                    result = performFTM(scanResult); // Initiate FTM to all FTM responders in the scan result
                    csi_result_list = get_csi_results();
                    
                    sensor_data_t sensor_data = {0};
                    if (sensor_bmp390) {
                        bmp390_temperature_and_pressure_shot_read(&sensor_data.temperature_c, &sensor_data.pressure_pa);
                        sensor_data.valid = true;
                    }

                    char* results_json_str = result2JsonStr(result, csi_result_list, &sensor_data); // Convert the results to a JSON string
                    
                    //HTTP POST to server
                    //vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
                    send_http_post(cJSON_GetObjectItemCaseSensitive(p_settings, "server_url")->valuestring, results_json_str);
                    free(results_json_str);
                    free(csi_result_list.list);
                    int end = esp_timer_get_time();
                    int heap = esp_get_free_heap_size();
                    ESP_LOGI("main", "Loop took %d ms heapsize: %d", (end - start) / 1000, heap);
                } else {
                    break;
                }
            }
            if (run_once) ESP_LOGI("main", "Task stopped");
            else ESP_LOGI("main", "No more FTM results, WIFI scan ALL channels now");

        } else {
            ESP_LOGI("main", "No APs found\n");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
        if (scanResult.scannedApList != NULL) {
            free(scanResult.scannedApList);
            scanResult.scannedApList = NULL;
            scanResult.numOfScannedAP = 0;
        }
        if (scanResult.uniqueChannels != NULL) {
            free(scanResult.uniqueChannels);
            scanResult.uniqueChannels = NULL;
            scanResult.uniqueChannelCount = 0;
        }
    } while (!run_once);
    xEventGroupSetBits(s_task_event_group, TASK_COMPLETED_BIT);
    vTaskDelete(main_task_handle);

} 

void app_main(void)
{
    // Initialize NVS.
    esp_err_t err = storageInit();
    if (err != ESP_OK) {
        printf("Failed to initialize NVS");
    }
    // load settings from NVS
    p_settings = readSettings();
    if (p_settings == NULL)
    {
        printf("No settings file found, using default settings\n");
        p_settings = useDefaultSettings();
    }
    p_settings = useDefaultSettings(); // Use default settings
    
    // Initialize & configure BMP390
    if (sensor_bmp390) {
        err = spi_bmp390_init();
        if (err == ESP_OK) {
            err = configure_BM390_for_temperature_and_pressure_shot();
            if (err != ESP_OK) {
                printf("Failed to configure BMP390 for temperature and pressure shot");
                sensor_bmp390 = false;
            }
        } else {
            ESP_LOGE("main", "Failed to initialize BMP390");
            sensor_bmp390 = false;
        }
    }

    // Initialize the Wifi
    ESP_ERROR_CHECK(wifiStaInit());

    // Initialize the CSI
    wifi_csi_init();

    // Set default Logging level. Can be changed via REPL
    esp_log_level_set("*", ESP_LOG_NONE);

    // Start the REPL
    repl(p_settings);

    // Join Ap
    joinAP(
        cJSON_GetObjectItemCaseSensitive(p_settings, "connect_to_ap_ssid")->valuestring, 
        cJSON_GetObjectItemCaseSensitive(p_settings, "connect_to_ap_password")->valuestring
    );

    // Create the task event group
    s_task_event_group = xEventGroupCreate();

    // Start the main task
    TaskHandle_t main_task_handle = NULL;
    cJSON *interval_json = cJSON_GetObjectItemCaseSensitive(p_settings, "interval");
    int interval = 0; // time in minutes
    if (cJSON_IsNumber(interval_json)) interval = cJSON_GetObjectItemCaseSensitive(p_settings, "interval")->valueint;
    
    if (interval > 0) run_once = true; // Run task only once
    else run_once = false; // Run task continuously

    xTaskCreatePinnedToCore(&main_task, "tag_task", 4096, NULL, 5, &main_task_handle, 1); // Run on core 1

    xEventGroupWaitBits(s_task_event_group, TASK_COMPLETED_BIT, pdTRUE, pdTRUE, portMAX_DELAY); // Wait for task to complete

    int since_boot_time = esp_timer_get_time();
    ESP_LOGI("MAIN", "Task completed, time since boot %d ms, going to sleep now!!!", since_boot_time / 1000);
    esp_sleep_enable_timer_wakeup(FACTOR_us_TO_s * 60 * interval - since_boot_time); // Sleep for interval - time since boot
    wifiStaDeInit();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,   ESP_PD_OPTION_AUTO);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_AUTO);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_AUTO);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,         ESP_PD_OPTION_AUTO);
    esp_sleep_enable_timer_wakeup(FACTOR_us_TO_s * 60 * interval - since_boot_time); // Sleep for interval - time since boot

    esp_deep_sleep_start();  // Go to sleep
}
 
