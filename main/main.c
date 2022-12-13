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
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "spi_bmp390.h"
#include "esp_event.h"


//#define PRODUCTION true
#define SENSOR_BMP390 true
#define RESET_SETTINGS true
#define MAC_ADDRESS_LENGTH 6
#define MIN_FTM_RESULTS 1
#define FACTOR_us_TO_s 1000000

#if CONFIG_PRODUCTION_MODE
    #define STA_DEFAULT_SSID            CONFIG_WIP_PRODUCTION_STA_SSID
    #ifdef CONFIG_WIP_PRODUCTION_STA_WPA_ENTERPRISE
        #define STA_DEFAULT_USERNAME    CONFIG_WIP_PRODUCTION_STA_USERNAME
    #endif
    #define STA_DEFAULT_PASS            CONFIG_WIP_PRODUCTION_STA_PASSWORD
    #define STA_DEFAULT_CHANNEL         CONFIG_WIP_PRODUCTION_STA_CHANNEL
    #define DEFAULT_SERVER_URL          CONFIG_WIP_PRODUCTION_STA_SERVER_URL

#elif CONFIG_DEVELOPMENT_MODE_FACTORY
    #define STA_DEFAULT_SSID            CONFIG_WIP_DEVELOPMENT_FACTORY_STA_SSID
    #ifdef CONFIG_WIP_DEVELOPMENT_FACTORY_STA_WPA_ENTERPRISE
        #define STA_DEFAULT_USERNAME    CONFIG_WIP_DEVELOPMENT_FACTORY_STA_USERNAME
    #endif
    #define STA_DEFAULT_PASS            CONFIG_WIP_DEVELOPMENT_FACTORY_STA_PASSWORD
    #define STA_DEFAULT_CHANNEL         CONFIG_WIP_DEVELOPMENT_FACTORY_STA_CHANNEL
    #define DEFAULT_SERVER_URL          CONFIG_WIP_DEVELOPMENT_FACTORY_STA_SERVER_URL

#elif CONFIG_DEVELOPMENT_MODE_LAB
    #define STA_DEFAULT_SSID            CONFIG_WIP_DEVELOPMENT_LAB_STA_SSID
    #ifdef CONFIG_WIP_DEVELOPMENT_LAB_STA_WPA_ENTERPRISE
        #define STA_DEFAULT_USERNAME    CONFIG_WIP_DEVELOPMENT_LAB_STA_USERNAME
    #endif
    #define STA_DEFAULT_PASS            CONFIG_WIP_DEVELOPMENT_LAB_STA_PASSWORD
    #define STA_DEFAULT_CHANNEL         CONFIG_WIP_DEVELOPMENT_LAB_STA_CHANNEL
    #define DEFAULT_SERVER_URL          CONFIG_WIP_DEVELOPMENT_LAB_STA_SERVER_URL
#endif    

static bool sensor_bmp390 = SENSOR_BMP390;

static control_t control = {
    .settings_ptr = NULL,
    .reboot = false,
    .main_task_handle = NULL,
    .task_event_group = NULL,
    .TASK_COMPLETED_BIT = BIT0,
    .SETTINGS_NOT_UPDATING_BIT = BIT1
};


// Default settings for the device
static cJSON* useDefaultSettings(void) {
    cJSON *settings_json = cJSON_CreateObject();
    cJSON_AddStringToObject(settings_json, "url", DEFAULT_SERVER_URL);
    cJSON_AddStringToObject(settings_json, "ssid", STA_DEFAULT_SSID);
    cJSON_AddStringToObject(settings_json, "password", STA_DEFAULT_PASS);
    #ifdef STA_DEFAULT_USERNAME
        cJSON_AddStringToObject(settings_json, "username", STA_DEFAULT_USERNAME);
    #endif
    cJSON_AddNumberToObject(settings_json, "interval", 1); // 0 = send data as soon as possible (minutes)
    cJSON_AddNumberToObject(settings_json, "log", ESP_LOG_INFO); // 0 = no logging, 1 = error, 2 = warning, 3 = info, 4 = debug
    cJSON_AddBoolToObject(settings_json, "repl", false); // Enable REPL
    return settings_json;
}

void main_task(void *pvParameter)
{

    do {
        int start = esp_timer_get_time();
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
            
            // Send the results to the server
            vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.
            send_to_server(results_json_str);
            free(results_json_str);
            free(csi_result_list.list);

            /* Scanning only channels found in all scan*/
            while (result.numOfFtmResponders >=  MIN_FTM_RESULTS && !control.reboot) {
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
                    send_to_server(results_json_str); // Send the results to the server
                    
                    // Free memory
                    free(results_json_str);
                    free(csi_result_list.list);

                    // Statistics for debugging
                    int end = esp_timer_get_time();
                    int heap = esp_get_free_heap_size();
                    ESP_LOGI("main", "Loop took %d ms heapsize: %d", (end - start) / 1000, heap);

                } else {
                    break;
                }
            }

            // Free memory from results
            if (result.ftmResultsList != NULL) {
                free(result.ftmResultsList);
                result.ftmResultsList = NULL;
                result.numOfResults = 0;
                result.numOfFtmResponders = 0;
            }

            if (control.reboot) ESP_LOGI("main", "Task finished");
            else ESP_LOGI("main", "No more FTM results, WIFI scan ALL channels now");

        } else {
            ESP_LOGI("main", "No APs found\n");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for subprocessing to finish, if it is running.

        // Clean up memory
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

        // Statistics for debugging
        int end = esp_timer_get_time();
        int heap = esp_get_free_heap_size();
        int min_heap = esp_get_minimum_free_heap_size();
        ESP_LOGI("main", "Loop took %d ms heapsize: %d min heap: %d", (end - start) / 1000, heap, min_heap);
        if ((end -start) / 1000 < 100 ) vTaskDelay(2000 / portTICK_PERIOD_MS); //  reduce if connecting to AP is locking wifi
    } while (!control.reboot);

    // Task completed
    xEventGroupWaitBits(control.task_event_group, control.SETTINGS_NOT_UPDATING_BIT, false, true, portMAX_DELAY);
    xEventGroupSetBits(control.task_event_group, control.TASK_COMPLETED_BIT);
    vTaskDelete(control.main_task_handle);

} 

void app_main(void)
{
    // Initialize NVS.
    esp_err_t err = storageInit();
    if (err != ESP_OK) {
        printf("Failed to initialize NVS");
    }

    // Initialize the control struct
    if (RESET_SETTINGS) { // reset for debugging
        ESP_LOGE("main", "Resetting settings");
        control.settings_ptr = useDefaultSettings();
    } else {
        control.settings_ptr = readSettings(); // load settings from NVS
    }
     // If no settings are found, uce default settings
    if (control.settings_ptr == NULL)  {
        ESP_LOGE("main", "No settings found, using default settings");
        control.settings_ptr = useDefaultSettings();
    }

    control.settings_mutex = xSemaphoreCreateMutex();
    control.task_event_group = xEventGroupCreate();
     

    control.interval = cJSON_GetObjectItemCaseSensitive(control.settings_ptr, "interval")->valueint;
    
    // set the interval before deep sleep. If 0, no deep sleep
    if (control.interval > 0) control.reboot = true; // Run task only once
    else control.reboot = false; // Run task continuously

    xEventGroupClearBits(control.task_event_group, control.TASK_COMPLETED_BIT);
    xEventGroupSetBits(control.task_event_group, control.SETTINGS_NOT_UPDATING_BIT);

    // Set log level
    esp_log_level_set("*", cJSON_GetObjectItem(control.settings_ptr, "log")->valueint);

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
    ESP_ERROR_CHECK(wifiStaInit(&control));

    // Initialize the CSI
    wifi_csi_init();


    //Start the REPL
    cJSON *repl_json = cJSON_GetObjectItemCaseSensitive(control.settings_ptr, "repl");
    if (cJSON_IsTrue(repl_json) == 1 ){
        ESP_LOGE("main", "Starting REPL");
        xEventGroupClearBits(control.task_event_group, control.SETTINGS_NOT_UPDATING_BIT);
        repl(&control);
    }
    // Wait for the settings to be updated in REPL
    xEventGroupWaitBits(control.task_event_group, control.SETTINGS_NOT_UPDATING_BIT, false, true, portMAX_DELAY);


    // Join Ap
    joinAP();

    // Start the main task
    xTaskCreatePinnedToCore(&main_task, "tag_task", 4096, NULL, 5, &control.main_task_handle, 1); // Run on core 1

    // Wait for task to complete
    xEventGroupWaitBits(control.task_event_group, control.TASK_COMPLETED_BIT | control.SETTINGS_NOT_UPDATING_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    
    // Go to deep sleep
    int since_boot_time = esp_timer_get_time();
    ESP_LOGI("MAIN", "Task completed, time since boot %d ms, going to sleep now!!!", since_boot_time / 1000);
    wifiStaDeInit();

    // Set deep sleep time. Interval is in minutes.
    if (control.interval > 0) {
        esp_sleep_enable_timer_wakeup(FACTOR_us_TO_s * 60 * control.interval - since_boot_time);
    } else {
        esp_sleep_enable_timer_wakeup(FACTOR_us_TO_s * 5); // 5 seconds
    }

    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,   ESP_PD_OPTION_AUTO);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_AUTO);
    //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_AUTO);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,         ESP_PD_OPTION_AUTO);

    // Debugging
    int heap = esp_get_free_heap_size();
    int min_heap = esp_get_minimum_free_heap_size();
    ESP_LOGI("MAIN", "Free heap: %d, min heap: %d", heap, min_heap);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    esp_deep_sleep_start();  // Go to sleep
}
 
