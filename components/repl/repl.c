#include <stdio.h>
#include "repl.h"
#include "storage.h"
#include "freertos/FreeRTOS.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_console.h"
#include "cmd_system.h"
#include "cJSON.h"
#include "argtable3/argtable3.h"
#include "esp_mac.h"


// Global variables
cJSON *settings_json;

typedef struct {
    struct arg_str *name;
    struct arg_end *end;
} deviceName_arg_t;

static deviceName_arg_t deviceName_args;


//Prints device information
void deviceInfo(void)
{
    printf("\nWifi Indoor Positioning Tag\nRunning on ");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("%s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%luMB %s flash\n", flash_size / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %ld bytes\n", esp_get_minimum_free_heap_size());
    
}

static int print_settings(int argc, char **argv)
{
    if (settings_json == NULL) {
        printf("No settings found\n");
    } else {
        printf("Settings:\n");
        printf("%s\n", cJSON_Print(settings_json));
    }
    fflush(stdout);
    return 0;
}

static int setDeviceName(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &deviceName_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, deviceName_args.end, argv[0]);
        return 1;
    }
    if ( deviceName_args.name != NULL ) {
        cJSON *deviceName = cJSON_GetObjectItemCaseSensitive(settings_json, "deviceName");
        if (cJSON_IsString(deviceName) && (deviceName->valuestring != NULL)) {
            cJSON_SetValuestring(deviceName, deviceName_args.name->sval[0]);
        } else {
            cJSON_AddStringToObject(settings_json, "deviceName", deviceName_args.name->sval[0]);
        }
        esp_err_t err = saveSettings(settings_json);
        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
        else printf("New settings saved\n");
    } else {
        printf("Failed on saving settings\n");
    }
    
    return 0;
}

static void register_settings()
{
    //Input to print settings
    const esp_console_cmd_t settings_cmd = {
        .command = "settings",
        .help = "Show settings",
        .hint = NULL,
        .func = &print_settings,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&settings_cmd) );

    // Input to set tag nickname
    deviceName_args.name = arg_str0(NULL, NULL, "<tag_nickname>", "Name of tag");
    deviceName_args.end = arg_end(1);

    const esp_console_cmd_t deviceName_cmd = {
        .command = "deviceName",
        .help = "Set tag nickname",
        .hint = NULL,
        .func = &setDeviceName,
        .argtable = &deviceName_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&deviceName_cmd) );

}

static void startupMenu()
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    repl_config.prompt = "tag>";
    // init console REPL environment
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
    /* Register commands */
    register_system();
    register_settings();

    printf("\n");
    fflush(stdout);
    printf("\n ============================================================\n");
    printf(" |                WIFI Indoor Position Tag                  |\n");
    printf(" |                                                          |\n");
    printf(" |  1. 'help' for detailed information on parameters        |\n");
    printf(" |  2. 'settings' to show current settings                  |\n");
    printf(" |  3. 'deviceName <name>' to set tag nickname              |\n");
    printf(" ============================================================\n");

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void repl(cJSON *pSettings)
{
    settings_json = pSettings;
    deviceInfo();
    startupMenu();
}
