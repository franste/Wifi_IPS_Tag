#include <stdio.h>
#include <string.h>
#include "repl.h"
#include "storage.h"
#include "freertos/FreeRTOS.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_console.h"
#include "cmd_system.h"
#include "esp_log.h"
#include "cJSON.h"
#include "argtable3/argtable3.h"
#include "esp_mac.h"


// Global variables
cJSON *settings_json;

typedef struct {
    struct arg_str *level;
    struct arg_end *end;
} log_level_arg_t;

typedef struct {
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_str *username;
    struct arg_end *end;
} ap_arg_t;

typedef struct {
    struct arg_str *url;
    struct arg_end *end;
} url_arg_t;

typedef struct {
    struct arg_str *name;
    struct arg_end *end;
} device_arg_t;

typedef struct {
    struct arg_int *interval;
    struct arg_end *end;
} interval_arg_t;

static log_level_arg_t log_level_args;
static ap_arg_t ap_args;
static url_arg_t url_args;
static device_arg_t device_args;
static interval_arg_t interval_args;

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

static int set_device_name(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &device_args);

    if (nerrors != 0) {
        arg_print_errors(stderr, device_args.end, argv[0]);
        return 1;
    }
    if (strcmp(device_args.name->sval[0], "") == 0) {
        cJSON *devicename = cJSON_GetObjectItemCaseSensitive(settings_json, "device_name");
        if (cJSON_IsString(devicename) && (devicename->valuestring != NULL)) {
            cJSON_DeleteItemFromObject(settings_json, "device_name");
        }   
    } else if ( device_args.name != NULL ) {
        cJSON *deviceName = cJSON_GetObjectItemCaseSensitive(settings_json, "device_name");
        if (cJSON_IsString(deviceName) && (deviceName->valuestring != NULL)) {
            cJSON_SetValuestring(deviceName, device_args.name->sval[0]);
        } else {
            cJSON_AddStringToObject(settings_json, "device_name", device_args.name->sval[0]);
        }
        esp_err_t err = saveSettings(settings_json);
        if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
        else printf("New settings saved\n");
    } else {
        printf("Failed on saving settings\n");
    }
    
    return 0;
}

static int set_log_level(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &log_level_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, log_level_args.end, argv[0]);
        return 1;
    }
    int level_int = 0;
    if (strcmp(log_level_args.level->sval[0], "none") == 0) {
        level_int = ESP_LOG_NONE;
    } else if (strcmp(log_level_args.level->sval[0], "error") == 0) {
        level_int = ESP_LOG_ERROR;
    } else if (strcmp(log_level_args.level->sval[0], "warn") == 0) {
        level_int = ESP_LOG_WARN;
    } else if (strcmp(log_level_args.level->sval[0], "info") == 0) {
        level_int = ESP_LOG_INFO;
    } else if (strcmp(log_level_args.level->sval[0], "debug") == 0) {
        level_int = ESP_LOG_DEBUG;
    } else if (strcmp(log_level_args.level->sval[0], "verbose") == 0) {
        level_int = ESP_LOG_VERBOSE;
    } else {
        printf("Invalid log level: %s", log_level_args.level->sval[0]);
        return 1;
    }
    esp_log_level_set("*", level_int);
    return 0;
   
}

static int set_AP(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &ap_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, ap_args.end, argv[0]);
        return 1;
    }
    if ( ap_args.ssid != NULL ) {
        cJSON *ssid = cJSON_GetObjectItemCaseSensitive(settings_json, "connect_to_ap_ssid");
        if (cJSON_IsString(ssid) && (ssid->valuestring != NULL)) {
            cJSON_SetValuestring(ssid, ap_args.ssid->sval[0]);
        } else {
            cJSON_AddStringToObject(settings_json, "connect_to_ap_ssid", ap_args.ssid->sval[0]);
        }
    }
    if ( ap_args.password != NULL ) {
        cJSON *password = cJSON_GetObjectItemCaseSensitive(settings_json, "connect_to_ap_password");
        if (cJSON_IsString(password) && (password->valuestring != NULL)) {
            cJSON_SetValuestring(password, ap_args.password->sval[0]);
        } else {
            cJSON_AddStringToObject(settings_json, "connect_to_ap_password", ap_args.password->sval[0]);
        }
    }
    if (strcmp(ap_args.username->sval[0], "") == 0) {
        cJSON *username = cJSON_GetObjectItemCaseSensitive(settings_json, "connect_to_ap_username");
        if (cJSON_IsString(username) && (username->valuestring != NULL)) {
            cJSON_DeleteItemFromObject(settings_json, "connect_to_ap_username");
        }   
    } else if ( ap_args.username != NULL ) {
        cJSON *username = cJSON_GetObjectItemCaseSensitive(settings_json, "connect_to_ap_username");
        if (cJSON_IsString(username) && (username->valuestring != NULL)) {
            cJSON_SetValuestring(username, ap_args.username->sval[0]);
        } else {
            cJSON_AddStringToObject(settings_json, "connect_to_ap_username", ap_args.username->sval[0]);
        }   
    } else {
        cJSON *username = cJSON_GetObjectItemCaseSensitive(settings_json, "connect_to_ap_username");
        if (cJSON_IsString(username) && (username->valuestring != NULL)) {
            cJSON_DeleteItemFromObject(settings_json, "connect_to_ap_username");
        }
    }
    esp_err_t err = saveSettings(settings_json);
    if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
    else printf("New settings saved\n");
    fflush(stdout);
    return 0;
}

static int set_url(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &url_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, url_args.end, argv[0]);
        return 1;
    }
    if ( url_args.url != NULL ) {
        cJSON *url = cJSON_GetObjectItemCaseSensitive(settings_json, "url");
        if (cJSON_IsString(url) && (url->valuestring != NULL)) {
            cJSON_SetValuestring(url, url_args.url->sval[0]);
        } else {
            cJSON_AddStringToObject(settings_json, "url", url_args.url->sval[0]);
        }
    }
    esp_err_t err = saveSettings(settings_json);
    if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
    else printf("New settings saved\n");
    fflush(stdout);
    return 0;
}

static int set_interval(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &interval_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, interval_args.end, argv[0]);
        return 1;
    }
    if ( interval_args.interval != NULL ) {
        cJSON *interval = cJSON_GetObjectItemCaseSensitive(settings_json, "interval");
        if (cJSON_IsNumber(interval)) {
            cJSON_SetNumberValue(interval, interval_args.interval->ival[0]);
        } else {
            cJSON_AddNumberToObject(settings_json, "interval", interval_args.interval->ival[0]);
        }
    }
    esp_err_t err = saveSettings(settings_json);
    if (err != ESP_OK) printf("Error (%s) reading data from NVS!\n", esp_err_to_name(err));
    else printf("New settings saved\n");
    fflush(stdout);
    return 0;
}

static void print_menu()
{
    printf("\n ============================================================\n");
    printf(" |                WIFI Indoor Position Tag                  |\n");
    printf(" |                                                          |\n");
    printf(" |  1. 'help' for detailed information on parameters        |\n");
    printf(" |  2. 'settings' to show current settings                  |\n");
    printf(" |  3. 'log <none|error|warning|info>' to set log level     |\n");
    printf(" |  4. 'ap <ssid> <password> <username>' ap to connect to   |\n");
    printf(" |  5. 'server <url>' a url to post json to                 |\n");
    printf(" |  6. 'interval <minutes>' set the loop time               |\n");
    printf(" |  7. 'device <name>' set the nickname of the device       |\n");
    printf(" |  8. 'menu' to show menu                                  |\n");
    printf(" ============================================================\n");
    fflush(stdout);

}

static int print_menu_helper(int argc, char **argv)
{
    print_menu();
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

    //Input to set log level
    log_level_args.level = arg_str0(NULL, NULL, "<level>", "Log level");
    log_level_args.end = arg_end(1);

    const esp_console_cmd_t log_level_cmd = {
        .command = "log",
        .help = "Set log level",
        .hint = NULL,
        .func = &set_log_level,
        .argtable = &log_level_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&log_level_cmd) );

    //Input to set AP
    ap_args.ssid = arg_str0(NULL, NULL, "<ssid>", "SSID of AP");
    ap_args.password = arg_str0(NULL, NULL, "<password>", "Password of AP");
    ap_args.username = arg_str0(NULL, NULL, "<username>", "Username of AP");
    ap_args.end = arg_end(1);

    const esp_console_cmd_t ap_cmd = {
        .command = "ap",
        .help = "Set AP",
        .hint = NULL,
        .func = &set_AP,
        .argtable = &ap_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&ap_cmd) );

    //Input to set url
    url_args.url = arg_str0(NULL, NULL, "<url>", "URL of server");
    url_args.end = arg_end(1);

    const esp_console_cmd_t url_cmd = {
        .command = "url",
        .help = "Set URL",
        .hint = NULL,
        .func = &set_url,
        .argtable = &url_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&url_cmd) );

    //Input to set interval
    interval_args.interval = arg_int0(NULL, NULL, "<interval>", "Interval in minutes");
    interval_args.end = arg_end(1);

    const esp_console_cmd_t interval_cmd = {
        .command = "interval",
        .help = "Set interval in minutes",
        .hint = NULL,
        .func = &set_interval,
        .argtable = &interval_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&interval_cmd) );

    // Input to set tag nickname
    device_args.name = arg_str0(NULL, NULL, "<tag_nickname>", "Name of tag");
    device_args.end = arg_end(1);

    const esp_console_cmd_t deviceName_cmd = {
        .command = "device",
        .help = "Set tag nickname",
        .hint = NULL,
        .func = &set_device_name,
        .argtable = &device_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&deviceName_cmd) );

    //Input to print menu
    const esp_console_cmd_t menu_cmd = {
        .command = "menu",
        .help = "Show menu",
        .hint = NULL,
        .func = &print_menu_helper,
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&menu_cmd) );

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
    print_menu();

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void repl(cJSON *pSettings)
{
    settings_json = pSettings;
    deviceInfo();
    startupMenu();
}
