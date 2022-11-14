#include <stdio.h>
#include <storage.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"

// Initialize NVS
esp_err_t storageInit(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    return err;
}

esp_err_t saveSettings(const cJSON *object)
{
    char *settings_json_string = cJSON_Print(object);
    nvs_handle_t my_handle;
    esp_err_t err;

     // Open
    err = nvs_open("settings", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Write string to NVS
    err = nvs_set_str(my_handle, "settings", settings_json_string);
    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;

}

cJSON* readSettings(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

     // Open
    err = nvs_open("settings", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return NULL;

    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_str(my_handle, "settings", NULL, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return NULL;

    // Read previously saved str if available
    char buf[required_size];
    if (required_size > 0) {
        err = nvs_get_str(my_handle, "settings", (char *)&buf, &required_size);
        if (err != ESP_OK) return NULL;
        return cJSON_Parse(buf);
    } else {
        return NULL;
    }
}
