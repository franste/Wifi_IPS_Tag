#ifndef WIFISTA_H
#define WIFISTA_H

#include <stdbool.h>
#include "esp_err.h"

void wifiStaInit();
bool wifiScanAP(uint8_t *bssid ,uint8_t channel);
//esp_err_t wifi_apsta(int timeout_ms);

#endif
