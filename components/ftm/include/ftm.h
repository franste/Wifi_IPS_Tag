#ifndef FTM_H
#define FTM_H

#define ETH_ALEN 6
#define MAX_CONNECT_RETRY_ATTEMPTS  5

typedef struct {
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_end *end;
} wifi_args_t;

typedef struct {
    struct arg_str *ssid;
    struct arg_end *end;
} wifi_scan_arg_t;

typedef struct {
    /* FTM Initiator */
    struct arg_lit *initiator;
    struct arg_int *frm_count;
    struct arg_int *burst_period;
    struct arg_str *ssid;
    /* FTM Responder */
    struct arg_lit *responder;
    struct arg_lit *enable;
    struct arg_lit *disable;
    struct arg_int *offset;
    struct arg_end *end;
} wifi_ftm_args_t;

static wifi_args_t sta_args;
static wifi_args_t ap_args;
static wifi_scan_arg_t scan_args;
static wifi_ftm_args_t ftm_args;



#endif