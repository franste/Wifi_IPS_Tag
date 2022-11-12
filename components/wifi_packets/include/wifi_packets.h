#ifndef WIFI_PACKETS_H
#define WIFI_PACKETS_H


typedef struct 
{
    uint8_t *buffer;
    uint8_t length;
} raw_packet_t;

void create_probe_request_packet();

#endif