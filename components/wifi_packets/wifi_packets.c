#include <stdio.h>
#include "wifi_packets.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include <string.h>

#define MAC_ADDRESS_LENGTH 6



void create_probe_request_packet()
{
    uint8_t source_mac[MAC_ADDRESS_LENGTH];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, source_mac);
     
    // uint8_t packet[] = {
    //     // Frame Control version=0 type=0(management)  subtype=100 (probe request)
    //     0x40, 0x00,
    //     0x00, 0x00,  // Duration
    //     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // Adress1 Destination address  broadcast 
    //     source_mac[0], source_mac[1], source_mac[2], source_mac[3], source_mac[4], source_mac[5], // Adress2 Source 
    //     0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  // Adress3 BSSID
    //     // sequence number 1 (This would be overwritten if 'en_sys_seq' is set to true)
    //     0x10, 0x00,

    //     // IEEE 802.11 Wireless Management
    //     // Tagged Paramneters
    //     // Tag: SSID parameter set: Wildcard SSID*/
    //     0x00,  // wlan.tag.number => 0
    //     0x00,  // wlan.tag.length

    //     // Tag: Supported Rates 5.5(B), 11(B), 1(B), 2(B), 6, 12, 24, 48, [Mbit/sec]*/
    //     0x01,  // wlan.tag.number => 1
    //     0x08,  // wlan.tag.length => 8
    //     0x8b,  // Supported Rates: 5.5(B) (0x8b)	(wlan.supported_rates)
    //     0x96,  // Supported Rates: 11(B) (0x96)		(wlan.supported_rates)
    //     0x82,  // Supported Rates: 1(B) (0x82)		(wlan.supported_rates)
    //     0x84,  // Supported Rates: 2(B) (0x84)		(wlan.supported_rates)
    //     0x0c,  // Supported Rates: 6 (0x0c)			(wlan.supported_rates)
    //     0x18,  // Supported Rates: 12 (018)			(wlan.supported_rates)
    //     0x30,  // Supported Rates: 24 (0x30)		(wlan.supported_rates)
    //     0x60,  // Supported Rates: 48 (0x60)		(wlan.supported_rates)

    //     // Tag: Extended Supported Rates 54, 9, 18, 36, [Mbit/sec]*/
    //     0x32,  // wlan.tag.number => 50
    //     0x04,  // wlan.tag.length => 4
    //     // Extended Supported Rates: 54 (0x6c)	(wlan.extended_supported_rates)
    //     0x6c,
    //     // Extended Supported Rates: 9 (0x12)	(wlan.extended_supported_rates
    //     0x12,
    //     // Extended Supported Rates: 18 (0x24)	(wlan.extended_supported_rates
    //     0x24,
    //     // Extended Supported Rates: 36 (0x48)	(wlan.extended_supported_rates
    //     0x48

    //     // Frame check sequence (CRC32) is added internaly.

    // };

//     uint8_t packet[128] = {
//   /* 00 */  0x40,                               /* Management Frame - Probe Request */
//   /* 01 */  0x00,                               /* Frame Control Field #2 - always ZERO */
//   /* 02 */  0x00, 0x00,                         /* Duration - always ZERO */
//   /* 04 */  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, /* Destination address */
//   /* 0a */  source_mac[0], source_mac[1], source_mac[2], source_mac[3], source_mac[4], source_mac[5], /* Source address */
//   /* 10 */  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, /* Broadcast */
//   /* 16 */  0x90, 0x00,                         /* Sequence control - no fragment */
//   /* 18 */  0x00, 0x0f,
//   /* 1a */  0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,
//   /* 28 */  0x01, 0x04,
//   /* 2a */  0x82,                               /* Basic rate - 1.0Mbit/s */
//   /* 2c */  0x84,                               /* Basic rate - 2.0Mbit/s */
//   /* 2d */  0x8b,                               /* Basic rate - 5.5Mbit/s */
//   /* 2e */  0x96                                /* Basic rate - 11Mbit/s  */
//     };

    uint8_t probe_req[] = {
		 /*IEEE 802.11 Probe Request*/
		 0x40, 0x00, //Frame Control version=0 type=0(management)  subtype=100 (probe request)
		 0x00, 0x00, //Duration
		 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //A1 Destination address  broadcast
		 source_mac[0], source_mac[1], source_mac[2], source_mac[3], source_mac[4], source_mac[5], //A2 Source address (STA Mac i.e. transmitter address)
		 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, //A3 BSSID
		 0x40, 0x9e, //sequence number (This would be overwritten if 'en_sys_seq' is set to true in fn call esp_wifi_80211_tx )

		 /*IEEE 802.11 Wireless Management*/
		 	 /*Tagged Paramneters*/
		 	 	 /*Tag: SSID parameter set: Wildcard SSID*/
		 	 	 0x00,	//wlan.tag.number => 0
				 0x00,	//wlan.tag.length

				 /*Tag: Supported Rates 5.5(B), 11(B), 1(B), 2(B), 6, 12, 24, 48, [Mbit/sec]*/
				 0x01,	//wlan.tag.number => 1
				 0x08,	//wlan.tag.length => 8
				 0x8b,	//Supported Rates: 5.5(B) (0x8b)	(wlan.supported_rates)
				 0x96,	//Supported Rates: 11(B) (0x96)		(wlan.supported_rates)
				 0x82,	//Supported Rates: 1(B) (0x82)		(wlan.supported_rates)
				 0x84,	//Supported Rates: 2(B) (0x84)		(wlan.supported_rates)
				 0x0c,	//Supported Rates: 6 (0x0c)			(wlan.supported_rates)
				 0x18,	//Supported Rates: 12 (018)			(wlan.supported_rates)
				 0x30,	//Supported Rates: 24 (0x30)		(wlan.supported_rates)
				 0x60,	//Supported Rates: 48 (0x60)		(wlan.supported_rates)

				 /*Tag: Extended Supported Rates 54, 9, 18, 36, [Mbit/sec]*/

				 0x32,	//wlan.tag.number => 50
				 0x04,	//wlan.tag.length => 4
				 0x6c,	//Extended Supported Rates: 54 (0x6c)	(wlan.extended_supported_rates)
				 0x12,	//Extended Supported Rates: 9 (0x12)	(wlan.extended_supported_rates
				 0x24,	//Extended Supported Rates: 18 (0x24)	(wlan.extended_supported_rates
				 0x48,	//Extended Supported Rates: 36 (0x48)	(wlan.extended_supported_rates

				 /*0x83, 0x2d, 0xbe, 0xce*/ //Frame check sequence: 0x1b8026b0 [unverified] (wlan.fcs)
    };

    esp_err_t err = esp_wifi_80211_tx(ESP_IF_WIFI_STA, probe_req, sizeof(probe_req), true);
    if (err != ESP_OK) {
        ESP_LOGE("PACKET", "esp_wifi_80211_tx failed: %s", esp_err_to_name(err));            
    }
}
