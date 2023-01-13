/* -*- tab-width: 2; mode: c; -*-
 *
 * A program for the nRF52840 to transmit ASTM F3411/ASD-STAN 4709-002/opendroneid 
 * remote identification signals over Bluetooth.
 * 
 * Copyright (c) 2023 Steve Jack
 *
 * Apache 2.0 licence
 *
 * Parts of this program are taken from samples which are
 * Copyright (c) Intel Corporation 
 * or
 * Copyright (c) Nordic Semiconductor
 *
 */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>

#include "opendroneid.h"

                        // 01234567890123456789
#define UAV_OPERATOR      "GBR-OP-ABCD12345678"      // "GBR-OP-ABCD12345678"
#define UAV_ID            "ZZZZH12345678"            // "ZZZZH12345678"
#define SELF_ID           "CAA UAS 7068"
#define UA_TYPE           ODID_UATYPE_AEROPLANE      // ODID_UATYPE_NONE

#define EU_CATEGORY       ODID_CATEGORY_EU_SPECIFIC  // ODID_CATEGORY_EU_UNDECLARED

#define DONGLE            0
#define GPS_PASSTHROUGH   1
#define TXT_BOTH_PORTS    0 // Sends error messages to both the serial and USB ports.
#define SPEKTRUM          0
#define REQ_SATS          5 // 8 for real use, 5 for indoor testing.

void      txt_message(const char *);   // Packages the message into a NMEA TXT.
void      txt_message(const char *,const int,const int,const int);
void      debug_message(const char *);

#ifdef __cplusplus
extern "C" {
#endif

  uint64_t  alt_unix_secs(int,int,int,int,int,int);

#ifdef __cplusplus
};
#endif

/*
 *
 */

