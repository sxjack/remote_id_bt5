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
 * Notes
 *
 * Needs a matching board overlay.
 *
 * This file typifies why I don't like Zephyr.
 * This should have taken 10 minutes. It took hours.
 * Poor documentation. Device tree. Fragile, opaque macros.
 * You have to find a sample that works, figure out how it
 * works and then extract the bits that you need (or just 
 * copy and hope). 
 *
 */

#include "odid_bt.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>

#if defined(CONFIG_BOARD_XIAO_BLE)
#define VBATT_CHANNEL            2
#define VBATT_CHANNEL_X          channel_7
static const float               calib_factor = 1510.0 / 510.0;
static const struct gpio_dt_spec read_bat = GPIO_DT_SPEC_GET(DT_ALIAS(readbat),gpios);
#elif defined(CONFIG_BOARD_NRF52840DONGLE_NRF52840)
#define VBATT_CHANNEL            1
#define VBATT_CHANNEL_X          channel_1
static const float               calib_factor = 1.0;
#else
#define VBATT_CHANNEL            1
#define VBATT_CHANNEL_X          channel_1
static const float               calib_factor = 1.0;
#endif

static const struct device      *adc_dev   = NULL;
static struct adc_dt_spec        vbatt_dt  = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user),VBATT_CHANNEL);
// static struct adc_channel_cfg    vbatt_cfg = ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc),VBATT_CHANNEL_X));
static struct adc_sequence       vbatt_seq;
static uint16_t                  results[4];

/*
 *
 */

float battery_voltage() {

  int        status;
  char       text[96];
  float      voltage = 0.5;
  int32_t    mV;
  static int status1, status2;

  if (!adc_dev) {

    if (adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc))) {

      memset(results,0,sizeof(results));

      memset(&vbatt_seq,0,sizeof(vbatt_seq));
      vbatt_seq.channels     = 1 << vbatt_dt.channel_id;
      vbatt_seq.resolution   = vbatt_dt.resolution;
      vbatt_seq.oversampling = vbatt_seq.oversampling;
      vbatt_seq.buffer       = results;
      vbatt_seq.buffer_size  = sizeof(results);
    }
#if defined(CONFIG_BOARD_XIAO_BLE)
    gpio_pin_configure_dt(&read_bat,GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&read_bat,1);
#endif
  }

  if (adc_dev) {

    if (status1 = adc_channel_setup(adc_dev,&vbatt_dt.channel_cfg)) {

      sprintf(text,"%s(): adc_channel_setup() returned %d",
              __func__,status1);
      txt_message(text,1,1,1);
    }

    if ((status2 = adc_read(adc_dev,&vbatt_seq)) == 0) {

      mV = results[0];

      if ((status = adc_raw_to_millivolts_dt(&vbatt_dt,&mV)) == 0) {
        voltage = calib_factor * 0.001 * (float) mV;
      } else {
        sprintf(text,"%s(): adc_raw_to_millivolts_dt(%u) returned %d, %u",
                __func__,(unsigned int) results[0],status,mV);
        txt_message(text,1,1,1);
        voltage = (float) ((double) results[0] * calib_factor);
      }

    } else {

      sprintf(text,"%s(): adc_read(%d) returned %d, (%d)",
              __func__,vbatt_dt.channel_id,status2,status1);
      txt_message(text,1,1,1);
    }
  }

  return voltage;
}

/*
 *
 */

/*
 *
 */



