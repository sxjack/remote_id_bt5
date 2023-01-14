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

#ifndef RID_ODID_BT_H
#define RID_ODID_BT_H

#include "remote_id.h"

#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#define PACK_MESSAGES 0 // TO DO.

//

class RID_open {

 public:

                           RID_open();
                           int                      begin(const struct gpio_dt_spec *,const char *,ODID_UAS_Data *,
                                                          struct bt_le_ext_adv_info *);
  int                      foreground(ODID_UAS_Data *);

  // These are public for debugging.
  int                      bt4_ad_phase = 0, bt5_ad_phase = 0;
  uint8_t                  msg_counter[16];
  uint32_t                 counter_4 = 0, counter_5 = 0;

 private:

  int                      update_message(int *,ODID_UAS_Data *UAS_data);

  int                      ext_records = 0;
#if PACK_MESSAGES
  uint8_t                  pack_buffer[256];
#endif
  uint8_t                 *odid_enc_bt4, *odid_seq_bt4, bt4_adv_buffer[36];
  uint32_t                 last_bt4_advert = 0, last_bt5_advert = 0;
  struct gpio_dt_spec     *status_led;
  struct bt_le_adv_param   bt4_adv_param, bt5_adv_param;
  struct bt_data           bt4_data[2], bt5_data[5];         
  struct bt_le_ext_adv    *bt5_advert;

  ODID_BasicID_encoded    basicID_enc[2];
  ODID_Location_encoded   location_enc;
  ODID_System_encoded     system_enc;
  ODID_OperatorID_encoded operatorID_enc;
  ODID_SelfID_encoded     selfID_enc;
  ODID_Auth_encoded       auth_enc[2];
};

#endif
