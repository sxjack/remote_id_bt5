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

#pragma GCC diagnostic warning "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wparentheses"
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include "odid_bt.h"

#define ENABLE_BT4      1
#define ENABLE_BT5      1

#define BT4_INT       190 // msecs, delays between messages.
#if ID_JAPAN              // Pack.
#define BT5_INT       470
#else                     // Individual messages.
#define BT5_INT       170
#endif

const uint8_t RID_open::odid_uuid_h = 0xff;
const uint8_t RID_open::odid_uuid_l = 0xfa;
const uint8_t RID_open::batt_uuid_h = 0x2b;
const uint8_t RID_open::batt_uuid_l = 0xf0;

/*
 *
 */

RID_open::RID_open() {

  return;
}

/*
 *
 */

int RID_open::begin(const struct gpio_dt_spec *led,const char *name,ODID_UAS_Data *UAS_data,
                    struct bt_le_ext_adv_info *ext_adv_info) {

  int            status, j;
  char           text[128];
  static uint8_t bt_data_flags, bt_uuid[8];

  //

  memset(pack_buffer,0,sizeof(pack_buffer));
  memset(batt_buffer,0,sizeof(batt_buffer));

  pack_buffer[0] = odid_uuid_l;
  pack_buffer[1] = odid_uuid_h;
  pack_buffer[2] = 0x0d;
  odid_seq_bt5   = &pack_buffer[3];
  pack_buffer[4] = (ODID_MESSAGETYPE_PACKED << 4) | 0x02;
  pack_buffer[5] =  ODID_MESSAGE_SIZE;
  pack_buffer[6] = 7;
  pack_encoded   = (ODID_Message_encoded *)    &pack_buffer[7];

  batt_buffer[0] = batt_uuid_l;
  batt_buffer[1] = batt_uuid_h;
  batt_buffer[2] = 0x0d;
  batt_buffer[3] = 0x02;

  // This order is for ID_JAPAN. Different orders would require a #define block.
  basicID_enc[0] = (ODID_BasicID_encoded *)    &pack_encoded[j = 0];
  basicID_enc[1] = (ODID_BasicID_encoded *)    &pack_encoded[++j];
  location_enc   = (ODID_Location_encoded *)   &pack_encoded[++j];
  auth_enc[0]    = (ODID_Auth_encoded *)       &pack_encoded[++j];
  system_enc     = (ODID_System_encoded *)     &pack_encoded[++j];
  operatorID_enc = (ODID_OperatorID_encoded *) &pack_encoded[++j];
  selfID_enc     = (ODID_SelfID_encoded *)     &pack_encoded[++j];
  auth_enc[1]    = (ODID_Auth_encoded *)       &pack_encoded[++j];

  encodeBasicIDMessage(basicID_enc[0],&UAS_data->BasicID[0]);
  encodeBasicIDMessage(basicID_enc[1],&UAS_data->BasicID[1]);
  encodeOperatorIDMessage(operatorID_enc,&UAS_data->OperatorID);
  encodeSelfIDMessage(selfID_enc,&UAS_data->SelfID);
  encodeAuthMessage(auth_enc[0],&UAS_data->Auth[0]);
#if 0
  encodeLocationMessage(location_enc,&UAS_data->Location);
  encodeSystemMessage(system_enc,&UAS_data->System);
#endif

  bt5_advert      = NULL;

  counter_4       =
  counter_5       = 0;
  bt4_ad_phase    =
  bt5_ad_phase    = 0;
  last_bt4_advert = k_uptime_get_32();
  last_bt5_advert = last_bt4_advert + 70;

  memset(msg_counter,   0,sizeof(msg_counter));
  
  memset(bt4_adv_buffer,0,sizeof(bt4_adv_buffer));
  memset(&bt4_adv_param,0,sizeof(bt4_adv_param));
  memset(&bt5_adv_param,0,sizeof(bt5_adv_param));
  memset(bt4_data,      0,sizeof(bt4_data));
  memset(bt5_data,      0,sizeof(bt5_data));
  memset(ext_adv_info,  0,sizeof(struct bt_le_ext_adv_info));

  bt4_adv_buffer[0] = odid_uuid_l;
  bt4_adv_buffer[1] = odid_uuid_h;
  bt4_adv_buffer[2] = 0x0d;

  odid_seq_bt4 = &bt4_adv_buffer[3];
  odid_enc_bt4 = &bt4_adv_buffer[4];

  encodeLocationMessage((ODID_Location_encoded *) odid_enc_bt4,&UAS_data->Location);

  //
  
  status_led = (struct gpio_dt_spec *) led;

  if (status_led) {

    gpio_pin_configure_dt(status_led,GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(status_led,0);
  }

  // BT
  
  if (status = bt_enable(NULL)) {

    sprintf(text,"RID_open::%s(): bt_enable() returned %d",__func__,status);
    txt_message(text);
    return -1;
  }

  // BT4

  bt4_adv_param.id           = BT_ID_DEFAULT;
  bt4_adv_param.sid          = 0;
  bt4_adv_param.options      = BT_LE_ADV_OPT_NONE;
#if STATIC_BLE_MAC
  bt4_adv_param.options     |= BT_LE_ADV_OPT_USE_IDENTITY;
#endif
  bt4_adv_param.interval_min = BT_GAP_ADV_FAST_INT_MIN_2;
  bt4_adv_param.interval_max = BT_GAP_ADV_FAST_INT_MAX_2;

  bt4_data[j = 0].type = BT_DATA_SVC_DATA16;
  bt4_data[j].data_len = 29;
  bt4_data[j].data     = bt4_adv_buffer;
  
  // BT5
  
  bt5_adv_param.id           = BT_ID_DEFAULT;
  bt5_adv_param.sid          = 1;
  bt5_adv_param.options      = BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CODED;
#if STATIC_BLE_MAC
  bt5_adv_param.options     |= BT_LE_ADV_OPT_USE_IDENTITY;
#endif
  // bt5_adv_param.options      = BT_LE_ADV_OPT_EXT_ADV;
  bt5_adv_param.interval_min = BT_GAP_ADV_FAST_INT_MIN_2;
  bt5_adv_param.interval_max = BT_GAP_ADV_FAST_INT_MAX_2;

  bt_data_flags  = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
  ext_records    = 0;

  bt_uuid[j = 0] = odid_uuid_l;
  bt_uuid[++j]   = odid_uuid_h;

  // The opendroneid app wants this first. Dronetag doesn't mind if it isn't.
  bt5_data[ext_records].type     = BT_DATA_SVC_DATA16;
#if ID_JAPAN
  bt5_data[ext_records].data_len = 7 + (4 * ODID_MESSAGE_SIZE);
  bt5_data[ext_records].data     = pack_buffer;
  pack_buffer[6]                 = 4;
#else
  bt5_data[ext_records].data_len = 4 + ODID_MESSAGE_SIZE;
  bt5_data[ext_records].data     = bt4_adv_buffer;
#endif
  bt5_data[++ext_records].type   = BT_DATA_FLAGS;
  bt5_data[ext_records].data_len = 1;
  bt5_data[ext_records].data     = &bt_data_flags;
#if BATT_VOLTS
  bt5_data[++ext_records].type   = BT_DATA_SVC_DATA16;
  bt5_data[ext_records].data_len = 6;
  bt5_data[ext_records].data     = batt_buffer;
#endif
#if 0
  bt5_data[++ext_records].type   = BT_DATA_UUID16_ALL;
  bt5_data[ext_records].data_len = j + 1;
  bt5_data[ext_records].data     = bt_uuid;
#endif
#if 0
  bt5_data[++ext_records].type   = BT_DATA_NAME_COMPLETE;
  bt5_data[ext_records].data_len = strlen(name);
  bt5_data[ext_records].data     = (const uint8_t *) name;
#endif

  ++ext_records;

#if ENABLE_BT4

  if (status = bt_le_adv_start(&bt4_adv_param,bt4_data,1,NULL,0)) {

    sprintf(text,"RID_open::%s(): bt_le_adv_start(BT4) returned %d",__func__,status);
    txt_message(text);
  }

#endif

#if ENABLE_BT5
  
  if (status = bt_le_ext_adv_create(&bt5_adv_param,NULL,&bt5_advert)) {

    sprintf(text,"RID_open::%s(): bt_le_ext_adv_create(BT5) returned %d",__func__,status);
    txt_message(text);
  }

  if (bt5_advert) {

    if (status = bt_le_ext_adv_set_data(bt5_advert,bt5_data,ext_records,NULL,0)) {

      sprintf(text,"RID_open::%s(): bt_le_ext_adv_set_data() returned %d",__func__,status);
      txt_message(text);

    } else {

      if (status = bt_le_ext_adv_start(bt5_advert,NULL)) {

        sprintf(text,"RID_open::%s(): bt_le_ext_adv_start() returned %d",__func__,status);
        txt_message(text);
      }  
    }

    bt_le_ext_adv_get_info(bt5_advert,ext_adv_info);
  }

#endif

  sprintf(text,"RID_open::%s(): returning",__func__);
  txt_message(text);

  return 0;
}

/*
 *
 */

int RID_open::foreground(ODID_UAS_Data *UAS_data) {

  int                  status;
  char                 text[128];
  uint32_t             msecs;

  msecs = k_uptime_get_32();

#if ENABLE_BT4

  if ((msecs - last_bt4_advert) > (BT4_INT - 1)) {

    last_bt4_advert = msecs;

    if (++counter_4 == 100) { // This is for a diagnostic message.
      ;
    }

    update_message(&bt4_ad_phase,UAS_data);

    if (status = bt_le_adv_update_data(bt4_data,1,NULL,0)) {

      sprintf(text,"RID_open::%s(): bt_le_adv_update_data() returned %d",__func__,status);
      txt_message(text);
    }    
  }

#endif

#if ENABLE_BT5

  if ((msecs - last_bt5_advert) > (BT5_INT - 1)) {

    last_bt5_advert = msecs;
    ++counter_5;

#if BATT_VOLTS
    batt_volt.f16  = (__fp16) battery_voltage();
    batt_buffer[4] = (uint8_t)  batt_volt.u16;
    batt_buffer[5] = (uint8_t) (batt_volt.u16 >> 8);  
#endif

#if ID_JAPAN
    ++*odid_seq_bt5;
    encodeLocationMessage(location_enc,&UAS_data->Location);
    encodeSystemMessage(system_enc,&UAS_data->System);
    auth_japan(UAS_data,pack_encoded,(uint8_t *) AUTH_IV);
#else
    update_message(&bt5_ad_phase,UAS_data);
#endif

    if (status = bt_le_ext_adv_set_data(bt5_advert,bt5_data,ext_records,NULL,0)) {

      sprintf(text,"RID_open::%s(): bt_le_ext_adv_set_data() returned %d",__func__,status);
      txt_message(text);
    }
  }

#endif
  
  return 0;
}

/*
 *
 */

int RID_open::update_message(int *_phase,ODID_UAS_Data *UAS_data) {

  int index = 0, phase;

  phase = *_phase;
  
  while (index == 0) {
  
    switch (phase++) {

    case  0:
    case  3:
      index = 1;
      encodeLocationMessage(location_enc,&UAS_data->Location);
      memcpy(odid_enc_bt4,location_enc,ODID_MESSAGE_SIZE);
      break;

    case  1:
    case  4:
      index = 2;
      encodeSystemMessage(system_enc,&UAS_data->System);
      memcpy(odid_enc_bt4,system_enc,ODID_MESSAGE_SIZE);
      break;

    case 2:
#if ID_JAPAN
      index = 6;
      memcpy(odid_enc_bt4,basicID_enc[1],ODID_MESSAGE_SIZE);
#else
      if (operatorID_enc->OperatorId[0]) {
        index = 3;
        memcpy(odid_enc_bt4,operatorID_enc,ODID_MESSAGE_SIZE);
      }
#endif
      break;

    case 5:
      index = 4;
      memcpy(odid_enc_bt4,basicID_enc[0],ODID_MESSAGE_SIZE);
      break;

    default:
      if (selfID_enc->Desc[0]) {
        index = 5;
        encodeSelfIDMessage(selfID_enc,&UAS_data->SelfID); // Because we sometimes use selfID 
        memcpy(odid_enc_bt4,selfID_enc,ODID_MESSAGE_SIZE); // for diagnostics.
      }
      phase = 0;
      break;
    }
  }

  *odid_seq_bt4 = ++msg_counter[index];
  
  return *_phase = phase;
}

/*
 *
 */

#if __ZEPHYR__


#endif

/*
 *
 */

