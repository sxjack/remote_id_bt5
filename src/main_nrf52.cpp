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

#include "remote_id.h"

#include <zephyr/logging/log.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/random/rand32.h>

#include "gps2.h"
#include "odid_bt.h"
#if SPEKTRUM
#include "spektrum_i2c.h"
#endif

//

static const char                 uav_operator[] = UAV_OPERATOR,
                                  self_id[]      = SELF_ID,
                                 *program_name   = "Remote ID";
static char                       uav_id[25]     = UAV_ID;
static const ODID_uatype_t        ua_type        = UA_TYPE;
static const ODID_category_EU_t   category       = EU_CATEGORY;

//

static GPS2                       gps;
static RID_open                   squitter;
#if SPEKTRUM
static Spektrum_I2C               spektrum;
#endif

static ODID_UAS_Data              UAS_data;
const uint64_t                    odid_datum  = 1546300800;

static const struct device       *usb_ser_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart),
                                 *uart_dev    = DEVICE_DT_GET(DT_NODELABEL(uart0));
#if DONGLE // nRF52840 dongle
static const struct gpio_dt_spec  led_green   = GPIO_DT_SPEC_GET(DT_ALIAS(led0_green),gpios);
static const struct gpio_dt_spec  led_red     = GPIO_DT_SPEC_GET(DT_ALIAS(led1_red),gpios);
static const struct gpio_dt_spec  led_green1  = GPIO_DT_SPEC_GET(DT_ALIAS(led1_green),gpios);
static const struct gpio_dt_spec  led_blue    = GPIO_DT_SPEC_GET(DT_ALIAS(led1_blue),gpios);
#else
static const struct gpio_dt_spec  led_red     = GPIO_DT_SPEC_GET(DT_ALIAS(led0),gpios);
static const struct gpio_dt_spec  led_green   = GPIO_DT_SPEC_GET(DT_ALIAS(led1),gpios);
static const struct gpio_dt_spec  led_blue    = GPIO_DT_SPEC_GET(DT_ALIAS(led2),gpios);
#endif

#if 0
static int                        time_set = 0;
#endif
static int                        base_set = 0, build_report = 0;

static void init_odid(const char *,char *,const char *);

/*
 *
 */

int main(void) {

  int                       status, toggle_period, heading;
  char                      text[128];
  float                     speed, height;
  uint32_t                  uptime, last_toggle = 0;
  uint64_t                  unix_secs;
  struct bt_le_ext_adv_info ext_adv_info;

  //

  init_odid(uav_operator,uav_id,self_id);

  //
  
  gpio_pin_configure_dt(&led_green,GPIO_OUTPUT_ACTIVE);
  gpio_pin_set_dt(&led_green,0);
  
  if (status = usb_enable(NULL)) {

    sprintf(text,"%s(): usb_enable() returned %d",__func__,status);
    printk("%s\n",text);
  }

  gps.begin(uart_dev,usb_ser_dev,&led_red);

  squitter.begin(&led_blue,uav_operator,&UAS_data,&ext_adv_info);

#if SPEKTRUM
  spektrum.begin();
#endif

#if ID_JAPAN
  crypto_init((uint8_t *) AUTH_KEY);
#endif

  txt_message(program_name);
  
  // Main loop.
  
  while (1) {

    gps.foreground();

#if 0
    if ((!time_set)&&(gps.fix)) {

      // To do.
      
      txt_message("system date & time set")
      time_set = 1;
    }

    if (gps.utc.tm_year > 100) {
      unix_secs = mktime(&gps.utc);
#else
    if (gps.utc.tm_year > 100) {
      unix_secs = alt_unix_secs(gps.utc.tm_year + 1900,gps.utc.tm_mon + 1,
                                gps.utc.tm_mday,gps.utc.tm_hour,gps.utc.tm_min,gps.utc.tm_sec);
#endif
      UAS_data.Auth[0].Timestamp  =
      UAS_data.System.Timestamp   = (unix_secs > odid_datum)? (unix_secs - odid_datum): 0;
    }

    UAS_data.Location.TimeStamp = (gps.utc.tm_min * 60) + gps.utc.tm_sec;

    if (gps.satellites >= REQ_SATS) {

      UAS_data.Location.Status      = ODID_STATUS_UNDECLARED;
      UAS_data.Location.Latitude    = gps.latitude_d;
      UAS_data.Location.Longitude   = gps.longitude_d;
      UAS_data.Location.AltitudeGeo = gps.altitude_msl_m;

      if (base_set) {
        height = UAS_data.Location.AltitudeGeo - UAS_data.System.OperatorAltitudeGeo;
        UAS_data.Location.Height = (height > MIN_ALT)? height: INV_ALT;
      }

      heading = gps.heading;
      UAS_data.Location.Direction = (float) ((heading >= MIN_DIR)&&(heading <= MAX_DIR)) ? heading: INV_DIR;

      // speed =  0.51444 * gps.speed_kn;
      speed =  0.01    * gps.speed_2d_cm;
      UAS_data.Location.SpeedHorizontal = ((speed >= 0.0)&&(speed <= MAX_SPEED_H)) ? speed: INV_SPEED_H;
      speed = -0.01 * (float) gps.vel_D_cm;
      UAS_data.Location.SpeedVertical = ((speed >= MIN_SPEED_V)&&(speed <= MAX_SPEED_V)) ? speed: INV_SPEED_V;
#if 0
      sprintf(UAS_data.SelfID.Desc,"%5u %5u ",gps.speed_2d_cm,gps.speed_3d_cm);
#endif
    } else {

      UAS_data.Location.Status    = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
    }

    if ((!base_set)&&(gps.fix)&&(gps.satellites >= 8)) { // REQ_SATS

      UAS_data.System.OperatorLatitude    = gps.latitude_d;
      UAS_data.System.OperatorLongitude   = gps.longitude_d;
      UAS_data.System.OperatorAltitudeGeo = gps.altitude_msl_m;

      base_set = 1;
    }

    squitter.foreground(&UAS_data);

#if SPEKTRUM
    if ((gps.nmea_flags & 0x03)&&(gps.ublox_flags & 0x54)) {

      spektrum.foreground(gps.fix,gps.satellites,gps.zulu_s,
                          gps.latitude_s,gps.north,gps.longitude_s,gps.east,
                          gps.altitude_msl_m,gps.heading,
                          gps.speed_kn,gps.speed_2d_cm,gps.hdop_f);
      gps.reset_flags();
    }
#endif

    k_sleep(K_MSEC(5));

    uptime        = k_uptime_get_32();
    toggle_period = 1200 - (gps.satellites * 50);

    if ((!build_report)&&(uptime > 20000)) {

      sprintf(text,"%s %s, %d dbm",
              program_name,__DATE__,(int) ext_adv_info.tx_power);
      txt_message(text);

      build_report = 1;
    }

    if ((uptime - last_toggle) > toggle_period) {

      last_toggle = uptime;  
      gpio_pin_toggle_dt(&led_green);
    }

#if not GPS_PASSTHROUGH
    sprintf(text,"\r %8u %6u %1d %6u %1d %3d %3d %3d ",
            uptime,squitter.counter_4,squitter.bt4_ad_phase, 
            squitter.counter_5,squitter.bt5_ad_phase,
            (int) squitter.msg_counter[1],(int) squitter.msg_counter[2],(int) squitter.msg_counter[3]);
    debug_message(text);
    sprintf(text," %2d %3d %3d ",
            gps.config_phase,gps.max_parse_index,gps.max_section_index);
    debug_message(text);
#endif
  }

  return 0;
}

/*
 *
 */

void txt_message(const char *text) {

  gps.txt_message(text,1,1,2);
  
  return;
}

//

void txt_message(const char *text,const int message,const int messages,const int severity) {

  gps.txt_message(text,message,messages,severity);
  
  return;
}

/*
 *
 */

void debug_message(const char *message) {

  while (*message) {
    uart_poll_out(usb_ser_dev,*message++);
  }
  
  return;
}

/*
 *
 */

static void init_odid(const char *uav_op,char *uav,const char *self) {

  uint32_t            sn1, sn2;
  ODID_Location_data *location;
  ODID_System_data   *system;

  memset(&UAS_data,0,sizeof(UAS_data));

  location = &UAS_data.Location;
  system   = &UAS_data.System;

  //

  if (uav[0] == '~') {

    sn1 = sys_rand32_get() % 1000000;
    sn2 = sys_rand32_get() % 1000000;

    sprintf(uav,"ZZZZL%06u%06u",sn1,sn2);
  }

  //

  UAS_data.BasicID[0].UAType =
  UAS_data.BasicID[1].UAType = ua_type;

#if ID_JAPAN
  UAS_data.BasicID[0].IDType = ODID_IDTYPE_SERIAL_NUMBER;
  strncpy(UAS_data.BasicID[0].UASID,uav,ODID_ID_SIZE);

  UAS_data.BasicID[1].IDType = ODID_IDTYPE_CAA_REGISTRATION_ID;
  strncpy(UAS_data.BasicID[1].UASID,uav_op,ODID_ID_SIZE);

  UAS_data.Auth[0].AuthType  = ODID_AUTH_MESSAGE_SET_SIGNATURE;
  UAS_data.Auth[0].Length    = 17;
#else
  if (uav[0]) {
    UAS_data.BasicID[0].IDType = ODID_IDTYPE_SERIAL_NUMBER;
    strncpy(UAS_data.BasicID[0].UASID,uav,ODID_ID_SIZE);
  }
#endif

  location->Status             = ODID_STATUS_UNDECLARED;
  location->Direction          = INV_DIR;
  location->SpeedHorizontal    = INV_SPEED_H;
  location->SpeedVertical      = INV_SPEED_V;
  location->AltitudeBaro       = INV_ALT;
  location->AltitudeGeo        = INV_ALT;
  location->Height             = INV_ALT;
  location->HeightType         = ODID_HEIGHT_REF_OVER_TAKEOFF;
  location->HorizAccuracy      = ODID_HOR_ACC_10_METER;
  location->VertAccuracy       = ODID_VER_ACC_10_METER;
  location->BaroAccuracy       = ODID_VER_ACC_10_METER;
  location->SpeedAccuracy      = ODID_SPEED_ACC_10_METERS_PER_SECOND;
  location->TSAccuracy         = ODID_TIME_ACC_1_5_SECOND;

  system->OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
  system->AreaCount            = 1;
  system->AreaRadius           = 500;
  system->AreaCeiling          =
  system->AreaFloor            = -1000.0;
  system->OperatorAltitudeGeo  = -1000.0;

#if ! (ID_USA || ID_JAPAN)
  if (category) {
    system->ClassificationType = ODID_CLASSIFICATION_TYPE_EU;
    system->CategoryEU         = category;
    system->ClassEU            = ODID_CLASS_EU_UNDECLARED;
  }
#endif

#if ! ID_USA
  if (uav_op[0]) {
    UAS_data.OperatorID.OperatorIdType = ODID_OPERATOR_ID;
    strncpy(UAS_data.OperatorID.OperatorId,uav_op,ODID_ID_SIZE);
  }
#endif

  if (self[0]) {
    UAS_data.SelfID.DescType   = ODID_DESC_TYPE_TEXT;
    strncpy(UAS_data.SelfID.Desc,self,ODID_STR_SIZE);
  }

  //

  return;
}

/*
 *
 */

/*
 *
 */

