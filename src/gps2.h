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

#ifndef RID_GPS2_H
#define RID_GPS2_H

#include "remote_id.h"

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#define PARSE_BUFFER_SIZE 256
#define MAX_SECTIONS       24

//

class GPS2 {

 public:
                        GPS2();
  void                  begin(const struct device *,const struct device *,const struct gpio_dt_spec *);
  int                   foreground(void);
  int                   txt_message(const char *);
  int                   txt_message(const char *,const int,const int,const int);
  void                  reset_flags(void);

  // These are public for debugging.
  int                   max_parse_index = 0, max_section_index = 0,
                        config_phase = 1;
  // And these are public because they are the outputs.                           // Source
  char                  latitude_s[16], longitude_s[16],                          // GGA
                        north = 'N', east = 'E',                                  // POSLLH
                        alt_msl_m_s[8], zulu_s[12],                               // GGA
                        speed_kn_s[8], heading_s[8],                              // RMC
                        hdop_s[8], vdop_s[8];                                     // GSA
  int                   heading = 0,                                              // VELNED
                        satellites = 0,                                           // SOL
                        csecs = 0;                                                // TIMEUTC
  float                 speed_kn = 0, speed_kn_3d = 0, altitude_msl_m = 0.0,      // POSLLH
                        hdop_f = 99.99, vdop_f = 99.99;                           // DOP
  double                latitude_d = 0.0, longitude_d = 0.0,                      // POSLLH
                        alt_speed_3d_cm = 0.0;
  uint8_t               fix = 0;                                                  // SOL
  uint16_t              nmea_flags = 0, ublox_flags = 0,
                        hdop_u = 0, vdop_u = 0;                                   // DOP
  int32_t               alt_msl_mm = 0,                                           // POSLLH
                        vel_N_cm = 0, vel_E_cm = 0, vel_D_cm = 0;                 // VELNED 
  uint32_t              speed_3d_cm = 0, speed_2d_cm = 0;                         // VELNED
  struct tm             utc;                                                      // TIMEUTC

 private:

  int                   send(const uint8_t *,int);
  int                   configure(int);
  void                  set_uart_baud(device *,int);
  int                   parse_nmea(int);
  int                   parse_ublox(void);
  int                   nmea_command(const char *);
  int                   nmea_message(const char *,char *,int);
  int                   ublox_command(const uint8_t *);
  void                  ublox_checksum(uint8_t *,uint8_t *,uint8_t *);

  int                   parse_index = 0, section_index = 0, read_mode = 0,
                        status1 = 0, status2 = 0, status3 = 0, status4 = 0,
                        config_subphase = 24,
                        calc_nmea_checksum = 0, ublox_length = 6;
  char                  nmea_cfg_uart1[32];
  uint8_t               parse_buffer[PARSE_BUFFER_SIZE], *section[MAX_SECTIONS + 1],
                        gps_protocols,
                        ubx_poll_nav5[6], ubx_cfg_rate[16],
                        ubx_cfg_prt[32],  ubx_cfg_msg[16], ubx_cfg_nav5[48];
  int32_t               latitude_u = 0, longitude_u = 0,                          // POSLLH
                        heading_u = 0;                                            // VELNED
  uint16_t              nmea_checksum = 0, fg_index = 0;
  uint32_t              last_config = 0,
                        last_nmea  = 0, nmea_messages  = 0, nmea_errors  = 0, 
                        last_ublox = 0, ublox_messages = 0, ublox_errors = 0;
  struct device        *gps_dev = NULL, *usb_dev = NULL;
  struct uart_config    serial_config;
  struct gpio_dt_spec  *status_led = NULL;
};

#endif // RID_GPS2_H
