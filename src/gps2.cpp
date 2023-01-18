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
 *
 */

#pragma GCC diagnostic warning "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wparentheses"
// #pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include "gps2.h"

#include <ctype.h>
#include <math.h>

#define BAUD_RATE      57600 // 9600 for initial configuration, 57600 thereafter.
#define GPS_RATE         200 // ms
#define RX_BUFFER_SIZE   512
#define TX_BUFFER_SIZE   128
#define CONFIG_PHASES     40

extern "C" {
  static void gps_rx_handler(const struct device *,void *);
  static void usb_rx_handler(const struct device *,void *);

  volatile void            *gps_dev2 = NULL;
  static volatile uint8_t   rx_buffer[RX_BUFFER_SIZE + 2];
  static volatile uint16_t  bg_rx_index = 0;
  static volatile uint32_t  rx_counter = 0;
};

/*
 *  I don't think that Zephyr actually calls this.
 *  Variable setup moved to begin().
 */

GPS2::GPS2() {

  return;
}

/*
 *
 */

void GPS2::begin(const struct device *uart,const struct device *usb_serial,const struct gpio_dt_spec *led) {

  int                 i;
  uint16_t            rate = GPS_RATE;
  uint32_t            baud = BAUD_RATE;
  struct uart_config  serial_config;
#if not GPS_PASSTHROUGH
  char                text[128];
#endif

  //
  // Initialise variables.
  //

  memset(parse_buffer,      0,sizeof(parse_buffer));
  memset((void *) rx_buffer,0,sizeof(rx_buffer));
  memset(&utc,              0,sizeof(utc));

  for (i = 0; i < MAX_SECTIONS + 1; ++i) {

    section[i] = parse_buffer;
  }

  parse_index   = max_parse_index   = 0;
  section_index = max_section_index = 0;
  
  // GPS commands.
  // Note that the payload indices below are offset by 4 from the Ublox literature.

  gps_protocols = 0x03; // Bit 0 Ublox, bit 1 NMEA.

  sprintf(nmea_cfg_uart1,"PUBX,41,1,%04x,%04x,%d,0",7,gps_protocols,(int) BAUD_RATE);

  memset(ubx_cfg_rate, 0,sizeof(ubx_cfg_rate));
  memset(ubx_cfg_prt,  0,sizeof(ubx_cfg_prt));
  memset(ubx_cfg_msg,  0,sizeof(ubx_cfg_msg));
  memset(ubx_cfg_nav5, 0,sizeof(ubx_cfg_nav5));
  memset(ubx_poll_nav5,0,sizeof(ubx_poll_nav5));

  ubx_cfg_rate[0]  = 0x06;
  ubx_cfg_rate[1]  = 0x08;
  ubx_cfg_rate[2]  = 0x06;
  ubx_cfg_rate[4]  = (uint8_t) (rate & 0xff);
  ubx_cfg_rate[5]  = (uint8_t) (rate >> 8);
  ubx_cfg_rate[6]  = 0x01;
  ubx_cfg_rate[8]  = 0x01;

  ubx_cfg_prt[0]   = 0x06;
  ubx_cfg_prt[2]   = 0x14;
  ubx_cfg_prt[4]   = 0x01;
  ubx_cfg_prt[8]   = 0xd0;
  ubx_cfg_prt[9]   = 0x08;
  ubx_cfg_prt[12]  = (uint8_t) (0xff &  baud);
  ubx_cfg_prt[13]  = (uint8_t) (0xff & (baud >>  8));
  ubx_cfg_prt[14]  = (uint8_t) (0xff & (baud >> 16));
  ubx_cfg_prt[16]  = 0x03;
  ubx_cfg_prt[18]  = gps_protocols;

  ubx_cfg_msg[0]   = 0x06;
  ubx_cfg_msg[1]   = 0x01;
  ubx_cfg_msg[2]   = 0x08;

  ubx_cfg_nav5[0]  = 0x06;
  ubx_cfg_nav5[1]  = 0x24;
  ubx_cfg_nav5[2]  = 0x24;
  ubx_cfg_nav5[4]  = 0x05;
  ubx_cfg_nav5[6]  = 0x07;
  ubx_cfg_nav5[7]  = 0x02;

  ubx_poll_nav5[0] = 0x06;
  ubx_poll_nav5[1] = 0x24;

  //
  // Initialise devices.
  //
  
  usb_dev    = (struct device *)       usb_serial;
  gps_dev    = (struct device *)       uart;
  gps_dev2   = (volatile void *)       uart;
  status_led = (struct gpio_dt_spec *) led;

  if (status_led) {

    gpio_pin_configure_dt(status_led,GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(status_led,0);
  }

  if ((!gps_dev)||(!usb_dev)) {
#if not GPS_PASSTHROUGH
    printk("%s(): No devices, aborting\n",__func__);
#endif
    return;
  }

  // USB
  
  status1 = uart_config_get(usb_dev,&serial_config);

  serial_config.baudrate  = 115200;
  serial_config.parity    = UART_CFG_PARITY_NONE;
  serial_config.data_bits = UART_CFG_DATA_BITS_8;
  serial_config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE; 
  
  status2 = uart_configure(usb_dev,&serial_config);

#if GPS_PASSTHROUGH
  uart_irq_callback_user_data_set(usb_dev,usb_rx_handler,NULL);
  uart_irq_rx_enable(usb_dev);
#endif

  // GPS

  status1 = uart_config_get(gps_dev,&serial_config);
  
  serial_config.baudrate  = 9600;
  serial_config.parity    = UART_CFG_PARITY_NONE;
  serial_config.data_bits = UART_CFG_DATA_BITS_8;
  serial_config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE; 
  status2 = uart_configure(gps_dev,&serial_config);

  uart_irq_callback_user_data_set(gps_dev,gps_rx_handler,NULL);
  uart_irq_rx_enable(gps_dev);

  return;
}

/*
 *
 */

int GPS2::foreground() {

  int             csum_OK = 1, reset_index = 0;
  char            text[128], *a;
  uint8_t         c, ck_a, ck_b;
  uint32_t        msecs;

  msecs = k_uptime_get_32();

  while (fg_index != bg_rx_index) {

    gpio_pin_set_dt(status_led,1);

    c = rx_buffer[fg_index];
#if GPS_PASSTHROUGH
    uart_poll_out(usb_dev,c);
#endif
    
    if (read_mode == 0) {

      if (c == '$') {         // Start of a NMEA message.

        read_mode = 1;

        nmea_checksum      = 0;
        calc_nmea_checksum = 0;

      } else if (c == 0xb5) { // Start of a Ublox message.

        read_mode = 2;
      } 
    }

    //

    if (read_mode == 1) { // NMEA

      switch (parse_buffer[parse_index] = c) {

      case 10: // LF

        sprintf(text,"%02X",nmea_checksum);

        a       = (char *) section[MAX_SECTIONS];
        csum_OK = ((text[0] == a[0])&&(text[1] == a[1])) ? 1: 0;

        if (csum_OK) {

          ++nmea_messages;
          
          parse_nmea(section_index);
          last_nmea = msecs;
          
        } else {

          ++nmea_errors;
        }

        reset_index = 1;
        break;

      case 13: // CR

        parse_buffer[parse_index] = 0;
        break;

      case ',':

        if (calc_nmea_checksum) {
          nmea_checksum ^= c;
        }

        section[section_index++]  = &parse_buffer[parse_index + 1];
        parse_buffer[parse_index] = 0;

        if (section_index > max_section_index) {
          max_section_index = section_index;
        }

        if (section_index > (MAX_SECTIONS - 1)) {
          reset_index = 1;
        }
        
        break;

      case '*':

        parse_buffer[parse_index] = 0;
        section[MAX_SECTIONS]     = &parse_buffer[parse_index + 1];
        calc_nmea_checksum        = 0;
        break;

      case '$':
        
        calc_nmea_checksum = 1;
        break;

      default:

        if (calc_nmea_checksum) {
          nmea_checksum ^= c;
        }

        break;
      }

      ++parse_index;
      
    } // NMEA

    if (read_mode == 2) { // Ublox

      parse_buffer[parse_index] = c;

      if (parse_index == 4) {

        ublox_length = 6 + (int) c;
      }

      if (parse_index == (ublox_length + 1)) {

        ublox_checksum(parse_buffer,&ck_a,&ck_b);

        csum_OK = ((parse_buffer[parse_index - 1] == ck_a)&&(parse_buffer[parse_index] == ck_b)) ? 1: 0;

        if (csum_OK) {

          ++ublox_messages;

          parse_ublox();
          last_ublox = msecs;

        } else {

          ++ublox_errors;
        }

        reset_index  = 1;
        ublox_length = 6;
      }

      ++parse_index;
     
    } // Ublox

    if (parse_index > max_parse_index) {
      max_parse_index = parse_index;
    }

    if ((reset_index)||(parse_index > (PARSE_BUFFER_SIZE - 8))) {

      read_mode     = 0;
      reset_index   = 0;
      parse_index   = 0;
      section_index = 1;
    }

    //

    if (++fg_index >= RX_BUFFER_SIZE) {
      fg_index = 0;
    }

    gpio_pin_set_dt(status_led,0);    
  }

  if ((msecs - last_config) > 249) {
    
    if (config_phase < CONFIG_PHASES) {
      configure(0);
    }
#if 0
    sprintf(text,"%7u %7u",msecs,rx_counter);
    txt_message(text);
#endif
  }

  if ((config_phase >= CONFIG_PHASES)&&
      ((msecs - last_nmea)   > 9999)&&
      ((msecs - last_config) > 4999)) {
    configure(1);
  }
  
  return 0;
}

/*
 *
 */

int GPS2::txt_message(const char *text) {

  return txt_message(text,1,1,2);
}

//

int GPS2::txt_message(const char *text,
                      const int message,const int messages,const int severity) {

  int  len;
  char text2[TX_BUFFER_SIZE], tx_buffer[TX_BUFFER_SIZE];

  if ((len = strlen(text)) > (TX_BUFFER_SIZE - 24)) {

    return -1;
  }

  sprintf(text2,"GNTXT,%02d,%02d,%02d,%s",messages,message,severity,text);
  nmea_message(text2,tx_buffer,sizeof(tx_buffer));

  len = strlen(tx_buffer);

  for (int i = 0; i < len; ++i) {

    uart_poll_out(usb_dev,tx_buffer[i]);
#if TXT_BOTH_PORTS
    uart_poll_out(gps_dev,tx_buffer[i]);
#endif
  }

  return 0;
}

/*
 *
 */

void GPS2::reset_flags() {

  nmea_flags  = 0;
  ublox_flags = 0;
  
  return;
}

/*
 *
 */

int GPS2::configure(int override) {

  int                rate;
  char               text[128];
  uint32_t           msecs;
  struct {char    msg_name[4];
          uint8_t msg_class; 
          uint8_t msg_id; 
          uint8_t msg_rate;} 
                     nmea_msgs[16] =  {{"DTM", 0xf0, 0x0a,  0}, {"GBS", 0xf0, 0x09,  0},
                                       {"GGA", 0xf0, 0x00,  1}, {"GLL", 0xf0, 0x01,  0},
                                       {"GPQ", 0xf0, 0x40,  0}, {"GRS", 0xf0, 0x06,  0},
                                       {"GSA", 0xf0, 0x02, 10}, {"GST", 0xf0, 0x07,  0},
                                       {"GSV", 0xf0, 0x03,  0}, {"RMC", 0xf0, 0x04,  1},
                                       {"THS", 0xf0, 0x0e,  0}, {"TXT", 0xf0, 0x41, 10},
                                       {"VTG", 0xf0, 0x05,  0}, {"ZDA", 0xf0, 0x08,  0},
                                       {"",    0x00, 0x00,  0}, {"",    0x00, 0x00,  0}},
                     ublox_msgs[8]  = {{"",    0x01, 0x04, 10}, {"",    0x01, 0x02,  1},
                                       {"",    0x01, 0x06,  1}, {"",    0x01, 0x03,  1},
                                       {"",    0x01, 0x21,  1}, {"",    0x01, 0x12,  1},
                                       {"",    0x00, 0x00,  0}, {"",    0x00, 0x00,  0}};   
  //

  if (override) {

    config_phase = override;
  }

  msecs = k_uptime_get_32();

  if ((msecs - last_config) < 100) {

    return 0;
  }
  
  last_config = msecs;

  switch (config_phase) {

  case  1:
    set_uart_baud(gps_dev,9600);
    break;

  case  2:
    config_subphase = 0;

  case  3: case  4: case  5: case  6:
  case  7: case  8: case  9: case 10:
  case 11: case 12: case 13: case 14:
  case 15: case 16: case 17:

    if (config_subphase < 16) {

      if (nmea_msgs[config_subphase].msg_name[0]) {

        rate = nmea_msgs[config_subphase].msg_rate;
        sprintf(text,"PUBX,40,%s,%d,%d,%d,%d,%d,0",
                nmea_msgs[config_subphase].msg_name,
                rate,rate,rate,rate,rate);
        nmea_command(text);
      }

      ++config_subphase;
    }

    break;

  case 20:
    config_subphase = 0;

  case 21: case 22: case 23: case 24:
  case 25: case 26: case 27:

    if (config_subphase < 8) {

      if (ublox_msgs[config_subphase].msg_class) {
        
        GPS2::ubx_cfg_msg[4] = ublox_msgs[config_subphase].msg_class;
        GPS2::ubx_cfg_msg[5] = ublox_msgs[config_subphase].msg_id;

        for (int j = 0; j < 6; ++j) {

          GPS2::ubx_cfg_msg[6 + j] = (gps_protocols & 0x01) ? ublox_msgs[config_subphase].msg_rate: 0;
        }

        ublox_command(GPS2::ubx_cfg_msg);
      }

      ++config_subphase;
    }
    break;

  case 30:
    ublox_command(ubx_cfg_nav5);
    break;

  case 32:
    ubx_cfg_prt[4] = 0x02;
    ublox_command(ubx_cfg_prt);
    break;

  case 33:
    // The baud rate change is fragile.
    // Be careful changing it.
    ubx_cfg_prt[4] = 0x01;
    ublox_command(ubx_cfg_prt);

    k_sleep(K_MSEC(50));

    set_uart_baud(gps_dev,BAUD_RATE);
    break;
    
  case 36:
    ublox_command(ubx_cfg_rate);
    break;

  default:
    break;
  }

  if (config_phase < CONFIG_PHASES) {
    ++config_phase;
  }
  
  return config_phase;
}

//

void GPS2::set_uart_baud(device *uart,int baud_rate) {

  char text[128];
  
  uart_irq_rx_disable(uart);

  status3 = uart_config_get(uart,&serial_config);
  serial_config.baudrate = baud_rate;
  status4 = uart_configure(uart,&serial_config);

  uart_irq_rx_enable(uart);
  uart_config_get(uart,&serial_config);

  sprintf(text,"GPS2::%s(%d): %d, %d",__func__,
          baud_rate,status4,serial_config.baudrate);;
  txt_message(text);

  return;
}

/*
 *  
 *  $GNGGA,200842.00,1212.12345,N,12312.12345,W,1,06,1.30,33.5,M,47.9,M,,*6C
 *  $GNRMC,200843.00,A,1212.12345,N,12312.12345,W,0.618,,160818,,,A*7B
 *  $GNGSA,A,3,09,02,06,23,05,29,,,,,,,2.89,1.30,2.59*11
 *
 *  This is here mostly because it's useful when debugging the serial link to
 *  turn the Ublox off and see text messages (and because I like NMEA).
 *
 */

int GPS2::parse_nmea(int section_index) {

  // GGA

  if ((rx_buffer[3] == 'G')&&(rx_buffer[4] == 'G')&&(rx_buffer[5] == 'A')&&
      (section_index > 10)) {

    nmea_flags |= 0x01;

    // Time
    
    if (*section[1]) {
      strncpy(zulu_s,(char *) section[1],11);
    }

    // Lat. and Long.

    if (*section[2]) {
      strncpy(latitude_s,(char *) section[2],15);
    }    
    
    if (*section[4]) {
      strncpy(longitude_s,(char *) section[4],15);
    }
    
    // Altitude

    if (*section[9]) {
      strncpy(alt_msl_m_s,(char *) section[9],7);
    }

    return 1;
  }

  // RMC

  if ((rx_buffer[3] == 'R')&&(rx_buffer[4] == 'M')&&(rx_buffer[5] == 'C')&&
      (section_index > 10)) {

    nmea_flags |= 0x02;

    // Speed and heading

    if (*section[7]) {
      strncpy(speed_kn_s,(char *) section[7],7);
    }

    if (*section[8]) {
      strncpy(heading_s,(char *) section[8],7);
    }

    return 1;
  }

  if ((rx_buffer[3] == 'G')&&(rx_buffer[4] == 'S')&&(rx_buffer[5] == 'A')&&
      (section_index >= 17)) {

    // Dilution of Precision

    if (*section[16]) {
      strncpy(hdop_s,(char *) section[16],7);
    }

    if (*section[17]) {
      strncpy(vdop_s,(char *) section[17],7);
    }

    return 1;
  }

  return 0;
}

 /*
 *
 */

int GPS2::parse_ublox() {

  uint16_t length;
  union {int32_t  i32;
         uint32_t u32;} 
           u2i;

  length = parse_buffer[4] | (((uint16_t) parse_buffer[5]) << 8);

  switch (parse_buffer[2]) {

  case 0x01:

    if (parse_buffer[3] < 16) {

      ublox_flags |= (0x01 << parse_buffer[3]);
    }

    switch (parse_buffer[3]) {

    case 0x02: // POSLLH

      u2i.u32     =  ((uint32_t) parse_buffer[10]) | 
                    (((uint32_t) parse_buffer[11]) <<  8) |
                    (((uint32_t) parse_buffer[12]) << 16) |
                    (((uint32_t) parse_buffer[13]) << 24);
      longitude_u = u2i.i32;
      longitude_d = (double) longitude_u / 10000000.0;
      east        = (longitude_u < 0) ? 'W': 'E';
      
      u2i.u32     =  ((uint32_t) parse_buffer[14]) | 
                    (((uint32_t) parse_buffer[15]) <<  8) |
                    (((uint32_t) parse_buffer[16]) << 16) |
                    (((uint32_t) parse_buffer[17]) << 24);
      latitude_u  = u2i.i32;
      latitude_d  = (double) latitude_u / 10000000.0;
      north       = (latitude_u < 0) ? 'S': 'N';

      u2i.u32        =  ((uint32_t) parse_buffer[22]) | 
                       (((uint32_t) parse_buffer[23]) <<  8) |
                       (((uint32_t) parse_buffer[24]) << 16) |
                       (((uint32_t) parse_buffer[25]) << 24);
      alt_msl_mm     = u2i.i32;
      altitude_msl_m = 0.001 * (float) alt_msl_mm;
      break;

    case 0x03: // STATUS

      break;

    case 0x04: // DOP

      vdop_u      =  ((uint16_t) parse_buffer[16]) | 
                    (((uint16_t) parse_buffer[17]) <<  8);
      vdop_f      = 0.01 * (float) vdop_u;
      hdop_u      =  ((uint16_t) parse_buffer[18]) | 
                    (((uint16_t) parse_buffer[19]) <<  8);
      hdop_f      = 0.01 * (float) hdop_u;
      break;

    case 0x06: // SOL

      fix        = parse_buffer[16];
      satellites = parse_buffer[53];
      break;

    case 0x12: // VELNED

      u2i.u32         =  ((uint32_t) parse_buffer[10]) | 
                        (((uint32_t) parse_buffer[11]) <<  8) |
                        (((uint32_t) parse_buffer[12]) << 16) |
                        (((uint32_t) parse_buffer[13]) << 24);
      vel_N_cm        = u2i.i32;

      u2i.u32         =  ((uint32_t) parse_buffer[14]) | 
                        (((uint32_t) parse_buffer[15]) <<  8) |
                        (((uint32_t) parse_buffer[16]) << 16) |
                        (((uint32_t) parse_buffer[17]) << 24);
      vel_E_cm        = u2i.i32;

      u2i.u32         =  ((uint32_t) parse_buffer[18]) | 
                        (((uint32_t) parse_buffer[19]) <<  8) |
                        (((uint32_t) parse_buffer[20]) << 16) |
                        (((uint32_t) parse_buffer[21]) << 24);
      vel_D_cm        = u2i.i32;

      alt_speed_3d_cm = sqrt((double) ((vel_N_cm * vel_N_cm) +
                                       (vel_E_cm * vel_E_cm) + 
                                       (vel_D_cm * vel_D_cm)));
      speed_kn_3d     = 0.0194384 * alt_speed_3d_cm;

      speed_3d_cm     =  ((uint32_t) parse_buffer[22]) | 
                        (((uint32_t) parse_buffer[23]) <<  8) |
                        (((uint32_t) parse_buffer[24]) << 16) |
                        (((uint32_t) parse_buffer[25]) << 24);

      speed_2d_cm     =  ((uint32_t) parse_buffer[26]) | 
                        (((uint32_t) parse_buffer[27]) <<  8) |
                        (((uint32_t) parse_buffer[28]) << 16) |
                        (((uint32_t) parse_buffer[29]) << 24);
      speed_kn        = 0.0194384 * (float) speed_2d_cm;

      u2i.u32         =  ((uint32_t) parse_buffer[30]) | 
                        (((uint32_t) parse_buffer[31]) <<  8) |
                        (((uint32_t) parse_buffer[32]) << 16) |
                        (((uint32_t) parse_buffer[33]) << 24);
      heading_u       = u2i.i32;
      heading         = heading_u / 100000L;

      break;

    case 0x21: // TIMEUTC

      ublox_flags |= 0x8000;

      if (parse_buffer[25] & 0x04) {

        utc.tm_year = (parse_buffer[18] + (((uint16_t) parse_buffer[19]) << 8)) - 1900;
        utc.tm_mon  =  parse_buffer[20] - 1;
        utc.tm_mday =  parse_buffer[21];
        utc.tm_hour =  parse_buffer[22];
        utc.tm_min  =  parse_buffer[23];
        utc.tm_sec  =  parse_buffer[24];

        u2i.u32 =   ((uint32_t) parse_buffer[14]) | 
                   (((uint32_t) parse_buffer[15]) <<  8) |
                   (((uint32_t) parse_buffer[16]) << 16) |
                   (((uint32_t) parse_buffer[17]) << 24);
        csecs   = u2i.i32 / 10000000L;
      }
      break;

    default:
      break;
    }

    break;

  case 0x06:

    switch (parse_buffer[3]) {

    case 0x24:
      break;

    default:
      break;
    }

    break;

  default:
    break;
  }

  return 0;
}

 /*
 *
 */

int GPS2::send(const uint8_t *buffer,int length) {

  int i;

  for (i = 0; i < length; ++i) {

    uart_poll_out(gps_dev,buffer[i]);
  }

  return i;
}

/*
 *
 */

int GPS2::nmea_command(const char *message) {

  int  len;
  char tx_buffer[TX_BUFFER_SIZE];

  if ((len = nmea_message(message,tx_buffer,TX_BUFFER_SIZE)) > 0) {

    send((uint8_t *) tx_buffer,len);
  }
  
  return len;
}

int GPS2::nmea_message(const char *message,char *buffer,int buffer_size) {

  int      i, len;
  uint16_t checksum;

  buffer[0] = 0;
  len       = strlen(message);

  if (len > (buffer_size - 8)) {

    return -1;
  }

  for (checksum = 0, i = 0; i < len; ++i) {

    checksum ^= (uint8_t) message[i];
  }

  sprintf(buffer,"$%s*%02X\r\n",message,checksum);

  return strlen(buffer);
}

/*
 *
 */

int GPS2::ublox_command(const uint8_t *command) {

  int  i, len;
  uint8_t ck_a = 0, ck_b = 0, tx_buffer[TX_BUFFER_SIZE];

  len = command[2] + 4;
  
  if (len > (TX_BUFFER_SIZE - 6)) {

    return -1;
  }

  tx_buffer[0] = 0xb5;
  tx_buffer[1] = 0x62;

  for (i = 0; i < len; ++i) {

    tx_buffer[2 + i] = command[i];
  }

  ublox_checksum(tx_buffer,&ck_a,&ck_b);

  tx_buffer[len + 2] = ck_a;
  tx_buffer[len + 3] = ck_b;

  send(tx_buffer,len + 4);

  return 0;
}

/*
 *
 */

void GPS2::ublox_checksum(uint8_t *buffer,uint8_t *ck_a,uint8_t *ck_b) {

  int i, len;

  *ck_a = *ck_b = 0;
  
  len =  4 + buffer[4]; // + (((uint16_t) buffer[5]) << 8)

  for (i = 0; i < len; ++i) {

    *ck_a += buffer[2 + i];
    *ck_b += *ck_a;
  }

  return;
} 

/*
 *
 */

void gps_rx_handler(const struct device *gps_uart,void *a) {

  uint8_t c;
  
  ++rx_counter;

  if (!uart_irq_update(gps_uart)) {

    return;
  }

  while (uart_irq_rx_ready(gps_uart)) {

    uart_fifo_read(gps_uart,&c,1);

    rx_buffer[bg_rx_index] = c;
    
    if (++bg_rx_index >= RX_BUFFER_SIZE) {
      bg_rx_index = 0;
    }
  }

  return;
}

/*
 *
 */

#if GPS_PASSTHROUGH

void usb_rx_handler(const struct device *usb_serial,void *a) {

  uint8_t c;

  if (!uart_irq_update(usb_serial)) {

    return;
  }

  while (uart_irq_rx_ready(usb_serial)) {

    uart_fifo_read(usb_serial,&c,1);
    uart_poll_out((device *) gps_dev2,c);
  }

  return;
}

#endif

/*
 *
 */


