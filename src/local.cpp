/* -*- tab-width: 2; mode: c; -*-
 *
 * A program for the nRF52840 to transmit ASTM F3411/ASD-STAN 4709-002/opendroneid 
 * remote identification signals over Bluetooth.
 * 
 * This file contains the code specific to regional/national variations 
 * on the standards.
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
 * The cryptography for the Japanese ID is probably wrong, but shouldn't be far off.
 * The English translation of the specification for the Japanese ID is available at -
 * https://www.mlit.go.jp/koku/content/001582250.pdf
 *
 * There is a slight conflict in the specification over the tag length, 12 or 16?
 *
 * TINYCRYPT will only do a 13 byte nonce in CCM mode and is therefore useless for Japanese IDs.
 *
 *
 */

#include "remote_id.h"

#include <zephyr/crypto/crypto.h>

#if defined(CONFIG_CRYPTO_TINYCRYPT_SHIM)
#define CRYPTO_DRV_NAME      CONFIG_CRYPTO_TINYCRYPT_SHIM_DRV_NAME
#elif defined(CONFIG_CRYPTO_MBEDTLS_SHIM)
#define CRYPTO_DRV_NAME      CONFIG_CRYPTO_MBEDTLS_SHIM_DRV_NAME
#else
#warning "CRYPTO_DRV_NAME not set"
#endif

static const int             japan_key_len    = 16, //
                             japan_nonce_len  = 12, // Specification 3.(2)
                             japan_tag_len    = 12; //
static uint32_t              crypto_dev_flags = 0;
static const struct device  *crypto_dev = NULL;
static struct cipher_ctx     japan_ctx;

/*
 *
 */

int crypto_init(uint8_t *key) {

  int  status = 0;
  char text[128];

  memset(&japan_ctx,0,sizeof(japan_ctx));

  japan_ctx.keylen                         = japan_key_len;
  japan_ctx.key.bit_stream                 = key;
  japan_ctx.mode_params.ccm_info.tag_len   = japan_tag_len;
  japan_ctx.mode_params.ccm_info.nonce_len = japan_nonce_len;
  japan_ctx.flags                          = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

  if (!(crypto_dev = device_get_binding(CRYPTO_DRV_NAME))) {
    sprintf(text,"%s(): %s not found",__func__,CRYPTO_DRV_NAME);
    txt_message(text,1,1,0);
    return -1;
  }

  if (!device_is_ready(crypto_dev)) {
    sprintf(text,"%s(): Crypto device %08x not ready",__func__,crypto_dev);
    txt_message(text,1,1,0);
    return -1;
  }

  crypto_dev_flags = crypto_query_hwcaps(crypto_dev);  

  return status;
}

#if ID_JAPAN

/*
 *
 */

int auth_japan(ODID_UAS_Data *UAS_data,ODID_Message_encoded *messages_enc,uint8_t *iv) {

  int                          i, j, status = 0, length = 0;
  const int                    max_cipher_len = 128;
  char                        *text;
  uint8_t                     *plain_text, cipher_text[160],
                               add_data[4], nonce[16], *tag, *auth,
                              *u8;
  uint16_t                     u16;
  uint32_t                     u32;
  struct cipher_pkt            cipher_packet;
  struct cipher_aead_pkt       aead_packet;
  ODID_Location_encoded       *location_enc;
  ODID_Auth_encoded_page_zero *auth_enc;

  // Set all the pointers.

  text        = (char *) cipher_text;
  tag         = &cipher_text[max_cipher_len];
  add_data[0] = 0;
  
  plain_text   = (uint8_t *)                      messages_enc;
  auth_enc     = (ODID_Auth_encoded_page_zero *) &messages_enc[3];
  location_enc = (ODID_Location_encoded *)       &messages_enc[2];
  auth         = &auth_enc->AuthData[1];
  u8           = cipher_text;

  //
  
  encodeAuthMessage((ODID_Auth_encoded *) auth_enc,&UAS_data->Auth[0]);

  memset(&cipher_packet,0,sizeof(cipher_packet));
  cipher_packet.in_buf      = plain_text;
  cipher_packet.in_len      = 4 * ODID_MESSAGE_SIZE;
  cipher_packet.out_buf_max = max_cipher_len;
  cipher_packet.out_buf     = cipher_text;

  memset(&aead_packet,0,sizeof(aead_packet));
  aead_packet.ad     = add_data;
  aead_packet.ad_len = 0;
  aead_packet.pkt    = &cipher_packet;
  aead_packet.tag    = tag;
  
  u16 = location_enc->TimeStamp;
  u32 = auth_enc->Timestamp;

  memset(nonce,0,sizeof(nonce));
  memcpy(nonce,iv,6);
  u8           = (uint8_t *) &location_enc->TimeStamp;
  nonce[j = 6] = u8[0];
  nonce[++j]   = u8[1];
  u8           = (uint8_t *) &auth_enc->Timestamp;
  nonce[++j]   = u8[0];
  nonce[++j]   = u8[1];
  nonce[++j]   = u8[2];
  nonce[++j]   = u8[3];

  if (crypto_dev) {

    if (status = cipher_begin_session(crypto_dev,&japan_ctx,CRYPTO_CIPHER_ALGO_AES,
                                      CRYPTO_CIPHER_MODE_CCM,CRYPTO_CIPHER_OP_ENCRYPT)) {
      sprintf(text,"%s(): cipher_begin_session() returned %d",__func__,status);
      txt_message(text,1,1,1);
    
    } else {

      if (status = cipher_ccm_op(&japan_ctx,&aead_packet,nonce)) {

        sprintf(text,"%s(): cipher_ccm_op() returned %d, %04x %04x",
                __func__,status,crypto_dev_flags,japan_ctx.flags);
        txt_message(text,1,1,1);

      } else {
#if 0
        for (i = 0; i < japan_tag_len; ++i) {
          sprintf(&text[i * 3],"%02x ",tag[i]);
        }

        txt_message(text);
#endif
        length = cipher_packet.out_len;
        memcpy(auth,tag,japan_tag_len);
      }
 
      cipher_free_session(crypto_dev,&japan_ctx);
    }
  }

  return length;
}

#endif

/*
 *
 */

