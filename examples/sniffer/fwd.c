/*
 * Copyright (c) 2012, Sucola Superiore Sant'Anna
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Andorid sniffer example for the Tmote Sky platform. To be used 
 *         together with the Sniffer 15.4 Andorid app.
 *
 * \author
 *         Daniele Alessandrelli - <d.alessandrelli@sssup.it>
 */



#include "contiki.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "dev/cc2420.h"
#define DEBUG DEBUG_NONE
#include "net/uip-debug.h"
#include "dev/slip.h"
#include "dev/leds.h"
#include "cmd.h"
#include "packetutils.h"
// #include "dev/serial-line.h"
#include "dev/uart1.h"
/*
 * We cannot receive the channel value 13 (0x0D) from the serial line correctly
 * unless we offset the channel number. That is because 0x0D is Carriage Return
 * and is omitted by Contiki
 */


#ifndef CHANNEL
#define CHANNEL 26
#endif

#ifndef NODEID
#define NODEID 0
#endif

#ifndef AUTOSEND
#define AUTOSEND 0
#endif

#define MAGIC_LEN 4
#define TAIL_LEN 7
/* 
 * The following defines identify the fields included in the USB packet sent 
 * to the Android device
 */
#define FIELD_CRC        1
#define FIELD_CRC_OK     2
#define FIELD_RSSI       4
#define FIELD_LQI        8
#define FIELD_TIMESTAMP 16
#define SLIP_END     0300
#define SLIP_ESC     0333
#define SLIP_ESC_END 0334
#define SLIP_ESC_ESC 0335
/* 
 * Packet type: sniffed packet
 * Format:  magic | type | len | pkt | crc_ok | rssi | lqi 
 */
#define MY_TYPE  (char) (FIELD_CRC_OK | FIELD_RSSI | FIELD_LQI)
/*
 * Packet type: start sniffing
 * Foramt: type | ch 
 */
#define TYPE_START_SNIF 0xFA
/* 
 * Packet type: stop sniffing
 * Format: type
 */
#define TYPE_STOP_SNIF  0xFB
#define SERIAL_BUF_SIZE 128
static char rx_buf[SERIAL_BUF_SIZE];
static uint8_t reading_flg;
static uint8_t cnt=0;
#define set_channel cc2420_set_channel
#define NETSTACK_RADIO cc2420_driver
static int
slip_radio_cmd_handler(const uint8_t *data, int len);
CMD_HANDLERS(slip_radio_cmd_handler);
/*---------------------------------------------------------------------------*/
/* The variable where the radio driver stores the result of the FCS check */
uint8_t sniffer_crc_ok;
/* The variable where the radio driver stores the FCS of the sniffed packet */
uint8_t sniffer_crc[2];

/* 
 * The magic sequence for synchronizing the communication from the sniffer to 
 * the android device. The magic sequence is sent before sending the actual 
 * data to the android device.
 */
/*                              0x53 0x4E 0x49 0x46                          */
//static const uint8_t magic[] = { 'S', 'N', 'I', 'F'};
//static const uint8_t magic[] = { 0x53, 0x4E, 0x49, 0x46 };
static const uint8_t magic[] = { 0xC1, 0x1F, 0xFE, 0x72 };
//extern process_event_t serial_line_event_message;
//static uint16_t current_pntr=0;
//static uint8_t snif_enabled;
//static uint8_t channel;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void
slip_cmd_output(const uint8_t *data, int data_len)
{
  leds_toggle(LEDS_RED);
  //slip_write(data,data_len);

}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static int
radio_auto()
{
uint8_t tx_bytes[57]={0x61,0xcc,0x20,0xcd,0xab,0xdb,0xd8,0x65,0x14,0x00,0x74,0x12,0x00,0x21,0xf1,0x6e,0x14,0x00,0x74,0x12,0x00,0x7e,0xf6,0x00,0x00,0x01,0xf0,0x22,0x3d,0x16,0x2e,0x8c,0x63,0x48,0x65,0x6c,0x6c,0x6f,0x20,0x32,0x30,0x20,0x66,0x72,0x6f,0x6d,0x20,0x74,0x68,0x65,0x20,0x63,0x6c,0x69,0x65,0x6e,0x74};
tx_bytes[2]+=cnt;

packetbuf_clear();
memcpy(packetbuf_dataptr(),tx_bytes, 57);
packetbuf_set_datalen(57);
NETSTACK_RADIO.send(packetbuf_dataptr(),57);

leds_toggle(LEDS_RED);
cnt++;
return 1;
}
static int
slip_auto()
{
/*uint8_t tx_bytes[99]={ 0xC1, 0x1F, 0xFE, 0x72,0x41,0xc8,0x01,0xcd,0xab,0xff,0xff,0xdb,0xd8,0x65,0x14,0x00,0x74,0x12,0x00,0x7a,0x3b,0x3a,0x1a,0x9b,0x01,0x1c,0xd2,0x1e,0xf0,0x01,0x00,0x10,0xf0,0x00,0x00,0xaa,0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xfe,0x00,0x00,0x01,0x04,0x0e,0x00,0x08,0x0c,0x0a,0x07,0x00,0x01,0x00,0x00,0x01,0x00,0xff,0xff,0xff,0x08,0x1e,0x40,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xaa,0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
slip_write(tx_bytes,99);*/
uint8_t tx_bytes[63]={0xC1, 0x1F, 0xFE, 0x72, 0x00,0x39,0x61,0xcc,0x20,0xcd,0xab,0xdb,0xd8,0x65,0x14,0x00,0x74,0x12,0x00,0x21,0xf1,0x6e,0x14,0x00,0x74,0x12,0x00,0x7e,0xf6,0x00,0x00,0x01,0xf0,0x22,0x3d,0x16,0x2e,0x8c,0x63,0x48,0x65,0x6c,0x6c,0x6f,0x20,0x32,0x30,0x20,0x66,0x72,0x6f,0x6d,0x20,0x74,0x68,0x65,0x20,0x63,0x6c,0x69,0x65,0x6e,0x74};
tx_bytes[2]+=cnt;
slip_write(tx_bytes,63);
leds_toggle(LEDS_RED);
cnt++;
return 1;
}
/*--------------------------------------------------------------------------------------------------*/
static int 
slip_radio_cmd_handler(const uint8_t *data, int len)
{
uint16_t i;
//check magic
for (i=0;i<MAGIC_LEN;i++)
{
if (data[i]!=magic[i])
{
return 0;
}
}
if (len==99){
packetbuf_clear();
memcpy(packetbuf_dataptr(),&data[MAGIC_LEN], len-MAGIC_LEN);
packetbuf_set_datalen(len-MAGIC_LEN);
NETSTACK_RADIO.send(packetbuf_dataptr(),len-MAGIC_LEN);
leds_toggle(LEDS_GREEN);
}
else
{
leds_toggle(LEDS_BLUE);
}
return 1;
}
/*---------------------------------------------------------------------------*/
void
sniffer_input()
{
//get packet from sniffer and send to cmd
  uint16_t pkt_len;
//  uint8_t rssi;
//  uint8_t lqi;
//  uint16_t timestamp;
  uint16_t i;
  uint8_t pkt[256];
  pkt_len = packetbuf_datalen();
//  phy_pkt_len=pkt_len;
//  memcpy(&pkt[MAGIC_LEN],packetbuf_dataptr(),pkt_len);
  memcpy(pkt,packetbuf_dataptr(),pkt_len);
//  rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
//  lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);
//  timestamp = packetbuf_attr(PACKETBUF_ATTR_TIMESTAMP);

     
/*    pkt[phy_pkt_len]=rssi;
    phy_pkt_len++;
    pkt[phy_pkt_len]=lqi;*/
//  leds_toggle(LEDS_GREEN);  
/*
for (i=0;i<MAGIC_LEN;i++)
{
pkt[i]=magic[i];
}
*/

//  slip_write(pkt,pkt_len);


//  packetbuf_clear();

 /* printf("New packet\n");
  printf("Pakcet len: %u\n", pkt_len);
  printf("Packet:");
  for (i = 0; i < pkt_len; i++) {
    printf(" %2x", pkt[i]);
  }
  printf("\n");
  printf("CRC: none\n");
  printf("CRC OK: %d\n", !!sniffer_crc_ok);
  printf("RSSI: %u\n", 255 - rssi);
  printf("LQI: %u\n", lqi);
  printf("Timestamp: %u\n", timestamp);*/

  /* magic | type | len | pkt | crc_ok | rssi | lqi */
/*
  for (i = 0; i < MAGIC_LEN; i++) {
    putchar(magic[i]);
  }
    putchar(0x02); //version
    putchar(0x00); //command
    phy_pkt_len+=2;
    putchar((phy_pkt_len >> 8) & 0xFF); //length
    putchar(phy_pkt_len & 0xFF);
  for (i = 0; i < phy_pkt_len; i++) {
    putchar(pkt[i]);
  }
 putchar(rssi & 0xFF);
 putchar(0x80 | (lqi & 0xFF));
*/
//    packetbuf_copyfrom(pkt,pkt_len);
/*  if (MY_TYPE & FIELD_CRC) {
    putchar(sniffer_crc[0]);
    putchar(sniffer_crc[1]);
  }
  if (MY_TYPE & FIELD_CRC_OK) {
    putchar(sniffer_crc_ok);
  }
  if (MY_TYPE & FIELD_RSSI) {
    putchar(rssi);
  }
  if (MY_TYPE & FIELD_LQI) {
    putchar(lqi);
  }
  if (MY_TYPE & FIELD_TIMESTAMP) {
    putchar((timestamp >> 8) & 0xFF);
    putchar(timestamp & 0xFF);
  }*/
}

static void
slip_input_callback(uint8_t* buffer,uint16_t blen)
{
slip_radio_cmd_handler(buffer,blen);
}
/*------------------------------------------------------------------*/
/*void send_packet(){
char msg[50];
putchar(0x44);
putchar(0x45);
putchar(0x46);
uint8_t i;
for (i=0;i<MAGIC_LEN;i++)
{
msg[i]=(char)magic[i];
}
#if NODEID
uint16_t len=sprintf(&msg[MAGIC_LEN],"ABC");
#else
uint16_t len=sprintf(&msg[MAGIC_LEN],"KLM");
#endif
slip_write(msg,len+MAGIC_LEN);
//printf("Hello!!!\n");
}
int
test_char(unsigned char c){
printf ("%2x ", (uint8_t)c);
}
*/
/*---------------------------------------------------------------------------*/
PROCESS(sniffer_process, "Sniffer process");
AUTOSTART_PROCESSES(&sniffer_process,&slip_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sniffer_process, ev, data)
{
static struct etimer et;
//uart1_init(115200);
//  uart1_init(BAUD2UBR(115200)); //set the baud rate as necessary
//  uart1_set_input(uart_rx_callback); //set the callback function
uart1_set_input(slip_input_byte); //set the callback function
//uart1_set_input(test_char);
//slip_arch_init(115200);
//uart1_set_input(recv_a_byte);
slip_set_raw_input_callback(slip_input_callback);
//  snif_enabled=1;
  PROCESS_BEGIN();
//  uart1_init(115200);
//  uart1_active();
//  slip_arch_init(115200);
//  slip_set_raw_input_callback(slip_input_callback);
  NETSTACK_RADIO.on();
  set_channel(CHANNEL);
//  process_poll(&slip_process);
//slip_set_tcpip_input_callback(slip_input_callback);
/* for(;;) {
     PROCESS_YIELD();
     if(ev == serial_line_event_message){
       slip_radio_cmd_handler(data, strlen(data));
     }
   }*/
#if AUTOSEND
etimer_set(&et, CLOCK_SECOND * 1);

	while(1){
	PROCESS_WAIT_EVENT();
	    if (ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
//		 send_packet();
//		slip_radio_cmd_handler(NULL,0);
//		radio_auto();
		slip_auto();
//		 leds_toggle(LEDS_RED);
		etimer_reset(&et);
	}
	}
#endif
//#endif
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
