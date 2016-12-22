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
static uint8_t rx_buf[256];
static uint8_t recv_buf[256];
static uint8_t read_pos,write_pos;
static uint16_t length_pkt;
static uint8_t slip_buf[256];
static uint16_t len, tmplen;
static uint8_t lastc;
#define set_channel cc2420_set_channel
#define NETSTACK_RADIO cc2420_driver
static int
slip_radio_cmd_handler(const uint8_t *data, uint16_t len);
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
uint8_t slipdev_char_poll(uint8_t *c){
if (write_pos==read_pos)
	{
	//empty buf
	write_pos=0;
	read_pos=0;
	return 0;
	}
else{
	*c=rx_buf[read_pos];
	read_pos++;
	return 1;
}
}
uint16_t
slipdev_poll(uint8_t *outbuf)
{
  uint8_t c;
  
  while(slipdev_char_poll(&c)) {
    switch(c) {
    case SLIP_ESC:
      lastc = c;
      break;
      
    case SLIP_END:
      lastc = c;
     
      memcpy(outbuf,slip_buf,len);
      tmplen = len;
      len = 0;
      return tmplen;
      
    default:     
      if(lastc == SLIP_ESC) {
	lastc = c;
	/* Previous read byte was an escape byte, so this byte will be
	   interpreted differently from others. */
	switch(c) {
	case SLIP_ESC_END:
	  c = SLIP_END;
	  break;
	case SLIP_ESC_ESC:
	  c = SLIP_ESC;
	  break;
	}
      } else {
	lastc = c;
      }
      
      slip_buf[len] = c;
      ++len;
//      printf ("%2x ",c);
      if(len > 256) {
	printf ("uhu \n");
	len = 0;
      }
    
      break;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void
slip_cmd_output(const uint8_t *data, int data_len)
{
  leds_toggle(LEDS_BLUE);
  //slip_write(data,data_len);

}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static int
slip_radio_cmd_handler(const uint8_t *data, uint16_t len)
{

//send the packet to radio
//while(reading_flg){
//;}
uint16_t i;
//check magic

for (i=0;i<MAGIC_LEN;i++)
{
if (data[i]!=magic[i])
{
return 0;
}
}
//check legnth

uint16_t length;
length=data[MAGIC_LEN]*256+data[MAGIC_LEN+1];

/*
if (length > (len-MAGIC_LEN-2))
{
 leds_toggle(LEDS_GREEN);
//uart1_init(UART1_BAUD2UBR(115200ul));
 return 1;

}*/
/*
for (i=MAGIC_LEN+2;i<len;i++)
{
printf (" %2x ",data[i]);
}
printf("\n");*/

while(NETSTACK_RADIO.pending_packet() || NETSTACK_RADIO.receiving_packet());
packetbuf_clear();
memcpy(packetbuf_dataptr(),&data[MAGIC_LEN+2], len-MAGIC_LEN-2);
packetbuf_set_datalen(len-MAGIC_LEN-2);
NETSTACK_RADIO.send(packetbuf_dataptr(),len-MAGIC_LEN-2);

leds_toggle(LEDS_RED);
return 1;
}
/*--------------------------------------------------------------------------*/
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
//use slip
	if (pkt_len >0)
	{
	memcpy(&pkt[MAGIC_LEN+2],packetbuf_dataptr(),pkt_len);
	for (i=0;i<MAGIC_LEN;i++)
	{
		pkt[i]=magic[i];
	}
	//length
	pkt[MAGIC_LEN]=(pkt_len >> 8) & 0xFF;
	pkt[MAGIC_LEN+1]=pkt_len & 0xFF;
        //write
	 slip_write(pkt,pkt_len+MAGIC_LEN+2);
//	 printf ("send length %d \n",pkt_len+MAGIC_LEN+2);
	 packetbuf_clear();
}
}
/*--------------------------------------------------------------*/
int write_to_buf(unsigned char c){
uint16_t plength;
rx_buf[write_pos]=(uint8_t)c;
write_pos++;
plength=slipdev_poll(recv_buf);
printf ("%2x ",(uint8_t)c);
if (plength>0){
	slip_radio_cmd_handler(recv_buf,plength);
	printf("\n receive length %d \n",plength);
}
return 0;
}
/*-----------------------------------------------------------*/
static void
slip_input_callback(uint8_t* buffer,uint16_t blen)
{
slip_radio_cmd_handler(buffer,blen);
}
/*------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS(sniffer_process, "Sniffer process");
AUTOSTART_PROCESSES(&sniffer_process,&slip_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sniffer_process, ev, data)
{
static struct etimer et;
write_pos=0;
read_pos=0;
// use slip
//uart1_set_input(write_to_buf); //set the callback function
//uart1_set_input(slip_input_byte);
//slip_arch_init(UART1_BAUD2UBR(115200ul));
//slip_set_raw_input_callback(slip_input_callback);
uart1_set_input(write_to_buf);
//read_pos=0;
//write_pos=0;

/*use sensniff*/
//length_pkt=0;
//uart1_set_input(rx_byte);

  PROCESS_BEGIN();
  NETSTACK_RADIO.on();
  NETSTACK_MAC.off(1);
  set_channel(CHANNEL);
/*
etimer_set(&et, CLOCK_SECOND * 10);

	while(1){
	PROCESS_WAIT_EVENT();
	    if (ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
//		 send_packet();
		slip_radio_cmd_handler(NULL,0);
//		 leds_toggle(LEDS_RED);
		etimer_reset(&et);
	}
	}*/
//#endif
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/

