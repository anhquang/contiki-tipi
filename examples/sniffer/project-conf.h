/*
 * Copyright (c) 2012, Scuola Superiore Sant'Anna
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
 */

/**
 * \file
 *         Project specific configuration defines for the android sniffer 
 *         example for the Tmote Sky platform
 *
 * \author
 *         Daniele Alessandrelli - <d.alessandrelli@sssup.it>
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#undef CC2420_CONF_AUTOACK
#define CC2420_CONF_AUTOACK 0
#define CC2420_SNIFFER 1
#undef NETSTACK_MAC 
#define NETSTACK_MAC nullmac_driver

#undef NETSTACK_FRAMER
#define NETSTACK_FRAMER   framer_nullmac

#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC      sniffer_rdc_driver

#undef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 16

#define CMD_CONF_HANDLERS slip_radio_cmd_handler
#define CMD_CONF_OUTPUT slip_cmd_output
#endif /* PROJECT_CONF_H_ */