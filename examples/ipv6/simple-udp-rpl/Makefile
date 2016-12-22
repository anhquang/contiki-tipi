all: broadcast-example unicast-sender unicast-receiver
APPS=servreg-hack
CONTIKI=../../..

WITH_UIP6=1
UIP_CONF_IPV6=1
CHANNEL=26
CFLAGS+= -DUIP_CONF_IPV6_RPL
CFLAGS+= -DCHANNEL=$(CHANNEL)
include $(CONTIKI)/Makefile.include
