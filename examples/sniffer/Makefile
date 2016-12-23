APPS += slip-cmd
DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
PROJECTDIRS = dev
PROJECT_SOURCEFILES += sniffer-rdc.c cc2420.c slip_uart1.c slip.c queue.c
CONTIKI_PROJECT = sniffer sniff-fwd test_slip fwd sniff-fwd-magic
CHANNEL=26
NODEID=0
AUTOSTART=0
CFLAGS += -DCHANNEL=$(CHANNEL)
CFLAGS += -DNODEID=$(NODEID)
CFLAGS += -DAUTOSEND=$(AUTOSEND)
all: $(CONTIKI_PROJECT) 
CONTIKI = ../..
include $(CONTIKI)/Makefile.include
