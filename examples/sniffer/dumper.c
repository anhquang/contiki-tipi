/*
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
 * No application for this example, this file is a place holder
 */
#include <stdio.h>
#include <string.h>
#include <pcap.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>

#define UDP_SOCKET	1
#define PCAP_FILE	2

#define ADAPTER		PCAP_FILE
//#define ADAPTER		UDP_SOCKET


#define UDP_PORT		2000

#define PCAPFILE_FILENAME	"output.dump"
#define PCAPFILE_BUFF_SIZE	2048

//#define PCAPFILE_LINK_TYPE	DLT_EN10MB
//#define PCAPFILE_LINK_TYPE	DLT_RAW
#define PCAPFILE_LINK_TYPE	DLT_IEEE802_15_4

static FILE* sPcapFp = NULL;
static int sSocket = -1;
static struct sockaddr_in sPeerEndpoint = { AF_INET, 0,	{0}};

typedef uint8_t u8_t;

/** timestamp format for frames in tcpdump files (taken from libpcap) */
struct pcap_timeval {
    bpf_int32 tv_sec;           /** seconds */
    bpf_int32 tv_usec;          /** microseconds */
};

/** packet header for tcpdump files (taken from libpcap) */
struct pcap_sf_pkthdr {
    struct pcap_timeval ts;     /** time stamp */
    bpf_u_int32 caplen;         /** length of portion present */
    bpf_u_int32 len;            /** length this packet (off wire) */
};


static void pcapfile_init()
{
	struct pcap_file_header hdr;
	
	sPcapFp = fopen (PCAPFILE_FILENAME, "w");
	if (sPcapFp == NULL) {
		printf ("Error: cannot write \"%s\"", PCAPFILE_FILENAME);
		exit(1);
	}

	hdr.magic = 0xa1b2c3d4;
	hdr.version_major = 2; // PCAP_VERSION_MAJOR
	hdr.version_minor = 4; // PCAP_VERSION_MINOR

	hdr.thiszone = 0; // time offset relative to GMT
	hdr.snaplen = 2000;
	hdr.sigfigs = 0;
	hdr.linktype = PCAPFILE_LINK_TYPE;
	
	fwrite ((char *)&hdr, sizeof(hdr), 1, sPcapFp);
	fflush (sPcapFp);
}

static u8_t pcapfile_write (const char* payload, int len)
{
	printf ("pcapfile_write\n");

	struct pcap_sf_pkthdr sf_hdr;
	struct timeval ts;

	const int offset = 0;

	gettimeofday (&ts, NULL);
	sf_hdr.ts.tv_sec  = ts.tv_sec;
	sf_hdr.ts.tv_usec = ts.tv_usec;
	sf_hdr.caplen     = len - offset;
	sf_hdr.len        = len - offset;

	fwrite (&sf_hdr, sizeof(sf_hdr), 1, sPcapFp);
	fwrite (payload + offset, len - offset, 1, sPcapFp);
	fflush (sPcapFp);
	
	return 0; // TODO
}

static void
socket_init()
{
	sSocket = socket (AF_INET, SOCK_DGRAM, 0);
	if (sSocket == -1) {
		perror ("Error: cannot create socket");
		exit(1);
	}

	struct sockaddr_in addr =
	{
		AF_INET,
		htons (UDP_PORT),
		{0}

	};

	int result = bind (sSocket, (struct sockaddr *) &addr, sizeof (addr));
	if (result != 0) {
		perror ("Error: cannot bind socket");
		exit(1);
	}

	// wait for the first packet
	char buf[2];
	puts ("Socket initialised, waiting for the initialisation packet");
	socklen_t addrlen = sizeof (sPeerEndpoint);
	result = recvfrom (sSocket, buf, 2, 0, (struct sockaddr *) &sPeerEndpoint, &addrlen);
	if (result == -1) {
		perror ("Error");
		exit (1);
	}
	if (result > 0) {
		fprintf (stderr, "Warning: the initialisation packet is not empty --> ignored\n");
	}
}

static u8_t
socket_write (const char* payload, int len)
{
	if (sPeerEndpoint.sin_port == 0)
	{
		fputs ("Warning: ignored frame (not sent over the udp socket because the remote endpoint is not yet known)\n", stderr);
		return 0;
	}

	if (-1 == sendto (sSocket, payload, len, 0, (struct sockaddr *) &sPeerEndpoint, sizeof (sPeerEndpoint)))
	{
		perror ("Warning: cannot send frame over udp port");
		return 0;
	}

	return 1;
}


#define SLIP_END	0300
#define SLIP_ESC	0333
#define SLIP_ESC_END	0334
#define SLIP_ESC_ESC	0335

#define ST_NONE		0
#define ST_FRAME	1
#define ST_ESC		2
#define ST_OVERRUN	3

int main (int argc, char* argv[])
{
  unsigned char buff[2048];
  unsigned char frame[2048];
  unsigned char *p, *bp, *fp, *end, *c;
  unsigned const char* frame_end = frame + 2048;
  int nb;
  int state;

#if ADAPTER == UDP_SOCKET
  socket_init();
#else
  pcapfile_init();
#endif

  fp = NULL;
  state = ST_NONE;
  for (;;) {
	nb = read (STDIN_FILENO, buff, 2048);

	bp = buff;
	end = buff+nb;
	
	for (bp = buff ; bp != end ; bp = p)
	{
		printf ("%2x\n", *bp);
		p = bp;

		switch (state)
		{
		case ST_NONE:
			// not in a frame
			// --> find the flag
			for (; p!=end ; p++)
			{
//				printf ("%02x ", *p);
				if (*p == SLIP_END)
					break;
			}
			write (STDOUT_FILENO, bp, p-bp);

			if (p == end)
				break;

			state = ST_FRAME;
			fp = frame;
			p++;

			break;

		case ST_FRAME:
			// in a frame
			for (; (p != end) ; p++)
			{
				if (*p == SLIP_END)
				{
					write (STDOUT_FILENO, "[FRAME]", 7);
					state = ST_NONE;
					break;

				} else if (*p == SLIP_ESC) {
					write (STDOUT_FILENO, "[TODO]", 6);
					state = ST_ESC;
					break;
				}
			}
			// write the content to the frame buffer
			nb = p-bp;
			if (fp+nb > frame_end)
			{
				write (STDOUT_FILENO, "[BUFFER OVERRUN]", 16);
				if (state != ST_NONE)
					state = ST_OVERRUN;
			} else {
				memcpy (fp, bp, nb);
				fp += nb;

				if (state == ST_NONE)
				{
					// report the frame
					
					printf ("\nNew frame (%ld bytes):", fp-frame);
					for (c = frame ; c!=fp ; c++)
					{
						printf (" %02X", *c);
					}
					puts("");
					fflush (stdout);
					
					#if ADAPTER == UDP_SOCKET
					  socket_write (frame, fp-frame);
					#else
					  pcapfile_write (frame, fp-frame);
					#endif
				}
			}

			if (p == end)
				break;

			p++;
			break;

		case ST_ESC:
			// in a frame
			
			if (fp == frame_end)
			{
				write (STDOUT_FILENO, "[BUFFER OVERRUN]", 16);
				state = ST_OVERRUN;
				p++;
				break;
			}

			switch (*p)
			{
			case SLIP_ESC_ESC:
				*fp++ = SLIP_ESC;
				state = ST_FRAME;
				break;
			case SLIP_ESC_END:
				*fp++ = SLIP_END;
				state = ST_FRAME;
				break;
			default:
				write (STDOUT_FILENO, "[BAD ESCAPE SEQ]", 16);
				state = ST_OVERRUN;
				break;
			}

			p++;
			break;

		case ST_OVERRUN:
			for (; p!=end ; p++)
			{
				if (*p == SLIP_END) {
					state = ST_NONE;
					p++;
					break;
				}
			}
			break;
		}

	end_of_buffer:
		;
	}
  }

}

