/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Hoernchen <la@tfc-server.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifndef _WIN32
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <fcntl.h>
#else
#include <WinSock2.h>
#include <getopt.h>
#endif

#include <pthread.h>

#include "hackrf.h"

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#define sleep(x) Sleep(x)

#else
#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1
#endif


enum Device {
    // These are compatible with rtl_tcp
    UNKNOWN = 0,
    RTLSDR_E4000,
    RTLSDR_FC0012,
    RTLSDR_FC0013,
    RTLSDR_FC2580,
    RTLSDR_R820T,       // Used by rsp_tcp
    RTLSDR_R828D,
    // SDRangel extensions that correspond to sample source devices
    AIRSPY = 0x80,
    AIRSPY_HF,
    AUDIO_INPUT,
    BLADE_RF1,
    BLADE_RF2,
    FCD_PRO,
    FCD_PRO_PLUS,
    FILE_INPUT,
    HACK_RF,
    KIWI_SDR,
    LIME_SDR,
    LOCAL_INPUT,
    PERSEUS,
    PLUTO_SDR,
    REMOTE_INPUT,
    REMOTE_TCP_INPUT,
    SDRPLAY_1,
    SDRPLAY_V3_RSP1,
    SDRPLAY_V3_RSP1A,
    SDRPLAY_V3_RSP2,
    SDRPLAY_V3_RSPDUO,
    SDRPLAY_V3_RSPDX,
    SIGMF_FILE_INPUT,
    SOAPY_SDR,
    TEST_SOURCE,
    USRP,
    XTRX
};

enum Command {
    // These are compatbile with osmocom rtl_tcp: https://github.com/osmocom/rtl-sdr/blob/master/src/rtl_tcp.c
    // and Android https://github.com/signalwareltd/rtl_tcp_andro-/blob/master/rtlsdr/src/main/cpp/src/tcp_commands.h
    setCenterFrequency = 0x1,           // rtlsdr_set_center_freq
    setSampleRate = 0x2,                // rtlsdr_set_sample_rate
    setTunerGainMode = 0x3,             // rtlsdr_set_tuner_gain_mode
    setTunerGain = 0x4,                 // rtlsdr_set_tuner_gain
    setFrequencyCorrection = 0x5,       // rtlsdr_set_freq_correction
    setTunerIFGain = 0x6,               // rtlsdr_set_tuner_if_gain - Used by SDRangel to set LNA/VGA/IF gain individually
    setTestMode = 0x7,                  // Not supported by SDRangel
    setAGCMode = 0x8,                   // rtlsdr_set_agc_mode
    setDirectSampling = 0x9,            // rtlsdr_set_direct_sampling
    setOffsetTuning = 0xa,
    setXtalFrequency = 0xb,             // Not supported by SDRangel
    setXtalFrequency2 = 0xc,            // Not supported by SDRangel
    setGainByIndex = 0xd,               // Not supported by SDRangel
    setBiasTee = 0xe,                   // rtlsdr_set_bias_tee (Not supported on Android)
    // These extensions are from rsp_tcp: https://github.com/SDRplay/RSPTCPServer/blob/master/rsp_tcp_api.h
    rspSetAntenna = 0x1f,
    rspSetLNAState = 0x20,
    rspSetIfGainR = 0x21,
    rspSetAGC = 0x22,
    rspSetAGCSetPoint = 0x23,
    rspSetNotch = 0x24,
    rspSetBiasT = 0x25,
    rspSetRefOut = 0x26,
     // These extensions are from librtlsdr rtl_tcp: https://github.com/librtlsdr/librtlsdr/blob/development/include/rtl_tcp.h
    setTunerBandwidth = 0x40,
    // Android extensions https://github.com/signalwareltd/rtl_tcp_andro-/blob/master/rtlsdr/src/main/cpp/src/tcp_commands.h
    androidExit = 0x7e,
    androidGainByPercentage = 0x7f,
    androidEnable16BitSigned = 0x80,    // SDRplay, not RTL SDR
    // These are SDRangel extensions
    setDCOffsetRemoval = 0xc0,
    setIQCorrection = 0xc1,
    setDecimation = 0xc2,               // Device to baseband decimation
    setChannelSampleRate = 0xc3,
    setChannelFreqOffset = 0xc4,
    setChannelGain = 0xc5,
    setSampleBitDepth = 0xc6,           // Bit depth for samples sent over network
    //setAntenna?
    //setLOOffset?
};


static SOCKET s;
static pthread_t tcp_worker_thread;
static pthread_t command_thread;
static pthread_cond_t exit_cond;
static pthread_mutex_t exit_cond_lock;
static volatile int dead[2] = {0, 0};

static pthread_mutex_t ll_mutex;
static pthread_cond_t cond;

struct llist {
	uint8_t *data;
	size_t len;
	struct llist *next;
};

//; -> 0x53445241 00000088 00000000 19ED92C0 00 00 00 00 00 00 00 00 00 24 9F 00 00 00 00 00 00 A0 00 A0 00 00 00 00 00 1A B3 F0 00 00 00 00 00 00 00 00 00 00 BB 80 00 00 00 08 00 00 00 00
typedef struct { /* structure size must be multiple of 2 bytes */
	char magic[4];
	uint32_t tuner_type;
	uint32_t unknown_0;
	uint32_t unknown_1;
	uint8_t other_stuff[48];
} dongle_info_t;

static hackrf_device *dev = NULL;

int global_numq = 0;
static struct llist *ll_buffers = 0;
int llbuf_num=500;
static int do_exit = 0;

void usage(void)
{
	printf("hackrf_tcp, an I/Q spectrum server for HackRF, similar to rtl_tcp\n\n"
		"Usage:\t[-a listen address]\n"
		"\t[-p listen port (default: 1234)]\n"
		"\t[-f frequency to tune to [Hz]]\n"
		"\t[-g gain (default: 0 for auto)]\n"
		"\t[-s samplerate in Hz (default: 2048000 Hz)]\n"
		"\t[-d device index (default: 0)]\n");
	exit(1);
}

#ifdef _WIN32
int gettimeofday(struct timeval *tv, void* ignored)
{
	FILETIME ft;
	unsigned __int64 tmp = 0;
	if (NULL != tv) {
		GetSystemTimeAsFileTime(&ft);
		tmp |= ft.dwHighDateTime;
		tmp <<= 32;
		tmp |= ft.dwLowDateTime;
		tmp /= 10;
		tmp -= 11644473600000000Ui64;
		tv->tv_sec = (long)(tmp / 1000000UL);
		tv->tv_usec = (long)(tmp % 1000000UL);
	}
	return 0;
}

BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stderr, "Signal caught, exiting!\n");
		//rtlsdr_cancel_async(dev);
		hackrf_stop_rx(dev);
		hackrf_close(dev);
		sleep(1.2);
		hackrf_init();
		hackrf_open(&dev);
		do_exit = 1;
		return TRUE;
	}
	return FALSE;
}
#else
static void sighandler(int signum)
{
	fprintf(stderr, "Signal caught, exiting!\n");
	if (!do_exit) {
		//rtlsdr_cancel_async(dev);
		hackrf_stop_rx(dev);
		hackrf_close(dev);
		sleep(1.2);
		hackrf_init();
		hackrf_open(&dev);
		do_exit = 1;
	}
}
#endif

int hackrf_callback(hackrf_transfer *transfer)
{
	//unsigned char *buf, uint32_t len, void *ctx)
	if(!do_exit) {
		struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));
		rpt->data = (uint8_t*)malloc(transfer->buffer_length);

		// sdrangel expects unsigned samples with a 0x80 offset
		for(uint32_t i = 0; i < transfer->buffer_length; i++) {
			rpt->data[i] = transfer->buffer[i] + 0x80;
		}
		//memcpy(rpt->data, transfer->buffer, transfer->buffer_length);
		rpt->len = transfer->buffer_length;
		rpt->next = NULL;

		pthread_mutex_lock(&ll_mutex);

		if (ll_buffers == NULL) {
			ll_buffers = rpt;
		} else {
			struct llist *cur = ll_buffers;
			int num_queued = 0;

			while (cur->next != NULL) {
				cur = cur->next;
				num_queued++;
			}

			if(llbuf_num && llbuf_num == num_queued-2){
				struct llist *curelem;

				free(ll_buffers->data);
				curelem = ll_buffers->next;
				free(ll_buffers);
				ll_buffers = curelem;
			}

			cur->next = rpt;

			if (num_queued > global_numq)
				printf("ll+, now %d\n", num_queued);
			else if (num_queued < global_numq)
				printf("ll-, now %d\n", num_queued);

			global_numq = num_queued;
		}
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&ll_mutex);
	}
	return 0;
}

static void *tcp_worker(void *arg)
{
	struct llist *curelem,*prev;
	long bytesleft, bytessent, index;
	struct timeval tv= {1,0};
	struct timespec ts;
	struct timeval tp;
	fd_set writefds;
	int r = 0;

	while(1) {
		if(do_exit)
			pthread_exit(0);

		pthread_mutex_lock(&ll_mutex);
		gettimeofday(&tp, NULL);
		ts.tv_sec  = tp.tv_sec+5;
		ts.tv_nsec = tp.tv_usec * 1000;
		r = pthread_cond_timedwait(&cond, &ll_mutex, &ts);
		if(r == ETIMEDOUT) {
			pthread_mutex_unlock(&ll_mutex);
			printf("worker cond timeout\n");
			sighandler(0);
			dead[0]=1;
			pthread_exit(NULL);
		}

		curelem = ll_buffers;
		ll_buffers = 0;
		pthread_mutex_unlock(&ll_mutex);

		while(curelem != 0) {
			bytesleft = curelem->len;
			index = 0;
			bytessent = 0;
			while(bytesleft > 0) {
				FD_ZERO(&writefds);
				FD_SET(s, &writefds);
				tv.tv_sec = 1;
				tv.tv_usec = 0;
				r = select(s+1, NULL, &writefds, NULL, &tv);
				if(r) {
					bytessent = send(s,  &curelem->data[index], bytesleft, 0);
					if (bytessent == SOCKET_ERROR) {
			                        perror("worker socket error");
						sighandler(0);
						dead[0]=1;
						pthread_exit(NULL);
					} else if (do_exit) {
						printf("do_exit\n");
						dead[0]=1;
						pthread_exit(NULL);
					} else {
						bytesleft -= bytessent;
						index += bytessent;
					}
				} else if(do_exit) {
						printf("worker socket bye\n");
						sighandler(0);
						dead[0]=1;
						pthread_exit(NULL);
				}
			}
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}
	}
}

static int set_tuner_amp(hackrf_device *_dev, unsigned int p)
{
	int res = 0;
	res = hackrf_set_amp_enable(_dev, p);
        return res;
}

static int set_tuner_gain(hackrf_device *_dev, int g, unsigned int p)
{
	int res = 0;
	if (g >= 1) {
		res = hackrf_set_lna_gain(_dev, p);
	} else {
		res = hackrf_set_vga_gain(_dev, p);
	}
        return res;
}

/*
static int set_tuner_if(hackrf_device *_dev, unsigned int p)
{
	int res = 0;
	// hackrf_set_if_freq does not exist in vanilla libhackrf?
	res = hackrf_set_if_freq(_dev, p);
        return res;
}
*/


#ifdef _WIN32
#define __attribute__(x)
#pragma pack(push, 1)
#endif
struct command{
	unsigned char cmd;
	unsigned int param;
}__attribute__((packed));
#ifdef _WIN32
#pragma pack(pop)
#endif
static void *command_worker(void *arg)
{
	int left, received;
	fd_set readfds;
	struct command cmd={0, 0};
	struct timeval tv= {1, 0};
	int r = 0;
	uint64_t freq;

	while(1) {
		left=sizeof(cmd);
		while(left >0) {
			FD_ZERO(&readfds);
			FD_SET(s, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(s+1, &readfds, NULL, NULL, &tv);
			if(r) {
				received = recv(s, (char*)&cmd+(sizeof(cmd)-left), left, 0);
				if(received == SOCKET_ERROR){
					perror("comm recv socket error");
					sighandler(0);
					dead[1]=1;
					pthread_exit(NULL);
				} else if(do_exit){
					printf("do exit\n");
					dead[1]=1;
					pthread_exit(NULL);
				} else {
					left -= received;
				}
			} else if(do_exit) {
				printf("comm recv bye\n");
				sighandler(0);
				dead[1] = 1;
				pthread_exit(NULL);
			}
		}


		uint32_t param_decoded = ntohl(cmd.param);
		switch(cmd.cmd) {
		case 0x01:
			// Since cmd.param is 32 bits, this only allows
			// setting frequencies up to about 4.3 GHz.
			freq = param_decoded;
			printf("set freq %lu\n", freq);
			hackrf_set_freq(dev, freq);
			break;
		//case 0x02:
		case 0xc3:
			printf("set sample rate %d\n", param_decoded);
			hackrf_set_sample_rate(dev, param_decoded);
			break;
		case 0x06:
		case 0xb1:
			param_decoded = (param_decoded & 0xFFFF) / 10;
    		printf("set tuner vga gain %d\n", param_decoded);
			set_tuner_gain(dev, 0, param_decoded);
			break;
		case 0x04:
		case 0xb2:
			param_decoded = param_decoded / 10;
			printf("set tuner lna gain %d\n", param_decoded);
			set_tuner_gain(dev, 1, param_decoded);
			break;
		case 0xb4:
			// Use for setting frequencies above 4.3 GHz.
			freq = ((uint64_t) param_decoded) + 0x100000000;
			printf("set freq %lu\n", freq);
			hackrf_set_freq(dev, freq);
			break;
		case 0x40: //setTunerBandwidth
			hackrf_set_baseband_filter_bandwidth(dev, param_decoded);
			printf("set BBW %lu\n", param_decoded);
		    break;
		case 0x0e: //setBiasTee
		case 0x25: //rspSetBiasT
			hackrf_set_antenna_enable(dev, (uint8_t) param_decoded);
			printf("set BiasT %lu\n", param_decoded);
			break;
		default:
			printf("[ignored] command %x value %d\n", cmd.cmd, param_decoded);
			break;
		}
		cmd.cmd = 0xff;
	}
}
int main(int argc, char **argv)
{
	int r, opt;
	char* addr = "0.0.0.0";
	int port = 1234;
	long frequency = 443500000;
	int samp_rate = 2800000;
	uint8_t tuner_gain_enable = 0;
	struct sockaddr_in local, remote;
	uint32_t dev_index = 0;
	int gain = 0;
	struct llist *curelem,*prev;
	uint8_t board_id = BOARD_ID_INVALID;
	char version[255 + 1];
	pthread_attr_t attr;
	void *status;
	struct timeval tv = {1,0};
	struct linger ling = {1,0};
	SOCKET listensocket;
	socklen_t rlen;
	fd_set readfds;
	dongle_info_t dongle_info;
#ifdef _WIN32
	WSADATA wsd;
	u_long blockmode = 1;
	int i = WSAStartup(MAKEWORD(2,2), &wsd);
#else
	struct sigaction sigact, sigign;
#endif

	while ((opt = getopt(argc, argv, "a:p:f:g:s:t:n:d:x")) != -1) {
		switch (opt) {
		case 'd':
			dev_index = atoi(optarg);
			break;
		case 'f':
			frequency = (long)atof(optarg);
			break;
		case 'g':
			gain = (int)(atof(optarg) * 10); /* tenths of a dB */
			break;
		case 's':
			samp_rate = (uint32_t)atof(optarg);
			break;
		case 'a':
			addr = optarg;
			break;
		case 'p':
			port = atoi(optarg);
			break;
	    case 't':
			tuner_gain_enable = 1;
			break;
		default:
			usage();
			break;
		}
	}

	if (argc < optind)
		usage();

	//device_count = rtlsdr_get_device_count();

//	if (!device_count) {
//		fprintf(stderr, "No supported devices found.\n");
//		exit(1);
//	}

//	printf("Found %d device(s).\n", device_count);
	hackrf_init();
	hackrf_open(&dev);
	if (NULL == dev) {
	fprintf(stderr, "Failed to open hackrf device #%d.\n", dev_index);
		exit(1);
	}
	r = hackrf_board_id_read(dev,&board_id);
        if (r != HACKRF_SUCCESS) {
                fprintf(stderr, "hackrf_board_id_read() failed: %s (%d)\n",
                                hackrf_error_name(r), r);
                exit(1);
        }

        r = hackrf_version_string_read(dev, &version[0], 255);
        if (r != HACKRF_SUCCESS) {
                fprintf(stderr, "hackrf_version_string_read() failed: %s (%d)\n",
                                hackrf_error_name(r), r);
                exit(1);
        }

	printf("Using HackRF %s with firmware %s\n", hackrf_board_id_name(board_id), version);
#ifndef _WIN32
	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigign.sa_handler = SIG_IGN;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigign, NULL);
#else
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#endif
	/* Set the sample rate */
	r = hackrf_set_sample_rate(dev, samp_rate);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");

	/* Set the frequency */
	r = hackrf_set_freq(dev, frequency);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set center freq.\n");
	else
		fprintf(stderr, "Tuned to %ld Hz.\n", frequency);

	if (0 == gain) {
		 /* Enable automatic gain */
		hackrf_set_lna_gain(dev,24);
		hackrf_set_vga_gain(dev,24);
		hackrf_set_amp_enable(dev,1);
	} else {
		/* Enable manual gain */
		hackrf_set_amp_enable(dev,1);
		hackrf_set_lna_gain(dev,gain);
		hackrf_set_vga_gain(dev,gain);
	}

	// sdrangel has no control for this AFAIK
	if(tuner_gain_enable) {
		set_tuner_amp(dev, 1);
	}

	/* Reset endpoint before we start reading from it (mandatory) */
//	r = rtlsdr_reset_buffer(dev);

	pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_mutex_init(&ll_mutex, NULL);
	pthread_mutex_init(&exit_cond_lock, NULL);
	pthread_cond_init(&cond, NULL);
	pthread_cond_init(&exit_cond, NULL);

	memset(&local,0,sizeof(local));
	local.sin_family = AF_INET;
	local.sin_port = htons(port);
	local.sin_addr.s_addr = inet_addr(addr);

	listensocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	r = 1;
	setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
	setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));
	bind(listensocket,(struct sockaddr *)&local,sizeof(local));

	#ifdef _WIN32
	ioctlsocket(listensocket, FIONBIO, &blockmode);
	#else
	r = fcntl(listensocket, F_GETFL, 0);
	r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);
	#endif

	while(1) {
		printf("listening...\n");
		printf("Use the device argument 'hackrf_tcp=%s:%d' in OsmoSDR "
		       "(gr-osmosdr) source\n"
		       "to receive samples in GRC and control "
		       "hackrf_tcp parameters (frequency, gain, ...).\n",
		       addr, port);
		listen(listensocket,1);

		while(1) {
			FD_ZERO(&readfds);
			FD_SET(listensocket, &readfds);
			tv.tv_sec = 1;
			tv.tv_usec = 0;
			r = select(listensocket+1, &readfds, NULL, NULL, &tv);
			if(do_exit) {
				goto out;
			} else if(r) {
				rlen = sizeof(remote);
				s = accept(listensocket,(struct sockaddr *)&remote, &rlen);
				break;
			}
		}

		setsockopt(s, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

		printf("client accepted!\n");

		static const uint8_t other_stuff_template[48] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x9F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0xB3, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00};
		memset(&dongle_info, 0, sizeof(dongle_info));
		memcpy(&dongle_info.magic, "SDRA", 4);
		dongle_info.tuner_type = htonl(HACK_RF);
		dongle_info.unknown_0 = 0;
		dongle_info.unknown_1 = 0xC092ED19;
		memcpy(dongle_info.other_stuff, other_stuff_template, sizeof(other_stuff_template));

		r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
		if (sizeof(dongle_info) != r)
			printf("failed to send dongle information\n");

		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
		r = pthread_create(&command_thread, &attr, command_worker, NULL);
		pthread_attr_destroy(&attr);

//		r = rtlsdr_read_async(dev, rtlsdr_callback, NULL, buf_num, 0);
		hackrf_start_rx(dev, hackrf_callback, NULL);

		if(!dead[0])
			pthread_join(tcp_worker_thread, &status);
		dead[0]=0;

		if(!dead[1])
			pthread_join(command_thread, &status);
		dead[1]=0;

		closesocket(s);

		printf("all threads dead..\n");
		curelem = ll_buffers;
		ll_buffers = 0;

		while(curelem != 0) {
			prev = curelem;
			curelem = curelem->next;
			free(prev->data);
			free(prev);
		}

		do_exit = 0;
		global_numq = 0;
	}

out:
	hackrf_close(dev);
	closesocket(listensocket);
	closesocket(s);
	#ifdef _WIN32
	WSACleanup();
	#endif
	printf("bye!\n");
	return r >= 0 ? r : -r;
}