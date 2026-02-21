/*
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 * Copyright 2014-2015 Benjamin Vernoux <bvernoux@airspy.com>
 * Copyright 2018 Andrea Montefusco IW0HDV <andrew@montefusco.com>
 *
 * This file is part of AirSpyHF (based on HackRF project).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <getopt.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>

#include <airspyhf.h>
#include <zmq.h>
#include <tagtracker_wireformat/zmq_iq_packet.h>

#if !defined __cplusplus
#if __STDC_VERSION__ < 202311L
#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif
#endif
#endif

#ifdef _WIN32
#include <windows.h>

#ifdef _MSC_VER

#ifdef _WIN64
typedef int64_t ssize_t;
#else
typedef int32_t ssize_t;
#endif

#define strtoull _strtoui64
#define snprintf _snprintf

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

#endif
#endif

#if defined(__GNUC__)
#include <unistd.h>
#include <sys/time.h>
#endif

#include <signal.h>

#if defined _WIN32
	#define sleep(a) Sleep( (a*1000) )
#endif

#define FREQ_ONE_MHZ (1000000ul)
#define FREQ_ONE_MHZ_U64 (1000000ull)
#define FD_BUFFER_SIZE (16*1024)
#define DEFAULT_FREQ_HZ (7100000ul) /* 7.1 MHz */
#define SAMPLES_TO_XFER_MAX_U64 (0x8000000000000000ull) /* Max value */

/* WAVE or RIFF WAVE file format containing data for AirSpy compatible with SDR# Wav IQ file */
typedef struct
{
	char groupID[4]; /* 'RIFF' */
	uint32_t size; /* File size + 8bytes */
	char riffType[4]; /* 'WAVE'*/
} t_WAVRIFF_hdr;

#define FormatID "fmt "   /* chunkID for Format Chunk. NOTE: There is a space at the end of this ID. */

typedef struct {
	char chunkID[4]; /* 'fmt ' */
	uint32_t chunkSize; /* 16 fixed */

	uint16_t wFormatTag; /* 1=PCM8/16, 3=Float32 */
	uint16_t wChannels;
	uint32_t dwSamplesPerSec; /* Freq Hz sampling */
	uint32_t dwAvgBytesPerSec; /* Freq Hz sampling x 2 */
	uint16_t wBlockAlign;
	uint16_t wBitsPerSample;
} t_FormatChunk;

typedef struct
{
	char chunkID[4]; /* 'data' */
	uint32_t chunkSize; /* Size of data in bytes */
	/* For IQ samples I(16 or 32bits) then Q(16 or 32bits), I, Q ... */
} t_DataChunk;

typedef struct
{
	t_WAVRIFF_hdr hdr;
	t_FormatChunk fmt_chunk;
	t_DataChunk data_chunk;
} t_wav_file_hdr;

t_wav_file_hdr wave_file_hdr =
{
	/* t_WAVRIFF_hdr */
	{
		{ 'R', 'I', 'F', 'F' }, /* groupID */
		0, /* size to update later */
		{ 'W', 'A', 'V', 'E' }
	},
	/* t_FormatChunk */
	{
		{ 'f', 'm', 't', ' ' }, /* char		chunkID[4];  */
		16, /* uint32_t chunkSize; */
		0, /* uint16_t wFormatTag; to update later */
		0, /* uint16_t wChannels; to update later */
		0, /* uint32_t dwSamplesPerSec; Freq Hz sampling to update later */
		0, /* uint32_t dwAvgBytesPerSec; to update later */
		0, /* uint16_t wBlockAlign; to update later */
		0, /* uint16_t wBitsPerSample; to update later  */
	},
	/* t_DataChunk */
	{
		{ 'd', 'a', 't', 'a' }, /* char chunkID[4]; */
		0, /* uint32_t	chunkSize; to update later */
	}
};

#define U64TOA_MAX_DIGIT (31)
typedef struct
{
	char data[U64TOA_MAX_DIGIT+1];
} t_u64toa;



// globals used both in callback and main

volatile bool do_exit = false;

FILE* fd = NULL;

bool verbose = false;
bool receive = false;
bool receive_wav = false;
bool limit_num_samples = false;
uint64_t samples_to_xfer = 0;
uint64_t bytes_to_xfer = 0;

struct timeval time_start;
struct timeval t_start;

bool got_first_packet = false;
float average_rate = 0.0f;
float global_average_rate = 0.0f;
uint32_t rate_samples = 0;
uint32_t buffer_count = 0;
uint32_t sample_count = 0;

bool use_zmq_output = true;
char zmq_host[128] = "127.0.0.1";
uint16_t zmq_port = 5555;
void* zmq_context = NULL;
void* zmq_pub_socket = NULL;
uint64_t zmq_sequence = 0;
uint32_t current_sample_rate = 0;


static float
TimevalDiff(const struct timeval *a, const struct timeval *b)
{
	return (a->tv_sec - b->tv_sec) + 1e-6f * (a->tv_usec - b->tv_usec);
}

static uint64_t get_time_us(void)
{
	struct timespec time_now;
	if (clock_gettime(CLOCK_MONOTONIC, &time_now) == 0) {
		return ((uint64_t)time_now.tv_sec * 1000000ull) + ((uint64_t)time_now.tv_nsec / 1000ull);
	}

	{
		struct timeval fallback_time;
		gettimeofday(&fallback_time, NULL);
		return ((uint64_t)fallback_time.tv_sec * 1000000ull) + (uint64_t)fallback_time.tv_usec;
	}
}

int parse_u64(char* s, uint64_t* const value) {
	uint_fast8_t base = 10;
	char* s_end;
	uint64_t u64_value;

	if( strlen(s) > 2 ) {
		if( s[0] == '0' ) {
			if( (s[1] == 'x') || (s[1] == 'X') ) {
				base = 16;
				s += 2;
			} else if( (s[1] == 'b') || (s[1] == 'B') ) {
				base = 2;
				s += 2;
			}
		}
	}

	s_end = s;
	u64_value = strtoull(s, &s_end, base);
	if( (s != s_end) && (*s_end == 0) ) {
		*value = u64_value;
		return AIRSPYHF_SUCCESS;
	} else {
		return AIRSPYHF_ERROR;
	}
}

int parse_u32(char* s, uint32_t* const value)
{
	uint_fast8_t base = 10;
	char* s_end;
	uint64_t ulong_value;

	if( strlen(s) > 2 ) {
		if( s[0] == '0' ) {
			if( (s[1] == 'x') || (s[1] == 'X') ) {
				base = 16;
				s += 2;
			} else if( (s[1] == 'b') || (s[1] == 'B') ) {
				base = 2;
				s += 2;
			}
		}
	}

	s_end = s;
	ulong_value = strtoul(s, &s_end, base);
	if( (s != s_end) && (*s_end == 0) ) {
		*value = (uint32_t)ulong_value;
		return AIRSPYHF_SUCCESS;
	} else {
		return AIRSPYHF_ERROR;
	}
}

static char *stringrev(char *str)
{
	char *p1, *p2;

	if(! str || ! *str)
		return str;

	for(p1 = str, p2 = str + strlen(str) - 1; p2 > p1; ++p1, --p2)
	{
		*p1 ^= *p2;
		*p2 ^= *p1;
		*p1 ^= *p2;
	}
	return str;
}

char* u64toa(uint64_t val, t_u64toa* str)
{
	#define BASE (10ull) /* Base10 by default */
	uint64_t sum;
	int pos;
	int digit;
	int max_len;
	char* res;

	sum = val;
	max_len = U64TOA_MAX_DIGIT;
	pos = 0;

	do {
		digit = (sum % BASE);
		str->data[pos] = digit + '0';
		pos++;

		sum /= BASE;
	} while( (sum>0) && (pos < max_len) );

	if( (pos == max_len) && (sum>0) )
		return NULL;

	str->data[pos] = '\0';
	res = stringrev(str->data);

	return res;
}

int rx_callback(airspyhf_transfer_t* transfer)
{
	uint32_t bytes_to_write;
	void* pt_rx_buffer;
	ssize_t bytes_written;
	ssize_t expected_bytes_written;
	struct timeval time_now;
	float time_difference, rate;

	if( fd || zmq_pub_socket ) {
		// #sample * float size * I+Q
		bytes_to_write = transfer->sample_count * 4 * 2;
		pt_rx_buffer = transfer->samples;

		gettimeofday(&time_now, NULL);

		if (!got_first_packet) {
			t_start = time_now;
			time_start = time_now;
			got_first_packet = true;
		} else {
			buffer_count++;
			sample_count += transfer->sample_count;
			if (buffer_count == 50) {
				time_difference = TimevalDiff(&time_now, &time_start);
				rate = (float) sample_count / time_difference;
				average_rate += 0.2f * (rate - average_rate);
				global_average_rate += average_rate;
				rate_samples++;
				time_start = time_now;
				sample_count = 0;
				buffer_count = 0;
			}
		}

		if (limit_num_samples) {
			if (bytes_to_write >= bytes_to_xfer) {
				bytes_to_write = (int)bytes_to_xfer;
			}
			bytes_to_xfer -= bytes_to_write;
		}

		if(pt_rx_buffer) {
			if (use_zmq_output) {
				ttwf_zmq_iq_packet_header_t packet_hdr;
				zmq_msg_t msg;
				size_t total_message_size;
				void* message_data;

				packet_hdr.magic = TTWF_ZMQ_IQ_MAGIC;
				packet_hdr.version = TTWF_ZMQ_IQ_VERSION;
				packet_hdr.header_size = TTWF_ZMQ_IQ_HEADER_SIZE;
				packet_hdr.sequence = zmq_sequence++;
				packet_hdr.timestamp_us = get_time_us();
				packet_hdr.sample_rate = current_sample_rate;
				packet_hdr.sample_count = bytes_to_write / (4 * 2);
				packet_hdr.payload_bytes = bytes_to_write;
				packet_hdr.flags = ((limit_num_samples == true) && (bytes_to_xfer == 0)) ? TTWF_ZMQ_IQ_FLAG_FINAL_CHUNK : 0u;

				total_message_size = TTWF_ZMQ_IQ_HEADER_SIZE + bytes_to_write;
				if (zmq_msg_init_size(&msg, total_message_size) != 0) {
					return -1;
				}

				message_data = zmq_msg_data(&msg);
				if (ttwf_encode_zmq_iq_header((uint8_t*)message_data,
						total_message_size,
						&packet_hdr) != TTWF_ZMQ_OK) {
					zmq_msg_close(&msg);
					return -1;
				}
				memcpy((uint8_t*)message_data + TTWF_ZMQ_IQ_HEADER_SIZE, pt_rx_buffer, bytes_to_write);

				bytes_written = zmq_msg_send(&msg, zmq_pub_socket, 0);
				expected_bytes_written = (ssize_t)total_message_size;
				zmq_msg_close(&msg);
			} else {
				bytes_written = fwrite(pt_rx_buffer, 1, bytes_to_write, fd);
				expected_bytes_written = (ssize_t)bytes_to_write;
			}
		} else {
			bytes_written = 0;
			expected_bytes_written = 0;
		}
		if  ( (bytes_written != expected_bytes_written) ||
			  ((limit_num_samples == true) && (bytes_to_xfer == 0))
			)
			return -1;
		else
			return 0;
	} else {
		return -1;
	}
}

static void usage(void)
{
	fprintf(stderr,
	"airspyhf_zeromq_rx\n"
	"Usage:\n"

	"\t-r <filename>\t\tReceive data into the file;\n"
	"\t\t\t\tstdout emits values on standard output\n"
	"\t-Z\t\t\tPublish IQ blocks over ZeroMQ (PUB)\n"
	"\t-I <ip_or_host>\t\tZeroMQ bind host/IP (default: 127.0.0.1)\n"
	"\t-P <port>\t\tZeroMQ bind port (default: 5555)\n"
	"\t\t\t\tPacket format: [header | IQ payload], header has sequence and timestamp\n"

	"\t-s <serial number>\tOpen device with specified 64bits serial number\n"

	"\t-f <frequency>\t\tSet frequency in MHz between 9 kHz - 31 MHz or 60 - 260 MHz\n"
	"\t-a <sample_rate>\tSet sample rate, at the moment only 768 kS/s supported\n"

	"\t-n <#samples>\t\tNumber of samples to transfer (default is unlimited)\n"

	"\t-d\t\t\tVerbose mode\n"

	"\t-w\t\t\tReceive data into file with WAV header and automatic name\n"
	"\t\t\t\tThis is for SDR# compatibility and may not work with other software\n"
	"\t\t\t\tIt works even with HDSDR\n"

	"\t-g on|off\t\tHF AGC on / off\n"
	"\t-l high|low\t\tHF AGC threshold high / low (when AGC On)\n"
	"\t-t <value>\t\tHF attenuator value 0..8 (each step increases 6 dB the attenuation)\n"
	"\t-m on|off\t\ton to activate LNA (preamplifier): +6 dB gain - compensated in digital.\n"

	"\t-z\t\t\tDo not attempt to use manual AGC/LNA commands\n"
	"\t\t\t\t(useful in order to avoid errors with old firmware)\n"

	);
}


#ifdef _MSC_VER
BOOL WINAPI
sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stdout, "Caught signal %d\n", signum);
		do_exit = true;
		return TRUE;
	}
	return FALSE;
}
#else
void sigint_callback_handler(int signum)
{
	fprintf(stdout, "Caught signal %d\n", signum);
	do_exit = true;
}
#endif


int main(int argc, char** argv)
{
	int opt;
	t_u64toa ascii_u64_data1;
	t_u64toa ascii_u64_data2;
	const char* path = 0;
	int result;

	struct timeval t_end;
	uint32_t file_pos;
	uint32_t wav_sample_per_sec;
	uint32_t nsrates;
	uint32_t *supported_samplerates;
	uint32_t sample_rate_u32 = 768000;
	uint32_t zmq_port_u32 = 0;

	struct airspyhf_device* device = 0;

	bool freq = false;
	uint32_t freq_hz = 0;
	bool sample_rate = true;
	uint32_t sample_rate_val = 0;
	bool serial_number = false;
	uint64_t serial_number_val;

	bool hf_agc = true;
	bool hf_agc_threshold = false; // false = low, true = high
	unsigned hf_att_val = 0;
	bool hf_lna = false; // false = off, true = on

	bool do_not_use_manual_commands = false;

	while( (opt = getopt(argc, argv, "r:ws:f:a:n:g:l:t:m:dhzZI:P:")) != EOF )
	{
		result = AIRSPYHF_SUCCESS;
		switch( opt )
		{
			case 'r':
				receive = true;
				path = optarg;
				use_zmq_output = false;
			break;

			case 'Z':
				use_zmq_output = true;
			break;

			case 'I':
				if (snprintf(zmq_host, sizeof(zmq_host), "%s", optarg) >= (int)sizeof(zmq_host)) {
					fprintf(stderr, "argument error: '-%c %s'\n", opt, optarg);
					goto exit_usage;
				}
			break;

			case 'P':
				result = parse_u32(optarg, &zmq_port_u32);
				if ((result == AIRSPYHF_SUCCESS) && ((zmq_port_u32 < 1) || (zmq_port_u32 > 65535))) {
					result = AIRSPYHF_ERROR;
				}
				if (result == AIRSPYHF_SUCCESS) {
					zmq_port = (uint16_t)zmq_port_u32;
				}
			break;

			case 'w':
				receive_wav = true;
				use_zmq_output = false;
			 break;

			case 's':
				serial_number = true;
				result = parse_u64(optarg, &serial_number_val);
			break;

			case 'f':
			{
				float freq_mhz_f;
				if (sscanf(optarg, "%f", &freq_mhz_f) == 1) {
					freq = true;
					freq_hz = freq_mhz_f * 1000000; // convert MHz to Hz
				}
			}
			break;

			case 'a': /* Sample rate */
				sample_rate = true;
				result = parse_u32(optarg, &sample_rate_u32);
			break;

			case 'n':
				limit_num_samples = true;
				result = parse_u64(optarg, &samples_to_xfer);
			break;

			case 'g':
			if (strcmp(optarg, "off") == 0) hf_agc = false;
			break;

			case 'l':
				fprintf (stderr, "************** [%s]\n", optarg);
				if (strcmp(optarg, "high") == 0) hf_agc_threshold = true;
			break;

			case 't':
				if (sscanf(optarg, "%d", &hf_att_val) == 1) {
					if (hf_att_val > 8) {
						hf_att_val = 0;
						fprintf (stderr, "Bad HF attenuator value.\n");
						goto exit_usage;
					}
				}
			break;

			case 'm':
				if (strcmp(optarg, "on") == 0) hf_lna = true;
			break;

			case 'd':
				verbose = true;
			break;

			case 'h':
				goto exit_usage;

			case 'z':
				do_not_use_manual_commands = true;
			break;

			default:
				fprintf(stderr, "unknown argument '-%c %s'\n", opt, optarg);
				goto exit_usage;
		}

		if( result != AIRSPYHF_SUCCESS ) {
			fprintf(stderr, "argument error: '-%c %s'\n", opt, optarg);
			goto exit_usage;
		}
	}

	if (sample_rate) sample_rate_val = sample_rate_u32;

	bytes_to_xfer = samples_to_xfer * (32 * 2 / 8);  // 32 bits per sample / 2 channels (I+Q) / 8 bits per byte

	if (samples_to_xfer >= SAMPLES_TO_XFER_MAX_U64) {
		fprintf(stderr, "argument error: num_samples must be less than %s/%sMio\n",
				u64toa(SAMPLES_TO_XFER_MAX_U64, &ascii_u64_data1),
				u64toa((SAMPLES_TO_XFER_MAX_U64/FREQ_ONE_MHZ_U64), &ascii_u64_data2) );
		goto exit_usage;
	}


	/*
	 * HF coverage between 9 kHz .. 31 MHz
	 * VHF coverage between 60 .. 260 MHz
	 */
	if( freq ) {  // check frequency value as specified on the command line

		if( !( ((freq_hz >= 9000) && (freq_hz <= 31000000))
									||
			((freq_hz >= 60000000 && freq_hz <= 260000000))) ) {

			fprintf(stderr, "argument error: frequency %d Hz out of range\n", freq_hz);
			goto exit_usage;
		}
	} else {
		/* Use default freq */
		freq_hz = DEFAULT_FREQ_HZ;
	}

	if( receive_wav ) {
		time_t rawtime;
		struct tm * timeinfo;
		char date_time[64];
		char path_file[256];

		time (&rawtime);
		timeinfo = localtime (&rawtime);
		/* File format AirSpy Year(2013), Month(11), Day(28), Hour Min Sec+Z, Freq kHz, IQ.wav */
		strftime(date_time, sizeof(date_time), "%Y%m%d_%H%M%S", timeinfo);
		snprintf(path_file, sizeof(path_file), "AirSpy_%sZ_%ukHz_IQ.wav", date_time, (uint32_t)(freq_hz/(1000ull)) );
		path = path_file;
		fprintf(stderr, "Receive wav file: [%s]\n", path);
	}

	if ((receive_wav == true) && (use_zmq_output == true)) {
		fprintf(stderr, "error: -w cannot be used together with -Z\n");
		goto exit_usage;
	}

	if ((path == 0) && (use_zmq_output == false)) {
		fprintf(stderr, "error: you shall specify at least -r <with filename> or -w option\n");
		goto exit_usage;
	}


	if(verbose == true) {
		uint32_t serial_number_msb_val;
		uint32_t serial_number_lsb_val;

		fprintf(stderr, "airspyhf_zeromq_rx\n");
		serial_number_msb_val = (uint32_t)(serial_number_val >> 32);
		serial_number_lsb_val = (uint32_t)(serial_number_val & 0xFFFFFFFF);
		if(serial_number)
			fprintf(stderr, "S/N 0x%08X%08X\n", serial_number_msb_val, serial_number_lsb_val);
		fprintf(stderr, "Frequency: -f %.6fMHz (%d Hz)\n",((double)freq_hz/(double)FREQ_ONE_MHZ), freq_hz );

		if( limit_num_samples ) {
			fprintf(stderr, "#samples -n %s (%sM)\n",
							u64toa(samples_to_xfer, &ascii_u64_data1),
							u64toa((samples_to_xfer/FREQ_ONE_MHZ), &ascii_u64_data2));
		}
	}

	if(serial_number == true) {
		if( airspyhf_open_sn(&device, serial_number_val) != AIRSPYHF_SUCCESS ) {
			fprintf(stderr, "airspyhf_open_sn() failed\n");
			goto exit_failure;
		}
	} else {
		if( airspyhf_open(&device) != AIRSPYHF_SUCCESS ) {
			fprintf(stderr, "airspyhf_open() failed\n");
			goto exit_failure;
		}
	}

	// check sample rates
	airspyhf_get_samplerates(device, &nsrates, 0);
	supported_samplerates = (uint32_t *) malloc(nsrates * sizeof(uint32_t));
	airspyhf_get_samplerates(device, supported_samplerates, nsrates);

	wav_sample_per_sec = 0;
	if (sample_rate == true) {
		uint32_t s;
		uint32_t *ps = supported_samplerates;
		// search into the available sample rates
		for (s=0; s < nsrates; ++s, ++ps)
			if (sample_rate_val == *ps) {
				wav_sample_per_sec = sample_rate_val;
				break;
			}
	} else {
		wav_sample_per_sec = *supported_samplerates;
	}
	free(supported_samplerates);

	// if arrives here an unsupported sample rate was requested
	if (wav_sample_per_sec == 0) {
		fprintf(stderr, "argument error: unsupported sample rate: %d\n", sample_rate_val);
		goto exit_failure;
	}
	// set sample rate
	if (airspyhf_set_samplerate(device, wav_sample_per_sec) != AIRSPYHF_SUCCESS) {
		fprintf(stderr, "airspyhf_set_samplerate() failed: %d\n", wav_sample_per_sec);
		goto exit_failure;
	}
	current_sample_rate = wav_sample_per_sec;

	if (verbose) {
		fprintf(stderr, "%f MS/s %s\n", wav_sample_per_sec * 0.000001f, "IQ");
	}

	{ // print receiver serial number
		airspyhf_read_partid_serialno_t read_partid_serialno;

		if (airspyhf_board_partid_serialno_read(device, &read_partid_serialno) != AIRSPYHF_SUCCESS) {
			fprintf(stderr, "airspyhf_board_partid_serialno_read() failed\n");
			goto exit_failure;
		} else
			fprintf(stderr, "Device Serial Number: 0x%08X%08X\n",
					read_partid_serialno.serial_no[0],
					read_partid_serialno.serial_no[1]);
	}

	// manual commands
	if (do_not_use_manual_commands == false) {
		/* 0 = off, 1 = on */
		if (airspyhf_set_hf_agc(device, (hf_agc == true) ? 1:0)  == AIRSPYHF_SUCCESS) {
			fprintf (stderr, "HF AGC %s\n", (hf_agc == true) ? "ON":"OFF");
		} else {
			fprintf(stderr, "airspyhf_set_hf_agc failed.\n");
			goto exit_failure;
		}

		if (hf_agc == true) {
			/* when agc on: 0 = low, 1 = high */
			if (airspyhf_set_hf_agc_threshold (device, hf_agc_threshold == true?1:0) == AIRSPYHF_SUCCESS) {
				fprintf (stderr, "HF AGC threshold %s\n", (hf_agc_threshold == true) ? "High":"Low");
			} else {
				fprintf(stderr, "airspyhf_set_agc_threshold() failed.\n");
				goto exit_failure;
			}
		} else {
			fprintf (stderr, "HF AGC threshold ignored as AGC is disabled.\n");
		}

		if (hf_agc == false) {
			/* when agc off: 0 .. 8 with attenuation = (value * 8) dB */
			if (airspyhf_set_hf_att(device, hf_att_val) == AIRSPYHF_SUCCESS) {
				fprintf (stderr, "HF Attenuator value: -%d dB\n", hf_att_val*6);
			} else {
				fprintf(stderr, "airspyhf_set_hf_att() failed: offending value: %d\n", hf_att_val);
				goto exit_failure;
			}
		} else {
			fprintf (stderr, "HF attenuator value ignored as AGC is enabled.\n");
		}
		/* 0 or 1: 1 to activate LNA (alias PreAmp): 1 = +6 dB gain - compensated in digital */
		if (airspyhf_set_hf_lna(device, hf_lna == false?0:1) == AIRSPYHF_SUCCESS) {
			fprintf (stderr, "HF LNA %s\n", (hf_lna == true) ? "ON":"OFF");
		} else {
			fprintf(stderr, "airspyhf_set_hf_lna() failed.\n");
			goto exit_failure;
		}
	}

	if (use_zmq_output) {
		char zmq_endpoint[256];
		int zmq_sndhwm = 32;
		int zmq_linger = 0;

		snprintf(zmq_endpoint, sizeof(zmq_endpoint), "tcp://%s:%u", zmq_host, zmq_port);

		zmq_context = zmq_ctx_new();
		if (zmq_context == NULL) {
			fprintf(stderr, "zmq_ctx_new() failed: %s\n", zmq_strerror(errno));
			goto exit_failure;
		}

		zmq_pub_socket = zmq_socket(zmq_context, ZMQ_PUB);
		if (zmq_pub_socket == NULL) {
			fprintf(stderr, "zmq_socket() failed: %s\n", zmq_strerror(errno));
			goto exit_failure;
		}

		if (zmq_setsockopt(zmq_pub_socket, ZMQ_SNDHWM, &zmq_sndhwm, sizeof(zmq_sndhwm)) != 0) {
			fprintf(stderr, "zmq_setsockopt(ZMQ_SNDHWM) failed: %s\n", zmq_strerror(errno));
			goto exit_failure;
		}

		if (zmq_setsockopt(zmq_pub_socket, ZMQ_LINGER, &zmq_linger, sizeof(zmq_linger)) != 0) {
			fprintf(stderr, "zmq_setsockopt(ZMQ_LINGER) failed: %s\n", zmq_strerror(errno));
			goto exit_failure;
		}

		if (zmq_bind(zmq_pub_socket, zmq_endpoint) != 0) {
			fprintf(stderr, "zmq_bind(%s) failed: %s\n", zmq_endpoint, zmq_strerror(errno));
			goto exit_failure;
		}

		fprintf(stderr, "ZeroMQ PUB output bound at %s\n", zmq_endpoint);
	} else {
		// output file management
		if (strcmp (path, "stdout") == 0) {
			fd = stdout;
		} else {
			if( !(fd = fopen(path, "wb")) ) {
				perror (path);
				goto exit_failure;
			}
		}

		/* Change fd buffer to have bigger one to store data to file */
		if( setvbuf(fd , NULL , _IOFBF , FD_BUFFER_SIZE) != 0 ) {
			perror("setvbuf() failed:");
			goto exit_failure;
		}

		/* Write Wav header */
		if( receive_wav ) fwrite(&wave_file_hdr, 1, sizeof(t_wav_file_hdr), fd);
	}


#ifdef _MSC_VER
	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );
#else
	signal(SIGINT, &sigint_callback_handler);
	signal(SIGILL, &sigint_callback_handler);
	signal(SIGFPE, &sigint_callback_handler);
	signal(SIGSEGV, &sigint_callback_handler);
	signal(SIGTERM, &sigint_callback_handler);
	signal(SIGABRT, &sigint_callback_handler);
#endif

	if( airspyhf_start(device, rx_callback, NULL) != AIRSPYHF_SUCCESS ) {
		fprintf(stderr, "airspyhf_start() failed.\n");
		goto exit_failure;
	}

	if( airspyhf_set_freq(device, freq_hz) != AIRSPYHF_SUCCESS ) {
		fprintf(stderr, "airspyhf_set_freq() failed.\n");
		goto exit_failure;
	}

	/*
	 * main loop
	 */
	fprintf(stderr, "Stop with Ctrl-C\n");

	average_rate = (float) wav_sample_per_sec;

	sleep(1);

	while(	(airspyhf_is_streaming(device) == true) &&
			(do_exit == false) )
	{
		char buf [64];
		float average_rate_now = average_rate * 1e-6f;

		if (verbose) {
			snprintf(buf, sizeof(buf),"%2.3f", average_rate_now);
			//average_rate_now = 9.5f;
			fprintf(stderr, "Streaming at %5s MS/s\n", buf);
		}
		if ((limit_num_samples == true) && (bytes_to_xfer == 0))
			do_exit = true;
		else
			sleep(1);
	}

	if (do_exit) {
		fprintf(stderr, "\nUser cancel, exiting...\n");
	} else {
		fprintf(stderr,"\nExiting...\n");
	}
	// print time elapsed
	{
		float time_diff;
		gettimeofday (&t_end, 0);
		time_diff = TimevalDiff(&t_end, &t_start);
		fprintf(stderr, "Total time: %5.4f s\n", time_diff);
	}

	if (rate_samples > 0)
		fprintf(stderr, "Average speed %2.4f MS/s %s\n", (global_average_rate * 1e-6f / rate_samples), "IQ");

	if(device) {
		if( airspyhf_stop(device) != AIRSPYHF_SUCCESS ) {
			fprintf(stderr, "airspyhf_stop() failed\n");
		}

		if( airspyhf_close(device) != AIRSPYHF_SUCCESS ) {
			fprintf(stderr,"airspyhf_close() failed\n");
		}
	}

	if (fd && receive_wav ) {
			/* Get size of file */
			file_pos = ftell(fd);
			/* Wav Header */
			wave_file_hdr.hdr.size = file_pos - 8;
			/* Wav Format Chunk */
			wave_file_hdr.fmt_chunk.wFormatTag = 3;  // 3=Float32
			wave_file_hdr.fmt_chunk.wChannels = 2;   // I + Q
			wave_file_hdr.fmt_chunk.dwSamplesPerSec = wav_sample_per_sec;
			wave_file_hdr.fmt_chunk.dwAvgBytesPerSec = wave_file_hdr.fmt_chunk.dwSamplesPerSec * 4;
			wave_file_hdr.fmt_chunk.wBlockAlign = 2 * (32 / 8);
			wave_file_hdr.fmt_chunk.wBitsPerSample = 32;
			/* Wav Data Chunk */
			wave_file_hdr.data_chunk.chunkSize = file_pos - sizeof(t_wav_file_hdr);
			/* Overwrite header with updated data */
			rewind(fd);
			fwrite(&wave_file_hdr, 1, sizeof(t_wav_file_hdr), fd);
	}
	if (fd && fd != stdout) fclose(fd);
	if (zmq_pub_socket) zmq_close(zmq_pub_socket);
	if (zmq_context) zmq_ctx_term(zmq_context);
	fprintf(stderr, "done\n");
	return EXIT_SUCCESS;

exit_failure:
	if (zmq_pub_socket) zmq_close(zmq_pub_socket);
	if (zmq_context) zmq_ctx_term(zmq_context);
	if (fd && fd != stdout) fclose(fd);
	airspyhf_close(device);
	return EXIT_FAILURE;

exit_usage:
	usage();
	return EXIT_FAILURE;
}
