/* Dummy TRX for sening PRBS test sequences into osmo-bts-trx to test
 * the decoder/receiver processing in osmo-bts-trx as well as any
 * additional PRBS testing code.
 *
 * The purpose of this program is to use it as a mock dummy MS-side
 * transmitter of GSM bursts that contain encoded PRBS sequences,
 * similar to what one would normally do with an arbitrary
 * function/waveform generator or BERT tester in hardware.
 *
 * (C) 2017 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * Licensed under terms of the GNU Generral Public License, Version 2,
 * or (at your option) any later version.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include <netinet/in.h>

#include <osmocom/core/bits.h>
#include <osmocom/core/utils.h>
#include <osmocom/core/prbs.h>
#include <osmocom/core/socket.h>
#include <osmocom/gsm/gsm_utils.h>
#include <osmocom/coding/gsm0503_coding.h>

/***********************************************************************
 * GSM Constants
 ***********************************************************************/

#define GSM_FR_BYTES	33
#define GSM_BURST_BITS	116
#define GSM_4BURST_BITS	(GSM_BURST_BITS*4)
#define GSM_BURST_LEN	148


/***********************************************************************
 * TRX Interface / Protocol
 ***********************************************************************/

#define TRX_BASE_PORT	5700
/* DATA port on the TRX side */
#define	TRX_PORT_CTRL_TRX(C) 	(TRX_BASE_PORT+(2*(C))+1)
#define	TRX_PORT_DATA_TRX(C) 	(TRX_BASE_PORT+(2*(C))+2)
#define	TRX_PORT_CTRL_BTS(C) 	(TRX_PORT_CTRL_TRX(C)+100)
#define	TRX_PORT_DATA_BTS(C) 	(TRX_PORT_DATA_TRX(C)+100)

struct trx_ul_msg {
	uint8_t		ts;
	uint32_t	fn;
	uint8_t		rssi;
	uint16_t	t_offs;
	uint8_t		bits[148];	/* 0..255, *NOT* sbit_t */
} __attribute__((packed));

struct trx_dl_msg {
	uint8_t		ts;
	uint32_t	fn;
	uint8_t		att_db;
	ubit_t		bits[148];
} __attribute__((packed));


/***********************************************************************
 * Helper Functions
 ***********************************************************************/

static int ubits2trxbits(uint8_t *sbits, const ubit_t *ubits, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		if ((*ubits++) & 1) {
			*sbits++ = 255;
		} else {
			*sbits++ = 0;
		}
	}

	return count;
}

static int __attribute__((__unused__)) dec(const ubit_t *bursts_u)
{
	sbit_t bursts_s[GSM_4BURST_BITS*2];
	uint8_t dec_tch_data[GSM_FR_BYTES];
	int n_errors, n_bits_total;
	int rc;

	/* convert from u_bit (tx) to s_bit (rx)  */
	osmo_ubit2sbit(bursts_s, bursts_u, sizeof(bursts_s));

	rc = gsm0503_tch_fr_decode(dec_tch_data, bursts_s, 1, 0, &n_errors, &n_bits_total);
	printf("rc=%d, n_errors=%d, n_bits_total=%d: %s\n", rc, n_errors, n_bits_total,
		osmo_hexdump(dec_tch_data, sizeof(dec_tch_data)));

	return rc;
}

/*! \brief Training Sequences (TS 05.02 Chapter 5.2.3) */
static const ubit_t _sched_tsc[8][26] = {
	{ 0,0,1,0,0,1,0,1,1,1,0,0,0,0,1,0,0,0,1,0,0,1,0,1,1,1, },
	{ 0,0,1,0,1,1,0,1,1,1,0,1,1,1,1,0,0,0,1,0,1,1,0,1,1,1, },
	{ 0,1,0,0,0,0,1,1,1,0,1,1,1,0,1,0,0,1,0,0,0,0,1,1,1,0, },
	{ 0,1,0,0,0,1,1,1,1,0,1,1,0,1,0,0,0,1,0,0,0,1,1,1,1,0, },
	{ 0,0,0,1,1,0,1,0,1,1,1,0,0,1,0,0,0,0,0,1,1,0,1,0,1,1, },
	{ 0,1,0,0,1,1,1,0,1,0,1,1,0,0,0,0,0,1,0,0,1,1,1,0,1,0, },
	{ 1,0,1,0,0,1,1,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,1,1, },
	{ 1,1,1,0,1,1,1,1,0,0,0,1,0,0,1,0,1,1,1,0,1,1,1,1,0,0, },
};

/***********************************************************************
 * state + processing functions
 ***********************************************************************/

/* state we have to keep for one physical channel */
struct pchan_data {
	/* PRBS state */
	struct osmo_prbs_state st;
	/* unpacked PRBS bits, generated from PRBS */
	ubit_t prbs_u[4+260];
	/* packed frame (to be sent) */
	uint8_t tch_data[GSM_FR_BYTES];
	/* burst bits (ubit) to be transmitted */
	ubit_t bursts[GSM_4BURST_BITS*2]; /* 116 * 8 */
	/* burst bits (sbit) 'as if received' */
	sbit_t bursts_s[GSM_4BURST_BITS*2];
	/* next to-be transmitted burst number */
	unsigned int burst_nr;
	/* training sequence code */
	unsigned int tsc;

	/* loose 'count' bursts every 'nth_mframe' on TRX-BTS interface */
	struct {
		unsigned int count;
		unsigned int nth_mframe;
	} sim_lost_bursts;

	/* zero 'count' bursts every 'nth_mframe' on TRX-BTS interface */
	struct {
		unsigned int count;
		unsigned int nth_mframe;
	} sim_zero_bursts;

	/* flip every 'nth_bit' of the PRNG oudput before encoding */
	struct {
		unsigned int nth_bit;
		unsigned int i;
	} sim_flip_codec_bits;

	unsigned int i;
};

struct ts_data {
	struct pchan_data pchan[2];
};

struct trx_data {
	struct ts_data ts[8];
};

static struct trx_data g_trx_data;

/* initialize the state for one TRX */
static void trx_data_init(struct trx_data *trx)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(trx->ts); i++) {
		struct ts_data *ts = &trx->ts[i];
		int j;
		for (j = 0; j < ARRAY_SIZE(ts->pchan); j++) {
			struct pchan_data *pchan = &ts->pchan[j];

			memset(pchan, 0, sizeof(*pchan));
			osmo_prbs_state_init(&pchan->st, &osmo_prbs9);
			pchan->tsc = 7;
		}
	}
}

/* apply any intentional errors to the output of the PRBS sequence */
static void apply_errors_prbs(struct pchan_data *pchan)
{
	int i;

	for (i = 0; i < sizeof(pchan->prbs_u)-4; i++) {
		pchan->sim_flip_codec_bits.i++;
		if (pchan->sim_flip_codec_bits.i == pchan->sim_flip_codec_bits.nth_bit) {
			pchan->sim_flip_codec_bits.i = 0;
			pchan->prbs_u[4+i] ^= 0x01;
		}
	}
}

/*! obtain the next to-be-transmitted burst for the given pchan
 *  \param pchan physical channel on which we operate
 *  \param[in] fn frame number
 *  \param[out] burst_out caller-provided buffer for 148 unpacked output bits
 *  \retruns number of bits stored in \a burst_out */
static int pchan_get_next_burst(struct pchan_data *pchan, uint32_t fn, ubit_t *burst_out)
{
	uint32_t fn26 = fn % 26;
	int rc;

	if (fn26 == 0 || fn26 == 4 || fn26 == 8 || fn26 == 13 || fn26 == 17 || fn26 == 21)
		pchan->burst_nr = 0;

	if (fn26 == 12 || fn26 == 25) {
		memset(burst_out, 0, GSM_BURST_LEN);
		return GSM_BURST_LEN;
	}

	if (pchan->burst_nr == 0) {
		/* generate PRBS output in ubit format, skipping first nibble for 260-264 padding */
		const uint8_t prefix[] = { 0xd0 };
		osmo_pbit2ubit(pchan->prbs_u, prefix, 4);
		rc = osmo_prbs_get_ubits(pchan->prbs_u+4, sizeof(pchan->prbs_u)-4, &pchan->st);
		OSMO_ASSERT(rc == sizeof(pchan->prbs_u)-4);

		apply_errors_prbs(pchan);

		/* pack to PBIT format */
		rc = osmo_ubit2pbit(pchan->tch_data, pchan->prbs_u, sizeof(pchan->prbs_u));
		//memset(pchan->tch_data, 0xff, sizeof(pchan->tch_data));

		printf("%s\n", osmo_hexdump(pchan->tch_data, GSM_FR_BYTES));

		/* shift buffer by 4 bursts for interleaving */
		memcpy(pchan->bursts, pchan->bursts + GSM_4BURST_BITS, GSM_4BURST_BITS);
		memset(pchan->bursts + GSM_4BURST_BITS, 0, GSM_4BURST_BITS);

		/* encode block (codec frame) into four bursts */
		rc = gsm0503_tch_fr_encode(pchan->bursts, pchan->tch_data, GSM_FR_BYTES, 1);
		OSMO_ASSERT(rc == 0);
#if 0
		int i;
		for (i = 0; i < sizeof(pchan->bursts); i += GSM_BURST_BITS)
			printf("\t%s\n", osmo_ubit_dump(pchan->bursts + i, GSM_BURST_BITS));

		dec(pchan->bursts);
#endif
	}

	/* for all bursts: format 148 symbols from 116 input bits */
	ubit_t *burst = pchan->bursts  + pchan->burst_nr * GSM_BURST_BITS;
//	printf("TX(%u): %s\n", pchan->burst_nr, osmo_ubit_dump(burst, GSM_BURST_BITS));
	memset(burst_out, 0, 3);		/* guard bits */
	memcpy(burst_out+3, burst, 58);		/* firrst half */
	memcpy(burst_out+61, _sched_tsc[pchan->tsc], 26);	/* midamble */
	memcpy(burst_out+87, burst+58, 58);	/* second half */
	memset(burst_out+145, 0, 3);		/* guard bits */

	/* increment burst number for next call */
	pchan->burst_nr += 1;

	return GSM_BURST_LEN;
}

static int pchan_process_ts_fn(struct pchan_data *pchan, uint32_t fn, uint8_t *burst_t)
{
	ubit_t burst_u[GSM_BURST_LEN];
	int rc;

	rc = pchan_get_next_burst(pchan, fn, burst_u);
	OSMO_ASSERT(rc == sizeof(burst_u));

	/* convert from u_bit (tx) to s_bit (rx)  */
	ubits2trxbits(burst_t, burst_u, GSM_BURST_LEN);

	return GSM_BURST_LEN;
}

/* read TRX DL data from BTS, write TRX UL data to BTS */
static int read_and_process(int fd)
{
	/* receive (downlink) buffer */
	uint8_t rx_dl_buf[1024];
	struct trx_dl_msg *dl_msg = (struct trx_dl_msg *) rx_dl_buf;
	/* transmit (uplink) buffer */
	uint8_t tx_ul_buf[1024];
	struct trx_ul_msg *ul_msg = (struct trx_ul_msg *) tx_ul_buf;
	/* other variables */
	struct pchan_data *pchan;
	uint32_t fn;
	uint8_t rc;

	/* do a blocking read on the socket and receive DL from BTS */
	rc = read(fd, rx_dl_buf, sizeof(rx_dl_buf));
	if (rc < sizeof(*dl_msg))
		return rc;

	fn = ntohl(dl_msg->fn);

	if (dl_msg->ts >= ARRAY_SIZE(g_trx_data.ts))
		return -ENODEV;

	if (dl_msg->ts != 2)
		return 0;

	printf("FN=%s TS=%u\n", gsm_fn_as_gsmtime_str(fn), dl_msg->ts);

	/* FIXME: second pchan for TCH/H */
	pchan = &g_trx_data.ts[dl_msg->ts].pchan[0];

	rc = pchan_process_ts_fn(pchan, fn, (uint8_t *) ul_msg->bits);
	OSMO_ASSERT(rc == sizeof(ul_msg->bits));

	/* copy over timeslot and frame number */
	ul_msg->fn = htonl(fn);
	ul_msg->ts = dl_msg->ts;

	/* simulate lost frames on TRX <-> BTS interface */
	if (pchan->sim_lost_bursts.count) {
		/* count number of 26-multiframes */
		static int count = 0;
		if (fn % 26 == 0)
			count++;

		/* every 10th multiframe, drop two entire block of 8 bursts */
		if (count % pchan->sim_lost_bursts.nth_mframe == 0 &&
		    (fn % 26) <= pchan->sim_lost_bursts.count) {
			printf("===> SKIPPING BURST\n");
			return 0;
		}
	}

	/* simulate zero-ed frames on TRX <-> BTS interface */
	if (pchan->sim_zero_bursts.count) {
		/* count number of 26-multiframes */
		static int count = 0;
		if (fn % 26 == 0)
			count++;

		/* every 10th multiframe, drop two entire block of 8 bursts */
		if (count % pchan->sim_zero_bursts.nth_mframe == 0 &&
		    (fn % 26) <= pchan->sim_zero_bursts.count) {
			memset(ul_msg->bits, 0, sizeof(ul_msg->bits));
			printf("===> ZEROING BURST\n");
		}
	}

	/* write uplink message towards BTS */
	rc = write(fd, tx_ul_buf, sizeof(*ul_msg));
	if (rc < sizeof(*ul_msg))
		return -EIO;

	return 0;
}

static int open_trx_data_sock(unsigned int trx_nr, const char *bts_host)
{
	int rc;

	rc = osmo_sock_init2(AF_INET, SOCK_DGRAM, IPPROTO_UDP, NULL, TRX_PORT_DATA_TRX(trx_nr),
				bts_host, TRX_PORT_DATA_BTS(trx_nr),
				OSMO_SOCK_F_CONNECT | OSMO_SOCK_F_BIND);
	return rc;
}


int main(int argc, char **argv)
{
	int fd;

	trx_data_init(&g_trx_data);

	//g_trx_data.ts[2].pchan[0].sim_zero_bursts.count = 8;
	//g_trx_data.ts[2].pchan[0].sim_zero_bursts.nth_mframe = 10;
	g_trx_data.ts[2].pchan[0].sim_flip_codec_bits.nth_bit = 260*4;

	fd = open_trx_data_sock(0, "127.0.0.1");
	if (fd < 0)
		exit(1);

	while (1) {
		read_and_process(fd);
	}

	return 0;
}
