
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#ifdef __cplusplus
}
#endif

/* ------------------------------------------------------------------------ */
/* Data interface handlers                                                  */
/* ------------------------------------------------------------------------ */
/* DATA interface                                                           */
/*                                                                          */
/* Messages on the data interface carry one radio burst per UDP message.    */
/*                                                                          */
/* Received Data Burst:                                                     */
/* 1 byte timeslot index                                                    */
/* 4 bytes GSM frame number, BE                                             */
/* 1 byte RSSI in -dBm                                                      */
/* 2 bytes correlator timing offset in 1/256 symbol steps, 2's-comp, BE     */
/* 148 bytes soft symbol estimates, 0 -> definite "0", 255 -> definite "1"  */
/* 2 bytes are not used, but being sent by OsmoTRX                          */
/*                                                                          */
/* Transmit Data Burst:                                                     */
/* 1 byte timeslot index                                                    */
/* 4 bytes GSM frame number, BE                                             */
/* 1 byte transmit level wrt ARFCN max, -dB (attenuation)                   */
/* 148 bytes output symbol values, 0 & 1                                    */
/* ------------------------------------------------------------------------ */

struct __attribute__((packed)) trxd_to_trx {
	uint8_t ts;
	uint32_t fn;
	uint8_t txlev;
	uint8_t symbols[148];
};

struct __attribute__((packed)) trxd_from_trx {
	uint8_t ts;
	uint32_t fn;
	uint8_t rssi;
	uint16_t toa;
	uint8_t symbols[148];
	uint8_t pad[2];
};

#define TRXC_BUF_SIZE	1024

struct TRX_C {
    char cmd[TRXC_BUF_SIZE];
};

#ifdef __cplusplus
void push_c(TRX_C* i);
TRX_C* pop_c();

void push_d(trxd_from_trx* i);
trxd_to_trx* pop_d();

#else


char* trxif_from_trx_c();
void trxif_to_trx_c(char* msg);

struct trxd_from_trx* trxif_from_trx_d();
void trxif_to_trx_d(struct trxd_to_trx* msg);

struct osmo_fd* get_c_fd();
struct osmo_fd* get_d_fd();

#endif

