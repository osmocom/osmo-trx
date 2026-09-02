#pragma once

#include <stdint.h>

struct proxy_trx_chan;
struct osmo_trxd_burst_ind;
struct osmo_trxd_burst_req;
struct path_sim_cfg;

#define PATH_SIM_F_FAKE_RSSI	(1 << 0) /*!< use rssi instead of the path-loss formula */

/*! Per-channel RF path simulation state: ToA/RSSI/C-I measurement emulation
 * and burst dropping (simulated path loss / weak signal / interference). */
struct path_sim_state {
	uint32_t flags;		/*!< PATH_SIM_F_* */
	int tx_power;		/*!< dBm, nominal Tx power (NOMTXPOWER) */
	int tx_att;		/*!< dB, Tx power attenuation (SETPOWER) */
	int ta;			/*!< Timing Advance, symbol periods (SETTA) */
	int toa256;		/*!< reported ToA, 1/256 symbol periods (FAKE_TOA) */
	int toa256_jitter;	/*!< +/- random jitter around toa256 (FAKE_TOA "<val> <threshold>") */
	int rssi;		/*!< reported RSSI in dBm, if PATH_SIM_F_FAKE_RSSI (FAKE_RSSI) */
	int rssi_jitter;	/*!< +/- random jitter around rssi (FAKE_RSSI "<val> <threshold>") */
	int ci;			/*!< reported C/I in cB (FAKE_CI) */
	int ci_jitter;		/*!< +/- random jitter around ci (FAKE_CI "<val> <threshold>") */
	unsigned int burst_drop_amount; /*!< bursts left to drop (FAKE_DROP) */
	unsigned int burst_drop_period; /*!< drop if (fn % period) == 0 */
};

void path_sim_state_reset(struct path_sim_state *ps, int tx_power, int toa256, int ci);
void path_sim_apply(struct osmo_trxd_burst_ind *bi,
		    struct proxy_trx_chan *dst,
		    const struct osmo_trxd_burst_req *br,
		    const struct proxy_trx_chan *src,
		    const struct path_sim_cfg *cfg);
int path_sim_measure(uint32_t freq_hz, const struct path_sim_cfg *cfg);

struct path_sim_cfg *path_sim_cfg_alloc(void *talloc_ctx);

void path_sim_cfg_set_noise_dbm(struct path_sim_cfg *cfg, int noise_dbm);
int path_sim_cfg_get_noise_dbm(const struct path_sim_cfg *cfg);

void path_sim_cfg_set_path_loss_db(struct path_sim_cfg *cfg, int path_loss_db);
int path_sim_cfg_get_path_loss_db(const struct path_sim_cfg *cfg);

void path_sim_cfg_set_nom_toa256(struct path_sim_cfg *cfg, int nom_toa256);
int path_sim_cfg_get_nom_toa256(const struct path_sim_cfg *cfg);

void path_sim_cfg_set_nom_ci_cb(struct path_sim_cfg *cfg, int nom_ci_cb);
int path_sim_cfg_get_nom_ci_cb(const struct path_sim_cfg *cfg);
