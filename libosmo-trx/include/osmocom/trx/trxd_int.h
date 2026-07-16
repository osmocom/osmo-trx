/*! \file osmocom/trx/trxd_int.h
 * TRXD codec: internal declarations. */

#pragma once

#include <stdint.h>

struct osmo_trxd_burst_ind;
struct osmo_trxd_burst_req;

/* MTS (Modulation and Training Sequence) field, for TRXDv1 and higher */
int trxd_mts_parse_ind(struct osmo_trxd_burst_ind *bi, uint8_t mts);
int trxd_mts_parse_req(struct osmo_trxd_burst_req *br, uint8_t mts);
int trxd_mts_build_ind(uint8_t *mts, const struct osmo_trxd_burst_ind *bi);
int trxd_mts_build_req(uint8_t *mts, const struct osmo_trxd_burst_req *br);
