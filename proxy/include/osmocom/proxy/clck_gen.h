#pragma once

/*! Special value for clck_gen_{get,set}_start_fn() meaning
 * a random starting TDMA frame number (default). */
#define CLCK_GEN_START_FN_RANDOM	(-1)

int clck_gen_init(void);

void clck_gen_trx_list_updated(void);

int clck_gen_set_start_fn(int fn);
int clck_gen_get_start_fn(void);

int clck_gen_set_ind_period(unsigned int period);
unsigned int clck_gen_get_ind_period(void);
