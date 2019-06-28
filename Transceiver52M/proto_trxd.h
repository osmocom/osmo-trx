#pragma once

#include <stdint.h>
#include <osmocom/core/endian.h>

struct trxd_hdr_common {
#if OSMO_IS_LITTLE_ENDIAN
        uint8_t tn:3,
                reserved:1,
                version:4;
#elif OSMO_IS_BIG_ENDIAN
        uint8_t version:4,
                reserved:1,
                tn:3;
#endif
        uint32_t fn; /* big endian */
} __attribute__ ((packed));

struct trxd_hdr_v0_specific {
        uint8_t rssi;
        uint16_t toa; /* big endian */
} __attribute__ ((packed));

struct trxd_hdr_v0 {
        struct trxd_hdr_common common;
        struct trxd_hdr_v0_specific v0;
        uint8_t soft_bits[0];
} __attribute__ ((packed));
