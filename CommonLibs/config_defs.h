#pragma once

/*
 * This file contains structures used by both VTY (C, dir CommonLibs) and
 * osmo-trx (CXX, dir Transceiver52)
 */

enum FillerType {
  FILLER_DUMMY,
  FILLER_ZERO,
  FILLER_NORM_RAND,
  FILLER_EDGE_RAND,
  FILLER_ACCESS_RAND,
};

enum ReferenceType {
  REF_INTERNAL,
  REF_EXTERNAL,
  REF_GPS,
};
