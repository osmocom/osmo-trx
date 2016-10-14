#!/bin/sh
set -ex
autoreconf --install --force
./configure
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh
