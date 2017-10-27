#!/bin/sh
set -ex

osmo-clean-workspace.sh

autoreconf --install --force
./configure
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh

osmo-clean-workspace.sh
