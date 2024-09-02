#!/bin/sh
# jenkins build helper script for osmo-trx.  This is how we build on jenkins.osmocom.org
#
# environment variables:
# * INSTR: configure the CPU instruction set ("--with-sse", "--with-neon" or "--with-neon-vfpv4")
# * WITH_MANUALS: build manual PDFs if set to "1"
# * PUBLISH: upload manuals after building if set to "1" (ignored without WITH_MANUALS = "1")
# * INSIDE_CHROOT: (used internally) set to "1" when the script runs with QEMU in an ARM chroot
#
set -ex

case "$INSTR" in
	"--with-neon"*)
		case "$(arch)" in
			arm*)
				;;
			*)
				set +x
				echo "ERROR: trying to build with INSTR=$INSTR but not running on a 32-bit arm machine! (arch=$(arch))"
				exit 1
				;;
		esac
		;;
esac

if ! [ -x "$(command -v osmo-build-dep.sh)" ]; then
	echo "Error: We need to have scripts/osmo-deps.sh from http://git.osmocom.org/osmo-ci/ in PATH !"
	exit 2
fi

base="$PWD"
deps="$base/deps"
inst="$deps/install"
export deps inst

osmo-clean-workspace.sh

mkdir "$deps" || true

osmo-build-dep.sh libosmocore "" "--enable-sanitize --disable-doxygen --disable-pcsc"
PARALLEL_MAKE="" osmo-build-dep.sh libusrp

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"
export PATH="$inst/bin:$PATH"

CONFIG="
	--enable-sanitize
	--enable-werror
	--with-bladerf
	--with-ipc
	--with-lms
	--with-mstrx
	--with-uhd
	--with-usrp1
	$INSTR
"

# Additional configure options and depends
if [ "$WITH_MANUALS" = "1" ]; then
	CONFIG="$CONFIG --enable-manuals"
fi

set +x
echo
echo
echo
echo " =============================== osmo-trx ==============================="
echo
set -x

cd "$base"
git submodule status
autoreconf --install --force
./configure $CONFIG
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh

if arch | grep -v -q arm; then
	DISTCHECK_CONFIGURE_FLAGS="$(echo $CONFIG | tr -d '\n')" $MAKE $PARALLEL_MAKE distcheck \
	  || cat-testlogs.sh
fi

if [ "$WITH_MANUALS" = "1" ] && [ "$PUBLISH" = "1" ]; then
	make -C "$base/doc/manuals" publish
fi

$MAKE $PARALLEL_MAKE maintainer-clean

osmo-clean-workspace.sh
