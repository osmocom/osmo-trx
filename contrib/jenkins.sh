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

substr() { [ -z "${2##*$1*}" ]; }

#apt-get install qemu qemu-user-static qemu-system-arm debootstrap fakeroot proot
mychroot_nocwd() {
        # LC_ALL + LANGUAGE set to avoid lots of print errors due to locale not being set inside container
        # PATH is needed to be able to reach binaries like ldconfig without logging in to root, which adds the paths to PATH.
        # PROOT_NO_SECCOMP is required due to proot bug #106
        LC_ALL=C LANGUAGE=C PATH="$PATH:/usr/sbin:/sbin" PROOT_NO_SECCOMP=1 proot -r "$ROOTFS" -w / -b /proc --root-id -q qemu-arm-static "$@"
}

mychroot() {
        mychroot_nocwd -w / "$@"
}

if [ -z "${INSIDE_CHROOT}" ]; then

        # Only use ARM chroot if host is not ARM and the target is ARM:
        if ! $(substr "arm" "$(uname -m)") && [ "x${INSTR}" = "x--with-neon" -o "x${INSTR}" = "x--with-neon-vfpv4" ]; then

                OSMOTRX_DIR="$PWD" # we assume we are called as contrib/jenkins.sh
                ROOTFS_PREFIX="${ROOTFS_PREFIX:-$HOME}"
                ROOTFS="${ROOTFS_PREFIX}/qemu-img"
                mkdir -p "${ROOTFS_PREFIX}"

                # Prepare chroot:
                if [ ! -d "$ROOTFS" ]; then
                        mkdir -p "$ROOTFS"
                        if [ "x${USE_DEBOOTSTRAP}" = "x1" ]; then
                                fakeroot qemu-debootstrap --foreign --include="linux-image-armmp-lpae" --arch=armhf stretch "$ROOTFS" http://ftp.de.debian.org/debian/
                                # Hack to avoid debootstrap trying to mount /proc, as it will fail with "no permissions" and anyway proot takes care of it:
                                sed -i "s/setup_proc//g" "$ROOTFS/debootstrap/suite-script"
                                mychroot /debootstrap/debootstrap --second-stage --verbose http://ftp.de.debian.org/debian/
                        else
                                YESTERDAY=$(python -c 'import datetime ; print((datetime.datetime.now() - datetime.timedelta(days=1)).strftime("%Y%m%d"))')
                                wget -nc -q "https://uk.images.linuxcontainers.org/images/debian/stretch/armhf/default/${YESTERDAY}_22:42/rootfs.tar.xz"
                                tar -xf rootfs.tar.xz -C "$ROOTFS/" || true
                                echo "nameserver 8.8.8.8" > "$ROOTFS/etc/resolv.conf"
                        fi
                        mychroot -b /dev apt-get update
                        mychroot apt-get -y install build-essential dh-autoreconf pkg-config libuhd-dev libusb-1.0-0-dev libusb-dev git libtalloc-dev libgnutls28-dev stow
                fi
                # Run jenkins.sh inside the chroot:
                INSIDE_CHROOT=1 mychroot_nocwd \
                 -w /osmo-trx \
                 -b "$OSMOTRX_DIR:/osmo-trx" \
                 -b "$(which osmo-clean-workspace.sh):/usr/bin/osmo-clean-workspace.sh" \
                 -b "$(which osmo-build-dep.sh):/usr/bin/osmo-build-dep.sh" \
                 -b "$(which osmo-deps.sh):/usr/bin/osmo-deps.sh" \
                  ./contrib/jenkins.sh
                exit 0
        fi
fi

set -ex

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

CONFIG="--enable-sanitize --enable-werror --with-uhd --with-usrp1 --with-lms --with-ipc $INSTR"

# Additional configure options and depends
if [ "$WITH_MANUALS" = "1" ]; then
	osmo-build-dep.sh osmo-gsm-manuals
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
autoreconf --install --force
./configure $CONFIG
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh
DISTCHECK_CONFIGURE_FLAGS="$CONFIG" $MAKE distcheck \
  || cat-testlogs.sh

if [ "$WITH_MANUALS" = "1" ] && [ "$PUBLISH" = "1" ]; then
	make -C "$base/doc/manuals" publish
fi

$MAKE maintainer-clean

# Verify distro-specific package patches apply:
for patch in debian/patches/*.patch; do
        patch --dry-run -p1 < "$patch"
done

osmo-clean-workspace.sh
