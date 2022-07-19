#
# Copyright 2008 Free Software Foundation, Inc.
# Copyright 2010 Range Networks, Inc.
#
# This software is distributed under the terms of the GNU Public License.
# See the COPYING file in the main directory for details.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

include $(top_srcdir)/Makefile.common

SUBDIRS = arch device

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/arch/common -I${srcdir}/device/common
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS) -mcpu=cortex-a72 -mfloat-abi=hard -mfpu=neon-fp-armv8
AM_CFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS) -mcpu=cortex-a72 -mfloat-abi=hard -mfpu=neon-fp-armv8

noinst_LTLIBRARIES = libtransceiver_common.la

COMMON_SOURCES = \
	l1if.cpp \
	radioInterface.cpp \
	radioVector.cpp \
	radioClock.cpp \
	radioBuffer.cpp \
	sigProcLib.cpp \
	signalVector.cpp \
	Transceiver.cpp \
	ChannelizerBase.cpp \
	Channelizer.cpp \
	Synthesis.cpp \
	proto_trxd.c \
	sch.c \
	grgsm_vitac/grgsm_vitac.cpp \
	grgsm_vitac/viterbi_detector.cc

libtransceiver_common_la_SOURCES = \
	$(COMMON_SOURCES) \
	Resampler.cpp \
	radioInterfaceResamp.cpp \
	radioInterfaceMulti.cpp

noinst_HEADERS = \
	Complex.h \
	radioInterface.h \
	radioVector.h \
	radioClock.h \
	radioBuffer.h \
	sigProcLib.h \
	signalVector.h \
	Transceiver.h \
	Resampler.h \
	ChannelizerBase.h \
	Channelizer.h \
	Synthesis.h \
	proto_trxd.h \
	grgsm_vitac/viterbi_detector.h \
	grgsm_vitac/constants.h

COMMON_LDADD = \
	libtransceiver_common.la \
	$(ARCH_LA) \
	$(GSM_LA) \
	$(COMMON_LA) \
	$(FFTWF_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCODING_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOVTY_LIBS)

bin_PROGRAMS =

if DEVICE_UHD
bin_PROGRAMS += osmo-trx-uhd
osmo_trx_uhd_SOURCES = osmo-trx.cpp
osmo_trx_uhd_LDADD = \
	$(builddir)/device/uhd/libdevice.la \
	$(COMMON_LDADD) \
	$(UHD_LIBS)
osmo_trx_uhd_CPPFLAGS  = $(AM_CPPFLAGS) $(UHD_CFLAGS)

bin_PROGRAMS += osmo-trx-ms-uhd
osmo_trx_ms_uhd_SOURCES = ms/syncthing.cpp ms/ms_rx_lower.cpp ms/ms_rx_upper.cpp ms/ms_commandhandler.cpp
osmo_trx_ms_uhd_LDADD = \
	$(builddir)/device/bladerf/libdevice.la \
	$(COMMON_LDADD) \
	$(UHD_LIBS) \
	$(TRXCON_LA)
osmo_trx_ms_uhd_CPPFLAGS  = $(AM_CPPFLAGS) $(UHD_CFLAGS) -DBUILDUHD

bin_PROGRAMS += osmo-trx-syncthing-uhd
osmo_trx_syncthing_uhd_SOURCES = ms/syncthing.cpp ms/ms_rx_lower.cpp ms/ms_rx_burst.cpp
osmo_trx_syncthing_uhd_LDADD = \
	$(builddir)/device/bladerf/libdevice.la \
	$(COMMON_LDADD) \
	$(UHD_LIBS)
osmo_trx_syncthing_uhd_CPPFLAGS  = $(AM_CPPFLAGS) $(UHD_CFLAGS) -DSYNCTHINGONLY -DBUILDUHD
#osmo_trx_syncthing_LDFLAGS  = -fsanitize=address,undefined -shared-libsan
endif

if DEVICE_USRP1
bin_PROGRAMS += osmo-trx-usrp1
osmo_trx_usrp1_SOURCES = osmo-trx.cpp
osmo_trx_usrp1_LDADD = \
	$(builddir)/device/usrp1/libdevice.la \
	$(COMMON_LDADD) \
	$(USRP_LIBS)
osmo_trx_usrp1_CPPFLAGS  = $(AM_CPPFLAGS) $(USRP_CFLAGS)
endif

if DEVICE_LMS
bin_PROGRAMS += osmo-trx-lms
osmo_trx_lms_SOURCES = osmo-trx.cpp
osmo_trx_lms_LDADD = \
	$(builddir)/device/lms/libdevice.la \
	$(COMMON_LDADD) \
	$(LMS_LIBS)
osmo_trx_lms_CPPFLAGS  = $(AM_CPPFLAGS) $(LMS_CFLAGS)
endif

if DEVICE_BLADE
bin_PROGRAMS += osmo-trx-blade
osmo_trx_blade_SOURCES = osmo-trx.cpp
osmo_trx_blade_LDADD = \
	$(builddir)/device/bladerf/libdevice.la \
	$(COMMON_LDADD) \
	$(BLADE_LIBS)
osmo_trx_blade_CPPFLAGS  = $(AM_CPPFLAGS) $(LMS_CFLAGS)

bin_PROGRAMS += osmo-trx-ms-blade
osmo_trx_ms_blade_SOURCES = ms/syncthing.cpp ms/ms_rx_lower.cpp ms/ms_rx_upper.cpp ms/ms_commandhandler.cpp
osmo_trx_ms_blade_LDADD = \
	$(builddir)/device/bladerf/libdevice.la \
	$(COMMON_LDADD) \
	$(BLADE_LIBS) \
	$(TRXCON_LA)
osmo_trx_ms_blade_CPPFLAGS  = $(AM_CPPFLAGS) $(BLADE_CFLAGS) -DBUILDBLADE

bin_PROGRAMS += osmo-trx-syncthing-blade
osmo_trx_syncthing_blade_SOURCES = ms/syncthing.cpp ms/ms_rx_lower.cpp ms/ms_rx_burst.cpp
osmo_trx_syncthing_blade_LDADD =  \
	$(builddir)/device/bladerf/libdevice.la \
	$(COMMON_LDADD) \
	$(BLADE_LIBS)
osmo_trx_syncthing_blade_CPPFLAGS  = $(AM_CPPFLAGS) $(BLADE_CFLAGS) -DSYNCTHINGONLY -DBUILDBLADE -mcpu=cortex-a72 -mfloat-abi=hard -mfpu=neon-fp-armv8 -I../device/ipc
#osmo_trx_syncthing_LDFLAGS  = -fsanitize=address,undefined -shared-libsan
endif

if DEVICE_IPC
bin_PROGRAMS += osmo-trx-ipc
osmo_trx_ipc_SOURCES = osmo-trx.cpp
osmo_trx_ipc_LDADD = \
	$(builddir)/device/ipc/libdevice.la \
	$(COMMON_LDADD)
osmo_trx_ipc_CPPFLAGS  = $(AM_CPPFLAGS)

bin_PROGRAMS += osmo-trx-ipc2
osmo_trx_ipc2_SOURCES = osmo-trx.cpp
osmo_trx_ipc2_LDADD = \
	$(builddir)/device/ipc2/libdevice.la \
	$(COMMON_LDADD)
osmo_trx_ipc2_CPPFLAGS  = $(AM_CPPFLAGS)

bin_PROGRAMS += osmo-trx-ms-ipc
osmo_trx_ms_ipc_SOURCES = ms/syncthing.cpp ms/ms_rx_lower.cpp ms/ms_rx_upper.cpp ms/ms_commandhandler.cpp
osmo_trx_ms_ipc_LDADD = \
	$(COMMON_LDADD) \
	$(TRXCON_LA)
osmo_trx_ms_ipc_CPPFLAGS  = $(AM_CPPFLAGS) -DBUILDIPC  -I./device/ipc2 -I../device/ipc2

bin_PROGRAMS += osmo-trx-syncthing-ipc
osmo_trx_syncthing_ipc_SOURCES = ms/syncthing.cpp ms/ms_rx_lower.cpp ms/ms_rx_burst.cpp
osmo_trx_syncthing_ipc_LDADD =  $(COMMON_LDADD)
osmo_trx_syncthing_ipc_CPPFLAGS  = $(AM_CPPFLAGS) -DSYNCTHINGONLY -DBUILDIPC  -I./device/ipc2 -I../device/ipc2
endif

noinst_HEADERS += \
	ms/syncthing.h \
	ms/bladerf_specific.h \
	ms/uhd_specific.h \
	ms/ms_rx_upper.h \
	ms/ms_state.h \
	itrq.h
# -fsanitize=address,undefined -shared-libsan -O0
#
