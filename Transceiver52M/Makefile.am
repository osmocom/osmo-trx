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

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/arch/common -I${srcdir}/device
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)

rev2dir = $(datadir)/usrp/rev2
rev4dir = $(datadir)/usrp/rev4

dist_rev2_DATA = std_inband.rbf
dist_rev4_DATA = std_inband.rbf

EXTRA_DIST = \
	README \
	README.Talgorithm

noinst_LTLIBRARIES = libtransceiver_common.la

COMMON_SOURCES = \
	radioInterface.cpp \
	radioVector.cpp \
	radioClock.cpp \
	radioBuffer.cpp \
	sigProcLib.cpp \
	signalVector.cpp \
	Transceiver.cpp \
	ChannelizerBase.cpp \
	Channelizer.cpp \
	Synthesis.cpp

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
	Synthesis.h

COMMON_LDADD = \
	libtransceiver_common.la \
	$(ARCH_LA) \
	$(GSM_LA) \
	$(COMMON_LA) \
	$(FFTWF_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOVTY_LIBS)

bin_PROGRAMS = osmo-trx
osmo_trx_SOURCES = osmo-trx.cpp

if USRP1
osmo_trx_LDADD = \
	$(COMMON_LDADD) \
	$(USRP_LIBS) \
	$(builddir)/device/usrp1/libdevice.la
osmo_trx_CPPFLAGS  = $(AM_CPPFLAGS) $(USRP_CFLAGS)
else
osmo_trx_LDADD = \
	$(COMMON_LDADD) \
	$(UHD_LIBS) \
	$(builddir)/device/uhd/libdevice.la
osmo_trx_CPPFLAGS  = $(AM_CPPFLAGS) $(UHD_CFLAGS)
endif
