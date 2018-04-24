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

SUBDIRS = arch

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I${srcdir}/arch/common
AM_CXXFLAGS = -lpthread $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCTRL_CFLAGS) $(LIBOSMOVTY_CFLAGS)

if USRP1
AM_CPPFLAGS += $(USRP_CFLAGS)
else
AM_CPPFLAGS += $(UHD_CFLAGS)
endif

rev2dir = $(datadir)/usrp/rev2
rev4dir = $(datadir)/usrp/rev4

dist_rev2_DATA = std_inband.rbf
dist_rev4_DATA = std_inband.rbf

EXTRA_DIST = \
	README \
	README.Talgorithm

noinst_LTLIBRARIES = libtransceiver.la

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

libtransceiver_la_SOURCES = \
	$(COMMON_SOURCES) \
	Resampler.cpp \
	radioInterfaceResamp.cpp \
	radioInterfaceMulti.cpp

bin_PROGRAMS = osmo-trx

noinst_HEADERS = \
	Complex.h \
	radioInterface.h \
	radioVector.h \
	radioClock.h \
	radioDevice.h \
	radioBuffer.h \
	sigProcLib.h \
	signalVector.h \
	Transceiver.h \
	USRPDevice.h \
	Resampler.h \
	ChannelizerBase.h \
	Channelizer.h \
	Synthesis.h

osmo_trx_SOURCES = osmo-trx.cpp
osmo_trx_LDADD = \
	libtransceiver.la \
	$(ARCH_LA) \
	$(GSM_LA) \
	$(COMMON_LA) \
	$(FFTWF_LIBS) \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOVTY_LIBS)

if USRP1
libtransceiver_la_SOURCES += USRPDevice.cpp
osmo_trx_LDADD += $(USRP_LIBS)
else
libtransceiver_la_SOURCES += UHDDevice.cpp
osmo_trx_LDADD += $(UHD_LIBS)
endif
