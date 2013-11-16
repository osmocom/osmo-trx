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

AM_CPPFLAGS = -Wall $(STD_DEFINES_AND_INCLUDES) -I./common
AM_CXXFLAGS = -ldl -lpthread

SUBDIRS = arm x86

if ARCH_ARM
ARCH_LA = arm/libarch.la
else
ARCH_LA = x86/libarch.la
endif

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
	sigProcLib.cpp \
	signalVector.cpp \
	Transceiver.cpp

libtransceiver_la_SOURCES = \
	$(COMMON_SOURCES) \
	Resampler.cpp \
	radioInterfaceResamp.cpp \
	radioInterfaceDiversity.cpp

bin_PROGRAMS = osmo-trx

noinst_HEADERS = \
	Complex.h \
	radioInterface.h \
	radioVector.h \
	radioClock.h \
	radioDevice.h \
	sigProcLib.h \
	signalVector.h \
	Transceiver.h \
	USRPDevice.h \
	Resampler.h \
	common/convolve.h \
	common/convert.h \
	common/scale.h \
	common/mult.h

osmo_trx_SOURCES = osmo-trx.cpp
osmo_trx_LDADD = \
	libtransceiver.la \
	$(ARCH_LA) \
	$(GSM_LA) \
	$(COMMON_LA) $(SQLITE_LA)

if USRP1 
libtransceiver_la_SOURCES += USRPDevice.cpp
osmo_trx_LDADD += $(USRP_LIBS)
else
libtransceiver_la_SOURCES += UHDDevice.cpp
osmo_trx_LDADD += $(UHD_LIBS)
endif
