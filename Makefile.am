#
# Copyright 2008 Free Software Foundation, Inc.
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

ACLOCAL_AMFLAGS = -I config
AM_CPPFLAGS = $(STD_DEFINES_AND_INCLUDES) $(USB_INCLUDES) $(WITH_INCLUDES)
AM_CXXFLAGS = -Wall -pthread
#AM_CXXFLAGS = -Wall -O2 -NDEBUG -pthread
#AM_CFLAGS = -Wall -O2 -NDEBUG -pthread

SUBDIRS =

if ENABLE_MS_TRX
	SUBDIRS += osmocom-bb/src/host/trxcon
endif

# Order must be preserved
SUBDIRS += \
	osmocom-bb/src/host/trxcon \
	CommonLibs \
	GSM \
	Transceiver52M \
	contrib \
	tests \
	utils \
	doc \
	$(NULL)

EXTRA_DIST = \
	LEGAL \
	COPYING \
	README.md \
	contrib/osmo-trx.spec.in \
	debian \
	$(NULL)

AM_DISTCHECK_CONFIGURE_FLAGS = \
	--with-systemdsystemunitdir=$$dc_install_base/$(systemdsystemunitdir)

.PHONY: release

@RELMAKE@

dox: FORCE
	doxygen doxconfig

FORCE:
