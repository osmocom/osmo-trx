dnl
dnl Copyright 2008, 2009, 2010 Free Software Foundation, Inc.
dnl
dnl This software is distributed under the terms of the GNU Public License.
dnl See the COPYING file in the main directory for details.
dnl
dnl This program is free software: you can redistribute it and/or modify
dnl it under the terms of the GNU General Public License as published by
dnl the Free Software Foundation, either version 3 of the License, or
dnl (at your option) any later version.
dnl
dnl This program is distributed in the hope that it will be useful,
dnl but WITHOUT ANY WARRANTY; without even the implied warranty of
dnl MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
dnl GNU General Public License for more details.
dnl
dnl You should have received a copy of the GNU General Public License
dnl along with this program.  If not, see <http://www.gnu.org/licenses/>.
dnl

AC_INIT([osmo-trx],
	m4_esyscmd([./git-version-gen .tarball-version]),
	[openbsc@lists.osmocom.org])
AC_PREREQ(2.57)
AC_CONFIG_SRCDIR([Transceiver52M/Makefile.am])
AC_CONFIG_AUX_DIR([.])
AC_CONFIG_MACRO_DIR([config])
AM_CONFIG_HEADER(config.h)
AC_CONFIG_TESTDIR(tests)

AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_CANONICAL_TARGET

AM_INIT_AUTOMAKE([foreign subdir-objects])

dnl Linux kernel KBuild style compile messages
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

dnl include release helper
RELMAKE='-include osmo-release.mk'
AC_SUBST([RELMAKE])

AM_PROG_AS
AC_PROG_CC
AC_PROG_CXX
AX_CXX_COMPILE_STDCXX_11
AC_PROG_LN_S
AC_PROG_MAKE_SET
AC_PROG_INSTALL
AC_PATH_PROG([RM_PROG], [rm])
AC_LANG([C++])

dnl check for pkg-config (explained in detail in libosmocore/configure.ac)
AC_PATH_PROG(PKG_CONFIG_INSTALLED, pkg-config, no)
if test "x$PKG_CONFIG_INSTALLED" = "xno"; then
        AC_MSG_WARN([You need to install pkg-config])
fi
PKG_PROG_PKG_CONFIG([0.20])

AC_LIBTOOL_WIN32_DLL
AC_ENABLE_SHARED	dnl do build shared libraries
AC_DISABLE_STATIC	dnl don't build static libraries
AC_PROG_LIBTOOL

dnl Checks for header files.
AC_HEADER_STDC
dnl This is required for GnuRadio includes to understand endianess correctly:
AC_CHECK_HEADERS([byteswap.h])

dnl Checks for typedefs, structures, and compiler characteristics.
AC_C_CONST
AC_C_INLINE
AC_TYPE_SIZE_T
AC_HEADER_TIME
AC_C_BIGENDIAN

PKG_CHECK_MODULES(LIBOSMOCORE, libosmocore >= 0.12.0)
PKG_CHECK_MODULES(LIBOSMOVTY, libosmovty >= 0.12.0)
PKG_CHECK_MODULES(LIBOSMOCTRL, libosmoctrl >= 0.12.0)

AC_ARG_ENABLE(sanitize,
	[AS_HELP_STRING(
		[--enable-sanitize],
		[Compile with address sanitizer enabled],
	)],
	[sanitize=$enableval], [sanitize="no"])
if test x"$sanitize" = x"yes"
then
	CPPFLAGS="$CPPFLAGS -fsanitize=address -fsanitize=undefined"
	LDFLAGS="$LDFLAGS -fsanitize=address -fsanitize=undefined"
fi

AC_ARG_ENABLE(werror,
	[AS_HELP_STRING(
		[--enable-werror],
		[Turn all compiler warnings into errors, with exceptions:
		 a) deprecation (allow upstream to mark deprecation without breaking builds);
		 b) "#warning" pragmas (allow to remind ourselves of errors without breaking builds)
		]
	)],
	[werror=$enableval], [werror="no"])
if test x"$werror" = x"yes"
then
	WERROR_FLAGS="-Werror"
	WERROR_FLAGS+=" -Wno-error=deprecated -Wno-error=deprecated-declarations"
	WERROR_FLAGS+=" -Wno-error=cpp" # "#warning"
	CFLAGS="$CFLAGS $WERROR_FLAGS"
	CPPFLAGS="$CPPFLAGS $WERROR_FLAGS"
fi


AC_ARG_WITH(uhd, [
    AS_HELP_STRING([--with-uhd],
        [enable UHD based transceiver])
])

AC_ARG_WITH(usrp1, [
    AS_HELP_STRING([--with-usrp1],
        [enable USRP1 gnuradio based transceiver])
])

AC_ARG_WITH(lms, [
    AS_HELP_STRING([--with-lms],
        [enable LimeSuite based transceiver])
])

AC_ARG_WITH(xtrx, [
    AS_HELP_STRING([--with-xtrx],
        [enable XTRX based transceiver])
])

AC_ARG_WITH(singledb, [
    AS_HELP_STRING([--with-singledb],
        [enable single daughterboard use on USRP1])
])

AC_ARG_WITH(neon, [
    AS_HELP_STRING([--with-neon],
        [enable ARM NEON support])
])

AC_ARG_WITH(neon-vfpv4, [
    AS_HELP_STRING([--with-neon-vfpv4],
        [enable ARM NEON FMA support])
])

AC_ARG_WITH(sse, [
    AS_HELP_STRING([--with-sse],
        [enable x86 SSE support (default)])
])

AS_IF([test "x$with_neon" = "xyes"], [
    AC_DEFINE(HAVE_NEON, 1, Support ARM NEON)
])

AS_IF([test "x$with_neon_vfpv4" = "xyes"], [
    AC_DEFINE(HAVE_NEON, 1, Support ARM NEON)
    AC_DEFINE(HAVE_NEON_FMA, 1, Support ARM NEON with FMA)
])

AS_IF([test "x$with_usrp1" = "xyes"], [
    AC_CHECK_HEADER([boost/config.hpp],[],
        [AC_MSG_ERROR([boost/config.hpp not found, install e.g. libboost-dev])])
    PKG_CHECK_MODULES(USRP, usrp >= 3.3)
])

AS_IF([test "x$with_lms" = "xyes"], [
    PKG_CHECK_MODULES(LMS, LimeSuite)
])

AS_IF([test "x$with_xtrx" = "xyes"], [
    PKG_CHECK_MODULES(XTRX, libxtrx)
])

AS_IF([test "x$with_uhd" != "xno"],[
    PKG_CHECK_MODULES(UHD, uhd >= 003.011,
        [AC_DEFINE(USE_UHD_3_11, 1, UHD version 3.11.0 or higher)],
        [PKG_CHECK_MODULES(UHD, uhd >= 003.009,
            [AC_DEFINE(USE_UHD_3_9, 1, UHD version 3.9.0 or higher)],
            [PKG_CHECK_MODULES(UHD, uhd >= 003.005)]
        )]
    )
])

AS_IF([test "x$with_singledb" = "xyes"], [
    AC_DEFINE(SINGLEDB, 1, Define to 1 for single daughterboard)
])

# Find and define supported SIMD extensions
AS_IF([test "x$with_sse" != "xno"], [
    AX_SSE
], [
    AM_CONDITIONAL(HAVE_SSE3, false)
    AM_CONDITIONAL(HAVE_SSE4_1, false)
])

dnl Check if the compiler supports specified GCC's built-in function
AC_DEFUN([CHECK_BUILTIN_SUPPORT], [
  AC_CACHE_CHECK(
    [whether ${CC} has $1 built-in],
    [osmo_cv_cc_has_$1], [
      AC_LINK_IFELSE([
        AC_LANG_PROGRAM([], [
          $2
        ])
      ],
      [AS_VAR_SET([osmo_cv_cc_has_$1], [yes])],
      [AS_VAR_SET([osmo_cv_cc_has_$1], [no])])
    ]
  )

  AS_IF([test yes = AS_VAR_GET([osmo_cv_cc_has_$1])], [
    AC_DEFINE_UNQUOTED(AS_TR_CPP(HAVE_$1), 1,
      [Define to 1 if compiler has the '$1' built-in function])
  ], [
    AC_MSG_WARN($3)
  ])
])

dnl Check if the compiler supports runtime SIMD detection
CHECK_BUILTIN_SUPPORT([__builtin_cpu_supports], [__builtin_cpu_supports("sse");],
  [Runtime SIMD detection will be disabled])

dnl Check for __sync_fetch_and_add().
CHECK_BUILTIN_SUPPORT([__sync_fetch_and_and], [int x;__sync_fetch_and_and(&x,1);],
  [Atomic operation not available, will use mutex])
dnl Check for __sync_or_and_fetch().
CHECK_BUILTIN_SUPPORT([__sync_or_and_fetch], [int x;__sync_or_and_fetch(&x,1);],
  [Atomic operation not available, will use mutex])
AS_IF([test "x$osmo_cv_cc_has___sync_fetch_and_and" = "xyes" && test "x$osmo_cv_cc_has___sync_or_and_fetch" = "xyes"], [
    AC_DEFINE(HAVE_ATOMIC_OPS, 1, [Support all required atomic operations], [AC_MSG_WARN("At least one aotmic operation missing, will use mutex")])
])

AM_CONDITIONAL(DEVICE_UHD, [test "x$with_uhd" != "xno"])
AM_CONDITIONAL(DEVICE_USRP1, [test "x$with_usrp1" = "xyes"])
AM_CONDITIONAL(DEVICE_LMS, [test "x$with_lms" = "xyes"])
AM_CONDITIONAL(DEVICE_XTRX, [test "x$with_xtrx" = "xyes"])
AM_CONDITIONAL(ARCH_ARM, [test "x$with_neon" = "xyes" || test "x$with_neon_vfpv4" = "xyes"])
AM_CONDITIONAL(ARCH_ARM_A15, [test "x$with_neon_vfpv4" = "xyes"])

PKG_CHECK_MODULES(LIBUSB, libusb-1.0)
PKG_CHECK_MODULES(FFTWF, fftw3f)

# Generate manuals
AC_ARG_ENABLE(manuals,
	[AS_HELP_STRING(
		[--enable-manuals],
		[Generate manual PDFs [default=no]],
	)],
	[osmo_ac_build_manuals=$enableval], [osmo_ac_build_manuals="no"])
AM_CONDITIONAL([BUILD_MANUALS], [test x"$osmo_ac_build_manuals" = x"yes"])
AC_ARG_VAR(OSMO_GSM_MANUALS_DIR, [path to common osmo-gsm-manuals files, overriding pkg-config and "../osmo-gsm-manuals"
	fallback])
if test x"$osmo_ac_build_manuals" = x"yes"
then
	# Find OSMO_GSM_MANUALS_DIR (env, pkg-conf, fallback)
	if test -n "$OSMO_GSM_MANUALS_DIR"; then
		echo "checking for OSMO_GSM_MANUALS_DIR... $OSMO_GSM_MANUALS_DIR (from env)"
	else
		OSMO_GSM_MANUALS_DIR="$($PKG_CONFIG osmo-gsm-manuals --variable=osmogsmmanualsdir 2>/dev/null)"
		if test -n "$OSMO_GSM_MANUALS_DIR"; then
			echo "checking for OSMO_GSM_MANUALS_DIR... $OSMO_GSM_MANUALS_DIR (from pkg-conf)"
		else
			OSMO_GSM_MANUALS_DIR="../osmo-gsm-manuals"
			echo "checking for OSMO_GSM_MANUALS_DIR... $OSMO_GSM_MANUALS_DIR (fallback)"
		fi
	fi
	if ! test -d "$OSMO_GSM_MANUALS_DIR"; then
		AC_MSG_ERROR("OSMO_GSM_MANUALS_DIR does not exist! Install osmo-gsm-manuals or set OSMO_GSM_MANUALS_DIR.")
	fi

	# Find and run check-depends
	CHECK_DEPENDS="$OSMO_GSM_MANUALS_DIR/check-depends.sh"
	if ! test -x "$CHECK_DEPENDS"; then
		CHECK_DEPENDS="osmo-gsm-manuals-check-depends"
	fi
	if ! $CHECK_DEPENDS; then
		AC_MSG_ERROR("missing dependencies for --enable-manuals")
	fi

	# Put in Makefile with absolute path
	OSMO_GSM_MANUALS_DIR="$(realpath "$OSMO_GSM_MANUALS_DIR")"
	AC_SUBST([OSMO_GSM_MANUALS_DIR])
fi

    # https://www.freedesktop.org/software/systemd/man/daemon.html
    AC_ARG_WITH([systemdsystemunitdir],
         [AS_HELP_STRING([--with-systemdsystemunitdir=DIR], [Directory for systemd service files])],,
         [with_systemdsystemunitdir=auto])
    AS_IF([test "x$with_systemdsystemunitdir" = "xyes" -o "x$with_systemdsystemunitdir" = "xauto"], [
         def_systemdsystemunitdir=$($PKG_CONFIG --variable=systemdsystemunitdir systemd)

         AS_IF([test "x$def_systemdsystemunitdir" = "x"],
       [AS_IF([test "x$with_systemdsystemunitdir" = "xyes"],
        [AC_MSG_ERROR([systemd support requested but pkg-config unable to query systemd package])])
        with_systemdsystemunitdir=no],
       [with_systemdsystemunitdir="$def_systemdsystemunitdir"])])
    AS_IF([test "x$with_systemdsystemunitdir" != "xno"],
          [AC_SUBST([systemdsystemunitdir], [$with_systemdsystemunitdir])])
    AM_CONDITIONAL([HAVE_SYSTEMD], [test "x$with_systemdsystemunitdir" != "xno"])

AC_MSG_RESULT([CPPFLAGS="$CPPFLAGS"])
AC_MSG_RESULT([CFLAGS="$CFLAGS"])
AC_MSG_RESULT([CXXFLAGS="$CXXFLAGS"])
AC_MSG_RESULT([LDFLAGS="$LDFLAGS"])

dnl Output files
AC_CONFIG_FILES([\
    Makefile \
    CommonLibs/Makefile \
    GSM/Makefile \
    Transceiver52M/Makefile \
    Transceiver52M/arch/Makefile \
    Transceiver52M/arch/common/Makefile \
    Transceiver52M/arch/arm/Makefile \
    Transceiver52M/arch/x86/Makefile \
    Transceiver52M/device/Makefile \
    Transceiver52M/device/common/Makefile \
    Transceiver52M/device/uhd/Makefile \
    Transceiver52M/device/usrp1/Makefile \
    Transceiver52M/device/lms/Makefile \
    Transceiver52M/device/xtrx/Makefile \
    tests/Makefile \
    tests/CommonLibs/Makefile \
    tests/Transceiver52M/Makefile \
    doc/Makefile \
    doc/examples/Makefile \
    contrib/Makefile \
    contrib/systemd/Makefile \
])

AC_OUTPUT(
	doc/manuals/Makefile)
