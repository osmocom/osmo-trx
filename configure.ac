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
	m4_esyscmd([./git-version-gen .tarball-veresion]),
	[openbsc@lists.osmocom.org])
AC_PREREQ(2.57)
AC_CONFIG_SRCDIR([Transceiver52M/Makefile.am])
AC_CONFIG_AUX_DIR([.])
AC_CONFIG_MACRO_DIR([config])
AM_CONFIG_HEADER(config.h)

AC_CANONICAL_BUILD
AC_CANONICAL_HOST
AC_CANONICAL_TARGET

AM_INIT_AUTOMAKE([subdir-objects])

dnl Linux kernel KBuild style compile messages
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

dnl include release helper
RELMAKE='-include osmo-release.mk'
AC_SUBST([RELMAKE])

AM_PROG_AS
AC_PROG_CXX
AX_CXX_COMPILE_STDCXX_11
AC_PROG_LN_S
AC_PROG_MAKE_SET
AC_PROG_INSTALL
AC_PATH_PROG([RM_PROG], [rm])

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

AC_ARG_WITH(usrp1, [
    AS_HELP_STRING([--with-usrp1],
        [enable USRP1 gnuradio based transceiver])
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
    PKG_CHECK_MODULES(USRP, usrp >= 3.3)
])

AS_IF([test "x$with_usrp1" != "xyes"],[
    PKG_CHECK_MODULES(UHD, uhd >= 003.011,
        [AC_DEFINE(USE_UHD_3_11, 1, UHD version 3.11.0 or higher)],
        [PKG_CHECK_MODULES(UHD, uhd >= 003.009,
            [AC_DEFINE(USE_UHD_3_9, 1, UHD version 3.9.0 or higher)],
            [PKG_CHECK_MODULES(UHD, uhd >= 003.005)]
        )]
    )
    AC_DEFINE(USE_UHD, 1, All UHD versions)
    PKG_CHECK_MODULES(FFTWF, fftw3f)
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
    [osmo_cv_cc_has_builtin], [
      AC_LINK_IFELSE([
        AC_LANG_PROGRAM([], [
          __builtin_cpu_supports("sse");
        ])
      ],
      [AS_VAR_SET([osmo_cv_cc_has_builtin], [yes])],
      [AS_VAR_SET([osmo_cv_cc_has_builtin], [no])])
    ]
  )

  AS_IF([test yes = AS_VAR_GET([osmo_cv_cc_has_builtin])], [
    AC_DEFINE_UNQUOTED(AS_TR_CPP(HAVE_$1), 1,
      [Define to 1 if compiler has the '$1' built-in function])
  ], [
    AC_MSG_WARN($2)
  ])
])

dnl Check if the compiler supports runtime SIMD detection
CHECK_BUILTIN_SUPPORT([__builtin_cpu_supports],
  [Runtime SIMD detection will be disabled])

AM_CONDITIONAL(USRP1, [test "x$with_usrp1" = "xyes"])
AM_CONDITIONAL(ARCH_ARM, [test "x$with_neon" = "xyes" || test "x$with_neon_vfpv4" = "xyes"])
AM_CONDITIONAL(ARCH_ARM_A15, [test "x$with_neon_vfpv4" = "xyes"])

PKG_CHECK_MODULES(LIBUSB, libusb-1.0)

AC_CHECK_HEADER([boost/config.hpp],[],
    [AC_MSG_ERROR([boost/config.hpp not found, install e.g. libboost-dev])])

dnl Output files
AC_CONFIG_FILES([\
    Makefile \
    CommonLibs/Makefile \
    GSM/Makefile \
    Transceiver52M/Makefile \
    Transceiver52M/arm/Makefile \
    Transceiver52M/x86/Makefile \
])

AC_OUTPUT
