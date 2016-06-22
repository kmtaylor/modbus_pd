AC_INIT([modbus_pd], [0.1])
AC_CONFIG_AUX_DIR([config])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_HEADERS([config/config.h])

AM_INIT_AUTOMAKE([-Wall])
AM_SILENT_RULES([yes])

AM_PROG_AR
LT_INIT
AC_PROG_CC
AM_PROG_CC_C_O

PKG_CHECK_MODULES([MODBUS], [libmodbus])
PKG_CHECK_MODULES([PD], [pd])
PKG_CHECK_VAR([PD_EXT_SONAME],	[pd], [extension])
PKG_CHECK_VAR([PD_EXT_DIR],	[pd], [externaldir])

AC_CONFIG_FILES([
    Makefile
    src/Makefile
])

AC_OUTPUT
