noinst_PROGRAMS = modbus_test

modbus_test_SOURCES = modbus_test.c
modbus_test_CFLAGS = $(AM_CFLAGS) $(MODBUS_CFLAGS)
modbus_test_LDADD = $(AM_LIBS) $(MODBUS_LIBS)

EXTRA_DIST = modbus-regs.h

# Puredata External format:
pd_external_LTLIBRARIES = modbus.la

modbus_la_SOURCES = modbus_pd.c
modbus_la_LDFLAGS = -module -avoid-version -shared -shrext .@PD_EXT_SONAME@
modbus_la_LIBADD = $(AM_LIBS) $(MODBUS_LIBS) $(PD_LIBS)
modbus_la_CFLAGS = $(AM_CFLAGS) $(MODBUS_CFLAGS) $(PD_CFLAGS)

pd_externaldir = $(PD_EXT_DIR)/modbus
