#include <stdio.h>
#include <stdlib.h>

#include <modbus-rtu.h>

#include "modbus-regs.h"

#define MODBUS_DEBUG	TRUE
#define MODBUS_SLAVE	1
#define MODBUS_UART	"/dev/ttyACM0"

int main(void) {
    modbus_t *ctx;
    uint16_t version[MB_VERSION_REGS];
    int bytes;
    uint8_t slave_id[MODBUS_RTU_MAX_ADU_LENGTH];

    if (!(ctx = modbus_new_rtu(MODBUS_UART, 115200, 'N', 8, 1))) {
	fprintf(stderr, "Error initialising modbus library\n");
	exit(EXIT_FAILURE);
    }

    modbus_set_debug(ctx, MODBUS_DEBUG);

    /* Broadcast address is 0 (MODBUS_BROADCAST_ADDRESS) */
    modbus_set_slave(ctx, MODBUS_SLAVE);

    if (modbus_connect(ctx) < 0) {
	fprintf(stderr, "Unable to open serial port\n");
	exit(EXIT_FAILURE);
    }

    if ((bytes = modbus_report_slave_id(ctx, MODBUS_RTU_MAX_ADU_LENGTH, slave_id)) < 0) {
	fprintf(stderr, "Unable to read slave ID\n");
	exit(EXIT_FAILURE);
    }

    if (modbus_read_registers(ctx, MB_VERSION, MB_VERSION_REGS, version) < 0) {
	fprintf(stderr, "Unable to read register\n");
	exit(EXIT_FAILURE);
    }

    printf("Slave ID: %s\n", &slave_id[2]);
    printf("Version: %x\n", version[0] << 16 | version[1]);

    return 0;
}
