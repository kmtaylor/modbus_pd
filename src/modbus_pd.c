/* modbus -- Wrapper for libmodbus
 * Copyright 2016 Kim Taylor
 * BSD license; see README.txt in this distribution for details.
 */

#define MODBUS_DEBUG FALSE

#include "m_pd.h"
#include <stdio.h>
#include <string.h>
#include <alloca.h>

#include <modbus-rtu.h>
#include "modbus-regs.h"

#define LIST_NGETBYTE 100
#define LIST_ALLOCA(type, x, n) ((x) = (type *)((n) < LIST_NGETBYTE ?  \
	alloca((n) * sizeof(type)) : getbytes((n) * sizeof(type))))
#define LIST_FREEA(type, x, n) ( \
	((n) < LIST_NGETBYTE || (freebytes((x), (n) * sizeof(type)), 0)))

static t_class *modbus_class;
static t_symbol *recent_tty;

typedef struct {
    t_object x_obj;

    t_symbol *tty;
    int baud;

    modbus_t *ctx;

    uint8_t address;
    uint16_t first_reg_no;
    uint16_t last_reg_no;
    int num_regs;
} t_modbus_class;

static void *modbus_class_new(t_floatarg address, t_floatarg first_reg_no,
		t_floatarg last_reg_no) {
    t_modbus_class *x = (t_modbus_class *) pd_new(modbus_class);

    x->tty = NULL;
    x->ctx = NULL;
    x->baud = 115200;
    x->address = (uint16_t) address;
    x->first_reg_no = (uint16_t) first_reg_no;
    x->last_reg_no = (uint16_t) last_reg_no;
    x->num_regs = x->last_reg_no ? x->last_reg_no - x->first_reg_no + 1 : 1;

    outlet_new(&x->x_obj, &s_list);

    return x;
}

static void modbus_send(t_modbus_class *x, uint16_t *reg_data) {
    int i;

    if (!x->ctx) {
	if (!recent_tty) {
	    pd_error(x, "Modbus port not open");
	    return;
	}
	post("Modbus port not open, using most recent TTY");
	x->ctx = (modbus_t *) recent_tty->s_thing;
    }

    modbus_set_slave(x->ctx, x->address);

    if (x->last_reg_no) {
	if (modbus_write_registers(x->ctx, x->first_reg_no, x->num_regs,
			    reg_data) < 0) {
	    pd_error(x, "Unable to write multiple registers");
	}
	return;
    }

    if (modbus_write_register(x->ctx, x->first_reg_no, reg_data[0]) < 0) {
	pd_error(x, "Unable to write single register");
    }
}

static void modbus_class_list(t_modbus_class *x, t_symbol *s,
		int ac, t_atom *av) {
    int i;
    uint16_t *reg_data;

    LIST_ALLOCA(uint16_t, reg_data, x->num_regs);

    for (i = 0; i < x->num_regs; i++) {
	reg_data[i] = atom_getfloat(av);
	if (ac > 1) {
	    ac--;
	    av++;
	}
    }

    modbus_send(x, reg_data);

    LIST_FREEA(uint16_t, reg_data, x->num_regs);
}

static void modbus_class_bang(t_modbus_class *x) {
    uint16_t val[x->num_regs];
    t_atom *outv;
    int i;

    if (!x->ctx) {
	if (!recent_tty) {
	    pd_error(x, "Modbus port not open");
	    return;
	}
	post("Modbus port not open, using most recent TTY");
	x->ctx = (modbus_t *) recent_tty->s_thing;
    }

    modbus_set_slave(x->ctx, x->address);
    if (modbus_read_registers(x->ctx, x->first_reg_no, x->num_regs, val) < 0) {
	pd_error(x, "Unimplemented register");
	return;
    }

    LIST_ALLOCA(t_atom, outv, x->num_regs);
    for (i = 0; i < x->num_regs; i++)
	SETFLOAT(outv + i, val[i]);
    outlet_list(x->x_obj.ob_outlet, &s_list, x->num_regs, outv);
    LIST_FREEA(t_atom, outv, x->num_regs);
}

static void modbus_class_open(t_modbus_class *x, t_symbol *s) {
    x->tty = s;
    uint8_t slave_id[MODBUS_RTU_MAX_ADU_LENGTH];
    uint16_t version[MB_VERSION_REGS];

    if (s->s_thing) {
	post("TTY already open");
	x->ctx = (modbus_t *) s->s_thing;
    } else {
	post("Opening modbus TTY %s", s->s_name);
    
	if (!(x->ctx = modbus_new_rtu(s->s_name, x->baud, 'N', 8, 1))) {
	    pd_error(x, "Error initialising modbus library");
	    return;
	}

	if (modbus_connect(x->ctx) < 0) {
	    pd_error(x, "Unable to open serial port:");
	    return;
	}

	s->s_thing = (void *) x->ctx;
	recent_tty = s;
    }

    modbus_set_slave(x->ctx, x->address);
    if (modbus_report_slave_id(x->ctx, slave_id) < 0) {
	pd_error(x, "Unable to retrieve ID string");
	return;
    }

    if (modbus_read_registers(x->ctx,
			    MB_VERSION, MB_VERSION_REGS, version) < 0) {
	pd_error(x, "Unable to retrieve version number");
	return;
    }

    post("Slave ID: %s", &slave_id[2]);
    post("Firmware version: %x", version[0] << 16 | version[1]);
}

static void modbus_class_baud(t_modbus_class *x, t_float f) {
    pd_error(x, "Not implemented");
}

static void modbus_class_free(t_modbus_class *x) {
    /* Data space is automatically freed */
}

void modbus_setup(void) {
    modbus_class = class_new(gensym("modbus"),
		    (t_newmethod) modbus_class_new,	/* Class constructor */
		    (t_method) modbus_class_free,	/* Class destructor */
		    sizeof(t_modbus_class),		/* Class data space */
		    CLASS_DEFAULT,			/* Class flags */
		    A_DEFFLOAT, A_DEFFLOAT,		/* Class atom type(s) */
		    A_DEFFLOAT, A_NULL);

    class_addbang(modbus_class, modbus_class_bang);
    class_addlist(modbus_class, modbus_class_list);

    class_addmethod(modbus_class, (t_method) modbus_class_open, gensym("open"),
		    A_SYMBOL, A_NULL);
    class_addmethod(modbus_class, (t_method) modbus_class_baud, gensym("baud"),
		    A_FLOAT, A_NULL);
}
