/*  nutdrv_modbus.c - Driver for Modbus power devices (UPS, meters, ...)
 *
 *  Copyright (C)
 *  	2012-2019	Arnaud Quette <arnaud.quette@free.fr>
 * 		2019		Eaton
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * TODO list:
 * * make a generic engine out of it
 * * create a DMF format for Modbus, including
 * 		** to be completed...
 *  * doc, ... 
 *
 * "NUT Modbus definition file"		xxx.modbus
 * # A comment header may contain information such as
 * # - author and device information
 * # - default communication settings (baudrate, parity, data_bit, stop_bit)
 * device.type: <ups,meter,...>
 * [device.mfr: <Manufacturer name>]
 * [device.model: <Model name>]
 * <nut varname>: <modbus address>, <modbus register>, ..., <convertion function name>
 *
 */

#include "main.h"
#include <modbus.h>

#define DRIVER_NAME	"NUT Modbus driver"
#define DRIVER_VERSION	"0.03"

#define DEFAULT_MODBUS_TCP_PORT_STR	"502"

/* Variables */
modbus_t *ctx = NULL;

/* driver description structure */
upsdrv_info_t upsdrv_info = {
	DRIVER_NAME,
	DRIVER_VERSION,
	"Arnaud Quette <arnaud.quette@gmail.com>\n" \
	"Baptiste Guyard <guyardbaptiste@eaton.com>",
	DRV_EXPERIMENTAL,
	{ NULL }
};

/* modes to process data in modbus_info_t */
#define INFO_MODE_INIT   0
#define INFO_MODE_UPDATE 1

enum modbus_data_types {
	MODBUS_DATA_TYPE_FLOAT_ABCD = 1,
	MODBUS_DATA_TYPE_STRING
};

enum info_flags_t {
	INFO_FLAG_NONE = 0,
	INFO_FLAG_STATIC,
	INFO_FLAG_ABSENT,
	INFO_FLAG_OK
};

/* Mapping structure */
typedef struct {
	const char   *info_type;  /* INFO_ or CMD_ element */
	int           info_flags; /* flags to set in addinfo */
	double        info_len;   /* length of strings if ST_FLAG_STRING, multiplier otherwise. */
	int           modbus_register_nb;
	int           modbus_register_size;
	int           modbus_data_type;
	const char   *default_value;
	int flags;      /* driver internal flags */
} modbus_info_t;


/* Modbus to NUT lookup table for Eaton EMECMODB */
static modbus_info_t eaton_pwmeter_emecmodb[] = {

	/* Device collection */
/*
	{ "device.mfr", ST_FLAG_STRING, 6, 0, 0, MODBUS_DATA_TYPE_FLOAT_ABCD,
		"EATON", INFO_FLAG_STATIC | INFO_FLAG_ABSENT | INFO_FLAG_OK },
	{ "device.model", ST_FLAG_STRING, SU_INFOSIZE, 0, 0, MODBUS_DATA_TYPE_FLOAT_ABCD,
		"PX Meter EMECMODB", INFO_FLAG_STATIC | INFO_FLAG_ABSENT | INFO_FLAG_OK },
	{ "device.type", ST_FLAG_STRING, SU_INFOSIZE, 0, 0, MODBUS_DATA_TYPE_FLOAT_ABCD,
		"power-meter", INFO_FLAG_STATIC | INFO_FLAG_ABSENT | INFO_FLAG_OK },


	dstate_setinfo ("device.type", "power-meter");
 */
	/* FIXME:
	 * IMP Energy: input information, what comes from the Grid?
	 * */
	//	float Active Power 1st phase (W);
	{ "input.L1.realpower", 0, 1000, 4151, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//	float Active Power 2nd phase (W);
	{ "input.L2.realpower", 0, 1000, 4153, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//	float Active Power 3rd phase (W);
	{ "input.L3.realpower", 0, 1000, 4155, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//  Active Power - Sum of all phases (W);
	{ "input.realpower", 0, 1000, 4157, 4, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//L1-N voltage (V)
	{ "input.L1-N.voltage", 0, 1, 4267, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//L2-N voltage (V)
	{ "input.L2-N.voltage", 0, 1, 4269, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//L3-N voltage (V)
	{ "input.L3-N.voltage", 0, 1, 4271, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//L1-L2 voltage (V)
	{ "input.L1-L2.voltage", 0, 1, 4273, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//L2-L3 voltage (V)
	{ "input.L2-L3.voltage", 0, 1, 4275, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//L3-L1 voltage (V)
	{ "input.L3-L1.voltage", 0, 1, 4277, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//phase1 current (A)
	{ "input.L1.current", 0, 1, 4279, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//phase2 current (A)
	{ "input.L2.current", 0, 1, 4281, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//phase3 current (A)
	{ "input.L3.current", 0, 1, 4283, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//apparent power phase1 (VA)
	{ "input.L1.power", 0, 1000, 4285, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//apparent power phase2 (VA)
	{ "input.L2.power", 0, 1000, 4287, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//apparent power phase3 (VA)
	{ "input.L3.power", 0, 1000, 4289, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//apparent power - sum (VA)
	{ "input.power", 0, 1000, 4291, 4, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	//power factor cos phi phase1
	{ "input.L1.powerfactor", 0, 1, 4295, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	/* power factor cos phi phase2 */
	{ "input.L2.powerfactor", 0, 1, 4297, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	/* power factor cos phi phase3 */
	{ "input.L3.powerfactor", 0, 1, 4299, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	/* power factor cos phi - sum */
	{ "input.powerfactor", 0, 1, 4301, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },
	/* frequency (Hz) */
	{ "input.frequency", 0, 1, 4303, 2, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 },

	/* Active Energy 1st phase T1, imp (Wh);
	{ "Unmapped_RealEnergy_L1_T1_output", 0, 1000, 4119, 4, MODBUS_DATA_TYPE_FLOAT_ABCD, NULL, 0 }, */
#if 0
//Active Energy 2nd phase T1, imp (Wh);
	ret=mb_read_value(ctx,4123, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L2_T1_output : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L2_T1_output", "%f", real*1000);

//Active Energy 3rd phase T1, imp (Wh);
	ret=mb_read_value(ctx,4127, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L3_T1_output : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L3_T1_output", "%f", real*1000);

//Active Energy Σ T1, imp (Wh);
	ret=mb_read_value(ctx,4131, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_T1_output : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_T1_output", "%f", real*1000);

//Active Energy 1st phase T2, imp (Wh);
	ret=mb_read_value(ctx,4135, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L1_T2_output : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L1_T2_output", "%f", real*1000);

//Active Energy 2nd phase T2, imp (Wh);
	ret=mb_read_value(ctx,4139, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L2_T2_output : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L2_T2_output", "%f", real*1000);

//Active Energy 3nd phase T2, imp (Wh);
	ret=mb_read_value(ctx,4143, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L3_T2_output : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L3_T2_output", "%f", real*1000);

//Active Energy Σ T2, imp (Wh);
	ret=mb_read_value(ctx, 4147, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_T2_output : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_T2_output", "%f", real*1000);
// end of IMP

//Active Energy 1st phase T1, exp (Wh);
	ret=mb_read_value(ctx,4161, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L1_T1 : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L1_T1", "%f", real*1000);

//Active Energy 2nd phase T1, exp (Wh);
	ret=mb_read_value(ctx,4165, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L2_T1 : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L2_T1", "%f", real*1000);

//Active Energy 3rd phase T1, exp (Wh);
	ret=mb_read_value(ctx,4169, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L3_T1 : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L3_T1", "%f", real*1000);

//Active Energy Σ T1, exp (Wh);
	ret=mb_read_value(ctx,4173, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_T1 : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_T1", "%f", real*1000);

//Active Energy 1st phase T2, exp (Wh);
	ret=mb_read_value(ctx,4177, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L1_T2 : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L1_T2", "%f", real*1000);

//Active Energy 2nd phase T2, exp (Wh);
	ret=mb_read_value(ctx,4181, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L2_T2 : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L2_T2", "%f", real*1000);

//Active Energy 3nd phase T2, exp (Wh);
	ret=mb_read_value(ctx,4185, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_L3_T2 : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_L3_T2", "%f", real*1000);

//Active Energy Σ T2, exp (Wh);
	ret=mb_read_value(ctx,4189, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "RealEnergy_T2 : %f", real);
	dstate_setinfo("Unmapped_RealEnergy_T2", "%f", real*1000);

//Reactive Energy 1st phase T1, exp (varh);
	ret=mb_read_value(ctx,4225, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactiveEnergy_L1_T1 : %f", real);
	dstate_setinfo("Unmapped_ReactiveEnergy_L1_T1", "%f", real*1000);

//Reactive Energy 2nd phase T1, exp (varh);
	ret=mb_read_value(ctx,4229, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactiveEnergy_L2_T1 : %f", real);
	dstate_setinfo("Unmapped_ReactiveEnergy_L2_T1", "%f", real*1000);

//Reactive Energy 3rd phase T1, exp (varh);
	ret=mb_read_value(ctx,4233, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactiveEnergy_L3_T1 : %f", real);
	dstate_setinfo("Unmapped_ReactiveEnergy_L3_T1", "%f", real*1000);

//Reactive Energy Σ T1, exp (varh);
	ret=mb_read_value(ctx,4237, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactiveEnergy_T1 : %f", real);
	dstate_setinfo("Unmapped_ReactiveEnergy_T1", "%f", real*1000);

//Reactive Energy 1st phase T2, exp (varh);
	ret=mb_read_value(ctx,4241, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactiveEnergy_L1_T2 : %f", real);
	dstate_setinfo("Unmapped_ReactiveEnergy_L1_T2 ", "%f", real*1000);

//Reactive Energy 2nd phase T2, exp (varh);
	ret=mb_read_value(ctx,4245, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactiveEnergy_L2_T2 : %f", real);
	dstate_setinfo("Unmapped_ReactiveEnergy_L2_T2 ", "%f", real*1000);

//Reactive Energy 3rd phase T2, exp (varh);
	ret=mb_read_value(ctx,4249, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactiveEnergy_L3_T2 : %f", real);
	dstate_setinfo("Unmapped_ReactiveEnergy_L3_T2 ", "%f", real*1000);

//Reactive Energy Σ T2, exp (varh);
	ret=mb_read_value(ctx,4253, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactiveEnergy_T2 : %f", real);
	dstate_setinfo("ReactiveEnergy_T2", "%f", real*1000);

//Reactive Power 1st phase (var);
	ret=mb_read_value(ctx,4257, 2, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactivePower_L1 : %f", real);	
	dstate_setinfo("Unmapped_ReactivePower_L1", "%f", real*1000);

//Reactive Power 2nd phase (var);
	ret=mb_read_value(ctx,4259, 2, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactivePower_L2 : %f", real);
	dstate_setinfo("Unmapped_ReactivePower_L2", "%f", real*1000);

//Reactive Power 3nd phase (var);
	ret=mb_read_value(ctx,4261, 2, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactivePower_L3 : %f", real);
	dstate_setinfo("Unmapped_ReactivePower_L3", "%f", real*1000);

//Reactive Power Σ (var);
	ret=mb_read_value(ctx,4263, 4, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "ReactivePower : %f", real);
	dstate_setinfo("Unmapped_ReactivePower", "%f", real*1000);
#endif

	/* end of structure. */
	{ NULL, 0, 0, 0, 0, 0, NULL, 0 }
};

/* Read Modbus Registers */
static int mb_read_value(modbus_t * ctx, int addr, int nb, uint16_t * dest)
{
	upsdebugx(1, "%s", __func__);

	// FIXME: increase timeout in case of failure?
	int r = -1, i = 3;
	while ((r == -1) && i > 0) {
		upsdebugx(3, "%s: try %i/3", __func__, i);

		/* Check if we are asked to stop */
		if (exit_flag != 0) {
			upsdebugx(1, "%s: aborting because exit_flag was set", __func__);
			return -1;
		}

		r = modbus_read_registers(ctx, addr, nb, dest);
		if (r == -1) {
			upsdebugx(3, "%s: ERROR in modbus_read_registers(addr:%d, count:%d): %s (%s)",
					__func__, addr, nb, modbus_strerror(errno), device_path);
			i--;
		}
	}
	/* FIXME: invalidate data after 3 unsuccessful tries? */
	return r;
}

void upsdrv_initinfo(void)
{
	uint16_t tab_reg[64];
	int ret;

	upsdebugx(1, "%s", __func__);

	/* FIXME: switch to modbus_info_t */
	dstate_setinfo ("device.mfr", "Eaton");
	dstate_setinfo ("device.model", "PX Meter EMECMODB");
	dstate_setinfo ("device.type", "power-meter");

	/* Device type identifies the phasing type */
	ret = mb_read_value(ctx, 4099, 1, tab_reg);
	if (ret != -1) {
		upsdebugx(3, "device type identifier: %i", tab_reg[0]);
		switch(tab_reg[0]) {
			case 1: /* three-phase full */
			case 2: /* three-phase basic */
				dstate_setinfo ("input_phases", "3");
				break;
			case 3: /* single-phase full */
			case 4: /* single-phase basic */
				dstate_setinfo ("input_phases", "1");
				break;
		}
	}
	else
		fatalx(EXIT_FAILURE, "No communication with device");

	/* Firmware version */
	memset(tab_reg, 0, sizeof(tab_reg));
	ret = mb_read_value(ctx, 4100, 1, tab_reg);
	upsdebugx(1, "firmware version: %i",tab_reg[0]);
	dstate_setinfo ("device.firmware","%i",(int) (tab_reg[0]));

	//PID (Product identification)
	char serial[15];
	memset(serial, 0, sizeof(serial));
	for (int i = 0; i < 7; i++) {

		ret = mb_read_value(ctx, 4104 + i, 1, tab_reg);
		if (ret != -1) {
			upsdebugx(2, "%s: received '%c%c'", __func__,
				MODBUS_GET_HIGH_BYTE(tab_reg[0]),
				MODBUS_GET_LOW_BYTE(tab_reg[0]));
			snprintfcat(serial, 15, "%c%c",
				MODBUS_GET_HIGH_BYTE(tab_reg[0]),
				MODBUS_GET_LOW_BYTE(tab_reg[0]));
		}
		else
			upsdebugx(2, "Failed to received register %i", 4104 + i);
	}
	upsdebugx(1, "Part number: %s", &serial[0]);
	dstate_setinfo ("device.part", "%s", serial);
	// FIXME: value published is borked with '4"'

	dstate_dataok();

	/* 1/ Open the NUT Modbus definition file and load the data
	 * 2/ Iterate through these data and call dstate_setinfo */
}

void upsdrv_updateinfo(void)
{
	modbus_info_t *mb_info_p;
	int valid_data = 0; /* 1 to call dstate_dataok(), 0 for _datastale() */
	uint16_t tab_reg[64];
	float real;
	int ret;

	upsdebugx(1, "%s", __func__);

	/* Loop through all mapping entries for the current_device_number */
	for (mb_info_p = &eaton_pwmeter_emecmodb[0]; mb_info_p->info_type != NULL ; mb_info_p++) {

		/* Check if we are asked to stop */
		if (exit_flag != 0) {
			upsdebugx(1, "%s: aborting because exit_flag was set", __func__);
			return;
		}

		memset(tab_reg, 0, sizeof(tab_reg));
		ret = mb_read_value(ctx,
				mb_info_p->modbus_register_nb,
				mb_info_p->modbus_register_size, tab_reg);
		if (ret != -1) {
			valid_data = 1;
			switch (mb_info_p->modbus_data_type) {
				case MODBUS_DATA_TYPE_FLOAT_ABCD:
					real = modbus_get_float_abcd(tab_reg);
					upsdebugx(1, "received value: %s: %.4f", mb_info_p->info_type, real);
					/* Publish value, with factor applied */
					dstate_setinfo(mb_info_p->info_type, "%.4f", real * mb_info_p->info_len);
					break;
				case MODBUS_DATA_TYPE_STRING:
				default:
					// FIXME:
					break;
			}
		}
		else
			upsdebugx(1, "Error: failed to get %s from register %i",
						mb_info_p->info_type, mb_info_p->modbus_register_nb);
	}

	/* check for (communication) status */
	if (valid_data)
		dstate_dataok();
	else
		dstate_datastale();
}

void upsdrv_shutdown(void)
{
	upsdebugx(1, "%s", __func__);
	/* FIXME: shutdown only applies to UPS! */
	fatalx(EXIT_FAILURE, "shutdown not supported");
}

/*
static int instcmd(const char *cmdname, const char *extra)
{
	// FIXME: counter reset should go here
	if (!strcasecmp(cmdname, "test.battery.stop")) {
		ser_send_buf(upsfd, ...);
		return STAT_INSTCMD_HANDLED;
	}

	upslogx(LOG_NOTICE, "instcmd: unknown command [%s]", cmdname);
	return STAT_INSTCMD_UNKNOWN;

#if NOT_USEFUL
//Protocol
	ret=mb_read_value(ctx,4111,1,tab_reg);
	upsdebugx(1,"Protocol_type : %i",tab_reg[0]);
	dstate_setinfo("Unmapped_protocolType","%i",(int) (tab_reg[0]));

//Baudrate 1200,2400,4800,9600,19200 ou 38400
	ret=mb_read_value(ctx,4112,1,tab_reg);
	upsdebugx(1,"Speed : %i",tab_reg[0]);
	dstate_setinfo("Unmapped_Speed","%i",(int) (tab_reg[0]));

//Parity
	char parity[5];
	ret=mb_read_value(ctx,4113,1,tab_reg);
	if (tab_reg[0]==0)
		strcpy(parity , "none");
	else {if (tab_reg[0]==1)
		strcpy(parity , "even");
		else {strcpy(parity , "odd");
			}
		}

	upsdebugx(1,"Parity : %s",parity);
	dstate_setinfo("Unmapped_Parity","%s",parity);
// Stop bits
	ret=mb_read_value(ctx,4114,1,tab_reg);
	upsdebugx(1,"Stop_bits : %i",tab_reg[0]);
	dstate_setinfo("Unmapped_Stop_bits","%i",(int) (tab_reg[0]));

//Modbus address, Slave ID
	ret=mb_read_value(ctx,4115,1,tab_reg);
	upsdebugx(1,"Modbus address : %i",tab_reg[0]);
	dstate_setinfo("ups.productid","%i",(int) (tab_reg[0]));
//Reset interface command
	ret=mb_read_value(ctx,4116,1,tab_reg);
	upsdebugx(1,"Reset interface command : %i",tab_reg[0]);
	dstate_setinfo("Unmapped_ResetInterfaceCommand","%i",(int) (tab_reg[0]));
//Value format
	ret=mb_read_value(ctx,4117,1,tab_reg);
	upsdebugx(1,"Value format : %i",tab_reg[0]);
	dstate_setinfo("Unmapped_Value_format","%i",(int) (tab_reg[0]));
#endif // NOT_USEFUL

	//Reset energy command
	addcmd (reset.counters.energy.{all,active, reactive} // 3,1,2)
	// FIXME: map to a command
	ret=mb_read_value(ctx,4118,1,tab_reg);
	upsdebugx(1,"Reset energy counters command : %i",tab_reg[0]);
	dstate_setinfo("Unmapped_Reset_energy_counters_command","%i",(int) (tab_reg[0]));
}
*/

/*
static int setvar(const char *varname, const char *val)
{
	if (!strcasecmp(varname, "ups.test.interval")) {
		ser_send_buf(upsfd, ...);
		return STAT_SET_HANDLED;
	}

	upslogx(LOG_NOTICE, "setvar: unknown variable [%s]", varname);
	return STAT_SET_UNKNOWN;
}
*/

void upsdrv_help(void)
{
}

/* list flags and values that you want to receive via -x */
void upsdrv_makevartable(void)
{
	char temp [MAX_STRING_SIZE];

	upsdebugx(1, "%s", __func__);

	snprintf(temp, sizeof(temp), "Modbus TCP ID of the slave (default=%s).", "auto");
	addvar (VAR_VALUE, "slave_id", temp);

	/* FIXME: deprecate in favor of the standard ip:port notation */
	snprintf(temp, sizeof(temp), "Modbus TCP port of the slave (default=%s).", DEFAULT_MODBUS_TCP_PORT_STR);
	addvar (VAR_VALUE, "tcp_port", temp);
}

void upsdrv_initups(void)
{
	upsdebugx(1, "%s", __func__);

	char *slave_id = ((getval("slave_id")!=NULL)?getval("slave_id"):"auto");
	/* FIXME: use upscli_splitaddr, keep for a transition period, then remove */
	char *tcp_port = ((getval("tcp_port")!=NULL)?getval("tcp_port"):DEFAULT_MODBUS_TCP_PORT_STR);

	upsdebugx(2, "%s: connecting to Modbus device %s", __func__, device_path);

	/* Determine if it's a RTU (serial) or ethernet (TCP) connection */
	/* FIXME: need to address windows COM port too!
	 * is '|| !strncmp(device_path[0], "COM", 3)' sufficient? */
	if (device_path[0] == '/') {
		upsdebugx(2, "%s: Modbus RTU (serial) device: %s", __func__, device_path);

		/* FIXME: handle serial comm. params (params in ups.conf
		 * and/or definition file) */
		ctx = modbus_new_rtu(device_path, 115200, 'N', 8, 2);
		if (ctx == NULL)
			fatalx(EXIT_FAILURE, "Unable to create the libmodbus context");
	}
	else {
		/* FIXME:
			* use upscli_splitaddr(device_path[0] ? device_path[0] : "localhost", &hostname, &port)
			to get port
			* also propagate to nut-scanner & ftys */
		upsdebugx(2, "%s: Modbus TCP (network) device: %s:%s", __func__, device_path, tcp_port);
		/* FIXME: modbus_new_tcp_pi() is supposed to support IPv6, worth a test */
		ctx = modbus_new_tcp_pi(device_path, tcp_port);
		if (ctx == NULL)
			fatalx(EXIT_FAILURE, "Unable to create the libmodbus context");
	}

	/* Set the target slave */
	upsdebugx(2, "%s: Modbus slave_id: %s", __func__, slave_id);
	if (strncmp(slave_id, "auto", 4)) {
		if (modbus_set_slave(ctx, atoi(slave_id)) == -1) {
			fatalx(EXIT_FAILURE, "%s: Modbus set slave failed: %s", __func__, modbus_strerror(errno));
		}
		else
			upsdebugx(1, "upsdrv_initups: successfully set Modbus slave");
	}
	else {
		/* FIXME: implement slave discovery, as per nutscan_modbus
			but WARN that it's slow, and should be limited to exploration,
			which should better be done with nut-scanner :D */
		fatalx(EXIT_FAILURE, "slave_id='auto' is not yet supported!");
	}

	/* Connect to the device */
	if (modbus_connect(ctx) == -1) {
		modbus_free(ctx);
		fatalx(EXIT_FAILURE, "%s: Modbus connection failed: %s", __func__, modbus_strerror(errno));
	}
	else
		upsdebugx(1, "upsdrv_initups: successfully connected to Modbus device");

	/* Enable Modbus library debug info */
	if (nut_debug_level >= 5)
		modbus_set_debug(ctx, TRUE);

	/* Seems to impact performances a looooot!!!
	modbus_set_error_recovery(ctx,
							  MODBUS_ERROR_RECOVERY_LINK |
							  MODBUS_ERROR_RECOVERY_PROTOCOL); */

	// FIXME: create timeout params (sec, usec) in upsdrv_makevartable()
	uint32_t response_to_sec;
	uint32_t response_to_usec;
	modbus_get_response_timeout(ctx, &response_to_sec, &response_to_usec);
	upsdebugx(2, "default response timeout: %i / %i", response_to_sec, response_to_usec);

	modbus_set_response_timeout(ctx, 3, 0);
	modbus_get_response_timeout(ctx, &response_to_sec, &response_to_usec);
	upsdebugx(2, "setting response timeout: %i / %i", response_to_sec, response_to_usec);

	/* don't try to detect the device here */
}

void upsdrv_cleanup(void)
{
	upsdebugx(1, "%s", __func__);

	/* free(dynamic_mem); */
	if (ctx != NULL) {
		modbus_close(ctx);
		modbus_free(ctx);
	}
}
