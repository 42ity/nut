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
#define DRIVER_VERSION	"0.02"

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

/* Read Modbus Registers */
static int mb_read_value(modbus_t * ctx, int addr, int nb, uint16_t * dest)
{
	upsdebugx(1, "%s", __func__);

	int r;
	r = modbus_read_registers(ctx, addr, nb, dest);
	if (r == -1) {
		upslogx(LOG_ERR,
				"%s: modbus_read_registers(addr:%d, count:%d): %s (%s)",
				__func__, addr, nb, modbus_strerror(errno), device_path);
		//errcount++;
	}
	return r;
}

void upsdrv_initinfo(void)
{
	uint16_t tab_reg[64];
	int ret;

	upsdebugx(1, "%s", __func__);

	dstate_setinfo ("device.mfr","Eaton");
	dstate_setinfo ("device.model","PX Meter");

	/* Device type */
	/* ret = modbus_read_registers(ctx, 4099, 1, tab_reg);
	upsdebugx(1, "device.type: %i",tab_reg[0]); */
	dstate_setinfo ("device.type","power-meter");

	/* Firmware version */
	ret = mb_read_value(ctx, 4100, 1, tab_reg);	
	upsdebugx(1, "firmware version: %i",tab_reg[0]);
	dstate_setinfo ("ups.firmware","%i",(int) (tab_reg[0]));
	// FIXME: firmware is supposed to be a string!

	//PID (Product identification)
	char serial[128];
	ret = mb_read_value(ctx, 4104, 8, tab_reg);
	for (int i = 0; i < ret; i++) {
		if (tab_reg[i] != 0) {
			/* FIXME: if it's the standard way to interpret strings,
				make a function out of it */
			serial[i * 2] = MODBUS_GET_HIGH_BYTE(tab_reg[i]);
			serial[(i * 2) + 1] = MODBUS_GET_LOW_BYTE(tab_reg[i]);
		}
	}
	upsdebugx(1, "Serial: %s", serial);
	dstate_setinfo ("device.serial", "%s", serial);
	// FIXME: value published is borked

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
#endif /* NOT_USEFUL  */

	//Reset energy command
	// FIXME: map to a command
	ret=mb_read_value(ctx,4118,1,tab_reg);
	upsdebugx(1,"Reset energy counters command : %i",tab_reg[0]);
	dstate_setinfo("Unmapped_Reset_energy_counters_command","%i",(int) (tab_reg[0]));

	/* FIXME: check 'ret' for (communication) failure (<= 0)
		and call dstate_datastale() */
	dstate_dataok();
	/* try to detect the device here - call fatal_with_errno(EXIT_FAILURE, ) if it fails */

	/* 1/ Open the NUT Modbus definition file and load the data
	 * 2/ Iterate through these data and call dstate_setinfo */
}

void upsdrv_updateinfo(void)
{
	uint16_t tab_reg[64];
	int ret;

	upsdebugx(1, "%s", __func__);

/* FIXME:
 *	* create a preliminary mapping struct, to iterate over, and simplify code
		int register_nb, int register_size, data_type; // FLOAT_ABCD, STRING, ...
		char *nut_name;
 *  * check ret for failure (see 1rst example below)
 */

/* IMP Energy: input information, what comes from the Grid */
	//Active Energy 1st phase T1, imp (Wh);
	ret = mb_read_value(ctx, 4119, 4, tab_reg);
	if (ret != -1) {
		float real = modbus_get_float_abcd(tab_reg);
		upsdebugx(1, "RealEnergy_L1_T1_output : %f", real);
		dstate_setinfo("Unmapped_RealEnergy_L1_T1_output", "%f", real*1000);
	}
	else
		upsdebugx(1, "Error: failed to get %s from register %i",
					"Unmapped_RealEnergy_L1_T1_output", 4119);

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


//	float Active Power 1st phase (W);
	ret=mb_read_value(ctx, 4151,2, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "input.L1.realpower : %f",real);
	dstate_setinfo("input.L1.realpower", "%f", real*1000);
	
//	float Active Power 2nd phase (W);
	ret=mb_read_value(ctx, 4153, 2, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "input.L2.realpower : %f",real);
	dstate_setinfo("input.L2.realpower", "%f", real*1000);

//	float Active Power 3rd phase (W);
	ret=mb_read_value(ctx,4155, 2, tab_reg);
	real = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "input.L3.realpower : %f", real);
	dstate_setinfo("input.L3.realpower", "%f", real*1000);

//  Active Power Σ (W);
	ret = mb_read_value(ctx, 4157, 4, tab_reg);
	float realpower = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "input.realpower : %f", realpower);
	dstate_setinfo ("input.realpower","%f", realpower*1000);

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



//L1-N voltage (V)
ret = mb_read_value(ctx, 4267, 2, tab_reg);
	float voltage = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Voltage_L1_N : %f", voltage);
	dstate_setinfo ("input.L1-N.voltage","%f", voltage);

//L2-N voltage (V)
ret = mb_read_value(ctx, 4269, 2, tab_reg);
	voltage = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Voltage_L2_N : %f", voltage);
	dstate_setinfo ("input.L2-N.voltage","%f", voltage);
//L3-N voltage (V)
ret = mb_read_value(ctx, 4271, 2, tab_reg);
	voltage = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Voltage_L3_N : %f", voltage);
	dstate_setinfo ("input.L3-N.voltage","%f", voltage);
//L1-L2 voltage (V)
ret = mb_read_value(ctx, 4273, 2, tab_reg);
	voltage = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Voltage_L1_L2 : %f", voltage);
	dstate_setinfo ("input.L1-L2.voltage","%f", voltage);
//L2-L3 voltage (V)
ret = mb_read_value(ctx, 4275, 2, tab_reg);
	voltage = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Voltage_L2_L3 : %f", voltage);
	dstate_setinfo ("input.L2-L3.voltage","%f", voltage);
//L3-L1 voltage (V)
ret = mb_read_value(ctx, 4277, 2, tab_reg);
	voltage = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Voltage_L3_L1 : %f", voltage);
	dstate_setinfo ("input.L3-L1.voltage","%f", voltage);

//phase1 current (A)
ret = mb_read_value(ctx, 4279, 2, tab_reg);
	float current = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Current.L1 : %f", current);
	dstate_setinfo ("input.L1.current","%f", current);
//phase2 current (A)
ret = mb_read_value(ctx, 4281, 2, tab_reg);
	current = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Current.L2 : %f", current);
	dstate_setinfo ("input.L2.current.L2","%f", current);
//phase3 current (A)
ret = mb_read_value(ctx, 4283, 2, tab_reg);
	current = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Current.L3 : %f", current);
	dstate_setinfo ("input.L3.current","%f", current);

//apparent power phase1 (VA)
ret = mb_read_value(ctx, 4285, 2, tab_reg);
	float power = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Power.L1 : %f", power);
	dstate_setinfo ("input.L1.power","%f", power*1000);
//apparent power phase2 (VA)
ret = mb_read_value(ctx, 4287, 2, tab_reg);
	power = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Power.L2 : %f", power);
	dstate_setinfo ("input.L2.power.L2","%f", power*1000);
//apparent power phase3 (VA)
ret = mb_read_value(ctx, 4289, 2, tab_reg);
	power = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Power.L3 : %f", power);
	dstate_setinfo ("input.L3.power","%f", power*1000);
 
//apparent power Σ (VA)
ret = mb_read_value(ctx, 4291, 4, tab_reg);
	power = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Power : %f", power);
	dstate_setinfo ("input.power","%f", power*1000);

//power factor cos ϕ phase1
ret = mb_read_value(ctx, 4295, 2, tab_reg);
	float powerFactor_L1 = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Power.Factor.L1 : %f", powerFactor_L1);
	dstate_setinfo ("input.L1.powerfactor","%f", powerFactor_L1);

	/* power factor cos ϕ phase2 */
	ret = mb_read_value(ctx, 4297, 2, tab_reg);
	float powerFactor_L2 = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Power.Factor.L2 : %f", powerFactor_L2);
	dstate_setinfo ("input.L2.powerfactor","%f", powerFactor_L2);

	/* power factor cos ϕ phase3 */
	ret = mb_read_value(ctx, 4299, 2, tab_reg);
	float powerFactor_L3 = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Power.Factor.L3 : %f", powerFactor_L3);
	dstate_setinfo ("input.L3.powerfactor","%f", powerFactor_L3);

	/* power factor cos ϕ Σ */
	ret = mb_read_value(ctx, 4301, 2, tab_reg);
	float powerFactor = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Power.Factor : %f", powerFactor);
	dstate_setinfo ("input.powerfactor","%f", powerFactor);

	/* frequency (Hz) */
	ret = mb_read_value(ctx, 4303, 2, tab_reg);
	float frequency = modbus_get_float_abcd(tab_reg);
	upsdebugx(1, "Frequency : %f", frequency);
	dstate_setinfo ("input.frequency","%f", frequency);

	/* FIXME: check 'ret' for (communication) failure (<= 0)
		and call dstate_datastale() */
	dstate_dataok();

	/* Not useful when device.type != ups
	 * we should get rid of this
	 * status_init();
	 *
	 * if (ol)
	 * 	status_set("OL");
	 * else
	 * 	status_set("OB");
	 * ...
	 *
	 * status_commit();
	 *
	 * dstate_dataok();
	 */

	/*
	 * poll_interval = 2;
	 */
}

void upsdrv_shutdown(void)
{
	upsdebugx(1, "%s", __func__);

	/* tell the UPS to shut down, then return - DO NOT SLEEP HERE */

	/* maybe try to detect the UPS here, but try a shutdown even if
	   it doesn't respond at first if possible */

	/* replace with a proper shutdown function */
	fatalx(EXIT_FAILURE, "shutdown not supported");

	/* you may have to check the line status since the commands
	   for toggling power are frequently different for OL vs. OB */

	/* OL: this must power cycle the load if possible */

	/* OB: the load must remain off until the power returns */
}

/*
static int instcmd(const char *cmdname, const char *extra)
{
	if (!strcasecmp(cmdname, "test.battery.stop")) {
		ser_send_buf(upsfd, ...);
		return STAT_INSTCMD_HANDLED;
	}

	upslogx(LOG_NOTICE, "instcmd: unknown command [%s]", cmdname);
	return STAT_INSTCMD_UNKNOWN;
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

	snprintf(temp, sizeof(temp), "Modbus TCP port of the slave (default=%s).", DEFAULT_MODBUS_TCP_PORT_STR);
	addvar (VAR_VALUE, "tcp_port", temp);
}

void upsdrv_initups(void)
{
	upsdebugx(1, "%s", __func__);

	char *tcp_port = ((getval("tcp_port")!=NULL)?getval("tcp_port"):DEFAULT_MODBUS_TCP_PORT_STR);
	char *slave_id = ((getval("slave_id")!=NULL)?getval("slave_id"):"auto");

	/* Determine if it's a RTU (serial) or ethernet (TCP) connection */
	/* FIXME: need to address windows COM port too!
	 * is '|| !strncmp(device_path[0], "COM", 3)' sufficient? */
	if (device_path[0] == '/') {
		upsdebugx(2, "%s: RTU (serial) device: %s", __func__, device_path);

		/* FIXME: handle serial comm. params (params in ups.conf
		 * and/or definition file) */
		ctx = modbus_new_rtu(device_path, 115200, 'N', 8, 2);
		if (ctx == NULL)
			fatalx(EXIT_FAILURE, "Unable to create the libmodbus context");
	}
	else {
		/* FIXME:
			use upscli_splitaddr(device_path[0] ? device_path[0] : "localhost", &hostname, &port)
			to get port? */
		upsdebugx(2, "%s: TCP (network) device: %s:%s", __func__, device_path, tcp_port);
		/* Note: modbus_new_tcp_pi() is supposed to support IPv6
			FIXME: worth a test */
		ctx = modbus_new_tcp_pi(device_path, tcp_port);
		if (ctx == NULL)
			fatalx(EXIT_FAILURE, "Unable to create the libmodbus context");

		if (modbus_connect(ctx) == -1) {
			modbus_free(ctx);
			fatalx(EXIT_FAILURE, "%s: Modbus TCP connection failed: %s", __func__, modbus_strerror(errno));
		}
		else
			upsdebugx(1, "upsdrv_initups: successfully connected to TCP (network) device");
	}

	upsdebugx(2, "%s: Modbus slave_id: %s", __func__, slave_id);
	if (strncmp(slave_id, "auto", 4)) {
		modbus_set_slave(ctx, atoi(slave_id));
	}
	else {
		/* FIXME: implement slave discovery, as per nutscan_modbus
			but WARN that it's slow, and should be limited to exploration,
			which should better be done with nut-scanner :D */
		fatalx(EXIT_FAILURE, "slave_id='auto' is not yet supported!");
	}

	// FIXME: create timeout params (sec, usec) in upsdrv_makevartable()
	//	modbus_set_response_timeout(ctx, 2, 0);

	/* Enable Modbus library debug info */
	if (nut_debug_level <= 5)
		modbus_set_debug(ctx, TRUE);

	modbus_set_error_recovery(ctx,
							  MODBUS_ERROR_RECOVERY_LINK |
							  MODBUS_ERROR_RECOVERY_PROTOCOL);

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
