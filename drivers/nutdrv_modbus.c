/*  nutdrv_modbus.c - Driver for Modbus power devices (UPS, meters, ...)
 *
 *  Copyright (C)
 *    2012-2014  Arnaud Quette <arnaud.quette@free.fr>
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
 * - everything...
 * - create the format for the "NUT Modbus definition file"
 * - complete initups
 * - create initinfo
 * - create updateinfo
 *
 * "NUT Modbus definition file"		xxx.modbus
 * # A comment header may contain information such as
 * # - author and device information
 * # - default communication settings (baudrate, parity, data_bit, stop_bit)
 * device.type: <ups,meter,...>
 * [device.mfr: <Manufacturer name>]
 * [device.model: <Model name>]
 * <nut varname>: <modbus address>, <modbus register>, ..., <convertion function name>
 */

#include "main.h"
#include <modbus.h>

#define DRIVER_NAME	"NUT Modbus driver"
#define DRIVER_VERSION	"0.01"

/* Variables */
modbus_t *ctx = NULL;

/* driver description structure */
upsdrv_info_t upsdrv_info = {
	DRIVER_NAME,
	DRIVER_VERSION,
	"Arnaud Quette <arnaud.quette@gmail.com>\n",
	DRV_EXPERIMENTAL,
	{ NULL }
};

/* Modbus Read Input Registers */
static int mb_read_value(modbus_t * ctx, int addr, int nb, uint16_t * dest)
{
	int r;
	r = modbus_read_input_registers(ctx, addr, nb, dest);
	if (r == -1) {
		upslogx(LOG_ERR, "%s: modbus_read_input_registers(addr:%d, count:%d): %s (%s)", __func__, addr, nb, modbus_strerror(errno), device_path);
		//errcount++;
	}
	return r;
}

char *mb_read_mfr()
{
	uint16_t tab_reg[64];
	int ret;
	char mfr[128];

	upsdebugx(2, "mb_read_mfr");

	memset (mfr, 0, 128);

	//int modbus_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest);
	//ret = modbus_read_registers(ctx, 0x600, 8, tab_reg);
	//int modbus_read_input_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest);
	ret = modbus_read_registers(ctx, 1536, 8, tab_reg);
	if (ret == -1)
		upsdebugx(1, "Error reading register 1536: %s", modbus_strerror(errno));
	else {
		upsdebugx(1, "Read register 1536 successfully");

		for (int i = 0; i < ret; i++) {
			if (tab_reg[i] != 0) {
				mfr[i * 2] = MODBUS_GET_HIGH_BYTE(tab_reg[i]);
				mfr[(i * 2) + 1] = MODBUS_GET_LOW_BYTE(tab_reg[i]);
			}
		}
		upsdebugx(1, "MFR: %s", mfr);
		dstate_setinfo ("device.mfr", mfr);
	}
	return NULL;
}
void upsdrv_initinfo(void)
{
	upsdebugx(2, "upsdrv_initinfo");

	uint16_t tab_reg[64];

/* AQU:
lire:

	dstate_setinfo("device.mfr", ...);
	dstate_setinfo("device.model", ...);
	dstate_setinfo("device.serial", ...);
	+ battery.type
*/
	// FIXME: test return value
	char mfr[128];
	int ret;
	ret = mb_read_value(ctx, 1536, 8, tab_reg);

	for (int i = 0; i < ret; i++) {
		if (tab_reg[i] != 0) {
			mfr[i * 2] = MODBUS_GET_HIGH_BYTE(tab_reg[i]);
			mfr[(i * 2) + 1] = MODBUS_GET_LOW_BYTE(tab_reg[i]);
		}
	}
	upsdebugx(1, "MFR: %s", mfr);
	dstate_setinfo ("device.mfr", mfr);

	char model[128];
	
	ret = mb_read_value(ctx, 1544, 8, tab_reg);

	for (int i = 0; i < ret; i++) {
		if (tab_reg[i] != 0) {
			model[i * 2] = MODBUS_GET_HIGH_BYTE(tab_reg[i]);
			model[(i * 2) + 1] = MODBUS_GET_LOW_BYTE(tab_reg[i]);
		}
	}
	upsdebugx(1, "Model: %s", model);
	dstate_setinfo ("device.model", model);

	char serial[128];
	ret = mb_read_value(ctx, 1552, 8, tab_reg);

	for (int i = 0; i < ret; i++) {
		if (tab_reg[i] != 0) {
			serial[i * 2] = MODBUS_GET_HIGH_BYTE(tab_reg[i]);
			serial[(i * 2) + 1] = MODBUS_GET_LOW_BYTE(tab_reg[i]);
		}
	}
	upsdebugx(1, "Serial: %s", serial);
	dstate_setinfo ("device.serial", serial);
//1569	4	2	Float (CDAB)	Battery module Nominal Voltage	Nominal voltage of Battery module	V	TRUE	24.000000	battery.voltage.nominal
	ret = mb_read_value(ctx, 1569, 2, tab_reg);
	float real = modbus_get_float_badc(tab_reg);
	upsdebugx(1, "batteryVoltageNominal : %f",real);
	dstate_setinfo("battery.voltage.nominal","%f",real); 
	
//1571	4	2	Float (CDAB)	Battery module Nominal Charge Capacity	Nominal charge capacity of Battery module	Ah/Wh	TRUE	9.000000	
// Pas de nom générique	nut
	ret = mb_read_value(ctx, 1571, 2, tab_reg);
	real = modbus_get_float_badc(tab_reg);
	upsdebugx(1, "battery Nominal Charge Capacity : %f",real);
	dstate_setinfo("batteryNominalChargeCapacity","%f",real);

/* Fixme: for later
	char batteryType[128];
	ret = mb_read_value(ctx, 1560, 8, tab_reg);

	for (int i = 0; i < ret; i++) {
		if (tab_reg[i] != 0) {
			batteryType[i * 2] = MODBUS_GET_HIGH_BYTE(tab_reg[i]);
			batteryType[(i * 2) + 1] = MODBUS_GET_LOW_BYTE(tab_reg[i]);
		}
	}
	upsdebugx(1, "batteryType: %s", batteryType);
	dstate_setinfo ("battery.type", batteryType);
 */
	dstate_dataok();
	/* try to detect the device here - call fatal_with_errno(EXIT_FAILURE, ) if it fails */

	//mb_read_mfr();

	/* 1/ Open the NUT Modbus definition file and load the data
	 * 2/ Iterate through these data and call dstate_setinfo */

	/* dstate_setinfo("device.mfr", "skel manufacturer"); */
	/* dstate_setinfo("device.model", "longrun 15000"); */
	/* dstate_setinfo("device.type", "longrun 15000"); */
	/* ... */

	/* upsh.instcmd = instcmd; */
	/* upsh.setvar = setvar; */
}
// Function that convert Decimal to binary 
 
char *decimal_to_binary(int n)
{
   int c, d, count;
   char *pointer;
   
   count = 0;
   pointer = (char*)malloc(32+1);
   
   if (pointer == NULL)
      exit(EXIT_FAILURE);
     
   for (c = 31 ; c >= 0 ; c--)
   {
      d = n >> c;
     
      if (d & 1)
         *(pointer+count) = 1 + '0';
      else
         *(pointer+count) = 0 + '0';
     
      count++;
   }
   *(pointer+count) = '\0';
   
   return  pointer;
}

void upsdrv_updateinfo(void)
{
	uint16_t tab_reg[64];
	int ret;

//	int outputCurrent;
	ret=mb_read_value(ctx, 265, 1, tab_reg);
	upsdebugx(1, "outputCurrent %i",(int) (tab_reg[0]));
	dstate_setinfo("output.current", "%i", (int) (tab_reg[0]));
	
//	int outputVoltage;
	ret=mb_read_value(ctx, 292, 1, tab_reg);
	upsdebugx(1, "outputVoltage %i", (int) (tab_reg[0]));
	dstate_setinfo("output.voltage", "%i", (int) (tab_reg[0]));

//	int outputRealPower;
	ret=mb_read_value(ctx,304, 1, tab_reg);
	upsdebugx(1, "outputRealPower %i", tab_reg[0]);
	dstate_setinfo("output.realpower", "%i", (int) (tab_reg[0]));

//	int outputPower;
	ret=mb_read_value(ctx,307,1,tab_reg);
	upsdebugx(1,"outputPower : %i",tab_reg[0]);
	dstate_setinfo("output.power","%i",(int) (tab_reg[0]));

//	int inputVoltage;
	ret=mb_read_value(ctx,336, 1, tab_reg);
	upsdebugx(1, "inputVoltage : %i", tab_reg[0]);
	dstate_setinfo("input.voltage", "%i", (int) (tab_reg[0]));

//1575	4	2	Float (CDAB)	Battery module Voltage	Voltage of Battery module	V	TRUE	27.600000	battery.voltage
	ret=mb_read_value(ctx,1575, 2, tab_reg);
	float real = modbus_get_float_badc(tab_reg);
	upsdebugx(1, "batteryVoltage : %f",real);
	dstate_setinfo("battery.voltage","%f",real); 	

//1589	4	2	INT32 (CDAB) 	Battery module Remaining Charge Capacity	Remaining charge capacity of Battery module	%	TRUE	100	battery.charge
	ret=mb_read_value(ctx,1589,2,tab_reg);
	uint16_t AB = tab_reg[0];
	tab_reg[0] = tab_reg[1];
	tab_reg[1]=AB;	
	uint32_t tab_conv=MODBUS_GET_INT32_FROM_INT16(tab_reg, 0);
	upsdebugx(1,"batteryCharge : %i", tab_conv);
	dstate_setinfo("battery.charge","%i",tab_conv);

//1592	4	2	INT32 (CDAB) 	Battery module Remaining Time	Remaining Time of Battery module	s	TRUE	11520	battery.runtime
	ret=mb_read_value(ctx,1592,2,tab_reg);
	AB = tab_reg[0];
	tab_reg[0] = tab_reg[1];
	tab_reg[1]=AB;	
	tab_conv=MODBUS_GET_INT32_FROM_INT16(tab_reg, 0);
	upsdebugx(1,"batteryRuntime : %i", tab_conv);
	dstate_setinfo("battery.runtime","%i",tab_conv);


//262	2	1	Int16	Current phase 1 main 2	Bypass input phase 1 current	A	TRUE	0	
// Pas de nom générique	nut
	ret=mb_read_value(ctx,262,1,tab_reg);
	upsdebugx(1,"Current phase 1 main 2 : %i",tab_reg[0]);
	dstate_setinfo("currentPhase1Main2","%i",(int) (tab_reg[0]));

//286	2	1	Int16	Voltage phase 1 main 2	Bypass input phase 1 voltage	V	TRUE	240	
// Pas de nom générique	nut	
	ret=mb_read_value(ctx,286,1,tab_reg);
	upsdebugx(1,"Voltage Phase 1 main 2 : %i",tab_reg[0]);
	dstate_setinfo("voltagePhase1Main2","%i",(int) (tab_reg[0]));

//1573	2	1	Uint16	Battery module Capacity Unit	Capacity unit of Battery module	0	TRUE	0	
// Pas de nom générique	nut	
	ret=mb_read_value(ctx,1573,1,tab_reg);
	upsdebugx(1,"Battery module Capacity Unit : %i",tab_reg[0]);
	dstate_setinfo("batteryModuleCapacityUnit","%i",(int) (tab_reg[0])); 

	dstate_dataok();

/* AQU: a faire:
lecture des mesures */
/*uint16_t outputCurrent;

mb_read_value(ctx, 265, 1, tab_reg);
outputCurrent=tab_reg[0]
upsdebugx(1, "outputCurrent %i", outputCurrent);
	dstate_setinfo ("output.current", outputCurrent);*/


/*	output.*
	battery.*
	input.*

 ex:
	mb_read_value(ctx, 265, 1, tab_reg);
	dstate_setinfo("output.current", "%d", (int) (tab_reg[0]);

+ dstate_dataok();
*/

//	mb_read_mfr();
//	dstate_setinfo ("device.model", "Bapt'man");
//	dstate_dataok();

	/* int flags; */
	/* char temp[256]; */

	/* ser_sendchar(upsfd, 'A'); */
	/* ser_send(upsfd, "foo%d", 1234); */
	/* ser_send_buf(upsfd, bincmd, 12); */

	/*
	 * ret = ser_get_line(upsfd, temp, sizeof(temp), ENDCHAR, IGNCHARS);
	 *
	 * if (ret < STATUS_LEN) {
	 * 	upslogx(LOG_ERR, "Short read from UPS");
	 *	dstate_datastale();
	 *	return;
	 * }
	 */

	/* dstate_setinfo("var.name", ""); */

	/* if (ioctl(upsfd, TIOCMGET, &flags)) {
	 *	upslog_with_errno(LOG_ERR, "TIOCMGET");
	 *	dstate_datastale();
	 *	return;
	 * }
	 */

	/* status_init();
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
	/* allow '-x xyzzy' */
	/* addvar(VAR_FLAG, "xyzzy", "Enable xyzzy mode"); */

	/* allow '-x foo=<some value>' */
	/* addvar(VAR_VALUE, "foo", "Override foo setting"); */
}

void upsdrv_initups(void)
{
	upsdebugx(2, "upsdrv_initups");

	/* Determine if it's a RTU (serial) or ethernet (TCP) connection */
	/* FIXME: need to address windows COM port too!
	 * || !strncmp(device_path[0], "COM", 3) */
	if (device_path[0] == '/') {
		upsdebugx(2, "upsdrv_initups: RTU (serial) device");

		/* FIXME: handle serial comm. params (params in ups.conf
		 * and/or definition file) */
		ctx = modbus_new_rtu(device_path, 115200, 'N', 8, 1);
		if (ctx == NULL)
			fatalx(EXIT_FAILURE, "Unable to create the libmodbus context");
	}
	else {
		/* FIXME: upscli_splitaddr(device_path[0] ? device_path[0] : "localhost", &hostname, &port */
		upsdebugx(2, "upsdrv_initups: TCP (network) device %s", device_path);
		//ctx = modbus_new_tcp(device_path, 502);
		ctx = modbus_new_tcp_pi(device_path, "502");
		if (ctx == NULL)
			fatalx(EXIT_FAILURE, "Unable to create the libmodbus context");

		if (modbus_connect(ctx) == -1) {
			modbus_free(ctx);
			fatalx(EXIT_FAILURE, "Connection failed: %s\n", modbus_strerror(errno));
		}
		else
			upsdebugx(2, "upsdrv_initups: successfully connected to TCP (network) device");
	}

	modbus_set_slave(ctx, 1);

// AQU: a gicler

//	modbus_set_response_timeout(ctx, 2, 0);

//	uint32_t old_response_to_sec;
//	uint32_t old_response_to_usec;
//	modbus_get_response_timeout(ctx, &old_response_to_sec, &old_response_to_usec);
//	upsdebugx(2, "response timeout: %i / %i", old_response_to_sec, old_response_to_usec);
	modbus_set_debug(ctx, TRUE);
	modbus_set_error_recovery(ctx,
							  MODBUS_ERROR_RECOVERY_LINK |
							  MODBUS_ERROR_RECOVERY_PROTOCOL);


//	mb_read_mfr();

	/* don't try to detect the device here */
}

void upsdrv_cleanup(void)
{
	/* free(dynamic_mem); */
	if (ctx != NULL) {
		modbus_close(ctx);
		modbus_free(ctx);
	}
}
