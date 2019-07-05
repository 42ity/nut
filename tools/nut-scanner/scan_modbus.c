/*
 *  Copyright (C) 2019 Arnaud Quette <arnaud.quette@free.fr>
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*! \file scan_modbus.c
    \brief detect NUT supported Modbus devices
    \author Arnaud Quette <arnaud.quette@free.fr>
*/

/* FIXME: check how to address RTU (serial) beside from TCP */
#include "common.h"
#include "nut-scan.h"

#ifdef WITH_MODBUS

#include <sys/socket.h>
#include <stdio.h>
#include <string.h>
#include <ltdl.h>

#include <modbus.h>
#ifdef HAVE_PTHREAD
#include <pthread.h>
#endif
#include "nutscan-modbus.h"

#define MAX_SLAVE_ID 254

static nutscan_device_t * dev_ret = NULL;
#ifdef HAVE_PTHREAD
static pthread_mutex_t dev_mutex;
#endif
long g_usec_timeout ;

/* dynamic link library stuff */
static lt_dlhandle dl_handle = NULL;
static const char *dl_error = NULL;

static int (*nut_modbus_connect)(modbus_t *ctx);
static void (*nut_modbus_close)(modbus_t *ctx);
static modbus_t* (*nut_modbus_new_rtu)(const char *device, int baud, char parity,
                                    int data_bit, int stop_bit);
static modbus_t* (*nut_modbus_new_tcp)(const char *ip_address, int port);
static modbus_t* (*nut_modbus_new_tcp_pi)(const char *node, const char *service); /* IP / port */
static int (*nut_modbus_set_slave)(modbus_t* ctx, int slave);
static void (*nut_modbus_free)(modbus_t *ctx);
static const char *(*nut_modbus_strerror)(int errnum);
static int (*nut_modbus_read_registers)(modbus_t *ctx, int addr, int nb, uint16_t *dest);
static int (*nut_modbus_set_response_timeout)(modbus_t *ctx, uint32_t to_sec, uint32_t to_usec);
static int (*nut_modbus_get_response_timeout)(modbus_t *ctx, uint32_t *to_sec, uint32_t *to_usec);

/* return 0 on error */
int nutscan_load_modbus_library(const char *libname_path)
{
	if( dl_handle != NULL ) {
		/* if previous init failed */
		if( dl_handle == (void *)1 ) {
			return 0;
		}
		/* init has already been done */
		return 1;
	}

	if (libname_path == NULL) {
		upsdebugx(1, "Modbus library not found. Modbus search disabled");
		return 0;
	}

	if( lt_dlinit() != 0 ) {
		upsdebugx(1, "Error initializing lt_init");
		return 0;
	}

	dl_handle = lt_dlopen(libname_path);
	if (!dl_handle) {
		dl_error = lt_dlerror();
		goto err;
	}

	lt_dlerror();	/* Clear any existing error */
	*(void **) (&nut_modbus_connect) = lt_dlsym(dl_handle, "modbus_connect");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}


	*(void **) (&nut_modbus_close) = lt_dlsym(dl_handle, "modbus_close");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	*(void **) (&nut_modbus_free) = lt_dlsym(dl_handle, "modbus_free");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	*(void **) (&nut_modbus_new_tcp) = lt_dlsym(dl_handle, "modbus_new_tcp");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	*(void **) (&nut_modbus_new_tcp_pi) = lt_dlsym(dl_handle, "modbus_new_tcp_pi");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	*(void **) (&nut_modbus_new_rtu) = lt_dlsym(dl_handle, "modbus_new_rtu");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	*(void **) (&nut_modbus_set_slave) = lt_dlsym(dl_handle, "modbus_set_slave");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	*(void **) (&nut_modbus_strerror) = lt_dlsym(dl_handle, "modbus_strerror");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	*(void **) (&nut_modbus_read_registers) = lt_dlsym(dl_handle, "modbus_read_registers");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	*(void **) (&nut_modbus_set_response_timeout) = lt_dlsym(dl_handle, "modbus_set_response_timeout");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	*(void **) (&nut_modbus_get_response_timeout) = lt_dlsym(dl_handle, "modbus_get_response_timeout");
	if ((dl_error = lt_dlerror()) != NULL)  {
		goto err;
	}

	return 1;
err:
	fprintf(stderr, "Cannot load Modbus library (%s) : %s. Modbus search disabled.\n", libname_path, dl_error);
	dl_handle = (void *)1;
	lt_dlexit();
	return 0;
}
/* end of dynamic link library stuff */

// Port for TCP
static void scan_modbus_add_device(const char *address, int slave_id, char * tcp_port)
{
	nutscan_device_t * dev = NULL;
	char str_slave_id[5];

upsdebugx(1, "FIXME: adding device %s, %i", address, slave_id);
	dev = nutscan_new_device();
	dev->type = TYPE_MODBUS;
	dev->driver = strdup("nutdrv_modbus");
	dev->port = strdup(address);
	sprintf(str_slave_id, "%i", slave_id);
	nutscan_add_option_to_device(dev,"slave_id", str_slave_id);
	if( tcp_port ) {
		nutscan_add_option_to_device(dev,"tcp_port",tcp_port);
	}

#ifdef HAVE_PTHREAD
	pthread_mutex_lock(&dev_mutex);
#endif
	dev_ret = nutscan_add_device_to_device(dev_ret, dev);
#ifdef HAVE_PTHREAD
	pthread_mutex_unlock(&dev_mutex);
#endif

}

#if 0
static struct void * scan_snmp_get_manufacturer(char* oid_str,void* handle)
{
	size_t name_len;
	oid name[MAX_OID_LEN];
	struct snmp_pdu *pdu, *response = NULL;
	int status;
	int index = 0;

	/* create and send request. */
	name_len = MAX_OID_LEN;
	if (!(*nut_snmp_parse_oid)(oid_str, name, &name_len)) {
		index++;
		return NULL;
	}

	pdu = (*nut_snmp_pdu_create)(SNMP_MSG_GET);

	if (pdu == NULL) {
		index++;
		return NULL;
	}

	(*nut_snmp_add_null_var)(pdu, name, name_len);

	status = (*nut_snmp_sess_synch_response)(handle,pdu, &response);
	if( response == NULL ) {
		index++;
		return NULL;
	}

	if(status!=STAT_SUCCESS||response->errstat!=SNMP_ERR_NOERROR||
			response->variables == NULL ||
			response->variables->name == NULL ||
			(*nut_snmp_oid_compare)(response->variables->name,
				response->variables->name_length,
				name, name_len) != 0 || 
			response->variables->val.string == NULL ) {
		(*nut_snmp_free_pdu)(response);
		index++;
		return NULL;
	}
	return response;
	return NULL;
}

static void try_all_oid(void * arg)
{
	struct snmp_pdu *response = NULL;
	int index = 0;
	nutscan_snmp_t * sec = (nutscan_snmp_t *)arg;

	while(snmp_device_table[index].oid != NULL) {

		response = scan_snmp_get_manufacturer(snmp_device_table[index].oid,sec->handle);
		if( response == NULL ) {
			index++;
			continue;
		}

		scan_snmp_add_device(sec,response,snmp_device_table[index].mib);

		(*nut_snmp_free_pdu)(response);
		response = NULL;

		index++;
	}
}
#endif

static void * nutscan_scan_modbus_device(void * port_arg)
{
	modbus_t *ctx = NULL;
	uint16_t tab_reg[64];
	char *port_name = (char*)port_arg;
	int max_slave_id = MAX_SLAVE_ID;
	int slave_id = 1;
	int ret = 0;

	/* FIXME: borrowed from nutdrv_modbus, see how to share */

	/* Determine if it's a RTU (serial) or ethernet (TCP) connection */
	/* FIXME: need to address windows COM port too!
	 * || !strncmp(port_name, "COM", 3) */
	if (port_name[0] == '/') {
		upsdebugx(2, "%s: RTU (serial) device", __func__);

		/* FIXME: handle serial comm. params */
		ctx = (*nut_modbus_new_rtu)(port_name, 115200, 'N', 8, 1);
		if (ctx == NULL) {
			upsdebugx(2, "Unable to create the libmodbus context");
			return NULL;
		}
	}
	else {
		/* FIXME: upscli_splitaddr(port_name ? port_name : "localhost", &hostname, &port
			or address the port param someway*/
		upsdebugx(2, "%s: TCP (network) device %s", __func__, port_name);
		ctx = (*nut_modbus_new_tcp)(port_name, MODBUS_TCP_DEFAULT_PORT);
		//ctx = (*nut_modbus_new_tcp_pi)(port_name, "502");
		if (ctx == NULL) {
			upsdebugx(2, "Unable to create the libmodbus context");
			return NULL;
		}
	}

	/* Initialize session */
	if ((*nut_modbus_connect)(ctx) == -1) {
		(*nut_modbus_free)(ctx);
		upsdebugx(1, "Connection failed: %s\n", (*nut_modbus_strerror)(errno));
		return NULL;
	}
	else
		upsdebugx(1, "upsdrv_initups: successfully connected to TCP (network) device %s", port_name);

	uint32_t old_response_to_sec;
	uint32_t old_response_to_usec;
	(*nut_modbus_get_response_timeout)(ctx, &old_response_to_sec, &old_response_to_usec);
	upsdebugx(2, "response timeout: %i / %i", old_response_to_sec, old_response_to_usec);

	// FIXME: Check for lowering value, or parallelizing slaves check!
	(*nut_modbus_set_response_timeout)(ctx, 0, 100000); // 100ms

	/* Now loop to find the slave(s) ID(s)
		Note that RTU supports only 1
		and TCP multiple
		FIXME:
		* use diagnostic command 17 (Report Slave ID) to discover */
	for ( ; slave_id <= max_slave_id ; slave_id++) {

		upsdebugx(2, "Checking host %s for slave %i", port_name, slave_id);
		(*nut_modbus_set_slave)(ctx, slave_id);

		// FIXME: get something, to check for slave presence
		// mb_read_mfr();

		/* FIXME: try a list of known and unique registers */
		// for now, use a hard coded pw meter!

		// Reg. 4099 is Device type
		ret = (*nut_modbus_read_registers)(ctx, 4099, 8, tab_reg);
		// Note: whatever register requested, getting an answer means that
		// there is a slave device listening! ;)

		if (ret == -1) {
			upsdebugx(2, "Error reading register 4099: %s", (*nut_modbus_strerror)(errno));
			// FIXME: goto clean
			//return NULL; => continue
		}
		else {
			upsdebugx(1, "Read register 4099 successfully");
			scan_modbus_add_device(port_name, slave_id, "502");
		}
	}

	/* free(dynamic_mem); */
	if (ctx != NULL) {
		(*nut_modbus_close)(ctx);
		(*nut_modbus_free)(ctx);
	}

	return NULL;
}

nutscan_device_t * nutscan_scan_modbus_tcp(const char * start_ip, const char * stop_ip,long usec_timeout, int port)
{
	int i;
	nutscan_ip_iter_t ip;
	char * ip_str = NULL;
#ifdef HAVE_PTHREAD
	pthread_t thread;
	pthread_t * thread_array = NULL;
	int thread_count = 0;

	pthread_mutex_init(&dev_mutex,NULL);
#endif

	if( !nutscan_avail_modbus ) {
		return NULL;
	}

	g_usec_timeout = usec_timeout;

	/* Initialize the Modbus library */
	// FIXME: check for RTU Vs TCP

	upsdebugx(2, "%s: Scanning from IP %s", __func__, start_ip);

	ip_str = nutscan_ip_iter_init(&ip, start_ip, stop_ip);

	while(ip_str != NULL) {

		upsdebugx(2, "%s: Scanning IP %s", __func__, ip_str);
#ifdef HAVE_PTHREAD
		if (pthread_create(&thread, NULL, nutscan_scan_modbus_device, ip_str) == 0){
			thread_count++;
			pthread_t *new_thread_array = realloc(thread_array,
						thread_count*sizeof(pthread_t));
			if (new_thread_array == NULL) {
				upsdebugx(1, "%s: Failed to realloc thread", __func__);
				break;
			}
			else {
				thread_array = new_thread_array;
			}
			thread_array[thread_count-1] = thread;
		}
#else
		nutscan_scan_modbus_device(ip_str);
#endif
		ip_str = nutscan_ip_iter_inc(&ip);
	};

#ifdef HAVE_PTHREAD
	for ( i=0; i < thread_count ; i++) {
		pthread_join(thread_array[i],NULL);
	}
	pthread_mutex_destroy(&dev_mutex);
	free(thread_array);
#endif
	nutscan_device_t * result = nutscan_rewind_device(dev_ret);
	dev_ret = NULL;
	return result;
}
#else /* WITH_MODBUS */
nutscan_device_t * nutscan_scan_modbus_tcp(const char * start_ip, const char * stop_ip,long usec_timeout, int port)
{
	return NULL;
}
#endif /* WITH_MODBUS */
