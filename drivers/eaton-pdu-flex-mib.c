/* eaton-pdu-flex-mib.c - subdriver to monitor eaton-pdu-flex SNMP devices with NUT
 *
 *  Copyright (C)
 *  2011 - 2016	Arnaud Quette <arnaud.quette@free.fr>
 *
 *  Note: this subdriver was initially generated as a "stub" by the
 *  gen-snmp-subdriver script. It must be customized!
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

#include "eaton-pdu-flex-mib.h"

#define EATON_PDU_FLEX_MIB_VERSION  "0.1"

#define EATON_PDU_FLEX_SYSOID       ".1.3.6.1.4.1.55508.1"
#define SYSOID                      EATON_PDU_FLEX_SYSOID

/* To create a value lookup structure (as needed on the 2nd line of the example
 * below), use the following kind of declaration, outside of the present snmp_info_t[]:
 * static info_lkp_t onbatt_info[] = {
 * 	{ 1, "OB" },
 * 	{ 2, "OL" },
 * 	{ 0, NULL }
 * };
 */

/* EATON_PDU_FLEX Snmp2NUT lookup table */
static snmp_info_t eaton_pdu_flex_mib[] = {

/* Data format:
 * { info_type, info_flags, info_len, OID, dfl, flags, oid2info },
 *
 *	info_type:	NUT INFO_ or CMD_ element name
 *	info_flags:	flags to set in addinfo
 *	info_len:	length of strings if ST_FLAG_STRING, multiplier otherwise
 *	OID: SNMP OID or NULL
 *	dfl: default value
 *	flags: snmp-ups internal flags (FIXME: ...)
 *	oid2info: lookup table between OID and NUT values
 *
 * Example:
 * { "input.voltage", 0, 0.1, ".1.3.6.1.4.1.705.1.6.2.1.2.1", "", SU_INPUT_1, NULL },
 * { "ups.status", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.705.1.7.3.0", "", SU_FLAG_OK | SU_STATUS_BATT, onbatt_info },
 *
 * To create a value lookup structure (as needed on the 2nd line), use the
 * following kind of declaration, outside of the present snmp_info_t[]:
 * static info_lkp_t onbatt_info[] = {
 * 	{ 1, "OB" },
 * 	{ 2, "OL" },
 * 	{ 0, NULL }
 * };
 */

/* standard MIB items; if the vendor MIB contains better OIDs for
 * this (e.g. with daisy-chain support), consider adding those here
 */
	/* Device collection */
	{ "device.type", ST_FLAG_STRING, SU_INFOSIZE, NULL, "pdu", SU_FLAG_STATIC | SU_FLAG_ABSENT | SU_FLAG_OK, NULL },
	{ "device.mfr", ST_FLAG_STRING, SU_INFOSIZE, NULL, "Eaton", SU_FLAG_STATIC | SU_FLAG_ABSENT | SU_FLAG_OK, NULL },
	{ "device.description", ST_FLAG_STRING | ST_FLAG_RW, SU_INFOSIZE, ".1.3.6.1.2.1.1.1.0", NULL, SU_FLAG_OK, NULL },
	{ "device.contact", ST_FLAG_STRING | ST_FLAG_RW, SU_INFOSIZE, ".1.3.6.1.2.1.1.4.0", NULL, SU_FLAG_OK, NULL },
	{ "device.location", ST_FLAG_STRING | ST_FLAG_RW, SU_INFOSIZE, ".1.3.6.1.2.1.1.6.0", NULL, SU_FLAG_OK, NULL },
	{ "device.macaddr", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.2.1.2.2.1.6.2", "", SU_FLAG_OK | SU_FLAG_STATIC, NULL },

	{ "outlet.count", 0, 1, NULL, "8", SU_FLAG_STATIC | SU_FLAG_OK, NULL },

/*
 * SYSOID specific
 */

///	/* sourceID.0 = STRING:  */                { "unmapped.sourceID", ST_FLAG_STRING, SU_INFOSIZE,      SYSOID ".1.1.1.0", NULL, SU_FLAG_OK, NULL },
///	/* powerDescr.0 = STRING:  */              { "unmapped.powerDescr", ST_FLAG_STRING, SU_INFOSIZE,    SYSOID ".1.1.2.0", NULL, SU_FLAG_OK, NULL },
///	/* eventSource.0 = STRING: */              { "unmapped.eventSource", ST_FLAG_STRING, SU_INFOSIZE,   SYSOID ".1.1.3.0", NULL, SU_FLAG_OK, NULL },
///	/* sensorDescr.0 = STRING: */              { "unmapped.sensorDescr", ST_FLAG_STRING, SU_INFOSIZE,   SYSOID ".1.1.4.0", NULL, SU_FLAG_OK, NULL },
///	/* eventType.0 = STRING:  */               { "unmapped.eventType", ST_FLAG_STRING, SU_INFOSIZE,     SYSOID ".1.1.5.0", NULL, SU_FLAG_OK, NULL },
///	/* eventDateTime.0 = STRING: */            { "unmapped.eventDateTime", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".1.1.6.0", NULL, SU_FLAG_OK, NULL },
///	/* loadValue.0 = INTEGER: 0 tenth A */     { "unmapped.loadValue", 0, 1,                            SYSOID ".1.1.7.0", NULL, SU_FLAG_OK, NULL },
///	/* loadLowValue.0 = INTEGER: 0 tenth A */  { "unmapped.loadLowValue", 0, 1,                         SYSOID ".1.1.8.0", NULL, SU_FLAG_OK, NULL },
///	/* loadHighValue.0 = INTEGER: 0 tenth A */ { "unmapped.loadHighValue", 0, 1,                        SYSOID ".1.1.9.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageValue.0 = INTEGER: 0 V */        { "unmapped.voltageValue", 0, 1,                         SYSOID ".1.1.10.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageLowValue.0 = INTEGER: 0 V */     { "unmapped.voltageLowValue", 0, 1,                      SYSOID ".1.1.11.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageHighValue.0 = INTEGER: 0 V */    { "unmapped.voltageHighValue", 0, 1,                     SYSOID ".1.1.12.0", NULL, SU_FLAG_OK, NULL },
	{ "device.sourceID", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".1.1.1.0",  NULL, SU_FLAG_OK, NULL },
	{ "power.desc", ST_FLAG_STRING, SU_INFOSIZE,      SYSOID ".1.1.2.0",  NULL, SU_FLAG_OK, NULL },
	{ "event.source", ST_FLAG_STRING, SU_INFOSIZE,    SYSOID ".1.1.3.0",  NULL, SU_FLAG_OK, NULL },
	{ "sensor.desc", ST_FLAG_STRING, SU_INFOSIZE,     SYSOID ".1.1.4.0",  NULL, SU_FLAG_OK, NULL },
	{ "event.type", ST_FLAG_STRING, SU_INFOSIZE,      SYSOID ".1.1.5.0",  NULL, SU_FLAG_OK, NULL },
	{ "event.datetime", ST_FLAG_STRING, SU_INFOSIZE,  SYSOID ".1.1.6.0",  NULL, SU_FLAG_OK, NULL },
	{ "load.value", 0, 0.1,                           SYSOID ".1.1.7.0",  NULL, SU_FLAG_OK, NULL },
	{ "load.value.low", 0, 0.1,                       SYSOID ".1.1.8.0",  NULL, SU_FLAG_OK, NULL },
	{ "load.value.high", 0, 0.1,                      SYSOID ".1.1.9.0",  NULL, SU_FLAG_OK, NULL },
	{ "voltage.value", 0, 1,                          SYSOID ".1.1.10.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.value.low", 0, 1,                      SYSOID ".1.1.11.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.value.high", 0, 1,                     SYSOID ".1.1.12.0", NULL, SU_FLAG_OK, NULL },

///	/* ipAddress.0 = IpAddress: 10.130.245.91 */      { "unmapped.ipAddress", 0, 1,        SYSOID ".1.2.1.0", NULL, SU_FLAG_OK, NULL },
///	/* maskIpAddress.0 = IpAddress: 255.255.255.0 */  { "unmapped.maskIpAddress", 0, 1,    SYSOID ".1.2.2.0", NULL, SU_FLAG_OK, NULL },
///	/* gatewayIpAddress.0 = IpAddress: 192.168.0.1 */ { "unmapped.gatewayIpAddress", 0, 1, SYSOID ".1.2.3.0", NULL, SU_FLAG_OK, NULL },
///	/* dnsIpAddress1.0 = IpAddress: 0.0.0.0 */        { "unmapped.dnsIpAddress1", 0, 1,    SYSOID ".1.2.4.0", NULL, SU_FLAG_OK, NULL },
///	/* dnsIpAddress2.0 = IpAddress: 0.0.0.0 */        { "unmapped.dnsIpAddress2", 0, 1,    SYSOID ".1.2.5.0", NULL, SU_FLAG_OK, NULL },
///	/* rebootSystem.0 = INTEGER: 0 */                 { "unmapped.rebootSystem", 0, 1,     SYSOID ".1.2.6.0", NULL, SU_FLAG_OK, NULL },
///	/* trapDestIP1.0 = IpAddress: 0.0.0.0 */          { "unmapped.trapDestIP1", 0, 1,      SYSOID ".1.2.7.0", NULL, SU_FLAG_OK, NULL },
///	/* trapDestIP2.0 = IpAddress: 0.0.0.0 */          { "unmapped.trapDestIP2", 0, 1,      SYSOID ".1.2.8.0", NULL, SU_FLAG_OK, NULL },
///	/* trapDestIP3.0 = IpAddress: 0.0.0.0 */          { "unmapped.trapDestIP3", 0, 1,      SYSOID ".1.2.9.0", NULL, SU_FLAG_OK, NULL },
///	/* trapDestIP4.0 = IpAddress: 0.0.0.0 */          { "unmapped.trapDestIP4", 0, 1,      SYSOID ".1.2.10.0", NULL, SU_FLAG_OK, NULL },
	{ "device.ip.address", 0, 1,     SYSOID ".1.2.1.0",  NULL, SU_FLAG_OK, NULL },
	{ "device.ip.mask", 0, 1,        SYSOID ".1.2.2.0",  NULL, SU_FLAG_OK, NULL },
	{ "device.ip.gateway", 0, 1,     SYSOID ".1.2.3.0",  NULL, SU_FLAG_OK, NULL },
	{ "device.ip.dns.1", 0, 1,       SYSOID ".1.2.4.0",  NULL, SU_FLAG_OK, NULL },
	{ "device.ip.dns.2", 0, 1,       SYSOID ".1.2.5.0",  NULL, SU_FLAG_OK, NULL },
	{ "device.system.reboot", 0, 1,  SYSOID ".1.2.6.0",  NULL, SU_FLAG_OK, NULL },
	{ "device.ip.trap.dest.1", 0, 1, SYSOID ".1.2.7.0",  NULL, SU_FLAG_OK, NULL },
	{ "device.ip.trap.dest.2", 0, 1, SYSOID ".1.2.8.0",  NULL, SU_FLAG_OK, NULL },
	{ "device.ip.trap.dest.3", 0, 1, SYSOID ".1.2.9.0",  NULL, SU_FLAG_OK, NULL },
	{ "device.ip.trap.dest.4", 0, 1, SYSOID ".1.2.10.0", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletSeq.0 = INTEGER: 0 */ { "unmapped.p1OutletSeq", 0, 1, SYSOID ".2.1.1.1.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSeq.1 = INTEGER: 1 */ { "unmapped.p1OutletSeq", 0, 1, SYSOID ".2.1.1.1.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSeq.2 = INTEGER: 2 */ { "unmapped.p1OutletSeq", 0, 1, SYSOID ".2.1.1.1.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSeq.3 = INTEGER: 3 */ { "unmapped.p1OutletSeq", 0, 1, SYSOID ".2.1.1.1.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSeq.4 = INTEGER: 4 */ { "unmapped.p1OutletSeq", 0, 1, SYSOID ".2.1.1.1.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSeq.5 = INTEGER: 5 */ { "unmapped.p1OutletSeq", 0, 1, SYSOID ".2.1.1.1.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSeq.6 = INTEGER: 6 */ { "unmapped.p1OutletSeq", 0, 1, SYSOID ".2.1.1.1.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSeq.7 = INTEGER: 7 */ { "unmapped.p1OutletSeq", 0, 1, SYSOID ".2.1.1.1.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.seq.0", 0, 1, SYSOID ".2.1.1.1.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.seq.1", 0, 1, SYSOID ".2.1.1.1.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.seq.2", 0, 1, SYSOID ".2.1.1.1.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.seq.3", 0, 1, SYSOID ".2.1.1.1.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.seq.4", 0, 1, SYSOID ".2.1.1.1.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.seq.5", 0, 1, SYSOID ".2.1.1.1.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.seq.6", 0, 1, SYSOID ".2.1.1.1.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.seq.7", 0, 1, SYSOID ".2.1.1.1.7", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletNo.0 = STRING: 1 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletNo.1 = STRING: 2 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletNo.2 = STRING: 3 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletNo.3 = STRING: 4 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletNo.4 = STRING: 5 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletNo.5 = STRING: 6 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletNo.6 = STRING: 7 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletNo.7 = STRING: 8 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.no.0", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.no.1", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.no.2", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.no.3", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.no.4", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.no.5", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.no.6", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.no.7", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.2.7", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletDesc.0 = STRING: Outlet_1 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletDesc.1 = STRING: Outlet_2 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletDesc.2 = STRING: Outlet_3 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletDesc.3 = STRING: Outlet_4 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletDesc.4 = STRING: Outlet_5 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletDesc.5 = STRING: Outlet_6 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletDesc.6 = STRING: Outlet_7 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletDesc.7 = STRING: Outlet_8 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.desc.0", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.desc.1", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.desc.2", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.desc.3", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.desc.4", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.desc.5", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.desc.6", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.desc.7", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.3.7", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletSocket.0 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSocket.1 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSocket.2 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSocket.3 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSocket.4 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSocket.5 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSocket.6 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletSocket.7 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.socket.0", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.socket.1", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.socket.2", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.socket.3", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.socket.4", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.socket.5", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.socket.6", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.socket.7", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.4.7", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletFuse.0 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletFuse.1 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletFuse.2 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletFuse.3 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletFuse.4 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletFuse.5 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletFuse.6 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletFuse.7 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.fuse.0", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.fuse.1", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.fuse.2", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.fuse.3", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.fuse.4", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.fuse.5", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.fuse.6", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.fuse.7", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.1.1.5.7", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletOnOff.0 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, SYSOID ".2.1.1.6.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletOnOff.1 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, SYSOID ".2.1.1.6.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletOnOff.2 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, SYSOID ".2.1.1.6.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletOnOff.3 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, SYSOID ".2.1.1.6.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletOnOff.4 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, SYSOID ".2.1.1.6.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletOnOff.5 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, SYSOID ".2.1.1.6.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletOnOff.6 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, SYSOID ".2.1.1.6.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletOnOff.7 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, SYSOID ".2.1.1.6.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.onOff.0", 0, 1, SYSOID ".2.1.1.6.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.onOff.1", 0, 1, SYSOID ".2.1.1.6.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.onOff.2", 0, 1, SYSOID ".2.1.1.6.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.onOff.3", 0, 1, SYSOID ".2.1.1.6.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.onOff.4", 0, 1, SYSOID ".2.1.1.6.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.onOff.5", 0, 1, SYSOID ".2.1.1.6.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.onOff.6", 0, 1, SYSOID ".2.1.1.6.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.onOff.7", 0, 1, SYSOID ".2.1.1.6.7", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletLoad.0 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, SYSOID ".2.1.1.7.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoad.1 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, SYSOID ".2.1.1.7.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoad.2 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, SYSOID ".2.1.1.7.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoad.3 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, SYSOID ".2.1.1.7.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoad.4 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, SYSOID ".2.1.1.7.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoad.5 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, SYSOID ".2.1.1.7.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoad.6 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, SYSOID ".2.1.1.7.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoad.7 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, SYSOID ".2.1.1.7.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.0", 0, 0.1, SYSOID ".2.1.1.7.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.1", 0, 0.1, SYSOID ".2.1.1.7.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.2", 0, 0.1, SYSOID ".2.1.1.7.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.3", 0, 0.1, SYSOID ".2.1.1.7.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.4", 0, 0.1, SYSOID ".2.1.1.7.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.5", 0, 0.1, SYSOID ".2.1.1.7.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.6", 0, 0.1, SYSOID ".2.1.1.7.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.7", 0, 0.1, SYSOID ".2.1.1.7.7", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletLoadLowLimit.0 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, SYSOID ".2.1.1.8.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadLowLimit.1 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, SYSOID ".2.1.1.8.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadLowLimit.2 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, SYSOID ".2.1.1.8.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadLowLimit.3 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, SYSOID ".2.1.1.8.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadLowLimit.4 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, SYSOID ".2.1.1.8.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadLowLimit.5 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, SYSOID ".2.1.1.8.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadLowLimit.6 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, SYSOID ".2.1.1.8.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadLowLimit.7 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, SYSOID ".2.1.1.8.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.low.0", 0, 0.1, SYSOID ".2.1.1.8.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.low.1", 0, 0.1, SYSOID ".2.1.1.8.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.low.2", 0, 0.1, SYSOID ".2.1.1.8.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.low.3", 0, 0.1, SYSOID ".2.1.1.8.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.low.4", 0, 0.1, SYSOID ".2.1.1.8.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.low.5", 0, 0.1, SYSOID ".2.1.1.8.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.low.6", 0, 0.1, SYSOID ".2.1.1.8.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.low.7", 0, 0.1, SYSOID ".2.1.1.8.7", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletLoadHighLimit.0 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, SYSOID ".2.1.1.9.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadHighLimit.1 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, SYSOID ".2.1.1.9.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadHighLimit.2 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, SYSOID ".2.1.1.9.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadHighLimit.3 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, SYSOID ".2.1.1.9.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadHighLimit.4 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, SYSOID ".2.1.1.9.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadHighLimit.5 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, SYSOID ".2.1.1.9.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadHighLimit.6 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, SYSOID ".2.1.1.9.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletLoadHighLimit.7 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, SYSOID ".2.1.1.9.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.high.0", 0, 0.1, SYSOID ".2.1.1.9.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.high.1", 0, 0.1, SYSOID ".2.1.1.9.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.high.2", 0, 0.1, SYSOID ".2.1.1.9.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.high.3", 0, 0.1, SYSOID ".2.1.1.9.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.high.4", 0, 0.1, SYSOID ".2.1.1.9.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.high.5", 0, 0.1, SYSOID ".2.1.1.9.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.high.6", 0, 0.1, SYSOID ".2.1.1.9.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.load.limit.high.7", 0, 0.1, SYSOID ".2.1.1.9.7", NULL, SU_FLAG_OK, NULL },

///	/* p1OutletEnergy.0 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, SYSOID ".2.1.1.10.0", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletEnergy.1 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, SYSOID ".2.1.1.10.1", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletEnergy.2 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, SYSOID ".2.1.1.10.2", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletEnergy.3 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, SYSOID ".2.1.1.10.3", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletEnergy.4 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, SYSOID ".2.1.1.10.4", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletEnergy.5 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, SYSOID ".2.1.1.10.5", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletEnergy.6 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, SYSOID ".2.1.1.10.6", NULL, SU_FLAG_OK, NULL },
///	/* p1OutletEnergy.7 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, SYSOID ".2.1.1.10.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.energy.0", 0, 0.01, SYSOID ".2.1.1.10.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.energy.1", 0, 0.01, SYSOID ".2.1.1.10.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.energy.2", 0, 0.01, SYSOID ".2.1.1.10.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.energy.3", 0, 0.01, SYSOID ".2.1.1.10.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.energy.4", 0, 0.01, SYSOID ".2.1.1.10.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.energy.5", 0, 0.01, SYSOID ".2.1.1.10.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.energy.6", 0, 0.01, SYSOID ".2.1.1.10.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.energy.7", 0, 0.01, SYSOID ".2.1.1.10.7", NULL, SU_FLAG_OK, NULL },

///	/* p1Power.0 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, SYSOID ".2.1.1.11.0", NULL, SU_FLAG_OK, NULL },
///	/* p1Power.1 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, SYSOID ".2.1.1.11.1", NULL, SU_FLAG_OK, NULL },
///	/* p1Power.2 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, SYSOID ".2.1.1.11.2", NULL, SU_FLAG_OK, NULL },
///	/* p1Power.3 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, SYSOID ".2.1.1.11.3", NULL, SU_FLAG_OK, NULL },
///	/* p1Power.4 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, SYSOID ".2.1.1.11.4", NULL, SU_FLAG_OK, NULL },
///	/* p1Power.5 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, SYSOID ".2.1.1.11.5", NULL, SU_FLAG_OK, NULL },
///	/* p1Power.6 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, SYSOID ".2.1.1.11.6", NULL, SU_FLAG_OK, NULL },
///	/* p1Power.7 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, SYSOID ".2.1.1.11.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.power.0", 0, 0.01, SYSOID ".2.1.1.11.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.power.1", 0, 0.01, SYSOID ".2.1.1.11.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.power.2", 0, 0.01, SYSOID ".2.1.1.11.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.power.3", 0, 0.01, SYSOID ".2.1.1.11.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.power.4", 0, 0.01, SYSOID ".2.1.1.11.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.power.5", 0, 0.01, SYSOID ".2.1.1.11.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.power.6", 0, 0.01, SYSOID ".2.1.1.11.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.power.7", 0, 0.01, SYSOID ".2.1.1.11.7", NULL, SU_FLAG_OK, NULL },

///	/* p1PowerFactor.0 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, SYSOID ".2.1.1.12.0", NULL, SU_FLAG_OK, NULL },
///	/* p1PowerFactor.1 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, SYSOID ".2.1.1.12.1", NULL, SU_FLAG_OK, NULL },
///	/* p1PowerFactor.2 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, SYSOID ".2.1.1.12.2", NULL, SU_FLAG_OK, NULL },
///	/* p1PowerFactor.3 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, SYSOID ".2.1.1.12.3", NULL, SU_FLAG_OK, NULL },
///	/* p1PowerFactor.4 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, SYSOID ".2.1.1.12.4", NULL, SU_FLAG_OK, NULL },
///	/* p1PowerFactor.5 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, SYSOID ".2.1.1.12.5", NULL, SU_FLAG_OK, NULL },
///	/* p1PowerFactor.6 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, SYSOID ".2.1.1.12.6", NULL, SU_FLAG_OK, NULL },
///	/* p1PowerFactor.7 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, SYSOID ".2.1.1.12.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.powerFactor.0", 0, 0.01, SYSOID ".2.1.1.12.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.powerFactor.1", 0, 0.01, SYSOID ".2.1.1.12.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.powerFactor.2", 0, 0.01, SYSOID ".2.1.1.12.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.powerFactor.3", 0, 0.01, SYSOID ".2.1.1.12.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.powerFactor.4", 0, 0.01, SYSOID ".2.1.1.12.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.powerFactor.5", 0, 0.01, SYSOID ".2.1.1.12.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.powerFactor.6", 0, 0.01, SYSOID ".2.1.1.12.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.powerFactor.7", 0, 0.01, SYSOID ".2.1.1.12.7", NULL, SU_FLAG_OK, NULL },

///	/* power1OutletEntry.13.0 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, SYSOID ".2.1.1.13.0", NULL, SU_FLAG_OK, NULL },
///	/* power1OutletEntry.13.1 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, SYSOID ".2.1.1.13.1", NULL, SU_FLAG_OK, NULL },
///	/* power1OutletEntry.13.2 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, SYSOID ".2.1.1.13.2", NULL, SU_FLAG_OK, NULL },
///	/* power1OutletEntry.13.3 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, SYSOID ".2.1.1.13.3", NULL, SU_FLAG_OK, NULL },
///	/* power1OutletEntry.13.4 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, SYSOID ".2.1.1.13.4", NULL, SU_FLAG_OK, NULL },
///	/* power1OutletEntry.13.5 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, SYSOID ".2.1.1.13.5", NULL, SU_FLAG_OK, NULL },
///	/* power1OutletEntry.13.6 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, SYSOID ".2.1.1.13.6", NULL, SU_FLAG_OK, NULL },
///	/* power1OutletEntry.13.7 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, SYSOID ".2.1.1.13.7", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.entry.0", 0, 1, SYSOID ".2.1.1.13.0", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.entry.1", 0, 1, SYSOID ".2.1.1.13.1", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.entry.2", 0, 1, SYSOID ".2.1.1.13.2", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.entry.3", 0, 1, SYSOID ".2.1.1.13.3", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.entry.4", 0, 1, SYSOID ".2.1.1.13.4", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.entry.5", 0, 1, SYSOID ".2.1.1.13.5", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.entry.6", 0, 1, SYSOID ".2.1.1.13.6", NULL, SU_FLAG_OK, NULL },
	{ "power.1.outlet.entry.7", 0, 1, SYSOID ".2.1.1.13.7", NULL, SU_FLAG_OK, NULL },

///	/* p3OutletSeq.0 = INTEGER: 0 */ { "unmapped.p3OutletSeq", 0, 1, SYSOID ".2.3.1.1.0", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSeq.1 = INTEGER: 1 */ { "unmapped.p3OutletSeq", 0, 1, SYSOID ".2.3.1.1.1", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSeq.2 = INTEGER: 2 */ { "unmapped.p3OutletSeq", 0, 1, SYSOID ".2.3.1.1.2", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSeq.3 = INTEGER: 3 */ { "unmapped.p3OutletSeq", 0, 1, SYSOID ".2.3.1.1.3", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSeq.4 = INTEGER: 4 */ { "unmapped.p3OutletSeq", 0, 1, SYSOID ".2.3.1.1.4", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSeq.5 = INTEGER: 5 */ { "unmapped.p3OutletSeq", 0, 1, SYSOID ".2.3.1.1.5", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSeq.6 = INTEGER: 6 */ { "unmapped.p3OutletSeq", 0, 1, SYSOID ".2.3.1.1.6", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSeq.7 = INTEGER: 7 */ { "unmapped.p3OutletSeq", 0, 1, SYSOID ".2.3.1.1.7", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.seq.0", 0, 1, SYSOID ".2.3.1.1.0", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.seq.1", 0, 1, SYSOID ".2.3.1.1.1", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.seq.2", 0, 1, SYSOID ".2.3.1.1.2", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.seq.3", 0, 1, SYSOID ".2.3.1.1.3", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.seq.4", 0, 1, SYSOID ".2.3.1.1.4", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.seq.5", 0, 1, SYSOID ".2.3.1.1.5", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.seq.6", 0, 1, SYSOID ".2.3.1.1.6", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.seq.7", 0, 1, SYSOID ".2.3.1.1.7", NULL, SU_FLAG_OK, NULL },

///	/* p3OutletNo.0 = STRING: 1 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.0", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletNo.1 = STRING: 2 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.1", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletNo.2 = STRING: 3 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.2", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletNo.3 = STRING: 4 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.3", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletNo.4 = STRING: 5 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.4", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletNo.5 = STRING: 6 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.5", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletNo.6 = STRING: 7 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.6", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletNo.7 = STRING: 8 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.7", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.no.0", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.0", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.no.1", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.1", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.no.2", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.2", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.no.3", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.3", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.no.4", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.4", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.no.5", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.5", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.no.6", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.6", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.no.7", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.2.7", NULL, SU_FLAG_OK, NULL },

///	/* p3OutletDesc.0 = STRING: ALL */     { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.0", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletDesc.1 = STRING: Group 2 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.1", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletDesc.2 = STRING: Group 3 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.2", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletDesc.3 = STRING: Group 4 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.3", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletDesc.4 = STRING: Group 5 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.4", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletDesc.5 = STRING: Group 6 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.5", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletDesc.6 = STRING: Group 7 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.6", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletDesc.7 = STRING: Group 8 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.7", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.desc.0", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.0", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.desc.1", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.1", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.desc.2", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.2", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.desc.3", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.3", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.desc.4", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.4", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.desc.5", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.5", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.desc.6", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.6", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.desc.7", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.3.7", NULL, SU_FLAG_OK, NULL },

///	/* p3OutletSocket.0 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.0", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSocket.1 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.1", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSocket.2 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.2", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSocket.3 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.3", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSocket.4 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.4", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSocket.5 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.5", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSocket.6 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.6", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletSocket.7 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.7", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.socket.0", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.0", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.socket.1", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.1", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.socket.2", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.2", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.socket.3", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.3", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.socket.4", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.4", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.socket.5", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.5", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.socket.6", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.6", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.socket.7", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.4.7", NULL, SU_FLAG_OK, NULL },

///	/* p3OutletFuse.0 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.0", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletFuse.1 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.1", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletFuse.2 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.2", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletFuse.3 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.3", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletFuse.4 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.4", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletFuse.5 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.5", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletFuse.6 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.6", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletFuse.7 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.7", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.fuse.0", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.0", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.fuse.1", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.1", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.fuse.2", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.2", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.fuse.3", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.3", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.fuse.4", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.4", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.fuse.5", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.5", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.fuse.6", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.6", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.fuse.7", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.3.1.5.7", NULL, SU_FLAG_OK, NULL },

///	/* p3OutletOnOff.0 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, SYSOID ".2.3.1.6.0", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletOnOff.1 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, SYSOID ".2.3.1.6.1", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletOnOff.2 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, SYSOID ".2.3.1.6.2", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletOnOff.3 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, SYSOID ".2.3.1.6.3", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletOnOff.4 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, SYSOID ".2.3.1.6.4", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletOnOff.5 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, SYSOID ".2.3.1.6.5", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletOnOff.6 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, SYSOID ".2.3.1.6.6", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletOnOff.7 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, SYSOID ".2.3.1.6.7", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.onOff.0", 0, 1, SYSOID ".2.3.1.6.0", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.onOff.1", 0, 1, SYSOID ".2.3.1.6.1", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.onOff.2", 0, 1, SYSOID ".2.3.1.6.2", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.onOff.3", 0, 1, SYSOID ".2.3.1.6.3", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.onOff.4", 0, 1, SYSOID ".2.3.1.6.4", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.onOff.5", 0, 1, SYSOID ".2.3.1.6.5", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.onOff.6", 0, 1, SYSOID ".2.3.1.6.6", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.onOff.7", 0, 1, SYSOID ".2.3.1.6.7", NULL, SU_FLAG_OK, NULL },

///	/* p3OutletLoad.0 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, SYSOID ".2.3.1.7.0", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletLoad.1 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, SYSOID ".2.3.1.7.1", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletLoad.2 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, SYSOID ".2.3.1.7.2", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletLoad.3 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, SYSOID ".2.3.1.7.3", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletLoad.4 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, SYSOID ".2.3.1.7.4", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletLoad.5 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, SYSOID ".2.3.1.7.5", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletLoad.6 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, SYSOID ".2.3.1.7.6", NULL, SU_FLAG_OK, NULL },
///	/* p3OutletLoad.7 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, SYSOID ".2.3.1.7.7", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.load.0", 0, 0.1, SYSOID ".2.3.1.7.0", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.load.1", 0, 0.1, SYSOID ".2.3.1.7.1", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.load.2", 0, 0.1, SYSOID ".2.3.1.7.2", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.load.3", 0, 0.1, SYSOID ".2.3.1.7.3", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.load.4", 0, 0.1, SYSOID ".2.3.1.7.4", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.load.5", 0, 0.1, SYSOID ".2.3.1.7.5", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.load.6", 0, 0.1, SYSOID ".2.3.1.7.6", NULL, SU_FLAG_OK, NULL },
	{ "power.3.outlet.load.7", 0, 0.1, SYSOID ".2.3.1.7.7", NULL, SU_FLAG_OK, NULL },

///	/* powerSeq.0 = INTEGER: 0 */            { "unmapped.powerSeq", 0, 1,                          SYSOID ".2.5.1.1.0", NULL, SU_FLAG_OK, NULL },
///	/* powerID.0 = STRING: 505197 */         { "unmapped.powerID", ST_FLAG_STRING, SU_INFOSIZE,    SYSOID ".2.5.1.2.0", NULL, SU_FLAG_OK, NULL },
///	/* powerName.0 = STRING: FlexPDU */      { "unmapped.powerName", ST_FLAG_STRING, SU_INFOSIZE,  SYSOID ".2.5.1.3.0", NULL, SU_FLAG_OK, NULL },
///	/* powerType.0 = STRING: AC */           { "unmapped.powerType", ST_FLAG_STRING, SU_INFOSIZE,  SYSOID ".2.5.1.4.0", NULL, SU_FLAG_OK, NULL },
///	/* powerModel.0 = STRING: FLXTSWC20C1 */ { "unmapped.powerModel", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.5.1.5.0", NULL, SU_FLAG_OK, NULL },
///	/* powerOnGap.0 = INTEGER: 1 */          { "unmapped.powerOnGap", 0, 1,                        SYSOID ".2.5.1.6.0", NULL, SU_FLAG_OK, NULL },
	{ "power.seq", 0, 1,                          SYSOID ".2.5.1.1.0", NULL, SU_FLAG_OK, NULL },
	{ "power.ID", ST_FLAG_STRING, SU_INFOSIZE,    SYSOID ".2.5.1.2.0", NULL, SU_FLAG_OK, NULL },
	{ "power.name", ST_FLAG_STRING, SU_INFOSIZE,  SYSOID ".2.5.1.3.0", NULL, SU_FLAG_OK, NULL },
	{ "power.type", ST_FLAG_STRING, SU_INFOSIZE,  SYSOID ".2.5.1.4.0", NULL, SU_FLAG_OK, NULL },
	{ "power.model", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.5.1.5.0", NULL, SU_FLAG_OK, NULL },
	{ "power.onGap", 0, 1,                        SYSOID ".2.5.1.6.0", NULL, SU_FLAG_OK, NULL },

	{ "device.model", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.5.1.5.0", NULL, SU_FLAG_OK, NULL }, // power.model

///	/* totalLoadA.0 = INTEGER: 0 tenth A */            { "unmapped.totalLoadA", 0, 1,          SYSOID ".2.5.1.7.0", NULL, SU_FLAG_OK, NULL },
///	/* totalLoadALowLimit.0 = INTEGER: 0 tenth A */    { "unmapped.totalLoadALowLimit", 0, 1,  SYSOID ".2.5.1.8.0", NULL, SU_FLAG_OK, NULL },
///	/* totalLoadAHighLimit.0 = INTEGER: 160 tenth A */ { "unmapped.totalLoadAHighLimit", 0, 1, SYSOID ".2.5.1.9.0", NULL, SU_FLAG_OK, NULL },
///	/* totalLoadB.0 = INTEGER: -1 tenth A */           { "unmapped.totalLoadB", 0, 1,          SYSOID ".2.5.1.10.0", NULL, SU_FLAG_OK, NULL },
///	/* totalLoadBLowLimit.0 = INTEGER: -1 tenth A */   { "unmapped.totalLoadBLowLimit", 0, 1,  SYSOID ".2.5.1.11.0", NULL, SU_FLAG_OK, NULL },
///	/* totalLoadBHighLimit.0 = INTEGER: -1 tenth A */  { "unmapped.totalLoadBHighLimit", 0, 1, SYSOID ".2.5.1.12.0", NULL, SU_FLAG_OK, NULL },
///	/* totalLoadC.0 = INTEGER: -1 tenth A */           { "unmapped.totalLoadC", 0, 1,          SYSOID ".2.5.1.13.0", NULL, SU_FLAG_OK, NULL },
///	/* totalLoadCLowLimit.0 = INTEGER: -1 tenth A */   { "unmapped.totalLoadCLowLimit", 0, 1,  SYSOID ".2.5.1.14.0", NULL, SU_FLAG_OK, NULL },
///	/* totalLoadCHighLimit.0 = INTEGER: -1 tenth A */  { "unmapped.totalLoadCHighLimit", 0, 1, SYSOID ".2.5.1.15.0", NULL, SU_FLAG_OK, NULL },
	{ "load.A.total", 0, 0.1,            SYSOID ".2.5.1.7.0", NULL, SU_FLAG_OK, NULL },
	{ "load.A.total.limit.low", 0, 0.1,  SYSOID ".2.5.1.8.0", NULL, SU_FLAG_OK, NULL },
	{ "load.A.total.limit.high", 0, 0.1, SYSOID ".2.5.1.9.0", NULL, SU_FLAG_OK, NULL },
	{ "load.B.total", 0, 0.1,            SYSOID ".2.5.1.10.0", NULL, SU_FLAG_OK, NULL },
	{ "load.B.total.limit.low", 0, 0.1,  SYSOID ".2.5.1.11.0", NULL, SU_FLAG_OK, NULL },
	{ "load.B.total.limit.high", 0, 0.1, SYSOID ".2.5.1.12.0", NULL, SU_FLAG_OK, NULL },
	{ "load.C.total", 0, 0.1,            SYSOID ".2.5.1.13.0", NULL, SU_FLAG_OK, NULL },
	{ "load.C.total.limit.low", 0, 0.1,  SYSOID ".2.5.1.14.0", NULL, SU_FLAG_OK, NULL },
	{ "load.C.total.limit.high", 0, 0.1, SYSOID ".2.5.1.15.0", NULL, SU_FLAG_OK, NULL },

///	/* voltageA.0 = INTEGER: 241 V */          { "unmapped.voltageA", 0, 1,          SYSOID ".2.5.1.16.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageB.0 = INTEGER: -1 V */           { "unmapped.voltageB", 0, 1,          SYSOID ".2.5.1.17.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageC.0 = INTEGER: -1 V */           { "unmapped.voltageC", 0, 1,          SYSOID ".2.5.1.18.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageALowLimit.0 = INTEGER: 190 V */  { "unmapped.voltageALowLimit", 0, 1,  SYSOID ".2.5.1.48.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageAHighLimit.0 = INTEGER: 280 V */ { "unmapped.voltageAHighLimit", 0, 1, SYSOID ".2.5.1.49.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageBLowLimit.0 = INTEGER: -1 V */   { "unmapped.voltageBLowLimit", 0, 1,  SYSOID ".2.5.1.50.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageBHighLimit.0 = INTEGER: -1 V */  { "unmapped.voltageBHighLimit", 0, 1, SYSOID ".2.5.1.51.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageCLowLimit.0 = INTEGER: -1 V */   { "unmapped.voltageCLowLimit", 0, 1,  SYSOID ".2.5.1.52.0", NULL, SU_FLAG_OK, NULL },
///	/* voltageCHighLimit.0 = INTEGER: -1 V */  { "unmapped.voltageCHighLimit", 0, 1, SYSOID ".2.5.1.53.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.A", 0, 1,            SYSOID ".2.5.1.16.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.B", 0, 1,            SYSOID ".2.5.1.17.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.C", 0, 1,            SYSOID ".2.5.1.18.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.A.limit.low", 0, 1,  SYSOID ".2.5.1.48.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.A.limit.high", 0, 1, SYSOID ".2.5.1.49.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.B.limit.low", 0, 1,  SYSOID ".2.5.1.50.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.B.limit.high", 0, 1, SYSOID ".2.5.1.51.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.C.limit.low", 0, 1,  SYSOID ".2.5.1.52.0", NULL, SU_FLAG_OK, NULL },
	{ "voltage.C.limit.high", 0, 1, SYSOID ".2.5.1.53.0", NULL, SU_FLAG_OK, NULL },

///	/* powerEnergy.0 = INTEGER: 0 tenth kWh */    { "unmapped.powerEnergy", 0, 1,    SYSOID ".2.5.1.19.0", NULL, SU_FLAG_OK, NULL },
///	/* activePowerA.0 = INTEGER: 0 tenth W */     { "unmapped.activePowerA", 0, 1,   SYSOID ".2.5.1.20.0", NULL, SU_FLAG_OK, NULL },
///	/* ratedPowerA.0 = INTEGER: 35200 tenth W */  { "unmapped.ratedPowerA", 0, 1,    SYSOID ".2.5.1.21.0", NULL, SU_FLAG_OK, NULL },
///	/* remainPowerA.0 = INTEGER: 35200 tenth W */ { "unmapped.remainPowerA", 0, 1,   SYSOID ".2.5.1.22.0", NULL, SU_FLAG_OK, NULL },
///	/* apparentPowerA.0 = INTEGER: 0 tenth W */   { "unmapped.apparentPowerA", 0, 1, SYSOID ".2.5.1.23.0", NULL, SU_FLAG_OK, NULL },
///	/* powerFactorA.0 = INTEGER: 0 hundredth */   { "unmapped.powerFactorA", 0, 1,   SYSOID ".2.5.1.24.0", NULL, SU_FLAG_OK, NULL },
///	/* activePowerB.0 = INTEGER: -1 tenth W */    { "unmapped.activePowerB", 0, 1,   SYSOID ".2.5.1.25.0", NULL, SU_FLAG_OK, NULL },
///	/* ratedPowerB.0 = INTEGER: -1 tenth W */     { "unmapped.ratedPowerB", 0, 1,    SYSOID ".2.5.1.26.0", NULL, SU_FLAG_OK, NULL },
///	/* remainPowerB.0 = INTEGER: -1 tenth W */    { "unmapped.remainPowerB", 0, 1,   SYSOID ".2.5.1.27.0", NULL, SU_FLAG_OK, NULL },
///	/* apparentPowerB.0 = INTEGER: -1 tenth W */  { "unmapped.apparentPowerB", 0, 1, SYSOID ".2.5.1.28.0", NULL, SU_FLAG_OK, NULL },
///	/* powerFactorB.0 = INTEGER: -1 hundredth */  { "unmapped.powerFactorB", 0, 1,   SYSOID ".2.5.1.29.0", NULL, SU_FLAG_OK, NULL },
///	/* activePowerC.0 = INTEGER: -1 tenth W */    { "unmapped.activePowerC", 0, 1,   SYSOID ".2.5.1.30.0", NULL, SU_FLAG_OK, NULL },
///	/* ratedPowerC.0 = INTEGER: -1 tenth W */     { "unmapped.ratedPowerC", 0, 1,    SYSOID ".2.5.1.31.0", NULL, SU_FLAG_OK, NULL },
///	/* remainPowerC.0 = INTEGER: -1 tenth W */    { "unmapped.remainPowerC", 0, 1,   SYSOID ".2.5.1.32.0", NULL, SU_FLAG_OK, NULL },
///	/* apparentPowerC.0 = INTEGER: -1 tenth W */  { "unmapped.apparentPowerC", 0, 1, SYSOID ".2.5.1.33.0", NULL, SU_FLAG_OK, NULL },
///	/* powerFactorC.0 = INTEGER: -1 hundredth */  { "unmapped.powerFactorC", 0, 1,   SYSOID ".2.5.1.34.0", NULL, SU_FLAG_OK, NULL },
	{ "power.energy", 0, 0.1,         SYSOID ".2.5.1.19.0", NULL, SU_FLAG_OK, NULL },
	{ "power.A.active", 0, 0.1,       SYSOID ".2.5.1.20.0", NULL, SU_FLAG_OK, NULL },
	{ "power.A.rated", 0, 0.1,        SYSOID ".2.5.1.21.0", NULL, SU_FLAG_OK, NULL },
	{ "power.A.remain", 0, 0.1,       SYSOID ".2.5.1.22.0", NULL, SU_FLAG_OK, NULL },
	{ "power.A.apparent", 0, 0.1,     SYSOID ".2.5.1.23.0", NULL, SU_FLAG_OK, NULL },
	{ "power.A.powerFactor", 0, 0.01, SYSOID ".2.5.1.24.0", NULL, SU_FLAG_OK, NULL },
	{ "power.B.active", 0, 0.1,       SYSOID ".2.5.1.25.0", NULL, SU_FLAG_OK, NULL },
	{ "power.B.rated", 0, 0.1,        SYSOID ".2.5.1.26.0", NULL, SU_FLAG_OK, NULL },
	{ "power.B.remain", 0, 0.1,       SYSOID ".2.5.1.27.0", NULL, SU_FLAG_OK, NULL },
	{ "power.B.apparent", 0, 0.1,     SYSOID ".2.5.1.28.0", NULL, SU_FLAG_OK, NULL },
	{ "power.B.powerFactor", 0, 0.01, SYSOID ".2.5.1.29.0", NULL, SU_FLAG_OK, NULL },
	{ "power.C.active", 0, 0.1,       SYSOID ".2.5.1.30.0", NULL, SU_FLAG_OK, NULL },
	{ "power.C.rated", 0, 0.1,        SYSOID ".2.5.1.31.0", NULL, SU_FLAG_OK, NULL },
	{ "power.C.remain", 0, 0.1,       SYSOID ".2.5.1.32.0", NULL, SU_FLAG_OK, NULL },
	{ "power.C.apparent", 0, 0.1,     SYSOID ".2.5.1.33.0", NULL, SU_FLAG_OK, NULL },
	{ "power.C.powerFactor", 0, 0.01, SYSOID ".2.5.1.34.0", NULL, SU_FLAG_OK, NULL },

///	/* atsInput.0 = STRING: -- */              { "unmapped.atsInput", ST_FLAG_STRING, SU_INFOSIZE,      SYSOID ".2.5.1.35.0", NULL, SU_FLAG_OK, NULL },
///	/* atsPriority.0 = STRING: -- */           { "unmapped.atsPriority", ST_FLAG_STRING, SU_INFOSIZE,   SYSOID ".2.5.1.36.0", NULL, SU_FLAG_OK, NULL },
///	/* atsAVoltage.0 = INTEGER: -1 V */        { "unmapped.atsAVoltage", 0, 1,                          SYSOID ".2.5.1.37.0", NULL, SU_FLAG_OK, NULL },
///	/* atsBVoltage.0 = INTEGER: -1 V */        { "unmapped.atsBVoltage", 0, 1,                          SYSOID ".2.5.1.38.0", NULL, SU_FLAG_OK, NULL },
///	/* atsAEnergy.0 = INTEGER: -1 tenth kWh */ { "unmapped.atsAEnergy", 0, 1,                           SYSOID ".2.5.1.39.0", NULL, SU_FLAG_OK, NULL },
///	/* atsBEnergy.0 = INTEGER: -1 tenth kWh */ { "unmapped.atsBEnergy", 0, 1,                           SYSOID ".2.5.1.40.0", NULL, SU_FLAG_OK, NULL },
///	/* atsSwitchLow.0 = INTEGER: -1 V */       { "unmapped.atsSwitchLow", 0, 1,                         SYSOID ".2.5.1.41.0", NULL, SU_FLAG_OK, NULL },
///	/* atsSwitchHigh.0 = INTEGER: -1 V */      { "unmapped.atsSwitchHigh", 0, 1,                        SYSOID ".2.5.1.42.0", NULL, SU_FLAG_OK, NULL },
///	/* atsSwitchTime.0 = INTEGER: -1 min */    { "unmapped.atsSwitchTime", 0, 1,                        SYSOID ".2.5.1.43.0", NULL, SU_FLAG_OK, NULL },
///	/* atsSwitchLock.0 = STRING: -- */         { "unmapped.atsSwitchLock", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.5.1.44.0", NULL, SU_FLAG_OK, NULL },
///	/* atsMonitor.0 = STRING: -- */            { "unmapped.atsMonitor", ST_FLAG_STRING, SU_INFOSIZE,    SYSOID ".2.5.1.45.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.input", ST_FLAG_STRING, SU_INFOSIZE,       SYSOID ".2.5.1.35.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.priority", ST_FLAG_STRING, SU_INFOSIZE,    SYSOID ".2.5.1.36.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.A.voltage", 0, 1,                          SYSOID ".2.5.1.37.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.B.voltage", 0, 1,                          SYSOID ".2.5.1.38.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.A.energy", 0, 0.1,                         SYSOID ".2.5.1.39.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.B.energy", 0, 0.1,                         SYSOID ".2.5.1.40.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.switch.low", 0, 1,                         SYSOID ".2.5.1.41.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.switch.high", 0, 1,                        SYSOID ".2.5.1.42.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.switch.time", 0, 1,                        SYSOID ".2.5.1.43.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.switch.lock", ST_FLAG_STRING, SU_INFOSIZE, SYSOID ".2.5.1.44.0", NULL, SU_FLAG_OK, NULL },
	{ "ats.monitor", ST_FLAG_STRING, SU_INFOSIZE,     SYSOID ".2.5.1.45.0", NULL, SU_FLAG_OK, NULL },

///	/* allOutletsSwitch.0 = INTEGER: none(-1) */ { "unmapped.allOutletsSwitch", 0, 1, SYSOID ".2.5.1.46.0", NULL, SU_FLAG_OK, NULL },
///	/* frequency.0 = INTEGER: 50 Hz */           { "unmapped.frequency", 0, 1, SYSOID ".2.5.1.47.0", NULL, SU_FLAG_OK, NULL },
	{ "outlets.switch", 0, 1,  SYSOID ".2.5.1.46.0", NULL, SU_FLAG_OK, NULL },
	{ "input.frequency", 0, 1, SYSOID ".2.5.1.47.0", NULL, SU_FLAG_OK, NULL },

/* Please revise values discovered by data walk for mappings to
 * docs/nut-names.txt and group the rest under the ifdef below:
 */
#if WITH_UNMAPPED_DATA_POINTS
	/* powerSummaryEntry.54.0 = INTEGER: 16 */ { "unmapped.powerSummaryEntry", 0, 1, SYSOID ".2.5.1.54.0", NULL, SU_FLAG_OK, NULL },
#endif	/* if WITH_UNMAPPED_DATA_POINTS */

	/* end of structure. */
	{ NULL, 0, 0, NULL, NULL, 0, NULL }
};

mib2nut_info_t eaton_pdu_flex = { "eaton_pdu_flex", EATON_PDU_FLEX_MIB_VERSION, NULL, NULL, eaton_pdu_flex_mib, EATON_PDU_FLEX_SYSOID, NULL };




#if 0
/**
///SMART-PDU-MIB.txt

------------------------------------------------------------------------------------------------------------------------
--
-- MIB file for the PDU (Power Distribution Unit) equipments.
-- Product Series: SMART-PDUATS
-- Last Updated: 2023-07-12
--
------------------------------------------------------------------------------------------------------------------------
SMART-PDUATS-MIB DEFINITIONS ::= BEGIN

    IMPORTS
        TruthValue, DisplayString FROM SNMPv2-TC
        MODULE-IDENTITY, OBJECT-TYPE,
        NOTIFICATION-TYPE, enterprises, IpAddress, Integer32 FROM SNMPv2-SMI;

    smart-PDU MODULE-IDENTITY
        LAST-UPDATED "202307120000Z"
        ORGANIZATION "SMART-PDU"
        CONTACT-INFO ""
        DESCRIPTION  "MIB file for the PDU (Power Distribution Unit) equipments."
        ::= { enterprises 55508 }

    pduAts OBJECT IDENTIFIER ::= { smart-PDU 1 }

    -- **********************************************
    --                  system
    -- **********************************************

    system OBJECT IDENTIFIER ::= {pduAts 1}

    eventInfo OBJECT IDENTIFIER ::= {system 1}

    sourceID OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The ID (address code) of sources(PDU/ATS/SENSOR).
                The OID is bound in a trap message."
            ::= { eventInfo 1 }

    powerDescr OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The description to source PDU/ATS.
                The OID is bound in a trap message."
            ::= { eventInfo 2 }

    eventSource OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The event sources: PDU(fuse/load), ATS(switching/voltage), SENSOR(temp./humid./wind/environment).
                The OID is bound in a trap message."
            ::= { eventInfo 3 }

    sensorDescr OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The description to source SENSOR.
                The OID is bound in a trap message."
            ::= { eventInfo 4 }

    eventType OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The type of the event (trap). The value would be 'recovery' or 'alarm' or 'notify'.
                'recovery': An alarm is recovery.
                'alarm': An alarm is issued or repeated.
                'notify': Only a notification, not an alarm or a recovery event.
                NMS can use this OID to judge and how to handle the trap message.
                The OID is bound in a trap message."
            ::= { eventInfo 5 }

    eventDateTime OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Alert time.
                The OID is bound in a trap message."
            ::= { eventInfo 6 }

    loadValue OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The load value of OUTLET/TOTAL.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available.
                The OID is bound in a trap message."
            ::= { eventInfo 7 }

    loadLowValue OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The setting of low limit of load of OUTLET/TOTAL.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available.
                The OID is bound in a trap message."
            ::= { eventInfo 8 }

    loadHighValue OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The setting of high limit of load of OUTLET/TOTAL.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available.
                The OID is bound in a trap message."
            ::= { eventInfo 9 }

    voltageValue OBJECT-TYPE
            SYNTAX      INTEGER(-1..5000)
            UNITS       "V"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The voltage value of input.
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available.
                The OID is bound in a trap message."
            ::= { eventInfo 10 }

    voltageLowValue OBJECT-TYPE
            SYNTAX      INTEGER(-1..5000)
            UNITS       "V"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The setting of low limit of voltage of input.
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available.
                The OID is bound in a trap message."
            ::= { eventInfo 11 }

    voltageHighValue OBJECT-TYPE
            SYNTAX      INTEGER(-1..5000)
            UNITS       "V"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The setting of high limit of voltage of input.
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available.
                The OID is bound in a trap message."
            ::= { eventInfo 12 }
    sysNetwork OBJECT IDENTIFIER ::= {system 2}

    ipAddress OBJECT-TYPE
            SYNTAX      IpAddress
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The IP address of this management node where this SNMP agent is running(Default value: 192.168.0.254).
                NOTATION: When ipAddress is changed, the IP address of the node pointing to this equipment in NMS
                should be changed accordingly.
                NOTATION: When changed, this equipment should be reboot to take effect.
                WARNING: TAKE CARE BEFORE SET!
                The OID is also bound in a trap message."
            ::= { sysNetwork 1}

    maskIpAddress OBJECT-TYPE
            SYNTAX      IpAddress
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The mask IP address,default value: 255.255.255.0
                NOTATION: When changed, this equipment should be reboot to take effect."
            ::= { sysNetwork 2}

    gatewayIpAddress OBJECT-TYPE
            SYNTAX      IpAddress
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The gateway IP address,default value: 0.0.0.0
                NOTATION: When changed, this equipment should be reboot to take effect."
            ::= { sysNetwork 3}

    dnsIpAddress1 OBJECT-TYPE
            SYNTAX      IpAddress
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The DNS#1 IP address,default value: 0.0.0.0
                NOTATION: When changed, this equipment should be reboot to take effect."
            ::= { sysNetwork 4}

    dnsIpAddress2 OBJECT-TYPE
            SYNTAX      IpAddress
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The DNS#2 IP address,default value: 0.0.0.0
                NOTATION: When changed, this equipment should be reboot to take effect."
            ::= { sysNetwork 5}

    rebootSystem OBJECT-TYPE
            SYNTAX      INTEGER(0..1)
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Set value 1 to the OID to reboot the system. set value 0 would not be allowed.
                WARNING: TAKE CARE BEFORE SET!"
            ::= { sysNetwork 6}

    trapDestIP1 OBJECT-TYPE
            SYNTAX      IpAddress
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The trap destination IP address #1,default value: 0.0.0.0"
            ::= { sysNetwork 7}

    trapDestIP2 OBJECT-TYPE
            SYNTAX      IpAddress
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The trap destination IP address #2,default value: 0.0.0.0"
            ::= { sysNetwork 8}

    trapDestIP3 OBJECT-TYPE
            SYNTAX      IpAddress
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The trap destination IP address #3,default value: 0.0.0.0"
            ::= { sysNetwork 9}

    trapDestIP4 OBJECT-TYPE
            SYNTAX      IpAddress
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The trap destination IP address #2,default value: 0.0.0.0"
            ::= { sysNetwork 10}

    powerTables OBJECT IDENTIFIER ::= {pduAts 2}

    -- **********************************************
    --                  power1
    -- **********************************************

    power1OutletTable OBJECT-TYPE
            SYNTAX  SEQUENCE OF Power1OutletEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "A table list the status of all outlets. "
            ::= { powerTables 1 }

    power1OutletEntry OBJECT-TYPE
            SYNTAX  Power1OutletEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "Row definition for the status list."
            INDEX       { p1OutletSeq }
            ::= { power1OutletTable 1 }

    Power1OutletEntry ::= SEQUENCE {
            p1OutletSeq           INTEGER,
            p1OutletNo            DisplayString,
            p1OutletDesc          DisplayString,
            p1OutletSocket        DisplayString,
            p1OutletFuse          DisplayString,
            p1OutletOnOff         INTEGER,
            p1OutletLoad          INTEGER,
            p1OutletLoadLowLimit  INTEGER,
            p1OutletLoadHighLimit INTEGER,
            p1OutletEnergy        INTEGER,
            p1Power               INTEGER,
            p1PowerFactor         INTEGER}

    p1OutletSeq OBJECT-TYPE
            SYNTAX  INTEGER(0..47)
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sequence of table."
            ::= { power1OutletEntry 1 }

    p1OutletNo OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's No."
            ::= { power1OutletEntry 2 }

    p1OutletDesc OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..30))
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Outlet's description."
            ::= { power1OutletEntry 3 }

    p1OutletSocket OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's socket standard description."
            ::= { power1OutletEntry 4 }

    p1OutletFuse OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's fuse status. Normal or Alarm."
            ::= { power1OutletEntry 5 }

    p1OutletOnOff OBJECT-TYPE
            SYNTAX      INTEGER{none(-1),off(0),on(1)}
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Outlet's ON/OFF status."
            ::= { power1OutletEntry 6 }

    p1OutletLoad OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's load amperes.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { power1OutletEntry 7 }

    p1OutletLoadLowLimit OBJECT-TYPE
            SYNTAX  INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Outlet's load low-limit in amperes.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { power1OutletEntry 8 }

    p1OutletLoadHighLimit OBJECT-TYPE
            SYNTAX  INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Outlet's load high-limit in amperes.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { power1OutletEntry 9 }

    p1OutletEnergy OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth kWh"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's energy (unit in 0.01kWh).
                A non-negative value indicates the measured energy hundredth of kWh.
                A negative value indicates that the energy value was not available."
            ::= { power1OutletEntry 10 }

    p1Power OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's active power (unit in 0.01W).
                A non-negative value indicates the measured active power hundredth of W.
                A negative value indicates that the active power value was not available."
            ::= { power1OutletEntry 11 }

    p1PowerFactor OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's power factor (unit in 0.01).
                A non-negative value indicates the measured power factor hundredth.
                A negative value indicates that the power factor value was not available."
            ::= { power1OutletEntry 12 }

    -- **********************************************
    --                  power2
    -- **********************************************

    power2OutletTable OBJECT-TYPE
            SYNTAX  SEQUENCE OF Power2OutletEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "A table list the status of all outlets. "
            ::= { powerTables 2 }

    power2OutletEntry OBJECT-TYPE
            SYNTAX  Power2OutletEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "Row definition for the status list."
            INDEX       { p2OutletSeq }
            ::= { power2OutletTable 1 }

    Power2OutletEntry ::= SEQUENCE {
            p2OutletSeq           INTEGER,
            p2OutletNo            DisplayString,
            p2OutletDesc          DisplayString,
            p2OutletSocket        DisplayString,
            p2OutletFuse          DisplayString,
            p2OutletOnOff         INTEGER,
            p2OutletLoad          INTEGER,
            p2OutletLoadLowLimit  INTEGER,
            p2OutletLoadHighLimit INTEGER,
            p2OutletEnergy        INTEGER,
            p2Power               INTEGER,
            p2PowerFactor         INTEGER}

    p2OutletSeq OBJECT-TYPE
            SYNTAX  INTEGER(0..47)
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sequence of table."
            ::= { power2OutletEntry 1 }

    p2OutletNo OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's No."
            ::= { power2OutletEntry 2 }

    p2OutletDesc OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..30))
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Outlet's description."
            ::= { power2OutletEntry 3 }

    p2OutletSocket OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's socket standard description."
            ::= { power2OutletEntry 4 }

    p2OutletFuse OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's fuse status. Normal or Alarm."
            ::= { power2OutletEntry 5 }

    p2OutletOnOff OBJECT-TYPE
            SYNTAX      INTEGER{none(-1),off(0),on(1)}
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Outlet's ON/OFF status."
            ::= { power2OutletEntry 6 }

    p2OutletLoad OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's load amperes.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { power2OutletEntry 7 }

    p2OutletLoadLowLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Outlet's load low-limit in amperes.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { power2OutletEntry 8 }

    p2OutletLoadHighLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Outlet's load high-limit in amperes.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { power2OutletEntry 9 }

    p2OutletEnergy OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth kWh"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's energy (unit in 0.01kWh).
                A non-negative value indicates the measured energy hundredth of kWh.
                A negative value indicates that the energy value was not available."
            ::= { power2OutletEntry 10 }

    p2Power OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's active power (unit in 0.01W).
                A non-negative value indicates the measured active power hundredth of W.
                A negative value indicates that the active power value was not available."
            ::= { power2OutletEntry 11 }

    p2PowerFactor OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's power factor (unit in 0.01).
                A non-negative value indicates the measured power factor hundredth.
                A negative value indicates that the power factor value was not available."
            ::= { power2OutletEntry 12 }

    -- **********************************************
    --                  power3
    -- **********************************************

    power3OutletTable OBJECT-TYPE
            SYNTAX  SEQUENCE OF Power3OutletEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "A table list the status of all outlets. "
            ::= { powerTables 3 }

    power3OutletEntry OBJECT-TYPE
            SYNTAX  Power3OutletEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "Row definition for the status list."
            INDEX       { p3OutletSeq }
            ::= { power3OutletTable 1 }

    Power3OutletEntry ::= SEQUENCE {
            p3OutletSeq           INTEGER,
            p3OutletNo            DisplayString,
            p3OutletDesc          DisplayString,
            p3OutletSocket        DisplayString,
            p3OutletFuse          DisplayString,
            p3OutletOnOff         INTEGER,
            p3OutletLoad          INTEGER,
            p3OutletLoadLowLimit  INTEGER,
            p3OutletLoadHighLimit INTEGER,
            p3OutletEnergy        INTEGER,
            p3Power               INTEGER,
            p3PowerFactor         INTEGER}

    p3OutletSeq OBJECT-TYPE
            SYNTAX  INTEGER(0..47)
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sequence of table."
            ::= { power3OutletEntry 1 }

    p3OutletNo OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's No."
            ::= { power3OutletEntry 2 }

    p3OutletDesc OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..30))
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Outlet's description."
            ::= { power3OutletEntry 3 }

    p3OutletSocket OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's socket standard description."
            ::= { power3OutletEntry 4 }

    p3OutletFuse OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's fuse status. Normal or Alarm."
            ::= { power3OutletEntry 5 }

    p3OutletOnOff OBJECT-TYPE
            SYNTAX      INTEGER{none(-1),off(0),on(1)}
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Outlet's ON/OFF status."
            ::= { power3OutletEntry 6 }

    p3OutletLoad OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's load amperes.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { power3OutletEntry 7 }

    p3OutletLoadLowLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Outlet's load low-limit in amperes.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { power3OutletEntry 8 }

    p3OutletLoadHighLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Outlet's load high-limit in amperes.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { power3OutletEntry 9 }

    p3OutletEnergy OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth kWh"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's energy (unit in 0.01kWh).
                A non-negative value indicates the measured energy hundredth of kWh.
                A negative value indicates that the energy value was not available."
            ::= { power3OutletEntry 10 }

    p3Power OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's active power (unit in 0.01W).
                A non-negative value indicates the measured active power hundredth of W.
                A negative value indicates that the active power value was not available."
            ::= { power3OutletEntry 11 }

    p3PowerFactor OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Outlet's power factor (unit in 0.01).
                A non-negative value indicates the measured power factor hundredth.
                A negative value indicates that the power factor value was not available."
            ::= { power3OutletEntry 12 }

    -- **********************************************
    --                  power Branch
    -- **********************************************

    powerBranchTable OBJECT-TYPE
            SYNTAX  SEQUENCE OF powerBranchEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "A table list the status of all branchs. "
            ::= { powerTables 4 }

    powerBranchEntry OBJECT-TYPE
            SYNTAX  powerBranchEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "Row definition for the status list."
            INDEX       { branchSeq }
            ::= { powerBranchTable 1 }

    powerBranchEntry ::= SEQUENCE {
            branchSeq             INTEGER,
            branchRate            INTEGER,
            branchVoltage         INTEGER,
            branchEnergy          INTEGER,
            branchPower           INTEGER,
            branchPowerFactor     INTEGER,
            branchLoad            INTEGER,
            branchLoadHighLimit   INTEGER}

    branchSeq OBJECT-TYPE
            SYNTAX  INTEGER(0..7)
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sequence of table."
            ::= { powerBranchEntry 1 }

    branchRate OBJECT-TYPE
            SYNTAX      INTEGER(-1..6400)
            UNITS       "hundredth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "Branch rated current (unit in 0.01A).
                A non-negative value indicates the measured load in hundredth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerBranchEntry 2 }

   branchVoltage OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "tenth V"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The voltage of branch (unit in 0.1V).
                A non-negative value indicates the measured voltage in tenth of volts.
                A negative value indicates that a voltage value was not available."
            ::= { powerBranchEntry 3 }

    branchEnergy OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "tenth kWh"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The energy of branch (unit in 0.1kWh).
                A non-negative value indicates the measured energy tenth of kWh.
                A negative value indicates that the energy value was not available."
            ::= { powerBranchEntry 4 }

    branchPower OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The active power of branch (unit in W).
                A non-negative value indicates the measured active power.
                A negative value indicates that the active power value was not available."
            ::= { powerBranchEntry 5 }

    branchPowerFactor OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The power factor of branch (unit in 0.01).
                A non-negative value indicates the measured power factor hundredth.
                A negative value indicates that the power factor value was not available."
            ::= { powerBranchEntry 6 }

    branchLoad OBJECT-TYPE
            SYNTAX      INTEGER
            UNITS       "hundredth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The load of branch (unit in 0.01A).
                A non-negative value indicates the measured load in hundredth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerBranchEntry 7 }

    branchLoadHighLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..6400)
            UNITS       "hundredth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "Branch's load high-limit in amperes.
                A non-negative value indicates the measured load in hundredth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerBranchEntry 8 }

    -- **********************************************
    --                  power summary
    -- **********************************************

    powerSummaryTable OBJECT-TYPE
            SYNTAX  SEQUENCE OF PowerSummaryEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "A table list the summary status information for PDU."
            ::= { powerTables 5 }

    powerSummaryEntry OBJECT-TYPE
            SYNTAX  PowerSummaryEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "Row definition for the status list."
            INDEX       { powerSeq }
            ::= { powerSummaryTable 1 }

    PowerSummaryEntry ::= SEQUENCE {
            powerSeq            INTEGER,
            powerID             DisplayString,
            powerName           DisplayString,
            powerType           DisplayString,
            powerModel          DisplayString,
            powerOnGap          INTEGER,
            totalLoadA          INTEGER,
            totalLoadALowLimit  INTEGER,
            totalLoadAHighLimit INTEGER,
            totalLoadB          INTEGER,
            totalLoadBLowLimit  INTEGER,
            totalLoadBHighLimit INTEGER,
            totalLoadC          INTEGER,
            totalLoadCLowLimit  INTEGER,
            totalLoadCHighLimit INTEGER,
            voltageA            INTEGER,
            voltageB            INTEGER,
            voltageC            INTEGER,
            powerEnergy         INTEGER,
            activePowerA        INTEGER,
            ratedPowerA         INTEGER,
            remainPowerA        INTEGER,
            apparentPowerA      INTEGER,
            powerFactorA        INTEGER,
            activePowerB        INTEGER,
            ratedPowerB         INTEGER,
            remainPowerB        INTEGER,
            apparentPowerB      INTEGER,
            powerFactorB        INTEGER,
            activePowerC        INTEGER,
            ratedPowerC         INTEGER,
            remainPowerC        INTEGER,
            apparentPowerC      INTEGER,
            powerFactorC        INTEGER,
            atsInput            DisplayString,
            atsPriority         DisplayString,
            atsAVoltage         INTEGER,
            atsBVoltage         INTEGER,
            atsAEnergy          INTEGER,
            atsBEnergy          INTEGER,
            atsSwitchLow        INTEGER,
            atsSwitchHigh       INTEGER,
            atsSwitchTime       INTEGER,
            atsSwitchLock       DisplayString,
            atsMonitor          DisplayString,
            allOutletsSwitch    INTEGER,
            frequency           INTEGER,
            voltageALowLimit    INTEGER,
            voltageAHighLimit   INTEGER,
            voltageBLowLimit    INTEGER,
            voltageBHighLimit   INTEGER,
            voltageCLowLimit    INTEGER,
            voltageCHighLimit   INTEGER}

    powerSeq OBJECT-TYPE
            SYNTAX  INTEGER(0..0)
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sequence of table."
            ::= { powerSummaryEntry 1 }

    powerID OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Outlet's No."
            ::= { powerSummaryEntry 2 }

    powerName OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..30))
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The PDU description."
            ::= { powerSummaryEntry 3 }

    powerType OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The PDU type."
            ::= { powerSummaryEntry 4 }

    powerModel OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The PDU type."
            ::= { powerSummaryEntry 5 }

    powerOnGap OBJECT-TYPE
            SYNTAX  INTEGER(0..255)
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "The interval of outlets' of the PDU when power on."
            ::= { powerSummaryEntry 6 }

    totalLoadA OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The A total load of the PDU when it is Three-phase input,
                or the total load of the PDU when it is Single-phase input.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerSummaryEntry 7 }

    totalLoadALowLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The A total load low limit of the PDU when it is Three-phase input,
                or the total load of the PDU when it is Single-phase input.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerSummaryEntry 8 }

    totalLoadAHighLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The A total load high limit of the PDU when it is Three-phase input,
                or the total load of the PDU when it is Single-phase input.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerSummaryEntry 9 }

    totalLoadB OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The B total load of the PDU when it is Three-phase input.
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerSummaryEntry 10 }

    totalLoadBLowLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The B total load low limit of the PDU when it is Three-phase input,
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerSummaryEntry 11 }

    totalLoadBHighLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The B total load high limit of the PDU when it is Three-phase input,
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerSummaryEntry 12 }

    totalLoadC OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The C total load of the PDU when it is Three-phase input,
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerSummaryEntry 13 }

    totalLoadCLowLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The C total load low limit of the PDU when it is Three-phase input,
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerSummaryEntry 14 }

    totalLoadCHighLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..640)
            UNITS       "tenth A"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The C total load high limit of the PDU when it is Three-phase input,
                A non-negative value indicates the measured load in tenth of amperes.
                A negative value indicates that a load value was not available."
            ::= { powerSummaryEntry 15 }

    voltageA OBJECT-TYPE
            SYNTAX      INTEGER(-1..500)
            UNITS       "V"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The A voltage of the PDU when it is Three-phase input,
                or the voltage of the PDU when it is Single-phase input,
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 16 }

    voltageB OBJECT-TYPE
            SYNTAX      INTEGER(-1..500)
            UNITS       "V"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The B voltage of the PDU when it is Three-phase input,
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 17 }

    voltageC OBJECT-TYPE
            SYNTAX      INTEGER(-1..500)
            UNITS       "V"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The C voltage of the PDU when it is Three-phase input,
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 18 }

    powerEnergy OBJECT-TYPE
            SYNTAX      INTEGER(-1..2147483647)
            UNITS       "tenth kWh"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The ACC. energy of the PDU,
                A non-negative value indicates the measured value in tenth of kWh.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 19 }

    activePowerA OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The active power of the input power A.
                A non-negative value indicates the measured value in tenth of watts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 20 }

    ratedPowerA OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The rated power of the input power B.
                A non-negative value indicates the measured value in tenth of watts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 21 }

    remainPowerA OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The remain power of the input power A.
                A non-negative value indicates the measured value in tenth of watts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 22 }

    apparentPowerA OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The apparent power of the input power A.
                A non-negative value indicates the measured value in tenth of W.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 23 }

    powerFactorA OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "hundredth"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The power factor of the input power A.
                A non-negative value indicates the power factor in hundredth.
                A negative value indicates that the power factor was not able to be measured."
            ::= { powerSummaryEntry 24 }

    activePowerB OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The active power of the input power B.
                A non-negative value indicates the measured value in tenth of watts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 25 }

    ratedPowerB OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The rated power of the input power B.
                A non-negative value indicates the measured value in tenth of watts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 26 }

    remainPowerB OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The remain power of the input power B.
                A non-negative value indicates the measured value in tenth of watts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 27 }

    apparentPowerB OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The apparent power of the input power B.
                A non-negative value indicates the measured value in tenth of W.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 28 }

    powerFactorB OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "hundredth"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The power factor of the input power B.
                A non-negative value indicates the power factor in hundredth.
                A negative value indicates that the power factor was not able to be measured."
            ::= { powerSummaryEntry 29 }

    activePowerC OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The active power of the input power C.
                A non-negative value indicates the measured value in tenth of watts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 30 }

    ratedPowerC OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The rated power of the input power C.
                A non-negative value indicates the measured value in tenth of watts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 31 }

    remainPowerC OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The remain power of the input powerC.
                A non-negative value indicates the measured value in tenth of watts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 32 }

    apparentPowerC OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "tenth W"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The apparent power of the input power C.
                A non-negative value indicates the measured value in tenth of W.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 33 }

    powerFactorC OBJECT-TYPE
            SYNTAX      INTEGER(-1..14417700)
            UNITS       "hundredth"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The power factor of the input power C.
                A non-negative value indicates the power factor in hundredth.
                A negative value indicates that the power factor was not able to be measured."
            ::= { powerSummaryEntry 34 }

    atsInput OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The power input of ATS, which would be A or B."
            ::= { powerSummaryEntry 35 }

    atsPriority OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..10))
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The priority input of ATS."
            ::= { powerSummaryEntry 36 }

    atsAVoltage OBJECT-TYPE
            SYNTAX      INTEGER(-1..500)
            UNITS       "V"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The A voltage of ATS,
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 37 }

    atsBVoltage OBJECT-TYPE
            SYNTAX      INTEGER(-1..500)
            UNITS       "V"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The B voltage of ATS,
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 38 }

    atsAEnergy OBJECT-TYPE
            SYNTAX      INTEGER(-1..2147483647)
            UNITS       "tenth kWh"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The A energy of ATS,
                A non-negative value indicates the measured value in tenth of kWh.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 39 }

    atsBEnergy OBJECT-TYPE
            SYNTAX      INTEGER(-1..2147483647)
            UNITS       "tenth kWh"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The B energy of ATS,
                A non-negative value indicates the measured value in tenth of kWh.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 40 }

    atsSwitchLow OBJECT-TYPE
            SYNTAX      INTEGER(-1..500)
            UNITS       "V"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The switch voltage low-limit of ATS,
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 41 }

    atsSwitchHigh OBJECT-TYPE
            SYNTAX      INTEGER(-1..500)
            UNITS       "V"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The switch voltage high-limit of ATS,
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 42 }

    atsSwitchTime OBJECT-TYPE
            SYNTAX      INTEGER(-1..255)
            UNITS       "min"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The switch time of ATS.
                A non-negative value indicates the measured load in minutes.
                A negative value indicates that a time value was not available."
            ::= { powerSummaryEntry 43 }

    atsSwitchLock OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..10))
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The switch lock status of ATS."
            ::= { powerSummaryEntry 44 }

    atsMonitor OBJECT-TYPE
            SYNTAX      DisplayString(SIZE(0..10))
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The stability monitor status of ATS."
            ::= { powerSummaryEntry 45 }

    allOutletsSwitch OBJECT-TYPE
            SYNTAX      INTEGER{none(-1),off(0),on(1)}
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "To set the switches of all outlets belong to this Power(PDU).
                A negative value none(-1) would be returned for GET command. Nothing to do when SET with the value none(-1)."
            ::= { powerSummaryEntry 46 }

    frequency OBJECT-TYPE
            SYNTAX      INTEGER(-1..65)
            UNITS       "Hz"
            MAX-ACCESS  read-only
            STATUS      current
            DESCRIPTION
                "The frequency value of the input.
                A non-negative value indicates the measured value in hertz."
            ::= { powerSummaryEntry 47 }

    voltageALowLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..5000)
            UNITS       "V"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The A voltage low limit of the PDU when it is Three-phase input,
				or the voltage low limit of the PDU when it is Single-phase input.
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 48 }

    voltageAHighLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..5000)
            UNITS       "V"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The A voltage high limit of the PDU when it is Three-phase input,
				or the voltage high limit of the PDU when it is Single-phase input.
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 49 }

    voltageBLowLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..5000)
            UNITS       "V"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The B voltage low limit of the PDU when it is Three-phase input.
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 50 }

    voltageBHighLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..5000)
            UNITS       "V"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The B voltage high limit of the PDU when it is Three-phase input.
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 51 }

    voltageCLowLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..5000)
            UNITS       "V"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The C voltage low limit of the PDU when it is Three-phase input.
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 52 }

    voltageCHighLimit OBJECT-TYPE
            SYNTAX      INTEGER(-1..5000)
            UNITS       "V"
            MAX-ACCESS  read-write
            STATUS      current
            DESCRIPTION
                "The C voltage high limit of the PDU when it is Three-phase input.
                A non-negative value indicates the measured value in volts.
                A negative value indicates that a value was not available."
            ::= { powerSummaryEntry 53 }
    -- **********************************************
    --                  sensor tables
    -- **********************************************

    environmentTable OBJECT IDENTIFIER ::= {pduAts 3}

    sensorTable OBJECT-TYPE
            SYNTAX  SEQUENCE OF SensorEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "A table list the status of all external sensors. "
            ::= { environmentTable 1 }

    sensorEntry OBJECT-TYPE
            SYNTAX  SensorEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "Row definition for the status list."
            INDEX       { sensorSeq }
            ::= { sensorTable 1 }

    SensorEntry ::= SEQUENCE {
            sensorSeq            INTEGER,
            sensorNo             DisplayString,
            sensorType           DisplayString,
            sensorID             DisplayString,
            sensorLocation       DisplayString,
            sensorValue          DisplayString,
            temperatureValue     INTEGER,
            temperatureLowLimit  INTEGER,
            temperatureHighLimit INTEGER,
            humidityValue        INTEGER,
            humidityLowLimit     INTEGER,
            humidityHighLimit    INTEGER,
            windValue            INTEGER,
            windLowLimit         INTEGER,
            windHighLimit        INTEGER}

    sensorSeq OBJECT-TYPE
            SYNTAX  INTEGER(0..7)
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sequence of table."
            ::= { sensorEntry 1 }

    sensorNo OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sensor's No."
            ::= { sensorEntry 2 }

    sensorType OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sensor's Type."
            ::= { sensorEntry 3 }

    sensorID OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sensor's ID."
            ::= { sensorEntry 4 }

    sensorLocation OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..30))
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Sensor's location."
            ::= { sensorEntry 5 }

    sensorValue OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..30))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sensor's value of smog/door/water."
            ::= { sensorEntry 6 }

    temperatureValue OBJECT-TYPE
            SYNTAX  INTEGER(-40..120)
            UNITS       "Deg.C or Deg.F"
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Temperature Value.
                NOTE: the UNIT is Deg.C or Deg.F which is select from the setting WEB page."
            ::= { sensorEntry 7 }

    temperatureLowLimit OBJECT-TYPE
            SYNTAX  INTEGER(-40..120)
            UNITS       "Deg.C or Deg.F"
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Temperature low limit.
                NOTE: the UNIT is Deg.C or Deg.F which is select from the setting WEB page."
            ::= { sensorEntry 8 }

    temperatureHighLimit OBJECT-TYPE
            SYNTAX  INTEGER(-40..120)
            UNITS       "Deg.C or Deg.F"
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Temperature high limit.
                NOTE: the UNIT is Deg.C or Deg.F which is select from the setting WEB page."
            ::= { sensorEntry 9 }

    humidityValue OBJECT-TYPE
            SYNTAX  INTEGER(-1..100)
            UNITS       "percentage relative humidity"
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "The humidity measured by the sensor.  A non-negative value
                indicates the measured humidity in percentage relative
                humidity.  A negative value indicates that a humidity value
                was not available."
            ::= { sensorEntry 10 }

    humidityLowLimit OBJECT-TYPE
            SYNTAX  INTEGER(0..100)
            UNITS       "percentage relative humidity"
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Humidity low limit."
            ::= { sensorEntry 11 }

    humidityHighLimit OBJECT-TYPE
            SYNTAX  INTEGER(0..100)
            UNITS       "percentage relative humidity"
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Humidity high limit."
            ::= { sensorEntry 12 }

    windValue OBJECT-TYPE
            SYNTAX  INTEGER(0..836)
            UNITS       "tenth of ft/s or m/s"
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "The wind speed is measured by the sensor.  A non-negative value
                indicates the measured wind speed. A negative value indicates that
                a wind speed value was not available.
                NOTE: When the unit is selected as m/s, a value of 123 means 12.3m/s. If the value is 255,
                means the wind speed is equal to or great than 25.5m/s.
                NOTE: when unit is ft/s: range would be (0..836); when unit is m/s: range would be (0..255).
                NOTE: the UNIT is tenth of ft/s or m/s which is select from the setting WEB page."
            ::= { sensorEntry 13 }

    windLowLimit OBJECT-TYPE
            SYNTAX  INTEGER(0..836)
            UNITS       "tenth of ft/s or m/s"
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Wind speed low limit. see windValue for more detail."
            ::= { sensorEntry 14 }

    windHighLimit OBJECT-TYPE
            SYNTAX  INTEGER(0..836)
            UNITS       "tenth of ft/s or m/s"
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Wind speed high limit. see windValue for more detail."
            ::= { sensorEntry 15 }

    -- **********************************************
    --                  Internal Sensor
    -- **********************************************

    internalSensorTable OBJECT-TYPE
            SYNTAX  SEQUENCE OF InternalSensorEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "A table list the status of internal sensors. "
            ::= { environmentTable 2 }

    internalSensorEntry OBJECT-TYPE
            SYNTAX  InternalSensorEntry
            MAX-ACCESS  not-accessible
            STATUS  current
            DESCRIPTION
                "Row definition for the status list."
            INDEX       { internalSensorSeq }
            ::= { internalSensorTable 1 }

    InternalSensorEntry ::= SEQUENCE {
            internalSensorSeq            INTEGER,
			fanStatus                    DisplayString,
			temperatureValue             INTEGER,
			temperatureHighLimit         INTEGER,
			temperatureStatus            DisplayString}

    internalSensorSeq OBJECT-TYPE
            SYNTAX  INTEGER(0..7)
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Sequence of table."
            ::= { internalSensorEntry 1 }

    fanStatus OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Fan's status. Normal or Alarm."
            ::= { internalSensorEntry 2 }

    temperatureValue OBJECT-TYPE
            SYNTAX  INTEGER(-40..120)
            UNITS       "Deg.C"
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Internal temperature Value. The UNIT is Deg.C."
            ::= { internalSensorEntry 3 }

    temperatureHighLimit OBJECT-TYPE
            SYNTAX  INTEGER(-40..120)
            UNITS       "Deg.C"
            MAX-ACCESS  read-write
            STATUS  current
            DESCRIPTION
                "Internal temperature high limit. The UNIT is Deg.C."
            ::= { internalSensorEntry 4 }

    temperatureStatus OBJECT-TYPE
            SYNTAX  DisplayString(SIZE(0..10))
            MAX-ACCESS  read-only
            STATUS  current
            DESCRIPTION
                "Temperature's status. Normal or Alarm."
            ::= { internalSensorEntry 5 }

    -- **********************************************
    --                  Notifications
    -- **********************************************

    netPowerTraps OBJECT IDENTIFIER ::= { pduAts 100 }

    netPowerPduAtsEvents OBJECT IDENTIFIER ::= { netPowerTraps 1 }

    fuseEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                eventSource,
                powerDescr,
                sourceID,
                eventType}
            STATUS      current
            DESCRIPTION
                "Fuse event for the PDU outlets. The event would occurred when the fuse is burned/repaired.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 1 }

    loadEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                eventSource,
                powerDescr,
                sourceID,
                eventType,
                loadValue,
                loadLowValue,
                loadHighValue}
            STATUS      current
            DESCRIPTION
                "Load event for the PDU outlets and total power. The event would occurred when load is
                lower/higher than the limit setting or recovery to normal.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 2 }

    temperatureSensorEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                sensorDescr,
                powerDescr,
                sourceID,
                eventType,
                temperatureValue,
                temperatureLowLimit,
                temperatureHighLimit}
            STATUS      current
            DESCRIPTION
                "Temperature sensor event. The event would occurred when temperature value is
                lower/higher than the limit setting or recovery to normal.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 3 }

    humiditySensorEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                sensorDescr,
                powerDescr,
                sourceID,
                eventType,
                humidityValue,
                humidityLowLimit,
                humidityHighLimit}
            STATUS      current
            DESCRIPTION
                "Humidity sensor event.The event would occurred when humidity value is
                lower/higher than the limit setting or recovery to normal.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 4 }

    windSensorEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                sensorDescr,
                powerDescr,
                sourceID,
                eventType,
                windValue,
                windLowLimit,
                windHighLimit}
            STATUS      current
            DESCRIPTION
                "Wind sensor event.The event would occurred when wind speed value is
                lower/higher than the limit setting or recovery to normal.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 5 }

    environmentSensorEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                sensorDescr,
                powerDescr,
                sourceID,
                eventType}
            STATUS      current
            DESCRIPTION
                "Environment sensor event, including smog, water, door and so on. The
                event would occurred when the sensor detects an alarm or recovery.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 6 }

    atsSwitchingEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                eventSource,
                powerDescr,
                sourceID,
                eventType,
                atsAVoltage,
                atsBVoltage}
            STATUS      current
            DESCRIPTION
                "ATS switch event. The event would occurred when ATS switched the input
                power A to B, or B to A.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 7 }

    atsVoltageEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                eventSource,
                powerDescr,
                sourceID,
                eventType,
                atsSwitchLow,
                atsSwitchHigh}
            STATUS      current
            DESCRIPTION
                "ATS voltage event. The event would occurred when voltage is
                lower/higher than the limit setting or recovery to normal.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 8 }

    snmpQueryTimeoutEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                eventSource,
                powerDescr,
                sourceID,
                eventType}
            STATUS      current
            DESCRIPTION
                "Snmp Query Timeout event for the PDU snmp connection. The event would occurred when no any query until timeout is come.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 9 }

    voltageEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                eventSource,
                powerDescr,
                sourceID,
                eventType,
                voltageValue,
                voltageLowValue,
                voltageHighValue}
            STATUS      current
            DESCRIPTION
                "Voltage event for the PDU input. The event would occurred when voltage is
                lower/higher than the limit setting or recovery to normal.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 10 }

    connectEvent NOTIFICATION-TYPE
            OBJECTS {
                ipAddress,
                eventDateTime,
                eventSource,
                powerDescr,
                sourceID,
                eventType}
            STATUS      current
            DESCRIPTION
                "Connect event for the PDU failure. The event would occurred when the power module
				or outlets module is disconnect/repaired.
                NMS can check the bound OID 'eventType' for handling the trap."
            ::= { netPowerPduAtsEvents 11 }
END
------------------------------------------------------------------------------------------------------------------------

*/
#endif
