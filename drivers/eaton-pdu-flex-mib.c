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

	{ "device.name", ST_FLAG_STRING, SU_INFOSIZE,     ".1.3.6.1.4.1.55508.1.2.5.1.3.0", NULL, SU_FLAG_OK, NULL },
	{ "device.model", ST_FLAG_STRING, SU_INFOSIZE,    ".1.3.6.1.4.1.55508.1.2.5.1.5.0", NULL, SU_FLAG_OK, NULL },

/* Please revise values discovered by data walk for mappings to
 * docs/nut-names.txt and group the rest under the ifdef below:
 */
#if WITH_UNMAPPED_DATA_POINTS
	/* sourceID.0 = STRING:  */                { "unmapped.sourceID", ST_FLAG_STRING, SU_INFOSIZE,       ".1.3.6.1.4.1.55508.1.1.1.1.0", NULL, SU_FLAG_OK, NULL },
	/* powerDescr.0 = STRING:  */              { "unmapped.powerDescr", ST_FLAG_STRING, SU_INFOSIZE,     ".1.3.6.1.4.1.55508.1.1.1.2.0", NULL, SU_FLAG_OK, NULL },
	/* eventSource.0 = STRING: */              { "unmapped.eventSource", ST_FLAG_STRING, SU_INFOSIZE,    ".1.3.6.1.4.1.55508.1.1.1.3.0", NULL, SU_FLAG_OK, NULL },
	/* sensorDescr.0 = STRING: */              { "unmapped.sensorDescr", ST_FLAG_STRING, SU_INFOSIZE,    ".1.3.6.1.4.1.55508.1.1.1.4.0", NULL, SU_FLAG_OK, NULL },
	/* eventType.0 = STRING:  */               { "unmapped.eventType", ST_FLAG_STRING, SU_INFOSIZE,      ".1.3.6.1.4.1.55508.1.1.1.5.0", NULL, SU_FLAG_OK, NULL },
	/* eventDateTime.0 = STRING: */            { "unmapped.eventDateTime", ST_FLAG_STRING, SU_INFOSIZE,  ".1.3.6.1.4.1.55508.1.1.1.6.0", NULL, SU_FLAG_OK, NULL },
	/* loadValue.0 = INTEGER: 0 tenth A */     { "unmapped.loadValue", 0, 1,                             ".1.3.6.1.4.1.55508.1.1.1.7.0", NULL, SU_FLAG_OK, NULL },
	/* loadLowValue.0 = INTEGER: 0 tenth A */  { "unmapped.loadLowValue", 0, 1,                          ".1.3.6.1.4.1.55508.1.1.1.8.0", NULL, SU_FLAG_OK, NULL },
	/* loadHighValue.0 = INTEGER: 0 tenth A */ { "unmapped.loadHighValue", 0, 1,                         ".1.3.6.1.4.1.55508.1.1.1.9.0", NULL, SU_FLAG_OK, NULL },
	/* voltageValue.0 = INTEGER: 0 V */        { "unmapped.voltageValue", 0, 1,                          ".1.3.6.1.4.1.55508.1.1.1.10.0", NULL, SU_FLAG_OK, NULL },
	/* voltageLowValue.0 = INTEGER: 0 V */     { "unmapped.voltageLowValue", 0, 1,                       ".1.3.6.1.4.1.55508.1.1.1.11.0", NULL, SU_FLAG_OK, NULL },
	/* voltageHighValue.0 = INTEGER: 0 V */    { "unmapped.voltageHighValue", 0, 1,                      ".1.3.6.1.4.1.55508.1.1.1.12.0", NULL, SU_FLAG_OK, NULL },

	/* ipAddress.0 = IpAddress: 10.130.245.91 */      { "unmapped.ipAddress", 0, 1,        ".1.3.6.1.4.1.55508.1.1.2.1.0", NULL, SU_FLAG_OK, NULL },
	/* maskIpAddress.0 = IpAddress: 255.255.255.0 */  { "unmapped.maskIpAddress", 0, 1,    ".1.3.6.1.4.1.55508.1.1.2.2.0", NULL, SU_FLAG_OK, NULL },
	/* gatewayIpAddress.0 = IpAddress: 192.168.0.1 */ { "unmapped.gatewayIpAddress", 0, 1, ".1.3.6.1.4.1.55508.1.1.2.3.0", NULL, SU_FLAG_OK, NULL },
	/* dnsIpAddress1.0 = IpAddress: 0.0.0.0 */        { "unmapped.dnsIpAddress1", 0, 1,    ".1.3.6.1.4.1.55508.1.1.2.4.0", NULL, SU_FLAG_OK, NULL },
	/* dnsIpAddress2.0 = IpAddress: 0.0.0.0 */        { "unmapped.dnsIpAddress2", 0, 1,    ".1.3.6.1.4.1.55508.1.1.2.5.0", NULL, SU_FLAG_OK, NULL },
	/* rebootSystem.0 = INTEGER: 0 */                 { "unmapped.rebootSystem", 0, 1,     ".1.3.6.1.4.1.55508.1.1.2.6.0", NULL, SU_FLAG_OK, NULL },
	/* trapDestIP1.0 = IpAddress: 0.0.0.0 */          { "unmapped.trapDestIP1", 0, 1,      ".1.3.6.1.4.1.55508.1.1.2.7.0", NULL, SU_FLAG_OK, NULL },
	/* trapDestIP2.0 = IpAddress: 0.0.0.0 */          { "unmapped.trapDestIP2", 0, 1,      ".1.3.6.1.4.1.55508.1.1.2.8.0", NULL, SU_FLAG_OK, NULL },
	/* trapDestIP3.0 = IpAddress: 0.0.0.0 */          { "unmapped.trapDestIP3", 0, 1,      ".1.3.6.1.4.1.55508.1.1.2.9.0", NULL, SU_FLAG_OK, NULL },
	/* trapDestIP4.0 = IpAddress: 0.0.0.0 */          { "unmapped.trapDestIP4", 0, 1,      ".1.3.6.1.4.1.55508.1.1.2.10.0", NULL, SU_FLAG_OK, NULL },

	/* p1OutletSeq.0 = INTEGER: 0 */ { "unmapped.p1OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.1.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSeq.1 = INTEGER: 1 */ { "unmapped.p1OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.1.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSeq.2 = INTEGER: 2 */ { "unmapped.p1OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.1.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSeq.3 = INTEGER: 3 */ { "unmapped.p1OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.1.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSeq.4 = INTEGER: 4 */ { "unmapped.p1OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.1.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSeq.5 = INTEGER: 5 */ { "unmapped.p1OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.1.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSeq.6 = INTEGER: 6 */ { "unmapped.p1OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.1.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSeq.7 = INTEGER: 7 */ { "unmapped.p1OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.1.7", NULL, SU_FLAG_OK, NULL },

	/* p1OutletNo.0 = STRING: 1 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.2.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletNo.1 = STRING: 2 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.2.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletNo.2 = STRING: 3 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.2.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletNo.3 = STRING: 4 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.2.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletNo.4 = STRING: 5 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.2.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletNo.5 = STRING: 6 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.2.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletNo.6 = STRING: 7 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.2.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletNo.7 = STRING: 8 */ { "unmapped.p1OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.2.7", NULL, SU_FLAG_OK, NULL },

	/* p1OutletDesc.0 = STRING: Outlet_1 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.3.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletDesc.1 = STRING: Outlet_2 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.3.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletDesc.2 = STRING: Outlet_3 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.3.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletDesc.3 = STRING: Outlet_4 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.3.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletDesc.4 = STRING: Outlet_5 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.3.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletDesc.5 = STRING: Outlet_6 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.3.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletDesc.6 = STRING: Outlet_7 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.3.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletDesc.7 = STRING: Outlet_8 */ { "unmapped.p1OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.3.7", NULL, SU_FLAG_OK, NULL },

	/* p1OutletSocket.0 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.4.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSocket.1 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.4.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSocket.2 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.4.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSocket.3 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.4.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSocket.4 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.4.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSocket.5 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.4.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSocket.6 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.4.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletSocket.7 = STRING: IEC 320 C13 */ { "unmapped.p1OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.4.7", NULL, SU_FLAG_OK, NULL },

	/* p1OutletFuse.0 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.5.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletFuse.1 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.5.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletFuse.2 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.5.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletFuse.3 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.5.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletFuse.4 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.5.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletFuse.5 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.5.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletFuse.6 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.5.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletFuse.7 = STRING: -- */ { "unmapped.p1OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.1.1.5.7", NULL, SU_FLAG_OK, NULL },

	/* p1OutletOnOff.0 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.6.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletOnOff.1 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.6.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletOnOff.2 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.6.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletOnOff.3 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.6.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletOnOff.4 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.6.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletOnOff.5 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.6.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletOnOff.6 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.6.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletOnOff.7 = INTEGER: on(1) */ { "unmapped.p1OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.6.7", NULL, SU_FLAG_OK, NULL },

	/* p1OutletLoad.0 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.7.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoad.1 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.7.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoad.2 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.7.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoad.3 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.7.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoad.4 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.7.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoad.5 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.7.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoad.6 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.7.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoad.7 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.7.7", NULL, SU_FLAG_OK, NULL },

	/* p1OutletLoadLowLimit.0 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.8.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadLowLimit.1 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.8.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadLowLimit.2 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.8.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadLowLimit.3 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.8.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadLowLimit.4 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.8.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadLowLimit.5 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.8.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadLowLimit.6 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.8.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadLowLimit.7 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadLowLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.8.7", NULL, SU_FLAG_OK, NULL },

	/* p1OutletLoadHighLimit.0 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.9.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadHighLimit.1 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.9.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadHighLimit.2 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.9.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadHighLimit.3 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.9.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadHighLimit.4 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.9.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadHighLimit.5 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.9.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadHighLimit.6 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.9.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletLoadHighLimit.7 = INTEGER: -1 tenth A */ { "unmapped.p1OutletLoadHighLimit", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.9.7", NULL, SU_FLAG_OK, NULL },

	/* p1OutletEnergy.0 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.10.0", NULL, SU_FLAG_OK, NULL },
	/* p1OutletEnergy.1 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.10.1", NULL, SU_FLAG_OK, NULL },
	/* p1OutletEnergy.2 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.10.2", NULL, SU_FLAG_OK, NULL },
	/* p1OutletEnergy.3 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.10.3", NULL, SU_FLAG_OK, NULL },
	/* p1OutletEnergy.4 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.10.4", NULL, SU_FLAG_OK, NULL },
	/* p1OutletEnergy.5 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.10.5", NULL, SU_FLAG_OK, NULL },
	/* p1OutletEnergy.6 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.10.6", NULL, SU_FLAG_OK, NULL },
	/* p1OutletEnergy.7 = INTEGER: -1 hundredth kWh */ { "unmapped.p1OutletEnergy", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.10.7", NULL, SU_FLAG_OK, NULL },

	/* p1Power.0 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.11.0", NULL, SU_FLAG_OK, NULL },
	/* p1Power.1 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.11.1", NULL, SU_FLAG_OK, NULL },
	/* p1Power.2 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.11.2", NULL, SU_FLAG_OK, NULL },
	/* p1Power.3 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.11.3", NULL, SU_FLAG_OK, NULL },
	/* p1Power.4 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.11.4", NULL, SU_FLAG_OK, NULL },
	/* p1Power.5 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.11.5", NULL, SU_FLAG_OK, NULL },
	/* p1Power.6 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.11.6", NULL, SU_FLAG_OK, NULL },
	/* p1Power.7 = INTEGER: -1 hundredth W */ { "unmapped.p1Power", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.11.7", NULL, SU_FLAG_OK, NULL },

	/* p1PowerFactor.0 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.12.0", NULL, SU_FLAG_OK, NULL },
	/* p1PowerFactor.1 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.12.1", NULL, SU_FLAG_OK, NULL },
	/* p1PowerFactor.2 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.12.2", NULL, SU_FLAG_OK, NULL },
	/* p1PowerFactor.3 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.12.3", NULL, SU_FLAG_OK, NULL },
	/* p1PowerFactor.4 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.12.4", NULL, SU_FLAG_OK, NULL },
	/* p1PowerFactor.5 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.12.5", NULL, SU_FLAG_OK, NULL },
	/* p1PowerFactor.6 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.12.6", NULL, SU_FLAG_OK, NULL },
	/* p1PowerFactor.7 = INTEGER: -1 hundredth */ { "unmapped.p1PowerFactor", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.12.7", NULL, SU_FLAG_OK, NULL },

	/* power1OutletEntry.13.0 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.13.0", NULL, SU_FLAG_OK, NULL },
	/* power1OutletEntry.13.1 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.13.1", NULL, SU_FLAG_OK, NULL },
	/* power1OutletEntry.13.2 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.13.2", NULL, SU_FLAG_OK, NULL },
	/* power1OutletEntry.13.3 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.13.3", NULL, SU_FLAG_OK, NULL },
	/* power1OutletEntry.13.4 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.13.4", NULL, SU_FLAG_OK, NULL },
	/* power1OutletEntry.13.5 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.13.5", NULL, SU_FLAG_OK, NULL },
	/* power1OutletEntry.13.6 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.13.6", NULL, SU_FLAG_OK, NULL },
	/* power1OutletEntry.13.7 = INTEGER: 0 */ { "unmapped.power1OutletEntry", 0, 1, ".1.3.6.1.4.1.55508.1.2.1.1.13.7", NULL, SU_FLAG_OK, NULL },

	/* p3OutletSeq.0 = INTEGER: 0 */ { "unmapped.p3OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.1.0", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSeq.1 = INTEGER: 1 */ { "unmapped.p3OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.1.1", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSeq.2 = INTEGER: 2 */ { "unmapped.p3OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.1.2", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSeq.3 = INTEGER: 3 */ { "unmapped.p3OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.1.3", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSeq.4 = INTEGER: 4 */ { "unmapped.p3OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.1.4", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSeq.5 = INTEGER: 5 */ { "unmapped.p3OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.1.5", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSeq.6 = INTEGER: 6 */ { "unmapped.p3OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.1.6", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSeq.7 = INTEGER: 7 */ { "unmapped.p3OutletSeq", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.1.7", NULL, SU_FLAG_OK, NULL },

	/* p3OutletNo.0 = STRING: 1 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.2.0", NULL, SU_FLAG_OK, NULL },
	/* p3OutletNo.1 = STRING: 2 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.2.1", NULL, SU_FLAG_OK, NULL },
	/* p3OutletNo.2 = STRING: 3 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.2.2", NULL, SU_FLAG_OK, NULL },
	/* p3OutletNo.3 = STRING: 4 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.2.3", NULL, SU_FLAG_OK, NULL },
	/* p3OutletNo.4 = STRING: 5 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.2.4", NULL, SU_FLAG_OK, NULL },
	/* p3OutletNo.5 = STRING: 6 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.2.5", NULL, SU_FLAG_OK, NULL },
	/* p3OutletNo.6 = STRING: 7 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.2.6", NULL, SU_FLAG_OK, NULL },
	/* p3OutletNo.7 = STRING: 8 */ { "unmapped.p3OutletNo", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.2.7", NULL, SU_FLAG_OK, NULL },

	/* p3OutletDesc.0 = STRING: ALL */     { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.3.0", NULL, SU_FLAG_OK, NULL },
	/* p3OutletDesc.1 = STRING: Group 2 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.3.1", NULL, SU_FLAG_OK, NULL },
	/* p3OutletDesc.2 = STRING: Group 3 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.3.2", NULL, SU_FLAG_OK, NULL },
	/* p3OutletDesc.3 = STRING: Group 4 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.3.3", NULL, SU_FLAG_OK, NULL },
	/* p3OutletDesc.4 = STRING: Group 5 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.3.4", NULL, SU_FLAG_OK, NULL },
	/* p3OutletDesc.5 = STRING: Group 6 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.3.5", NULL, SU_FLAG_OK, NULL },
	/* p3OutletDesc.6 = STRING: Group 7 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.3.6", NULL, SU_FLAG_OK, NULL },
	/* p3OutletDesc.7 = STRING: Group 8 */ { "unmapped.p3OutletDesc", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.3.7", NULL, SU_FLAG_OK, NULL },

	/* p3OutletSocket.0 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.4.0", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSocket.1 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.4.1", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSocket.2 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.4.2", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSocket.3 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.4.3", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSocket.4 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.4.4", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSocket.5 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.4.5", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSocket.6 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.4.6", NULL, SU_FLAG_OK, NULL },
	/* p3OutletSocket.7 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletSocket", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.4.7", NULL, SU_FLAG_OK, NULL },

	/* p3OutletFuse.0 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.5.0", NULL, SU_FLAG_OK, NULL },
	/* p3OutletFuse.1 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.5.1", NULL, SU_FLAG_OK, NULL },
	/* p3OutletFuse.2 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.5.2", NULL, SU_FLAG_OK, NULL },
	/* p3OutletFuse.3 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.5.3", NULL, SU_FLAG_OK, NULL },
	/* p3OutletFuse.4 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.5.4", NULL, SU_FLAG_OK, NULL },
	/* p3OutletFuse.5 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.5.5", NULL, SU_FLAG_OK, NULL },
	/* p3OutletFuse.6 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.5.6", NULL, SU_FLAG_OK, NULL },
	/* p3OutletFuse.7 = Wrong Type (should be OCTET STRING): INTEGER: -1 */ { "unmapped.p3OutletFuse", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.3.1.5.7", NULL, SU_FLAG_OK, NULL },

	/* p3OutletOnOff.0 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.6.0", NULL, SU_FLAG_OK, NULL },
	/* p3OutletOnOff.1 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.6.1", NULL, SU_FLAG_OK, NULL },
	/* p3OutletOnOff.2 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.6.2", NULL, SU_FLAG_OK, NULL },
	/* p3OutletOnOff.3 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.6.3", NULL, SU_FLAG_OK, NULL },
	/* p3OutletOnOff.4 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.6.4", NULL, SU_FLAG_OK, NULL },
	/* p3OutletOnOff.5 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.6.5", NULL, SU_FLAG_OK, NULL },
	/* p3OutletOnOff.6 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.6.6", NULL, SU_FLAG_OK, NULL },
	/* p3OutletOnOff.7 = INTEGER: none(-1) */ { "unmapped.p3OutletOnOff", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.6.7", NULL, SU_FLAG_OK, NULL },

	/* p3OutletLoad.0 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.7.0", NULL, SU_FLAG_OK, NULL },
	/* p3OutletLoad.1 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.7.1", NULL, SU_FLAG_OK, NULL },
	/* p3OutletLoad.2 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.7.2", NULL, SU_FLAG_OK, NULL },
	/* p3OutletLoad.3 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.7.3", NULL, SU_FLAG_OK, NULL },
	/* p3OutletLoad.4 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.7.4", NULL, SU_FLAG_OK, NULL },
	/* p3OutletLoad.5 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.7.5", NULL, SU_FLAG_OK, NULL },
	/* p3OutletLoad.6 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.7.6", NULL, SU_FLAG_OK, NULL },
	/* p3OutletLoad.7 = INTEGER: -1 tenth A */ { "unmapped.p3OutletLoad", 0, 1, ".1.3.6.1.4.1.55508.1.2.3.1.7.7", NULL, SU_FLAG_OK, NULL },

	/* powerSeq.0 = INTEGER: 0 */                     { "unmapped.powerSeq", 0, 1,                             ".1.3.6.1.4.1.55508.1.2.5.1.1.0", NULL, SU_FLAG_OK, NULL },
	/* powerID.0 = STRING: 505197 */                  { "unmapped.powerID", ST_FLAG_STRING, SU_INFOSIZE,       ".1.3.6.1.4.1.55508.1.2.5.1.2.0", NULL, SU_FLAG_OK, NULL },
//	/* powerName.0 = STRING: FlexPDU */               { "unmapped.powerName", ST_FLAG_STRING, SU_INFOSIZE,     ".1.3.6.1.4.1.55508.1.2.5.1.3.0", NULL, SU_FLAG_OK, NULL },
	/* powerType.0 = STRING: AC */                    { "unmapped.powerType", ST_FLAG_STRING, SU_INFOSIZE,     ".1.3.6.1.4.1.55508.1.2.5.1.4.0", NULL, SU_FLAG_OK, NULL },
//	/* powerModel.0 = STRING: FLXTSWC20C1 */          { "unmapped.powerModel", ST_FLAG_STRING, SU_INFOSIZE,    ".1.3.6.1.4.1.55508.1.2.5.1.5.0", NULL, SU_FLAG_OK, NULL },
	/* powerOnGap.0 = INTEGER: 1 */                   { "unmapped.powerOnGap", 0, 1,                           ".1.3.6.1.4.1.55508.1.2.5.1.6.0", NULL, SU_FLAG_OK, NULL },
	/* totalLoadA.0 = INTEGER: 0 tenth A */           { "unmapped.totalLoadA", 0, 1,                           ".1.3.6.1.4.1.55508.1.2.5.1.7.0", NULL, SU_FLAG_OK, NULL },
	/* totalLoadALowLimit.0 = INTEGER: 0 tenth A */   { "unmapped.totalLoadALowLimit", 0, 1,                   ".1.3.6.1.4.1.55508.1.2.5.1.8.0", NULL, SU_FLAG_OK, NULL },
	/* totalLoadAHighLimit.0 = INTEGER: 160 tenth A */{ "unmapped.totalLoadAHighLimit", 0, 1,                  ".1.3.6.1.4.1.55508.1.2.5.1.9.0", NULL, SU_FLAG_OK, NULL },
	/* totalLoadB.0 = INTEGER: -1 tenth A */          { "unmapped.totalLoadB", 0, 1,                           ".1.3.6.1.4.1.55508.1.2.5.1.10.0", NULL, SU_FLAG_OK, NULL },
	/* totalLoadBLowLimit.0 = INTEGER: -1 tenth A */  { "unmapped.totalLoadBLowLimit", 0, 1,                   ".1.3.6.1.4.1.55508.1.2.5.1.11.0", NULL, SU_FLAG_OK, NULL },
	/* totalLoadBHighLimit.0 = INTEGER: -1 tenth A */ { "unmapped.totalLoadBHighLimit", 0, 1,                  ".1.3.6.1.4.1.55508.1.2.5.1.12.0", NULL, SU_FLAG_OK, NULL },
	/* totalLoadC.0 = INTEGER: -1 tenth A */          { "unmapped.totalLoadC", 0, 1,                           ".1.3.6.1.4.1.55508.1.2.5.1.13.0", NULL, SU_FLAG_OK, NULL },
	/* totalLoadCLowLimit.0 = INTEGER: -1 tenth A */  { "unmapped.totalLoadCLowLimit", 0, 1,                   ".1.3.6.1.4.1.55508.1.2.5.1.14.0", NULL, SU_FLAG_OK, NULL },
	/* totalLoadCHighLimit.0 = INTEGER: -1 tenth A */ { "unmapped.totalLoadCHighLimit", 0, 1,                  ".1.3.6.1.4.1.55508.1.2.5.1.15.0", NULL, SU_FLAG_OK, NULL },
	/* voltageA.0 = INTEGER: 241 V */                 { "unmapped.voltageA", 0, 1,                             ".1.3.6.1.4.1.55508.1.2.5.1.16.0", NULL, SU_FLAG_OK, NULL },
	/* voltageB.0 = INTEGER: -1 V */                  { "unmapped.voltageB", 0, 1,                             ".1.3.6.1.4.1.55508.1.2.5.1.17.0", NULL, SU_FLAG_OK, NULL },
	/* voltageC.0 = INTEGER: -1 V */                  { "unmapped.voltageC", 0, 1,                             ".1.3.6.1.4.1.55508.1.2.5.1.18.0", NULL, SU_FLAG_OK, NULL },
	/* powerEnergy.0 = INTEGER: 0 tenth kWh */        { "unmapped.powerEnergy", 0, 1,                          ".1.3.6.1.4.1.55508.1.2.5.1.19.0", NULL, SU_FLAG_OK, NULL },
	/* activePowerA.0 = INTEGER: 0 tenth W */         { "unmapped.activePowerA", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.20.0", NULL, SU_FLAG_OK, NULL },
	/* ratedPowerA.0 = INTEGER: 35200 tenth W */      { "unmapped.ratedPowerA", 0, 1,                          ".1.3.6.1.4.1.55508.1.2.5.1.21.0", NULL, SU_FLAG_OK, NULL },
	/* remainPowerA.0 = INTEGER: 35200 tenth W */     { "unmapped.remainPowerA", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.22.0", NULL, SU_FLAG_OK, NULL },
	/* apparentPowerA.0 = INTEGER: 0 tenth W */       { "unmapped.apparentPowerA", 0, 1,                       ".1.3.6.1.4.1.55508.1.2.5.1.23.0", NULL, SU_FLAG_OK, NULL },
	/* powerFactorA.0 = INTEGER: 0 hundredth */       { "unmapped.powerFactorA", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.24.0", NULL, SU_FLAG_OK, NULL },
	/* activePowerB.0 = INTEGER: -1 tenth W */        { "unmapped.activePowerB", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.25.0", NULL, SU_FLAG_OK, NULL },
	/* ratedPowerB.0 = INTEGER: -1 tenth W */         { "unmapped.ratedPowerB", 0, 1,                          ".1.3.6.1.4.1.55508.1.2.5.1.26.0", NULL, SU_FLAG_OK, NULL },
	/* remainPowerB.0 = INTEGER: -1 tenth W */        { "unmapped.remainPowerB", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.27.0", NULL, SU_FLAG_OK, NULL },
	/* apparentPowerB.0 = INTEGER: -1 tenth W */      { "unmapped.apparentPowerB", 0, 1,                       ".1.3.6.1.4.1.55508.1.2.5.1.28.0", NULL, SU_FLAG_OK, NULL },
	/* powerFactorB.0 = INTEGER: -1 hundredth */      { "unmapped.powerFactorB", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.29.0", NULL, SU_FLAG_OK, NULL },
	/* activePowerC.0 = INTEGER: -1 tenth W */        { "unmapped.activePowerC", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.30.0", NULL, SU_FLAG_OK, NULL },
	/* ratedPowerC.0 = INTEGER: -1 tenth W */         { "unmapped.ratedPowerC", 0, 1,                          ".1.3.6.1.4.1.55508.1.2.5.1.31.0", NULL, SU_FLAG_OK, NULL },
	/* remainPowerC.0 = INTEGER: -1 tenth W */        { "unmapped.remainPowerC", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.32.0", NULL, SU_FLAG_OK, NULL },
	/* apparentPowerC.0 = INTEGER: -1 tenth W */      { "unmapped.apparentPowerC", 0, 1,                       ".1.3.6.1.4.1.55508.1.2.5.1.33.0", NULL, SU_FLAG_OK, NULL },
	/* powerFactorC.0 = INTEGER: -1 hundredth */      { "unmapped.powerFactorC", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.34.0", NULL, SU_FLAG_OK, NULL },
	/* atsInput.0 = STRING: -- */                     { "unmapped.atsInput", ST_FLAG_STRING, SU_INFOSIZE,      ".1.3.6.1.4.1.55508.1.2.5.1.35.0", NULL, SU_FLAG_OK, NULL },
	/* atsPriority.0 = STRING: -- */                  { "unmapped.atsPriority", ST_FLAG_STRING, SU_INFOSIZE,   ".1.3.6.1.4.1.55508.1.2.5.1.36.0", NULL, SU_FLAG_OK, NULL },
	/* atsAVoltage.0 = INTEGER: -1 V */               { "unmapped.atsAVoltage", 0, 1,                          ".1.3.6.1.4.1.55508.1.2.5.1.37.0", NULL, SU_FLAG_OK, NULL },
	/* atsBVoltage.0 = INTEGER: -1 V */               { "unmapped.atsBVoltage", 0, 1,                          ".1.3.6.1.4.1.55508.1.2.5.1.38.0", NULL, SU_FLAG_OK, NULL },
	/* atsAEnergy.0 = INTEGER: -1 tenth kWh */        { "unmapped.atsAEnergy", 0, 1,                           ".1.3.6.1.4.1.55508.1.2.5.1.39.0", NULL, SU_FLAG_OK, NULL },
	/* atsBEnergy.0 = INTEGER: -1 tenth kWh */        { "unmapped.atsBEnergy", 0, 1,                           ".1.3.6.1.4.1.55508.1.2.5.1.40.0", NULL, SU_FLAG_OK, NULL },
	/* atsSwitchLow.0 = INTEGER: -1 V */              { "unmapped.atsSwitchLow", 0, 1,                         ".1.3.6.1.4.1.55508.1.2.5.1.41.0", NULL, SU_FLAG_OK, NULL },
	/* atsSwitchHigh.0 = INTEGER: -1 V */             { "unmapped.atsSwitchHigh", 0, 1,                        ".1.3.6.1.4.1.55508.1.2.5.1.42.0", NULL, SU_FLAG_OK, NULL },
	/* atsSwitchTime.0 = INTEGER: -1 min */           { "unmapped.atsSwitchTime", 0, 1,                        ".1.3.6.1.4.1.55508.1.2.5.1.43.0", NULL, SU_FLAG_OK, NULL },
	/* atsSwitchLock.0 = STRING: -- */                { "unmapped.atsSwitchLock", ST_FLAG_STRING, SU_INFOSIZE, ".1.3.6.1.4.1.55508.1.2.5.1.44.0", NULL, SU_FLAG_OK, NULL },
	/* atsMonitor.0 = STRING: -- */                   { "unmapped.atsMonitor", ST_FLAG_STRING, SU_INFOSIZE,    ".1.3.6.1.4.1.55508.1.2.5.1.45.0", NULL, SU_FLAG_OK, NULL },
	/* allOutletsSwitch.0 = INTEGER: none(-1) */      { "unmapped.allOutletsSwitch", 0, 1,                     ".1.3.6.1.4.1.55508.1.2.5.1.46.0", NULL, SU_FLAG_OK, NULL },
	/* frequency.0 = INTEGER: 50 Hz */                { "unmapped.frequency", 0, 1,                            ".1.3.6.1.4.1.55508.1.2.5.1.47.0", NULL, SU_FLAG_OK, NULL },
	/* voltageALowLimit.0 = INTEGER: 190 V */         { "unmapped.voltageALowLimit", 0, 1,                     ".1.3.6.1.4.1.55508.1.2.5.1.48.0", NULL, SU_FLAG_OK, NULL },
	/* voltageAHighLimit.0 = INTEGER: 280 V */        { "unmapped.voltageAHighLimit", 0, 1,                    ".1.3.6.1.4.1.55508.1.2.5.1.49.0", NULL, SU_FLAG_OK, NULL },
	/* voltageBLowLimit.0 = INTEGER: -1 V */          { "unmapped.voltageBLowLimit", 0, 1,                     ".1.3.6.1.4.1.55508.1.2.5.1.50.0", NULL, SU_FLAG_OK, NULL },
	/* voltageBHighLimit.0 = INTEGER: -1 V */         { "unmapped.voltageBHighLimit", 0, 1,                    ".1.3.6.1.4.1.55508.1.2.5.1.51.0", NULL, SU_FLAG_OK, NULL },
	/* voltageCLowLimit.0 = INTEGER: -1 V */          { "unmapped.voltageCLowLimit", 0, 1,                     ".1.3.6.1.4.1.55508.1.2.5.1.52.0", NULL, SU_FLAG_OK, NULL },
	/* voltageCHighLimit.0 = INTEGER: -1 V */         { "unmapped.voltageCHighLimit", 0, 1,                    ".1.3.6.1.4.1.55508.1.2.5.1.53.0", NULL, SU_FLAG_OK, NULL },
	/* powerSummaryEntry.54.0 = INTEGER: 16 */        { "unmapped.powerSummaryEntry", 0, 1,                    ".1.3.6.1.4.1.55508.1.2.5.1.54.0", NULL, SU_FLAG_OK, NULL },
#endif	/* if WITH_UNMAPPED_DATA_POINTS */

	/* end of structure. */
	{ NULL, 0, 0, NULL, NULL, 0, NULL }
};

mib2nut_info_t eaton_pdu_flex = { "eaton_pdu_flex", EATON_PDU_FLEX_MIB_VERSION, NULL, NULL, eaton_pdu_flex_mib, EATON_PDU_FLEX_SYSOID, NULL };
