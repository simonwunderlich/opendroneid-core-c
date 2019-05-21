/*
Copyright (C) 2019 Intel Corporation

SPDX-License-Identifier: Apache-2.0

Open Drone ID C Library

Maintainer:
Gabriel Cox
gabriel.c.cox@intel.com
*/

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <byteswap.h>

#include "opendroneid.h"

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define cpu_to_le16(x)  (x)
#else
#define cpu_to_le16(x)      (bswap_16(x))
#endif

#define IEEE80211_FTYPE_MGMT            0x0000
#define IEEE80211_STYPE_ACTION          0x00D0


int odid_message_encode_pack(ODID_UAS_Data *UAS_Data, void *pack, size_t buflen)
{
	ODID_Message_Pack *outPack;
	size_t len = 0;

	/* check if there is enough space for the header. */
	if (sizeof(*outPack) > buflen)
		return -ENOMEM;

	/* TODO: flexibly set optional fields as available */

	outPack = (ODID_Message_Pack *) pack;
	outPack->ProtoVersion = 0;
	outPack->MessageType = 0;
	outPack->SingleMessageSize = ODID_MESSAGE_SIZE;
	outPack->MsgPackSize = 5;
	len += sizeof(*outPack);

	if (len + (outPack->MsgPackSize * ODID_MESSAGE_SIZE) > buflen)
		return -ENOMEM;

	encodeBasicIDMessage((void *)&outPack->Messages[0], &UAS_Data->BasicID);
	encodeLocationMessage((void *)&outPack->Messages[1], &UAS_Data->Location);
	encodeAuthMessage((void *)&outPack->Messages[2], &UAS_Data->Auth);
	encodeSelfIDMessage((void *)&outPack->Messages[3], &UAS_Data->SelfID);
	encodeSystemMessage((void *)&outPack->Messages[4], &UAS_Data->System);
	len += ODID_MESSAGE_SIZE * outPack->MsgPackSize;

	return len;
}

int odid_wifi_build_message_pack_nan_action_frame(ODID_UAS_Data *UAS_Data, char *mac,
						  uint8_t send_counter,
				     		  uint8_t *buf, size_t buf_size)
{
	uint8_t broadcast_addr[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	/* "org.opendroneid.remoteid" hash */
	uint8_t service_id[6] = { 0x88, 0x69, 0x19, 0x9D, 0x92, 0x09 };
	uint8_t wifi_alliance_oui[3] = { 0x50, 0x6F, 0x9A };
	struct ieee80211_mgmt *mgmt;
	struct nan_service_discovery *nsd;
	struct nan_service_descriptor_attribute *nsda;
	struct ODID_service_info *si;
	int ret, len = 0;

	/* IEEE 802.11 Management Header */
	if (len + sizeof(*mgmt) > buf_size)
		return -ENOMEM;

	mgmt = (struct ieee80211_mgmt *)(buf + len);
	memset(mgmt, 0, sizeof(*mgmt));
	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_ACTION);
	mgmt->duration = 0;
	memcpy(mgmt->sa, mac, sizeof(mgmt->sa));
	memcpy(mgmt->da, broadcast_addr, sizeof(mgmt->da));
	memcpy(mgmt->bssid, mac, sizeof(mgmt->bssid));
	mgmt->seq_ctrl = 0;

	len += sizeof(*mgmt);

	/* NAN Service Discovery header */
	if (len + sizeof(*nsd) > buf_size)
		return -ENOMEM;

	nsd = (struct nan_service_discovery *)(buf + len);
	memset(nsd, 0, sizeof(*nsd));
	nsd->category = 0x04;			/* IEEE 802.11 Public Action frame */
	nsd->action_code = 0x09;		/* IEEE 802.11 Public Action frame Vendor Specific*/
	memcpy(nsd->oui, wifi_alliance_oui, sizeof(nsd->oui));
	nsd->oui_type = 0x13;			/* Identify Type and version of the NAN */
	len += sizeof(*nsd);

	/* NAN Attribute for Service Descriptor header */
	if (len + sizeof(*nsda) > buf_size)
		return -ENOMEM;

	nsda = (struct nan_service_descriptor_attribute *)(buf + len);
	nsda->attribute_id = 0x3;		/* Service Descriptor Attribute type */
	memcpy(nsda->service_id, service_id, sizeof(service_id));
	/* always 1 */
	nsda->instance_id = 0x01;		/* always 1 */
	nsda->requestor_instance_id = 0x00;	/* from triggering frame */
	nsda->service_control = 0x10;		/* follow up */
	len += sizeof(*nsda);

	/* ODID Service Info Attribute header */
	if (len + sizeof(*si) > buf_size)
		return -ENOMEM;

	si = (struct ODID_service_info *)(buf + len);
	memset(si, 0, sizeof(*si));
	si->message_counter = send_counter;
	len += sizeof(*si);

	ret = odid_message_encode_pack(UAS_Data, buf + len, buf_size - len);
	if (ret < 0)
		return ret;
	len += ret;

	/* set the lengths according to the message pack lengths */
	nsda->service_info_length = sizeof(*si) + ret;
	nsda->length = sizeof(*nsda) + nsda->service_info_length;

	return len;
}
