#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/un.h>
#include <netinet/ip.h>
#include <linux/types.h>
#include <linux/if_ether.h>

#include <netlink/attr.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/genl.h>
#include <netlink/handlers.h>
#include <netlink/msg.h>
#include <netlink/netlink.h>
#include <netlink/socket.h>
#include <linux/nl80211.h>

#include <gps.h>
#include <math.h>

#include <opendroneid.h>

struct global {
	char server[1024];
	char port[16];
	char mac[6];
	uint8_t send_counter;
};

void usage(char *name)
{
	/* TODO */
	fprintf(stderr,"%s\n", name);
}

static int nl80211_id = -1;

static struct nl_sock *nl80211_socket_create(void)
{
	struct nl_sock *nl_sock = NULL;

	nl_sock = nl_socket_alloc();
	if (!nl_sock) {
		fprintf(stderr, "Failed to create netlink socket\n");
		goto err;
	}

	if (genl_connect(nl_sock)) {
		fprintf(stderr, "Failed to connect to generic netlink\n");
		goto err;
	}

	nl80211_id = genl_ctrl_resolve(nl_sock, "nl80211");
	if (nl80211_id < 0) {
		fprintf(stderr, "nl80211 not found\n");
		goto err;
	}

	return nl_sock;

err:
	nl_socket_free(nl_sock);
	return NULL;
}

int send_nl80211_action(struct nl_sock *nl_sock, int if_index, void *action, size_t len)
{
	struct nl_msg *msg = NULL;
	struct nl_cb *s_cb;
	int ret = -1;

	s_cb = nl_cb_alloc(NL_CB_DEBUG);
	if (!s_cb) {
		fprintf(stderr, "\n");
		goto nla_put_failure;
	}
	nl_socket_set_cb(nl_sock, s_cb);

	msg = nlmsg_alloc();
	if (!msg) {
		fprintf(stderr, "Could not create netlink message\n");
		goto nla_put_failure;
	}

	genlmsg_put(msg, 0, 0, nl80211_id, 0, 0, NL80211_CMD_FRAME, 0);
	NLA_PUT_U32(msg, NL80211_ATTR_IFINDEX, if_index);
	NLA_PUT(msg, NL80211_ATTR_FRAME, len, action);
	NLA_PUT_FLAG(msg, NL80211_ATTR_DONT_WAIT_FOR_ACK);
	ret = nl_send_auto_complete(nl_sock, msg);
	if (ret < 0)
		goto nla_put_failure;

	nl_wait_for_ack(nl_sock);
	nlmsg_free(msg);
	return 0;

nla_put_failure:
	nl_cb_put(s_cb);
	nlmsg_free(msg);
	return ret;

}

int read_arguments(int argc, char *argv[], ODID_UAS_Data *drone, struct global *global)
{
	int opt;

	strncpy(drone->BasicID.UASID, "12345678901234567890", sizeof(drone->BasicID.UASID));
	drone->BasicID.IDType = ODID_IDTYPE_SERIAL_NUMBER;
	drone->BasicID.UASType = ODID_UAVTYPE_LTA_UNPOWERED; /* balloon */

	strncpy(global->server, "127.0.0.1", sizeof(global->server));
	strncpy(global->port, (char *)DEFAULT_GPSD_PORT, sizeof(global->port));

	while((opt = getopt(argc, argv, ":p:")) != -1) {
		switch (opt) {
			case 'p':
				strncpy(global->port, optarg, sizeof(global->port));
		}
	}

	printf("global: %s:%s", global->server, global->port);

	return 0;
}

/**
 * drone_adopt_gps_data - adopt GPS data into the drone status info
 * @gpsdata: gps data from gpsd
 * @drone: general drone status information
 */
static void drone_adopt_gps_data(ODID_UAS_Data *drone,
				 struct gps_data_t *gpsdata)
{
	/*
	*	ALL READOUTS FROM GPSD
	*/
	printf("\nGPS:\tmode %d\n", gpsdata->fix.mode);

	/* Latitude/Longitude */
	drone->Location.Latitude = gpsdata->fix.latitude;
	drone->Location.Longitude = gpsdata->fix.longitude;
	drone->Location.HorizAccuracy = gpsdata->fix.epy > gpsdata->fix.epx ? gpsdata->fix.epy : gpsdata->fix.epx;

	/* Altitude */
	drone->Location.AltitudeGeo = gpsdata->fix.altitude;
	drone->Location.VertAccuracy = gpsdata->fix.epv;

	/* Horizontal movement */
/*	drone->Location.Heading = gpsdata->fix.track;*/
	drone->Location.SpeedAccuracy = gpsdata->fix.eps;
	drone->Location.SpeedNS = cos(gpsdata->fix.track*M_PI/180) * gpsdata->fix.speed;	// TODO con return type
	drone->Location.SpeedEW = sin(gpsdata->fix.track*M_PI/180) * gpsdata->fix.speed;	// TODO sin return type

	/* Vertical movement */
	drone->Location.SpeedVertical = gpsdata->fix.climb;
	/* drone->Location.SpeedAccuracy = gpsdata->fix.epc; */
	/* TODO: shouldn't vertical accuracy be part of the speed accuracy ? */

	/* Time */
	drone->Location.TimeStamp = gpsdata->fix.time;
	drone->Location.TSAccuracy = gpsdata->fix.ept;

	printf("drone:\n\t"
		"TimeStamp: %f, time since last hour (100ms): %ld, TSAccuracy: %f\n\t"
		"Latitude: %f, Longitude: %f\n\t"
		"SpeedNS: %f, SpeedEW: %f\n\t"
		"SpeedVertical: %f\n",
		drone->Location.TimeStamp, (uint64_t)(drone->Location.TimeStamp*10)%36000, drone->Location.TSAccuracy,	// TODO
		drone->Location.Latitude, drone->Location.Longitude,
		drone->Location.SpeedNS, drone->Location.SpeedEW,
		drone->Location.SpeedVertical
	);
}

/**
 * drone_export_gps_data - adopt GPS data into the drone status info
 * @gpsdata: gps data from gpsd
 * @drone: general drone status information
 */
void drone_export_gps_data(ODID_UAS_Data *UAS_Data, char* filename)
{
	FILE *fp;

	/* build json object from UAS_Data */
	/*
	{
		"Version": "x.x",
		"Response": {
			"BasicID": {
				"UASType": <>,
				"IDType": <>,
				"UASID": <>
			},
			"Location": {
				"Status": <>,
				"SpeedNS": <>,
				"SpeedEW": <>,
				"SpeedVertical": <>,
				"Latitude": <>,
				"Longitude": <>,
				"AltitudeBaro": <>,
				"AltitudeGeo": <>,
				"HeightAboveTakeoff": <>,
				"HAccuracy": <>,
				"VAccuracy": <>,
				"SpAccuracy": <>,
				"TSAccuracy": <>,
				"TimeStamp": <>
			},
			"Authentication": {
				"AuthType": <>,
				"AuthToken": <>
			},
			"SelfID": {
				"Name": <>,
				"Description": <>
			},
			"Operator": {
				"LocationSource": <>,
				"Latitude": <>,
				"Longitude": <>,
				"GroupCount": <>,
				"GroupRadius": <>,
				"GroupCeiling": <>
			}
		}
	}
	*/

	fp = fopen(filename, "w");
	if (fp != NULL) {
		fprintf(fp, "{\n\t\"Version\": \"0.0\",\n\t\"Response\": {\n");

		fprintf(fp, "\t\t\"BasicID\": {\n");
		fprintf(fp, "\t\t\t\"UASType\": %i,\n", UAS_Data->BasicID.UASType);
		fprintf(fp, "\t\t\t\"IDType\": %i,\n", UAS_Data->BasicID.IDType);
		fprintf(fp, "\t\t\t\"UASID\": %s\n", UAS_Data->BasicID.UASID);
		fprintf(fp, "\t\t},\n");

		fprintf(fp, "\t\t\"Location\": {\n");
		fprintf(fp, "\t\t\t\"Status\": %i,\n", UAS_Data->Location);
		fprintf(fp, "\t\t\t\"SpeedNS\": %f,\n", UAS_Data->Location.SpeedNS);
		fprintf(fp, "\t\t\t\"SpeedEW\": %f,\n", UAS_Data->Location.SpeedEW);
		fprintf(fp, "\t\t\t\"SpeedVertical\": %f,\n", UAS_Data->Location.SpeedVertical);
		fprintf(fp, "\t\t\t\"Latitude\": %f,\n", UAS_Data->Location.Latitude);
		fprintf(fp, "\t\t\t\"Longitude\": %f,\n", UAS_Data->Location.Longitude);
		fprintf(fp, "\t\t\t\"AltitudeBaro\": %f,\n", UAS_Data->Location.AltitudeBaro);
		fprintf(fp, "\t\t\t\"AltitudeGeo\": %f,\n", UAS_Data->Location.AltitudeGeo);
		fprintf(fp, "\t\t\t\"HeightAboveTakeoff\": %f,\n", UAS_Data->Location.HeightAboveTakeoff);
		fprintf(fp, "\t\t\t\"HorizAccuracy\": %f,\n", UAS_Data->Location.HorizAccuracy);
		fprintf(fp, "\t\t\t\"VertAccuracy\": %f,\n", UAS_Data->Location.VertAccuracy);
		fprintf(fp, "\t\t\t\"SpeedAccuracy\": %f,\n", UAS_Data->Location.SpeedAccuracy);
		fprintf(fp, "\t\t\t\"TSAccuracy\": %f,\n", UAS_Data->Location.TSAccuracy);
		fprintf(fp, "\t\t\t\"TimeStamp\": %f\n", UAS_Data->Location.TimeStamp);
		fprintf(fp, "\t\t},\n");

		fprintf(fp, "\t\t\"Authentication\": {\n");
		fprintf(fp, "\t\t\t\"AuthType\": %i,\n", UAS_Data->Auth.AuthType);
		fprintf(fp, "\t\t\t\"AuthToken\": %s\n", UAS_Data->Auth.AuthData);
		fprintf(fp, "\t\t},\n");

		fprintf(fp, "\t\t\"SelfID\": {\n");
		fprintf(fp, "\t\t\t\"Name\": \"string\",\n");
		fprintf(fp, "\t\t\t\"Description\": %s\n", UAS_Data->SelfID.Desc);
		fprintf(fp, "\t\t},\n");

		fprintf(fp, "\t\t\"Operator\": {\n");
		fprintf(fp, "\t\t\t\"LocationSource\": %i,\n", UAS_Data->System.LocationSource);
		fprintf(fp, "\t\t\t\"Latitude\": %f,\n", UAS_Data->System.Latitude);
		fprintf(fp, "\t\t\t\"Longitude\": %f,\n", UAS_Data->System.Longitude);
		fprintf(fp, "\t\t\t\"GroupCount\": %i,\n", UAS_Data->System.GroupCount);
		fprintf(fp, "\t\t\t\"GroupRadius\": %i,\n", UAS_Data->System.GroupRadius);
		fprintf(fp, "\t\t\t\"GroupCeiling\": %f\n", UAS_Data->System.GroupCeiling);
		fprintf(fp, "\t\t}\n");

		fprintf(fp, "\t}\n}");
		fclose(fp);
	}

	// printBasicID_data(UAS_Data->BasicID);
	// printLocation_data(UAS_Data->Location);
	// printAuth_data(UAS_Data->Auth);
	// printSelfID_data(UAS_Data->SelfID);
	// printSystem_data(UAS_Data->System);
}

/**
 * drone_test_receive_data - receive and process drone information
 */
static void drone_test_receive_data(uint8_t *buf, size_t buf_size, char *mac)
{
	ODID_UAS_Data rcvd;
	int ret;
	char filename[] = "rcvd_drone.json";

	ret = odid_wifi_receive_message_pack_nan_action_frame(&rcvd, mac, buf, buf_size);
	if (ret < 0) {
		return;
	}

	drone_export_gps_data(&rcvd, filename);
}

/**
 * drone_send_data - send information about the drone out
 * @drone: general drone status information
 */
static void drone_send_data(ODID_UAS_Data *drone, struct global *global, struct nl_sock *nl_sock, int if_index)
{
	uint8_t frame_buf[1024];
	int ret;

	ret = odid_wifi_build_message_pack_nan_action_frame(drone, global->mac, global->send_counter++, frame_buf, sizeof(frame_buf));
	if (ret < 0) {
		fprintf(stderr, "%s: odid_wifi_build_message_pack_nan_action_frame failed: %d (%s)", __func__, ret, strerror(ret));
		return;
	}
	{
		int i;

		printf("frame (len %i):\n\t", ret);
		for (i = 0; i < ret; i++) {
			printf("%02x ", frame_buf[i]);
			if (i % 4 == 3)
				printf(" ");

			if (i % 16 == 15)
				printf("\n\t");
		}
		printf("\n");
	}

	char filename[] = "drone.json";
	drone_export_gps_data(drone, filename);
	drone_test_receive_data(frame_buf, (uint8_t)ret, global->mac);

	ret = send_nl80211_action(nl_sock, if_index, frame_buf, ret);
	if (ret < 0) {
		fprintf(stderr, "%s: send_nl80211_action failed: %d (%s)", __func__, ret, strerror(ret));
		return;
	}
}

static int get_device_mac(const char *iface, char *mac, int *if_index)
{
	struct ifreq ifr;
	int sock;

	*if_index = if_nametoindex(iface);
	if (*if_index == 0)
		return -1;

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock < 0)
		return sock;

	strncpy(ifr.ifr_name, iface, sizeof(ifr.ifr_name)-1);
	ifr.ifr_name[sizeof(ifr.ifr_name) - 1] = '\0';

	if (ioctl(sock, SIOCGIFHWADDR, &ifr)== -1) {
		return -1;
	}
	close(sock);

	memcpy(mac, &ifr.ifr_hwaddr.sa_data, 6);

	return 0;
}

int main(int argc, char *argv[])
{
	ODID_UAS_Data drone;
	struct global global;
	struct gps_data_t gpsdata;
	struct nl_sock *nl_sock = NULL;
	int if_index;
	int ret, errno;

	memset(&drone, 0, sizeof(drone));
	memset(&global, 0, sizeof(global));

	if (read_arguments(argc, argv, &drone, &global) < 0) {
		usage(argv[0]);
		return -1;
	}

	/* TODO acquire dynamically */
	if (get_device_mac("wlp2s0", global.mac, &if_index) < 0) {
		fprintf(stderr, "%s: Couldn't acquire wlan0 address\n", argv[0]);

		return -1;
	}

	nl_sock = nl80211_socket_create();
	if (!nl_sock) {
		fprintf(stderr, "%s: Couldn't open nl80211 socket\n", argv[0]);
		goto out;
	}

	if (gps_open(global.server, global.port, &gpsdata) != 0) {
		fprintf(stderr, "%s: gpsd error: %d, %s\n", argv[0],
			errno, gps_errstr(errno));
		goto out;
	}

	gps_stream(&gpsdata, WATCH_ENABLE | WATCH_JSON, NULL);

	/* polling interval is once every 5 seconds */
	while (1) {
		usleep(500000);
		/* read as much as we can using gps_read() */
#if GPSD_API_MAJOR_VERSION >= 7
		while ((ret = gps_read(&gpsdata, NULL, 0)) > 0);
#else
		while ((ret = gps_read(&gpsdata)) > 0);
#endif
		if (ret < 0) {
			fprintf(stderr, "%s: gpsd_read error: %d, %s\n", argv[0],
				errno, gps_errstr(errno));
		}

		drone_adopt_gps_data(&drone, &gpsdata);
		drone_send_data(&drone, &global, nl_sock, if_index);
	}

out:
	nl_socket_free(nl_sock);
	gps_stream(&gpsdata, WATCH_DISABLE, NULL);
	gps_close(&gpsdata);

	return 0;
}