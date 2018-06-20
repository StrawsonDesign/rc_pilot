/**
 * @file mavlink_manager.c
 *
 * @ brief
 *
 *
 * @author Jae
 * @date 2/5/2018
 */

//TODO
// read and understand fly_function_declaration.h, add doxy comments once fully understood
// sending to multiple listeners?
// cstate in fly_types.h, add a part about mocap packet data, data update flag


#include <stdio.h>
#include <rc/mavlink_udp.h>
#include <mavlink_manager.h>

#define LOCALHOST_IP	"127.0.0.1"
#define DEFAULT_SYS_ID	1



void callback_func_any()
{
	int sysid = rc_mav_get_sys_id_of_last_msg_any();
	int msg_id = rc_mav_msg_id_of_last_msg();
	printf("received msg_id: %d ", msg_id);
	// TODO uncomment print msg name when this works
	//rc_mav_print_msg_name(msg_id);
	printf(" from sysid: %d \n", sysid);
	return;
}

/*
void* heartbeat_sender_thread(__attribute__ ((unused)) void* dummy)
{
	while(rc_get_state() != EXITING){
		if(rc_mav_send_heartbeat_abbreviated()){
			fprintf(stderr,"failed to send heartbeat\n");
		}
		rc_usleep(1000000);
	}
	return;

}
*/

int start_mavlink_manager()
{
	// set default options before checking options
	dest_ip=LOCALHOST_IP;
	my_sys_id=DEFAULT_SYS_ID;
	port=RC_MAV_DEFAULT_UDP_PORT;

	// initialize the UDP port and listening thread with the rc_mav lib
	if(rc_mav_init(my_sys_id, dest_ip, port) < 0) return -1;

	// set the heartbeat callback to print something when receiving
	rc_mav_set_callback_all(callback_func_any);
	return 0;
}

int cleanup_mavlink_manager()
{
	return rc_mav_cleanup();
}