/**
 * @file mavlink_helper.c
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

#include <ctype.h> // for isprint()
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h> // to SIGINT signal handler
#include <rc/mavlink_udp.h>

#define LOCALHOST_IP	"127.0.0.1"
#define DEFAULT_SYS_ID	1

const char* dest_ip;
uint8_t my_sys_id;
uint16_t port;
int running;



// called by the rc_mav lib whenever a packet is received
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

int main(int argc, char * argv[])
{
	// set default options before checking options
	dest_ip=LOCALHOST_IP;
	my_sys_id=DEFAULT_SYS_ID;
	port=RC_MAV_DEFAULT_UDP_PORT;

	// parse arguments
	if(parse_args(argc,argv)){
		fprintf(stderr,"failed to parse arguments\n");
		return -1;
	}

	printf("run with -h option to see usage and other options\n");
	// inform the user what settings are being used
	printf("\n");
	printf("Initializing with the following settings:\n");
	printf("dest ip addr: %s\n", dest_ip);
	printf("my system id: %d\n", my_sys_id);
	printf("UDP port: %d\n", port);
	printf("\n");

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, signal_handler);

	// initialize the UDP port and listening thread with the rc_mav lib
	if (rc_mav_init(my_sys_id, dest_ip, port) < 0){
		return -1;
	}

	// set the heartbeat callback to print something when receiving
	rc_mav_set_callback_all(callback_func_any);
	running=1;
	while(running){
		sleep(1);
		if(rc_mav_send_heartbeat_abbreviated()){
			fprintf(stderr,"failed to send heartbeat\n");
		}
		else{
			printf("sent heartbeat\n");
		}
	}

	// stop listening thread and close UDP port
	printf("closing UDP port\n");
	rc_mav_cleanup();
}