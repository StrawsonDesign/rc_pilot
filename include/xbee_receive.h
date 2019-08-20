/**
 * <xbee_receive.h>
 *
 * @brief       Functions for connecting to and recieving xbee messages
 *
 * @addtogroup  XbeeReceive
 * @{
 */

#ifndef XBEE_RECEIVE_H
#define XBEE_RECEIVE_H

#include <stdint.h>

/**
 * @brief       Possition and orientation data sent/received from xbee
 *
 * This message definition must be identical to the sending message.  This depends on the ground
 * station and should be checked prior to using.  There are many different xbee_packet_t's floating
 * around
 */
typedef struct __attribute__((packed)) xbee_packet_t
{
    uint32_t time;           ///< Unique id for the rigid body being described
    float x;                 ///< x-position in the Optitrack frame
    float y;                 ///< y-position in the Optitrack frame
    float z;                 ///< z-position in the Optitrack frame
    float qx;                ///< qx of quaternion
    float qy;                ///< qy of quaternion
    float qz;                ///< qz of quaternion
    float qw;                ///< qw of quaternion
    uint32_t trackingValid;  ///< (bool) of whether or not tracking was valid (0 or 1)
} xbee_packet_t;

#define NUM_FRAMING_BYTES 4                     ///< 2 START bytes + 2 Fletcher-16 checksum bytes
#define OPTI_DATA_LENGTH sizeof(xbee_packet_t)  ///< Actual Packet Being Sent
#define OPTI_PACKET_LENGTH OPTI_DATA_LENGTH + NUM_FRAMING_BYTES

extern xbee_packet_t xbeeMsg;
extern int xbee_portID;

/**
 * @brief       Xbee initialization function
 *
 * @return      0 on success, -1 on failure
 */
int XBEE_init();

/**
 * @brief       Read message recieved from XBee
 *
 * @return      0 on success, -1 on failure
 */
int XBEE_getData();

/**
 * @brief       Print current XBee message to stdout
 */
void XBEE_printData();

#endif /*__XBEE_RECEIVE__ */

/* @} end group XbeeReceive */