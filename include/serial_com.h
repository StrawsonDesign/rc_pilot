/**
 * <serial_com.h>
 *
 * @brief       Simplified serial utilities
 *
 * @author eolson@mit.edu (2004)
 *
 * @addtogroup SerialCom
 * @{
 */

#ifndef SERIAL_COM_H
#define SERIAL_COM_H

// clang-format off

#ifdef __cplusplus
extern "C"
{
#endif
    
/**
 * @brief   Creates a basic fd, setting baud to 9600, raw data i/o
 * (no flow control, no fancy character handling. Configures
 * it for blocking reads.  8 data bits, 1 stop bit, no parity.
 *
 * @return      the fd or -1 on error
 */
int serial_open(const char *port, int baud, int blocking);

/** @brief   Set the baud rate, where the baudrate is just the integer value
 *  desired.
 *
 * @return      non-zero on error.
 */
int serial_setbaud(int fd, int baudrate);

/**
 * @brief   Enable cts/rts flow control.
 *
 * @return      non-zero on error.
 */
int serial_enablectsrts(int fd);

/**
 * @brief   Enable xon/xoff flow control.
 *
 * @returns non-zero on error.
 */
int serial_enablexon(int fd);

/**
 * @brief   Set the port to 8 data bits, 2 stop bits, no parity.
 *
 * @return non-zero on error.
 */
int serial_set_N82(int fd);

/**
 * @brief   Closes the serial interface
 *
 * @return non-zero on error
 */
int serial_close(int fd);
#ifdef __cplusplus
}
#endif

// clang-format on

#endif /*__SERIAL_COM__ */

/* @} end group SerialCom */