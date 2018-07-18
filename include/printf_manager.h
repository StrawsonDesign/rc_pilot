/**
 * <printf_manager.h>
 *
 * @brief      Functions to start and stop the printf mnaager which is a
 *             separate thread printing data to the console for debugging.
 */


#ifndef PRINTF_MANAGER_H
#define PRINTF_MANAGER_H

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

/**
 * @brief      Start the printf_manager thread which should be the only thing
 *             printing to the screen besides error messages from other threads.
 *
 * @return     0 on success, -1 on failure
 */
int printf_init();

/**
 * @brief      Waits for the printf manager thread to exit.
 *
 * @return     0 on clean exit, -1 on exit time out/force close
 */
int printf_cleanup();



#endif //PRINTF_MANAGER_H
