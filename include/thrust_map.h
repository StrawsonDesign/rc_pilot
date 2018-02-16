/**
 * @headerfile thrust_map.h
 *
 * @brief      Functions to start and stop the printf mnaager which is a
 *             separate thread printing data to the console for debugging.
 */


#ifndef THURST_MAP_H
#define THURST_MAP_H

#include <thrust_map_defs.h>

/**
 * @brief      Check the thrust map for validity and populate data arrays.
 *
 *
 * @param[in]  map   The map
 *
 * @return     0 on success, -1 on failure
 */
int initialize_thrust_map(thrust_map_t map);

/**
 * @brief      Corrects the motor signal m for non-linear thrust curve in place.
 *
 *
 * @param[in]  m     { parameter_description }
 *
 * @return     0 on success, -1 on error
 */
float map_motor_signal(float m);

#endif // THURST_MAP_H
