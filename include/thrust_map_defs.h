/**
 * @headerfile thrust_map_defs.h
 *
 * This are the experimental results that map ESC input to thrust output. The
 * inputs (left column) must range from 0 to 1. The units of thrust (right
 * column) don't matter as the max thrust is normalized form 0 to 1 too. There
 * may be as many rows as you like, as long as they are monotonically
 * increasing.
 **/


#ifndef THRUST_MAP_DEFS_H
#define THRUST_MAP_DEFS_H

/**
 * enum thrust_map_t
 *
 * the user may select from the following preconfigured thrust maps
 */
typedef enum thrust_map_t{
	MN1806_1400KV_4S,
	F20_2300KV_2S
} thrust_map_t;



// Tiger Motor MN1806, 1400KV 6x4.5" 3-blade prop, 14.8V,
// BLheli ESC Low Timing
// this one is in Newtons but it doesn't really matter
const int mn1806_1400kv_4s_points = 11;
double mn1806_1400kv_4s_map[][2] =  \
{{0.0,	0.0000}, \
 {0.1,	0.2982}, \
 {0.2,	0.6310}, \
 {0.3,	1.0281}, \
 {0.4,	1.5224}, \
 {0.5,	2.0310}, \
 {0.6,	2.5791}, \
 {0.7,	3.1365}, \
 {0.8,	3.7282}, \
 {0.9,	4.3147}, \
 {1.0,	4.7258}};



// tiger motor F20 2300kv motor, 2S lipo, 4x4.0" 3-blade props
// blheli esc med-low timing
// thrust units in gram-force but doesn't really matter
const int f20_2300kv_2s_points = 21;
double f20_2300kv_2s_map[][2] = \
{{0.00,	0.000000}, \
 {0.05,	6.892067}, \
 {0.10,	12.57954}, \
 {0.15,	18.84790}, \
 {0.20,	26.16294}, \
 {0.25,	33.98255}, \
 {0.30,	41.60790}, \
 {0.35,	49.32732}, \
 {0.40,	58.27048}, \
 {0.45,	67.83613}, \
 {0.50,	78.20817}, \
 {0.55,	88.27728}, \
 {0.60,	100.1058}, \
 {0.65,	110.3643}, \
 {0.70,	121.6316}, \
 {0.75,	132.2155}, \
 {0.80,	145.0420}, \
 {0.85,	154.6838}, \
 {0.90,	162.0185}, \
 {0.95,	168.4321}, \
 {1.00,	177.1643}};




#endif // THRUST_MAP_DEFS_H
