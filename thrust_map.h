/*******************************************************************************
* thrust_map.h
*
* This is the experimental data mapping control input to the ESC to thrust
* output. The inputs (left column) must range from 0 to 1. The units of thrust
* (right column) don't matter as the max thrust is normalized form 0 to 1 too.
* There may be as many rows as you like, as long as they are monotonically
* increasing. 
*******************************************************************************/

// Tiger Motor MN1806, 6"x4.5" 3-blade prop, 14.8V, afro 12A ESC
#define THRUST_MAP {{ 0.0   , 0.0 	}, \
					{ 0.1	, 0.154	}, \
					{ 0.2	, 0.328	}, \
					{ 0.3	, 0.578	}, \
					{ 0.4	, 0.896	}, \
					{ 0.5	, 1.282	}, \
					{ 0.6	, 1.773	}, \
					{ 0.7	, 2.284	}, \
					{ 0.8	, 2.727	}, \
					{ 0.9	, 3.180	}, \
					{ 1.0	, 3.498	}}

