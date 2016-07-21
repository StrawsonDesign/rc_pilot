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
#define THRUST_MAP {{ 0.0   , 0.0 		}, \
					{ 0.060	, 0.1226	}, \
					{ 0.104	, 0.2302	}, \
					{ 0.148	, 0.3392	}, \
					{ 0.186	, 0.4630	}, \
					{ 0.230	, 0.6105	}, \
					{ 0.261	, 0.7286	}, \
					{ 0.299	, 0.8856	}, \
					{ 0.336	, 1.0604	}, \
					{ 0.381	, 1.2790	}, \
					{ 0.437	, 1.6160	}, \
					{ 0.487	, 1.9339	}, \
					{ 0.550	, 2.3147	}, \
					{ 0.632	, 2.8009	}, \
					{ 0.714	, 3.2181	}, \
					{ 0.802	, 3.6542	}, \
					{ 0.896	, 4.1300	}, \
					{ 0.972	, 4.3612	}, \
					{ 1.000	, 4.4056	}}

