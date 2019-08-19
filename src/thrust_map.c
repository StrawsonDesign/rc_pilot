/*
 * @file thrust_map.c
 *
 * Most ESC/motor/propeller combinations provide a highly non-linear map from
 * input to thrust. For the thrust table defined in thrust_map.h, this provides
 * the function to translate a desired normalized thrust (0-1) to the necessary
 * input (also 0-1).
 **/

#include <stdio.h>
#include <stdlib.h>

#include <thrust_map.h>

static double* signal;
static double* thrust;
static int points;

// clang-format off

// Generic linear mapping
static const int linear_map_points = 11;
static double linear_map[][2] = {
    {0.0, 0.0000}, 
    {0.1, 0.1000}, 
    {0.2, 0.2000}, 
    {0.3, 0.3000},
    {0.4, 0.4000}, 
    {0.5, 0.5000}, 
    {0.6, 0.6000}, 
    {0.7, 0.7000}, 
    {0.8, 0.8000}, 
    {0.9, 0.9000},
    {1.0, 1.0000}};

// Tiger Motor MN1806, 1400KV 6x4.5" 3-blade prop, 14.8V,
// BLheli ESC Low Timing
// this one is in Newtons but it doesn't really matter
static const int mn1806_1400kv_4s_points = 11;
static double mn1806_1400kv_4s_map[][2] = {
    {0.0, 0.0000}, 
    {0.1, 0.2982}, 
    {0.2, 0.6310},
    {0.3, 1.0281}, 
    {0.4, 1.5224}, 
    {0.5, 2.0310}, 
    {0.6, 2.5791}, 
    {0.7, 3.1365}, 
    {0.8, 3.7282},
    {0.9, 4.3147}, 
    {1.0, 4.7258}};

// tiger motor F20 2300kv motor, 2S lipo, 4x4.0" 3-blade props
// blheli esc med-low timing
// thrust units in gram-force but doesn't really matter
static const int f20_2300kv_2s_points = 21;
static double f20_2300kv_2s_map[][2] = {
    {0.00, 0.000000}, 
    {0.05, 6.892067}, 
    {0.10, 12.57954},
    {0.15, 18.84790}, 
    {0.20, 26.16294}, 
    {0.25, 33.98255}, 
    {0.30, 41.60790}, 
    {0.35, 49.32732},
    {0.40, 58.27048}, 
    {0.45, 67.83613}, 
    {0.50, 78.20817}, 
    {0.55, 88.27728}, 
    {0.60, 100.1058},
    {0.65, 110.3643}, 
    {0.70, 121.6316}, 
    {0.75, 132.2155}, 
    {0.80, 145.0420}, 
    {0.85, 154.6838},
    {0.90, 162.0185}, 
    {0.95, 168.4321}, 
    {1.00, 177.1643}};

/*
 * Lumenier RX2206-13 2000kv motor, 4S lipo, 5x45" lumenier prop
 * blheli esc high timing
 * for 5" monocoque hex
 */
static const int rx2206_4s_points = 12;
static double rx2206_4s_map[][2] = {
    {0.0, 0.00000000000000}, 
    {0.05, 17.8844719758775},
    {0.145, 44.8761484808831}, 
    {0.24, 80.0271164157384}, 
    {0.335, 122.556484678150},
    {0.43, 168.358712108506}, 
    {0.525, 220.433636910433}, 
    {0.62, 277.201919870206},
    {0.715, 339.008615108196}, 
    {0.81, 418.819295994349}, 
    {0.905, 505.430124336786},
    {1.0, 566.758535098236}};

/*
 * T-motor AIR 2213 920kv motor, 3S lipo, 8x4.5 MR prop
 * T-motor AIR 20A 600Hz esc
 */
static const int air2213_3s_points = 22;
static double air2213_3s_map[][2] = {{0.000, 0.0}, {0.127, 0.022556517697878},
    {0.168, 0.146930900746933}, {0.209, 0.290547149026484}, {0.250, 0.453283925181439},
    {0.292, 0.636337098755366}, {0.333, 0.821683249995852}, {0.374, 1.00901761511471},
    {0.415, 1.20659090143905}, {0.456, 1.39841671964909}, {0.497, 1.63818127442106},
    {0.539, 1.93095707576772}, {0.580, 2.26657347053131}, {0.621, 2.66038767610247},
    {0.662, 3.04036747880942}, {0.703, 3.48544790945891}, {0.744, 3.89440663076139},
    {0.785, 4.3262274318871}, {0.827, 4.76347857182283}, {0.868, 5.1475601251139},
    {0.909, 5.67761582028645}, {1.000, 5.68764923230496}};

// clang-format on

int thrust_map_init(thrust_map_t map)
{
    int i;
    double max;
    double(*data)[2];  // pointer to constant data

    switch (map)
    {
        case LINEAR_MAP:
            points = linear_map_points;
            data = linear_map;
            break;
        case MN1806_1400KV_4S:
            points = mn1806_1400kv_4s_points;
            data = mn1806_1400kv_4s_map;
            break;
        case F20_2300KV_2S:
            points = f20_2300kv_2s_points;
            data = f20_2300kv_2s_map;
            break;
        case RX2206_4S:
            points = rx2206_4s_points;
            data = rx2206_4s_map;
            break;
        case AIR2213_3S:
            points = air2213_3s_points;
            data = air2213_3s_map;
            break;
        default:
            fprintf(stderr, "ERROR: unknown thrust map\n");
            return -1;
    }

    // sanity checks
    if (points < 2)
    {
        fprintf(stderr, "ERROR: need at least 2 datapoints in THRUST_MAP\n");
        return -1;
    }
    if (data[0][0] != 0.0)
    {
        fprintf(stderr, "ERROR: first row input must be 0.0\n");
        return -1;
    }
    if (data[points - 1][0] != 1.0)
    {
        fprintf(stderr, "ERROR: last row input must be 1.0\n");
        printf("data: %f\n", data[points - 1][0]);
        return -1;
    }
    if (data[0][1] != 0.0)
    {
        fprintf(stderr, "ERROR: first row thrust must be 0.0\n");
        return -1;
    }
    if (data[points - 1][1] < 0.0)
    {
        fprintf(stderr, "ERROR: last row thrust must be > 0.0\n");
        return -1;
    }
    for (i = 1; i < points; i++)
    {
        if (data[i][0] <= data[i - 1][0] || data[i][1] <= data[i - 1][1])
        {
            fprintf(stderr, "ERROR: thrust_map must be monotonically increasing\n");
            return -1;
        }
    }

    // create new global array of normalized thrust and inputs
    if (signal != NULL) free(signal);
    if (thrust != NULL) free(thrust);
    signal = (double*)malloc(points * sizeof(double));
    thrust = (double*)malloc(points * sizeof(double));
    max = data[points - 1][1];
    for (i = 0; i < points; i++)
    {
        signal[i] = data[i][0];
        thrust[i] = data[i][1] / max;
    }
    return 0;
}

double map_motor_signal(double m)
{
    int i;
    double pos;

    // sanity check
    if (m > 1.0 || m < 0.0)
    {
        printf("ERROR: desired thrust t must be between 0.0 & 1.0\n");
        return -1;
    }

    // return quickly for boundary conditions
    if (m == 0.0 || m == 1.0) return m;

    // scan through the data to pick the upper and lower points to interpolate
    for (i = 1; i < points; i++)
    {
        if (m <= thrust[i])
        {
            pos = (m - thrust[i - 1]) / (thrust[i] - thrust[i - 1]);
            return signal[i - 1] + (pos * (signal[i] - signal[i - 1]));
        }
    }

    fprintf(stderr, "ERROR: something in map_motor_signal went wrong\n");
    return -1;
}
