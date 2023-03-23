#pragma once
#include "../rtk/rtk.h"
class CPublic
{
public:
	static bool working;

    typedef struct {        /* time struct */
        time_t time;        /* time (s) expressed by standard time_t */
        double sec;         /* fraction of second under 1 s */
    } gtime_t;
    static gtime_t pro_time;
    typedef struct {        /* observation data record */
        gtime_t time;       /* receiver sampling time (GPST) */
        unsigned char sat, rcv; /* satellite/receiver number */
        unsigned char SNR[NFREQ + NEXOBS]; /* signal strength (0.25 dBHz) */
        unsigned char LLI[NFREQ + NEXOBS]; /* loss of lock indicator */
        unsigned char code[NFREQ + NEXOBS]; /* code indicator (CODE_???) */
        double L[NFREQ + NEXOBS]; /* observation data carrier-phase (cycle) */
        double P[NFREQ + NEXOBS]; /* observation data pseudorange (m) */
        float  D[NFREQ + NEXOBS]; /* observation data doppler frequency (Hz) */
    } obsd_t;
    typedef struct {        /* observation data */
        int n, nmax;         /* number of obervation data/allocated */
        obsd_t* data;       /* observation data records */
    } obs_t;
};

