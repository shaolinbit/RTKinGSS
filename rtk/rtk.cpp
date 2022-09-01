/*------------------------------------------------------------------------------
* rnx2rtkp.c : read rinex obs/nav files and compute receiver positions
*
*          Copyright (C) 2007-2009 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:55:16 $
* history : 2007/01/16  1.0 new
*           2007/03/15  1.1 add library mode
*           2007/05/08  1.2 separate from postpos.c
*           2009/01/20  1.3 support rtklib 2.2.0 api
*           2009/12/12  1.4 support glonass
*                           add option -h, -a, -l, -x
*           2010/01/28  1.5 add option -k
*           2010/08/12  1.6 add option -y implementation (2.4.0_p1)
*           2014/01/27  1.7 fix bug on default output time format
*-----------------------------------------------------------------------------*/
#include <stdarg.h>
#include "rtk.h"


static const char rcsid[] = "$Id: rnx2rtkp.c,v 1.1 2008/07/17 21:55:16 ttaka Exp $";

#define PROGNAME    "rnx2rtkp"          /* program name */
#define MAXFILE     8                   /* max number of input files */

/* help text -----------------------------------------------------------------*/
static const char* help[] = {
"",
" usage: rnx2rtkp [option]... file file [...]",
"",
" Read RINEX OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS message log files and ccompute ",
" receiver (rover) positions and output position solutions.",
" The first RINEX OBS file shall contain receiver (rover) observations. For the",
" relative mode, the second RINEX OBS file shall contain reference",
" (base station) receiver observations. At least one RINEX NAV/GNAV/HNAV",
" file shall be included in input files. To use SP3 precise ephemeris, specify",
" the path in the files. The extension of the SP3 file shall be .sp3 or .eph.",
" All of the input file paths can include wild-cards (*). To avoid command",
" line deployment of wild-cards, use \"...\" for paths with wild-cards.",
" Command line options are as follows ([]:default). With -k option, the",
" processing options are input from the configuration file. In this case,",
" command line options precede options in the configuration file.",
"",
" -?        print help",
" -k file   input options from configuration file [off]",
" -o file   set output file [stdout]",
" -ts ds ts start day/time (ds=y/m/d ts=h:m:s) [obs start time]",
" -te de te end day/time   (de=y/m/d te=h:m:s) [obs end time]",
" -ti tint  time interval (sec) [all]",
" -p mode   mode (0:single,1:dgps,2:kinematic,3:static,4:moving-base,",
"                 5:fixed,6:ppp-kinematic,7:ppp-static) [2]",
" -m mask   elevation mask angle (deg) [15]",
" -f freq   number of frequencies for relative mode (1:L1,2:L1+L2,3:L1+L2+L5) [2]",
" -v thres  validation threshold for integer ambiguity (0.0:no AR) [3.0]",
" -b        backward solutions [off]",
" -c        forward/backward combined solutions [off]",
" -i        instantaneous integer ambiguity resolution [off]",
" -h        fix and hold for integer ambiguity resolution [off]",
" -e        output x/y/z-ecef position [latitude/longitude/height]",
" -a        output e/n/u-baseline [latitude/longitude/height]",
" -n        output NMEA-0183 GGA sentence [off]",
" -g        output latitude/longitude in the form of ddd mm ss.ss' [ddd.ddd]",
" -t        output time in the form of yyyy/mm/dd hh:mm:ss.ss [sssss.ss]",
" -u        output time in utc [gpst]",
" -d col    number of decimals in time [3]",
" -s sep    field separator [' ']",
" -r x y z  reference (base) receiver ecef pos (m) [average of single pos]",
" -l lat lon hgt reference (base) receiver latitude/longitude/height (deg/m)",
" -y level  output soltion status (0:off,1:states,2:residuals) [0]",
" -x level  debug trace level (0:off) [0]"
};
/* show message --------------------------------------------------------------*/
extern int showmsg(char* format, ...)
{
    va_list arg;
    va_start(arg, format); vfprintf(stderr, format, arg); va_end(arg);
    fprintf(stderr, "\r");
    return 0;
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}


