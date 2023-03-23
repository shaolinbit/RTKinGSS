/*------------------------------------------------------------------------------
* postpos.c : post-processing positioning
*
*          Copyright (C) 2007-2016 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/05/08  1.0  new
*           2008/06/16  1.1  support binary inputs
*           2009/01/02  1.2  support new rtk positioing api
*           2009/09/03  1.3  fix bug on combined mode of moving-baseline
*           2009/12/04  1.4  fix bug on obs data buffer overflow
*           2010/07/26  1.5  support ppp-kinematic and ppp-static
*                            support multiple sessions
*                            support sbas positioning
*                            changed api:
*                                postpos()
*                            deleted api:
*                                postposopt()
*           2010/08/16  1.6  fix bug sbas message synchronization (2.4.0_p4)
*           2010/12/09  1.7  support qzss lex and ssr corrections
*           2011/02/07  1.8  fix bug on sbas navigation data conflict
*           2011/03/22  1.9  add function reading g_tec file
*           2011/08/20  1.10 fix bug on freez if solstatic=single and combined
*           2011/09/15  1.11 add function reading stec file
*           2012/02/01  1.12 support keyword expansion of rtcm ssr corrections
*           2013/03/11  1.13 add function reading otl and erp data
*           2014/06/29  1.14 fix problem on overflow of # of satellites
*           2015/03/23  1.15 fix bug on ant type replacement by rinex header
*                            fix bug on combined filter for moving-base mode
*           2015/04/29  1.16 fix bug on reading rtcm ssr corrections
*                            add function to read satellite fcb
*                            add function to read stec and troposphere file
*                            add keyword replacement in dcb, erp and ionos file
*           2015/11/13  1.17 add support of L5 antenna phase center parameters
*                            add *.stec and *.trp file for ppp correction
*           2015/11/26  1.18 support opt->freqopt(disable L2)
*           2016/01/12  1.19 add carrier-phase bias correction by ssr
*           2016/07/31  1.20 fix error message problem in rnx2rtkp
*           2016/08/29  1.21 suppress warnings
*           2016/10/10  1.22 fix bug on identification of file fopt->blq
*           2017/06/13  1.23 add smoother of velocity solution
*-----------------------------------------------------------------------------*/
#include "rtk.h"

#define MIN(x,y)    ((x)<(y)?(x):(y))
#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))

#define MAXPRCDAYS  100          /* max days of continuous processing */
#define MAXINFILE   1000         /* max number of input files */

/* constants/global variables ------------------------------------------------*/


static pcvs_t pcvss = { 0 };        /* receiver antenna parameters */
static pcvs_t pcvsr = { 0 };        /* satellite antenna parameters */
static obs_t obss = { 0 };          /* observation data */
static nav_t navs = { 0 };          /* navigation data */
static sbs_t sbss = { 0 };          /* sbas messages */
static lex_t lexs = { 0 };          /* lex messages */
static sta_t stas[MAXRCV];      /* station information */
static int nepoch = 0;            /* number of observation epochs */
static int nitm = 0;            /* number of invalid time marks */
static int iobsu = 0;            /* current rover observation data index */
static int iobsr = 0;            /* current reference observation data index */
static int isbs = 0;            /* current sbas message index */
static int ilex = 0;            /* current lex message index */
static int iitm = 0;            /* current invalid time mark index */
static int revs = 0;            /* analysis direction (0:forward,1:backward) */
static int aborts = 0;            /* abort status */
static sol_t* solf;             /* forward solutions */
static sol_t* solb;             /* backward solutions */
static double* rbf;             /* forward base positions */
static double* rbb;             /* backward base positions */
static int isolf = 0;             /* current forward solutions index */
static int isolb = 0;             /* current backward solutions index */
static char proc_rov[64] = "";   /* rover for current processing */
static char proc_base[64] = "";   /* base station for current processing */
static char rtcm_file[1024] = ""; /* rtcm data file */
static char rtcm_path[1024] = ""; /* rtcm data path */
static gtime_t invalidtm[100] = { {0} };/* invalid time marks */
static rtcm_t rtcm;             /* rtcm control struct */
static FILE* fp_rtcm = NULL;      /* rtcm data file pointer */


/* show message and check break ----------------------------------------------*/
extern int checkbrk(const char* format, ...)
{
    va_list arg;
    char buff[1024], * p = buff;
    if (!*format) return showmsg("");
    va_start(arg, format);
    p += vsprintf(p, format, arg);
    va_end(arg);
    if (*proc_rov && *proc_base) sprintf(p, " (%s-%s)", proc_rov, proc_base);
    else if (*proc_rov) sprintf(p, " (%s)", proc_rov);
    else if (*proc_base) sprintf(p, " (%s)", proc_base);
    return showmsg(buff);
}
/* output reference position -------------------------------------------------*/
static void outrpos(FILE* fp, const double* r, const solopt_t* opt)
{
    double pos[3], dms1[3], dms2[3];
    const char* sep = opt->sep;

    trace(3, "outrpos :\n");

    if (opt->posf == SOLF_LLH || opt->posf == SOLF_ENU) {
        ecef2pos(r, pos);
        if (opt->degf) {
            deg2dms(pos[0] * R2D, dms1, 5);
            deg2dms(pos[1] * R2D, dms2, 5);
            fprintf(fp, "%3.0f%s%02.0f%s%08.5f%s%4.0f%s%02.0f%s%08.5f%s%10.4f",
                dms1[0], sep, dms1[1], sep, dms1[2], sep, dms2[0], sep, dms2[1],
                sep, dms2[2], sep, pos[2]);
        }
        else {
            fprintf(fp, "%13.9f%s%14.9f%s%10.4f", pos[0] * R2D, sep, pos[1] * R2D,
                sep, pos[2]);
        }
    }
    else if (opt->posf == SOLF_XYZ) {
        fprintf(fp, "%14.4f%s%14.4f%s%14.4f", r[0], sep, r[1], sep, r[2]);
    }
}
/* output header -------------------------------------------------------------*/
static void outheader(FILE* fp, char** file, int n, const prcopt_t* popt,
    const solopt_t* sopt)
{
    const char* s1[] = { "GPST","UTC","JST" };
    gtime_t ts, te;
    double t1, t2;
    int i, j, w1, w2;
    char s2[32], s3[32];

    trace(3, "outheader: n=%d\n", n);

    if (sopt->posf == SOLF_NMEA || sopt->posf == SOLF_STAT) {
        return;
    }
    if (sopt->outhead) {
        if (!*sopt->prog) {
            fprintf(fp, "%s program   : RTKLIB ver.%s %s\n", COMMENTH, VER_RTKLIB, PATCH_LEVEL);
        }
        else {
            fprintf(fp, "%s program   : %s\n", COMMENTH, sopt->prog);
        }
        for (i = 0; i < n; i++) {
            fprintf(fp, "%s inp file  : %s\n", COMMENTH, file[i]);
        }
        for (i = 0; i < obss.n; i++)    if (obss.data[i].rcv == 1) break;
        for (j = obss.n - 1; j >= 0; j--) if (obss.data[j].rcv == 1) break;
        if (j < i) { fprintf(fp, "\n%s no rover obs data\n", COMMENTH); return; }
        ts = obss.data[i].time;
        te = obss.data[j].time;
        t1 = time2gpst(ts, &w1);
        t2 = time2gpst(te, &w2);
        if (sopt->times >= 1) ts = gpst2utc(ts);
        if (sopt->times >= 1) te = gpst2utc(te);
        if (sopt->times == 2) ts = timeadd(ts, 9 * 3600.0);
        if (sopt->times == 2) te = timeadd(te, 9 * 3600.0);
        time2str(ts, s2, 1);
        time2str(te, s3, 1);
        fprintf(fp, "%s obs start : %s %s (week%04d %8.1fs)\n", COMMENTH, s2, s1[sopt->times], w1, t1);
        fprintf(fp, "%s obs end   : %s %s (week%04d %8.1fs)\n", COMMENTH, s3, s1[sopt->times], w2, t2);
    }
    if (sopt->outopt) {
        outprcopt(fp, popt);
    }
    if (PMODE_DGPS <= popt->mode && popt->mode <= PMODE_FIXED && popt->mode != PMODE_MOVEB) {
        fprintf(fp, "%s ref pos   :", COMMENTH);
        outrpos(fp, popt->rb, sopt);
        fprintf(fp, "\n");
    }
    if (sopt->outhead || sopt->outopt) fprintf(fp, "%s\n", COMMENTH);

    outsolhead(fp, sopt);
}
/* search next observation data index ----------------------------------------*/
extern int nextobsf(const obs_t* obs, int* i, int rcv)
{
    double tt;
    int n=0;

    for (; *i < obs->n; (*i)++) {
            if (obs->data[*i].rcv == rcv)
            break;
    }
    for (n = 0; *i + n < obs->n; n++) {
        tt = timediff(obs->data[*i + n].time, obs->data[*i].time);
        if (obs->data[*i + n].rcv != rcv || tt > DTTOL) break;
    }
    return n;
}
extern int nextobsb(const obs_t* obs, int* i, int rcv)
{
    double tt;
    int n;

    for (; *i >= 0; (*i)--) if (obs->data[*i].rcv == rcv) break;
    for (n = 0; *i - n >= 0; n++) {
        tt = timediff(obs->data[*i - n].time, obs->data[*i].time);
        if (obs->data[*i - n].rcv != rcv || tt < -DTTOL) break;
    }
    return n;
}
/* update rtcm ssr correction ------------------------------------------------*/
extern void update_rtcm_ssr(gtime_t time)
{
    char path[1024];
    int i;

    /* open or swap rtcm file */
    reppath(rtcm_file, path, time, "", "");

    if (strcmp(path, rtcm_path)) {
        strcpy(rtcm_path, path);

        if (fp_rtcm) fclose(fp_rtcm);
        fp_rtcm = fopen(path, "rb");
        if (fp_rtcm) {
            rtcm.time = time;
            input_rtcm3f(&rtcm, fp_rtcm);
            trace(2, "rtcm file open: %s\n", path);
        }
    }
    if (!fp_rtcm) return;

    /* read rtcm file until current time */
    while (timediff(rtcm.time, time) < 1E-3) {
        if (input_rtcm3f(&rtcm, fp_rtcm) < -1) break;

        /* update ssr corrections */
        for (i = 0; i < MAXSAT; i++) {
            if (!rtcm.ssr[i].update ||
                rtcm.ssr[i].iod[0] != rtcm.ssr[i].iod[1] ||
                timediff(time, rtcm.ssr[i].t0[0]) < -1E-3) continue;
            navs.ssr[i] = rtcm.ssr[i];
            rtcm.ssr[i].update = 0;
        }
    }
}
/* input obs data, navigation messages and sbas correction -------------------*/
static int inputobs(obsd_t* obs, int solq, const prcopt_t* popt)
{
    gtime_t time = { 0 };
    int i, nu, nr, n = 0;

    trace(3, "\ninfunc  : revs=%d iobsu=%d iobsr=%d isbs=%d\n", revs, iobsu, iobsr, isbs);

    if (0 <= iobsu && iobsu < obss.n) {
        settime((time = obss.data[iobsu].time));
        if (checkbrk("processing : %s Q=%d", time_str(time, 0), solq)) {
            aborts = 1; showmsg("aborted"); return -1;
        }
    }
    if (!revs) { /* input forward data */
        if ((nu = nextobsf(&obss, &iobsu, 1)) <= 0) return -1;
        if (popt->intpref) {
            for (; (nr = nextobsf(&obss, &iobsr, 2)) > 0; iobsr += nr)
                if (timediff(obss.data[iobsr].time, obss.data[iobsu].time) > -DTTOL) break;
        }
        else {
            for (i = iobsr; (nr = nextobsf(&obss, &i, 2)) > 0; iobsr = i, i += nr)
                if (timediff(obss.data[i].time, obss.data[iobsu].time) > DTTOL) break;
        }
        nr = nextobsf(&obss, &iobsr, 2);
        if (nr <= 0) {
            nr = nextobsf(&obss, &iobsr, 2);
        }
        for (i = 0; i < nu && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsu + i];
        for (i = 0; i < nr && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsr + i];
        iobsu += nu;

        /* update sbas corrections */
        while (isbs < sbss.n) {
            time = gpst2time(sbss.msgs[isbs].week, sbss.msgs[isbs].tow);

            if (getbitu(sbss.msgs[isbs].msg, 8, 6) != 9) { /* except for geo nav */
                sbsupdatecorr(sbss.msgs + isbs, &navs);
            }
            if (timediff(time, obs[0].time) > -1.0 - DTTOL) break;
            isbs++;
        }
        /* update lex corrections */
        while (ilex < lexs.n) {
            if (lexupdatecorr(lexs.msgs + ilex, &navs, &time)) {
                if (timediff(time, obs[0].time) > -1.0 - DTTOL) break;
            }
            ilex++;
        }
        /* update rtcm ssr corrections */
        if (*rtcm_file) {
            update_rtcm_ssr(obs[0].time);
        }
    }
    else { /* input backward data */
        if ((nu = nextobsb(&obss, &iobsu, 1)) <= 0) return -1;
        if (popt->intpref) {
            for (; (nr = nextobsb(&obss, &iobsr, 2)) > 0; iobsr -= nr)
                if (timediff(obss.data[iobsr].time, obss.data[iobsu].time) < DTTOL) break;
        }
        else {
            for (i = iobsr; (nr = nextobsb(&obss, &i, 2)) > 0; iobsr = i, i -= nr)
                if (timediff(obss.data[i].time, obss.data[iobsu].time) < -DTTOL) break;
        }
        nr = nextobsb(&obss, &iobsr, 2);
        for (i = 0; i < nu && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsu - nu + 1 + i];
        for (i = 0; i < nr && n < MAXOBS * 2; i++) obs[n++] = obss.data[iobsr - nr + 1 + i];
        iobsu -= nu;

        /* update sbas corrections */
        while (isbs >= 0) {
            time = gpst2time(sbss.msgs[isbs].week, sbss.msgs[isbs].tow);

            if (getbitu(sbss.msgs[isbs].msg, 8, 6) != 9) { /* except for geo nav */
                sbsupdatecorr(sbss.msgs + isbs, &navs);
            }
            if (timediff(time, obs[0].time) < 1.0 + DTTOL) break;
            isbs--;
        }
        /* update lex corrections */
        while (ilex >= 0) {
            if (lexupdatecorr(lexs.msgs + ilex, &navs, &time)) {
                if (timediff(time, obs[0].time) < 1.0 + DTTOL) break;
            }
            ilex--;
        }
    }
    return n;
}
/* output to file message of invalid time mark -------------------------------*/
extern void outinvalidtm(FILE* fptm, const solopt_t* opt, const gtime_t tm)
{
    gtime_t time = tm;
    double gpst;
    int week, timeu;
    char s[100];

    timeu = opt->timeu < 0 ? 0 : (opt->timeu > 20 ? 20 : opt->timeu);

    if (opt->times >= TIMES_UTC) time = gpst2utc(time);
    if (opt->times == TIMES_JST) time = timeadd(time, 9 * 3600.0);

    if (opt->timef) time2str(time, s, timeu);
    else {
        gpst = time2gpst(time, &week);
        if (86400 * 7 - gpst < 0.5 / pow(10.0, timeu)) {
            week++;
            gpst = 0.0;
        }
        sprintf(s, "%4d   %*.*f", week, 6 + (timeu <= 0 ? 0 : timeu + 1), timeu, gpst);
    }
    strcat(s, "   Q=0, Time mark is not valid\n");

    fwrite(s, strlen(s), 1, fptm);
}
/* fill structure sol_t for time mark ----------------------------------------*/
extern sol_t fillsoltm(const sol_t solold, const sol_t solnew, const gtime_t tm)
{
    gtime_t t1 = { 0 }, t2 = { 0 };
    sol_t sol = solold;
    int i = 0;

    if (solold.stat == 0 || solnew.stat == 0) {
        sol.stat = 0;
    }
    else {
        sol.stat = (solold.stat > solnew.stat) ? solold.stat : solnew.stat;
    }
    sol.ns = (solold.ns < solnew.ns) ? solold.ns : solnew.ns;
    sol.ratio = (solold.ratio < solnew.ratio) ? solold.ratio : solnew.ratio;

    /* interpolation position and speed of time mark */
    t1 = solold.time;
    t2 = solnew.time;
    sol.time = tm;

    for (i = 0; i < 6; i++)
    {
        sol.rr[i] = solold.rr[i] + timediff(tm, t1) / timediff(t2, t1) * (solnew.rr[i] - solold.rr[i]);
    }

    return sol;
}
/* carrier-phase bias correction by fcb --------------------------------------*/
extern void corr_phase_bias_fcb(obsd_t* obs, int n, const nav_t* nav)
{
    int i, j, k;

    for (i = 0; i < nav->nf; i++) {
        if (timediff(nav->fcb[i].te, obs[0].time) < -1E-3) continue;
        if (timediff(nav->fcb[i].ts, obs[0].time) > 1E-3) break;
        for (j = 0; j < n; j++) {
            for (k = 0; k < NFREQ; k++) {
                if (obs[j].L[k] == 0.0) continue;
                obs[j].L[k] -= nav->fcb[i].bias[obs[j].sat - 1][k];
            }
        }
        return;
    }
}
/* carrier-phase bias correction by ssr --------------------------------------*/
extern void corr_phase_bias_ssr(obsd_t* obs, int n, const nav_t* nav)
{
    double lam;
    int i, j, code;

    for (i = 0; i < n; i++) for (j = 0; j < NFREQ; j++) {

        if (!(code = obs[i].code[j])) continue;
        if ((lam = nav->lam[obs[i].sat - 1][j]) == 0.0) continue;

        /* correct phase bias (cyc) */
        obs[i].L[j] -= nav->ssr[obs[i].sat - 1].pbias[code - 1] / lam;
    }
}

/* read prec ephemeris, sbas data, lex data, tec grid and open rtcm ----------*/
extern void readpreceph(char** infile, int n, const prcopt_t* prcopt,
    nav_t* nav, sbs_t* sbs, lex_t* lex)
{
    seph_t seph0 = { 0 };
    int i;
    char* ext;

    trace(2, "readpreceph: n=%d\n", n);

    nav->ne = nav->nemax = 0;
    nav->nc = nav->ncmax = 0;
    nav->nf = nav->nfmax = 0;
    sbs->n = sbs->nmax = 0;
    lex->n = lex->nmax = 0;

    /* read precise ephemeris files */
    for (i = 0; i < n; i++) {
        if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
        readsp3(infile[i], nav, 0);
    }
    /* read precise clock files */
    for (i = 0; i < n; i++) {
        if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
        readrnxc(infile[i], nav);
    }
    /* read satellite fcb files */
    for (i = 0; i < n; i++) {
        if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
        if ((ext = strrchr(infile[i], '.')) &&
            (!strcmp(ext, ".fcb") || !strcmp(ext, ".FCB"))) {
            readfcb(infile[i], nav);
        }
    }
    /* read solution status files for ppp correction */
    for (i = 0; i < n; i++) {
        if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
        if ((ext = strrchr(infile[i], '.')) &&
            (!strcmp(ext, ".stat") || !strcmp(ext, ".STAT") ||
                !strcmp(ext, ".stec") || !strcmp(ext, ".STEC") ||
                !strcmp(ext, ".trp") || !strcmp(ext, ".TRP"))) {
            pppcorr_read(&nav->pppcorr, infile[i]);
        }
    }
    /* read sbas message files */
    for (i = 0; i < n; i++) {
        if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
        sbsreadmsg(infile[i], prcopt->sbassatsel, sbs);
    }
    /* read lex message files */
    for (i = 0; i < n; i++) {
        if (strstr(infile[i], "%r") || strstr(infile[i], "%b")) continue;
        lexreadmsg(infile[i], 0, lex);
    }
    /* allocate sbas ephemeris */
    nav->ns = nav->nsmax = NSATSBS * 2;
    if (!(nav->seph = (seph_t*)malloc(sizeof(seph_t) * nav->ns))) {
        showmsg("error : sbas ephem memory allocation");
        trace(1, "error : sbas ephem memory allocation");
        return;
    }
    for (i = 0; i < nav->ns; i++) nav->seph[i] = seph0;

    /* set rtcm file and initialize rtcm struct */
    rtcm_file[0] = rtcm_path[0] = '\0'; fp_rtcm = NULL;

    for (i = 0; i < n; i++) {
        if ((ext = strrchr(infile[i], '.')) &&
            (!strcmp(ext, ".rtcm3") || !strcmp(ext, ".RTCM3"))) {
            strcpy(rtcm_file, infile[i]);
            init_rtcm(&rtcm);
            break;
        }
    }
}
/* free prec ephemeris and sbas data -----------------------------------------*/
extern void freepreceph(nav_t* nav, sbs_t* sbs, lex_t* lex)
{
    int i;

    trace(3, "freepreceph:\n");

    free(nav->peph); nav->peph = NULL; nav->ne = nav->nemax = 0;
    free(nav->pclk); nav->pclk = NULL; nav->nc = nav->ncmax = 0;
    free(nav->fcb); nav->fcb = NULL; nav->nf = nav->nfmax = 0;
    free(nav->seph); nav->seph = NULL; nav->ns = nav->nsmax = 0;
    free(sbs->msgs); sbs->msgs = NULL; sbs->n = sbs->nmax = 0;
    free(lex->msgs); lex->msgs = NULL; lex->n = lex->nmax = 0;
    for (i = 0; i < nav->nt; i++) {
        free(nav->tec[i].data);
        free(nav->tec[i].rms);
    }
    free(nav->tec); nav->tec = NULL; nav->nt = nav->ntmax = 0;

    if (fp_rtcm) fclose(fp_rtcm);
    free_rtcm(&rtcm);
}
/* read obs and nav data -----------------------------------------------------*/
static int readobsnav(gtime_t ts, gtime_t te, double ti, char** infile,
    const int* index, int n, const prcopt_t* prcopt,
    obs_t* obs, nav_t* nav, sta_t* sta)
{
    int i, j, ind = 0, nobs = 0, rcv = 1;

    trace(3, "readobsnav: ts=%s n=%d\n", time_str(ts, 0), n);

    obs->data = NULL; obs->n = obs->nmax = 0;
    nav->eph = NULL; nav->n = nav->nmax = 0;
    nav->geph = NULL; nav->ng = nav->ngmax = 0;
    nav->seph = NULL; nav->ns = nav->nsmax = 0;
    nepoch = 0;

    for (i = 0; i < n; i++) {
        if (checkbrk("")) return 0;

        if (index[i] != ind) {
            if (obs->n > nobs) rcv++;
            ind = index[i]; nobs = obs->n;
        }
        /* read rinex obs and nav file */
        if (readrnxt(infile[i], rcv, ts, te, ti, prcopt->rnxopt[rcv <= 1 ? 0 : 1], obs, nav,
            rcv <= 2 ? sta + rcv - 1 : NULL) < 0) {
            checkbrk("error : insufficient memory");
            trace(1, "insufficient memory\n");
            return 0;
        }
    }
    if (obs->n <= 0) {
        checkbrk("error : no obs data");
        trace(1, "\n");
        return 0;
    }
    if (nav->n <= 0 && nav->ng <= 0 && nav->ns <= 0) {
        checkbrk("error : no nav data");
        trace(1, "\n");
        return 0;
    }
    /* sort observation data */
    nepoch = sortobs(obs);

    /* delete duplicated ephemeris */
    uniqnav(nav);

    /* set time span for progress display */
    if (ts.time == 0 || te.time == 0) {
        for (i = 0; i < obs->n; i++) if (obs->data[i].rcv == 1) break;
        for (j = obs->n - 1; j >= 0; j--) if (obs->data[j].rcv == 1) break;
        if (i < j) {
            if (ts.time == 0) ts = obs->data[i].time;
            if (te.time == 0) te = obs->data[j].time;
            settspan(ts, te);
        }
    }
    return 1;
}
/* free obs and nav data -----------------------------------------------------*/
extern void freeobsnav(obs_t* obs, nav_t* nav)
{
    trace(3, "freeobsnav:\n");

    free(obs->data); obs->data = NULL; obs->n = obs->nmax = 0;
    free(nav->eph); nav->eph = NULL; nav->n = nav->nmax = 0;
    free(nav->geph); nav->geph = NULL; nav->ng = nav->ngmax = 0;
    free(nav->seph); nav->seph = NULL; nav->ns = nav->nsmax = 0;
}
/* average of single position ------------------------------------------------*/
static int avepos(double* ra, int rcv, const obs_t* obs, const nav_t* nav,
    const prcopt_t* opt)
{
    obsd_t data[MAXOBS];
    gtime_t ts = { 0 };
    sol_t sol = { {0} };
    int i, j, n = 0, m, iobs;
    char msg[128];
	double rr_pos[3];
    trace(3, "avepos: rcv=%d obs.n=%d\n", rcv, obs->n);

    for (i = 0; i < 3; i++) ra[i] = 0.0;

    for (iobs = 0; (m = nextobsf(obs, &iobs, rcv)) > 0; iobs += m) {

        for (i = j = 0; i < m && i < MAXOBS; i++) {
            data[j] = obs->data[iobs + i];
            if ((satsys(data[j].sat, NULL) & opt->navsys) &&
                opt->exsats[data[j].sat - 1] != 1) j++;
        }
        if (j <= 0 || !screent(data[0].time, ts, ts, 1.0)) continue; /* only 1 hz */

        if (!pntpos(data, j, nav, opt, &sol, NULL, NULL, msg)) continue;
		for (i = 0; i < 3; i++) {
			ra[i] += sol.rr[i];
		}
        n++;
    }
    if (n <= 0) {
        trace(1, "no average of base station position\n");
        return 0;
    }
    for (i = 0; i < 3; i++) ra[i] /= n;
	return 1;
}
/* station position from file ------------------------------------------------*/
static int getstapos(const char* file, char* name, double* r)
{
    FILE* fp;
    char buff[256], sname[256], * p, * q;
    double pos[3];

    trace(3, "getstapos: file=%s name=%s\n", file, name);

    if (!(fp = fopen(file, "r"))) {
        trace(1, "station position file open error: %s\n", file);
        return 0;
    }
    while (fgets(buff, sizeof(buff), fp)) {
        if ((p = strchr(buff, '%'))) *p = '\0';

        if (sscanf(buff, "%lf %lf %lf %s", pos, pos + 1, pos + 2, sname) < 4) continue;

        for (p = sname, q = name; *p && *q; p++, q++) {
            if (toupper((int)*p) != toupper((int)*q)) break;
        }
        if (!*p) {
            pos[0] *= D2R;
            pos[1] *= D2R;
            pos2ecef(pos, r);
            fclose(fp);
            return 1;
        }
    }
    fclose(fp);
    trace(1, "no station position: %s %s\n", name, file);
    return 0;
}
/* antenna phase center position ---------------------------------------------*/
extern int antpos(prcopt_t* opt, int rcvno, const obs_t* obs, const nav_t* nav,
    const sta_t* sta, const char* posfile)
{
    double* rr = rcvno == 1 ? opt->ru : opt->rb, del[3], pos[3], dr[3] = { 0 };
    int i, postype = rcvno == 1 ? opt->rovpos : opt->refpos;
    char* name;

    trace(3, "antpos  : rcvno=%d\n", rcvno);

    if (postype == POSOPT_SINGLE) { /* average of single position *//*replace with ppp*/
        if (!avepos(rr, rcvno, obs, nav, opt)) {//base station
            showmsg("error : station pos computation");
            return 0;
        }
	}
    else if (postype == POSOPT_FILE) { /* read from position file */
        name = stas[rcvno == 1 ? 0 : 1].name;
        if (!getstapos(posfile, name, rr)) {
            showmsg("error : no position of %s in %s", name, posfile);
            return 0;
        }
    }
    else if (postype == POSOPT_RINEX) { /* get from rinex header */
        if (norm(stas[rcvno == 1 ? 0 : 1].pos, 3) <= 0.0) {
            showmsg("error : no position in rinex header");
            trace(1, "no position in rinex header\n");
            return 0;
        }
        /* add antenna delta unless already done in antpcv() */
        if (!strcmp(opt->anttype[rcvno], "*")) {
            if (stas[rcvno == 1 ? 0 : 1].deltype == 0) { /* enu */
                for (i = 0; i < 3; i++) del[i] = stas[rcvno == 1 ? 0 : 1].del[i];
                del[2] += stas[rcvno == 1 ? 0 : 1].hgt;
                ecef2pos(stas[rcvno == 1 ? 0 : 1].pos, pos);
                enu2ecef(pos, del, dr);
            }
            else { /* xyz */
                for (i = 0; i < 3; i++) dr[i] = stas[rcvno == 1 ? 0 : 1].del[i];
            }
        }
        for (i = 0; i < 3; i++) rr[i] = stas[rcvno == 1 ? 0 : 1].pos[i] + dr[i];
    }
    return 1;
}
/* open procssing session ----------------------------------------------------*/
extern int openses(const prcopt_t* popt, const solopt_t* sopt,
    const filopt_t* fopt, nav_t* nav, pcvs_t* pcvs, pcvs_t* pcvr)
{
    int i;

    trace(3, "openses :\n");

    /* read satellite antenna parameters */
    if (*fopt->satantp && !(readpcv(fopt->satantp, pcvs))) {
        showmsg("error : no sat ant pcv in %s", fopt->satantp);
        trace(1, "sat antenna pcv read error: %s\n", fopt->satantp);
        return 0;
    }
    /* read receiver antenna parameters */
    if (*fopt->rcvantp && !(readpcv(fopt->rcvantp, pcvr))) {
        showmsg("error : no rec ant pcv in %s", fopt->rcvantp);
        trace(1, "rec antenna pcv read error: %s\n", fopt->rcvantp);
        return 0;
    }
    /* open geoid data */
    if (sopt->geoid > 0 && *fopt->geoid) {
        if (!opengeoid(sopt->geoid, fopt->geoid)) {
            showmsg("error : no geoid data %s", fopt->geoid);
            trace(2, "no geoid data %s\n", fopt->geoid);
        }
    }
    /* use satellite L2 offset if L5 offset does not exists */
    for (i = 0; i < pcvs->n; i++) {
        if (norm(pcvs->pcv[i].off[2], 3) > 0.0) continue;
        matcpy(pcvs->pcv[i].off[2], pcvs->pcv[i].off[1], 3, 1);
        matcpy(pcvs->pcv[i].var[2], pcvs->pcv[i].var[1], 19, 1);
    }
    for (i = 0; i < pcvr->n; i++) {
        if (norm(pcvr->pcv[i].off[2], 3) > 0.0) continue;
        matcpy(pcvr->pcv[i].off[2], pcvr->pcv[i].off[1], 3, 1);
        matcpy(pcvr->pcv[i].var[2], pcvr->pcv[i].var[1], 19, 1);
    }
    return 1;
}
/* close procssing session ---------------------------------------------------*/
extern void closeses(nav_t* nav, pcvs_t* pcvs, pcvs_t* pcvr)
{
    trace(3, "closeses:\n");

    /* free antenna parameters */
    free(pcvs->pcv); pcvs->pcv = NULL; pcvs->n = pcvs->nmax = 0;
    free(pcvr->pcv); pcvr->pcv = NULL; pcvr->n = pcvr->nmax = 0;

    /* close geoid data */
    closegeoid();

    /* free erp data */
    free(nav->erp.data); nav->erp.data = NULL; nav->erp.n = nav->erp.nmax = 0;

    /* close solution statistics and debug trace */
    rtkclosestat();
    traceclose();
}
/* set antenna parameters ----------------------------------------------------*/
extern void setpcv(gtime_t time, prcopt_t* popt, nav_t* nav, const pcvs_t* pcvs,
    const pcvs_t* pcvr, const sta_t* sta)
{
    pcv_t* pcv;
    double pos[3], del[3];
    int i, j, mode = PMODE_DGPS <= popt->mode && popt->mode <= PMODE_FIXED;
    char id[64];

    /* set satellite antenna parameters */
    for (i = 0; i < MAXSAT; i++) {
        if (!(satsys(i + 1, NULL) & popt->navsys)) continue;
        if (!(pcv = searchpcv(i + 1, "", time, pcvs))) {
            satno2id(i + 1, id);
            trace(3, "no satellite antenna pcv: %s\n", id);
            continue;
        }
        nav->pcvs[i] = *pcv;
    }
    for (i = 0; i < (mode ? 2 : 1); i++) {
        if (!strcmp(popt->anttype[i], "*")) { /* set by station parameters */
            strcpy(popt->anttype[i], sta[i].antdes);
            if (sta[i].deltype == 1) { /* xyz */
                if (norm(sta[i].pos, 3) > 0.0) {
                    ecef2pos(sta[i].pos, pos);
                    ecef2enu(pos, sta[i].del, del);
                    for (j = 0; j < 3; j++) popt->antdel[i][j] = del[j];
                }
            }
            else { /* enu */
                for (j = 0; j < 3; j++) popt->antdel[i][j] = stas[i].del[j];
            }
        }
        if (!(pcv = searchpcv(0, popt->anttype[i], time, pcvr))) {
            trace(2, "no receiver antenna pcv: %s\n", popt->anttype[i]);
            *popt->anttype[i] = '\0';
            continue;
        }
        strcpy(popt->anttype[i], pcv->type);
        popt->pcvr[i] = *pcv;
    }
}
/* read ocean tide loading parameters ----------------------------------------*/
extern void readotl(prcopt_t* popt, const char* file, const sta_t* sta)
{
    int i, mode = PMODE_DGPS <= popt->mode && popt->mode <= PMODE_FIXED;

    for (i = 0; i < (mode ? 2 : 1); i++) {
        readblq(file, sta[i].name, popt->odisp[i]);
    }
}
/* write header to output file -----------------------------------------------*/
extern int outhead(const char* outfile, char** infile, int n,
    const prcopt_t* popt, const solopt_t* sopt)
{
    FILE* fp = stdout;

    trace(3, "outhead: outfile=%s n=%d\n", outfile, n);

    if (*outfile) {
        createdir(outfile);

        if (!(fp = fopen(outfile, "w"))) {
            showmsg("error : open output file %s", outfile);
            return 0;
        }
    }
    /* output header */
    outheader(fp, infile, n, popt, sopt);

    if (*outfile) fclose(fp);

    return 1;
}
/* open output file for append -----------------------------------------------*/
extern FILE* openfile(const char* outfile)
{
    trace(3, "openfile: outfile=%s\n", outfile);

    return !*outfile ? stdout : fopen(outfile, "a");
}
/* Name time marks file ------------------------------------------------------*/
extern void namefiletm(char* outfiletm, const char* outfile)
{
    int i;

    for (i = strlen(outfile); i > 0; i--) {
        if (outfile[i] == '.') {
            break;
        }
    }
    /* if no file extension, then name time marks file as name of outfile + _events.pos */
    if (i == 0) {
        i = strlen(outfile);
    }
    strncpy(outfiletm, outfile, i);
    strcat(outfiletm, "_events.pos");
}
