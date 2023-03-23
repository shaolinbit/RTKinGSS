#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include "../rtk/rtk.h"
//#include "StrctDef.h"
#include <vector>
#include <algorithm>
#include "DataConvertion.h"
#include "RTK_Optimizer.h"
#include "Rtkpost_ndlg.h"
#ifdef __unix
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),  (mode)))==NULL
#endif
using namespace std;
extern gtime_t pro_time;
extern bool working;
string showtime;
string rtkout;
bool flag;
int arminfix;
int SYNCRTKIMUFlag;
int dayflag;


extern vector<fr_check*> fr_c;
extern vector<int> bias_flg;//双差模糊度组合信息（vflg）：基站卫星、流动站卫星、频率
extern vector<ambi_infor*> d_ambi;//双差模糊度的时间

vector<int> dd_ambi_num;//临时变量用于统计双差模糊度
int data_num = 0; long long t0 = 0;  int frc_index_s;
//set<int> ambi_index_30s;


#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define MAXPRCDAYS  100          /* max days of continuous processing */
#define MAXINFILE   1000         /* max number of input files */
#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))
static pcvs_t pcvss = { 0 };        /* receiver antenna parameters */
static pcvs_t pcvsr = { 0 };        /* satellite antenna parameters */
static obs_t obss = { 0 };          /* observation data */
static obs_t obss_b = { 0 };          /* observation data */
static nav_t navs = { 0 };          /* navigation data */
static sbs_t sbss = { 0 };          /* sbas messages */
static lex_t lexs = { 0 };          /* lex messages */
static sta_t stas[MAXRCV];      /* station information */
static int nepoch = 0;            /* number of observation epochs */
static int nitm = 0;            /* number of invalid time marks */
static int iobsu = 0;            /* current rover observation data index */
static int iobsu_b = 0;
static int iobsr = 0;            /* current reference observation data index */
static int iobsr_b = 0;
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
gtime_t lasttime = { 0 };
//UINT Rtkpost(LPVOID pParam);
double ch_daytime(double time);

typedef struct {
	int week;
	double gpst;
	double latitude;
	double longitude;
	double height;
	int Q;
	int ns;
	double sdn, sdu, sde, sdne, sdeu, sdun;
	double ratio, age;
	double fix;
}rtkdata;


int Rtkpost()
{

	flag = 0;
	working = true;

	char* infile[3], *outfile = (char*)"";
	char* resultfile = (char*)"";
	prcopt_t popt = prcopt_default;
	solopt_t sopt = solopt_default;
	filopt_t fopt = { "" };
	gtime_t ts = { 0 }, te = { 0 };
	double tint = 0.0;
	//load options
	resetsysopts();
	loadopts("../rtk/opt_gnss_mutifreq.conf", sysopts);


	getsysopts(&popt, &sopt, &fopt);
	FILE *minfix_file = fopen("../rtk/arminfix.ini", "r");
	if (minfix_file != NULL) {
		int setting_fix;
		fscanf(minfix_file, "%d", &setting_fix);
		popt.minfix = setting_fix;
	}

	if (arminfix) {
		popt.minfix = arminfix;
	}

	//output file
	if(!flag_global)
	{
        GSS_slidwin_result_file="../data/Project 200602/gtsam_GSS_sildwin_result_elmask_15.txt";
	}
	else{
        GSS_global_result_file="../data/Project 200602/gtsam_GSS_global_result_elmask_15.txt";
        rtklib_ekf_result_file="../data/Project 200602/gtsam_rtklib_ekf_result_elmask_15.txt";
	}

	string rover_path, base_path, nav_path, save_text, ggaout;//gga_path,
	bool f_find;

	string strPath;

    save_text = "../data/result.txt";
    rover_path = "../data/Project 200602/rover/022220GNSS.20O";
    base_path = "../data/Project 200602/base/022220.20o";
    nav_path = "../data/Project 200602/base/022220.20n";


	outfile = (char*)save_text.c_str();

	infile[0] = (char*)rover_path.c_str();
	infile[1] = (char*)base_path.c_str();
	infile[2] = (char*)nav_path.c_str();

	int ret, ret1;
	ret = postpos(ts, te, tint, 0.0, &popt, &sopt, &fopt, infile, 3, outfile, "", "");
	dayflag = 0;


	return 1;
}



int postpos(gtime_t ts, gtime_t te, double ti, double tu,
	const prcopt_t* popt, const solopt_t* sopt,
	const filopt_t* fopt, char** infile, int n, char* outfile,
	const char* rov, const char* base)
{
	int i, stat = 0, index[MAXINFILE] = { 0 };

	trace(3, "postpos : ti=%.0f tu=%.0f n=%d outfile=%s\n", ti, tu, n, outfile);

	/* open processing session */
	if (!openses(popt, sopt, fopt, &navs, &pcvss, &pcvsr)) return -1;

	for (i = 0; i < n; i++) index[i] = i;

	/* execute processing session */
	stat = execses_b(ts, te, ti, popt, sopt, fopt, 1, infile, index, n, outfile, rov,
		base);

	/* close processing session */
	closeses(&navs, &pcvss, &pcvsr);
    //cout<<" postpos() end.  "<<endl;
	return stat;
}

/* execute processing session for each base station --------------------------*/
int execses_b(gtime_t ts, gtime_t te, double ti, const prcopt_t* popt,
	const solopt_t* sopt, const filopt_t* fopt, int flag,
	char** infile, const int* index, int n, char* outfile,
	const char* rov, const char* base)
{
	int stat = 0;

	trace(3, "execses_b: n=%d outfile=%s\n", n, outfile);

	/* read prec ephemeris and sbas data */
	readpreceph(infile, n, popt, &navs, &sbss, &lexs);

	stat = execses_r(ts, te, ti, popt, sopt, fopt, flag, infile, index, n, outfile, rov);
	/* free prec ephemeris and sbas data */
	freepreceph(&navs, &sbss, &lexs);
    //cout<<" execses_b() end.  "<<endl;
	return stat;
}

/* execute processing session for each rover ---------------------------------*/
int execses_r(gtime_t ts, gtime_t te, double ti, const prcopt_t* popt,
	const solopt_t* sopt, const filopt_t* fopt, int flag,
	char** infile, const int* index, int n, char* outfile,
	const char* rov)
{
	int stat = 0;

	trace(3, "execses_r: n=%d outfile=%s\n", n, outfile);

	stat = execses(ts, te, ti, popt, sopt, fopt, flag, infile, index, n, outfile);
	//cout<<" execses_r() end.  "<<endl;
	return stat;
}

/* execute processing session ------------------------------------------------*/
int execses(gtime_t ts, gtime_t te, double ti, const prcopt_t* popt,
	const solopt_t* sopt, const filopt_t* fopt, int flag,
	char** infile, const int* index, int n, char* outfile)
{
	FILE* fp, *fptm;// *file;
	rtk_t rtk;
	prcopt_t popt_ = *popt;
	solopt_t tmsopt = *sopt;
	char tracefile[1024], statfile[1024], path[1024], *ext, outfiletm[1024] = { 0 };

	//cout<<"execses() begin\n";
	trace(3, "execses : n=%d outfile=%s\n", n, outfile);

	/* read obs and nav data */
	if (!readobsnav(ts, te, ti, infile, index, n, &popt_, &obss, &obss_b, &navs, stas)) {
		/* free obs and nav data */
		freeobsnav(&obss, &navs);
		return 0;
	}
    //cout<<"1 execses() begin\n";
	/* set antenna parameters */
	if (popt_.mode != PMODE_SINGLE) {
		setpcv(obss.n > 0 ? obss.data[0].time : timeget(), &popt_, &navs, &pcvss, &pcvsr,
			stas);
	}
    //cout<<"2 execses() begin\n";
	/* rover/reference fixed position */
	if (popt_.mode == PMODE_FIXED) {
		if (!antpos_(&popt_, 1, &obss, &navs, stas, fopt->stapos)) {
			freeobsnav(&obss, &navs);
			return 0;
		}
		if (!antpos_(&popt_, 2, &obss, &navs, stas, fopt->stapos)) {
			freeobsnav(&obss, &navs);
			return 0;
		}
	}
	else if (PMODE_DGPS <= popt_.mode && popt_.mode <= PMODE_STATIC_START) {
		if (!antpos_(&popt_, 2, &obss, &navs, stas, fopt->stapos)) { // base station
			freeobsnav(&obss, &navs);
			return 0;
		}
	}
	//cout<<"3 execses() begin\n";
	/* open solution statistics */
	if (flag && sopt->sstat > 0) {
		strcpy(statfile, outfile);
		strcat(statfile, ".stat");
		rtkclosestat();
		rtkopenstat(statfile, sopt->sstat);
	}
	//cout<<"4 execses() begin\n";
	/* write header to output file */
	if (flag && !outhead(outfile, infile, n, &popt_, sopt)) {
		freeobsnav(&obss, &navs);
		return 0;
	}

	/* name time events file */
	namefiletm(outfiletm, outfile);
	/* write header to file with time marks */
	outhead(outfiletm, infile, n, &popt_, &tmsopt);

	iobsu = iobsr = isbs = ilex = revs = aborts = 0;

	if (popt_.mode == PMODE_SINGLE || popt_.soltype == 0) {
		if ((fp = openfile(outfile)) && (fptm = openfile(outfiletm))) {
			procpos(fp, fptm, &popt_, sopt, &rtk, 0); /* forward */
			fclose(fp);
			fclose(fptm);
		}
	}
	else if (popt_.soltype == 1) {
		if ((fp = openfile(outfile)) && (fptm = openfile(outfiletm))) {
			revs = 1; iobsu = iobsr = obss.n - 1; isbs = sbss.n - 1; ilex = lexs.n - 1;
			procpos(fp, fptm, &popt_, sopt, &rtk, 0); /* backward */
			fclose(fp);
			fclose(fptm);
		}
	}
	else { /* combined */
		solf = (sol_t*)malloc(sizeof(sol_t) * nepoch);
		solb = (sol_t*)malloc(sizeof(sol_t) * nepoch);
		rbf = (double*)malloc(sizeof(double) * nepoch * 3);
		rbb = (double*)malloc(sizeof(double) * nepoch * 3);

		if (solf && solb) {
			isolf = isolb = 0;

            //cout<<"2 execses() begin\n";
			//procpos(fp_rtk, fptm_rtk, &popt_, sopt, &rtk, 1); /* forward */
			procpos(NULL, NULL, &popt_, sopt, &rtk, 1); /* forward */



			//revs = 1; iobsu = iobsr = obss.n - 1; isbs = sbss.n - 1; ilex = lexs.n - 1;
			//procpos(NULL, NULL, &popt_, sopt, &rtk, 1); /* backward */
			///* combine forward/backward solutions */
			//if (!aborts && (fp = openfile(outfile)) && (fptm = openfile(outfiletm))) {
			//	combres(fp, fptm, &popt_, sopt);
			//	fclose(fp);
			//	fclose(fptm);
			//}

			vector<double> baseline;
            //cout<<"0 execses() end.  "<<endl;
			/* output RTK result to file */
			if(flag_global){

                FILE* fp = fopen((char*)rtklib_ekf_result_file.c_str(), "w");
                double ref_pos[3];
                ecef2pos(rtk.rb, ref_pos);
                fprintf(fp, "%refpos : %.10f\t%.10f\t%f\n", ref_pos[0] * Rad_Deg, ref_pos[1] * Rad_Deg, ref_pos[2]);
                fprintf(fp,"week\tsecond(s)\tlat(deg)\tlon(deg)\th(m)\tstat\tsat_ns\tcov_x\tcov_y\tcov_z\tvx\tvy\tvz\n");
                for (int i = 0; i < fr_c.size(); i++)//fr_c.size()
                {
                //week second(s) lat(deg) lon(deg) h(m) stat sat_ns cov_x cov_y cov_z vx vy vz
                    //fprintf(fp, "%d\t", fr_c.at(i)->epoch_num);
                    if (fr_c.at(i)->stat <= 0)
                        continue;

                    int week;
                    double sec;
                    sec = time2gpst(fr_c.at(i)->time, &week);
                    fprintf(fp, "%d\t%f\t", week, sec);


                    double pos[3], blh[3];
                    for (int j = 0; j < 3; j++)
                    {
                        pos[j] = fr_c.at(i)->xf[j];
                    }
                    ecef2pos(pos, blh);

                    for (int j = 0; j < 3; j++)
                    {
                        fprintf(fp, "%.10f\t", blh[j]);
                    }
                    fprintf(fp, "%d\t", fr_c.at(i)->stat);
                    fprintf(fp, "%d\t", fr_c.at(i)->sat_ns);
                    fprintf(fp, "%f\t%f\t%f\t", fr_c[i]->Pf[0], fr_c[i]->Pf[1 * 9 + 1], fr_c[i]->Pf[2 * 9 + +2]);
                    fprintf(fp, "%f\t%f\t%f\t", fr_c[i]->xf[3], fr_c[i]->xf[4], fr_c[i]->xf[5]);
                    fprintf(fp, "\n");
                }
                fclose(fp);
            }
			/* GSSM optimizer global */
			if(flag_global)
                gtsam::RTKO_GenerateTrajectory_dd_section();


			/*free memory*/
			//cout<<"1 execses() end.  "<<endl;
			dd_ambi_num.clear();
            for (int i = 0; i < fr_c.size(); i++)
			{
				delete(fr_c[i]);
			}
			fr_c.clear();
			bias_flg.clear();
			for (int i = 0; i < d_ambi.size(); i++)
			{
				delete(d_ambi[i]);
			}
			d_ambi.clear();
            for (int i = 0; i < data_win.size(); i++)
			{
				delete(data_win[i]);
				data_win[i] = NULL;
			}
			data_win.clear();
            baseline.clear();
			key_pos.clear();
		}
		else showmsg("error : memory allocation");
		free(solf);
		free(solb);
		free(rbf);
		free(rbb);
		rtkfree(&rtk);
		//cout<<"2 execses() end.  "<<endl;
	}
	/* free obs and nav data */
	freeobsnav(&obss, &navs);
	return aborts ? 1 : 0;
}

int antpos_(prcopt_t* opt, int rcvno, const obs_t* obs, const nav_t* nav,
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

int avepos(double* ra, int rcv, const obs_t* obs, const nav_t* nav,
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

int getstapos(const char* file, char* name, double* r)
{
	FILE* fp;
	char buff[256], sname[256], *p, *q;
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

double get_daytime_(gtime_t tinput)
{
	gtime_t timegps = gpst2utc(tinput);
	int gweek;
	double tow = time2gpst(timegps, &gweek);
	int daytime = 86400;
	double result = tow - (int)(tow / daytime) * 86400.0;
	return result;
}
/* process positioning -------------------------------------------------------*/
void procpos(FILE* fp, FILE* fptm, const prcopt_t* popt, const solopt_t* sopt,
	rtk_t* rtk, int mode)
{
	gtime_t time = { 0 };
	sol_t sol = { {0} }, oldsol = { {0} }, newsol = { {0} };
	obsd_t obs[MAXOBS * 2]; /* for rover and base */
	double rb[3] = { 0 };
	int i, nobs, n, solstatic, num = 0, pri[] = { 0,1,2,3,4,5,1,6 };

	//cout<<"procpos() begin\n";
	trace(3, "procpos : mode=%d\n", mode);

	solstatic = sopt->solstatic &&
		(popt->mode == PMODE_STATIC || popt->mode == PMODE_STATIC_START || popt->mode == PMODE_PPP_STATIC);

	/* initialize unless running backwards on a combined run with continuous AR in which case keep the current states */
	if (mode == 0 || !revs || popt->modear == ARMODE_FIXHOLD)
		rtkinit(rtk, popt);

	rtcm_path[0] = '\0';
	//GSSM output file
	if(!flag_global){
        FILE *f_out = fopen((char*)GSS_slidwin_result_file.c_str(), "w");
        fprintf(f_out,"week\tsecond(s)\tlat(deg)\tlon(deg)\th(m)\tstat\tnfix\tns\tnv\tmove_flag\tv_x(m/s)\tv_y(m/s)\tv_z(m/s)\tax(m/s^2)\tay(m/s^2)\taz(m/s^2)\toutputflag\n");

        fclose(f_out);
    }
	while ((nobs = inputobs(obs, rtk->sol.stat, popt, 1)) >= 0) {
		/* exclude satellites */
		for (i = n = 0; i < nobs; i++) {
			if ((satsys(obs[i].sat, NULL) & popt->navsys) &&
				popt->exsats[obs[i].sat - 1] != 1) obs[n++] = obs[i];
		}
		if (n <= 0) continue;

		/* carrier-phase bias correction */
		if (navs.nf > 0) {
			corr_phase_bias_fcb(obs, n, &navs);
		}
		else if (!strstr(popt->pppopt, "-DIS_FCB")) {
			corr_phase_bias_ssr(obs, n, &navs);
		}
		/* disable L2 */
#if 0
		if (popt->freqopt == 1) {
			for (i = 0; i < n; i++) obs[i].L[1] = obs[i].P[1] = 0.0;
		}
#endif
		int flg = 0;
		if (!(flg = rtkpos(rtk, obs, n, &navs))) {
			if (rtk->sol.eventime.time != 0) {
				if (mode == 0) {
					outinvalidtm(fptm, sopt, rtk->sol.eventime);
				}
				else if (!revs) {
					invalidtm[nitm++] = rtk->sol.eventime;
				}
			}
			continue;
		}

		if (mode == 0) { /* forward/backward */
			if (!solstatic) {
				outsol(fp, &rtk->sol, rtk->rb, sopt);
			}
			else if (time.time == 0 || pri[rtk->sol.stat] <= pri[sol.stat]) {
				sol = rtk->sol;
				for (i = 0; i < 3; i++) rb[i] = rtk->rb[i];
				if (time.time == 0 || timediff(rtk->sol.time, time) < 0.0) {
					time = rtk->sol.time;
				}
			}
			/* check time mark */
			if (rtk->sol.eventime.time != 0)
			{
				newsol = fillsoltm(oldsol, rtk->sol, rtk->sol.eventime);
				num++;
				if (!solstatic && mode == 0) {
					outsol(fptm, &newsol, rb, sopt);
				}
			}
			oldsol = rtk->sol;
		}
		else if (!revs) { /* combined-forward */
			if (isolf >= nepoch) return;
			solf[isolf] = rtk->sol;
			for (i = 0; i < 3; i++) rbf[i + isolf * 3] = rtk->rb[i];
			isolf++;

		}
		else { /* combined-backward */
			if (isolb >= nepoch) return;
			solb[isolb] = rtk->sol;
			for (i = 0; i < 3; i++) rbb[i + isolb * 3] = rtk->rb[i];
			isolb++;
		}
	}
	if (mode == 0 && solstatic && time.time != 0.0) {
		sol.time = time;
		outsol(fp, &sol, rb, sopt);
	}
}

int inputobs(obsd_t* obs, int solq, const prcopt_t* popt, int br)
{
	char* show_time;
	gtime_t time = { 0 };

	double showtt;
	int i, nu, nr, n = 0;

	trace(3, "\ninfunc  : revs=%d iobsu=%d iobsr=%d isbs=%d\n", revs, iobsu, iobsr, isbs);
	if (br) {
		if (0 <= iobsu && iobsu < obss.n) {
			settime((time = obss.data[iobsu].time));
			showtt = timediff(time, lasttime);
			if (fabs(showtt) >= 100) {
				lasttime = time;
				show_time = time_str(time, 0);
				//showtime = (LPTSTR)(LPCTSTR)show_time;
			}
			if (checkbrk("processing : %s Q=%d", time_str(time, 0), solq) || !working) {
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
	else {
		if (0 <= iobsu_b && iobsu_b < obss_b.n) {
			if (checkbrk("processing : %s Q=%d", time_str(time, 0), solq) || !working) {
				aborts = 1; showmsg("aborted"); return -1;
			}
		}
		if (!revs) { /* input forward data */
			if ((nu = nextobsf(&obss_b, &iobsu_b, 2)) <= 0) return -1;
			if (popt->intpref) {
				for (; (nr = nextobsf(&obss_b, &iobsr_b, 2)) > 0; iobsr_b += nr)
					if (timediff(obss_b.data[iobsr_b].time, obss_b.data[iobsu_b].time) > -DTTOL) break;
			}
			else {
				for (i = iobsr_b; (nr = nextobsf(&obss_b, &i, 2)) > 0; iobsr_b = i, i += nr)
					if (timediff(obss_b.data[i].time, obss_b.data[iobsu_b].time) > DTTOL) break;
			}
			nr = nextobsf(&obss_b, &iobsr_b, 2);
			if (nr <= 0) {
				nr = nextobsf(&obss_b, &iobsr_b, 2);
			}
			for (i = 0; i < nu && n < MAXOBS * 2; i++) obs[n++] = obss_b.data[iobsu_b + i];
			for (i = 0; i < nr && n < MAXOBS * 2; i++) obs[n++] = obss_b.data[iobsr_b + i];
			iobsu_b += nu;

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
		return n;
	}
}

void combres(FILE* fp, FILE* fptm, const prcopt_t* popt, const solopt_t* sopt)
{
	gtime_t time = { 0 };
	sol_t sols = { {0} }, sol = { {0} }, oldsol = { {0} }, newsol = { {0} };
	double tt, Qf[9], Qb[9], Qs[9], rbs[3] = { 0 }, rb[3] = { 0 }, rr_f[3], rr_b[3], rr_s[3];
	int i, j, k, solstatic, num = 0, pri[] = { 0,1,2,3,4,5,1,6 };

	trace(3, "combres : isolf=%d isolb=%d\n", isolf, isolb);

	solstatic = sopt->solstatic &&
		(popt->mode == PMODE_STATIC || popt->mode == PMODE_STATIC_START || popt->mode == PMODE_PPP_STATIC);

	for (i = 0, j = isolb - 1; i < isolf && j >= 0; i++, j--) {

		if ((tt = timediff(solf[i].time, solb[j].time)) < -DTTOL) {
			sols = solf[i];
			for (k = 0; k < 3; k++) rbs[k] = rbf[k + i * 3];
			j++;

		}
		else if (tt > DTTOL) {
			sols = solb[j];
			for (k = 0; k < 3; k++) rbs[k] = rbb[k + j * 3];
			i--;

		}
		else if (solf[i].stat < solb[j].stat) {
			sols = solf[i];
			for (k = 0; k < 3; k++) rbs[k] = rbf[k + i * 3];
			if (sols.fix != 7 && solf[i].fix != 7 && solb[j].fix != 7)
			{
				sols.fix = 1;
			}
		}
		else if (solf[i].stat > solb[j].stat) {
			sols = solb[j];
			for (k = 0; k < 3; k++) rbs[k] = rbb[k + j * 3];
			if (sols.fix != 7 && solf[i].fix != 7 && solb[j].fix != 7)
			{
				sols.fix = 1;
			}
		}
		else {
			sols = solf[i];
			//sols.time = timeadd(sols.time, -tt / 2.0);
			sols.time = timeadd(sols.time, -tt / 2.0);
			if (solf[i].fix == 2 && solb[j].fix == 2 && sols.fix != 7) {
				sols.fix = 2;
			}
			else if (solf[i].fix != 7 && solb[j].fix != 7) {
				sols.fix = 1;
			}
			for (int t = 0; t < 6; t++) {
				sols.rr[t] = (solf[i].rr[t] + solb[j].rr[t]) / 2;
			}
			if ((popt->mode == PMODE_KINEMA || popt->mode == PMODE_MOVEB) &&
				sols.stat == SOLQ_FIX) {

				/* degrade fix to float if validation failed */
				if (!valcomb(solf + i, solb + j)) sols.stat = SOLQ_FLOAT;
			}
			for (k = 0; k < 3; k++) {
				Qf[k + k * 3] = solf[i].qr[k];
				Qb[k + k * 3] = solb[j].qr[k];
			}
			Qf[1] = Qf[3] = solf[i].qr[3];
			Qf[5] = Qf[7] = solf[i].qr[4];
			Qf[2] = Qf[6] = solf[i].qr[5];
			Qb[1] = Qb[3] = solb[j].qr[3];
			Qb[5] = Qb[7] = solb[j].qr[4];
			Qb[2] = Qb[6] = solb[j].qr[5];

			if (popt->mode == PMODE_MOVEB) {
				for (k = 0; k < 3; k++) rr_f[k] = solf[i].rr[k] - rbf[k + i * 3];
				for (k = 0; k < 3; k++) rr_b[k] = solb[j].rr[k] - rbb[k + j * 3];
				if (smoother(rr_f, Qf, rr_b, Qb, 3, rr_s, Qs)) continue;
				for (k = 0; k < 3; k++) sols.rr[k] = rbs[k] + rr_s[k];
			}
			else {
				//if (smoother(solf[i].rr, Qf, solb[j].rr, Qb, 3, sols.rr, Qs)) continue;
			}
			sols.qr[0] = (float)Qs[0];
			sols.qr[1] = (float)Qs[4];
			sols.qr[2] = (float)Qs[8];
			sols.qr[3] = (float)Qs[1];
			sols.qr[4] = (float)Qs[5];
			sols.qr[5] = (float)Qs[2];

			/* smoother for velocity solution */
			if (popt->dynamics) {
				for (k = 0; k < 3; k++) {
					Qf[k + k * 3] = solf[i].qv[k];
					Qb[k + k * 3] = solb[j].qv[k];
				}
				Qf[1] = Qf[3] = solf[i].qv[3];
				Qf[5] = Qf[7] = solf[i].qv[4];
				Qf[2] = Qf[6] = solf[i].qv[5];
				Qb[1] = Qb[3] = solb[j].qv[3];
				Qb[5] = Qb[7] = solb[j].qv[4];
				Qb[2] = Qb[6] = solb[j].qv[5];
				if (smoother(solf[i].rr + 3, Qf, solb[j].rr + 3, Qb, 3, sols.rr + 3, Qs)) continue;
				sols.qv[0] = (float)Qs[0];
				sols.qv[1] = (float)Qs[4];
				sols.qv[2] = (float)Qs[8];
				sols.qv[3] = (float)Qs[1];
				sols.qv[4] = (float)Qs[5];
				sols.qv[5] = (float)Qs[2];
			}
		}
		if (!solstatic) {
			outsol(fp, &sols, rbs, sopt);
		}
		else if (time.time == 0 || pri[sols.stat] <= pri[sol.stat]) {
			sol = sols;
			for (k = 0; k < 3; k++) rb[k] = rbs[k];
			if (time.time == 0 || timediff(sols.time, time) < 0.0) {
				time = sols.time;
			}
		}

		if (iitm < nitm && timediff(invalidtm[iitm], sols.time) < 0.0)
		{
			outinvalidtm(fptm, sopt, invalidtm[iitm]);
			iitm++;
		}
		if (sols.eventime.time != 0)
		{
			newsol = fillsoltm(oldsol, sols, sols.eventime);
			num++;
			if (!solstatic) {
				outsol(fptm, &newsol, rb, sopt);
			}
		}
		oldsol = sols;
	}
	if (solstatic && time.time != 0.0) {
		sol.time = time;
		outsol(fp, &sol, rb, sopt);
	}

}
int valcomb(const sol_t* solf, const sol_t* solb)
{
	double dr[3], var[3];
	int i;
	char tstr[32];

	trace(3, "valcomb :\n");

	/* compare forward and backward solution */
	for (i = 0; i < 3; i++) {
		dr[i] = solf->rr[i] - solb->rr[i];
		var[i] = solf->qr[i] + solb->qr[i];
	}
	for (i = 0; i < 3; i++) {
		if (dr[i] * dr[i] <= 16.0 * var[i]) continue; /* ok if in 4-sigma */

		time2str(solf->time, tstr, 2);
		trace(2, "degrade fix to float: %s dr=%.3f %.3f %.3f std=%.3f %.3f %.3f\n",
			tstr + 11, dr[0], dr[1], dr[2], SQRT(var[0]), SQRT(var[1]), SQRT(var[2]));
		return 0;
	}
	return 1;
}

int readobsnav(gtime_t ts, gtime_t te, double ti, char** infile,
	const int* index, int n, const prcopt_t* prcopt,
	obs_t* obs, obs_t *obs_b, nav_t* nav, sta_t* sta)
{
	int i, j, ind = 0, nobs = 0, rcv = 1;

	trace(3, "readobsnav: ts=%s n=%d\n", time_str(ts, 0), n);

	obs->data = NULL; obs->n = obs->nmax = 0; obs->rover_n = 0;
	nav->eph = NULL; nav->n = nav->nmax = 0;
	nav->geph = NULL; nav->ng = nav->ngmax = 0;
	//nav->seph = NULL; nav->ns = nav->nsmax = 0;
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
		else {
			if (i == 0) {
				obs->rover_n = obs->n;
			}
			if (i == 2) {
				obs_b->n = obs->n;
				obs_b->data = obs->data;
			}
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

int Read_GGA(char* ggafile, std::vector<DGPSStruct>& realtimeggavector) {
	FILE *fp1;
	int rc = 0;
	if (rc = fopen_s(&fp1, ggafile, "r+"))
	{
		return rc;
	}
	char sentence[512];

	while (!feof(fp1))
	{
		//if (!m_bWorking)
		//	break;
		if (!working) {
			return 0;
		}
		fgets(sentence, sizeof(sentence), fp1);
		switch (minmea_sentence_id(sentence, false))
		{
		case MINMEA_SENTENCE_GGA:
		{
			struct minmea_sentence_gga gga;
			if (minmea_parse_gga(&gga, sentence)) {
				if (((int)(minmea_tocoord(&gga.latitude)) == 0
					|| (int)(minmea_tocoord(&gga.longitude)) == 0))
					continue;

				DGPSStruct dgps;
				ConvertNMEAstruct(gga, dgps);
				realtimeggavector.push_back(dgps);
			}
			else {
			}
		}
		break;
		}
	}
	if (!working) {
		return 0;
	}
	realtimeggavector.pop_back();
	fclose(fp1);

	double lastsec = realtimeggavector.back().GPSTime;
	double firstsec = realtimeggavector.front().GPSTime;

	double gtemp = (lastsec - firstsec)*1000.0 / (realtimeggavector.size() - 1);
	double NewNum = floor(gtemp);
	int gtemp2 = ((gtemp - NewNum) - 0.5 <= 0) ? NewNum : NewNum + 1;
	int gtemp2mod10 = gtemp2 % 10;
	gtemp2 = gtemp2 - gtemp2mod10;

	//NaviData.GPS_FILE_TM_COUNT = gtemp2 / INS_UPDATE_TIME / 1000;
	//NaviData.FEEDBACK_TIMES_STEP = (double)(INSFrequency / NaviData.GPS_FILE_TM_COUNT) * INS_UPDATE_TIME;
	//NaviData.FEEDBACK_TIMES_STEP = INS_UPDATE_TIME;
	return 1;
}


int combineGPSGGA(char* gpsfile, string ggafile, string ggaout, char* resultfile) {
	std::vector<DGPSStruct> realtimeggavector;
	rtkdata last_rtkdata, tmpdata;
	rtkdata rtkdata;
	ifstream gpsf(gpsfile, ios::in);
	ifstream ggain(ggafile, ios::in | ios::binary);
	ofstream ggaout1(ggaout);
	ofstream relt(resultfile, ios::out);
	char line[256];
	DGPSStruct realtimegga;
	int ind = 0;
	double last_time = -1;
	int start_check = 0;
	int position, position1, position2, checkpos;
	while (!ggain.eof() && working) {
		position1 = -1;
		position2 = -1;
		checkpos = -1;
		string buffer;
		string tmp_s, tmp_left;
		ggain >> buffer;
		checkpos = buffer.find(",,");
		position = buffer.find(",");
		tmp_s = buffer.substr(0, position);
		if (position >= 0)
		{
			tmp_left = buffer.substr(position, buffer.size() - position);
			position1 = tmp_s.find("$GPGGA");
			if (position1 < 0) {
				position1 = tmp_left.find("$GPGGA");
				if (position1 >= 0) {
					position1 += position;
				}
			}
		}
		if (position1 >= 0 && checkpos != position1 + 6) {
			buffer.erase(0, position1 + 1);
			position2 = buffer.find("*");
			if (position2 >= 0) {
				ggaout1 << "$" << buffer.substr(0, position2 + 3) << endl;
			}
		}
	}
	ggain.close();
	ggaout1.close();

	Read_GGA((char*)ggaout.c_str(), realtimeggavector);
	int i = 0;
	while (i < 16) {
		gpsf.getline(line, 256);
		i++;
	}
	gpsf.getline(line, 256);
	sscanf(line, "%d %lf %lf %lf %lf %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &last_rtkdata.week, &last_rtkdata.gpst, &last_rtkdata.latitude, &last_rtkdata.longitude, &last_rtkdata.height, &last_rtkdata.Q, &last_rtkdata.ns, &last_rtkdata.sdn, &last_rtkdata.sde, &last_rtkdata.sdu, &last_rtkdata.sdne, &last_rtkdata.sdeu, &last_rtkdata.sdun, &last_rtkdata.age, &last_rtkdata.ratio, &last_rtkdata.fix);

	while (!gpsf.eof() && ind < realtimeggavector.size() && working) {
		double timed;
		double day;

		if (start_check != 5)
		{
			gpsf.getline(line, 256);
			sscanf(line, "%d %lf %lf %lf %lf %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &rtkdata.week, &rtkdata.gpst, &rtkdata.latitude, &rtkdata.longitude, &rtkdata.height, &rtkdata.Q, &rtkdata.ns, &rtkdata.sdn, &rtkdata.sde, &rtkdata.sdu, &rtkdata.sdne, &rtkdata.sdeu, &rtkdata.sdun, &rtkdata.age, &rtkdata.ratio, &rtkdata.fix);
			printf("11");
		}
		else {
			start_check = 6;//the check of first five lines ends
			tmpdata = rtkdata;
			rtkdata = last_rtkdata;
			last_rtkdata = tmpdata;
		}
		if (start_check < 5) {
			if (fabs(rtkdata.gpst - last_rtkdata.gpst - 1) < 0.1) {
				tmpdata = rtkdata;
				rtkdata = last_rtkdata;
				last_rtkdata = tmpdata;
			}
			start_check++;
		}

		day = (int)(rtkdata.gpst / 86400)*86400.000;
		timed = ch_daytime(rtkdata.gpst) - 18;


		while (true && ind < realtimeggavector.size()) {
			realtimegga = realtimeggavector[ind];
			if (last_time >= 0) {
				if ((fabs(timed - last_time) > 86000) || fabs(realtimegga.GPSTime - last_time > 86000)) {
					dayflag += 1;
				}
			}
			if (dayflag) {
				day -= dayflag * 86400;
				timed += dayflag * 86400;
				realtimegga.GPSTime += dayflag * 86400;
			}
			//last_time = timed;
			if (fabs(realtimegga.GPSTime - timed) > 0.1) {
				if (timed > realtimegga.GPSTime)
				{
					relt << setiosflags(ios::fixed) << setprecision(6) << realtimegga.GPSTime << " " << realtimegga.fix_q << " " << setprecision(8) << realtimegga.Longitude << " " << realtimegga.Latitude << " " << setprecision(4) << realtimegga.Altitude << " " << setprecision(6) << realtimegga.GPSTime << " 0 " << setprecision(8) << realtimegga.Longitude << " " << realtimegga.Latitude << " " << setprecision(4) << realtimegga.Altitude << " " << setprecision(3) << day << " " << day << endl;
					ind++;
				}
				else {
					relt << setiosflags(ios::fixed) << setprecision(6) << timed << " " << "0" << " " << setprecision(8) << rtkdata.longitude << " " << rtkdata.latitude << " " << setprecision(4) << rtkdata.height << " " << setprecision(6) << timed << " 0 " << setprecision(8) << rtkdata.longitude << " " << rtkdata.latitude << " " << setprecision(4) << rtkdata.height << " " << setprecision(3) << day << " " << day << endl;
					last_time = timed;
					break;
				}
			}
			else {
				if (rtkdata.fix == 1)
				{
					relt << setiosflags(ios::fixed) << setprecision(6) << realtimegga.GPSTime << " " << realtimegga.fix_q << " " << setprecision(8) << realtimegga.Longitude << " " << realtimegga.Latitude << " " << setprecision(4) << realtimegga.Altitude << " " << setprecision(6) << timed << " 2 " << setprecision(8) << rtkdata.longitude << " " << rtkdata.latitude << " " << setprecision(4) << rtkdata.height << " " << setprecision(3) << day << " " << day << endl;
				}
				else if (rtkdata.fix == 2) {
					relt << setiosflags(ios::fixed) << setprecision(6) << realtimegga.GPSTime << " " << realtimegga.fix_q << " " << setprecision(8) << realtimegga.Longitude << " " << realtimegga.Latitude << " " << setprecision(4) << realtimegga.Altitude << " " << setprecision(6) << timed << " 5 " << setprecision(8) << rtkdata.longitude << " " << rtkdata.latitude << " " << setprecision(4) << rtkdata.height << " " << setprecision(3) << day << " " << day << endl;
				}
				else if (rtkdata.fix == 7) {
					relt << setiosflags(ios::fixed) << setprecision(6) << realtimegga.GPSTime << " " << realtimegga.fix_q << " " << setprecision(8) << realtimegga.Longitude << " " << realtimegga.Latitude << " " << setprecision(4) << realtimegga.Altitude << " " << setprecision(6) << timed << " 7 " << setprecision(8) << rtkdata.longitude << " " << rtkdata.latitude << " " << setprecision(4) << rtkdata.height << " " << setprecision(3) << day << " " << day << endl;
				}
				ind++;
				last_time = timed;
				break;
			}
			last_time = timed;
		}
	}
	gpsf.close();
	relt.close();
	return 1;
}

double ch_daytime(double time) {
	int daytime = 86400;
	double result;
	result = time - (int)(time / 86400) * 86400;
	return result;
}


int savedata_fr_c(rtk_t *rtk, const prcopt_t* popt, const obsd_t* obs, const int nobs, const int isolf)
{

	//no SPP result
	if (norm(rtk->sol.rr, 3) < 10)
		return 0;

	//invaild relpos result
	if (rtk->sol.stat == SOLQ_NONE)//||rtk->nv<=0
	{
		if(!flag_global)
            return 0;
	}
	double var = 0.0;
	for (int i = 0; i < 3; i++)
	{
		var += rtk->P[i + i * rtk->nx];
	}
	var /= 3.0;
	//var_thresshold = ((var_thresshold*epoch_now) + var) / (double)(epoch_now + 1);
	if (var > (3 * var_thresshold) && epoch_now > 300 && var > (3 * 0.3))
	{
        if(!flag_global)
            return 0;
	}
	if (epoch_now > 300)
		var_thresshold = ((var_thresshold*(epoch_now - 300)) + var) / (double)((epoch_now - 300) + 1);

	//save forward relpos result===============================================================
	int i, j, k;
	fr_check* fr_cc = new fr_check;

	//time
	fr_cc->time = rtk->sol.time;
	fr_cc->eventime = rtk->sol.eventime;
	fr_cc->dt = rtk->dt;
	fr_cc->epoch_num = isolf;

    fr_cc->stat = rtk->sol.stat;
	//save information related to ambiguity,sat index,fix or not,cycle silp etc.
	for (i = 0; i < MAXSAT; i++) {
		fr_cc->ssat[i] = rtk->ssat[i];
	}
	fr_cc->sat_ns = rtk->ns;//common satellite num

	if (rtk->nfix == 0)
	{
		//pva
		for (i = 0; i < fr_cc->x_pva_num; i++)
		{
			fr_cc->xf[i] = rtk->x[i];
		}
		for (i = 0; i < fr_cc->x_pva_num; i++)
		{
			for (int j = 0; j < fr_cc->x_pva_num; j++)
				fr_cc->Pf[i * fr_cc->x_pva_num + j] = rtk->P[i + j * rtk->nx];
		}
	}
	else if (rtk->nfix != 0)
	{

		//pva
		for (i = 0; i < fr_cc->x_pva_num; i++)
		{
			fr_cc->xf[i] = rtk->xa_[i];
		}
		for (i = 0; i < fr_cc->x_pva_num; i++)
		{
			for (int j = 0; j < fr_cc->x_pva_num; j++)
				fr_cc->Pf[i * fr_cc->x_pva_num + j] = rtk->Pa[i + j * rtk->na];
		}

	}


	fr_cc->nn = rtk->nn;//base+rover sat number
	fr_cc->nr = rtk->nr;//base sat number
	fr_cc->nu = rtk->nu;//rover sat number
	for (i = 0; i < nobs; i++) {
		fr_cc->obs[i] = obs[i];
	}

	/*double diffrence ambiguity-----------------------------------------------------*/
	fr_cc->nv = rtk->nv;//vaild double diffrence ambiguity number

	for (i = 0; i < rtk->nv; i++)
	{
		if (((rtk->vflg[i] >> 4) & 0xF) == 1)
			continue;
		if (d_ambi.size() == 0)
		{
			init_ambi_infor_dd(rtk->vflg[i], rtk->dd_bias[i], rtk->P_dd_ambiguity[i], fr_cc->epoch_num, fr_cc->time);
			fr_cc->bias_index[i] = d_ambi.size() - 1;
		}
		else if (d_ambi.size() > 0)
		{
			int flg = rtk->vflg[i];
			int index = -1, flag = 0;
			vector<int>::iterator iter;

			do
			{
				iter = find(bias_flg.begin() + 1 + index, bias_flg.end(), flg);
				index = iter - bias_flg.begin();

				if (iter == bias_flg.end())
				{
					flag = 1;

					init_ambi_infor_dd(rtk->vflg[i], rtk->dd_bias[i], rtk->P_dd_ambiguity[i], fr_cc->epoch_num, fr_cc->time);
					fr_cc->bias_index[i] = d_ambi.size() - 1;
					break;
				}
			} while (d_ambi.at(index)->epoch_e != 0);
			if (flag == 0)
			{
				int ref_sat, u_ref_sat, f;//ref sat index, non ref sat index, freq
				ref_sat = (rtk->vflg[i] >> 16) & 0xFF;
				u_ref_sat = (rtk->vflg[i] >> 8) & 0xFF;
				f = rtk->vflg[i] & 0xF;
				if ((rtk->ssat[ref_sat - 1].slip[f] & 1) || (rtk->ssat[u_ref_sat - 1].slip[f] & 1)
					|| (rtk->ssat[ref_sat - 1].rejc[f] > 2) || (rtk->ssat[u_ref_sat - 1].rejc[f] > 2))
				{
					d_ambi.at(index)->epoch_e = -1;
					init_ambi_infor_dd(rtk->vflg[i], rtk->dd_bias[i], rtk->P_dd_ambiguity[i], fr_cc->epoch_num, fr_cc->time);
					fr_cc->bias_index[i] = d_ambi.size() - 1;
				}
				else
				{
					if ((fr_cc->epoch_num > (d_ambi.at(index)->epoch_s + d_ambi.at(index)->num))||
						fabs(rtk->dd_bias[i] - d_ambi.at(index)->bias) > 5)
					{
						d_ambi.at(index)->epoch_e = -1;
						init_ambi_infor_dd(rtk->vflg[i], rtk->dd_bias[i], rtk->P_dd_ambiguity[i], fr_cc->epoch_num, fr_cc->time);
						fr_cc->bias_index[i] = d_ambi.size() - 1;
					}
					else
					{
						fr_cc->bias_index[i] = index;
						bias_mean(d_ambi.at(index), rtk->dd_bias[i], rtk->P_dd_ambiguity[i]);
						if ((fr_cc->epoch_num > (d_ambi.at(index)->epoch_s + d_ambi.at(index)->num-1)))
							d_ambi.at(index)->num = fr_cc->epoch_num - d_ambi.at(index)->epoch_s+1;
					}

				}
			}
		}
	}


	fr_c.push_back(fr_cc);
	epoch_now++;
	return 1;
}


void init_ambi_infor_dd(int flg, double bias, double q, int epoch_s, gtime_t t_start, int epoch_e)
{
	ambi_infor * bias_infor = new ambi_infor;

	key_ambi_global++;
	bias_infor->key = key_ambi_global;

	//int flg = (prn_ref << 16) | (prn_i << 8) | (0 << 4) | (f);

	//int flg = (0 << 16) | (prn << 8) | (0 << 4) | (f);
	bias_infor->bias_flag = flg;
	if (flg == 724992)
	{
		double bb = 0;
	}
	//bias_infor->slipcount = slipcount;

	bias_infor->bias = bias;
	bias_infor->q = q;
	bias_infor->epoch_s = epoch_s;
	bias_infor->t_start = t_start;

	bias_infor->epoch_e = epoch_e;

	bias_infor->num = 1;

	d_ambi.push_back(bias_infor);

	bias_flg.push_back(flg);
};


void bias_mean(ambi_infor * bias_infor, double bias, double q)
{
	if (fabs(bias) > 1000)
		double bb = 0;
	if (fabs(bias - bias_infor->bias) > 2)
	{
		double bb = 0;
	}

	double sum_bias, sum_q;
	sum_bias = bias_infor->bias*bias_infor->num + bias;
	sum_q = bias_infor->q*bias_infor->num + q;

	bias_infor->num++;

	if (fabs(bias - round(bias)) < 0.1)
	{
		bias_infor->bias = bias;
		bias_infor->q = q;
	}
	else
	{
		bias_infor->bias = sum_bias / double(bias_infor->num);
		bias_infor->q = sum_q / double(bias_infor->num);
	}


};




