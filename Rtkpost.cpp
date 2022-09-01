#include "Rtkpost.h"
#include <algorithm>
#include "RTK_Optimizer.h"
using namespace std;
namespace gtsam{
extern gtime_t pro_time;
bool working=false;
bool flag;
int arminfix;
//int SYNCRTKIMUFlag;
int dayflag;


//extern vector<fr_check*> fr_c;
//extern vector<int> bias_flg;//双差模糊度组合信息（vflg）：基站卫星、流动站卫星、频率
//extern vector<ambi_infor*> d_ambi;//双差模糊度的时间

//vector<int> dd_ambi_num;//临时变量用于统计双差模糊度
int data_num = 0;
//__int64 t0 = 0;
int frc_index_s;




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
	double sdn, sdu, sde,sdne,sdeu,sdun;
	double ratio, age;
	double fix;
}rtkdata;


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
	return stat;
}


/* execute processing session ------------------------------------------------*/
int execses(gtime_t ts, gtime_t te, double ti, const prcopt_t* popt,
	const solopt_t* sopt, const filopt_t* fopt, int flag,
	char** infile, const int* index, int n, char* outfile)
{
	//_CrtSetBreakAlloc(237227);//1253361

	FILE* fp, * fptm;// *file;
	rtk_t rtk;
	prcopt_t popt_ = *popt;
	solopt_t tmsopt = *sopt;
	char tracefile[1024], statfile[1024], path[1024], * ext, outfiletm[1024] = { 0 };

	trace(3, "execses : n=%d outfile=%s\n", n, outfile);

	/* read obs and nav data */

	if (!readobsnav(ts, te, ti, infile, index, n, &popt_, &obss,&obss_b, &navs, stas)) {
		freeobsnav(&obss, &navs);
		return 0;
	}


	/* set antenna parameters */
	if (popt_.mode != PMODE_SINGLE) {
		setpcv(obss.n > 0 ? obss.data[0].time : timeget(), &popt_, &navs, &pcvss, &pcvsr,
			stas);
	}

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

	/* open solution statistics */
	if (flag && sopt->sstat > 0) {
		strcpy(statfile, outfile);
		strcat(statfile, ".stat");
		rtkclosestat();
		rtkopenstat(statfile, sopt->sstat);
	}
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

			FILE* fp_time = fopen("../data/output/runtime.txt", "w");
			fprintf(fp_time, "%s\t%s\t%s\t%s\n", "runtime_slid", "runtime_opm",
				"runtime_file", "runtime_insert");

            procpos(NULL, NULL, &popt_, sopt, &rtk, 1); /* forward */
			/* combine forward/backward solutions */

			for (int i = 0; i < runtime_slid.size(); i++)
			{
				fprintf(fp_time, "%d\t%d\t%d\t%d\t%d\t%d\t%d\n", runtime_slid.at(i), runtime_opm.at(i),
					runtime_file.at(i), runtime_insert.at(i),runtime_Q1.at(i), runtime_Q2.at(i), runtime_Q3.at(i));
			}
			fclose(fp_time);

			vector<double> baseline;

			/* output RTK result to file */
			FILE* fp = fopen("../data/output/RTK_result.txt", "w");
			for (int i = 0; i < fr_c.size(); i++)//fr_c.size()
			{
				//fprintf(fp, "%d\t", fr_c.at(i)->epoch_num);
				if (fr_c.at(i)->stat <= 0)
					continue;

				int week;
				double sec;
				sec= time2gpst(fr_c.at(i)->time,&week);
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

				fprintf(fp, "%f\t%f\t%f\t", fr_c[i]->Pf[0], fr_c[i]->Pf[1], fr_c[i]->Pf[2]);
				fprintf(fp, "%f\t%f\t%f\t", fr_c[i]->vv[0], fr_c[i]->vv[1], fr_c[i]->vv[2]);
				fprintf(fp, "\n");
			}
			fclose(fp);

			/* 图优化 */
			gtsam::RTKO_GenerateTrajectory_dd(&navs, popt);

			/*free memory*/


		//	dd_ambi_num.clear();

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
			//Clear_RTKvariables();
		}
		else showmsg("error : memory allocation");
		free(solf);
		free(solb);
		free(rbf);
		free(rbb);
		rtkfree(&rtk);
	}
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
		if (!avepos_(rr, rcvno, obs, nav, opt)) {//base station
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

int avepos_(double* ra, int rcv, const obs_t* obs, const nav_t* nav,
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

		///* 输出基站单点定位结果 */
		//FILE *fp;
		//fp = fopen("basepos.txt", "a+");
		//fprintf(fp, "%d  %f  %f  %f \n", n,sol.rr[0], sol.rr[1], sol.rr[2]);
		//fclose(fp);



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



void bias_mean(ambi_infor * bias_infor, double bias, double q)
{
	if (fabs(bias) > 1000)
		double bb = 0;
	if (fabs(bias - bias_infor->bias) > 2)
	{
		double bb = 0;
	}
	/*if (bias_infor->bias_flag ==512  && bias_infor->epoch_s == 0)
	{
		FILE *fp;
		fp = fopen("sd_ambi_check_20191109_512.txt", "a");
		fprintf(fp, "%d  %f\n", bias_infor->epoch_s + bias_infor->num, bias);
		fclose(fp);
	}*/
	double sum_bias, sum_q;
	sum_bias = bias_infor->bias*bias_infor->num + bias;
	sum_q = bias_infor->q*bias_infor->num + q;

	bias_infor->num++;
	bias_infor->bias = sum_bias / double(bias_infor->num);
	bias_infor->q = sum_q / double(bias_infor->num);

};

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

int readobsnav(gtime_t ts, gtime_t te, double ti, char** infile,
	const int* index, int n, const prcopt_t* prcopt,
	obs_t* obs,obs_t *obs_b, nav_t* nav, sta_t* sta)
{
	int i, j, ind = 0, nobs = 0, rcv = 1;

	trace(3, "readobsnav: ts=%s n=%d\n", time_str(ts, 0), n);

	obs->data = NULL; obs->n = obs->nmax = 0; obs->rover_n=0;
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

void savedata_fr_c(rtk_t *rtk, const prcopt_t* popt, const obsd_t* obs, const int nobs, const int isolf)
{
	//单点定位结果都没有的时候不要存
	if (norm(rtk->sol.rr, 3) < 10)
		return;

	//没有rtk结果的历元该怎么处理，ddres_也不能直接找dd_pair里面存的数？？？
	if (rtk->sol.stat == SOLQ_NONE)//不能用此判断，L1不足4就会被置0,但是实际上是进行了kf计算，有可用的双差
	{
		double bb = rtk->nv;
	}


	//存储前向rtk的结果数据===============================================================
	int i, j, k;//*ix, p = (double*)malloc(sizeof(double) * n * m))
	fr_check* fr_cc = new fr_check;
	int sd_ambiguity_num;//单差模糊度数量

	/*-----------------------------------------------------------------*/
	//fr_cc->dd_bias = mat(rtk->nv, 1);
	matcpy(fr_cc->dd_bias, rtk->dd_bias, rtk->nv, 1);
	/*-----------------------------------------------------------------*/


	//时间
	fr_cc->time = rtk->sol.time;
	fr_cc->eventime = rtk->sol.eventime;
	fr_cc->dt = rtk->dt;
	fr_cc->epoch_num = isolf - 1;
	//流动站定位结果xyz
	for (i = 0; i < 3; i++)
	{
		fr_cc->xf[i] = rtk->sol.rr[i];
		fr_cc->vv[i] = rtk->sol.rr[i + 3];
	}

	//基站位置
	for (i = 0; i < 6; i++) {
		fr_cc->rb[i] = rtk->rb[i];//基站是固定的，应该不需要每个历元都存
	}

	//协方差
	////坐标协方差
	//for (i = 0; i < 3; i++)
	//{
	//	for (j = 0; j < 3; j++)
	//	{
	//		fr_cc->Pf[i*(3) + j] = rtk->P[i*rtk->nx + j];
	//	}
	//}

	//坐标协方差
	for (i = 0; i < 6; i++)
	{
		fr_cc->Pf[i] = rtk->sol.qr[i];
	}

	//本历元双差模糊度的信息记录，同时记录是哪两个卫星，是否固定了，有无周跳
	fr_cc->stat = rtk->sol.stat;
	for (i = 0; i < MAXSAT; i++) {/*卫星周跳状态，slip整周周跳，half半周周跳*/
		fr_cc->ssat[i] = rtk->ssat[i];
	}

	for (i = 0; i < rtk->ns; i++) {
		fr_cc->sat[i] = rtk->sat[i];
	}

	fr_cc->sat_ns = rtk->ns;//共视卫星数

	/*if (rtk->nfix == 0)
	{
		for (i = 0; i < rtk->ns; i++) {
			for (k = 0; k < popt->nf; k++)
			{
				fr_cc->xf[3 + i * popt->nf + k] = rtk->x[9 + MAXSAT * k + rtk->sat[i] - 1];
				fr_cc->P_bias[i * popt->nf + f] = rtk->P[index * rtk->nx + index];
			}
		}
	}
	else if (rtk->nfix != 0)
	{
		for (i = 0; i < rtk->ns; i++) {
			for (k = 0; k < popt->nf; k++)
			{
				fr_cc->xf[3 + i * popt->nf + k] = rtk->xa_[9 + MAXSAT * k + rtk->sat[i] - 1];
				fr_cc->P_bias[i * popt->nf + f] = rtk->P[index * rtk->nx + index];
			}
		}
	}*/

	if (rtk->nfix == 0)
	{
		for (i = 0; i < rtk->ns; i++) {
			for (int f = 0; f < popt->nf; f++)
			{
				int index = 9 + MAXSAT * f + rtk->sat[i] - 1;
				fr_cc->xf[3 + i * popt->nf + f] = rtk->x[index];
				fr_cc->P_bias[i * popt->nf + f] = rtk->P[index * rtk->nx + index];
			}
		}
	}
	else if (rtk->nfix != 0)
	{
		for (i = 0; i < rtk->ns; i++) {
			for (int f = 0; f < popt->nf; f++)
			{
				int index = 9 + MAXSAT * f + rtk->sat[i] - 1;
				fr_cc->xf[3 + i * popt->nf + f] = rtk->xa_[index];
				fr_cc->P_bias[i * popt->nf + f] = rtk->P[index * rtk->nx + index];
			}
		}
	}


	//本历元的参考站和流动站的卫星观测值
	fr_cc->nn = rtk->nn;//基站加流动站卫星数
	fr_cc->nr = rtk->nr;//基站卫星数
	fr_cc->nu = rtk->nu;//流动站卫星数
	for (i = 0; i < nobs; i++) {
		fr_cc->obs[i] = obs[i];//观测值本身存储全局变量里面，可以考虑写一个函数通过时间查找观测值，减少重复存储（但是这样需要每次遍历（观测值数组很大），计算量增加？？？）
	}
	for (i = 0; i < rtk->nu && i < rtk->nr; i++) {/*记录共视卫星的prn*/
		fr_cc->iu[i] = rtk->iu[i];
		fr_cc->ir[i] = rtk->ir[i];
	}

	/*统计双差模糊度-----------------------------------------------------*/


	for (i = 0, k = 0; i < rtk->nv; i++)/*双差信息，基站卫星号，流动站卫星号，观测值类型（code：0-载波，1-伪距），频率*/
	{
		//int type = (rtk->vflg[i] >> 4) & 0xF;
		if (((rtk->vflg[i] >> 4) & 0xF) == 0)//载波双差
		{
			fr_cc->vflg[k++] = rtk->vflg[i];
		}
	}
	fr_cc->nv = rtk->nv;
	fr_cc->num_sat_pair = k;//双差模糊度数量
	//fr_cc->bias_index = (int*)malloc(sizeof(int) * k * 1);//考虑记录双差模糊度在double_diff_pair中的位置


	for (i = 0; i < rtk->nv; i++)//ns:共视卫星数
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

			do//查找卫星的单差模糊度，找不到增加变量
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
			if (flag == 0)//有周跳要增变量
			{
				//----MW周跳探测尝试------------------------------------------
				detslp_mw_(rtk, obs, rtk->nu, &navs);
				//--------------------------------------------

				int ref_sat, u_ref_sat, f;//参考星prn, 非参考星prn, 频率 0-L1，1-L2
				ref_sat = (rtk->vflg[i] >> 16) & 0xFF;
				u_ref_sat = (rtk->vflg[i] >> 8) & 0xFF;
				f = rtk->vflg[i] & 0xF;
				if ((rtk->ssat[ref_sat].slip[f] & 1) || (rtk->ssat[u_ref_sat].slip[f] & 1))//有周跳要增变量
				{
					//-----周跳修复测试-----------------------------------------------
					/*double sat_slip_prn[2];
					int slip_num = 0;
					if (rtk->ssat[ref_sat].slip[f] & 1)
					{
						sat_slip_prn[slip_num] = ref_sat;
						slip_num++;
					}
					else if (rtk->ssat[u_ref_sat].slip[f] & 1)
					{
						sat_slip_prn[slip_num] = u_ref_sat;
						slip_num++;

					}
					for (int ii = 0; ii < slip_num; ii++)
					{
						int iu_index = 0, repiar_flag = 0;
						for (iu_index = 0; iu_index < MAXSAT; iu_index++)
						{
							if (rtk->sat[iu_index] == sat_slip_prn[ii])
								break;
						}
						if (iu_index < MAXSAT)
						{
							repiar_flag = repair_slp_mw_gf(rtk, obs + rtk->iu[iu_index], &navs, sat_slip_prn[ii]);
							if (repiar_flag == 1)
							{

							}
						}
					}
					*/

					//rtk->iu[u_ref_sat]
					//repair_slp_mw_gf(rtk, obs+, int n, const nav_t* nav)
					//--------------------------------------------------
					d_ambi.at(index)->epoch_e = -1;
					init_ambi_infor_dd(rtk->vflg[i], rtk->dd_bias[i], rtk->P_dd_ambiguity[i], fr_cc->epoch_num, fr_cc->time);
					fr_cc->bias_index[i] = d_ambi.size() - 1;
				}
				else
				{
					if ((fr_cc->epoch_num > (d_ambi.at(index)->epoch_s + d_ambi.at(index)->num))
						||fabs(rtk->dd_bias[i]- d_ambi.at(index)->bias)>5)//有双差从几万变成两位数但不显示有周跳的情况？？？
					{
						d_ambi.at(index)->epoch_e = -1;
						init_ambi_infor_dd(rtk->vflg[i], rtk->dd_bias[i], rtk->P_dd_ambiguity[i], fr_cc->epoch_num, fr_cc->time);
						fr_cc->bias_index[i] = d_ambi.size() - 1;
					}
					else
					{
						fr_cc->bias_index[i] = index;
						bias_mean(d_ambi.at(index), rtk->dd_bias[i], rtk->P_dd_ambiguity[i]);
					}

				}
			}
		}
	}


	fr_c.push_back(fr_cc);

}

int inputobs(obsd_t* obs, int solq, const prcopt_t* popt,int br)
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
			}
			if (checkbrk("processing : %s Q=%d", time_str(time, 0), solq)) {
				aborts = 1; showmsg("aborted");
				return -1;
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

/* process positioning -------------------------------------------------------*/
void procpos(FILE* fp, FILE* fptm, const prcopt_t* popt, const solopt_t* sopt,
	rtk_t* rtk, int mode)
{
	gtime_t time = { 0 };
	sol_t sol = { {0} }, oldsol = { {0} }, newsol = { {0} };
	obsd_t obs[MAXOBS * 2]; /* for rover and base */
	double rb[3] = { 0 };
	int i, nobs, n, solstatic, num = 0, pri[] = { 0,1,2,3,4,5,1,6 };

	trace(3, "procpos : mode=%d\n", mode);

	solstatic = sopt->solstatic &&
		(popt->mode == PMODE_STATIC || popt->mode == PMODE_STATIC_START || popt->mode == PMODE_PPP_STATIC);

	/* initialize unless running backwards on a combined run with continuous AR in which case keep the current states */
	if (mode == 0 || !revs || popt->modear == ARMODE_FIXHOLD)
		rtkinit(rtk, popt);

	rtcm_path[0] = '\0';
	//图优化输出
	//FILE *f_out = fopen("graph_result_dd_slid_win.txt", "w");
	//fclose(f_out);
	//f_out = fopen("graph_result_dd_slid_win.txt", "a+");

	while ((nobs = inputobs(obs, rtk->sol.stat, popt,1)) >= 0) {
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
		if (!rtkpos(rtk, obs, n, &navs)) {
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



			/*save data to fr_c*/
			savedata_fr_c(rtk, popt, obs, nobs, isolf);
			//savedata_fr_c_2(rtk, popt, obs, nobs, isolf);

			//savedata_fr_c_each_section(rtk, popt, obs, nobs, isolf, slidwin_size,2);
			//savedata_fr_c_each_section(rtk, popt, obs, nobs, isolf, slidwin_size, 1);

			//savedata_fr_c_slidwin(rtk, popt, obs, nobs, isolf, slidwin_size, 1,f_out);
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


	//savedata_fr_c_each_section(rtk, popt, obs, nobs, isolf, slidwin_size,2,1);
	//savedata_fr_c_each_section(rtk, popt, obs, nobs, isolf, slidwin_size, 1, 1);

	//savedata_fr_c_slidwin(rtk, popt, obs, nobs, isolf, slidwin_size, 1, f_out, 1);

	//fclose(f_out);
}

int repair_slp_mw_gf(rtk_t* rtk, const obsd_t* obs, const nav_t* nav,int prn)
{
	//double CLIGHT / FREQL1
	int i = 1;//Ŀǰֻ��L1 L2���б������
	/* -------------- GF -------------*/
	const double* lam = nav->lam[obs->sat - 1];
	//int i = (satsys(obs->sat, NULL) & (SYS_GAL | SYS_SBS)) ? 2 : 1;

	if (lam[0] == 0.0 || lam[i] == 0.0 || obs->L[0] == 0.0 || obs->L[i] == 0.0)
		return 0;

	double gf= lam[0] * obs->L[0] - lam[i] * obs->L[i];

	/* -------------- MW -------------*/
	//const double* lam = nav->lam[obs->sat - 1];
	//int i = (satsys(obs->sat, NULL) & (SYS_GAL | SYS_SBS)) ? 2 : 1;

	if (lam[0] == 0.0 || lam[i] == 0.0 || obs->L[0] == 0.0 || obs->L[i] == 0.0 ||
		obs->P[0] == 0.0 || obs->P[i] == 0.0)
		return 0;

	double lam_mw_L12 = CLIGHT / (FREQL1 - FREQL2);

	double mw=(obs->L[0] - obs->L[i]) -
		(FREQL1 * obs->P[0] + FREQL2 * obs->P[i]) / ((FREQL1 + FREQL2)*lam_mw_L12);

	/* ����޸�����*/
	double d_bias_mw, d_bias_gf;//Ҫ����һ����Ԫ�������Ҫ����һ����Ԫ�Ľ��
	double gf0, mw0;
	fr_check *fr_c_0 = fr_c.at(fr_c.size() - 1);

	//��һ����Ԫ��gf��MW��Ϲ۲�ֵ
	int iu_index = 0, repiar_flag = 0;
	for (iu_index = 0; iu_index < MAXSAT; iu_index++)
	{
		if (fr_c_0->sat[iu_index] == prn)
			break;
	}
	if (iu_index >= MAXSAT)
		return 0;
	int obs_index = fr_c_0->iu[iu_index];

	if (lam[0] == 0.0 || lam[i] == 0.0 || fr_c_0->obs[obs_index].L[0] == 0.0 || fr_c_0->obs[obs_index].L[i] == 0.0)
		return 0;

	gf0 = lam[0] * fr_c_0->obs[obs_index].L[0] - lam[i] * fr_c_0->obs[obs_index].L[i];

	/* -------------- MW -------------*/
	//const double* lam = nav->lam[obs->sat - 1];
	//int i = (satsys(obs->sat, NULL) & (SYS_GAL | SYS_SBS)) ? 2 : 1;

	if (lam[0] == 0.0 || lam[i] == 0.0 || fr_c_0->obs[obs_index].L[0] == 0.0 || fr_c_0->obs[obs_index].L[i] == 0.0 ||
		fr_c_0->obs[obs_index].P[0] == 0.0 || fr_c_0->obs[obs_index].P[i] == 0.0)
		return 0;

	//double lam_mw_L12 = CLIGHT / (FREQL1 - FREQL2);

	mw0 = (fr_c_0->obs[obs_index].L[0] - fr_c_0->obs[obs_index].L[i]) -
		(FREQL1 * fr_c_0->obs[obs_index].P[0] + FREQL2 * fr_c_0->obs[obs_index].P[i]) / ((FREQL1 + FREQL2)*lam_mw_L12);

	d_bias_mw = mw - mw0;
	d_bias_gf = gf - gf0;
	if (fabs(d_bias_mw) < 0.9&&fabs(d_bias_gf) < rtk->opt.thresslip)
		return 2;
	double slip_L1, slip_L2;
	slip_L1 = (d_bias_gf - lam[i] * d_bias_mw) / (lam[0] - lam[i]);
	slip_L2 = slip_L1 - d_bias_mw;

	return 1;
}
};
