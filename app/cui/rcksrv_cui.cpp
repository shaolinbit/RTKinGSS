#include "stdafx.h"
#include"rtklib.h"
#include "rcksrv_cui.h"
/*
this source file inplement all the same functionbilities as GUI rtknavi
Notes:
1) all options for rtknavi can be loaded in gui, which then stored in struct PrcOpt and,
a litte bit scattered, variables in gui navimain and its subguis.

*/

QString OPT_FILE = "C:/Users/pc/Desktop/nav-ppp-k-gc.conf";
QString TLE_FILE = "";
QString TLE_SAT_FILE = "";
#define MAXSTR      1024                /* max length of a string */

// receiver options table ---------------------------------------------------
static int strtype[] = {                  /* stream types, these are default value, can be alter later */
	STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE,STR_NONE
};
static char strpath[8][MAXSTR] = { "" };    /* stream paths */
static int strfmt[] = {                   /* stream formats，these are default value, can be alter later */
	STRFMT_RTCM3,STRFMT_RTCM3,STRFMT_SP3,SOLF_LLH,SOLF_NMEA,0,0,0
};
static int svrcycle = 10;            /* server cycle (ms) */
static int timeout = 10000;         /* timeout time (ms) */
static int reconnect = 10000;         /* reconnect interval (ms) */
static int nmeacycle = 5000;          /* nmea request cycle (ms) */
static int fswapmargin = 30;            /* file swap marign (s) */
static int buffsize = 32768;         /* input buffer size (bytes) */
static int navmsgsel = 0;             /* navigation mesaage select */
/* nmeareq, nmeapos read and save by loadopts(qPrintable(file), rcvopts) and saveopts(qPrintable(file), rcvopts) */
static int nmeareq = 0;             /* nmea request type (0:off,1:lat/lon,2:single) */
static double nmeapos[] = { 0,0 };         /* nmea position (lat/lon) (deg) */
static char proxyaddr[MAXSTR] = "";       /* proxy address */

#define TIMOPT  "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT  "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT  "0:off,1:std+2:age/ratio/ns"
#define ISTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,7:ntripcli,8:ftp,9:http"
#define OSTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr"
#define FMTOPT  "0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,10:nvs,11:binex,12:rt17,13:sbf,14:cmr,17:sp3"
#define NMEOPT  "0:off,1:latlon,2:single"
#define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea"
#define MSGOPT  "0:all,1:rover,2:base,3:corr"

static opt_t rcvopts[] = {
	{"inpstr1-type",    3,  (void *)&strtype[0],         ISTOPT },// stream类型:tcp...
	{"inpstr2-type",    3,  (void *)&strtype[1],         ISTOPT },
	{"inpstr3-type",    3,  (void *)&strtype[2],         ISTOPT },
	{"inpstr1-path",    2,  (void *)strpath[0],         ""     },
	{"inpstr2-path",    2,  (void *)strpath[1],         ""     },
	{"inpstr3-path",    2,  (void *)strpath[2],         ""     },
	{"inpstr1-format",  3,  (void *)&strfmt[0],         FMTOPT }, // stream格式：rtcm2，3...
	{"inpstr2-format",  3,  (void *)&strfmt[1],         FMTOPT },
	{"inpstr3-format",  3,  (void *)&strfmt[2],         FMTOPT },
	{"inpstr2-nmeareq", 3,  (void *)&nmeareq,            NMEOPT },
	{"inpstr2-nmealat", 1,  (void *)&nmeapos[0],         "deg"  },
	{"inpstr2-nmealon", 1,  (void *)&nmeapos[1],         "deg"  },
	{"outstr1-type",    3,  (void *)&strtype[3],         OSTOPT },
	{"outstr2-type",    3,  (void *)&strtype[4],         OSTOPT },
	{"outstr1-path",    2,  (void *)strpath[3],         ""     },
	{"outstr2-path",    2,  (void *)strpath[4],         ""     },
	{"outstr1-format",  3,  (void *)&strfmt[3],         SOLOPT },
	{"outstr2-format",  3,  (void *)&strfmt[4],         SOLOPT },
	{"logstr1-type",    3,  (void *)&strtype[5],         OSTOPT },
	{"logstr2-type",    3,  (void *)&strtype[6],         OSTOPT },
	{"logstr3-type",    3,  (void *)&strtype[7],         OSTOPT },
	{"logstr1-path",    2,  (void *)strpath[5],         ""     },
	{"logstr2-path",    2,  (void *)strpath[6],         ""     },
	{"logstr3-path",    2,  (void *)strpath[7],         ""     },

	{"misc-svrcycle",   0,  (void *)&svrcycle,           "ms"   },
	{"misc-timeout",    0,  (void *)&timeout,            "ms"   },
	{"misc-reconnect",  0,  (void *)&reconnect,          "ms"   },
	{"misc-nmeacycle",  0,  (void *)&nmeacycle,          "ms"   },
	{"misc-buffsize",   0,  (void *)&buffsize,           "bytes"},
	{"misc-navmsgsel",  3,  (void *)&navmsgsel,          MSGOPT },
	{"misc-proxyaddr",  2,  (void *)proxyaddr,           ""     },
	{"misc-fswapmargin",0,  (void *)&fswapmargin,        "s"    },

	{"",0,NULL,""}
};

/* 这个函数就是把GUI中的LoadOpt SetOpt合并了，因为没有GUI，所以设置信息直接存到后台内存中去了
	1）variables from instrdlg, see BtnOkClick
		streamC		int		checkbox variables in inputStrDialog
							streamC[0]-[3]:input
							streamC[4]-[5]:output
							streamC[6]-[7]:log
		Stream		string	stream type index specified by user in gui
		Format		int		stream format: e.g. RTCM2...
		Paths[][]	string	stream parameter defined by rtklib, Paths[0]:serial,[1]:tcp,[2]:file,[3]:ftp
		CmdEna		int		Cmds enable or not
		Cmds		string
		CmdEnaTcp	int		CmdsTcp enable or not
		CmdsTcp		string
	Notes:
	1)tcp based stream(tcp,ntrip)
		ip,port,mount,account,pswd					->	account:pswd@ip:port/mount:
	i.g.:
		ntrip.gnsslab.cn,2101,yzmcsu1994,yzmcsu1994	->	yzmcsu1994:yzmcsu1994@ntrip.gnsslab.cn:2101/CLK93:
		123.56.239.141,4023							->	:@123.56.239.141:4023/:
*/
void LoadOpt(QString file)
{
	int itype[] = { STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPCLI,STR_FILE,STR_FTP,STR_HTTP };
	int otype[] = { STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPSVR,STR_FILE };

	string buff("");
	char id[32];
	int sat;
	prcopt_t prcopt = prcopt_default;
	solopt_t solopt = solopt_default;
	filopt_t filopt;

	memset(&filopt, 0, sizeof(filopt_t));

	resetsysopts();
	// read options to sysopts and rcvopts
    if (!loadopts(qPrintable(file), sysopts))
		return;
    if (!loadopts(qPrintable(file), rcvopts))
		return;
	getsysopts(&prcopt, &solopt, &filopt);
#if 0 //[bug] getsysopts()拷贝prcopt有bug，冲掉该函数里面的局部变量
    /* --------------------------------------------------------------------------
     * 到底是什么原因？
     * 0)与vs项目关于栈堆的配置有关？？[未验证]
     * 1)minGW验证没问题
     * 2)vs2015同样存在问题
    --------------------------------------------------------------------------*/
	int temp[] = { STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPCLI,STR_FILE,STR_FTP,STR_HTTP };
	int tempb[]= { STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPSVR,STR_FILE };
	for (int i = 0; i < 7; i++) itype[i] = temp[i];
	for (int i = 0; i < 5; i++) otype[i] = tempb[i];
#endif

	for (int i = 0; i < 8; i++) {
		StreamC[i] = strtype[i] != STR_NONE;
		Stream[i] = STR_NONE;
		for (int j = 0; j < (i < 3 ? 7 : 5); j++) { // 对于i=0~2，是inputstream，共有7种类型；而output只有5种类型
			if (strtype[i] != (i < 3 ? itype[j] : otype[j])) continue;
			Stream[i] = j; // 记录对应的itype或otype索引下标。。。
			break;
		}
		if (i < 5) Format[i] = strfmt[i];

		if (strtype[i] == STR_SERIAL) {
			Paths[i][0] = strpath[i];
		}
		else if (strtype[i] == STR_FILE) {
			Paths[i][2] = strpath[i];
		}
		else if (strtype[i] <= STR_NTRIPCLI) {
			Paths[i][1] = strpath[i];
		}
		else if (strtype[i] <= STR_HTTP) {
			Paths[i][3] = strpath[i];
		}
	}
	NmeaReq = nmeareq; // see comments in its definition
	NmeaPos[0] = nmeapos[0];
	NmeaPos[1] = nmeapos[1];

	for (sat = 1; sat <= MAXSAT; sat++) {
		if (!prcopt.exsats[sat - 1]) continue;
		satno2id(sat, id);
		buff += QString(buff.empty() ? "" : " ") + (prcopt.exsats[sat - 1] == 2 ? "+" : "") + id;
	}
	ExSats = buff;
	BaselineC = prcopt.baseline[0] > 0.0;
	Baseline[0] = prcopt.baseline[0];
	Baseline[1] = prcopt.baseline[1];

	DebugStatusF = solopt.trace;
	DebugTraceF = solopt.sstat;

	RovAntPcvF = *prcopt.anttype[0] > 0; // anttype字段非空，则说明PovAntPcvF被勾选（即checked)
	RefAntPcvF = *prcopt.anttype[1] > 0;
	RovAntF=QString(prcopt.anttype[0]);
	RefAntF=QString(prcopt.anttype[1]); 
	RovAntDel[0]= prcopt.antdel[0][0];
	RovAntDel[1]= prcopt.antdel[0][1];
	RovAntDel[2]= prcopt.antdel[0][2];
	RefAntDel[0]= prcopt.antdel[1][0];
	RefAntDel[1]= prcopt.antdel[1][1];
	RefAntDel[2]= prcopt.antdel[1][2];


	RovPosTypeF = 0; // {0：BLH(dms), 1:BLH(deg),2:xyz(m) }只有在fix模式下，gui才能选择其他类型，
	switch (prcopt.refpos) {
	case POSOPT_RTCM: RefPosTypeF = 3; break;
	case POSOPT_RAW: RefPosTypeF = 4; break;
	case POSOPT_SINGLE: RefPosTypeF = 5; break;
	default: RefPosTypeF = 0;
	}

	SatPcvFileF = QString(filopt.satantp);
	AntPcvFileF = QString(filopt.rcvantp);
	StaPosFileF = QString(filopt.stapos);
	GeoidDataFileF = QString(filopt.geoid);
	DCBFileF = QString(filopt.dcb);
	LocalDirectory = QString(filopt.tempdir);

	
	EOPFileF = QString(filopt.eop);
	// [bug] RTKLIB源码b31中，这两个变量有没有在*.cong文件中支持，且GUI修改后，没有及时更新该数据
	// TLEData初始化
	TLEData.n = TLEData.nmax = 0;
	TLEData.data = NULL;
	TLESatFileF = string(TLE_SAT_FILE);
	TLEFileF = string(TLE_FILE);
	if (TLEFileF.compare("")) {
		tle_read(qPrintable(TLEFileF), &TLEData);
	}
	if (TLESatFileF.compare("")) {
		tle_name_read(qPrintable(TLESatFileF), &TLEData);
	}// [bug]

	PrcOpt = prcopt;
	SolOpt = solopt;
}

void  InitSolBuff(void)
{
	double ep[] = { 2000,1,1,0,0,0 };
	int i, j;

	trace(3, "InitSolBuff\n");

	delete[] Time;   delete[] SolStat; delete[] Nvsat;  delete[] SolRov;
	delete[] SolRef; delete[] Qr;      delete[] VelRov; delete[] Age;
	delete[] Ratio;

	if (SolBuffSize <= 0) SolBuffSize = 1;
	Time = new gtime_t[SolBuffSize];
	SolStat = new int[SolBuffSize];
	Nvsat = new int[SolBuffSize];
	SolRov = new double[SolBuffSize * 3];
	SolRef = new double[SolBuffSize * 3];
	VelRov = new double[SolBuffSize * 3];
	Qr = new double[SolBuffSize * 9];
	Age = new double[SolBuffSize];
	Ratio = new double[SolBuffSize];
	PSol = PSolS = PSolE = 0;
	for (i = 0; i < SolBuffSize; i++) {
		Time[i] = epoch2time(ep);
		SolStat[i] = Nvsat[i] = 0;
		for (j = 0; j < 3; j++) SolRov[j + i * 3] = SolRef[j + i * 3] = VelRov[j + i * 3] = 0.0;
		for (j = 0; j < 9; j++) Qr[j + i * 9] = 0.0;
		Age[i] = Ratio[i] = 0.0;
	}
}

void init() {
	// MainForm constructor
	rtksvrinit(&rtksvr);
	strinit(&monistr);

	// MainForm showEvent
	InitSolBuff();
	strinitcom();

	LoadOpt(OPT_FILE);
	//LoadNav(&rtksvr.nav); // rtknavi在关闭软件时，会把最近的星历保存为自定义的字符串形式，方便在较短时间内再次启动时，重新利用最近的星历
}

// alway overwrite output
int ConfOverwrite(const char * path) { return 1; }

/* start rtk server ---------------------------------------------------------
 阉割的功能：
发GGA到基站，不支持VRS
----------------------------------------------------------------------------*/
extern void rtksrv_cui_start() 
{
	init();
	solopt_t solopt[2];
	double pos[3], nmeapos[3];
	int itype[] = { STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPCLI,STR_FILE,STR_FTP,STR_HTTP }; // 比output多了FTP模式
	int otype[] = { STR_SERIAL,STR_TCPCLI,STR_TCPSVR,STR_NTRIPSVR,STR_FILE };
	int i, strs[MAXSTRRTK] = { 0 }, sat, ex, stropt[8] = { 0 };
	char *paths[8], *cmds[3] = { 0 }, *rcvopts[3] = { 0 };
	char buff[1024], *p;
	/* qt build debug */
	char* cmds_periodic[3] = { NULL,NULL,NULL };
	char errmsg[20] = { 0 };

	gtime_t time = timeget();
	pcvs_t pcvr, pcvs;
	pcv_t *pcv;

	trace(3, "SvrStart\n");

	memset(&pcvr, 0, sizeof(pcvs_t));
	memset(&pcvs, 0, sizeof(pcvs_t));

	if (RovPosTypeF <= 2) { // LLH,XYZ
		PrcOpt.rovpos = 0;
		PrcOpt.ru[0] = RovPos[0];
		PrcOpt.ru[1] = RovPos[1];
		PrcOpt.ru[2] = RovPos[2];
	}
	else { // RTCM position
		PrcOpt.rovpos = 4;
		for (i = 0; i < 3; i++) PrcOpt.ru[i] = 0.0;
	}
	if (RefPosTypeF <= 2) { // LLH,XYZ
		PrcOpt.refpos = 0;
		PrcOpt.rb[0] = RefPos[0];
		PrcOpt.rb[1] = RefPos[1];
		PrcOpt.rb[2] = RefPos[2];
	}
	else if (RefPosTypeF == 3) { // RTCM position
		PrcOpt.refpos = 4;
		for (i = 0; i < 3; i++) PrcOpt.rb[i] = 0.0;
	}
	else { // average of single position
		PrcOpt.refpos = 1;
		for (i = 0; i < 3; i++) PrcOpt.rb[i] = 0.0;
	}

	// execlude sat
	for (i = 0; i < MAXSAT; i++) {
		PrcOpt.exsats[i] = 0;
	}
	if (ExSats != "") { // excluded satellites
		strcpy(buff, qPrintable(ExSats));
		for (p = strtok(buff, " "); p; p = strtok(NULL, " ")) {
			if (*p == '+') { ex = 2; p++; }
			else ex = 1;
			if (!(sat = satid2no(p))) continue;
			PrcOpt.exsats[sat - 1] = ex;
		}
	}// execlude sat

	// read ant pcv file, and its correction
	if ((RovAntPcvF || RefAntPcvF) && !readpcv(qPrintable(AntPcvFileF), &pcvr)) {
		cout << "rcv ant file read error" << AntPcvFileF << endl;
		return;
	}
	if (RovAntPcvF) {
		if ((pcv = searchpcv(0, qPrintable(RovAntF), time, &pcvr))) {
			PrcOpt.pcvr[0] = *pcv;
		}
		else {
			cout << "no antenna pcv" << qPrintable(RovAntF) << endl;
		}
		for (i = 0; i < 3; i++) PrcOpt.antdel[0][i] = RovAntDel[i];
	}
	if (RefAntPcvF) {
		if ((pcv = searchpcv(0, qPrintable(RefAntF), time, &pcvr))) {
			PrcOpt.pcvr[1] = *pcv;
		}
		else {
			cout << "no antenna pcv" << qPrintable(RefAntF) << endl;
		}
		for (i = 0; i < 3; i++) PrcOpt.antdel[1][i] = RefAntDel[i];
	}
	if (RovAntPcvF || RefAntPcvF) {
		free(pcvr.pcv);
	}
	if (PrcOpt.sateph == EPHOPT_PREC || PrcOpt.sateph == EPHOPT_SSRCOM) {
		if (!readpcv(qPrintable(SatPcvFileF), &pcvs)) {
			cout << "sat ant file read error" << SatPcvFileF << endl;
			return;
		}
		for (i = 0; i < MAXSAT; i++) {
			if (!(pcv = searchpcv(i + 1, "", time, &pcvs))) continue;
			rtksvr.nav.pcvs[i] = *pcv;
		}
		free(pcvs.pcv);
	}// read ant pcv file, and its correction

	if (BaselineC) {
		PrcOpt.baseline[0] = Baseline[0];
		PrcOpt.baseline[1] = Baseline[1];
	}
	else {
		PrcOpt.baseline[0] = 0.0;
		PrcOpt.baseline[1] = 0.0;
	}

	for (i = 0; i < 3; i++) strs[i] = StreamC[i] ? itype[Stream[i]] : STR_NONE; // input  stream
	for (i = 3; i < 5; i++) strs[i] = StreamC[i] ? otype[Stream[i]] : STR_NONE; // output stream
	for (i = 5; i < 8; i++) strs[i] = StreamC[i] ? otype[Stream[i]] : STR_NONE; // log    stream
	for (i = 0; i < 8; i++) { // read Paths
		paths[i] = new char[1024];
		paths[i][0] = '\0';
		switch (strs[i])
		{
		case STR_NONE:		strcpy(paths[i], ""); break;
		case STR_SERIAL:	strcpy(paths[i], qPrintable(Paths[i][0])); break;
		case STR_FILE:		strcpy(paths[i], qPrintable(Paths[i][2])); break;
		case STR_FTP:
		case STR_HTTP:		strcpy(paths[i], qPrintable(Paths[i][3])); break;
		default:			strcpy(paths[i], qPrintable(Paths[i][1])); break;
		}
	}// read Paths
	for (i = 0; i < 3; i++) {// read commands
		cmds[i] = new char[1024];
		rcvopts[i] = new char[1024];
		cmds[i][0] = rcvopts[i][0] = '\0';
		if (strs[i] == STR_SERIAL) {
			if (CmdEna[i][0]) strcpy(cmds[i], qPrintable(Cmds[i][0]));
		}
		else if (strs[i] == STR_TCPCLI || strs[i] == STR_TCPSVR ||
			strs[i] == STR_NTRIPCLI) {
			if (CmdEnaTcp[i][0]) strcpy(cmds[i], qPrintable(CmdsTcp[i][0]));
		}
		strcpy(rcvopts[i], qPrintable(RcvOpt[i]));
	}// read commands
	NmeaCycle = NmeaCycle < 1000 ? 1000 : NmeaCycle;
	pos[0] = NmeaPos[0] * D2R;
	pos[1] = NmeaPos[1] * D2R;
	pos[2] = 0.0;
	pos2ecef(pos, nmeapos);

	strsetdir(qPrintable(LocalDirectory));
	strsetproxy(qPrintable(ProxyAddr));

	for (i = 3; i < 8; i++) {
		if (strs[i] == STR_FILE && !ConfOverwrite(paths[i])) return;
	}
	if (DebugTraceF > 0) { // 如果要开启，需要定义TRACE符号
		traceopen(TRACEFILE);
		tracelevel(DebugTraceF);
	}
	if (DebugStatusF > 0) {
		rtkopenstat(STATFILE, DebugStatusF);
	}
	if (SolOpt.geoid > 0 && GeoidDataFileF != "") {
		opengeoid(SolOpt.geoid, qPrintable(GeoidDataFileF));
	}
	if (DCBFileF != "") {
		readdcb(qPrintable(DCBFileF), &rtksvr.nav, NULL);
	}
	for (i = 0; i < 2; i++) {
		solopt[i] = SolOpt;
		solopt[i].posf = Format[i + 3];
	}
	stropt[0] = TimeoutTime;
	stropt[1] = ReconTime;
	stropt[2] = 1000;
	stropt[3] = SvrBuffSize;
	stropt[4] = FileSwapMargin;
	strsetopt(stropt);

	// start rtk server
	if (!rtksvrstart(&rtksvr, SvrCycle, SvrBuffSize, strs, paths, Format, NavSelect, cmds, cmds_periodic, 
		rcvopts, NmeaCycle, NmeaReq, nmeapos, &PrcOpt, solopt, &monistr, errmsg)) {
		traceclose();
		for (i = 0; i < 8; i++) delete[] paths[i];
		for (i = 0; i < 3; i++) delete[] rcvopts[i];
		for (i = 0; i < 3; i++) delete[] cmds[i];
		cout << "Error occurs: " << errmsg << endl;
		return;
	}
	for (i = 0; i < 8; i++) delete[] paths[i];
	for (i = 0; i < 3; i++) delete[] rcvopts[i];
	for (i = 0; i < 3; i++) delete[] cmds[i];
	PSol = PSolS = PSolE = 0;
	SolStat[0] = Nvsat[0] = 0;
	for (i = 0; i < 3; i++) SolRov[i] = SolRef[i] = VelRov[i] = 0.0;
	for (i = 0; i < 9; i++) Qr[i] = 0.0;
	Age[0] = Ratio[0] = 0.0;
	Nsat[0] = Nsat[1] = 0;
	cout << "rtksvr stopped " << endl;
	return;
}
