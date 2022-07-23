#pragma once
#include<string>
#include<rtklib.h>
using namespace std;

#define MAX_FILE_NAME 1024
#define STATFILE    "rtknavi_%Y%m%d%h%M.stat"  // solution status file
#define TRACEFILE   "rtknavi_%Y%m%d%h%M.trace" // debug trace file

typedef std::string QString;
double NmeaIntv[2];

const char * qPrintable(const QString &file) { return file.data(); }

//#pragma region definition in navmain.h private and public
tle_t TLEData;
#define MAXSCALE	18
#define MAXMAPPNT	10
int PanelStack, PanelMode;
int SvrCycle, SvrBuffSize, Scale, SolBuffSize, NavSelect, SavedSol;
int NmeaReq, NmeaCycle, InTimeTag, OutTimeTag, OutAppend, LogTimeTag, LogAppend;
int TimeoutTime, ReconTime, SbasCorr, DgpsCorr, TideCorr, FileSwapMargin;
int Stream[MAXSTRRTK], StreamC[MAXSTRRTK], Format[MAXSTRRTK];
int CmdEna[3][2], CmdEnaTcp[3][2];
int TimeSys, SolType, PlotType1, FreqType1, PlotType2, FreqType2;
int TrkType1, TrkType2, TrkScale1, TrkScale2, BLMode1, BLMode2;
int MoniPort, OpenPort;

int PSol, PSolS, PSolE, Nsat[2], SolCurrentStat;
int Sat[2][MAXSAT], Snr[2][MAXSAT][NFREQ], Vsat[2][MAXSAT];
double Az[2][MAXSAT], El[2][MAXSAT];
gtime_t *Time;
int *SolStat, *Nvsat;
double *SolRov, *SolRef, *Qr, *VelRov, *Age, *Ratio;
double TrkOri[3];

QString Paths[MAXSTRRTK][4], Cmds[3][2], CmdsTcp[3][2];
QString InTimeStart, InTimeSpeed, ExSats;
QString RcvOpt[3], ProxyAddr;
QString OutSwapInterval, LogSwapInterval;

prcopt_t PrcOpt;
solopt_t SolOpt;

int DebugTraceF, DebugStatusF, OutputGeoidF, BaselineC;
int RovPosTypeF, RefPosTypeF, RovAntPcvF, RefAntPcvF;
QString RovAntF, RefAntF, SatPcvFileF, AntPcvFileF;
double RovAntDel[3], RefAntDel[3], RovPos[3], RefPos[3], NmeaPos[2];
double Baseline[2];

QString GeoidDataFileF, StaPosFileF, DCBFileF, EOPFileF, TLEFileF;
QString TLESatFileF, LocalDirectory, PntName[MAXMAPPNT];

double PntPos[MAXMAPPNT][3];
int NMapPnt;

QString MarkerName, MarkerComment;
//#pragma endregion
//#pragma region define in navmain.cpp
#define PRGNAME     "RTKNAVI-QT"           // program name
#define TRACEFILE   "rtknavi_%Y%m%d%h%M.trace" // debug trace file
#define STATFILE    "rtknavi_%Y%m%d%h%M.stat"  // solution status file
#define SATSIZE     20                  // satellite circle size in skyplot
#define MINSNR      10                  // minimum snr
#define MAXSNR      60                  // maximum snr
#define POSFONTNAME "Palatino Linotype"
#define POSFONTSIZE 12
#define MINBLLEN    0.01                // minimum baseline length to show

#define KACYCLE     1000                // keep alive cycle (ms)
#define TIMEOUT     10000               // inactive timeout time (ms)
#define DEFAULTPORT 52001               // default monitor port number
#define MAXPORTOFF  9                   // max port number offset
#define MAXTRKSCALE 23                  // track scale

#define SQRT(x)     ((x)<0.0?0.0:sqrt(x))
#define MIN(x,y)    ((x)<(y)?(x):(y))

//---------------------------------------------------------------------------

rtksvr_t rtksvr;                        // rtk server struct
stream_t monistr;                       // monitor stream
//#pragma endregion

#if 0 //input_file
AntPcvFileF, flag= RovAntPcvF || RefAntPcvF


#endif
