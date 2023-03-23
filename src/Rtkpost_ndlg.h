#pragma once

#pragma once
//#include "stdafx.h"
//#include <afx.h>
//
//#include "CPublic.h"
#include "global.h"
/*
	CString rover_path;
	CString base_path;
	CString nav_path;
	CString save_path;
	CString gga_path;
*/
//void ShowProcess();
//void ShowReading();
int Rtkpost();
int postpos(gtime_t ts, gtime_t te, double ti, double tu,
	const prcopt_t* popt, const solopt_t* sopt,
	const filopt_t* fopt, char** infile, int n, char* outfile,
	const char* rov, const char* base);
int execses_b(gtime_t ts, gtime_t te, double ti, const prcopt_t* popt,
	const solopt_t* sopt, const filopt_t* fopt, int flag,
	char** infile, const int* index, int n, char* outfile,
	const char* rov, const char* base);
int execses_r(gtime_t ts, gtime_t te, double ti, const prcopt_t* popt,
	const solopt_t* sopt, const filopt_t* fopt, int flag,
	char** infile, const int* index, int n, char* outfile,
	const char* rov);
int execses(gtime_t ts, gtime_t te, double ti, const prcopt_t* popt,
	const solopt_t* sopt, const filopt_t* fopt, int flag,
	char** infile, const int* index, int n, char* outfile);
int antpos_(prcopt_t* opt, int rcvno, const obs_t* obs, const nav_t* nav,
	const sta_t* sta, const char* posfile);
int avepos(double* ra, int rcv, const obs_t* obs, const nav_t* nav,
	const prcopt_t* opt);
int getstapos(const char* file, char* name, double* r);
void procpos(FILE* fp, FILE* fptm, const prcopt_t* popt, const solopt_t* sopt,
	rtk_t* rtk, int mode);
int inputobs(obsd_t* obs, int solq, const prcopt_t* popt, int br);
void combres(FILE* fp, FILE* fptm, const prcopt_t* popt, const solopt_t* sopt);
int valcomb(const sol_t* solf, const sol_t* solb);
int readobsnav(gtime_t ts, gtime_t te, double ti, char** infile,
	const int* index, int n, const prcopt_t* prcopt,
	obs_t* obs, obs_t *obs_b, nav_t* nav, sta_t* sta);
double get_daytime_(gtime_t tinput);
//afx_msg void OnBnClickedButton7();
int combineGPSGGA(char* gpsfile, string ggafile, string ggaout, char* resultfile);





//save information needed for factors
int savedata_fr_c(rtk_t *rtk, const prcopt_t* popt, const obsd_t* obs, const int nobs, const int isolf);
//save double difference ambiguities(double diffrence bias)
void init_ambi_infor_dd(int flg, double bias, double q, int epoch_s, gtime_t t_start, int epoch_e = 0);
//calculate mean of difference ambiguities(double diffrence bias)
void bias_mean(ambi_infor * bias_infor, double bias, double q);

