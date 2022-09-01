#pragma once
#include "rtk/rtk.h"
//#include "StrctDef.h"
#include <stack>
#include <vector>


#include <set>
#include <map>
//#include <crtdbg.h>

//#include <time.h>

#include <ctime>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>


using namespace std;

void ambi_infor_copy(ambi_infor * A, const ambi_infor * B);

class data_slidwin
{
public:
	int frc_s;
	int frc_e;
	//set<int> sd_index_30s;
	vector<ambi_infor*> ambi_detail;
	//vector<fr_check*> frcc;
public:
	data_slidwin() {};
	data_slidwin(int frc_s_, int frc_e_, vector<ambi_infor*> sd_ambi) :
		frc_s(frc_s_), frc_e(frc_e_)
	{
		for (int jj = 0; jj < sd_ambi.size(); jj++)
		{
			ambi_infor * bias_infor = new ambi_infor;

			ambi_infor_copy(bias_infor, sd_ambi.at(jj));

			ambi_detail.push_back(bias_infor);

		}
	}
	~data_slidwin()
	{
		for (int j = 0; j < ambi_detail.size(); j++)
		{
			delete(ambi_detail[j]);
			ambi_detail[j] = NULL;
		}
		ambi_detail.clear();
	}
};
namespace gtsam
{
//extern BOOL	m_bWorking;				//仿真程序运行标志
//extern vector<rtk_check*> check_f;
extern vector<fr_check*> fr_c;
extern vector<int> bias_flg;//双差模糊度组合信息（vflg）：基站卫星、流动站卫星、频率
extern set<int> dd_pair_set;
//extern set<int> sat_used;
extern int pre_ambi_size;
extern vector<ambi_infor*> d_ambi;//双差模糊度的值、时间、在卡尔曼滤波里计算的系统噪声


extern vector<int> key_pos;
extern vector<data_slidwin*> data_win;


extern vector<clock_t> runtime_slid;
extern vector<clock_t> runtime_opm;
extern vector<clock_t> runtime_file;
extern vector<clock_t> runtime_insert;
extern vector<clock_t> runtime_Q1;
extern vector<clock_t> runtime_Q2;
extern vector<clock_t> runtime_Q3;
extern int key_ambi_global;
extern int slidwin_size;//一次图优化单差向量持续的时长



//没用到的量
extern vector<double> cycle_slip_t;
extern vector <biaskey*> b_key;
extern vector <sat_pair*> sat_pairs;
extern std::map<int, int> sat_used_index;
//extern vector<NAVIGATION_C*>Nav_Data;

extern NonlinearFactorGraph RTKFactors;
//extern std::map<int, minimatrix*> RTKvalues;
//extern std::map<int, minimatrix*> RTK_Optimize_result;
//extern std::map<int, Matrix*> RTKvalues;
extern Values RTKvalues;
extern Values RTK_Optimize_result;
extern int valid_count;
};
