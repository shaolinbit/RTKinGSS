//#include "StdAfx.h"
#include "global.h"
#include <map>
namespace gtsam
{
vector<fr_check*> fr_c;
vector<int> bias_flg;//双差模糊度组合信息（vflg）：基站卫星、流动站卫星、频率
set<int> dd_pair_set;
vector<ambi_infor*> d_ambi;//双差模糊度的时间
vector<int> key_pos;

//vector<ambi_infor*> d_ambi;//单差模糊度

//set<ambi_infor*> sd_ambi_30s;//30s的单差模糊度统计

//vector<set<int>> sd_index;
vector<data_slidwin*> data_win;

vector<clock_t> runtime_slid;
vector<clock_t> runtime_opm;
vector<clock_t> runtime_file;
vector<clock_t> runtime_insert;
vector<clock_t> runtime_Q1;
vector<clock_t> runtime_Q2;
vector<clock_t> runtime_Q3;

int key_ambi_global = 0;
int slidwin_size = 10;

vector<double> cycle_slip_t;
vector <biaskey*> b_key;
vector <sat_pair*> sat_pairs;
std::map<int, int> sat_used_index;

NonlinearFactorGraph RTKFactors;

Values RTKvalues;
Values RTK_Optimize_result;

int valid_count = 0;
};

void ambi_infor_copy(ambi_infor * A, const ambi_infor * B)
{
	A->bias = B->bias;
	A->bias_flag = B->bias_flag;
	A->epoch_e = B->epoch_e;
	A->epoch_s = B->epoch_s;
	A->num = B->num;
	A->q = B->q;
	A->slipcount = B->slipcount;
	A->t_end = B->t_end;
	A->t_start = B->t_start;
}
