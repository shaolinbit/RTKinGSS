#pragma once

#include <stack>
#include <vector>
#include <set>
#include <map>
#include <ctime>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
//#include "../rtk/rtk.h"
#include "RTK_ddres_Factor.h"
using namespace std;

#define Rad_Deg 57.29577951308232
extern int flag_global;//0-slid window 1-global graph
extern int slidwin_size;//size of sliding window
extern string GSS_slidwin_result_file;//sliding window result
extern string GSS_global_result_file;//global result
extern string rtklib_ekf_result_file;//only use to compare with GSSM result

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
    //extern NonlinearFactorGraph RTKFactors;
    extern Values RTKvalues;
    //extern Values RTK_Optimize_result;

    extern NonlinearFactorGraph RTKPriorFactor;
    //extern NonlinearFactorGraph RTKddresFactor;
    extern vector<shared_ptr<RTK_ddres_Factor_L_dd_without_ekf>> RTKddresFactor_L;
    extern vector<shared_ptr<RTK_ddres_Factor_P_without_ekf>> RTKddresFactor_P;
    extern NonlinearFactorGraph RTKPreFactor;
    extern NonlinearFactorGraph RTKDopplerFactor;
};

extern vector<fr_check*> fr_c;
extern vector<int> bias_flg;//vector of the index of double diffrence ambiguities
extern set<int> dd_pair_set;
extern int pre_ambi_size;
extern vector<ambi_infor*> d_ambi;//information of double diffrence ambiguities in silding window


extern vector<int> key_pos;
extern vector<data_slidwin*> data_win;

extern int key_ambi_global;
extern int epoch_now;//epoch count
extern int first_epoch_flag;

extern int move_flag;//0-rover is stationary £¬1-rover is moving
extern double static_count;//count of static epoch
extern int fix_count;//count of continuous fix epoch

extern int dop_flag;

extern double var_thresshold;


