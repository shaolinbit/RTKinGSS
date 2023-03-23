#include "global.h"
#include <map>

int flag_global=0;
int slidwin_size = 10;//5
string GSS_slidwin_result_file;
string GSS_global_result_file;
string rtklib_ekf_result_file;

namespace gtsam
{
    //NonlinearFactorGraph RTKFactors;
    Values RTKvalues;
    //Values RTK_Optimize_result;

    NonlinearFactorGraph RTKPriorFactor;
    //NonlinearFactorGraph RTKddresFactor;
    vector<shared_ptr<RTK_ddres_Factor_L_dd_without_ekf>> RTKddresFactor_L;
    vector<shared_ptr<RTK_ddres_Factor_P_without_ekf>> RTKddresFactor_P;
    NonlinearFactorGraph RTKPreFactor;
    NonlinearFactorGraph RTKDopplerFactor;
};
vector<fr_check*> fr_c;
vector<int> bias_flg;
set<int> dd_pair_set;
vector<ambi_infor*> d_ambi;
vector<int> key_pos;

vector<data_slidwin*> data_win;


int key_ambi_global = 0;
int epoch_now = 0;
int first_epoch_flag = 1;
int move_flag = 0;
double static_count = 0;
int fix_count=0;

int dop_flag = 0;
double var_thresshold = 0;


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
