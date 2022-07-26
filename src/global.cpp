#include "rtklib.h"

vector<fr_check*> fr_c;
 vector<int> bias_flg;//双差模糊度组合信息（vflg）：基站卫星、流动站卫星、频率
 set<int> dd_pair_set;
 vector<ambi_infor*> d_ambi;//双差模糊度的时间
 vector<int> key_pos;
int key_ambi_global =0;
