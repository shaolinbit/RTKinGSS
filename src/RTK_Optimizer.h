#ifndef RTK_OPTIMIZER_H
#define RTK_OPTIMIZER_H
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include "../src/global.h"

namespace gtsam
{

extern Matrix Information_bias;

//add pva[pos vel acc] value and prior factor
void RTKO_insertnewvalue_xc(int tcount, const double* x,  int x_pva_num);//add pva value
void RTKO_add_Position_Priori(const fr_check* frc, NonlinearFactorGraph *Factors);//add pva priorfactor
//add double diff ambiguity value and prior factor
void RTKO_insertnewvalue_xb(const int ambi_count, const double dd_bias);//add double diff ambiguity value
void RTKO_add_ddambi_Priori(const int ambi_count, const ambi_infor* dd_ambi, NonlinearFactorGraph *Factors);//add double diff ambiguity priorfactor
void RTKO_insert_newValuePirorFactor_xb_vector(vector<ambi_infor*> d_ambi, NonlinearFactorGraph *Factors);//add double diff ambiguities value and prior factor in a vector

//add double diffrence pseudorange and carrier Factor
void RTK_add_ddresFactor_without_ekf(gtime_t time, rtk_t* rtk, fr_check* frc, int epoch_index, const nav_t* nav,
	const obsd_t* obs, double* rs, double* dts, double* var, int* svh, double* y, double *v, double* R
	); //, NonlinearFactorGraph *Factors

//add Time Update Factor
void RTK_add_TimeUpdateFactor(int epoch_count_pre, gtime_t time_pre, int epoch_count_now, gtime_t time_now, NonlinearFactorGraph *Factors);
//add single diffrence doppler measurement factor
void RTK_add_doppler_sd_factor(int timecount, fr_check* frc, const nav_t* nav, const prcopt_t* opt, int nu
	, double *rs, double *dts, int *svh,double *var, double *var_doppler, int nv, int y_num, NonlinearFactorGraph *Factors , double *res=NULL);


//GSS optimization
void RTKO_GenerateTrajectory_dd_slidwin_ambivector_fix(rtk_t* rtk, fr_check *frc, int timecount, int win_size, FILE *fp = NULL);//sild window
void RTKO_GenerateTrajectory_dd_section(FILE *fp = NULL);//global

void update_slidwin_vector(rtk_t* rtk, fr_check *frc, int timecount, int win_size);//update silding window graph
void delete_outsize_factorvalue_vector(int win_size);//delete old factors and values
//delete factor
int FactorDelete(Key key, NonlinearFactorGraph *Factors);
int FactorDelete(Key key, vector<shared_ptr<RTK_ddres_Factor_L_dd_without_ekf>> *Factors);
int FactorDelete(Key key, vector<shared_ptr<RTK_ddres_Factor_P_without_ekf>> *Factors);
//insert factor
void insertRTKFactors(NonlinearFactorGraph *FactorA, NonlinearFactorGraph *FactorB);
void insertRTKFactors(NonlinearFactorGraph *FactorA, vector<shared_ptr<RTK_ddres_Factor_L_dd_without_ekf>> *FactorB);
void insertRTKFactors(NonlinearFactorGraph *FactorA, vector<shared_ptr<RTK_ddres_Factor_P_without_ekf>> *FactorB);
}

#endif
