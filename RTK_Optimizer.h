//#include "rtk/rtk.h"
#include "global.h"
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

namespace gtsam
{
void RTKO_GenerateTrajectory_dd(const nav_t* nav, const prcopt_t* opt);
void RTKO_insertnewvalue_xb(const int ambi_count, const double dd_bias);
void RTKO_add_ddambi_Priori(const int ambi_count, const ambi_infor* dd_ambi);//, NonlinearFactorGraph *Factors = &RTKFactors);
void RTKO_insertnewvalue_xc(int tcount, const double* x,  const fr_check* frc);
void RTKO_add_Position_Priori(const fr_check* frc);//, NonlinearFactorGraph *Factors = &RTKFactors);
void RTKO_add_ddres_Factor(fr_check* frc, const nav_t* nav, const prcopt_t* opt);
}
