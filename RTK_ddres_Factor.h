#ifndef RTK_DDRES_FACTOR_H
#define RTK_DDRES_FACTOR_H
#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include "rtk/rtk.h"
//#include "global.h"
namespace gtsam
{

	/**
	 * A class for a rtk ddres
	 * @addtogroup INS
	 */

	class RTK_ddres_Factor_P : public NoiseModelFactor1<Vector3>
	{
	public:

		fr_check *frc;
		const nav_t* nav_;
		const prcopt_t* opt_;
		double base_measure;
		int sat_index_, ref_sat_;
		int f_;
		double ddres0;

	public:
		RTK_ddres_Factor_P() {};
		RTK_ddres_Factor_P(int keypos, fr_check *frc_, const nav_t* nav, const prcopt_t* opt, double base_measure,
			int sat_index, int ref_sat, int f, const SharedNoiseModel& model, double ddres0) :NoiseModelFactor1(model, keypos),
			base_measure(base_measure), frc(frc_), nav_(nav), opt_(opt), sat_index_(sat_index), ref_sat_(ref_sat), f_(f),
			ddres0(ddres0)
		{
		};

		~RTK_ddres_Factor_P()
		{
			frc = NULL;
			nav_ = NULL;
			opt_ = NULL;
		};


			         Vector evaluateError(const Vector3& X1,
      boost::optional<Matrix&> H1 = boost::none) const
		{

			Vector1 VV;


			double newxyz_[3];
			for (int k = 0; k < 3; k++)
			{
				newxyz_[k] = frc->xf[k] + X1(k);//
			}


			int i = sat_index_;
			const obsd_t* obs_uref = &(frc->obs[frc->iu[i]]);
			double *rs_i, *dts_i, *var_i, *e_i, *azel_i, *y_i;
			double *rs_ref, *dts_ref, *var_ref, *e_ref, *azel_ref, *y_ref;
			int svh_i[1], svh_ref[1];


			rs_i = mat(6, 1);            /* range to satellites */
			dts_i = mat(2, 1);           /* satellite clock biases */
			y_i = mat(opt_->nf * 2, 1);
			var_i = mat(1, 1);
			e_i = mat(3, 1);
			azel_i = rtklib_zeros(2, 1);

			satposs(frc->time, &(frc->obs[frc->iu[i]]), 1, nav_, opt_->sateph, rs_i, dts_i, var_i, svh_i);

			zdres_(0, &(frc->obs[frc->iu[i]]), 1, rs_i, dts_i, var_i, svh_i, nav_, newxyz_, opt_, 0, y_i, e_i, azel_i);


			rs_ref = mat(6, 1);            /* range to satellites */
			dts_ref = mat(2, 1);           /* satellite clock biases */
			y_ref = mat(opt_->nf * 2, 1);
			var_ref = mat(1, 1);
			e_ref = mat(3, 1);
			azel_ref = rtklib_zeros(2, 1);

			satposs(frc->time, &(frc->obs[frc->iu[ref_sat_]]), 1, nav_, opt_->sateph, rs_ref, dts_ref, var_ref, svh_ref);

			zdres_(0, &(frc->obs[frc->iu[ref_sat_]]), 1, rs_ref, dts_ref, var_ref, svh_ref, nav_, newxyz_, opt_, 0, y_ref, e_ref, azel_ref);



			VV(0)= base_measure + y_ref[f_] - y_i[f_];




			double Hx=0;
			if(H1)
			{
			    H1->resize(1,3);
			for (int k = 0; k < 3; k++) {
				(*H1)(k) = -e_ref[k] + e_i[k];  /* translation of innovation to position states */
				Hx += (*H1)(k) * X1(k);
			}
			}


			VV(0) *= (-1.0);




			if (frc->epoch_num == 884)
			{
				double bb = 0;
			}

			if (fabs(VV(0)) > 20)
			{
				double bb = 0;
			}

			free(rs_i); free(dts_i); free(y_i); free(var_i); free(e_i); free(azel_i);
			free(rs_ref); free(dts_ref); free(y_ref); free(var_ref); free(e_ref); free(azel_ref);
			/****************************************************************/

			return VV;
		}
	};


class RTK_ddres_Factor_L_dd : public NoiseModelFactor2<Vector3,Vector1>
	{
	public:
		//int type;
		int key_ambi;//
		fr_check *frc;
		const nav_t* nav_;
		const prcopt_t* opt_;
		double base_measure;
		int sat_index_, ref_sat_;
		int f_;//f<nf ? carrywave:pseudorange
		double ddres0, lami,ambi0;
		double H[3];
	public:
		RTK_ddres_Factor_L_dd() {};
		RTK_ddres_Factor_L_dd(int keypos, int key_ambi, fr_check *frc_, const nav_t* nav, const prcopt_t* opt, double base_measure,
			int sat_index, int ref_sat, int f, const SharedNoiseModel&  model,double ddres0,double lami,double ambi0,double *H_=NULL) :NoiseModelFactor2(model, keypos, key_ambi),
			base_measure(base_measure), frc(frc_), nav_(nav), opt_(opt), sat_index_(sat_index), ref_sat_(ref_sat), f_(f),
			ddres0(ddres0), lami(lami), ambi0(ambi0)
		{
			if(H_!=NULL)
			for (int i = 0; i < 3; i++)
			{
				H[i] = H_[i];
			}
		};
		~RTK_ddres_Factor_L_dd()
		{
			frc = NULL;
			nav_ = NULL;
			opt_ = NULL;
		};

			 Vector evaluateError(const Vector3& X1, const Vector1& X2,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const
		{
			Vector1 VV;
			double newxyz_[3];
			for (int k = 0; k < 3; k++)
			{
				newxyz_[k] = frc->xf[k]+ X1(k) ;//
			}


			int i = sat_index_;
			const obsd_t* obs_uref = &(frc->obs[frc->iu[i]]);
			double *rs_i, *dts_i, *var_i, *e_i, *azel_i, *y_i;
			double *rs_ref, *dts_ref, *var_ref, *e_ref, *azel_ref, *y_ref;
			int svh_i[1], svh_ref[1];

     //not reference satellite no residual calculation
			rs_i = mat(6, 1);            /* range to satellites */
			dts_i = mat(2, 1);           /* satellite clock biases */
			y_i = mat(opt_->nf * 2, 1);
			var_i = mat(1, 1);
			e_i = mat(3, 1);
			azel_i = rtklib_zeros(2, 1);
			//calculate single satellite position and clock drift
			satposs(frc->time, &(frc->obs[frc->iu[i]]), 1, nav_, opt_->sateph, rs_i, dts_i, var_i, svh_i);
			//单个卫星到接收机观测值的残差
			zdres_(0, &(frc->obs[frc->iu[i]]), 1, rs_i, dts_i, var_i, svh_i, nav_, newxyz_, opt_, 0, y_i, e_i, azel_i);

			/*reference satellite no residual calculation*/
			rs_ref = mat(6, 1);            /* range to satellites */
			dts_ref = mat(2, 1);           /* satellite clock biases */
			y_ref = mat(opt_->nf * 2, 1);
			var_ref = mat(1, 1);
			e_ref = mat(3, 1);
			azel_ref = rtklib_zeros(2, 1);
				//calculate single satellite position and clock drift
			satposs(frc->time, &(frc->obs[frc->iu[ref_sat_]]), 1, nav_, opt_->sateph, rs_ref, dts_ref, var_ref, svh_ref);
			//the residual of the range from the single satellite to receiver
			zdres_(0, &(frc->obs[frc->iu[ref_sat_]]), 1, rs_ref, dts_ref, var_ref, svh_ref, nav_, newxyz_, opt_, 0, y_ref, e_ref, azel_ref);


			VV(0) = base_measure + y_ref[f_] - y_i[f_];
			double lami_ = nav_->lam[frc->sat[i] - 1][f_%opt_->nf];
			if (f_ < opt_->nf)
			{
				VV(0) -= (lami_ * X2(0)+lami_ * ambi0);
			}

			/*H1、H2*/
			//H1:position measure matrix
			if(H1)
			{
			H1->resize(1,3);

			double Hx=0;
			for (int k = 0; k < 3; k++) {
				(*H1)(k)= -e_ref[k] + e_i[k];  /* translation of innovation to position states */
				Hx += (*H1)(k) * X1(k);
			}
	     	}

			VV(0)*= (-1.0);

			//H2:dd ambiguity bias measure matrix
			if(H2)
			{
			H2->resize(1,1);
			(*H2)(0) = lami;
			}

			if (frc->epoch_num == 884)
			{
				double bb = 0;
			}

			if (fabs(VV(0)) > 5)
			{
				double bb = 0;
			}

			free(rs_i); free(dts_i); free(y_i); free(var_i); free(e_i); free(azel_i);
			free(rs_ref); free(dts_ref); free(y_ref); free(var_ref); free(e_ref); free(azel_ref);
			/****************************************************************/



			return VV;
		}
	};

};
#endif // !1
