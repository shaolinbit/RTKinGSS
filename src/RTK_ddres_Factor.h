#ifndef RTK_DDRES_FACTOR_H
#define RTK_DDRES_FACTOR_H
#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include "../rtk/rtk.h"

namespace gtsam
{
	/**
	 * A class for a rtk ddres
	 * @addtogroup INS
	 */

	/*doble diffrence carrier measurement factor*/
	class RTK_ddres_Factor_L_dd_without_ekf : public NoiseModelFactor2<Vector,Vector>
	{
	public:

		double ddres0, ambi0;//just for check,if there is no error,delete
		double H[3];//just for check,if there is no error,delete


		const nav_t* nav_;//
		const prcopt_t* opt_;//some option
		const obsd_t* obs_ref, *obs_i;//observation of refrence satellite,observation of non-refrence satellite
		double base_measure;//single diff between sat of base station:- y[f + ir[i] * nf * 2]+y[f + ir[j] * nf * 2]
		// v[nv] = (y[f + iu[i] * nf * 2] - y[f + ir[i] * nf * 2]) - (y[f + iu[j] * nf * 2] - y[f + ir[j] * nf * 2]);
		double lami;//wave length
		int f_;//f<nf ? carrier:code
		double rs_ref[6], rs_i[6];//sat pos and vel[x,y,z,vx,vy,vz]
		double dts_ref[2], dts_i[2];//sat time {bias,drift} (s|s/s)
		double var_ref, var_i;//position and clock error variance (m^2)
		int svh_ref, svh_i;//health flag
		int ambi_index;
	public:
		RTK_ddres_Factor_L_dd_without_ekf() {};
		RTK_ddres_Factor_L_dd_without_ekf(Key keypos, Key key_ambi,  const SharedNoiseModel& model, const nav_t* nav, const prcopt_t* opt
			, const  obsd_t* obs_ref_, const obsd_t*obs_i_, double base_measure, double lami_, int f, double *rs_ref_, double *rs_i_
			, double *dts_ref_, double *dts_i_, double var_ref_, double var_i_, int svh_ref_, int svh_i_
			, int ambiindex=0,  double ddres0 = 0, double ambi0 = 0, double *H_ = NULL) :
			NoiseModelFactor2(model, keypos, key_ambi), nav_(nav), opt_(opt), obs_ref(obs_ref_), obs_i(obs_i_),
			base_measure(base_measure), lami(lami_), f_(f), var_ref(var_ref_), var_i(var_i_), svh_ref(svh_ref_), svh_i(svh_i_),
			ddres0(ddres0), ambi0(ambi0),ambi_index(ambiindex)
		{
			if (H_ != NULL)
				for (int i = 0; i < 3; i++)
				{
					H[i] = H_[i];
				}
			for (int i = 0; i < 6; i++)
			{
				rs_ref[i] = rs_ref_[i];
				rs_i[i] = rs_i_[i];
			}
			for (int i = 0; i < 2; i++)
			{
				dts_ref[i] = dts_ref_[i];
				dts_i[i] = dts_i_[i];

			}
		};
		~RTK_ddres_Factor_L_dd_without_ekf()
		{

			nav_ = NULL;
			opt_ = NULL;
			obs_ref = NULL;
			obs_i = NULL;
		};

        Vector evaluateError(const Vector& X1, const Vector& X2,
			boost::optional<Matrix&> H1 = boost::none,
            boost::optional<Matrix&> H2 = boost::none) const
		{
			Vector1 VV;

			double  *e_i, *azel_i, *y_i;
			double  *e_ref, *azel_ref, *y_ref;
			double *X1_=mat(X1.size(),1);

			for(int i=0;i<X1.size();i++)
			{
                X1_[i]=X1(i);
			}

			/*non ref sat zero res*/
			y_i = mat(opt_->nf * 2, 1);
			e_i = mat(3, 1);
			azel_i = rtklib_zeros(2, 1);
			zdres_(0, obs_i, 1, rs_i, dts_i, &var_i, &svh_i, nav_, X1_, opt_, 0, y_i, e_i, azel_i);
			/*ref sat zero res*/
			y_ref = mat(opt_->nf * 2, 1);
			e_ref = mat(3, 1);
			azel_ref = rtklib_zeros(2, 1);
			zdres_(0, obs_ref, 1, rs_ref, dts_ref, &var_ref, &svh_ref, nav_, X1_, opt_, 0, y_ref, e_ref, azel_ref);



			if (ambi_index >= X2.size())
			{
                cout<<"ddres L factor dim error 1!"<<endl;
				double bb = 0;
			}
			VV(0)= base_measure + y_ref[f_] - y_i[f_];

			if (f_ < opt_->nf)
			{
                VV(0) -= (lami * X2(ambi_index));
			}
			VV(0) *= (-1.0);

			/*H1¡¢H2*/
			if(H1)
			{
                H1->resize(1,X1.size());
                H1->setZero();

                for (int k = 0; k < 3; k++) {
                    (*H1)(k) = -e_ref[k] + e_i[k];
                }
			}
			if(H2)
			{
                if(X2.size()<1)
                {
                    cout<<"ddres L factor dim error 2!"<<endl;
                    double bb=0;
                }
                H2->resize(1,X2.size());
                H2->setZero();
                (*H2)(ambi_index) = lami;
			}

			if (fabs(VV(0)) > 5)
			{
				double bb = 0;
			}

			free(X1_);
			free(y_i); free(e_i); free(azel_i);
			free(y_ref); free(e_ref); free(azel_ref);
			/****************************************************************/
			return VV;
		}
	};

    /*doble diffrence pseduorange measurement factor*/
	class RTK_ddres_Factor_P_without_ekf : public NoiseModelFactor1<Vector>
	{
	public:
		double ddres0;


		const nav_t* nav_;
		const prcopt_t* opt_;
		const obsd_t* obs_ref, *obs_i;//observation of refrence satellite,observation of non-refrence satellite
		double base_measure;

		int f_;
		double rs_ref[6], rs_i[6];//sat [x,y,z,vx,vy,vz]
		double dts_ref[2], dts_i[2];//time{bias,drift} (s|s/s)
		double var_ref, var_i;//position and clock error variance (m^2)
		int svh_ref, svh_i;//health flag
	public:
		RTK_ddres_Factor_P_without_ekf() {};
		RTK_ddres_Factor_P_without_ekf(Key keypos, const SharedNoiseModel& model, const nav_t* nav, const prcopt_t* opt,
			const obsd_t* obs_ref_, const obsd_t*obs_i_, double base_measure,int f, double *rs_ref_, double *rs_i_, double *dts_ref_
			, double *dts_i_, double var_ref_, double var_i_, int svh_ref_, int svh_i_,double ddres0) :
			NoiseModelFactor1(model, keypos),nav_(nav), opt_(opt), obs_ref(obs_ref_), obs_i(obs_i_),
			base_measure(base_measure) ,  f_(f), var_ref(var_ref_), var_i(var_i_), svh_ref(svh_ref_), svh_i(svh_i_),
			ddres0(ddres0)
		{
			for (int i = 0; i < 6; i++)
			{
				rs_ref[i] = rs_ref_[i];
				rs_i[i] = rs_i_[i];
			}
			for (int i = 0; i < 2; i++)
			{
				dts_ref[i] = dts_ref_[i];
				dts_i[i] = dts_i_[i];

			}
		};

		~RTK_ddres_Factor_P_without_ekf()
		{

			nav_ = NULL;
			opt_ = NULL;
			obs_ref = NULL;
			obs_i = NULL;
		};

		Vector evaluateError(const Vector& X1,
			boost::optional<Matrix&> H1 = boost::none) const
		{
			Vector1 VV;
			double  *e_i, *azel_i, *y_i;
			double  *e_ref, *azel_ref, *y_ref;
			double *X1_=mat(X1.size(),1);

			for(int i=0;i<X1.size();i++)
			{
                X1_[i]=X1(i);
			}


			/*non reference satellite zero residual*/
			y_i = mat(opt_->nf * 2, 1);
			e_i = mat(3, 1);
			azel_i = rtklib_zeros(2, 1);
			zdres_(0, obs_i, 1, rs_i, dts_i, &var_i, &svh_i, nav_, X1_, opt_, 0, y_i, e_i, azel_i);
			/*reference satellite zero residual*/
			y_ref = mat(opt_->nf * 2, 1);
			e_ref = mat(3, 1);
			azel_ref = rtklib_zeros(2, 1);
			zdres_(0, obs_ref, 1, rs_ref, dts_ref, &var_ref, &svh_ref, nav_, X1_, opt_, 0, y_ref, e_ref, azel_ref);

			VV(0) = base_measure + y_ref[f_] - y_i[f_];
			VV(0) *= (-1.0);

			if(H1)
			{
                H1->resize(1,X1.size());
                H1->setZero();
                for (int k = 0; k < 3; k++) {
                    (*H1)(k) = -e_ref[k] + e_i[k];}
			}


			if (fabs(VV(0)) > 5)
			{
				double bb = 0;
			}

			free(X1_);
			free(y_i); free(e_i); free(azel_i);
			free(y_ref); free(e_ref); free(azel_ref);
			/****************************************************************/

			return VV;
		}
	};

}
#endif // !1
