#pragma once
#ifndef RTK_DOPPLER_FACTOR
#define RTK_DOPPLER_FACTOR

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include "../rtk/rtk.h"

namespace gtsam
{
    /*single diffrence doppler measurement factor*/
	class RTK_doppler_sd_factor : public NoiseModelFactor1<Vector>//星间单差的多普勒观测因子
	{
	public:

		int n;//sat num of rover
		const nav_t* nav;
		const prcopt_t* opt;

		const obsd_t* obs;//rover obs
		double *rs;
		double *dts;
		double *var;
		int* svh;
		const ssat_t* ssat;

		double *res;//just for check
		int doppler_num;//just for check
		gtime_t time;
	public:
		RTK_doppler_sd_factor() {};
		RTK_doppler_sd_factor(Key keypos, const SharedNoiseModel& model, const nav_t* nav_, const prcopt_t* opt_,
			int nu, const obsd_t* obs_, double *rs_, double *dts_, int *svh_, double *var_,ssat_t *ssat_, double *res_, int doppler_num_ = 0, gtime_t time_ = {0}) :
			NoiseModelFactor1(model, keypos), nav(nav_), opt(opt_), n(nu),obs(obs_),ssat(ssat_), doppler_num(doppler_num_),time(time_)
		{
			//rs : nu * 6, dts : nu * 2, svh : nu
			//res:opt->nf*n - 1
			rs = mat(6, nu);
			dts = mat(2, nu);
			svh = imat(1, nu);
			var = mat(1, nu);
			memcpy(rs, rs_, nu * 6*sizeof(double));
			memcpy(dts, dts_, nu * 2*sizeof(double));
			memcpy(svh, svh_, nu*sizeof(int));
			memcpy(var, var_, nu * sizeof(double));

			if (res_ != NULL)
			{
				res = mat(1, opt->nf*nu - 1);
				memcpy(res, res_, (opt->nf*nu - 1) * sizeof(double));
			}
			else
				res = NULL;

		};

		~RTK_doppler_sd_factor()
		{
			nav = NULL;
			opt = NULL;
			obs = NULL;
			ssat = NULL;
			free(rs); free(dts); free(svh); free(var); free(res);


		};

		Vector evaluateError(const Vector& X1,
		boost::optional<Matrix&> H1 = boost::none) const
		{
			int nv = 0;
			int ny = opt->nf*n - 1;

			double *v = mat(ny, 1);
			double *var_doppler = mat(ny, 1);
			double *H = rtklib_zeros(X1.size(), ny);
            double *X1_=mat(X1.size(),1);

            for(int i=0;i<X1.size();i++)
            {
                X1_[i]=X1(i);
            }

			if (keys_[0] == 1)
			{
				double bb = 0;
			}

			if ((nv = resdop_mulfreq(obs, n, rs, dts, svh,var,
				nav, X1_, opt, X1_ + 3, ssat
				, v, var_doppler, H, 0, X1.size()))<1)
			{
                cout<<"doppler factor dim error 1!"<<endl;
				double error = 1;
			}
			if(nv!=doppler_num)
                {
                    cout<<"doppler factor dim error 2!"<<endl;
                }
			Vector VV(nv);
            for(int i=0;i<nv;i++)
            {
                VV(i)=-v[i];
            }

            if(H1)
            {

                H1->resize(nv,X1.size());
                if(nv!=doppler_num||H1->rows()<1||H1->cols()<1)
                {
                    cout<<"doppler factor dim error 3!"<<endl;
                }
                for (int i = 0; i < nv; i++)
                {
                    for (int j = 0; j < X1.size(); j++)
                    {
                        (*H1)(i,j) = H[j + i * X1.size()];
                    }
                }
            }

			free(X1_);
			free(v); free(var_doppler); free(H);
			return VV;
		}
	};
}
#endif // RTK_DOPPLER_FACTOR
