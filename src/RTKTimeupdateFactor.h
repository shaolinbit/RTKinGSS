#pragma once
#ifndef RTK_TIMEUPDATE_FACTOR_H
#define RTK_TIMEUPDATE_FACTOR_H
#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>

namespace gtsam {

    /*time update factor*/
	class RTKTimeupdateFactor : public NoiseModelFactor2<Vector,Vector>
	{
	public:
		double dt;
	public:
		RTKTimeupdateFactor() {};
		RTKTimeupdateFactor(Key key_pre, Key key_now, double dt_, const SharedNoiseModel& model)
			:NoiseModelFactor2(model, key_pre, key_now), dt(dt_){};

		Vector evaluateError(const Vector& X1, const Vector& X2,
		 boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const
		{
			Vector result(X1.size());
            Matrix F=Matrix::Identity(X1.size(), X1.size());
            for (int i = 0; i < (X1.size()-3); i++)
                {
                    F(i,i+3) = dt;
                }
                if (X1.size() >= 9)
                {
                    for (int i = 0; i < 3; i++) {
                        F(i,i+6) = dt * dt / 2.0;
                    }
                }

			if(H1)
			{
                H1->resize(X1.size(),X1.size());
                (*H1)=(-1)*F;
			}
            if(H2)
            {
                H2->resize(X1.size(),X1.size());
                H2->setIdentity();
            }

            result=X2-F*X1;
            return result;
		}
	};


}
#endif
