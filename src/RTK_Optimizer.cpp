#include "RTK_Optimizer.h"
#include "RTK_ddres_Factor.h"
#include "RTKTimeupdateFactor.h"
#include "RTK_doppler_factor.h"
#include <gtsam/nonlinear/Marginals.h>


int pre_ambi_size=0;
int valid_count = 0;
namespace gtsam{
Matrix Information_bias;


//add pva value
void RTKO_insertnewvalue_xc(int tcount, const double* x, int x_pva_num)
{
    Vector cb(x_pva_num);
    for(int i=0;i<x_pva_num;i++)
    {
        cb(i)=x[i];
    }
    /*if(tcount==0)
     cout<<"1:add keypair key=0"<<endl;*/
    RTKvalues.insert(tcount,cb);
}
//add pva priorfactor
void RTKO_add_Position_Priori(const fr_check* frc, NonlinearFactorGraph *Factors)
{
	int i;
	for (auto bff : *Factors)
	{
		if (bff->keys()[0] == frc->epoch_num)
			return ;
	}

	for (i = 0; i < frc->x_pva_num; i++)
	{
		if (frc->Pf[i]>5&&frc->stat> SOLQ_NONE&&RTKPriorFactor.size()>1)//&&frc->nv>0
		{
			return;
		}
	}

    Vector cbb(frc->x_pva_num);
	Vector nvq(frc->x_pva_num);
	for (i = 0; i < frc->x_pva_num; i++)
	{
		cbb(i) = frc->xf[i];
	}
    for (int i = 0; i < frc->x_pva_num; i++)
    {
        nvq(i) = sqrt(frc->Pf[i*frc->x_pva_num+i]);///2.0
        if (frc->stat!= SOLQ_FIX)
            nvq(i) *= 10;
    }

	noiseModel::Diagonal::shared_ptr Priornoise = noiseModel::Diagonal::Sigmas(nvq);
    Factors->emplace_shared<PriorFactor<Vector>>((int)frc->epoch_num, cbb, Priornoise);
}

//add double diff ambiguity value
void RTKO_insertnewvalue_xb(const int ambi_count, const double dd_bias)
{
	Key key = Symbol('b', ambi_count).key();
	Vector cb(1);
	cb(0)=dd_bias;
	RTKvalues.insert(key, cb);
}
//add double diff ambiguity priorfactor
void RTKO_add_ddambi_Priori(const int ambi_count, const ambi_infor* dd_ambi, NonlinearFactorGraph *Factors)
{
	Key key = Symbol('b', ambi_count).key();

	Vector cb(1);
	cb(0) = dd_ambi->bias;

	Vector nvq(1);
	nvq(0) = sqrt(dd_ambi->q);

	noiseModel::Diagonal::shared_ptr Priornoise = noiseModel::Diagonal::Sigmas(nvq);
    Factors->emplace_shared<PriorFactor<Vector1>>(key, cb, Priornoise);
}

//add Time Update Factor
void RTK_add_TimeUpdateFactor(int epoch_count_pre, gtime_t time_pre, int epoch_count_now, gtime_t time_now, NonlinearFactorGraph *Factors)
{
	double dt;
	dt = timediff(time_now, time_pre);
	/*if (dt > 5)
	{

		RTKO_add_Position_Priori(fr_c.at(fr_c.size() - 1), &RTKPriorFactor);

		return;
	}*/
	Vector sigma_Q(frc_dimX);

	for (int i = 0; i < 3; i++)
		sigma_Q(i) = 5 * dt;
	for (int i = 3; i < 6; i++)
		sigma_Q(i) = 10 * dt;
	if (frc_dimX > 6) {
		for (int i = 6; i < 9; i++)
			sigma_Q(i) = 10 * dt;
	}

	Values::iterator iter1 = RTKvalues.find(epoch_count_pre);
	Values::iterator iter2 = RTKvalues.find(epoch_count_now);
	if (iter1 == RTKvalues.end() || iter2 == RTKvalues.end())
	{
		cout<<"cannot find value when adding timeupdate factor."<<endl;
		return;
	}

    noiseModel::Diagonal::shared_ptr GN = noiseModel::Diagonal::Sigmas(sigma_Q);
	Factors->emplace_shared<RTKTimeupdateFactor>(epoch_count_pre, epoch_count_now, dt,GN);
}

//add single diffrence doppler measurement factor
void RTK_add_doppler_sd_factor(int timecount, fr_check* frc, const nav_t* nav, const prcopt_t* opt,int nu
	, double *rs, double *dts, int *svh, double *var, double *var_doppler,int nv,int y_num, NonlinearFactorGraph *Factors,double *res)
{
	if (y_num < 1)
	{
		return;
	}
	Vector sigma_doppler(y_num);
	for (int i = 0; i < y_num; i++)
	{
		sigma_doppler(i) = sqrt(var_doppler[nv + i]);
	}
	noiseModel::Diagonal::shared_ptr GN = noiseModel::Diagonal::Sigmas(sigma_doppler);
	Factors->emplace_shared<RTK_doppler_sd_factor>(timecount, GN, nav, opt, nu
		,frc->obs, rs, dts, svh, var,frc->ssat, res,y_num,frc->obs[0].time);
}

//add double diffrence pseudorange and carrier Factor
void RTK_add_ddresFactor_without_ekf(gtime_t time,rtk_t* rtk, fr_check* frc,int epoch_index, const nav_t* nav,
	const obsd_t* obs, double* rs, double* dts, double* var, int* svh, double* y,double *v, double* R)
{
	int i;
	int nf = rtk->opt.ionoopt == IONOOPT_IFLC ? 1 : rtk->opt.nf;
	prcopt_t* opt = &rtk->opt;
	for (i = 0; i < rtk->nv; i++)
	{
		int refsat_index, sat_index, code, f,frq;
		refsat_index = rtk->sat_index[i][0];
		sat_index = rtk->sat_index[i][1];
		code = rtk->sat_index[i][2];
		f = rtk->sat_index[i][3];
		frq = f % nf;

		double data = sqrt(R[ i +  i * rtk->nv]);
        Vector1 sigma_dd;
        sigma_dd(0)=data;
        noiseModel::Diagonal::shared_ptr GN=noiseModel::Diagonal::Sigmas(sigma_dd);

		double base_measure;//single diffrence between satellites of base station
		base_measure= - y[f + rtk->ir[refsat_index] * nf * 2] + y[f + rtk->ir[sat_index] * nf * 2];
		if (code)//pseduorange
		{
			std::shared_ptr<RTK_ddres_Factor_P_without_ekf> ddpf= std::make_shared<RTK_ddres_Factor_P_without_ekf>(epoch_index, GN, nav, opt
				, &(frc->obs[rtk->iu[refsat_index]]), &(frc->obs[rtk->iu[sat_index]]), base_measure, f, rs + rtk->iu[refsat_index] * 6, rs + rtk->iu[sat_index] * 6
				, dts + rtk->iu[refsat_index] * 2, dts + rtk->iu[sat_index] * 2, var[rtk->iu[refsat_index]], var[rtk->iu[sat_index]]
				, svh[rtk->iu[refsat_index]], svh[rtk->iu[sat_index]], v[i]);
            RTKddresFactor_P.push_back(ddpf);
		}
		else//carrier
		{
			Key key_ambi;
			double lami = nav->lam[rtk->sat[sat_index] - 1][frq];
			if(flag_global)
			{
                key_ambi = Symbol('b', d_ambi.at(frc->bias_index[i])->key).key();
                std::shared_ptr<RTK_ddres_Factor_L_dd_without_ekf> ddLf=std::make_shared<RTK_ddres_Factor_L_dd_without_ekf>(epoch_index, key_ambi, GN, nav, opt
                    , &(frc->obs[rtk->iu[refsat_index]]), &(frc->obs[rtk->iu[sat_index]]), base_measure, lami, f, rs + rtk->iu[refsat_index] * 6, rs + rtk->iu[sat_index] * 6
                    , dts + rtk->iu[refsat_index] * 2, dts + rtk->iu[sat_index] * 2, var[rtk->iu[refsat_index]], var[rtk->iu[sat_index]], svh[rtk->iu[refsat_index]], svh[rtk->iu[sat_index]]
                    , 0, v[i], rtk->dd_bias[i]);
                RTKddresFactor_L.push_back(ddLf);
			}
			else
			{
                key_ambi = Symbol('b', 0).key();
                std::shared_ptr<RTK_ddres_Factor_L_dd_without_ekf> ddLf=std::make_shared<RTK_ddres_Factor_L_dd_without_ekf>(epoch_index, key_ambi, GN, nav, opt
                    , &(frc->obs[rtk->iu[refsat_index]]), &(frc->obs[rtk->iu[sat_index]]), base_measure, lami, f, rs + rtk->iu[refsat_index] * 6, rs + rtk->iu[sat_index] * 6
                    , dts + rtk->iu[refsat_index] * 2, dts + rtk->iu[sat_index] * 2, var[rtk->iu[refsat_index]], var[rtk->iu[sat_index]], svh[rtk->iu[refsat_index]], svh[rtk->iu[sat_index]]
                    , frc->bias_index[i], v[i], rtk->dd_bias[i]);
                RTKddresFactor_L.push_back(ddLf);
			}

		}
	}


}
//add double diff ambiguities value and prior factor in a vector
void RTKO_insert_newValuePirorFactor_xb_vector(vector<ambi_infor*> d_ambi, NonlinearFactorGraph *Factors)
{
	Key key = Symbol('b', 0).key();
	int ambi_num = (int)d_ambi.size();

	Vector x(ambi_num);
	Vector prior(ambi_num);
	Vector q(ambi_num);
	for (int i = 0; i < ambi_num; i++)
	{
		x(i) = d_ambi.at(i)->bias;
		prior(i)= d_ambi.at(i)->bias;
		q(i) =  sqrt(d_ambi.at(i)->q);
	}
	RTKvalues.insert(key, x);

	noiseModel::Diagonal::shared_ptr noise=noiseModel::Diagonal::Sigmas(q);
	Factors->emplace_shared<PriorFactor<Vector>>(key, prior, noise);


}
//slid window  GSS optimization
void RTKO_GenerateTrajectory_dd_slidwin_ambivector_fix(rtk_t* rtk,fr_check *frc,int timecount, int win_size, FILE *fp)//const nav_t* nav, const prcopt_t* opt,
{
    cout<<"1 GenerateTrajectory:timecount="<<timecount<<".  "<<endl;
	if (key_pos.size() < 2 || rtk->nv <= 0)
	{
		return;
	}
	int i,j,k;
	//dd ambiguity value and priorfactor
	if ( (pre_ambi_size==0&& d_ambi.size() > 0))
	{

		RTKO_insert_newValuePirorFactor_xb_vector(d_ambi, &RTKPriorFactor);
		pre_ambi_size = d_ambi.size();
	}
	else
	{
		//add new ambiguities into value and prior
		if (pre_ambi_size != d_ambi.size())
		{
			Key key_ambi = Symbol('b', 0).key();
			Values::iterator valuesrase = RTKvalues.find(key_ambi);
			Matrix Qb_new=zeros(d_ambi.size(), d_ambi.size());
			Vector bias_new(d_ambi.size());
			Vector bias_prior(d_ambi.size());


			for (int i = 0; i < pre_ambi_size; i++)
			{
				bias_new(i) = valuesrase->value.cast<Vector>()(i);//这样写对吗
				for (int j = 0; j < pre_ambi_size; j++)
				{
					Qb_new(i,j) = Information_bias(i,j);

				}
			}
			for (int index = pre_ambi_size; index < d_ambi.size(); index++)
			{
				bias_new(index) = d_ambi.at(index)->bias;
				Qb_new(index,index) = 1.0 / d_ambi.at(index)->q;
			}
			bias_prior=bias_new;
			//delete
			if (valuesrase != RTKvalues.end())
			{
				RTKvalues.erase(key_ambi);
			}
			//delete ambi prior
			if (FactorDelete(key_ambi, &RTKPriorFactor))
			{
				//double bb = 0;
			}
			RTKvalues.insert(key_ambi, bias_new);

			noiseModel::Gaussian::shared_ptr noise=noiseModel::Gaussian::Information(Qb_new);
			RTKPriorFactor.emplace_shared<PriorFactor<Vector>>(key_ambi, bias_prior, noise);
			//rtk->sol.stat = SOLQ_FLOAT;
		}
		else
		{
		//	if(fr_c.at(fr_c.size()-2)->stat== SOLQ_FIX)
		//		rtk->sol.stat = SOLQ_FIX;
		}
		pre_ambi_size = d_ambi.size();
	}
	//GSS optimization
	if (key_pos.size() >= 2 &&rtk->nv>0)//&& move_flag&&fix_count <10
	{
        NonlinearFactorGraph RTKFactors;
		//RTKFactors.clear();
		insertRTKFactors(&RTKFactors, &RTKPriorFactor);
		//insertRTKFactors(&RTKFactors, &RTKddresFactor);
		insertRTKFactors(&RTKFactors, &RTKddresFactor_L);
		insertRTKFactors(&RTKFactors, &RTKddresFactor_P);
		insertRTKFactors(&RTKFactors, &RTKDopplerFactor);
		insertRTKFactors(&RTKFactors, &RTKPreFactor);

		GaussNewtonParams parameters;
        parameters.setVerbosity("ERROR");
        parameters.setMaxIterations(20);
        parameters.setLinearSolverType("MULTIFRONTAL_CHOLESKY");

		GaussNewtonOptimizer optimizer(RTKFactors, RTKvalues);

		double olderror = 0, newerror = 0;
		olderror = optimizer.error();
		Values RTK_Optimize_result = optimizer.optimize();
		newerror = optimizer.error();
		if (newerror > olderror)
		{
			rtk->sol.stat = SOLQ_NONE;
			frc->stat = rtk->sol.stat;
			cout<<"timecount="<<timecount<<", error:newerror > olderror!!!!!"<<endl;
			return;
		}
        RTKvalues.clear();
        for (auto fb : RTK_Optimize_result)
        {
            RTKvalues.insert(fb.key,Vector(RTK_Optimize_result.at(fb.key).cast<Vector>()));
        }
		update_slidwin_vector(rtk, frc, timecount, win_size);//update
		frc->stat = rtk->sol.stat;
	}

	if (key_pos.size() >= win_size)
	{
		delete_outsize_factorvalue_vector(win_size);
	}
}

void update_slidwin_vector(rtk_t* rtk, fr_check *frc, int timecount, int win_size)
{
    //cout<<"2 update:timecount="<<timecount<<".  "<<endl;
	int i, j;
	NonlinearFactorGraph RTKFactors;

    insertRTKFactors(&RTKFactors, &RTKPriorFactor);
    //insertRTKFactors(&RTKFactors, &RTKddresFactor);
    insertRTKFactors(&RTKFactors, &RTKddresFactor_L);
    insertRTKFactors(&RTKFactors, &RTKddresFactor_P);
    insertRTKFactors(&RTKFactors, &RTKDopplerFactor);
    insertRTKFactors(&RTKFactors, &RTKPreFactor);

	boost::shared_ptr<GaussianFactorGraph> graph_=RTKFactors.linearize(RTKvalues);
	boost::shared_ptr<GaussianBayesTree> bayesTree_=
    graph_->eliminateMultifrontal(boost::none, EliminatePreferCholesky);


	Key key_ambi = Symbol('b', 0).key();
	Values::iterator bias_value = RTKvalues.find(key_ambi);
	if (bias_value != RTKvalues.end())
	{
		Vector bias_data=RTKvalues.at(key_ambi).cast<Vector>();
        //Matrix Qi=bayesTree_->marginalFactor(key_ambi, EliminatePreferCholesky)->information();

		//cout<<"Mariginal 2"<<endl;
		boost::shared_ptr<GaussianFactorGraph> graph_new=RTKFactors.linearize(RTKvalues);
        boost::shared_ptr<GaussianBayesTree> bayesTree_new=
        graph_new->eliminateMultifrontal(boost::none, EliminatePreferCholesky);

		//update ambiguity-----------------------------------------------------------------------------------
        Matrix bias_infor=bayesTree_new->marginalFactor(key_ambi, EliminatePreferCholesky)->information();
		Information_bias.resize(bias_infor.rows(),bias_infor.cols());
		Information_bias=bias_infor;


		int n_ambi = 0;
		for (i = 0; i < frc->nv; i++)
		{
			if (((rtk->vflg[i] >> 4) & 0xF) == 1)
				continue;
			int nf = rtk->opt.ionoopt == IONOOPT_IFLC ? 1 : rtk->opt.nf;
			int refsat_index, sat_index, code, f, frq;
			refsat_index = rtk->sat_index[i][0];
			sat_index = rtk->sat_index[i][1];
			code = rtk->sat_index[i][2];
			f = rtk->sat_index[i][3];
			frq = f % nf;
			//双差和单差之间索引的关系
			int index_ref = 9 + MAXSAT * f + rtk->sat[refsat_index] - 1;
			int index_uref = 9 + MAXSAT * f + rtk->sat[sat_index] - 1;

			d_ambi.at(frc->bias_index[i])->bias = bias_data(frc->bias_index[i]);
		}

	}
	//update pva in rtklib----------------------------------------------------------------------------------------------
	Vector Xi=RTKvalues.at(timecount).cast<Vector>();
	for (i = 0; i < frc_dimX; i++)
	{
		/*for (j = 0; j < frc_dimX; j++)
		{
			rtk->P[i + j * rtk->nx] = Qx.data[i*Qx.prd + j];
		}*/
		rtk->x[i] = Xi(i);
		frc->xf[i] = Xi(i);
	}
	//-------------------------------------------------------------------------------------------------------

}

void delete_outsize_factorvalue_vector(int win_size)
{
    //cout<<"3 delete:timecount="<<key_pos.at((int)key_pos.size()-1)<<".  "<<endl;
	NonlinearFactorGraph RTKFactors;

	insertRTKFactors(&RTKFactors, &RTKPriorFactor);
	//insertRTKFactors(&RTKFactors, &RTKddresFactor);
	insertRTKFactors(&RTKFactors, &RTKddresFactor_L);
	insertRTKFactors(&RTKFactors, &RTKddresFactor_P);
	insertRTKFactors(&RTKFactors, &RTKDopplerFactor);
	insertRTKFactors(&RTKFactors, &RTKPreFactor);

	Key key_ambi = Symbol('b', 0).key();

    boost::shared_ptr<GaussianFactorGraph> graph_=RTKFactors.linearize(RTKvalues);
    boost::shared_ptr<GaussianBayesTree> bayesTree_=
    graph_->eliminateMultifrontal(boost::none, EliminatePreferCholesky);

    //update anbiguity value and priorfactor,update ambiguity index in double diffrence carrier factor
    if (RTKvalues.find(key_ambi) != RTKvalues.end()) {
        //delete old ambiguity priorfactor
		Matrix Qb = bayesTree_->marginalFactor(key_ambi, EliminatePreferCholesky)->information();
		if (FactorDelete(key_ambi, &RTKPriorFactor))
        {
            double bb = 0;
        }
        //add new ambi priorfactor
        noiseModel::Gaussian::shared_ptr Priornoise=noiseModel::Gaussian::Information(Qb);
        Vector bias_prior0=RTKvalues.at(key_ambi).cast<Vector>();
        RTKPriorFactor.emplace_shared<PriorFactor<Vector>>(key_ambi, bias_prior0, Priornoise);

		//if the size of ambiguity vector changed, update anbiguity value , priorfactor,ambiguity index
		int old_key, i;
		int d_ambi_count = 0;
		int first_key = key_pos.at(key_pos.size() - win_size + 1);
		int *new_to_old_index = imat(d_ambi.size(), 1);
		for (i = 0; i < d_ambi.size(); i++)
		{
			if ((d_ambi.at(i)->epoch_s + d_ambi.at(i)->num - 1) < first_key)
			{
				//update ambiguity index in double diffrence carrier factor
                for (int j = 0; j < RTKddresFactor_L.size(); j++)
				{
					auto ddf = RTKddresFactor_L.at(j);
					if (ddf->keys().size() == 2)
					{
                        if (ddf->ambi_index > i)
						{
							ddf->ambi_index--;
						}
					}
				}
				delete(d_ambi[i]);
				d_ambi.erase(d_ambi.begin() + i);
				bias_flg.erase(bias_flg.begin() + i);
				i--;
			}
			else
			{
				new_to_old_index[i] = d_ambi_count;
			}
			d_ambi_count++;
		}

		//delete ambi value
		if (d_ambi.size() == 0)
		{
			Values::iterator valuesrase = RTKvalues.find(key_ambi);
			//delete
			if (valuesrase != RTKvalues.end())
			{
				RTKvalues.erase(key_ambi);
			}
			//delete ambi prior
			if (FactorDelete(key_ambi, &RTKPriorFactor))
			{
				double bb = 0;
			}

		}
		else if (d_ambi_count > i)
		{
			Information_bias.resize(d_ambi.size(), d_ambi.size());
			Values::iterator valuesrase = RTKvalues.find(key_ambi);
			Matrix Qb_new(d_ambi.size(), d_ambi.size());
			Vector bias_new(d_ambi.size());
			Vector bias_prior(d_ambi.size());

			for (int i = 0; i < d_ambi.size(); i++)
			{
				bias_new(i) = valuesrase->value.cast<Vector>()(new_to_old_index[i]);
				for (int j = 0; j < d_ambi.size(); j++)
				{
					Qb_new(i,j) = Qb(new_to_old_index[i],new_to_old_index[j]);
					Information_bias(i,j) = Qb(new_to_old_index[i],new_to_old_index[j]);
				}
			}
			bias_prior = bias_new;
			//delete
			if (valuesrase != RTKvalues.end())
			{
				RTKvalues.erase(key_ambi);
			}
			//delete ambi prior
			if (FactorDelete(key_ambi, &RTKPriorFactor))
			{
				double bb = 0;
			}
			RTKvalues.insert(key_ambi, bias_new);
			noiseModel::Gaussian::shared_ptr noise=noiseModel::Gaussian::Information(Qb_new);
			RTKPriorFactor.emplace_shared<PriorFactor<Vector>>(key_ambi, bias_prior, noise);
		}
		std::free(new_to_old_index);
	}
	pre_ambi_size = d_ambi.size();

	//delete factors values out of silding window
    while (key_pos.size() >= win_size)
	{
		//if (FactorDelete(key_pos.at(0), &RTKddresFactor))
		if (FactorDelete(key_pos.at(0), &RTKddresFactor_P)&&FactorDelete(key_pos.at(0), &RTKddresFactor_L))
		{
			FactorDelete(key_pos.at(0), &RTKDopplerFactor);
			FactorDelete(key_pos.at(0), &RTKPreFactor);
			FactorDelete(key_pos.at(0), &RTKPriorFactor);
			//delete old value
			Values::iterator valuesrase = RTKvalues.find(Key(key_pos.at(0)));
			if (valuesrase != RTKvalues.end())
			{
				RTKvalues.erase(key_pos.at(0));
			}
			key_pos.erase(key_pos.begin());
			for (int i = 0; i < fr_c.size(); i++)
			{
				vector<int>::iterator iter = std::find(key_pos.begin(), key_pos.end(), fr_c.at(i)->epoch_num);

				if (iter == key_pos.end())
				{
					delete(fr_c[i]);
					fr_c.erase(fr_c.begin() + i);
				}
			}
		}

	}

	//update priorfactor
	int i = 0;
	NonlinearFactorGraph::iterator factorsrase = RTKPriorFactor.begin();
	NonlinearFactorGraph RTKPriorFactor_update;
	for (auto bff : RTKPriorFactor)
	{
        Key key = bff->keys()[0];
		if (key< key_ambi)
		{
			RTKPriorFactor.erase(factorsrase);
			factorsrase = RTKPriorFactor.begin() + i;

			Matrix Qi = bayesTree_->marginalFactor(key, EliminatePreferCholesky)->information();
			Vector prior_new=RTKvalues.at(key).cast<Vector>();
			noiseModel::Gaussian::shared_ptr Priornoise_new=noiseModel::Gaussian::Information(Qi);
            RTKPriorFactor_update.emplace_shared<PriorFactor<Vector>>(key, prior_new, Priornoise_new);

			break;

		}
		factorsrase++;
		i++;
	}

	insertRTKFactors(&RTKPriorFactor, &RTKPriorFactor_update);

}

void RTKO_GenerateTrajectory_dd_section(FILE *fp)
{

    cout<<"Global GSSM FGO ......             "<<endl;
	int i, j, k;

	//add ambiguity values and priorfactors
	for (i = pre_ambi_size; i < d_ambi.size(); i++)
	{
		RTKO_insertnewvalue_xb(d_ambi.at(i)->key, d_ambi.at(i)->bias);
		RTKO_add_ddambi_Priori(d_ambi.at(i)->key, d_ambi.at(i), &RTKPriorFactor);
	}

	if (key_pos.size() >= 2)
	{

        NonlinearFactorGraph RTKFactors;
		//RTKFactors.clear();
		insertRTKFactors(&RTKFactors, &RTKPriorFactor);
		//insertRTKFactors(&RTKFactors, &RTKddresFactor);
		insertRTKFactors(&RTKFactors, &RTKddresFactor_L);
		insertRTKFactors(&RTKFactors, &RTKddresFactor_P);
		insertRTKFactors(&RTKFactors, &RTKDopplerFactor);
		insertRTKFactors(&RTKFactors, &RTKPreFactor);

		GaussNewtonParams parameters;
        parameters.setVerbosity("ERROR");
        parameters.setMaxIterations(20);
        parameters.setLinearSolverType("MULTIFRONTAL_CHOLESKY");

		GaussNewtonOptimizer optimizer(RTKFactors, RTKvalues);

		double olderror = 0, newerror = 0;
		olderror = optimizer.error();
		Values RTK_Optimize_result = optimizer.optimize();
		newerror = optimizer.error();
		if (newerror > olderror)
		{
			cout<<"global GSS error:newerror > olderror!!!!!"<<endl;
			return;
		}
        RTKvalues.clear();
        for (auto fb : RTK_Optimize_result)
        {
            RTKvalues.insert(fb.key,Vector(RTK_Optimize_result.at(fb.key).cast<Vector>()));
        }


		FILE* fp = fopen((char *)GSS_global_result_file.c_str(), "w");
		fprintf(fp,"week\tsecond(s)\tlat(deg)\tlon(deg)\th(m)\tv_x(m/s)\tv_y(m/s)\tv_z(m/s)\tstat\toutputflag\n");
		for (i = 0; i < fr_c.size(); i++)
		{
		//week second(s) lat(deg) lon(deg) h(m) stat nfix ns nv move_flag v_x(m/s) v_y(m/s) v_z(m/s) ax(m/s^2) ay(m/s^2) az(m/s^2) outputflag
			Values::iterator pva_value=RTK_Optimize_result.find(fr_c.at(i)->epoch_num);
			int week;
			double sec;
			sec = time2gpst(fr_c.at(i)->time, &week);
			fprintf(fp, "%d\t%f\t", week, sec);

			double pos[3], blh[3];

			if (pva_value != RTK_Optimize_result.end())
			{

                Vector x_data=pva_value->value.cast<Vector>();
				for (k = 0; k < 3; k++)
				{
					pos[k] = x_data(k);
				}
				ecef2pos(pos, blh);
				fprintf(fp, "%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%d\t%d\t", blh[0] * Rad_Deg, blh[1] * Rad_Deg, blh[2]
					, x_data(3), x_data(4), x_data(5), fr_c.at(i)->stat, 1);
			}
			else
			{
                //cout<<"cannot find key="<<fr_c.at(i)->epoch_num<<"in values!"<<endl;
				for (k = 0; k < 3; k++)
				{
					pos[k] = fr_c.at(i)->xf[k];
				}
				ecef2pos(pos, blh);
				fprintf(fp, "%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%d\t%d\t", blh[0] * Rad_Deg, blh[1] * Rad_Deg, blh[2]
					, fr_c.at(i)->xf[3], fr_c.at(i)->xf[4], fr_c.at(i)->xf[5], fr_c.at(i)->stat, 0);
			}

			fprintf(fp, "\n");
		}
		fclose(fp);
	}

}
//delete old factor
int FactorDelete(Key key, NonlinearFactorGraph *Factors)
{
	int i = 0;
	NonlinearFactorGraph::iterator factorsrase = Factors->begin();
	for (auto bff : *Factors)
	{
		if (bff->keys()[0] == key)
		{
			Factors->erase(factorsrase);
			factorsrase = Factors->begin() + i;
			break;

		}
		factorsrase++;
		i++;
	}

	if (factorsrase == Factors->end())
		return 1;
	else
		return 0;
}
int FactorDelete(Key key, vector<shared_ptr<RTK_ddres_Factor_L_dd_without_ekf>> *Factors)
{
	int i = 0;
	vector<shared_ptr<RTK_ddres_Factor_L_dd_without_ekf>>::iterator factorsrase = Factors->begin();
	for (auto bff : *Factors)
	{
		if (bff->keys()[0] == key)
		{
			Factors->erase(factorsrase);
			factorsrase = Factors->begin() + i;
			break;

		}
		factorsrase++;
		i++;
	}

	if (factorsrase == Factors->end())
		return 1;
	else
		return 0;
}
int FactorDelete(Key key, vector<shared_ptr<RTK_ddres_Factor_P_without_ekf>> *Factors)
{
	int i = 0;
	vector<shared_ptr<RTK_ddres_Factor_P_without_ekf>>::iterator factorsrase = Factors->begin();
	for (auto bff : *Factors)
	{
		if (bff->keys()[0] == key)
		{
			Factors->erase(factorsrase);
			factorsrase = Factors->begin() + i;
			break;

		}
		factorsrase++;
		i++;
	}

	if (factorsrase == Factors->end())
		return 1;
	else
		return 0;
}
void insertRTKFactors(NonlinearFactorGraph *FactorA, NonlinearFactorGraph *FactorB)
{
	for (auto bff : *FactorB)
	{
		FactorA->push_back(bff);
	}
}

void insertRTKFactors(NonlinearFactorGraph *FactorA, vector<shared_ptr<RTK_ddres_Factor_L_dd_without_ekf>> *FactorB)
{
	for (auto bff : *FactorB)
	{
		FactorA->push_back(*bff);
	}
}
void insertRTKFactors(NonlinearFactorGraph *FactorA, vector<shared_ptr<RTK_ddres_Factor_P_without_ekf>> *FactorB)
{
	for (auto bff : *FactorB)
	{
		FactorA->push_back(*bff);
	}
}

};
