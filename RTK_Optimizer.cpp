#include "RTK_Optimizer.h"
#include "RTK_ddres_Factor.h"

namespace gtsam
{
void RTKO_GenerateTrajectory_dd(const nav_t* nav,const prcopt_t* opt)
{
	int i, j, k;

	for (i = 0; i < d_ambi.size(); i++)
	{
		RTKO_insertnewvalue_xb(d_ambi.at(i)->key, d_ambi.at(i)->bias);//+
		RTKO_add_ddambi_Priori(d_ambi.at(i)->key, d_ambi.at(i));//
	}
	for (i = 0; i < fr_c.size(); i++)
	{
		if (fr_c.at(i)->stat <= 0)
		{
			valid_count = 0;
			continue;
		}
		else if (fr_c.at(i)->stat == 1)
			valid_count++;
		else
			valid_count = 0;

		RTKO_insertnewvalue_xc(fr_c.at(i)->epoch_num, fr_c.at(i)->xf,fr_c.at(i));//
		if(valid_count>5)
			RTKO_add_Position_Priori(fr_c.at(i));//
		RTKO_add_ddres_Factor(fr_c.at(i), nav, opt);
	}

	GaussNewtonParams parameters;               //test
    parameters.setVerbosity("ERROR");
    parameters.setMaxIterations(20);
    parameters.setLinearSolverType("MULTIFRONTAL_CHOLESKY");


	GaussNewtonOptimizer optimizer(RTKFactors, RTKvalues,parameters);

	//optimizer.params_.setLinearSolverType("MULTIFRONTAL_CHOLESKY");

	double nowerror = 0, newerror = 0,ii;
	nowerror = optimizer.error();
	RTK_Optimize_result = optimizer.optimize();
	newerror = optimizer.error();
	ii = newerror;

	if (newerror > nowerror)
	{
		return;
	}

	FILE* fp = fopen("../data/output/graph_result_dd.txt", "w");
	for (i = 0; i < fr_c.size(); i++)//fr_c.size()
	{


		Key bb=(Key)(fr_c.at(i)->epoch_num);
    Values::iterator xbegin=RTK_Optimize_result.find(bb);

		if (fr_c.at(i)->stat <= 0)
			continue;

		int week;
		double sec;
		sec = time2gpst(fr_c.at(i)->time, &week);
		fprintf(fp, "%d\t%f\t", week, sec);

		double pos[3], blh[3];

        Vector3 value_vector = xbegin->value.cast<gtsam::Vector3>();
		for (k = 0; k < 3; k++)
		{
            pos[k] = value_vector[k] + fr_c.at(i)->xf[k];
		}
		ecef2pos(pos, blh);

		for (j = 0; j < 3; j++)
		{
			fprintf(fp, "%.10f\t", blh[j]);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
    cout<<"program end"<<endl;


	return;
}

void RTKO_insertnewvalue_xb(const int ambi_count, const double dd_bias)
{

	int key = ambi_count + 100000;

	Vector1 cb;
  cb(0)= 0;//

	RTKvalues.insert(key,cb);
}

void RTKO_add_ddambi_Priori(const int ambi_count, const ambi_infor* dd_ambi)//, NonlinearFactorGraph *Factors)
{

    int key = ambi_count + 100000;


    	Vector1 cb;
	cb(0) =0;


	Vector1 nvq;

    nvq(0) = sqrt(dd_ambi->q);

        noiseModel::Diagonal::shared_ptr Priornoise = noiseModel::Diagonal::Sigmas(Vector1( sqrt(dd_ambi->q)));


		RTKFactors.emplace_shared<PriorFactor<Vector1>>((int)key, cb, Priornoise);
}

void RTKO_insertnewvalue_xc(int tcount, const double* x, const fr_check* frc)
{

	Vector3 cb ;
	for (int i = 0; i <3; i++)
	{
		cb(i) = 0;
	}

	RTKvalues.insert(tcount,cb);
}

void RTKO_add_Position_Priori(const fr_check* frc)
{
	int i;


	for (i = 0; i < 3; i++)
	{
		if (frc->Pf[i]>5&&frc->stat> SOLQ_NONE)
		{
			return;
		}
	}



	Vector3 cbb;
	Vector3 nvq;
	for (i = 0; i < 3; i++)
	{
		cbb(i) = 0;
	}

	if (frc->stat == SOLQ_FIX)
	{
		for (int i = 0; i < 3; i++)
		{
			nvq(i) = sqrt(frc->Pf[i]);
		}
	}
	else if (frc->stat == SOLQ_FLOAT)
	{
		for (int i = 0; i < 3; i++)
		{
			nvq(i) = sqrt(frc->Pf[i]);
		}
	}
	else
	{
		for (int i = 0; i < 3; i++)
		{
			nvq(i)= sqrt(frc->Pf[i]);
		}
	}


	    noiseModel::Diagonal::shared_ptr Priornoise = noiseModel::Diagonal::Sigmas(nvq);


		RTKFactors.emplace_shared<PriorFactor<Vector3>>((int)frc->epoch_num, cbb, Priornoise);
}

void RTKO_add_ddres_Factor(fr_check* frc, const nav_t* nav, const prcopt_t* opt)
{
	int i, j, k,f;
	gtime_t time = frc->obs[0].time;
	const obsd_t* obs = frc->obs;

	double *rs, *dts, *var, *e_, *azel;
	double *y,*v,*H,*R,*RR;
	int svh[MAXOBS * 2], vflg[MAXOBS * NFREQ * 2 + 1];
	int ny, n = frc->nn, nu = frc->nu, nr = frc->nr;
	int nf = opt->ionoopt == IONOOPT_IFLC ? 1 : opt->nf;

	rs = mat(6, n);
	dts = mat(2, n);
	var = mat(1, n);
	e_ = mat(3, n);
	azel = rtklib_zeros(2, n);

	y = mat(nf * 2, n);

	// compute satellite positions, velocities and clocks
	satposs(time, obs, n, nav, opt->sateph, rs, dts, var, svh);

	// calculate [range - measured pseudorange] for base station (phase and code)
	//	 output is in y[nu:nu+nr], see call for rover below for more details                                                 */
	trace(3, "base station:\n");

	if (!zdres_(1, obs + nu, nr, rs + nu * 6, dts + nu * 2, var + nu, svh + nu, nav, frc->rb, opt, 1,
		y + nu * nf * 2, e_ + nu * 3, azel + nu * 2)) {

		free(rs); free(dts); free(var); free(y); free(e_); free(azel);
		return;
	}

	if (!zdres_(0, obs, nu, rs, dts, var, svh, nav, frc->xf, opt, 0, y, e_, azel))
	{
		return;
	}

	ny = frc->sat_ns * nf * 2 + 2;
	v = mat(ny, 1); H = rtklib_zeros(9+MAXOBS*nf, ny); R = mat(ny, ny); RR = mat(ny, 1);

	int *ref_sat_index = new int[5 * 2 * nf];
	for (i = 0; i < (5 * 2 * nf); i++) { ref_sat_index[i] = -1; }

	if ((obs[0].time.time == 1574120750) && (obs[0].time.sec == 0.0))
	{
		double bb = 0;
	}
	if (frc->epoch_num == 1944)
	{
		double bb = 0;
	}
	if (frc->epoch_num == 1945)
	{
		double bb = 0;
	}

	int nv=ddres_(frc, opt, nav, y, e_, azel, v, H, R, vflg, ref_sat_index,RR);

	if (frc->epoch_num == 4054)
	{
		double bb = 0;
	}
	if (frc->epoch_num == 3256)
	{
		double bb = 0;
	}
	if (frc->epoch_num == 2792)
	{
		double bb = 0;
	}
	if (frc->epoch_num == 2794)
	{
		double bb = 0;
	}

	if (frc->nv != nv)
	{
		double bb = 0;
	}

	double base_singal_diff_res;
	int type, key_abmi,v_index=0;
	for (f = opt->mode > PMODE_DGPS ? 0 : nf; f < nf * 2; f++)
	{
		type = (f < nf) ? 0 : 1;
		for (i = 0; i < frc->sat_ns; i++)
		{

			int ref_index_index = ref_sat_index[frc->ssat[frc->sat[i] - 1].sys*(f / nf)*(f % nf)];
			int vflg_i = (frc->sat[ref_index_index] << 16) | (frc->sat[i] << 8) | ((f < nf ? 0 : 1) << 4) | (f%nf);

			int *vflg_index = find(vflg, vflg + nv, vflg_i);
			if (vflg_index == (vflg + nv))
				continue;
			if (i == ref_index_index)
				continue;

			base_singal_diff_res =  - y[f + frc->ir[ref_index_index] * nf * 2]+ y[f + frc->ir[i] * nf * 2];

			double data = sqrt(RR[v_index]);


			    noiseModel::Diagonal::shared_ptr GN = noiseModel::Diagonal::Sigmas(Vector1(data));

			if (!type)
			{
        key_abmi = d_ambi.at(frc->bias_index[v_index])->key + 100000;
				double lami = nav->lam[frc->sat[i] - 1][f%opt->nf];
				double ambi0 = d_ambi.at(frc->bias_index[v_index])->bias;

				if (v[v_index] < 10) {

            RTKFactors.emplace_shared<RTK_ddres_Factor_L_dd>(frc->epoch_num, key_abmi, frc, nav, opt, base_singal_diff_res,
						i, ref_index_index, f, GN, v[v_index], lami, ambi0);

				}

			}
			else
			{
				if (v[v_index] < 10) {

					    RTKFactors.emplace_shared<RTK_ddres_Factor_P>(frc->epoch_num, frc, nav, opt, base_singal_diff_res,
						i, ref_index_index, f, GN, v[v_index]);

				}

			}
			v_index++;

		}
	}
	delete []ref_sat_index; ref_sat_index = NULL;

	free(rs); free(dts); free(var); free(y); free(e_); free(azel);
	free(v); free(H); free(R); free(RR);
}

};
