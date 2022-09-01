#include <iostream>
#include "global.h"

#include "rtk/rtk.h"
//#include "CPublic.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include "StrctDef.h"
#include <vector>
#include <algorithm>
#include "DataConvertion.h"
#include "Rtkpost.h"
using namespace std;
//using namespace gtsam;

int main()
{
    cout<<"program start"<<endl;
    char* infile[3], * outfile = (char*)"";
	//char* resultfile = (char*)"";
    prcopt_t popt = prcopt_default;
	solopt_t sopt = solopt_default;
	filopt_t fopt = { "" };
    gtime_t ts = { 0 }, te = { 0 };
    double tint = 0.0;
    resetsysopts();
	loadopts("../rtk/opt.conf", sysopts);
	getsysopts(&popt, &sopt, &fopt);
	infile[0] = "../data/190424GNSS.19o";
	infile[1] = "../data/82051141.19o";
	infile[2] = "../data/82051141.19n";
	outfile = "../data/output/Rtkpos2022.txt";
    gtsam::postpos(ts, te, tint, 0.0, &popt, &sopt, &fopt, infile, 3, outfile, "", "");
    return 0;
}
