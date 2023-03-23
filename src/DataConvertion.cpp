
#include "DataConvertion.h"
#include <iostream>
#include <cstring>
//#include "StrctDef.h"

//using namespace std;
//extern NAVIGATION_DATA	NaviData;
//extern int INSFrequency;
//extern double INS_UPDATE_TIME;
#ifdef __unix
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),  (mode)))==NULL
#endif
unsigned char Bcd2Hex(unsigned char data)
{
	unsigned char temp;
	temp = ((data >> 4) * 10 + (data & 0x0f));
	return temp;
}
double ConvertToTime(unsigned char time[8])
{
	double hour = Bcd2Hex(time[0]);
	double minute = Bcd2Hex(time[1]);
	double second = Bcd2Hex(time[2]);
	int64_t insecond = (int64_t)(time[3]) << 32 | (int64_t)(time[4]) << 24 | (int64_t)(time[5]) << 16 | (int64_t)(time[6]) << 8 | (int64_t)(time[7]);
	double x = (double)insecond / 75e6;
	double t = hour * 3600 + minute * 60 + second + (double)insecond / 75e6;

	return t;
}


int ConvertCASGPS(const char* inputfile, const char* outputfile)
{
	FILE* fpRead, *fpWrite;

	int r = fopen_s(&fpRead, inputfile, "rb+");
	if (r)
	{
		printf("Open %s failed", inputfile);
		return r;
	}
	r = fopen_s(&fpWrite, outputfile, "w");
	if (r)
	{
		printf("Create text file %s failed", outputfile);
		return r;
	}
	return 0;
}
int ConvertDGPS(const char* inputfile, const char* outputfile)
{
	FILE* fpRead, *fpWrite;
	int r = fopen_s(&fpRead, inputfile, "rb+");
	if (r)
	{
		printf("Open %s failed", inputfile);
		return r;
	}
	r = fopen_s(&fpWrite, outputfile, "w");
	if (r)
	{
		printf("Create text file %s failed", outputfile);
		return r;
	}

	DGPSStruct buff;
	while (true)
	{
		memset(&buff, 0, sizeof(buff));
		int n = fread(&buff, 1, sizeof(buff), fpRead);
		if (n == 0)
			break;
		//		double t = ConvertToTime(buff.GPSTime);
		char text[1255];
		/*
		sprintf_s(text, "%s %s %f %d %f %s %.15lf %s %.15lf %.15lf %f %d %.15lf %.15lf %.15lf\n",
			"log0", "05/19/13", buff.GPSTime, 14, 1.6, "N", buff.Latitude, "E", buff.Longitude,
			buff.Altitude, 0.061, 1, buff.Vn, buff.Ve, buff.Vd);
		fwrite(text, strlen(text), 1, fpWrite);
		*/
	}
	fclose(fpRead);
	fclose(fpWrite);
	return 0;
}
int ConvertNMEA(const char* inputfile, const char* outputfile)
{
	FILE *fp, *fp1;
	int r = fopen_s(&fp, inputfile, "r+");
	if (r)
		return r;
	r = fopen_s(&fp1, outputfile, "w");
	if (r)
		return r;
	char sentence[1200];
	char buff[1200] = { 0 };
	double vn = 0.000, vd = 0.000, ve = 0.000;
	while (!feof(fp))
	{
		fgets(sentence, sizeof(sentence), fp);
		switch (minmea_sentence_id(sentence, false))
		{
		case MINMEA_SENTENCE_GGA:
		{
			struct minmea_sentence_gga gga;
			if (minmea_parse_gga(&gga, sentence)) {
                /*
				sprintf_s(buff, "%s %s %02d:%02d:%02d.%06d %d %lf %s %.15f %s %.15f %lf %lf %d %lf %lf %lf\n",
					"log0", "05/13/13",
					gga.time.hours, gga.time.minutes, gga.time.seconds, gga.time.microseconds,
					14,
					1.6,
					"N",
					minmea_tocoord(&gga.latitude),
					"E",
					minmea_tocoord(&gga.longitude),
					minmea_tofloat(&gga.altitude),
					0.061,
					1,
					vn,
					vd,
					ve
				);
				fputs(buff, fp1);
				*/
			}
			else {
				printf("$xxRMC sentence is not parsed\n");
			}
		}
		default:
			break;
		}
	}
	fclose(fp);
	fclose(fp1);
	return 0;
}
void ConvertNMEAstruct( minmea_sentence_gga gga1, DGPSStruct& outdgps)
{

	 outdgps.GPSTime = (double)(gga1.time.hours)*3600.0 +
		(double)(gga1.time.minutes)*60.0 + (double)(gga1.time.seconds) + (double)(gga1.time.microseconds)*1.0e-6;
	 outdgps.Latitude = minmea_tocoord(&gga1.latitude);
	 outdgps.Longitude = minmea_tocoord(&gga1.longitude);
	 outdgps.Altitude = minmea_tofloat(&gga1.altitude);
	 outdgps.fix_q = gga1.fix_quality;
	 outdgps.hdop = minmea_tofloat(&gga1.hdop);
}

