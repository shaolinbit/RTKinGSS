#ifndef DATACONVERTION_H
#define DATACONVERTION_H
#include <vector>
#pragma once
#include "minmea.h"
#pragma pack(1)
typedef struct {
	unsigned char frame[2];
	unsigned char time[8];
	double gyro_x;
	double gyro_y;
	double gyro_z;
	double acc_x;
	double acc_y;
	double acc_z;
	short temp;
	unsigned char count;
	unsigned char check;
	unsigned char end[2];
}Message;
typedef struct {
	double GPSTime;
	double Latitude;
	double Longitude;
	double Altitude;
	double Vn;
	double Ve;
	double Vd;
	double LantitudeError;
	double LongitudeError;
	double AltitudeError;
	int fix_q;
	double hdop;
} DGPSStruct;
unsigned char Bcd2Hex(unsigned char data);
double ConvertToTime(unsigned char time[8]);
int ConvertCASGPS(const char* inputfile, const char* outputfile);
int ConvertDGPS(const char* inputfile, const char* outputfile);
int ConvertNMEA(const char* inputfile, const char* outputfile);
void ConvertNMEAstruct( minmea_sentence_gga ingga,DGPSStruct& outdgps);


#endif
