
#ifndef STRUCTDEF_H
#define STRUCTDEF_H
#include "rtk/rtk.h"

//#define TRANCE_SIMULATION
#define POST_DIM 240000   //平滑数据维数
#define STS24		21		//INS/GPS状态维数
#define WAT3		3		//量测维数
#define DataLength 36000
#define TEN_MINI   600

//组合同步时间控制
#define T_990MS  (int(0.99*TM_COUNT))
#define T_1010MS (int(1.01*TM_COUNT))
#define T_1100MS (int(1.1*TM_COUNT))

//PPS误差最大为4周期,否则丢弃
#define PPS_DT 4

#define GPS_MAXTIME 30		//连续30秒未校正退出状态

#define DZS_GPS_MAXTIME 6		//连续30秒未校正退出状态

//物理参数
//#define PI			3.14159265358979		//圆周率
#define Rad_Deg		57.29577951308232		//弧度化为度的刻度系数 180./PI
#define Deg_Rad		0.0174532925199433		//度化为弧度的刻度系数 PI/180.
#define Min_Rad		2.908882086657216e-4	//角分化为弧度
#define Dph_Rps		4.8481368110953599e-6	//度/小时化为rad/s

#define Rp			6356752.				//地球短半轴, 单位：米
#define e			0.00335281093			//地球椭圆度 1/298.2572 
#define Wie			7.2921151467e-5     	//地球自转角速率, 单位：弧度/s
#define KM			1000.					//公里
#define Knot_Ms 	0.5144444444			//节化为m/s的系数 1852./3600.0	
#define mg        	9.80154454e-3			//mg
#define g0        	9.80154454				//g0
#define PPM        	1.0e-6					//ppm

#define GYRO_TAO  	3600.					//陀螺时间常数



//运行时间宏定义
#define SWAY_MIN		1				//粗对准时间(min)
#define START_INS_TIME      60
#define CALIGN_TIME 	60				//粗对准时间(s)
#define ALIGN_TIME    	300				//总对准时间设置(s)
#define ENOUGH_ALIGN_TIME    	150				//总对准时间设置(s)

#define TM_STEP	 		0.0025			//解算周期(s)
#define TM_COUNT 		400				//解算频率(Hz)
#define TF 		  	 	0.1				//KF状态转移周期
#define SmoothCount   2


#define DZS_TM_COUNT 		200				//解算频率(Hz)
#define GPS_TM_COUNT 		40				//解算频率(Hz)
#define DZS_TM_STEP	 		0.005			//解算周期(s)
#define SQRT_DZS_TM_STEP    sqrt(0.005)
//#define FEEDBACK_TIMES_STEP      5*DZS_TM_STEP


//组合同步时间控制
#define DZS_T_190MS  (int(GPS_TM_COUNT-2))
#define DZS_T_210MS (int(GPS_TM_COUNT+2))
#define DZS_T_230MS (int(GPS_TM_COUNT+6))
#define DZS_T_990MS  (int(0.99*DZS_TM_COUNT))
#define DZS_T_1010MS (int(1.01*DZS_TM_COUNT))
#define DZS_T_1100MS (int(1.1*DZS_TM_COUNT))

#define T_990MS  (int(0.99*TM_COUNT))
#define T_1010MS (int(1.01*TM_COUNT))
#define T_1100MS (int(1.1*TM_COUNT))


//打开文件名
//#define FILE_NUM  10
#define FILE_NUM  11
#define FILE_NUM_RAW  2
#define PARA_FILE_NAME  "系统参数表.txt"
#define GPS_FILE_NAME   "GPS_B.txt"
#define PPGPS_FILE_NAME "PP_Data.dat"
#define IMU_FILE_NAME   "IMU_DECODEFILE.txt"

#define OUT_GPS_NAME   "Gps.txt"
#define OUT_IMU_NAME   "ImuGps.txt"

#define OUT_NAV_NAME   "Navi.txt"
#define OUT_NAV_NAME_5MS   "Navi_5MS.txt"
#define OUT_NAV_NAME_Event   "Navi_Event.txt"
#define OUT_NAV_VM_5MS  "Navi_5MS_VM.txt"


#define VMDIM 9
#define Re			6378137.				//地球长半轴, 单位：米

//参数1
//#define RVMP   0.002*0.002/Re/Re
//#define RVMH   0.005*0.005
//#define RVMV   0.0001*0.0001   
//#define RVMA   0.0001*0.0001

#define RVMP   0.02*0.02/Re/Re
#define RVMH   0.05*0.05
#define RVMV   0.001*0.001   
#define RVMA   0.001*0.001
//参数表结构体
typedef struct
{
	double	Kg[3];
	double	Ka[3];
	double	G0[3];
	double	A0[3];

	double	Dg[3][3];
	double	Da[3][3];

	double	Rx[3];
	double	Ry[3];
	double	Rz[3];
}PARA_STRUCT;

//处理后的IMU_GPS数据
typedef struct 
{
	unsigned int	Count;
	double	Wm[3];
	double	Vm[3];
	unsigned char   PPS_Flag;
	double	Delay;
	unsigned char    Valid1;
	double	GpsPos1[3];
	unsigned char    Valid2;
	double	GpsPos2[3];
	int     Temp_d;
	double  Temp_f;//IMU time
	double	Sec1;
	double	Sec2;

}IMU_GPS;

typedef struct  
{
	int   LOST_Flag;
	double  GPSLOSTTime;
	double  GPSCalTime;
}GPS_LOST_CLOCK;

/*9维fpr滤波器*/


typedef struct 				//地球相关参数计算
{
	double 	Rs;				//sinL
	double 	Rc;				//cosL
	
	double	Rmh;			//子午圈曲率半径
	double	Rnh;			//卯酉圈曲率半径

	double 	g;				//重力加速度
	double 	Wz;				//Wie天向分量
	double	Wn;				//Wie北向分量
	double	Wen_n[3];		//Wen在n系投影
	double	Win_n[3];		//Win在n系投影
	double 	Win_b[3];		//Win在b系投影
	double	A_Cori[3];		//哥氏加速度

	double	A_Cori_XYZ[3];		//哥氏加速度
	double  g_XYZ[3];//重力加速度
	double Wie_b[3];//Wie在b系投影

}EARTH_DATA;

/********** 导航解算周期输出结构体 **********/
typedef struct
{
	unsigned char	WorkSts;		//工作状态(0-默认; 1-粗对准; 2-精对准; 3-纯惯; 4-GPS组合; 5-RTKGPS组合)
	unsigned char	Align_Fault;	//对准故障(0-默认; 1-故障)
	
	unsigned char	T5MS_Flag;		//5mS标志
	unsigned char	T1S_Flag;		//1S标志
	unsigned char	T10S_Flag;		//10S标志
	
	////

	unsigned char	T200MS_Flag;		//1S标志
	////
	unsigned char   UPDATE_KF_FLAG;   //更新卡尔曼滤波标志
	
	double 			Att[3];			//姿态角:  俯仰/横滚/航向(rad)
	double 			Vg[3];			//速度:    Ve/Vn/Vu(m/s)
	double			Pos[3];			//位置:    经度(rad)/纬度/(rad)高/(m)

	double          PoseBLH[3];
	double          PosXYZ[3];
	double          AttXYZ[3];
	double 			VXYZ[3];			//速度:    ecef,x,y,z

	
	double			Rb[3];			//杆臂:    右前上(m)
	double			Rms[4];			//精度:    姿态(度)/航向(度)/速度(m/s)/位置(m)
	
	unsigned int	Align_Time;		//对准时间(s)
	unsigned int	Navi_Time;		//导航时间(s)
	
	unsigned int	Inte_Time;		//组合次数(s)
	unsigned int	X2Over_Time;	//X2检验超差次数(s)
	unsigned char	FinishAlignFlag;//对准结束标志

}NAVI_OUTPUT;
typedef struct
{
	double          timef;          //UTC时间
	double 			Att[3];			//姿态角:  俯仰/横滚/航向(rad)
	double 			Vg[3];			//速度:    Ve/Vn/Vu(m/s)
	double			Pos[3];			//位置:    经度(rad)/纬度/(rad)高/(m)
}POS_OUT_DATA;

/********** 抗晃动粗对准结构体 **********/
typedef struct
{	
	unsigned int    pRead;
	unsigned int    pStore;	
	double 			Q_ib0[4];
	double 			Cb_ib0[3][3];
	double 			VGi0[3];
	double 			Sum_VGi0[3];
	double 			VFib0[3];
	double 			Sum_VFib0[3];
	double 			VGi0_S[SWAY_MIN][3];
	double 			VFib0_S[SWAY_MIN][3];
	double          Aligntime;
}CALIGN_DATA;

typedef struct
{
	double 			Q_ib0[4];
	double 			Cb_ib0[3][3];
	double 			Att[3];			//姿态角:  俯仰/横滚/航向(rad)
	int movecount;
	int continuousmovecount;
	int basesta;
	double LastGpsPos[3];
	double GpsPos[3];
}ONESECOND_ALIGN_DATA;

/********** 最小二乘精对准结构体 **********/
typedef struct
{	
	double Theta_gb_z;

    int    Tk;
    double delta_wz;
    double ae;
    double be;
    double an;
    double bn;
    double fe;
    double fn;
    
    double Align_W0[3];
	double Align_A0[3];
}FALIGN_DATA;
	
/********** 导航解算结构体 **********/
typedef struct
{
	unsigned long	TimeCount;		//工作时间计数
	unsigned char	T100MS_Flag;		//100MS标志
	unsigned char	T30S_Flag;			//30S标志
	unsigned char	T60S_Flag;			//30S标志

	double			Q[4];				//四元数
	double			Cb_n[3][3];			//姿态转换矩阵

	double         Qb_e[4];//四元数
	double         Cb_e[3][3];//姿态转换矩阵
	double         Cn_e[3][3];//姿态转换矩阵

	double			Kg[3];				//

	double			SumFb[3];			//加计比力累积
	double			SumWb[3];			//加计比力累积
	double			SumFai[3];			//平台修正量

	double			ThetaA1[3];			//补偿后角增量
	double			DeltaV1[3];			//补偿后速度增量
	double			Last_DTh[3];		//上周期角增量

	NAVI_OUTPUT		OutData;			//输出数据	

	//零速校正使用变量
	double			Xk[STS24];			//状态估计值
	double			Pk[STS24][STS24];	//状态估计方差阵
	double			Qt[STS24];			//状态噪声方差阵
    double          FeedBack[STS24];
	double          Kk[STS24][WAT3];
	double			Fk[STS24][STS24];	//状态转移矩阵

	double			H[WAT3][STS24];		//组合量测矩阵
	double			R[WAT3][WAT3];		//量测噪声方差	
	double			Z[WAT3];			//量测值

	double 			Zero_W0[3];			//陀螺零位组合补偿量
	double 			Zero_A0[3];			//加计零位组合补偿量

	unsigned int	TransCount;			//时间更新次数
	unsigned int	TransFlag;			//计算一步预测
	unsigned int	NoFilterTime;		//未滤波时间
	unsigned int	FilterCount;		//滤波次数
	unsigned char	MesFlag;			//滤波计算标志

	double preuptime;//0712
	int GPS_FILE_TM_COUNT;
	double FEEDBACK_TIMES_STEP;
	double VectorforGN[5];
	double tempQ0[4],tempQ1[4];
	double Navqpvbeween[10];
	int lastupdatecount;
	int Initial_GN;
	double Temp_f;
	int  screencount;
	int  graphitercount;
}NAVIGATION_DATA;


/********** INS/GPS组合时序结构体 **********/
typedef struct
{
    unsigned int	T990ms;
    unsigned int	T1010ms;
    unsigned int	T1100ms;

	double			InsPos[3][3];
	double			LockPos[3];

	unsigned char	SyncStatus;
	unsigned char	GpsMode;			//(0-GPS; 1-RTK_GPS)
}INS_GPS_SYNC;
/********** INS/GPS组合时序结构体 **********/
typedef struct
{
	unsigned int	T190ms;
	unsigned int	T210ms;
	unsigned int	T230ms;

	double			InsPos[3][3];
	double			LockPos[3];

	unsigned char	SyncStatus;
	unsigned char	GpsMode;			//(0-GPS; 1-RTK_GPS)
}DZS_INS_GPS_SYNC;


typedef struct
{
    double Ka[3];			//加计刻度因子，  0:x；1:y；2:z
    double Ka_z[3];		//加计正刻度因子，0:x；1:y；2:z
    double Ka_f[3];		//加计负刻度因子，0:x；1:y；2:z
    double a0[3];			//加计零位

    double Kg[3];			//陀螺刻度因子，0:x；1:y；2:z
    double Kg_z[3];		//陀螺正刻度因子，0:x；1:y；2:z
    double Kg_f[3];		//陀螺负刻度因子，0:x；1:y；2:z
    double w0[3];			//陀螺零位

	double Bal1[6];

	double Lim_l;     //子惯导纵轴相对质心的杆臂 168
	double Lim_b[3];
	double Lim_t[3];
	double Lim_G;    //Gps相对子惯导纵轴质心的杆臂169

	double GDalfa_XZ  ;   
	double GDalfa_XY  ;
	double GDalfa_YZ  ;
	double GDalfa_YX  ;
	double GDalfa_ZX  ;   
	double GDalfa_ZY  ;

	double ADalfa_XZ  ;   
	double ADalfa_XY  ;
  
	double ADalfa_YZ  ;
	double ADalfa_YX  ;
  
	double ADalfa_ZX  ;    
	double ADalfa_ZY  ;
	double TempeRate_Gyrox[6];	//x陀螺零位变化与温度斜率参数27~32
	double	TempeRate_Gyroy[6];	//y陀螺零位变化与温度斜率参数33~38
	double	TempeRate_Gyroz[6];	//z陀螺零位变化与温度斜率参数39~44

	double Lim_b_accx[3]; //	    加表质心杆臂X  176~178
	double Lim_b_accy[3]; //		加表质心杆臂Y  179~181
	double Lim_b_accz[3]; //		加表质心杆臂Z  182~184
}PARA_DATA;



typedef struct
{
	double Wm0[POST_DIM];
	double Wm1[POST_DIM];
    double Wm2[POST_DIM];

	double Vm0[POST_DIM];
    double Vm1[POST_DIM];
	double Vm2[POST_DIM];

	double GpsPos2_0[POST_DIM];
	double GpsPos2_1[POST_DIM];
	double GpsPos2_2[POST_DIM];

	double Delay[POST_DIM];
	unsigned char Valid[POST_DIM];
	unsigned char PPS_Flag[POST_DIM];
	int FWorkStatus[DataLength];
	int Fintetime[DataLength];
	int BWorkStatus[DataLength];
	int Bintetime[DataLength];
	int gpsvali[DataLength];

	double Fatti0[DataLength];
	double Fatti1[DataLength];
	double Fatti2[DataLength];
	double Fv0[DataLength];
	double Fv1[DataLength];
	double Fv2[DataLength];
	double Fpos0[DataLength];
	double Fpos1[DataLength];
	double Fpos2[DataLength];
	double Fkg0[DataLength];
	double Fkg1[DataLength];
	double Fkg2[DataLength];
	double FZeroW00[DataLength];
	double FZeroW01[DataLength];
	double FZeroW02[DataLength];
	double FZeroA00[DataLength];
    double FZeroA01[DataLength];
    double FZeroA02[DataLength];
	double FRb0[DataLength];
    double FRb1[DataLength];
    double FRb2[DataLength];
	double FTemp_f[DataLength];

	double Batti0[DataLength];
	double Batti1[DataLength];
	double Batti2[DataLength];
	double Bv0[DataLength];
	double Bv1[DataLength];
	double Bv2[DataLength];
	double Bpos0[DataLength];
	double Bpos1[DataLength];
	double Bpos2[DataLength];
	double Bkg0[DataLength];
	double Bkg1[DataLength];
	double Bkg2[DataLength];
	double BZeroW00[DataLength];
	double BZeroW01[DataLength];
	double BZeroW02[DataLength];
	double BZeroA00[DataLength];
    double BZeroA01[DataLength];
    double BZeroA02[DataLength];
	double BRb0[DataLength];
    double BRb1[DataLength];
    double BRb2[DataLength];
	double BTemp_f[DataLength];

	int FNavCount[DataLength];
	int BNavCount[DataLength];

	double FAttiRms[DataLength];
	double FHdgRms[DataLength];
	double FVeRms[DataLength];
	double FPosRms[DataLength];

	double BAttiRms[DataLength];
	double BHdgRms[DataLength];
	double BVeRms[DataLength];
	double BPosRms[DataLength];


	int flighttime;	
	int sflighttime;
	int StoreIndex;

	int CalIndex;
	int ControlCount;

	int PBufferLength;

	int ForWardIndex;
	int BackWardIndex;

}POST_DATA;



/********** 导航解算周期输入结构体 **********/
typedef struct
{
	double			DTheta0[3];		//XYZ轴角增量(rad)	
	double			DeltaV0[3];		//XYZ轴速度增量(m/s)
	
	unsigned char	PPS_Flag;		//PPS标志(1-有效; 0-无效,外部清除)
	double			Delay;			//PPS延迟时间
	unsigned char   GpsValid;		//GPS位置有效标志(0-无效; 1-GPS; 2-GPSRTK)
	double			GpsPos[3];		//GPS经纬高(rad/rad/m)
	double			GpsPosXYZ[3];	//ecef坐标系
	unsigned char	InteCmd;		//外部组合指令(0-纯惯, 1-GPS组合)
}NAVI_INPUT;


typedef struct _tagNAV_DATA
{
	double time;
	double latitude;
	double longitude;
	double altitude;
	double x_velocity;
	double y_velocity;
	double z_velocity;
	double roll;
	double pitch;
	double platform_heading;
	double wander_angle;
	double x_accleration;
	double y_accleration;
	double z_accleration;
	double x_angular_rate;
	double y_angular_rate;
	double z_angular_rate;
} POSOUT_NAV_DATA;
//#define TIME_LBounD 70e4
#define TIME_LBounD 6.0e4
#define TIME_BounD  130e8

#define SEG_SUCCESS 1;
#define SEG_ERROR_IN_THE_BEGIN_OF_DATA 97;
#define SEG_ERROR_IN_THE_MID_OF_DATA 98;
#define SEG_ERROR_IN_THE_END_OF_DATA 99;

#define PHASE_TIME_DIS 50
#define PHASE_DIS  600

typedef struct {
	int time;
	double data;
	int seconds_after_last_point;
	int seconds_between_bad_points_before;
	int seconds_between_bad_points_after;
}TIME_POINTS;

typedef struct
{
	double Kg[3];
	double Zero_W0[3];
	double Zero_A0[3];
	double Rb[3];

}KF_FEED_VAR;



#endif