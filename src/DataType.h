//--------------------------------------------------------------------------------------
//Data:    		20160901
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

//本文件存放了所有的数据类型，结构体类型定义
#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <string>
using namespace std;

typedef	float	float32;
typedef	unsigned int	uint32;
typedef	int	int32;

#define EPSILON 1.0e-5f//接近0的无穷小值

#define MAX_FILENAME_CHAR 100

//常用参数
#define PI  3.1415926f
#define pai 0.0174533f //3.1415926/180

#define SCN_CAM_TRAJQRT 0x0001//当前读的是相机的轨迹文件
#define SCN_TGT_TRAJQRT 0x0002//当前读的是目标的轨迹文件

#define MAX_TRAJ_NUM 100//目标或相机的traj格式轨迹的最大步数

#define PLUME_PARNUM 90//90//18//50//定义每个尾焰的粒子数,建议为6(old4(old5))的整数倍

#define IR_TMPR_MIN 150.0f//100.0f//红外辐射通量密度查找表的最低温度
#define IR_TMPR_MAX 360.0f//1250.0f//1000.0f//红外辐射通量密度查找表的最高温度
#define IR_TMPR_DELTA 50.0f//25.0f//2.0f//红外辐射通量密度查找表的温度步长
#define IR_TMPR_NUM  (int32)((IR_TMPR_MAX - IR_TMPR_MIN)/IR_TMPR_DELTA)//红外辐射通量密度查找表的长度

#define MAX_PROJ_POS 999999.0f //目标投影坐标的最值，不超过正负这么多米

#define TGT_RES		0.5f//设目标的分辨率为0.5

#define ATMOD_NUM 34
#define ATMOD_ITP 33
#define ATMOD_T0 273.16f	//unit:K
#define ATMOD_P0 1013.4f	//unit:mpa
#define ATMOD_PI 3.141593f
#define ATMOD_N0 651

//--------------------------------------------------------------------------------------
//常用宏函数
#ifndef SAFE_FREE
#define SAFE_FREE(p) if (p){ free(p); (p) = NULL;}
#endif

#ifndef minimum
#define minimum(a,b) (a<=b ? a:b)
#endif

#ifndef maximum
#define maximum(a,b) (a>=b ? a:b)
#endif

#ifndef MUL_SAMPLE_FLAG
#define MUL_SAMPLE_FLAG (1)//是的话进行过采样，否的话直接写结果到最终的图像缓冲区，不经过中间过采样缓冲区
#endif

#ifndef ADD_BKG_FLAG
#define ADD_BKG_FLAG (0)//是的话加入背景，否则不加入背景
#endif

#ifndef ADD_IR_IMG //添加红外图像
#define ADD_IR_IMG (1)
#endif

#ifndef ADD_IR_CAMERA //实时采集的图像，否则就是保存的图像
#define ADD_IR_CAMERA (0)
#endif

#ifndef ADD_IR_CAMERA_SW //实时采集短波相机图像，否则就是保存的图像
#define ADD_IR_CAMERA_SW (1)
#endif

#ifndef SINGLE_CORRECT //实时采集的图像一点校正
#define SINGLE_CORRECT (1)
#endif

#ifndef ADD_COLOR_IMG //实时采集短波相机图像，否则就是保存的图像
#define ADD_COLOR_IMG (1)
#endif


#ifndef ADD_WHITENOISE_FLAG
#define ADD_WHITENOISE_FLAG (0)//是的话加入白噪声，否则不加入白噪声
#endif

#ifndef ADD_ATMOD_FLAG
#define ADD_ATMOD_FLAG (0)//是的话加入白噪声，否则不加入白噪声
#endif

//============================立体视觉模块==========================
//调试信息输出宏,预编译宏,
#ifndef DEBUG_SHOW
#define	DEBUG_SHOW  (1)//显示调试图像信息
#endif

#ifndef DEBUF_INFO_SHOW
#define	DEBUF_INFO_SHOW  (0)//显示调试数据
#endif


//--------------------------------------------------------------------------------------
//3D Computer Graphics Definition
typedef struct _SCRPOS
{
    int32 x;
    int32 y;
}SCRPOS;

typedef struct _VECTOR3
{
    float32 x;
    float32 y;
    float32 z;

}VECTOR3;

typedef struct _VECTOR4
{
    float32 q1;
    float32 q2;
    float32 q3;
    float32 q4;

}VECTOR4;

typedef struct _MATRIX16
{
    float32 _11;
    float32 _12;
    float32 _13;
    float32 _14;
    float32 _21;
    float32 _22;
    float32 _23;
    float32 _24;
    float32 _31;
    float32 _32;
    float32 _33;
    float32 _34;
    float32 _41;
    float32 _42;
    float32 _43;
    float32 _44;

}MATRIX16;

//--------------------------------------------------------------------------------------
//camPara definition
typedef struct _CAMPARA_OPT 
{
	float32 apt;//孔径(直径):mm
	float32 frmfrq;//侦频
	float32 airattn;//大气衰减系数
	float32 opttrans;//光学透过率
	float32 foc;//焦距:mm
}CAMPARA_OPT;

typedef struct _CAMPARA_DET 
{
	int32 wid;//宽度，像元数
	int32 hei;//高度，像元数
	int32 utwid;//探测器像元宽度(um)
	int32 uthei;//探测器像元高度(um)
	float32 detresp;//响应率(1e8 v/w)
	float32 detwl1;//波长下限(um)
	float32 detwl2;//波长上限(um)
	float32 detwldelta;//对波长的积分步长(um)
	float32 adgain;//放大器增益
	float32 adbias;//放大器偏置(V)
	int32  adres;//放大器分辨率(bits)
	float32 advol1;//放大器输入电压下限
	float32 advol2;//放大器输入电压上限
}CAMPARA_DET;

typedef struct _CAMPARA
{
	CAMPARA_OPT camOpt;
	CAMPARA_DET camDet;
}CAMPARA;

typedef struct _CAMPJRECT {
    float32 z0;//焦距(米)乘以－1，
    float32 x1;//x1y1最小值，x2y2最大值
    float32 x2;//x1y1最小值，x2y2最大值
    float32 y1;//x1y1最小值，x2y2最大值
    float32 y2;//x1y1最小值，x2y2最大值
    int32 wid;//宽度像素个数
    int32 hei;//高度像素个数
    float32 xr;//相机原始投影平面上每像素代表的实际长度
    float32 yr;//相机原始投影平面上每像素代表的实际长度
} CAMPJRECT;//相机的原始采样平面

typedef struct _CAMSURECT {
    float32 z0;//焦距(米)乘以－1，
    float32 x1;//x1y1最小值，x2y2最大值
    float32 x2;//x1y1最小值，x2y2最大值
    float32 y1;//x1y1最小值，x2y2最大值
    float32 y2;//x1y1最小值，x2y2最大值
    int32 wid;//宽度像素个数
    int32 hei;//高度像素个数
    int32 widstart;//左上角像素在整个探测器投影平面的起始值
    int32 heistart;//左上角像素在整个探测器投影平面的起始值
    int32 sx;//相机精细采样平面相对于相机原始投影平面的放大倍数
    int32 sy;//相机精细采样平面相对于相机原始投影平面的放大倍数

} CAMSURECT;//相机的原始采样平面

//Traj definition,Target and Camera
typedef struct _TRAJECTORY
{
    float32 xpos;//单位:m
    float32 ypos;//单位:m
    float32 zpos;//单位:m
    float32 velocity;//速度m/s
    float32 azimation;//偏航角,单位：角度（-180~180）
    float32 elevation;//俯仰角,单位：角度（-90~90）
    float32 banking;//横滚角,单位：角度  （-180~180）
    float32 acceleration;//加速度m/s2
    float32 steerspd;//转向,delta偏航角,单位：角度/s
    float32 raisespd;//爬升,delta俯仰角,单位：角度/s
    float32 rollspd;//倾斜,delta横滚角,单位：角度/s
    float32 interval;//时间间隔,s
    float32 currenttime;//当前时间,s
}TRAJECTORY;

typedef struct _XSTATUS
{
    int32 stat;
    VECTOR3 pos;
    VECTOR3 dir;
    float32 vlc;  // velocity, acceleration
    float32 acl;  // velocity, acceleration
    float32 azi;//偏航角，俯仰角，横滚角，单位：角度
    float32 elv;//偏航角，俯仰角，横滚角，单位：角度
    float32 bnk;//偏航角，俯仰角，横滚角，单位：角度
} XSTATUS;

//IR Camera
typedef struct _IRCAMERA
{
    char cammovfilename[MAX_FILENAME_CHAR];

    CAMPARA cpara;

    TRAJECTORY *	m_pTraj;
    int32			TrajCamNum;//相机轨迹步数
    float32			TrajCamStoptime;//相机轨迹停止时间(秒)
    XSTATUS			m_stat;//当前traj状态

    CAMPJRECT		CamProjRect;//相机的原始投影平面
    CAMSURECT		CamSuperRect;//相机的精细采样平面

}IRCAMERA;

//Target info 目标模型总体信息
typedef struct _FTG_INFO
{
    unsigned int tgID;
    unsigned int vxNum;// 本目标所含顶点总数
    unsigned int ftNum;// 本目标所含面元总数
    unsigned int mtNum;// 本目标所含材料总数
    unsigned int pmNum;// 本目标所含尾焰总数
    //float rsv[16];//由整型改为浮点型，rsv[0]~rsv[2]存放发动机的模型空间坐标，rsv[3]~rsv[5]存放前缘点的模型空间坐标
    int rsv[16];
}FTG_INFO;

typedef struct _VIRTEXF {
    VECTOR3 p;//顶点坐标
    VECTOR3 n;//法线信息
    //float32		gray;//不能在这里加顶点颜色，否则写入的模型文件target就不对了
}VIRTEXF;

typedef struct _FACETF {
    int32 i1;	// 3 int32dex of virtex
    int32 i2;	// 3 int32dex of virtex
    int32 i3;	// 3 int32dex of virtex
    int32 im;			// int32dex of material

    int32 flat;		// surface flat or smooth?
    int32 rsv[3];
}FACETF;

typedef struct _REFFRAMEF {
    VECTOR3 o;
    VECTOR3 u;
    VECTOR3 v;
    VECTOR3 n;
}REFFRAMEF;

typedef struct _MATERIALF {
    float32 rfl;     // reflectivity
    float32 abs;		// absorptivity
    float32 ems;     // emissivity
    float32 spr;		// specular refectivity
    float32 spa;		// specular angle
    float32 rsv[4];
}MATERIALF;


typedef struct _PLUMEF
{
    REFFRAMEF rfFrm;
    int32 shape;

    uint32 pmID;

    float32 dia_h;   //  plume head diameter
    float32 dia_t;	//  plume tail diameter
    float32 len;		//  plume length
    float32 tmpr;	//	plume head temperature
    float32 dens;	//	gas density (cm)
    float32 abs;		//  gas absorption coef.
    float32 irg;
    float32 flu;
    float32 rsv[4];
}PLUMEF;//尾焰

typedef struct _PLUMEPAR
{
    VECTOR3 mdl_pos;//在模型空间中粒子中心位置（随机移动后）particle center position(after random move) in the model local space
    VECTOR3 inimdl_pos;//particle initiate center position in the model local space
    VECTOR3 pos;//开始计算时的温度position being calculated(world, camera, proj, etc)
    //VECTOR3 vel;
    float32 tmpr;//温度temperature(K)
    float32 rad;//辐射radiance
    float32 gray;//int32? ;
    float32 semidia;//粒子半径semidiameter of the particle(m)
    float32 distance;//粒子到相机的距离the distance from this particle to the camera(m)
    float32 opa;//粒子的大气吸收系数opacity of this particle
    //int32   status;//0:die,1:alive
    int32 type;//1:core section, 2:mix section, 3: main section
}PLUMEPAR;//尾焰粒子

typedef struct _TGTPJRECT {
    float32 x1;//目标所有顶点最小x1y1,最大x2y2的投影坐标
    float32 x2;//目标所有顶点最小x1y1,最大x2y2的投影坐标
    float32 y1;//目标所有顶点最小x1y1,最大x2y2的投影坐标
    float32 y2;//目标所有顶点最小x1y1,最大x2y2的投影坐标
    float32 dx;//目标的分辨率？不懂这里怎样算的，X3D设为0.5*f/L，f和L单位均为米
    float32 dy;//目标的分辨率？不懂这里怎样算的，X3D设为0.5*f/L，f和L单位均为米
} TGTPJRECT;//目标自身的投影矩形

typedef struct _WRAPBOX
{//拥有8个顶点的包围盒
    VECTOR3 bp[8];
    float32 xmin, xmax;
    float32 ymin, ymax;
    float32 zmin, zmax;

    float32 xpmin, xpmax;
    float32 ypmin, ypmax;
}WRAPBOX;

//IR Target
typedef struct _IRTARGET
{
    char			m_name[MAX_FILENAME_CHAR];
    char			m_model[MAX_FILENAME_CHAR];//模型文件名称
    char			m_mov[MAX_FILENAME_CHAR];//轨迹文件名称

    TRAJECTORY*		m_pTraj;//若采用的是Traj轨迹的话    
    float32			TrajTgtStoptime;//目标停止时间(秒)
    int32			TrajTgtNum;//目标轨迹步数
    XSTATUS			m_stat;//当前traj状态

    float32			CamTgtMod;//相机目标之间距离(米)
    VECTOR3			CamTgt;//目标减相机矢量

    TGTPJRECT		TgtProjRect;//目标的投影平面

    WRAPBOX			tgtbox;//目标的包围盒

    int32 tgtNo;//目标在场景中的序号
    int32 vxNo;//目标的顶点在场景顶点索引的起始值
    int32 vxNum;//目标的顶点个数
    int32 mtNo;//材料
    int32 mtNum;

    float32 airtrans;//从本目标到探测器的大气透过率

}IRTARGET;

//IR Scene
typedef struct _IRSCENE
{
    int32 tgtNum;//目标数目
    int32 vxNum;//所有目标顶点的个数之和
    int32 ftNum;//所有目标面元的个数之和
    int32 mtNum;//所有目标材料的个数之和
    int32 pmNum;//所有目标的尾焰之和

    FTG_INFO  * ftgInfoLst;//每个目标的头信息
    VIRTEXF   * mdl_vxLst;//所有目标的模型空间中顶点列表
    VIRTEXF   * vxLst;//与其他矩阵进行转换计算后的目标顶点值存放列表
    MATERIALF * mtLst;//目标的材料列表
    PLUMEF    * pmLst;//目标的尾焰列表

    int32     * Visible;//目标的可见状态列表
    float32   * facetGray;//目标面元的灰度列表
    uint32    * Gray;//图像的灰度缓存
    float32   * ZDepth;//图像的深度缓存
    float32   * Alpha;//图像的不透明度缓存for plume

    float32 ScnStopTime;

    int32   AimTgtNo;//光轴瞄准目标的序号

}IRSCENE;

//IRDV文件
typedef struct _IRDVFILEHEADER
{
    uint32 fccType;//IRDV
    uint32 flags;//0
    uint32 width;  //探测器宽度
    uint32 height;//探测器高度
    short bitCount; //单个像素位数 16	//WORD  bitCount;
    uint32 imgSize;  //一侦图象大小:宽度*高度*sizeof(short)+sizeof(IRDV_FMINFO)
    uint32 length;// 文件中图象侦数:探测器的结束时间*探测器的framerate
    double rate;//探测器frame rate
    double xRes;//探测器单元宽度尺寸(um)除以焦距(mm)，单位：mrad
    double yRes;//探测器单元高度尺寸(um)除以焦距(mm)，单位：mrad
    double imgParam[16];//全0.0
    char name[64];
    char calFile[64];
}IRDVFILEHEADER;

typedef struct _IRDV_FMINFO
{
    int32 x;// unit: 0.01m
    int32 y;// unit: 0.01m
    int32 z;// unit: 0.01m

    int32 elv;  // unit: 1e-12 rad
    int32 azm;  // unit: 1e-12 rad
    int32 bnk;  // unit: 1e-12 rad

} IRDV_FMINFO;

typedef struct _IRBKG
{
        int32 wid;//整个背景大窗口的宽度
        int32 hei;//整个背景大窗口的高度
        int32 widstart;//当前选定矩形区域的左上角起始点
        int32 heistart;//当前选定矩形区域的左上角起始点
        int32 grayres;//灰度精度，单位：bit
        char  filename[MAX_FILENAME_CHAR];
        int32 filenamelen;
        uint32 frameno;
        float32 btmpr;//背景图像中的平滑部分的平均温度(如天空）
        float32 ttmpr;//背景图像中的扰动部分的平均温度（如云）
        float32 bems;//btmpr部分的发射率
        float32 tems;//ttmpr部分的发射率
        float32 airtrans;//背景到相机的大气透过率
        uint32 bgray;//btmpr映射到的灰度
        uint32 tgray;//ttmpr映射到的灰度
        short maxgray;//当前选定矩形区域的最大灰度
        short mingray;//当前选定矩形区域的最小灰度
        short avggray;//当前选定矩形区域的平均灰度

        unsigned short * bkgBuf;

}IRBKG;

typedef struct _ATMOD
{
    //public:


    // Attributes
    //public:
    bool m_bMeteoChoice;
    bool m_bTranChoice;
    int32 m_aero;//气溶胶
    int32 m_atmd;//大气模式
    int32 m_month;//月份选择
    char m_newmodulefile[MAX_FILENAME_CHAR];//新建大气模式文件
    int32 m_icld;//云的种类选择
    float32 m_himin,m_himax,m_gndalt,m_dist,m_theta;//传输路径参数
    float32 m_temp,m_press,m_rh,m_visi;//气象数据参数
    float32 m_rainrt,m_cthick,m_calt,m_rextcld;//云雨参数
    float32 m_wl1,m_wl2,m_wvstep;//波长参数
    char m_sh;//是否计算斜程透过率
    char m_met;//有无该高度的气象数据
    char m_fn[MAX_FILENAME_CHAR];//大气模式文件
    float32 m_nh,m_nm,m_hsec,m_grt;//hour,minute,second,temperture(K) 用于批处理计算加权光谱透过率
    float32 m_wh2o0;
    char arsmodl[8+1][MAX_FILENAME_CHAR];
    char strAtmd[MAX_FILENAME_CHAR];
    bool IsCal;
    float32 att;

    float32 pr[ATMOD_NUM],tp[ATMOD_NUM];

    char AtmodDir[MAX_FILENAME_CHAR];
    int32 nmax,nmin;
    //	float32 m_fire;

    //private:
    float32 cn2o[3][365];
    float32 cch4[494];
    float32 co3[5][154];
    float32 cco2[9][226];
    float32 cn2[134];
    float32 ch2o[10][386];
    float32 slf296[931],slf260[931],frh[931];
    float32 TH2O[ATMOD_N0+1],TH2OC[ATMOD_N0+1],TN2[ATMOD_N0+1],TCO2[ATMOD_N0+1],TO3[ATMOD_N0+1],TN2O[ATMOD_N0+1],TCH4[ATMOD_N0+1],TRC[ATMOD_N0+1],WL[ATMOD_N0+1],Taero[ATMOD_N0+1];
    float32 cco[126+11+1],cnh3[126+126+101+1],cno[62+1],cso2[76+62+1];

}ATMOD;

typedef struct {
    unsigned short info[5];
    unsigned short frameBuffer[320*256];
} FTCAM_FRAME;


struct RunParams
{

    //程序运行参数calib，rectify，reconstruct，process
    string processType;
    //===================立体标定参数=====================================
    //图像序列列表#calib_image/imagelist.xml
    string calib_imagepath;

    //立体标定参数保存文件目录
    string stereocalib_path;

    //棋盘格长宽大小参数
    int cornerX;
    int cornerY;
    float squareSize;
    //====================================================================

    //=============立体校正参数============================================
    //校正参数计算后测试图像
    string image1_test;
    string image2_test;

    //立体校正参数 RECTIFY_HARTLEY，RECTIFY_BOUGUET
    string rectifymethod;

    string rectifyParams_path;
   //=============立体校正参数============================================

   //================立体重建参数=========================================
   //起始与终止索引
    int start_index;
    int end_index;

    //数据所在目录，及图像名称前缀和后缀
    string img_left;
    string left_extension;
    string img_right;
    string right_extension;

    //视差计算算法  BM , SGBM, VAR, ELAS
    string DisparityType;
    //================立体重建参数=========================================

    //================数据源==============================================
    //三种数据源：image，camera，video
    string file_type;

    int camera1;
    int camera2;

    string video1_dir;
    string video2_dir;

    string detector;
    string descriptor;
    double good_match_threshold;

};

#endif
