//--------------------------------------------------------------------------------------
//Data:    		20160901
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

#include "Variable.h"

//--------------------------------------------------------------------------------------
FILE* fpdebug;

IRCAMERA		cam;//相机

IRTARGET *		pTgt;//目标列表

IRSCENE			IRScn;//场景

float32			TgtTmpr;//目标的当前温度(K)

ATMOD           atm;

IRBKG			bkg;//背景

float32			CamWidLen;//相机宽度方向上的像素个数乘以像元宽度(米)
float32			CamHeiLen;//相机高度方向上的像素个数乘以像元高度(米)

float32			dx;//xp = dx * x/z, yp = dy *x/z, zd = z, dx是X轴由观察空间投影到投影空间的系数，dy是Y轴由观察空间投影到投影空间的系数
float32			dy;//xp = dx * x/z, yp = dy *x/z, zd = z, dx是X轴由观察空间投影到投影空间的系数，dy是Y轴由观察空间投影到投影空间的系数

bool            bIRDVOpened;
FILE *          DVFp;
IRDVFILEHEADER  dvInfo;
char			DVFileName[MAX_FILENAME_CHAR];//待返回的DV文件名

MATRIX16		matWorld;//世界矩阵，观察矩阵，二者乘积矩阵
MATRIX16		matCamera;//世界矩阵，观察矩阵，二者乘积矩阵
MATRIX16		matWorldCamera;//世界矩阵，观察矩阵，二者乘积矩阵

float32			BbRadiance[IR_TMPR_NUM];//黑体的红外辐射通量密度的查找表

float32			xpmin;//目标所有顶点投影坐标的最值
float32			xpmax;//目标所有顶点投影坐标的最值
float32			ypmin;//目标所有顶点投影坐标的最值
float32			ypmax;//目标所有顶点投影坐标的最值

uint32          sq0;

uint32	*		ImgTmpGrayBuf;//相机精细采样平面的临时图像灰度数组
float32 *		ImgTmpZdepth;//相机精细采样平面的临时图像深度缓存

int32			TgtNx1;//目标的投影矩形在相机原始投影平面上的整数坐标位置
int32			TgtNx2;//目标的投影矩形在相机原始投影平面上的整数坐标位置
int32			TgtNy1;//目标的投影矩形在相机原始投影平面上的整数坐标位置
int32			TgtNy2;//目标的投影矩形在相机原始投影平面上的整数坐标位置
float32			TgtGx1;//目标的投影矩形从相机原始投影平面上的整数坐标换算回的浮点坐标，已被裁剪只保留了可见部分
float32			TgtGx2;//目标的投影矩形从相机原始投影平面上的整数坐标换算回的浮点坐标，已被裁剪只保留了可见部分
float32			TgtGy1;//目标的投影矩形从相机原始投影平面上的整数坐标换算回的浮点坐标，已被裁剪只保留了可见部分
float32			TgtGy2;//目标的投影矩形从相机原始投影平面上的整数坐标换算回的浮点坐标，已被裁剪只保留了可见部分


int32 m_nContrast;
int32 m_nBright;
int32 contrast_max;
int32 bright_max;
int m_nContrast1;
int m_nBright1;

int32 * whitenoise;//白噪声
float stemi;//目标材料发射率
float scnSNR;//场景信噪比

unsigned short *buf_recv;
unsigned short *buf_recv1;
pthread_t pthread_id;
pthread_t pthread_id1;
int32 readImg_mutex;
int32 readImg_mutex1;
int32 render_mutex;

FTCAM_FRAME ftcam_frame;
FTCAM_FRAME ftcam_frame1;
unsigned short info[5];

//========================立体视觉========================
RunParams runParams;

//============SSCA===========
string ccName;
string caName;
string ppName;
double costAlpha; //0.3 for middllebury or 1.0 for kitti
int maxDis;  //`60` for Middlebury and `256` for KITTI dataets.
int disSc;      //`4` for Middlebury and `1` for KITTI datasets
//============SSCA===========
