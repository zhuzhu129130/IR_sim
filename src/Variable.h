//--------------------------------------------------------------------------------------
//Data:    		20160901
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//本文件定义了所有的全局变量、静态数组
//--------------------------------------------------------------------------------------


#ifndef VARIABLE_H
#define VARIABLE_H

#include <stdio.h>
#include "pthread.h"
#include "DataType.h"

extern FILE* fpdebug;  //调试结果文件

extern IRCAMERA		cam;//相机

extern 	IRTARGET *		pTgt;//目标列表

extern  IRSCENE			IRScn;//场景

extern 	float32			TgtTmpr;//目标的当前温度(K)

extern 	ATMOD           atm;

extern 	IRBKG			bkg;//背景

extern float32			CamWidLen;//相机宽度方向上的像素个数乘以像元宽度(米)
extern float32			CamHeiLen;//相机高度方向上的像素个数乘以像元高度(米)

extern float32			dx;//xp = dx * x/z, yp = dy *x/z, zd = z, dx是X轴由观察空间投影到投影空间的系数，dy是Y轴由观察空间投影到投影空间的系数
extern float32			dy;//xp = dx * x/z, yp = dy *x/z, zd = z, dx是X轴由观察空间投影到投影空间的系数，dy是Y轴由观察空间投影到投影空间的系数


extern bool             bIRDVOpened;
extern FILE *           DVFp;
extern IRDVFILEHEADER   dvInfo;
extern char			    DVFileName[MAX_FILENAME_CHAR];//待返回的DV文件名

extern  MATRIX16		matWorld;//世界矩阵，观察矩阵，二者乘积矩阵
extern  MATRIX16		matCamera;//世界矩阵，观察矩阵，二者乘积矩阵
extern  MATRIX16		matWorldCamera;//世界矩阵，观察矩阵，二者乘积矩阵

extern  float32			BbRadiance[IR_TMPR_NUM];//黑体的红外辐射通量密度的查找表

extern 	float32			xpmin;//目标所有顶点投影坐标的最值
extern 	float32			xpmax;//目标所有顶点投影坐标的最值
extern 	float32			ypmin;//目标所有顶点投影坐标的最值
extern 	float32			ypmax;//目标所有顶点投影坐标的最值

extern 	uint32          sq0;

extern  uint32	*		ImgTmpGrayBuf;//相机精细采样平面的临时图像灰度数组
extern  float32 *		ImgTmpZdepth;//相机精细采样平面的临时图像深度缓存

extern 	int32			TgtNx1;//目标的投影矩形在相机原始投影平面上的整数坐标位置
extern 	int32			TgtNx2;//目标的投影矩形在相机原始投影平面上的整数坐标位置
extern 	int32			TgtNy1;//目标的投影矩形在相机原始投影平面上的整数坐标位置
extern 	int32			TgtNy2;//目标的投影矩形在相机原始投影平面上的整数坐标位置
extern float32			TgtGx1;//目标的投影矩形从相机原始投影平面上的整数坐标换算回的浮点坐标，已被裁剪只保留了可见部分
extern 	float32			TgtGx2;//目标的投影矩形从相机原始投影平面上的整数坐标换算回的浮点坐标，已被裁剪只保留了可见部分
extern 	float32			TgtGy1;//目标的投影矩形从相机原始投影平面上的整数坐标换算回的浮点坐标，已被裁剪只保留了可见部分
extern 	float32			TgtGy2;//目标的投影矩形从相机原始投影平面上的整数坐标换算回的浮点坐标，已被裁剪只保留了可见部分


extern int32 m_nContrast;
extern int32 m_nBright;
extern int32 contrast_max;
extern int32 bright_max;
extern int m_nContrast1;
extern int m_nBright1;

extern	int32 * whitenoise;//白噪声
extern	float stemi;//目标材料发射率
extern	float scnSNR;//场景信噪比

extern  unsigned short *buf_recv;
extern  unsigned short *buf_recv1;
extern  pthread_t pthread_id;
extern  pthread_t pthread_id1;
extern  int32 readImg_mutex;
extern  int32 readImg_mutex1;
extern  int32 render_mutex;

extern FTCAM_FRAME ftcam_frame;
extern FTCAM_FRAME ftcam_frame1;
extern unsigned short info[5];

//======================立体视觉===================
extern RunParams runParams;

//============SSCA===========
extern string ccName;
extern string caName;
extern string ppName;
extern double costAlpha; //0.3 for middllebury or 1.0 for kitti
extern int maxDis;  //`60` for Middlebury and `256` for KITTI dataets.
extern int disSc;      //`4` for Middlebury and `1` for KITTI datasets
//============SSCA===========

#endif // VARIABLE_H
