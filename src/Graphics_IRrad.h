//--------------------------------------------------------------------------------------
//Data:    		20160902
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

#ifndef GRAPHICS_IRRAD_H
#define GRAPHICS_IRRAD_H

#include <math.h>
#include "stdio.h"
#include <string.h>
#include <iostream>

#include "DataType.h"
#include "Variable.h"

class Graphics_IRrad
{
public:
    Graphics_IRrad();

    //=====================几何一致性数学计算公式==================================
    void ScrRot(SCRPOS * sp, SCRPOS * opt, SCRPOS * ori, float32 theta);//屏幕坐标旋转,由(opt-ori)旋转theta角度到达(sp-ori)
    VECTOR3 Vec3SetValue(VECTOR3 * vec, float32 x, float32 y, float32 z);//设置三维矢量值
    VECTOR3 Vec3Add(VECTOR3 * vec, VECTOR3 * vec1, VECTOR3 * vec2);//二个三维矢量相加
    VECTOR3 Vec3Subtract(VECTOR3 * vec, VECTOR3 * vec1, VECTOR3 * vec2);//二个三维矢量相减
    float32 Vec3GetLength(float32 * len, VECTOR3 * vec);//三维矢量求模
    float32 Vec4GetLength(float32 * len, VECTOR4 * vec);//四维矢量求模
    VECTOR3 Vec3Normalize(VECTOR3 * vec, VECTOR3 * vec1);//对三维矢量归一化
    VECTOR3 Vec3Cross(VECTOR3 * vec, VECTOR3 * vec1, VECTOR3 * vec2);//二个三维矢量叉乘
    float32 Vec3Dot(float32 * result, VECTOR3 * vec1, VECTOR3 * vec2);//二个三维矢量点乘
    VECTOR3 Vec3MultiConstant(VECTOR3 * vec, VECTOR3 * vec1, float32 num);//矢量乘以常数


    MATRIX16 Mat16SetValue(MATRIX16 * mat, float32 _11, float32 _12, float32 _13, float32 _14,
                       float32 _21, float32 _22, float32 _23, float32 _24,
                       float32 _31, float32 _32, float32 _33, float32 _34,
                       float32 _41, float32 _42, float32 _43, float32 _44);//设置一个4*4矩阵的值
    MATRIX16 Mat16SetIdentity(MATRIX16 * mat);//设置一个4*4的单位矩阵

    MATRIX16 MatrixMultiplyMatrix(MATRIX16 * outmat, MATRIX16 * mat1, MATRIX16 * mat2);//二个4*4的矩阵相乘，结果作为输出返回值

    MATRIX16 SetTransformMatrix(MATRIX16 * mat, VECTOR3 * pos);//设置平移矩阵
    MATRIX16 SetRotateMatrix(MATRIX16 * mat, float32 azi, float32 elv, float32 bnk);//设置旋转矩阵，根据方位角(偏航角)绕Y轴、俯仰角绕X轴、横滚角绕Z轴进行旋转,传入旋转角单位azi,elv为角度,bnk为弧度
    MATRIX16 SetRotateMatrixQuaternion(MATRIX16 * mat, float32 azi, float32 elv, float32 bnk);//设置旋转矩阵，根据单位四元数旋转,q1q2q3是虚部，q4是实部
    MATRIX16 SetRotateMatrixVecToVec(MATRIX16 * mat, VECTOR3 * vec1, VECTOR3 * vec2);//设置旋转矩阵，由一个单位矢量vec1旋转到另一个单位矢量vec2

    MATRIX16 SetWorldMatrix(MATRIX16 * mat,XSTATUS * stat);//设置世界变换矩阵，旋转信息由traj轨迹中的三个欧拉角得到
    //MATRIX16 SetWorldMatrixQRT(MATRIX16 * mat, QRTState * currState);//设置世界变换矩阵,旋转矩阵由QRT轨迹中的四元组得到
    MATRIX16 SetCameraMatrix(MATRIX16 * mat, VECTOR3 * Eye, VECTOR3 * At, VECTOR3 * Up);//设置观察变换矩阵

    VECTOR3  Vec3MultiplyMatrix(VECTOR3 * vec, VECTOR3 * vec1, MATRIX16 * mat, float32 w);//vec = vec1 * mat
    VECTOR3  MatrixMultiplyVec3(VECTOR3 * vec,  MATRIX16 * mat, VECTOR3 * vec1,float32 w);//vec = mat * vec1 testonly20090505

    //VECTOR4 QuatMultiQuat(VECTOR4 * qprod,VECTOR4 * quat1, VECTOR4 * quat2 );//四元组相乘
    //VECTOR4 QUAT_Triple_Product(VECTOR4 * qprod,VECTOR4 *quat1, VECTOR4 *quat2, VECTOR4 *quat3 );//对quat2这个四维顶点做变换

    MATRIX16 SetPerspectiveMatrix(MATRIX16 * mat, float32 wid, float32 hei, float32 zn, float32 zf );//构建左手坐标系的投影矩阵
    MATRIX16 SetViewportMatrix(MATRIX16 * mat, int32 startX, int32 startY, int32 wid, int32 hei, float32 minZ, float32 maxZ);//设置视区矩阵
    //=====================几何一致性数学计算公式==================================

    //=====================红外辐射一致性辐射温度转换公式==================================
    float32	CalcBlackbodyRadiance(const float32 tmpr);// 计算黑体的辐射强度(W/cm2.sr)
    float32 *	CalcBbRadianceLookupTable(float32 * rad, float32 IRTmprMin, float32 IRTmprMax, float32 IRTmprDelta);//计算黑体的红外辐射通量密度的查找表
    float32	SearchRadInLookupTable(float32 * radtbl, const float32 tmpr, float32 IRTmprMin, float32 IRTmprMax, float32 IRTmprDelta);//根据指定温度从查找表中找到黑体的红外辐射通量密度
    int32	RadToGray(const float32 rad);//将红外辐射映射为灰度
    //=====================红外辐射一致性辐射温度转换公式==================================

};

#endif // GRAPHICS_IRRAD_H
