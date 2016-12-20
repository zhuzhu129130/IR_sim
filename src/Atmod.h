//--------------------------------------------------------------------------------------
//Data:    		20160505
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

#ifndef ATMOD_H
#define ATMOD_H

#include "DataType.h"
#include "Variable.h"

class Atmod
{
public:
    bool InitAtmod(ATMOD * patm);//读入大气参数文件中的数据

    void UpdateAtmPara(ATMOD * patm, IRTARGET * pt);//随着目标运动更新参数

    float32 OnCalculate(ATMOD * patm,IRTARGET * pt);

    int32 ParaCalcu(int32 cho,ATMOD * patm);//计算各种分子吸收、气溶胶颗粒衰减和透过率

    float32 CalcuSub1(float32 zzh1,float32 zzh2,float32 zzh,float32 d11,float32 d12);
    float32 CalcuSub2(float32 zzh1,float32 zzh2,float32 zzh,float32 d11,float32 d12);
    float32 CalcuSub3(float32 rh,float32 t);
    float32 CalcuSub4(float32 denw,float32 t);

};

#endif
