//--------------------------------------------------------------------------------------
//Data:    		20160902
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

#ifndef IRIMG_AR_H
#define IRIMG_AR_H

#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>

#include "DataType.h"
#include "Variable.h"
#include "Graphics_IRrad.h"
#include "Atmod.h"

class IRImg_AR
{
public:
    IRImg_AR();

    void ScnRender(float32 timeDelta);

    //----------计算顶点位置和法线从模型空间到相机空间（包括世界变换和相机变换）------------------------
    bool CalcWorldCameraTransform(float32 timeDelta);

    //获得当前时刻的轨迹状态,para1:传入的总体轨迹，para2:输出的当前轨迹状态，para3:仿真开始运行到现在的时间偏移量，para4：para1的步数
    bool GetTrajStatus(TRAJECTORY * totalTj, XSTATUS * stat, float32 timeDelta, int32 totalTj_num);

    void UpdateWrapBoxInfo(IRTARGET * pt, MATRIX16 * mat);//更新包围盒信息

    void MoveBkg(IRBKG * pbkg);//根据光轴指向目标的运动方向，反向移动背景,暂时只上下左右平行移动背景，不考虑旋转和背景远近的缩放先

    bool CalcFacetGray(void);//traj轨迹下计算所有目标面元的灰度（由红外辐射dll提供内部运算）

    bool CheckAndCalcPointTgtGray(void);//判断是否是点目标，是的话计算点目标的红外辐射与灰度

    bool CalcAreaTgtGray(IRTARGET * pt);//traj轨迹时，计算面目标的红外辐射与灰度

    bool CalcProjViewportTransform(float32 timeDelta);//计算投影与视区变换

    void ProjWrapBox(IRTARGET * pt);

    bool RasTypeSwitch(void);//本函数用在RasterizeFacets函数前，根据不同的分块光栅化方案，进行预处理，选择合适的渲染平面

    void Rasterize_IRimg(IRBKG * pbkg); //根据真实拍摄的红外场景计算背景灰度值，也许是因为背景与场景生成的目标的各项参数不一致，需要进行反演

    void RasterizeBkg(IRBKG * pbkg);//用环境参数重新计算矩形区域中的背景灰度值，将结果直接写入光栅化frame缓存中

    void SearchMaxMinAvgBkgGray(unsigned short * graybuf,IRBKG * pbkg);//在当前的灰度数组中指定的矩形区域内找出最大最小平均灰度

    bool RasterizeFacets(int32 tno, int32 fno,int32 fnum, CAMSURECT * rc, uint32 sq0);//面元光栅化

    bool RasterizeSingleFacet(int32 fno, CAMSURECT * rc, uint32 sq0);//渲染某片可见的三角形面元

    void AddWhiteNoise(void);//添加白噪声

    void GenWhiteNoise(float32 fCamInceptPower_tmp,float32 V_tmp,float32 Vlsb);//产生白噪声

    bool CleanScnBuffer(void);//Desc: to clean all scn buffer in memory

    void RayRender(float32 timeDelta);

    void RayTracking();

    int intersect_tgtbox(IRTARGET * pt);

    int intersect_triangle(VECTOR3 vert0, VECTOR3 vert1, VECTOR3 vert2);

public:
    Graphics_IRrad * graphicirrad;
    Atmod          * atmod;

    float32 airtrans;
    VECTOR3 eye;
    VECTOR3 dir;
    float32 t;
    int32 count;

};

#endif // IRIMG_AR_H
