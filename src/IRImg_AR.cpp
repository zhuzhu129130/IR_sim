//--------------------------------------------------------------------------------------
//Data:    		20160902
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

#include "IRImg_AR.h"

using namespace std;

IRImg_AR::IRImg_AR()
{
    graphicirrad = new Graphics_IRrad();
    atmod        = new Atmod();
}

void IRImg_AR::ScnRender(float32 timeDelta)
{
    //这里若是使用汇编语言将大大加速，每一帧都要进行清零的。
    memset(IRScn.Gray,0,cam.cpara.camDet.wid * cam.cpara.camDet.hei*sizeof(int32)); //清理灰度缓冲为固定值
    //memset(IRScn.ZDepth, 0.0f, cam.cpara.camDet.wid * cam.cpara.camDet.hei*sizeof(int32));//清理Z缓冲为固定值

    if (!CalcWorldCameraTransform(timeDelta))//计算顶点位置和法线从模型空间到相机空间,更新尾焰粒子位置，投影到相机空间
    {
        printf("world camera transform failed\n");
    }
    else
    {
        if (!CalcFacetGray())//计算每个面元的红外辐射
        {
            printf("IR radiance calc failed\n");
        }
        else
        {
            if (!CalcProjViewportTransform(timeDelta))//计算投影变换，直接投影到像元上
            {
                printf("project and viewport transform failed\n");
                //这里若是使用汇编语言将大大加速，每一帧都要进行清零的。
                memset(IRScn.Gray,0,cam.cpara.camDet.wid * cam.cpara.camDet.hei*sizeof(int32)); //清理灰度缓冲为固定值
            }
            else
            {
                if (!RasTypeSwitch())//if (!RasterizeFacets())//根据不同的分块光栅化方案，进行预处理，选择合适的渲染平面
                {
                    //这里若是使用汇编语言将大大加速，每一帧都要进行清零的。
                    memset(IRScn.Gray,0,cam.cpara.camDet.wid * cam.cpara.camDet.hei*sizeof(int32)); //清理灰度缓冲为固定值
                    printf("rasterization failed\n");
                }
                else
                {
#if ADD_WHITENOISE_FLAG
                    AddWhiteNoise();
#endif//ADD_WHITENOISE_FLAG
                }
            }
        }
    }
}

//----------计算顶点位置和法线从模型空间到相机空间（包括世界变换和相机变换）------------------------
bool IRImg_AR::CalcWorldCameraTransform(float32 timeDelta)
{//Desc: to calculate virtex's position and normal from model space to camera space, (world trans + camera trans)

    int32 i = 0, j = 0;
    VECTOR3 Up;

    if(timeDelta == 0)
    {
        //首先计算光轴指向目标的世界矩阵与观察矩阵,并对其顶点进行变换
        GetTrajStatus(cam.m_pTraj, &cam.m_stat,timeDelta,cam.TrajCamNum);//获得当前时刻的相机轨迹状态，相机位置在原点，运动反向向下z轴=1
        GetTrajStatus(pTgt[IRScn.AimTgtNo].m_pTraj, &pTgt[IRScn.AimTgtNo].m_stat,timeDelta,pTgt[IRScn.AimTgtNo].TrajTgtNum);//获得当前时刻的目标轨迹状态

        graphicirrad->Vec3Subtract(&pTgt[IRScn.AimTgtNo].CamTgt, &pTgt[IRScn.AimTgtNo].m_stat.pos , &cam.m_stat.pos);
        graphicirrad->Vec3GetLength(&pTgt[IRScn.AimTgtNo].CamTgtMod, &pTgt[IRScn.AimTgtNo].CamTgt);//求出相机目标之间距离，方便算辐射时用

        pTgt[IRScn.AimTgtNo].m_stat.elv *= -1.0f;//俯仰角乘以负号，使得俯仰角增大时飞机朝上，在IRSim5中也有类似处理(乘负号)，结果与X3D中的一样,参见DX Doc正负号与纸质笔记

        graphicirrad->SetWorldMatrix(&matWorld,&pTgt[IRScn.AimTgtNo].m_stat);//设置世界变换矩阵，就是从模型空间变换到世界坐标系，平移信息由位置得到，旋转信息由traj轨迹中的三个欧拉角得到

        graphicirrad->Vec3SetValue(&Up,0.0f,1.0f,0.0f);
        graphicirrad->SetCameraMatrix(&matCamera,&cam.m_stat.pos,&pTgt[IRScn.AimTgtNo].m_stat.pos,&Up);//(&Eye, &At, &Up);//设置相机观察变换矩阵，从世界坐标系变换到相机坐标系

        graphicirrad->MatrixMultiplyMatrix(&matWorldCamera, &matWorld, &matCamera);//求出了从模型空间经过世界空间直接变换到观察空间的坐标变换矩阵
        //printf("matWorldCamera is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",matWorldCamera._11,matWorldCamera._12,matWorldCamera._13,matWorldCamera._14,matWorldCamera._21,matWorldCamera._22,matWorldCamera._23,matWorldCamera._24,matWorldCamera._31,matWorldCamera._32,matWorldCamera._33,matWorldCamera._34,matWorldCamera._41,matWorldCamera._42,matWorldCamera._43,matWorldCamera._44);
        for (i = pTgt[IRScn.AimTgtNo].vxNo; i < (pTgt[IRScn.AimTgtNo].vxNum+pTgt[IRScn.AimTgtNo].vxNo); i++ )//对每个顶点从模型空间变换到相机坐标系观察空间
        {
            //fprintf(fpdebug, "matWorldCamera is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",matWorldCamera._11,matWorldCamera._12,matWorldCamera._13,matWorldCamera._14,matWorldCamera._21,matWorldCamera._22,matWorldCamera._23,matWorldCamera._24,matWorldCamera._31,matWorldCamera._32,matWorldCamera._33,matWorldCamera._34,matWorldCamera._41,matWorldCamera._42,matWorldCamera._43,matWorldCamera._44);
            graphicirrad->Vec3MultiplyMatrix(&IRScn.vxLst[i].p, &IRScn.mdl_vxLst[i].p, &matWorldCamera, 1.0f);
            //if (i%100 == 0)
            //    fprintf(fpdebug, "tgt%d: vxLst[%d].p=(%f,%f,%f), mdl_vxLst[%d].p=(%f,%f,%f)\n",IRScn.AimTgtNo,i,IRScn.vxLst[i].p.x,IRScn.vxLst[i].p.y,IRScn.vxLst[i].p.z,i,IRScn.mdl_vxLst[i].p.x,IRScn.mdl_vxLst[i].p.y,IRScn.mdl_vxLst[i].p.z);
        }

        UpdateWrapBoxInfo(&pTgt[IRScn.AimTgtNo], &matWorldCamera);//更新包围盒信息
    }
    else
    {
        GetTrajStatus(pTgt[IRScn.AimTgtNo].m_pTraj, &pTgt[IRScn.AimTgtNo].m_stat,timeDelta,pTgt[IRScn.AimTgtNo].TrajTgtNum);//获得当前时刻的目标轨迹状态

        graphicirrad->Vec3Subtract(&pTgt[IRScn.AimTgtNo].CamTgt, &pTgt[IRScn.AimTgtNo].m_stat.pos , &cam.m_stat.pos);
        graphicirrad->Vec3GetLength(&pTgt[IRScn.AimTgtNo].CamTgtMod, &pTgt[IRScn.AimTgtNo].CamTgt);//求出相机目标之间距离，方便算辐射时用

        pTgt[IRScn.AimTgtNo].m_stat.elv *= -1.0f;//俯仰角乘以负号，使得俯仰角增大时飞机朝上，在IRSim5中也有类似处理(乘负号)，结果与X3D中的一样,参见DX Doc正负号与纸质笔记

        graphicirrad->SetWorldMatrix(&matWorld,&pTgt[IRScn.AimTgtNo].m_stat);//设置世界变换矩阵，就是从模型空间变换到世界坐标系，平移信息由位置得到，旋转信息由traj轨迹中的三个欧拉角得到

        graphicirrad->MatrixMultiplyMatrix(&matWorldCamera, &matWorld, &matCamera);//求出了从模型空间经过世界空间直接变换到观察空间的坐标变换矩阵
        //printf("matWorldCamera is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",matWorldCamera._11,matWorldCamera._12,matWorldCamera._13,matWorldCamera._14,matWorldCamera._21,matWorldCamera._22,matWorldCamera._23,matWorldCamera._24,matWorldCamera._31,matWorldCamera._32,matWorldCamera._33,matWorldCamera._34,matWorldCamera._41,matWorldCamera._42,matWorldCamera._43,matWorldCamera._44);
        for (i = pTgt[IRScn.AimTgtNo].vxNo; i < (pTgt[IRScn.AimTgtNo].vxNum+pTgt[IRScn.AimTgtNo].vxNo); i++ )//对每个顶点从模型空间变换到相机坐标系观察空间
        {
            //fprintf(fpdebug, "matWorldCamera is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",matWorldCamera._11,matWorldCamera._12,matWorldCamera._13,matWorldCamera._14,matWorldCamera._21,matWorldCamera._22,matWorldCamera._23,matWorldCamera._24,matWorldCamera._31,matWorldCamera._32,matWorldCamera._33,matWorldCamera._34,matWorldCamera._41,matWorldCamera._42,matWorldCamera._43,matWorldCamera._44);
            graphicirrad->Vec3MultiplyMatrix(&IRScn.vxLst[i].p, &IRScn.mdl_vxLst[i].p, &matWorldCamera, 1.0f);
            //if (i%100 == 0)
            //    fprintf(fpdebug, "tgt%d: vxLst[%d].p=(%f,%f,%f), mdl_vxLst[%d].p=(%f,%f,%f)\n",IRScn.AimTgtNo,i,IRScn.vxLst[i].p.x,IRScn.vxLst[i].p.y,IRScn.vxLst[i].p.z,i,IRScn.mdl_vxLst[i].p.x,IRScn.mdl_vxLst[i].p.y,IRScn.mdl_vxLst[i].p.z);


        }

        UpdateWrapBoxInfo(&pTgt[IRScn.AimTgtNo], &matWorldCamera);//更新包围盒信息

    }

    //首先计算光轴指向目标的世界矩阵与观察矩阵,并对其顶点进行变换
   /* GetTrajStatus(cam.m_pTraj, &cam.m_stat,timeDelta,cam.TrajCamNum);//获得当前时刻的相机轨迹状态，相机位置在原点，运动反向向下z轴=1
    GetTrajStatus(pTgt[IRScn.AimTgtNo].m_pTraj, &pTgt[IRScn.AimTgtNo].m_stat,timeDelta,pTgt[IRScn.AimTgtNo].TrajTgtNum);//获得当前时刻的目标轨迹状态

    graphicirrad->Vec3Subtract(&pTgt[IRScn.AimTgtNo].CamTgt, &pTgt[IRScn.AimTgtNo].m_stat.pos , &cam.m_stat.pos);
    graphicirrad->Vec3GetLength(&pTgt[IRScn.AimTgtNo].CamTgtMod, &pTgt[IRScn.AimTgtNo].CamTgt);//求出相机目标之间距离，方便算辐射时用

    pTgt[IRScn.AimTgtNo].m_stat.elv *= -1.0f;//俯仰角乘以负号，使得俯仰角增大时飞机朝上，在IRSim5中也有类似处理(乘负号)，结果与X3D中的一样,参见DX Doc正负号与纸质笔记

    graphicirrad->SetWorldMatrix(&matWorld,&pTgt[IRScn.AimTgtNo].m_stat);//设置世界变换矩阵，就是从模型空间变换到世界坐标系，平移信息由位置得到，旋转信息由traj轨迹中的三个欧拉角得到

    graphicirrad->Vec3SetValue(&Up,0.0f,1.0f,0.0f);
    graphicirrad->SetCameraMatrix(&matCamera,&cam.m_stat.pos,&pTgt[IRScn.AimTgtNo].m_stat.pos,&Up);//(&Eye, &At, &Up);//设置相机观察变换矩阵，从世界坐标系变换到相机坐标系

    graphicirrad->MatrixMultiplyMatrix(&matWorldCamera, &matWorld, &matCamera);//求出了从模型空间经过世界空间直接变换到观察空间的坐标变换矩阵
    //printf("matWorldCamera is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",matWorldCamera._11,matWorldCamera._12,matWorldCamera._13,matWorldCamera._14,matWorldCamera._21,matWorldCamera._22,matWorldCamera._23,matWorldCamera._24,matWorldCamera._31,matWorldCamera._32,matWorldCamera._33,matWorldCamera._34,matWorldCamera._41,matWorldCamera._42,matWorldCamera._43,matWorldCamera._44);
    for (i = pTgt[IRScn.AimTgtNo].vxNo; i < (pTgt[IRScn.AimTgtNo].vxNum+pTgt[IRScn.AimTgtNo].vxNo); i++ )//对每个顶点从模型空间变换到相机坐标系观察空间
    {
        //fprintf(fpdebug, "matWorldCamera is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",matWorldCamera._11,matWorldCamera._12,matWorldCamera._13,matWorldCamera._14,matWorldCamera._21,matWorldCamera._22,matWorldCamera._23,matWorldCamera._24,matWorldCamera._31,matWorldCamera._32,matWorldCamera._33,matWorldCamera._34,matWorldCamera._41,matWorldCamera._42,matWorldCamera._43,matWorldCamera._44);
        graphicirrad->Vec3MultiplyMatrix(&IRScn.vxLst[i].p, &IRScn.mdl_vxLst[i].p, &matWorldCamera, 1.0f);


    }*/

    UpdateWrapBoxInfo(&pTgt[IRScn.AimTgtNo], &matWorldCamera);//更新包围盒信息

#if ADD_BKG_FLAG //添加背景，根据光轴指向的目标反向移动背景
    MoveBkg(&bkg);
#endif//ADD_BKG_FLAG


    //然后计算其它的目标的世界矩阵，由前面算出的观察矩阵一起，对顶点进行变换
    for (i = 0; i < IRScn.AimTgtNo; i++)
    {
        GetTrajStatus(pTgt[i].m_pTraj, &pTgt[i].m_stat,timeDelta,pTgt[i].TrajTgtNum);//获得当前时刻的目标轨迹状态

        graphicirrad->Vec3Subtract(&pTgt[i].CamTgt, &pTgt[i].m_stat.pos , &cam.m_stat.pos);
        graphicirrad->Vec3GetLength(&pTgt[i].CamTgtMod, &pTgt[i].CamTgt);//求出相机目标之间距离，方便算辐射时用

        pTgt[i].m_stat.elv *= -1.0f;//俯仰角乘以负号，使得俯仰角增大时飞机朝上，在IRSim5中也有类似处理(乘负号)，结果与X3D中的一样,参见DX Doc正负号与纸质笔记

        graphicirrad->SetWorldMatrix(&matWorld,&pTgt[i].m_stat);

        graphicirrad->MatrixMultiplyMatrix(&matWorldCamera, &matWorld, &matCamera);//求出了从模型空间经过世界空间直接变换到观察空间的坐标变换矩阵
        //fprintf(fpdebug, "matWorldCamera is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",matWorldCamera._11,matWorldCamera._12,matWorldCamera._13,matWorldCamera._14,matWorldCamera._21,matWorldCamera._22,matWorldCamera._23,matWorldCamera._24,matWorldCamera._31,matWorldCamera._32,matWorldCamera._33,matWorldCamera._34,matWorldCamera._41,matWorldCamera._42,matWorldCamera._43,matWorldCamera._44);
        for (j = pTgt[i].vxNo; j < (pTgt[i].vxNum+pTgt[i].vxNo); j++ )
        {
            graphicirrad->Vec3MultiplyMatrix(&IRScn.vxLst[j].p, &IRScn.mdl_vxLst[j].p, &matWorldCamera, 1.0f);
            //if (i%100==0)
            //    fprintf(fpdebug, "tgt%d: vxLst[%d].p=(%f,%f,%f), mdl_vxLst[%d].p=(%f,%f,%f)\n",i,j,IRScn.vxLst[j].p.x,IRScn.vxLst[j].p.y,IRScn.vxLst[j].p.z,j,IRScn.mdl_vxLst[j].p.x,IRScn.mdl_vxLst[j].p.y,IRScn.mdl_vxLst[j].p.z);
        }

        UpdateWrapBoxInfo(&pTgt[IRScn.AimTgtNo], &matWorldCamera);//更新包围盒信息
    }


    for (i = IRScn.AimTgtNo+1; i < IRScn.tgtNum ; i++)
    {
        GetTrajStatus(pTgt[i].m_pTraj, &pTgt[i].m_stat,timeDelta,pTgt[i].TrajTgtNum);//获得当前时刻的目标轨迹状态

        graphicirrad->Vec3Subtract(&pTgt[i].CamTgt, &pTgt[i].m_stat.pos , &cam.m_stat.pos);
        graphicirrad->Vec3GetLength(&pTgt[i].CamTgtMod, &pTgt[i].CamTgt);//求出相机目标之间距离，方便算辐射时用

        pTgt[i].m_stat.elv *= -1.0f;//俯仰角乘以负号，使得俯仰角增大时飞机朝上，在IRSim5中也有类似处理(乘负号)，结果与X3D中的一样,参见DX Doc正负号与纸质笔记

        graphicirrad->SetWorldMatrix(&matWorld,&pTgt[i].m_stat);

        graphicirrad->MatrixMultiplyMatrix(&matWorldCamera, &matWorld, &matCamera);//求出了从模型空间经过世界空间直接变换到观察空间的坐标变换矩阵
        //fprintf(fpdebug, "matWorldCamera is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",matWorldCamera._11,matWorldCamera._12,matWorldCamera._13,matWorldCamera._14,matWorldCamera._21,matWorldCamera._22,matWorldCamera._23,matWorldCamera._24,matWorldCamera._31,matWorldCamera._32,matWorldCamera._33,matWorldCamera._34,matWorldCamera._41,matWorldCamera._42,matWorldCamera._43,matWorldCamera._44);
        for (j = pTgt[i].vxNo; j < (pTgt[i].vxNum+pTgt[i].vxNo); j++ )
        {
            graphicirrad->Vec3MultiplyMatrix(&IRScn.vxLst[j].p, &IRScn.mdl_vxLst[j].p, &matWorldCamera, 1.0f);
            //if (i%100==0)
            //    fprintf(fpdebug, "tgt%d: vxLst[%d].p=(%f,%f,%f), mdl_vxLst[%d].p=(%f,%f,%f)\n",i,j,IRScn.vxLst[j].p.x,IRScn.vxLst[j].p.y,IRScn.vxLst[j].p.z,j,IRScn.mdl_vxLst[j].p.x,IRScn.mdl_vxLst[j].p.y,IRScn.mdl_vxLst[j].p.z);

        }

        UpdateWrapBoxInfo(&pTgt[IRScn.AimTgtNo], &matWorldCamera);//更新包围盒信息
    }
    //---------------------------------------------------------------------
    //debug only : test QRT not aimed at
    /*
    VECTOR3 At;
    Vec3SetValue(&Up, 0.0f, 0.0f, 1.0f);//若视线看向Y轴，Z轴为正上方
    Vec3SetValue(&At, 0.0f, 1.0f, 0.0f);
    //Vec3SetValue(&Up, 0.0f, 1.0f, 0.0f);//若视线看向Z轴，Y轴为正上方
    //Vec3SetValue(&At, 0.0f, 0.0f, 1.0f);
    ////Vec3SetValue(&Up, 0.0f, 1.0f, 0.0f);//若视线看向X轴，Y轴为正上方
    ////Vec3SetValue(&At, 1.0f, 0.0f, 0.0f);
    SetCameraMatrix(&matCamera,&CamState.pos,&At,&Up);//(&Eye, &At, &Up);
    */
    //---------------------------------------------------------------------

    return true;
}

//---获得当前时刻的轨迹状态,para1:传入的总体轨迹，para2:输出的当前轨迹状态，para3:仿真开始运行到现在的时间偏移量，para4：para1的步数----
bool IRImg_AR::GetTrajStatus(TRAJECTORY * totalTj, XSTATUS * stat, float32 timeDelta, int32 totalTj_num)
{//获得当前时刻的轨迹状态,para1:传入的总体轨迹，para2:输出的当前轨迹状态，para3:仿真开始运行到现在的时间偏移量，para4：para1的步数
    int32 i;
    float32 dt1,dt2;
    float32 elevation, azimation, velocity, banking;
    TRAJECTORY * tmpTj = totalTj;//保存的临时指针
    if (totalTj_num == 0)
    {//当前轨迹为空
        printf("total traj is null\n");
        return false;
    }

    for(i=0;i<totalTj_num;i++)
    {
        if(tmpTj[i].currenttime>= timeDelta) break;
    }

    if(i==0)
    {
        stat->stat= 1;
        stat->pos.x= tmpTj[0].xpos;
        stat->pos.y= tmpTj[0].ypos;
        stat->pos.z= tmpTj[0].zpos;
        stat->dir.x= cos((tmpTj[0].elevation)*pai)*sin((tmpTj[0].azimation)*pai);
        stat->dir.y= sin((tmpTj[0].elevation)*pai);
        stat->dir.z= cos((tmpTj[0].elevation)*pai)*cos((tmpTj[0].azimation)*pai);
        stat->vlc= tmpTj[0].velocity;
        stat->acl= tmpTj[0].acceleration;
        stat->azi= tmpTj[0].azimation;
        stat->elv= tmpTj[0].elevation;
        stat->bnk= tmpTj[0].banking;
        //stat->bnk= (float32)atan((tmpTj[0].velocity)*(tmpTj[0].steerspd)*pai/9.8f);
        return true;
    }

    if(i==totalTj_num)
    {
        stat->stat= 1;
        stat->pos.x= tmpTj[totalTj_num-1].xpos;
        stat->pos.y= tmpTj[totalTj_num-1].ypos;
        stat->pos.z= tmpTj[totalTj_num-1].zpos;
        stat->dir.x= cos((tmpTj[totalTj_num-1].elevation)*pai)*sin((tmpTj[totalTj_num-1].azimation)*pai);
        stat->dir.y= sin((tmpTj[totalTj_num-1].elevation)*pai);
        stat->dir.z= cos((tmpTj[totalTj_num-1].elevation)*pai)*cos((tmpTj[totalTj_num-1].azimation)*pai);
        stat->vlc= tmpTj[totalTj_num-1].velocity;
        stat->acl= tmpTj[totalTj_num-1].acceleration;
        stat->azi= tmpTj[totalTj_num-1].azimation;
        stat->elv= tmpTj[totalTj_num-1].elevation;
        stat->bnk= tmpTj[totalTj_num-1].banking;
        //stat->bnk=(float32)( atan((tmpTj[totalTj_num-1].velocity)*(tmpTj[totalTj_num-1].steerspd)*pai/9.8f));
        return true;
    }

    //dt1= (timeDelta- tmpTj[i-1].currenttime)/(tmpTj[i].currenttime- tmpTj[i-1].currenttime);
    //dt2= (tmpTj[i].currenttime -timeDelta)/(tmpTj[i].currenttime- tmpTj[i-1].currenttime);

    stat->stat= 1;

    azimation= tmpTj[i-1].azimation + tmpTj[i-1].steerspd * timeDelta;
    elevation= tmpTj[i-1].elevation + tmpTj[i-1].raisespd * timeDelta;
    banking= tmpTj[i-1].banking + tmpTj[i-1].rollspd * timeDelta;

    velocity= tmpTj[i-1].velocity;
    stat->dir.x= cos(elevation*pai)*sin(azimation*pai);
    stat->dir.y= sin(elevation*pai);
    stat->dir.z= cos(elevation*pai)*cos(azimation*pai);
    stat->vlc= velocity;
    stat->acl= tmpTj[i-1].acceleration;
    VECTOR3 deldis0;
    deldis0.x = stat->vlc*stat->dir.x;//侦频快的话每侦的背景移动像素数小些，不容易跑出背景范围
    deldis0.y = stat->vlc*stat->dir.y;
    deldis0.z = stat->vlc*stat->dir.z;
    stat->pos.x= (tmpTj[i-1].xpos) + deldis0.x * timeDelta;
    stat->pos.y= (tmpTj[i-1].ypos) + deldis0.y * timeDelta/2;
    stat->pos.z= (tmpTj[i-1].zpos) + 10 * timeDelta;//deldis0.z * timeDelta;

    stat->azi= azimation;
    stat->elv= elevation;
    stat->bnk= banking;
    //stat->bnk= (float32)(atan(velocity*banking*pai/9.8f));

    /*dt1= (timeDelta- tmpTj[i-1].currenttime)/(tmpTj[i].currenttime- tmpTj[i-1].currenttime);
    dt2= (tmpTj[i].currenttime -timeDelta)/(tmpTj[i].currenttime- tmpTj[i-1].currenttime);

    stat->stat= 1;
    stat->pos.x= (tmpTj[i].xpos)*dt1+ (tmpTj[i-1].xpos)*dt2;
    stat->pos.y= (tmpTj[i].ypos)*dt1+ (tmpTj[i-1].ypos)*dt2;
    stat->pos.z= (tmpTj[i].zpos)*dt1+ (tmpTj[i-1].zpos)*dt2;

    elevation= (tmpTj[i].elevation)*dt1+ (tmpTj[i-1].elevation)*dt2;
    azimation= (tmpTj[i].azimation)*dt1+ (tmpTj[i-1].azimation)*dt2;
    velocity= (tmpTj[i].velocity)*dt1+ (tmpTj[i-1].velocity)*dt2;
    //banking= (tmpTj[i].steerspd)*dt1+ (tmpTj[i-1].steerspd)*dt2;
    banking=  (tmpTj[i].banking)*dt1+ (tmpTj[i-1].banking)*dt2;

    stat->dir.x= cos(elevation*pai)*sin(azimation*pai);
    stat->dir.y= sin(elevation*pai);
    stat->dir.z= cos(elevation*pai)*cos(azimation*pai);
    stat->vlc= velocity;
    stat->acl= tmpTj[i-1].acceleration;
    stat->azi= azimation;
    stat->elv= elevation;
    stat->bnk= banking;
    //stat->bnk= (float32)(atan(velocity*banking*pai/9.8f));*/

    return true;
}

//--------------------------------------------------------------------------------------
void IRImg_AR::UpdateWrapBoxInfo(IRTARGET * pt, MATRIX16 * mat)
{
    VECTOR3 tmpbp[8];//临时包围盒顶点
    int32 i;

    //更新光轴指向目标的包围盒信息
    graphicirrad->Vec3SetValue(&tmpbp[0], pt->tgtbox.xmin,pt->tgtbox.ymin,pt->tgtbox.zmin);
    graphicirrad->Vec3SetValue(&tmpbp[1], pt->tgtbox.xmin,pt->tgtbox.ymin,pt->tgtbox.zmax);
    graphicirrad->Vec3SetValue(&tmpbp[2], pt->tgtbox.xmin,pt->tgtbox.ymax,pt->tgtbox.zmin);
    graphicirrad->Vec3SetValue(&tmpbp[3], pt->tgtbox.xmin,pt->tgtbox.ymax,pt->tgtbox.zmax);
    graphicirrad->Vec3SetValue(&tmpbp[4], pt->tgtbox.xmax,pt->tgtbox.ymin,pt->tgtbox.zmin);
    graphicirrad->Vec3SetValue(&tmpbp[5], pt->tgtbox.xmax,pt->tgtbox.ymin,pt->tgtbox.zmax);
    graphicirrad->Vec3SetValue(&tmpbp[6], pt->tgtbox.xmax,pt->tgtbox.ymax,pt->tgtbox.zmin);
    graphicirrad->Vec3SetValue(&tmpbp[7], pt->tgtbox.xmax,pt->tgtbox.ymax,pt->tgtbox.zmax);
    for (i=0; i<8; i++)
    {
        graphicirrad->Vec3MultiplyMatrix(&pt->tgtbox.bp[i], &tmpbp[i], mat, 1.0f);
    }

    //============用于光线追迹，不用光线追迹的话就注释掉以下内容=====================
    /*float32 txmin, txmax,tymin, tymax,tzmin, tzmax;
    txmin = txmax = pt->tgtbox.bp[0].x;
    tymin = tymax = pt->tgtbox.bp[0].y;
    tzmin = tzmax = pt->tgtbox.bp[0].z;
    for (i=1; i<8; i++)
    {
        if (pt->tgtbox.bp[i].x < txmin)
        {
            txmin = pt->tgtbox.bp[i].x;
        }
        if (pt->tgtbox.bp[i].x > txmax)
        {
            txmax = pt->tgtbox.bp[i].x;
        }
        if (pt->tgtbox.bp[i].y < tymin)
        {
            tymin = pt->tgtbox.bp[i].y;
        }
        if (pt->tgtbox.bp[i].y > tymax)
        {
            tymax = pt->tgtbox.bp[i].y;
        }
        if (pt->tgtbox.bp[i].z < tzmin)
        {
            tzmin = pt->tgtbox.bp[i].z;
        }
        if (pt->tgtbox.bp[i].z > tzmax)
        {
            tzmax = pt->tgtbox.bp[i].z;
        }
    }
    pt->tgtbox.xmin = txmin;
    pt->tgtbox.xmax = txmax;
    pt->tgtbox.ymin = tymin;
    pt->tgtbox.ymax = tymax;
    pt->tgtbox.zmin = tzmin;
    pt->tgtbox.zmax = tzmax;
    //更新光轴指向目标的包围盒信息
    graphicirrad->Vec3SetValue(&pt->tgtbox.bp[0], pt->tgtbox.xmin,pt->tgtbox.ymin,pt->tgtbox.zmin);
    graphicirrad->Vec3SetValue(&pt->tgtbox.bp[1], pt->tgtbox.xmin,pt->tgtbox.ymin,pt->tgtbox.zmax);
    graphicirrad->Vec3SetValue(&pt->tgtbox.bp[2], pt->tgtbox.xmin,pt->tgtbox.ymax,pt->tgtbox.zmin);
    graphicirrad->Vec3SetValue(&pt->tgtbox.bp[3], pt->tgtbox.xmin,pt->tgtbox.ymax,pt->tgtbox.zmax);
    graphicirrad->Vec3SetValue(&pt->tgtbox.bp[4], pt->tgtbox.xmax,pt->tgtbox.ymin,pt->tgtbox.zmin);
    graphicirrad->Vec3SetValue(&pt->tgtbox.bp[5], pt->tgtbox.xmax,pt->tgtbox.ymin,pt->tgtbox.zmax);
    graphicirrad->Vec3SetValue(&pt->tgtbox.bp[6], pt->tgtbox.xmax,pt->tgtbox.ymax,pt->tgtbox.zmin);
    graphicirrad->Vec3SetValue(&pt->tgtbox.bp[7], pt->tgtbox.xmax,pt->tgtbox.ymax,pt->tgtbox.zmax);*/
}

//-------根据光轴指向目标的运动方向，反向移动背景,暂时只上下左右平行移动背景，不考虑旋转和背景远近的缩放------------------------
void IRImg_AR::MoveBkg(IRBKG * pbkg)//根据光轴指向目标的运动方向，反向移动背景,暂时只上下左右平行移动背景，不考虑旋转和背景远近的缩放先
{//生成结果为pbkg->widstart和pbkg->heistart
    int32 delx,dely;
    VECTOR3 deldis,deldis0,pos;
    deldis0.x = pTgt[IRScn.AimTgtNo].m_stat.vlc*pTgt[IRScn.AimTgtNo].m_stat.dir.x/cam.cpara.camOpt.frmfrq;//侦频快的话每侦的背景移动像素数小些，不容易跑出背景范围
    deldis0.y = pTgt[IRScn.AimTgtNo].m_stat.vlc*pTgt[IRScn.AimTgtNo].m_stat.dir.y/cam.cpara.camOpt.frmfrq;
    deldis0.z = pTgt[IRScn.AimTgtNo].m_stat.vlc*pTgt[IRScn.AimTgtNo].m_stat.dir.z/cam.cpara.camOpt.frmfrq;


    //deldis0.x /= VELSOUND;//debug only,背景移动太快，考虑缩小步伐
    //deldis0.y /= VELSOUND;//debug only,背景移动太快，考虑缩小步伐
    //deldis0.z /= VELSOUND;//debug only,背景移动太快，考虑缩小步伐

    float32 xstep,ystep;
    xstep = (cam.CamSuperRect.x2 - cam.CamSuperRect.x1) / cam.CamSuperRect.wid;
    ystep = (cam.CamSuperRect.y2 - cam.CamSuperRect.y1) / cam.CamSuperRect.hei;

    graphicirrad->Vec3MultiplyMatrix(&deldis, &deldis0, &matCamera, 1.0f);//对运动方向做观察变换
    graphicirrad->Vec3MultiplyMatrix(&pos, &pTgt[IRScn.AimTgtNo].m_stat.pos, &matCamera, 1.0f);
    graphicirrad->Vec3Add(&pos,&pos,&deldis);

    delx = (int32)(deldis.x*cam.cpara.camOpt.foc* 1e-3f/pos.z/xstep + 0.5f);
    dely = (int32)(deldis.y*cam.cpara.camOpt.foc* 1e-3f/pos.z/ystep + 0.5f);

    pbkg->widstart -= delx;//背景与目标反向运动
    pbkg->heistart -= dely;


}

//-----------------traj轨迹下计算所有目标面元的灰度（由红外辐射dll提供内部运算）-------------------------------------------------
bool IRImg_AR::CalcFacetGray(void)
{
    int32 i;
    for (i=0; i< IRScn.tgtNum; i++)
    {
        if (! CheckAndCalcPointTgtGray())
        {//如果不是点目标，则按面目标计算红外辐射并转化为灰度；否则按点目标算红外辐射并转化为灰度
            CalcAreaTgtGray(&pTgt[i]);//traj轨迹时，计算面目标的红外辐射与灰度
        }
    }

    return true;
}

//-------------判断是否是点目标，是的话计算点目标的红外辐射与灰度---------------------------------------
bool IRImg_AR::CheckAndCalcPointTgtGray(void)//判断是否是点目标，是的话计算点目标的红外辐射与灰度
{//暂时先不考虑是不是点目标，全按面目标计算
    return false;
}

//---------------traj轨迹时，计算面目标的红外辐射与灰度-----------------------------------------------
bool IRImg_AR::CalcAreaTgtGray(IRTARGET * pt)//traj轨迹时，计算面目标的红外辐射与灰度
{//20090311这里需要修改，按对象计算红外辐射。

    float32 tgtrad;// = SearchRadInLookupTable(BbRadiance,TgtTmpr/*pt->tgtTmpr*/,IR_TMPR_MIN,IR_TMPR_MAX,IR_TMPR_DELTA);
    float32 fCamInceptPower_tmp;
    //单位转换要注意，因为用普朗克公式求得的W是瓦/cm2，因此这里要将探测器的单个像元面积由um2转化为cm2，所以乘以1e-8f。
    float32 Vlsb = (cam.cpara.camDet.advol2-cam.cpara.camDet.advol1)/pow(2.0f,cam.cpara.camDet.adres);//放大器电压范围/2e14，就是灰度值每增加1，增加的电压值
    float32 V_tmp = (cam.cpara.camDet.detresp/**1e8f*/) * cam.cpara.camDet.adgain;//探测器响应率 x 放大器增益
    float32 fCamInceptPower, V;

#if ADD_ATMOD_FLAG
    pt->airtrans = OnCalculate(&atm,pt);//用大气模块计算大气透过率
#else
    pt->airtrans = exp((-1.0f) * cam.cpara.camOpt.airattn * pt->CamTgtMod);//从本目标到探测器的大气透过率，大气衰减系数*目标到相机的距离
#endif

    //相机像元接收功率公式的固定值
    fCamInceptPower_tmp = (pt->airtrans * cam.cpara.camOpt.opttrans * cam.cpara.camDet.uthei * cam.cpara.camDet.utwid *1e-8f * pow(cam.cpara.camOpt.apt,2.0f)) / (4.0f * cam.cpara.camOpt.foc * cam.cpara.camOpt.foc);//将乘以verRad_tmp[i]的操作放到循环内进行，这里先算固定值

#if ADD_WHITENOISE_FLAG  //产生白噪声
    GenWhiteNoise(fCamInceptPower_tmp,V_tmp,Vlsb);
#endif //ADD_WHITENOISE_FLAG

    VECTOR3 nor1, nor2, nor, vec;
    float32 cosTheta, facetRad;
    graphicirrad->Vec3SetValue(&vec, 0.0f, 0.0f, -1.0f);
    //Vec3Subtract(&vec, &pTgt[IRScn.AimTgtNo].m_stat.pos,&cam.m_stat.pos);//test
    //Vec3Normalize(&vec, &vec);//test

    int32 start, stop ;
    start = (int32)pt->vxNo;
    stop = (int32)((pt->vxNum + pt->vxNo)/3.0f);
    for (int32 j =start; j < stop; j++)//计算每个面元的红外辐射
    {
        //这个放在这是为了之后添加面元蒙特卡洛方法时，对每个面元的温度进行查表计算。
        tgtrad = graphicirrad->SearchRadInLookupTable(BbRadiance,TgtTmpr,IR_TMPR_MIN,IR_TMPR_MAX,IR_TMPR_DELTA);//根据配置文件中设置的温度利用之前计算的查找表求黑体辐射

        graphicirrad->Vec3Subtract(&nor1, &IRScn.vxLst[3*j+1].p, &IRScn.vxLst[3*j].p);//得两个顶点三维矢量的差
        graphicirrad->Vec3Subtract(&nor2, &IRScn.vxLst[3*j+2].p, &IRScn.vxLst[3*j].p);//得两个顶点三维矢量的差
        graphicirrad->Vec3Cross(&nor, &nor1, &nor2);//二个三维矢量叉乘，求本面元的法线矢量
        graphicirrad->Vec3Normalize(&nor,&nor);//观察空间中的本面元的归一化法线矢量

        graphicirrad->Vec3Dot(&cosTheta, &nor, &vec);//二个三维矢量点乘，归一化法线与（0,0,-1）点乘

        if (cosTheta < 0)//背面剔除
        {
            /*
            IRScn.Visible[j] = 0;//标为不可见//这时速度会快，大部分都正常，但少数面元会缺失，如卫星星体中间交界处，或小船模型，不解
            */
            IRScn.Visible[j] = 1;//若标为全部可见，则虽然速度稍慢，但不会出现面元缺失情况
            cosTheta *= -1.0f;//若这时令为0则完全不对，条纹状

            //facetRad = tgtrad * 0.8f;
            facetRad = tgtrad  * 0.2f * 0.8f;//* materialF[j].ems//设发射率固定为0.8f，这里是把材料的发射率固定为0.8，也可计算每一种材料的发射率
            facetRad = facetRad + facetRad * 4.0f * cosTheta;//求出辐射(做了平均：facetRad = cosTheta * 0.8 * facetRad + 0.2 * facetRad)

            fCamInceptPower = fCamInceptPower_tmp * facetRad;
            V = fCamInceptPower * V_tmp + cam.cpara.camDet.adbias;
            IRScn.facetGray[j] =V / Vlsb;
            //cout << j << "  " << (int32)(IRScn.facetGray[j]) <<endl;

            //IRScn.facetGray[j] = (int32)((pt->FtTempr[j]-273)/**100*/);//debug only直接显示温度

            #if !MUL_SAMPLE_FLAG
            if ((IRScn.facetGray[j] > (pow(2.0f,cam.cpara.camDet.adres)-1.0f)))
            {//精细平面的灰度溢出不在这里判断，改在图像由精细采样平面恢复为普通平面时判断，否则如果sx=sy=2000的话，灰度16383/4000000＝0不显示了。
                IRScn.facetGray[j] = (pow(2.0f,cam.cpara.camDet.adres)-1.0f);//转为灰度
            }
            #endif

        }
        else
        {
            IRScn.Visible[j] = 1;//标为可见

            //facetRad = tgtrad * 0.8f;
            facetRad = tgtrad  * 0.2f * 0.8f;//* materialF[j].ems//设发射率固定为0.8f
            facetRad = facetRad + facetRad * 4.0f * cosTheta;//求出辐射(做了平均：facetRad = cosTheta * 0.8 * facetRad + 0.2 * facetRad)

            fCamInceptPower = fCamInceptPower_tmp * facetRad;
            V = fCamInceptPower * V_tmp + cam.cpara.camDet.adbias;
            IRScn.facetGray[j] =V / Vlsb;
            //cout << j << "  " << (int32)(IRScn.facetGray[j]) <<endl;

            //IRScn.facetGray[j] = (int32)((pt->FtTempr[j]-273)*100);//debug only直接显示温度

            #if !MUL_SAMPLE_FLAG
            if ((IRScn.facetGray[j] > (pow(2.0f,cam.cpara.camDet.adres)-1.0f)))
            {//精细采样平面灰度溢出不在这里判断，改在图像由精细采样平面恢复为普通平面时判断，否则如果sx=sy=2000的话，灰度16383/4000000＝0不显示了。
                IRScn.facetGray[j] = (pow(2.0f,cam.cpara.camDet.adres)-1.0f);//转为灰度
            }
            #endif
            /*if ((Gray[j] > (pow(2.0f,cam.cpara.camDet.adres)-1.0f)))
            {//灰度溢出不在这里判断，改在图像由精细采样平面恢复为普通平面时判断，否则如果sx=sy=2000的话，灰度16383/4000000＝0不显示了。
            Gray[j] = (pow(2.0f,cam.cpara.camDet.adres)-1.0f);//转为灰度
            }*/
        }
    }

    return true;
}

//------------计算投影与视区变换------------------------------------------------------
bool IRImg_AR::CalcProjViewportTransform(float32 timeDelta)
{//计算投影与视区变换
    //(目前作法：省去计算视区变换的步骤，
    //在计算投影时，不是映射到单位立方体上，
    //而是直接映射到宽高等于探测器宽高(像元个数乘以像元宽或高度)的长方体上)
    //所以在光栅化时，需要根据每个探测器像元的实际宽高度，
    //将现在的浮点坐标(代表了从探测器平面中点向四周的以米为单位的距离坐标)转化为整数坐标(屏幕坐标)

    int32 i, j ;

    //为了节省计算时间，CamWidLen,CamHeiLen,Max/MinClipWid,Max/MinClipHei,dx,dy这些每侦都要用到的不变的常量放在场景初始化中计算，只算一次

    //为了节省计算时间， 不用矩阵算变换，而是用手算
    //SetPerspectiveMatrix(matPerspective, CamWidLen, CamHeiLen, cam.cpara.camOpt.foc * 1e-3f,PERSPECTIVE_ZFAR);
    //SetViewportMatrix(matViewport, VIEWPORT_STARTX,VIEWPORT_STARTY, cam.cpara.camDet.wid, cam.cpara.camDet.hei,VIEWPORT_MINZ, VIEWPORT_MAXZ);
    //Mat16SetIdentity(matViewport);//暂时先用单位矩阵代替
    //MatrixMultiplyMatrix( matPerView, &matPerspective, &matViewport);



    for (i = 0; i < IRScn.tgtNum; i++)
    {
        for (j = pTgt[i].vxNo; j < (pTgt[i].vxNum+pTgt[i].vxNo); j++)
        {
            if(IRScn.Visible[(int)(j/3)] == 1)
            {
                IRScn.vxLst[j].p.x = dx * IRScn.vxLst[j].p.x/ fabs(IRScn.vxLst[j].p.z);//dx = 2f/w时可让x变换到正负1之内，
                IRScn.vxLst[j].p.y = dy * IRScn.vxLst[j].p.y/ fabs(IRScn.vxLst[j].p.z);//dy = 2f/h时可让y变换到正负1之内，这时z不变
                IRScn.vxLst[j].p.z = IRScn.vxLst[j].p.z ;
            }

            ProjWrapBox(&pTgt[i]);

            pTgt[i].TgtProjRect.x1 = pTgt[i].tgtbox.xpmin;
            pTgt[i].TgtProjRect.x2 = pTgt[i].tgtbox.xpmax;
            pTgt[i].TgtProjRect.y1 = pTgt[i].tgtbox.ypmin;
            pTgt[i].TgtProjRect.y2 = pTgt[i].tgtbox.ypmax;
            pTgt[i].TgtProjRect.dx = TGT_RES * cam.cpara.camOpt.foc * 1e-3f / pTgt[i].CamTgtMod;
            pTgt[i].TgtProjRect.dy = TGT_RES * cam.cpara.camOpt.foc * 1e-3f / pTgt[i].CamTgtMod;

        }
    }

    return true;
}

void IRImg_AR::ProjWrapBox(IRTARGET * pt)
{
    int32 i;
    float32 xx[8],yy[8];
    for (i=0; i<8; i++)
    {
        xx[i] = pt->tgtbox.bp[i].x * dx / fabs(pt->tgtbox.bp[i].z);
        yy[i] = pt->tgtbox.bp[i].y * dy / fabs(pt->tgtbox.bp[i].z);
    }
    float32 txpmin, txpmax,typmin, typmax;
    txpmin = txpmax = xx[0];
    typmin = typmax = yy[0];
    for (i=1; i<8; i++)
    {
        if (xx[i] < txpmin)
        {
            txpmin = xx[i];
        }
        if (xx[i] > txpmax)
        {
            txpmax = xx[i];
        }
        if (yy[i] < typmin)
        {
            typmin = yy[i];
        }
        if (yy[i] > typmax)
        {
            typmax = yy[i];
        }
    }

    pt->tgtbox.xpmin = txpmin;
    pt->tgtbox.xpmax = txpmax;
    pt->tgtbox.ypmin = typmin;
    pt->tgtbox.ypmax = typmax;

}

//--------------根据不同的分块光栅化方案，进行预处理，选择合适的渲染平面-------------------------------------
bool IRImg_AR::RasTypeSwitch(void)//本函数用在RasterizeFacets函数前，根据不同的分块光栅化方案，进行预处理，选择合适的渲染平面
{

    bool rassuc=true;//光栅化函数运行结果
    int32 tno;//加入目标索引
    int32 fno,fnum;//加入面元起始索引与数目，使渲染函数知道范围

    sq0 = 0;

#if ADD_BKG_FLAG //添加背景，根据光轴指向的目标反向移动背景
    RasterizeBkg(&bkg);
#endif//ADD_BKG_FLAG

#if ADD_IR_IMG  //添加实际拍摄的图像
    Rasterize_IRimg(&bkg);
#endif  //ADD_IR_IMG  //添加实际拍摄的图像

#if !MUL_SAMPLE_FLAG

    cam.CamSuperRect.x1 = cam.CamProjRect.x1;
    cam.CamSuperRect.x2 = cam.CamProjRect.x2;
    cam.CamSuperRect.y1 = cam.CamProjRect.y1;
    cam.CamSuperRect.y2 = cam.CamProjRect.y2;
    cam.CamSuperRect.wid = cam.cpara.camDet.wid;
    cam.CamSuperRect.hei = cam.cpara.camDet.hei;
    cam.CamSuperRect.z0 = cam.CamProjRect.z0;
    cam.CamSuperRect.sx = 1;
    cam.CamSuperRect.sy = 1;

    for (tno=0;tno<IRScn.tgtNum;tno++)
    {
        //fno = pTgt[tno].ftNo;
        //fnum = pTgt[tno].ftNum;
        fno = (int32)(pTgt[tno].vxNo/3);
        fnum = (int32)((pTgt[tno].vxNo + pTgt[tno].vxNum)/3);
        rassuc &= RasterizeFacets(tno,fno, fnum,&cam.CamSuperRect,sq0);//面元光栅化
    }

#endif // !MUL_SAMPLE_FLAG从循环中提取出来了

#if MUL_SAMPLE_FLAG//过采样只在PC仿真中用
    for (tno=0;tno<IRScn.tgtNum;tno++)
    {
        //fno = pTgt[tno].ftNo;
        //fnum = pTgt[tno].ftNum;
        fno = (int32)(pTgt[tno].vxNo/3);
        fnum = (int32)((pTgt[tno].vxNo + pTgt[tno].vxNum)/3);
        rassuc &= RasterizeFacets(tno,fno, fnum,&cam.CamSuperRect,sq0);//面元光栅化
    }
#endif//MUL_SAMPLE_FLAG
    return rassuc;
}

//============用环境参数重新计算实测图像中的背景灰度值，将结果直接写入光栅化frame缓存中====================
void IRImg_AR::Rasterize_IRimg(IRBKG * pbkg)
{
    //用环境参数重新计算矩形区域中的背景灰度值，将结果直接写入光栅化frame缓存中
    int32 i,j;
    long long sum = 0;
    float32 bpower,tpower;
    //====================计算背景中天空的灰度值======================
    bpower = pbkg->bems * graphicirrad->SearchRadInLookupTable( BbRadiance,pbkg->btmpr,IR_TMPR_MIN,IR_TMPR_MAX,IR_TMPR_DELTA);
    bpower = pbkg->airtrans * ( bpower * cam.cpara.camOpt.opttrans * cam.cpara.camDet.uthei * cam.cpara.camDet.utwid *1e-8f* pow(cam.cpara.camOpt.apt,2.0f)) / (4.0f * cam.cpara.camOpt.foc * cam.cpara.camOpt.foc);
    bpower = bpower * (cam.cpara.camDet.detresp/**1e8f*/) * cam.cpara.camDet.adgain + cam.cpara.camDet.adbias;
    pbkg->bgray = (uint32)(bpower/((cam.cpara.camDet.advol2-cam.cpara.camDet.advol1)/pow(2.0f,cam.cpara.camDet.adres)));
    //====================计算背景中云的灰度值======================
    tpower = pbkg->tems * graphicirrad->SearchRadInLookupTable( BbRadiance,pbkg->ttmpr,IR_TMPR_MIN,IR_TMPR_MAX,IR_TMPR_DELTA);
    tpower = pbkg->airtrans * ( tpower * cam.cpara.camOpt.opttrans * cam.cpara.camDet.uthei * cam.cpara.camDet.utwid *1e-8f* pow(cam.cpara.camOpt.apt,2.0f)) / (4.0f * cam.cpara.camOpt.foc * cam.cpara.camOpt.foc);
    tpower = tpower * (cam.cpara.camDet.detresp/**1e8f*/) * cam.cpara.camDet.adgain + cam.cpara.camDet.adbias;
    pbkg->tgray = (uint32)(tpower/((cam.cpara.camDet.advol2-cam.cpara.camDet.advol1)/pow(2.0f,cam.cpara.camDet.adres)));

    pbkg->maxgray = pbkg->bkgBuf[0];
    pbkg->mingray = pbkg->maxgray;
    pbkg->avggray = pbkg->maxgray;

    for (i=0; i<cam.cpara.camDet.wid; i++)
    {
        for (j=0; j<cam.cpara.camDet.hei; j++)
        {

            if (pbkg->maxgray < pbkg->bkgBuf[j*cam.cpara.camDet.wid+i])
            {
                pbkg->maxgray = pbkg->bkgBuf[j*cam.cpara.camDet.wid+i];
            }
            else if (pbkg->mingray > pbkg->bkgBuf[j*cam.cpara.camDet.wid+i])
            {
                pbkg->mingray = pbkg->bkgBuf[j*cam.cpara.camDet.wid+i];
            }
            sum += pbkg->bkgBuf[j*cam.cpara.camDet.wid+i];

        }
    }
    pbkg->avggray = sum/(cam.cpara.camDet.wid*cam.cpara.camDet.hei);

    float32 kk,bb;
    kk = (float32)pbkg->bgray/(float32)pbkg->avggray;
    bb = (float32)pbkg->tgray/(float32)(pbkg->maxgray - pbkg->mingray);
    /*cout << "kk = " << kk << endl;
    cout << "bb = " << bb << endl;
    cout << "pbkg->bgray = " << pbkg->bgray <<endl;
    cout << "pbkg->avggray = " << pbkg->avggray <<endl;*/

    for (i=0; i<cam.cpara.camDet.wid; i++)
    {
        for (j=0; j<cam.cpara.camDet.hei; j++)
        {
            IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[j*cam.cpara.camDet.wid+i]);//*kk+bb;
        }
    }
}

//--------------------用环境参数重新计算矩形区域中的背景灰度值，将结果直接写入光栅化frame缓存中--------------------------------
void IRImg_AR::RasterizeBkg(IRBKG * pbkg)
{	//用环境参数重新计算矩形区域中的背景灰度值，将结果直接写入光栅化frame缓存中
    int32 i,j;

    float32 bpower,tpower;
    //====================计算背景中天空的灰度值======================
    bpower = pbkg->bems * graphicirrad->SearchRadInLookupTable( BbRadiance,pbkg->btmpr,IR_TMPR_MIN,IR_TMPR_MAX,IR_TMPR_DELTA);
    bpower = pbkg->airtrans * ( bpower * cam.cpara.camOpt.opttrans * cam.cpara.camDet.uthei * cam.cpara.camDet.utwid *1e-8f* pow(cam.cpara.camOpt.apt,2.0f)) / (4.0f * cam.cpara.camOpt.foc * cam.cpara.camOpt.foc);
    bpower = bpower * (cam.cpara.camDet.detresp/**1e8f*/) * cam.cpara.camDet.adgain + cam.cpara.camDet.adbias;
    pbkg->bgray = (uint32)(bpower/((cam.cpara.camDet.advol2-cam.cpara.camDet.advol1)/pow(2.0f,cam.cpara.camDet.adres)));
    //====================计算背景中云的灰度值======================
    tpower = pbkg->tems * graphicirrad->SearchRadInLookupTable( BbRadiance,pbkg->ttmpr,IR_TMPR_MIN,IR_TMPR_MAX,IR_TMPR_DELTA);
    tpower = pbkg->airtrans * ( tpower * cam.cpara.camOpt.opttrans * cam.cpara.camDet.uthei * cam.cpara.camDet.utwid *1e-8f* pow(cam.cpara.camOpt.apt,2.0f)) / (4.0f * cam.cpara.camOpt.foc * cam.cpara.camOpt.foc);
    tpower = tpower * (cam.cpara.camDet.detresp/**1e8f*/) * cam.cpara.camDet.adgain + cam.cpara.camDet.adbias;
    pbkg->tgray = (uint32)(tpower/((cam.cpara.camDet.advol2-cam.cpara.camDet.advol1)/pow(2.0f,cam.cpara.camDet.adres)));

    SearchMaxMinAvgBkgGray(pbkg->bkgBuf,&bkg);

    float32 kk,bb;
    kk = (float32)pbkg->bgray/(float32)pbkg->avggray;
    bb = (float32)pbkg->tgray/(float32)(pbkg->maxgray - pbkg->mingray);

    //判断确保指定区域完全落在背景范围内，如果溢出则对背景进行镜像
    int32 widtm, heitm;
    widtm = pbkg->widstart / pbkg->wid;
    heitm = pbkg->heistart / pbkg->hei;
    pbkg->widstart = pbkg->widstart<0?(pbkg->wid+pbkg->widstart):pbkg->widstart;
    pbkg->heistart = pbkg->heistart<0?(pbkg->hei+pbkg->heistart):pbkg->heistart;
    pbkg->widstart = pbkg->widstart - widtm * pbkg->wid;
    pbkg->heistart = pbkg->heistart - heitm * pbkg->hei;

    if (pbkg->widstart + cam.cpara.camDet.wid < pbkg->wid && pbkg->heistart + cam.cpara.camDet.hei < pbkg->hei)
    {//小窗口完全落在大窗口内
        for (i=0; i<cam.cpara.camDet.wid; i++)
        {
            for (j=0; j<cam.cpara.camDet.hei; j++)
            {
                IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart]*kk+bb);
            }
        }
    }

    else if (pbkg->widstart + cam.cpara.camDet.wid >= pbkg->wid && pbkg->heistart + cam.cpara.camDet.hei < pbkg->hei)
    {//小窗口只有右半部分落在大窗口外
            for (j=0; j<cam.cpara.camDet.hei; j++)
            {
                for (i=0; i<(pbkg->wid-pbkg->widstart);i++)
                {
                    IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart]*kk+bb);
                }
                for (i=(pbkg->wid-pbkg->widstart);i<cam.cpara.camDet.wid;i++)
                {
                    IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-1-i-pbkg->widstart]*kk+bb);
                }
            }

    }

    else if (pbkg->widstart + cam.cpara.camDet.wid < pbkg->wid && pbkg->heistart + cam.cpara.camDet.hei >= pbkg->hei)
    {//小窗口只有下半部分落在大窗口外
        for (i=0; i<cam.cpara.camDet.wid; i++)
        {
            for (j=0; j<(pbkg->hei-pbkg->heistart); j++)
            {
                IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart]*kk+bb);
            }
            for (j=(pbkg->hei-pbkg->heistart);j<cam.cpara.camDet.hei;j++)
            {
                IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[(2*pbkg->hei-1-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart]*kk+bb);
            }
        }

    }

    else if (pbkg->widstart + cam.cpara.camDet.wid >= pbkg->wid && pbkg->heistart + cam.cpara.camDet.hei >= pbkg->hei)
    {//小窗口的右半部分和下半部分均落在大窗口外


        for (i=0; i<(pbkg->wid-pbkg->widstart); i++)
        {
            for (j=0; j<(pbkg->hei-pbkg->heistart); j++)
            {
                IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart]*kk+bb);
            }
            for (j=(pbkg->hei-pbkg->heistart);j<cam.cpara.camDet.hei;j++)
            {
                IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[(2*pbkg->hei-1-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart]*kk+bb);
            }
        }

        for (i=(pbkg->wid-pbkg->widstart); i<cam.cpara.camDet.wid; i++)
        {
            for (j=0; j<(pbkg->hei-pbkg->heistart); j++)
            {
                IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-1-i-pbkg->widstart]*kk+bb);
            }
            for (j=(pbkg->hei-pbkg->heistart);j<cam.cpara.camDet.hei;j++)
            {
                IRScn.Gray[j*cam.cpara.camDet.wid+i]=(uint32)(pbkg->bkgBuf[(2*pbkg->hei-1-j-pbkg->heistart)*pbkg->wid+2*pbkg->wid-1-i-pbkg->widstart]*kk+bb);
            }
        }
    }

}

//=================在当前的灰度数组中指定的矩形区域内找出最大最小平均灰度===============
void IRImg_AR::SearchMaxMinAvgBkgGray(unsigned short * graybuf,IRBKG * pbkg)
{//在当前的灰度数组中指定的矩形区域内找出最大最小平均灰度(指定区域为从widstart,heistart开始的地方，有cam.cpara.camDet.wid*cam.cpara.camDet.hei大小的区域）
    int32 i, j;
    int32 sum=0;
    //for (i=pbkg->widstart;i<pbkg->widstart+cam.cpara.camDet.wid;i++)
    //{
    //	for (j=pbkg->heistart;j<pbkg->heistart+cam.cpara.camDet.hei;j++)
    //	{
    //		if (pbkg->maxgray<graybuf[i*pbkg->hei+j])
    //		{
    //			pbkg->maxgray = graybuf[i*pbkg->hei+j];
    //		}
    //		else if (pbkg->mingray > graybuf[i*pbkg->hei+j])
    //		{
    //			pbkg->mingray = graybuf[i*pbkg->hei+j];
    //		}
    //		sum += graybuf[i*pbkg->hei+j];
    //	}
    //}
    //pbkg->avggray = sum/(cam.cpara.camDet.wid*cam.cpara.camDet.hei);

    //判断确保指定区域完全落在背景范围内，如果溢出则对背景进行镜像
    int32 widtm, heitm;
    widtm = pbkg->widstart / pbkg->wid;
    heitm = pbkg->heistart / pbkg->hei;
    pbkg->widstart = pbkg->widstart<0?(pbkg->wid+pbkg->widstart):pbkg->widstart;
    pbkg->heistart = pbkg->heistart<0?(pbkg->hei+pbkg->heistart):pbkg->heistart;
    pbkg->widstart = pbkg->widstart - widtm * pbkg->wid;
    pbkg->heistart = pbkg->heistart - heitm * pbkg->hei;


    pbkg->maxgray = graybuf[pbkg->widstart*pbkg->hei+pbkg->heistart];
    pbkg->mingray = pbkg->maxgray;
    pbkg->avggray = pbkg->maxgray;

    if (pbkg->widstart + cam.cpara.camDet.wid < pbkg->wid && pbkg->heistart + cam.cpara.camDet.hei < pbkg->hei)
    {//小窗口完全落在大窗口内
        for (i=0; i<cam.cpara.camDet.wid; i++)
        {
            for (j=0; j<cam.cpara.camDet.hei; j++)
            {

                if (pbkg->maxgray<pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->maxgray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                else if (pbkg->mingray > pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->mingray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                sum += pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];

            }
        }
    }

    else if (pbkg->widstart + cam.cpara.camDet.wid >= pbkg->wid && pbkg->heistart + cam.cpara.camDet.hei < pbkg->hei)
    {//小窗口只有右半部分落在大窗口外
        for (j=0; j<cam.cpara.camDet.hei; j++)
        {
            for (i=0; i<(pbkg->wid-pbkg->widstart);i++)
            {
                if (pbkg->maxgray<pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->maxgray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                else if (pbkg->mingray > pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->mingray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                sum += pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
            }
            for (i=(pbkg->wid-pbkg->widstart);i<cam.cpara.camDet.wid;i++)
            {

                if (pbkg->maxgray<pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart])
                {
                    pbkg->maxgray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart];
                }
                else if (pbkg->mingray > pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart])
                {
                    pbkg->mingray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart];
                }
                sum += pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart];
            }
        }

    }

    else if (pbkg->widstart + cam.cpara.camDet.wid < pbkg->wid && pbkg->heistart + cam.cpara.camDet.hei >= pbkg->hei)
    {//小窗口只有下半部分落在大窗口外
        for (i=0; i<cam.cpara.camDet.wid; i++)
        {
            for (j=0; j<(pbkg->hei-pbkg->heistart); j++)
            {
                if (pbkg->maxgray<pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->maxgray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                else if (pbkg->mingray > pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->mingray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                sum += pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
            }
            for (j=(pbkg->hei-pbkg->heistart);j<cam.cpara.camDet.hei;j++)
            {

                if (pbkg->maxgray<pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->maxgray = pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                else if (pbkg->mingray > pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->mingray = pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                sum += pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
            }
        }

    }

    else if (pbkg->widstart + cam.cpara.camDet.wid >= pbkg->wid && pbkg->heistart + cam.cpara.camDet.hei >= pbkg->hei)
    {//小窗口的右半部分和下半部分均落在大窗口外


        for (i=0; i<(pbkg->wid-pbkg->widstart); i++)
        {
            for (j=0; j<(pbkg->hei-pbkg->heistart); j++)
            {
                if (pbkg->maxgray<pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->maxgray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                else if (pbkg->mingray > pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->mingray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                sum += pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
            }
            for (j=(pbkg->hei-pbkg->heistart);j<cam.cpara.camDet.hei;j++)
            {

                if (pbkg->maxgray<pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->maxgray = pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                else if (pbkg->mingray > pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart])
                {
                    pbkg->mingray = pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
                }
                sum += pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+i+pbkg->widstart];
            }
        }

        for (i=(pbkg->wid-pbkg->widstart); i<cam.cpara.camDet.wid; i++)
        {
            for (j=0; j<(pbkg->hei-pbkg->heistart); j++)
            {

                if (pbkg->maxgray<pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart])
                {
                    pbkg->maxgray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart];
                }
                else if (pbkg->mingray > pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart])
                {
                    pbkg->mingray = pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart];
                }
                sum += pbkg->bkgBuf[(j+pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart];
            }
            for (j=(pbkg->hei-pbkg->heistart);j<cam.cpara.camDet.hei;j++)
            {

                if (pbkg->maxgray<pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart])
                {
                    pbkg->maxgray = pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart];
                }
                else if (pbkg->mingray > pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart])
                {
                    pbkg->mingray = pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart];
                }
                sum += pbkg->bkgBuf[(2*pbkg->hei-j-pbkg->heistart)*pbkg->wid+2*pbkg->wid-i-pbkg->widstart];
            }
        }
    }

    pbkg->avggray = sum/(cam.cpara.camDet.wid*cam.cpara.camDet.hei);
}

//---------------------面元光栅化---------------------------------------------------
bool IRImg_AR::RasterizeFacets(int32 tno, int32 fno,int32 fnum, CAMSURECT * rc, uint32 sq0)
{//面元光栅化
    //现在得到的参数有投影坐标(以探测器的实际宽高度(米)做为坐标)、面元可见情况、面元灰度值
    //需要使用的中间变量有图像宽度*高度大小的深度值
    //输出结果是图像宽度*高度大小的灰度值(即图像)
    //int32 tno;
    int32 j;

    //for (tno=0; tno<IRScn.tgtNum; tno++)
    //{

#if MUL_SAMPLE_FLAG

    int32 k;
    int32 l;
    int32 snn;
    uint32 sq;
    uint32 sq2;
    int32 tmpGray = 0;
    uint32 * tmpGrayPtr;

    cam.CamProjRect.xr = cam.cpara.camDet.utwid * 1e-6f;//(CamProjRect.x2 - CamProjRect.x1) / CamProjRect.wid;
    cam.CamProjRect.yr = cam.cpara.camDet.uthei * 1e-6f;//(CamProjRect.y2 - CamProjRect.y1) / CamProjRect.hei;

    if (pTgt[tno].TgtProjRect.x1 <= cam.CamProjRect.x1) TgtNx1 = 0;
    else TgtNx1 = (int32)((pTgt[tno].TgtProjRect.x1 - cam.CamProjRect.x1) / cam.CamProjRect.xr);
    if (pTgt[tno].TgtProjRect.x2 >= cam.CamProjRect.x2) TgtNx2 = cam.CamProjRect.wid;
    else TgtNx2 = cam.CamProjRect.wid - (int32)((cam.CamProjRect.x2 - pTgt[tno].TgtProjRect.x2) / cam.CamProjRect.xr);
    if (pTgt[tno].TgtProjRect.y1 <= cam.CamProjRect.y1) TgtNy1 = 0;
    else TgtNy1 = (int32)((pTgt[tno].TgtProjRect.y1 - cam.CamProjRect.y1) / cam.CamProjRect.yr);
    if (pTgt[tno].TgtProjRect.y2 >= cam.CamProjRect.y2) TgtNy2 = cam.CamProjRect.hei;
    else TgtNy2 = cam.CamProjRect.hei -(int32) ((cam.CamProjRect.y2 - pTgt[tno].TgtProjRect.y2) / cam.CamProjRect.yr);
    if (TgtNx1 >= TgtNx2 || TgtNy1 >= TgtNy2)
    {
        printf("nx1>=nx2 or ny1 >=ny2, target is unvisible\n");
        return false;
    }

    TgtGx1 = cam.CamProjRect.x1 + TgtNx1 * cam.CamProjRect.xr;
    TgtGx2 = cam.CamProjRect.x1 + TgtNx2 * cam.CamProjRect.xr;
    TgtGy1 = cam.CamProjRect.y1 + TgtNy1 * cam.CamProjRect.yr;
    TgtGy2 = cam.CamProjRect.y1 + TgtNy2 * cam.CamProjRect.yr;

    cam.CamSuperRect.sx = (int32)(cam.CamProjRect.xr/pTgt[tno].TgtProjRect.dx + 0.5f);//1.0f);//
    cam.CamSuperRect.sy = (int32)(cam.CamProjRect.yr/pTgt[tno].TgtProjRect.dy + 0.5f);//1.0f);//
    if (cam.CamSuperRect.sx < 3) cam.CamSuperRect.sx = 3;
    if (cam.CamSuperRect.sy < 3) cam.CamSuperRect.sy = 3;
    //if (CamSuperRectSx >1000) CamSuperRectSx = 1000;//如果手动将采样率调低，则系统内存不会用尽，但很远的点不能显示了
    //if (CamSuperRectSy >1000) CamSuperRectSy = 1000;

    cam.CamSuperRect.x1 = TgtGx1;
    cam.CamSuperRect.x2 = TgtGx2;
    cam.CamSuperRect.y1 = TgtGy1;
    cam.CamSuperRect.y2 = TgtGy2;
    cam.CamSuperRect.wid = (TgtNx2 - TgtNx1) * cam.CamSuperRect.sx;
    cam.CamSuperRect.hei = (TgtNy2 - TgtNy1) * cam.CamSuperRect.sy;
    cam.CamSuperRect.z0 = cam.CamProjRect.z0;

    ImgTmpGrayBuf = (uint32 *) malloc(cam.CamSuperRect.wid * cam.CamSuperRect.hei * sizeof(uint32));//在CalcProjViewportTransform函数中malloc,在RasterizeFacets中free。
    if (ImgTmpGrayBuf == NULL)
    {
        printf("there isn't enough space to malloc ImgTmpGrayBuf \n ");
        return false;
    }
    ImgTmpZdepth = (float32 *)malloc(cam.CamSuperRect.wid * cam.CamSuperRect.hei * sizeof(float32));//在CalcProjViewportTransform函数中malloc,在RasterizeFacets中free。
    if (ImgTmpZdepth == NULL)
    {
        printf("there isn't enough space to malloc ImgTmpZdepth \n ");
        return false;
    }

    //这里若是使用汇编语言将大大加速，每一帧都要进行清零的。
    memset(ImgTmpGrayBuf,0,cam.CamSuperRect.wid * cam.CamSuperRect.hei*sizeof(uint32)); //清理灰度缓冲为固定值
    memset(ImgTmpZdepth, 0.0f, cam.CamSuperRect.wid * cam.CamSuperRect.hei*sizeof(float32));//清理Z缓冲为固定值

    tmpGrayPtr = ImgTmpGrayBuf;

    rc = &cam.CamSuperRect;
#endif//MUL_SAMPLE_FLAG

#if !MUL_SAMPLE_FLAG//后面的cam.CamSuperRect.全改为了rc->
    if (pTgt[tno].TgtProjRect.x1 >= rc->x2 || pTgt[tno].TgtProjRect.x2 <= rc->x1 || pTgt[tno].TgtProjRect.y1 >= rc->y2 || pTgt[tno].TgtProjRect.y2 <= rc->y1 )
    {
        printf("target is unvisible\n");
        return false;
    }
#endif //!MUL_SAMPLE_FLAG , end of old MUL_SAMPLE_FLAG

    //for (j = pTgt[tno].ftNo; j < pTgt[tno].ftNo + pTgt[tno].ftNum; j++)
    for (j = fno; j < fno + fnum; j++)
    {//读入顶点变换后的坐标、面元灰度、面元可见状态,进行光栅化
        if (IRScn.Visible[j] == 1)//当前面元可见
        {
            RasterizeSingleFacet(j,rc,sq0);
        }
    }	//暂时不显示目标，只显示背景


#if MUL_SAMPLE_FLAG
    //由精细采样平面变回相机原始平面
    snn = cam.CamSuperRect.sx * cam.CamSuperRect.sy;
    int32 i;

    if (ImgTmpGrayBuf != NULL)
    {
        for(i=TgtNy1;i<TgtNy2;i++)
            for(j=TgtNx1;j<TgtNx2;j++)
            {
                sq = (cam.CamProjRect.wid)*i+j;
                tmpGray = 0;
                for(k=0;k<rc->sy;k++)
                    for(l=0;l<rc->sx;l++)
                    {
                        sq2= (rc->wid)*((i-TgtNy1)*rc->sy+k)+ ((j-TgtNx1)*rc->sx+l);
                        tmpGray += tmpGrayPtr[sq2];
                        //op += cam->opBuf_t[sq2];

                    }
                    IRScn.Gray[sq] += tmpGray/snn;// +=可防止目标覆盖
                    if (IRScn.Gray[sq] > (uint32)(pow(2.0f, cam.cpara.camDet.adres)-1.0f))
                    {
                        IRScn.Gray[sq] = (uint32)(pow(2.0f, cam.cpara.camDet.adres)-1.0f);//防止灰度溢出，本来是在IRRadiance里做的，为防止过采样后灰度均值为零放在这里了
                    }
            }
        SAFE_FREE(ImgTmpGrayBuf);
    }
    SAFE_FREE(ImgTmpZdepth);
#endif
    //}

    return true;

}

//====================渲染某片可见的三角形面元============================
bool IRImg_AR::RasterizeSingleFacet(int32 fno, CAMSURECT * rc, uint32 sq0)
{//渲染某片可见的三角形面元

    VECTOR3 v0,v1,v2;//三个顶点的投影坐标
    int32 i=0, j=0;
    int32 x0=0, y0=0, x1=0, y1=0;//, x2, y2;//三个顶点转化后的整数屏幕坐标
    float32 xcurr=0.0f, ycurr=0.0f;//当前待填充的点的坐标
    float32 xstep=0.0f, ystep=0.0f;//用于计算投影坐标转化到整数屏幕坐标时的步长
    float32 ca=0.0f, cb=0.0f,cc=0.0f, za=0.0f, zb=0.0f, zc=0.0f,zd=0.0f, xa=0.0f, xb=0.0f;//用于计算插值比例的系数:首先由y的比例求出cacb，由此求出相应的xaxb和zazb的值，然后在每一行(y固定)扫描线中，由x在xaxb中的比例求出cc，由此加上zazb即可求出当前像素的深度值zc，作为1/Z Buffer的判断来源(1/z buffer比z buffer更正确，因为1/z是线性分布的),目前未用1/Zbuffer，而是用的zbuffer,因为投影时未将z求倒数
    float32 xmax=0.0f, xmin=0.0f;//当前的扫描线的x的起始值与结束值

    uint32 sq=0;//当前像素在整个图像从左上角开始的序号(PC渲染　RASTYPE!=0x1100的情况适用)
    //#if RASTYPE_FLAG == 0x1100
    //	uint32 sq0=0;//渲染RASTYPE=0x1100的情况下，如果rcnum>1，则需要计算起始偏移值sq0
    //#endif//RASTYPE_FLAG ==0x1100

    int32 i1,i2,i3;//,im
#if MUL_SAMPLE_FLAG
    uint32 * tmpGrayPtr = ImgTmpGrayBuf;
    float32 * tmpZdepthPtr = ImgTmpZdepth;
#else
    uint32 * tmpGrayPtr = IRScn.Gray;
    float32 * tmpZdepthPtr = IRScn.ZDepth;
#endif



    i1 = fno*3;//IRScn.ftLst[fno].i1;
    i2 = fno*3+1;//IRScn.ftLst[fno].i2;
    i3 = fno*3+2;//IRScn.ftLst[fno].i3;
    //im = IRScn.ftLst[fno].im;

    if ((IRScn.vxLst[i1].p.x == IRScn.vxLst[i2].p.x  && IRScn.vxLst[i2].p.x == IRScn.vxLst[i3].p.x) ||
        (IRScn.vxLst[i1].p.y == IRScn.vxLst[i2].p.y  && IRScn.vxLst[i2].p.y == IRScn.vxLst[i3].p.y))
    {//该面元是条水平或垂直的线，无法渲染，返回
        return false;
    }

    if (((IRScn.vxLst[i1].p.x <= rc->x1)  && (IRScn.vxLst[i2].p.x <= rc->x1)  && (IRScn.vxLst[i3].p.x <= rc->x1)) ||
        ((IRScn.vxLst[i1].p.x >= rc->x2)  && (IRScn.vxLst[i2].p.x >= rc->x2)  && (IRScn.vxLst[i3].p.x >= rc->x2)) ||
        ((IRScn.vxLst[i1].p.y <= rc->y1)  && (IRScn.vxLst[i2].p.y <= rc->y1)  && (IRScn.vxLst[i3].p.y <= rc->y1)) ||
        ((IRScn.vxLst[i1].p.y >= rc->y2)  && (IRScn.vxLst[i2].p.y >= rc->y2)  && (IRScn.vxLst[i3].p.y >= rc->y2)))
    {//本面元投影落在了屏幕外面，不可见，返回
        return false;
    }

    /////////////////////
    ////按y坐标，从小到大排序，v0.y<v1.y<v2.y
    if (IRScn.vxLst[i1].p.y > IRScn.vxLst[i2].p.y)
    {
        graphicirrad->Vec3SetValue(&v0, IRScn.vxLst[i2].p.x,IRScn.vxLst[i2].p.y,IRScn.vxLst[i2].p.z);
        graphicirrad->Vec3SetValue(&v1, IRScn.vxLst[i1].p.x,IRScn.vxLst[i1].p.y,IRScn.vxLst[i1].p.z);
    }
    else
    {
        graphicirrad->Vec3SetValue(&v1, IRScn.vxLst[i2].p.x,IRScn.vxLst[i2].p.y,IRScn.vxLst[i2].p.z);
        graphicirrad->Vec3SetValue(&v0, IRScn.vxLst[i1].p.x,IRScn.vxLst[i1].p.y,IRScn.vxLst[i1].p.z);
    }
    if (IRScn.vxLst[i3].p.y > v1.y)
    {
        graphicirrad->Vec3SetValue(&v2, IRScn.vxLst[i3].p.x,IRScn.vxLst[i3].p.y,IRScn.vxLst[i3].p.z);
    }
    else
    {
        if (IRScn.vxLst[i3].p.y > v0.y)
        {
            graphicirrad->Vec3SetValue(&v2, v1.x, v1.y, v1.z);
            graphicirrad->Vec3SetValue(&v1, IRScn.vxLst[i3].p.x, IRScn.vxLst[i3].p.y, IRScn.vxLst[i3].p.z);
        }
        else
        {
            graphicirrad->Vec3SetValue(&v2, v1.x, v1.y, v1.z);
            graphicirrad->Vec3SetValue(&v1, v0.x, v0.y, v0.z);
            graphicirrad->Vec3SetValue(&v0, IRScn.vxLst[i3].p.x, IRScn.vxLst[i3].p.y, IRScn.vxLst[i3].p.z);
        }
    }
    /////////////////////

    // Fill lower part of triangle
    xstep = (rc->x2 - rc->x1) / rc->wid;
    ystep = (rc->y2 - rc->y1) / rc->hei;
    if(v0.y < rc->y1) y0 = 0;
    else y0 = (int32)((v0.y- rc->y1)/ystep + 0.5f);//加0.5可有效防止转整数时的误差，防止图像出现白色横线

    if(v1.y > rc->y2) y1 = rc->hei;
    else y1 = (int32)((v1.y- rc->y1)/ystep + 0.5f);

    i= y0;
    while(i<y1)
    {
        ycurr = rc->y1 + (i + 0.5f) * ystep;
        ca = (ycurr - v0.y) / (v1.y - v0.y);
        cb= (ycurr - v0.y) / (v2.y - v0.y);

        za = v0.z + (v1.z - v0.z)* ca;
        zb = v0.z + (v2.z - v0.z)* cb;

        xa = v0.x + (v1.x - v0.x)* ca;
        xb = v0.x + (v2.x - v0.x)* cb;
        if(xa > xb)
        {
            xmax = xa;
            xmin = xb;
        }
        else
        {
            xmax = xb;
            xmin = xa;
        }
        if((xmin <= rc->x2) && (xmax >= rc->x1))
        {
            if(xmin < rc->x1) x0 = 0;
            else x0 = (int32)((xmin - rc->x1) / xstep + 0.5f);

            if(xmax > rc->x2) x1 = rc->wid;
            else x1 = (int32)((xmax - rc->x1) / xstep + 0.5f);

            for(j = x0; j < x1; j++)
            {
                sq = i * rc->wid + j;

                xcurr = rc->x1 + (j + 0.5f) * xstep;
                cc = (xcurr - xa) / (xb - xa);

                zc = za + (zb - za) * cc;
                zd = 1.0f / zc;//(CamSuperRect.z0 * zc);//
                //#if MUL_SAMPLE_FLAG
                if((zd > tmpZdepthPtr[sq+sq0])/*&&(zd < 10.0f)*/)//zbuffer时新z小于旧z时可见，1/zbuffer时新z大于旧z时可见
                {//深度判断发现面元的z大于阈值，可见
                    tmpZdepthPtr[sq+sq0] = zd;
                    tmpGrayPtr[sq+sq0] =(uint32) IRScn.facetGray[fno];
                }
                //#else
                //if(zd > ImgZdepth[sq])//zbuffer时新z小于旧z时可见，1/zbuffer时新z大于旧z时可见
                //{//深度判断发现面元的z大于阈值，可见
                //	ImgZdepth[sq] = zd;
                //	ImgGrayBuf[sq] =(uint32) Gray[facetNo];
                //}

                //#endif
            }
        }
        i++;
    }
    //fprintf(fpdebug, "fno= %d,v0=(%f, %f, %f), v1=(%f, %f, %f), v2=(%f, %f, %f)\n     x0= %d, y0= %d,  x1= %d, y1= %d\n",fno,v0.x, v0.y, v0.z,v1.x, v1.y, v1.z,v2.x, v2.y, v2.z, x0,y0,x1,y1);

    // Fill upper part of triangle
    if(v1.y < rc->y1) y0 = 0;
    else y0 = (int32)((v1.y - rc->y1) / ystep + 0.5f);

    if(v2.y > rc->y2) y1 = rc->hei;
    else y1 = (int32)((v2.y - rc->y1) / ystep + 0.5f);

    i = y0;
    while(i<y1)
    {
        ycurr = rc->y1 + (i + 0.5f) * ystep;
        ca = (ycurr - v1.y) / (v2.y - v1.y);
        cb = (ycurr - v0.y) / (v2.y - v0.y);


        za = v1.z + (v2.z - v1.z) * ca;
        zb = v0.z + (v2.z - v0.z) * cb;

        xa = v1.x + (v2.x - v1.x) * ca;
        xb = v0.x + (v2.x - v0.x) * cb;
        if(xa > xb)
        {
            xmax = xa;
            xmin = xb;
        }
        else
        {
            xmax = xb;
            xmin = xa;
        }
        if((xmin <= rc->x2) && (xmax >= rc->x1))
        {
            if(xmin < rc->x1) x0 = 0;
            else x0 = (int32)((xmin - rc->x1) / xstep + 0.5f);

            if(xmax > rc->x2) x1= rc->wid;
            else x1 = (int32)((xmax - rc->x1) / xstep + 0.5f);

            for(j = x0; j < x1; j++)
            {
                sq = i * rc->wid + j;

                xcurr = rc->x1 + (j + 0.5f) * xstep;
                cc = (xcurr - xa) / (xb - xa);

                zc = za + (zb - za) * cc;
                zd = 1.0f / zc;//(CamSuperRect.z0 * zc);//

                //#if MUL_SAMPLE_FLAG
                if((zd > tmpZdepthPtr[sq+sq0])/*&&(zd < 10.0f)*/)//zbuffer时新z小于旧z时可见，1/zbuffer时新z大于旧z时可见
                {//深度判断发现面元的z大于阈值，可见
                    tmpZdepthPtr[sq+sq0] = zd;
                    tmpGrayPtr[sq+sq0] =(uint32) IRScn.facetGray[fno];
                }
                //#else
                //if(zd > ImgZdepth[sq])//zbuffer时新z小于旧z时可见，1/zbuffer时新z大于旧z时可见
                //{//深度判断发现面元的z大于阈值，可见
                //	ImgZdepth[sq] = zd;
                //	ImgGrayBuf[sq] =(uint32) Gray[facetNo];
                //}

                //#endif
            }
        }
        i++;
    }

    /////////////////////
    return true;
}

//----------加白噪声,使得符合SNR要求-----------------------------------------------------
void IRImg_AR::AddWhiteNoise(void)
{//加白噪声,使得符合SNR要求
    int32 i=0;
    for (i=0; i<cam.cpara.camDet.hei*cam.cpara.camDet.wid;i++)
    {
        IRScn.Gray[i] += whitenoise[i];
    }
}

//-----------场景初始化时计算白噪声-----------------------------------------------------
void IRImg_AR::GenWhiteNoise(float32 fCamInceptPower_tmp,float32 V_tmp,float32 Vlsb)
{//场景初始化时计算白噪声
    int32 maxNoiseGray;
    float32 fr;

    float32 tmprad = stemi * graphicirrad->SearchRadInLookupTable(BbRadiance,TgtTmpr,IR_TMPR_MIN,IR_TMPR_MAX,IR_TMPR_DELTA);//根据配置文件中设置的温度利用之前计算的查找表求黑体辐射
    float32 fCamInceptPower = fCamInceptPower_tmp * tmprad;
    float32 V = fCamInceptPower * V_tmp + cam.cpara.camDet.adbias;

    maxNoiseGray  = (int32)((V / Vlsb) / scnSNR);
    srand((uint32)time(NULL));
    for (int32 i=0; i<cam.cpara.camDet.hei*cam.cpara.camDet.wid;i++)
    {
        fr = (float32)rand()/(float32)RAND_MAX;
        whitenoise[i] = (int32)(fr * maxNoiseGray);
    }

}

//------------------清除内存中所有的场景缓存--------------------------------------------------
bool IRImg_AR::CleanScnBuffer(void)
{//Desc: to clean all scn buffer in memory
    int32 i;

    for (i = 0; i < IRScn.tgtNum; i++)
    {
        SAFE_FREE (pTgt[i].m_pTraj);
    }
    SAFE_FREE(cam.m_pTraj);
    SAFE_FREE (IRScn.ftgInfoLst);
    SAFE_FREE (IRScn.mdl_vxLst);
    SAFE_FREE (IRScn.vxLst);
    SAFE_FREE (IRScn.mtLst);

    SAFE_FREE (IRScn.facetGray);
    SAFE_FREE (IRScn.Visible);

    SAFE_FREE (IRScn.Gray);
    SAFE_FREE (IRScn.ZDepth);

    SAFE_FREE (pTgt);

    SAFE_FREE (IRScn.pmLst);

    SAFE_FREE (bkg.bkgBuf);
    SAFE_FREE (buf_recv);

    SAFE_FREE (whitenoise);

    delete graphicirrad;

    fclose(fpdebug);

    return true;
}

//------------------光线追迹法实现目标渲染-------------------------------------------------
void IRImg_AR::RayRender(float32 timeDelta)
{
    //float32 WindowX1,WindowX2,WindowY1,WindowY2,WindowZ1,WindowZ2;
    float32 StartX,StartY,StartZ;
    //float32 angle;
    //VECTOR3 eye;
    //VECTOR3 dir;
    VECTOR3 pos;
    //float32 dist;
    float32 nearest_dist = 1e+6f;
    float32 dir_length;

    /*if(timeDelta == 0)
    {
        //首先计算光轴指向目标的世界矩阵与观察矩阵,并对其顶点进行变换
        GetTrajStatus(cam.m_pTraj, &cam.m_stat,timeDelta,cam.TrajCamNum);//获得当前时刻的相机轨迹状态，相机位置在原点，运动反向向下z轴=1
        GetTrajStatus(pTgt[IRScn.AimTgtNo].m_pTraj, &pTgt[IRScn.AimTgtNo].m_stat,timeDelta,pTgt[IRScn.AimTgtNo].TrajTgtNum);//获得当前时刻的目标轨迹状态

        graphicirrad->Vec3Subtract(&pTgt[IRScn.AimTgtNo].CamTgt, &pTgt[IRScn.AimTgtNo].m_stat.pos , &cam.m_stat.pos);
        angle = atan(pTgt[IRScn.AimTgtNo].CamTgt.y /pTgt[IRScn.AimTgtNo].CamTgt.z);
    }

    WindowX1 = -CamWidLen /2;
    WindowX2 = CamWidLen /2;

    WindowY1 = -(CamHeiLen /2) * sinf(angle);
    WindowY2 = (CamHeiLen /2) * sinf(angle);

    WindowZ1 = -(CamHeiLen /2) * cosf(angle);
    WindowZ2 = (CamHeiLen /2) * cosf(angle);*/

    memset(IRScn.Gray,0,cam.cpara.camDet.wid * cam.cpara.camDet.hei*sizeof(int32)); //清理灰度缓冲为固定值

    StartX = cam.CamProjRect.x1;
    StartY = cam.CamProjRect.y1;
    StartZ = cam.CamProjRect.z0;

    float32 tgtrad = graphicirrad->SearchRadInLookupTable(BbRadiance,TgtTmpr/*pt->tgtTmpr*/,IR_TMPR_MIN,IR_TMPR_MAX,IR_TMPR_DELTA);
    float32 fCamInceptPower_tmp;
    //单位转换要注意，因为用普朗克公式求得的W是瓦/cm2，因此这里要将探测器的单个像元面积由um2转化为cm2，所以乘以1e-8f。
    float32 Vlsb = (cam.cpara.camDet.advol2-cam.cpara.camDet.advol1)/pow(2.0f,cam.cpara.camDet.adres);//放大器电压范围/2e14，就是灰度值每增加1，增加的电压值
    float32 V_tmp = (cam.cpara.camDet.detresp/**1e8f*/) * cam.cpara.camDet.adgain;//探测器响应率 x 放大器增益
    float32 fCamInceptPower, V;

    float32 facetRad = tgtrad  * 0.2f * 0.8f;//* materialF[j].ems//设发射率固定为0.8f

    GetTrajStatus(pTgt[IRScn.AimTgtNo].m_pTraj, &pTgt[IRScn.AimTgtNo].m_stat,timeDelta,pTgt[IRScn.AimTgtNo].TrajTgtNum);//获得当前时刻的目标轨迹状态

    pTgt[IRScn.AimTgtNo].m_stat.elv *= -1.0f;//俯仰角乘以负号，使得俯仰角增大时飞机朝上，在IRSim5中也有类似处理(乘负号)，结果与X3D中的一样,参见DX Doc正负号与纸质笔记

    graphicirrad->SetWorldMatrix(&matWorld,&pTgt[IRScn.AimTgtNo].m_stat);//设置世界变换矩阵，就是从模型空间变换到世界坐标系，平移信息由位置得到，旋转信息由traj轨迹中的三个欧拉角得到

    for (int i = pTgt[IRScn.AimTgtNo].vxNo; i < (pTgt[IRScn.AimTgtNo].vxNum+pTgt[IRScn.AimTgtNo].vxNo); i++ )//对每个顶点从模型空间变换到相机坐标系观察空间
    {
        //fprintf(fpdebug, "matWorldCamera is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",matWorldCamera._11,matWorldCamera._12,matWorldCamera._13,matWorldCamera._14,matWorldCamera._21,matWorldCamera._22,matWorldCamera._23,matWorldCamera._24,matWorldCamera._31,matWorldCamera._32,matWorldCamera._33,matWorldCamera._34,matWorldCamera._41,matWorldCamera._42,matWorldCamera._43,matWorldCamera._44);
        graphicirrad->Vec3MultiplyMatrix(&IRScn.vxLst[i].p, &IRScn.mdl_vxLst[i].p, &matWorld, 1.0f);
        //if (i%100 == 0)
        //    fprintf(fpdebug, "tgt%d: vxLst[%d].p=(%f,%f,%f), mdl_vxLst[%d].p=(%f,%f,%f)\n",IRScn.AimTgtNo,i,IRScn.vxLst[i].p.x,IRScn.vxLst[i].p.y,IRScn.vxLst[i].p.z,i,IRScn.mdl_vxLst[i].p.x,IRScn.mdl_vxLst[i].p.y,IRScn.mdl_vxLst[i].p.z);
    }

    UpdateWrapBoxInfo(&pTgt[IRScn.AimTgtNo], &matWorld);//更新包围盒信息

#if ADD_IR_IMG  //添加实际拍摄的图像
    Rasterize_IRimg(&bkg);
#endif  //ADD_IR_IMG  //添加实际拍摄的图像

    //cam.CamProjRect.z0
    graphicirrad->Vec3SetValue(&eye,0,0,0);
    for(int y=0;y<cam.cpara.camDet.hei;y=y+1)
    {
        StartX = cam.CamProjRect.x1;
        for(int x=0;x<cam.cpara.camDet.wid;x=x+1)
        {
            graphicirrad->Vec3SetValue(&pos,StartX,StartY,StartZ);
            graphicirrad->Vec3Subtract(&dir,&pos,&eye);

            if(intersect_tgtbox(&pTgt[IRScn.AimTgtNo]))
            {
                graphicirrad->Vec3GetLength(&dir_length, &dir);//求出相机目标之间距离，方便算辐射时用

                for(int i = pTgt[IRScn.AimTgtNo].vxNo; i < (pTgt[IRScn.AimTgtNo].vxNum+pTgt[IRScn.AimTgtNo].vxNo)/3.0f; i++)
                {
                    if(intersect_triangle(IRScn.vxLst[3*i].p, IRScn.vxLst[3*i+1].p, IRScn.vxLst[3*i+2].p))
                    {
                        if(nearest_dist > t)
                        {
                            nearest_dist = t;
                        }
                        t = nearest_dist*dir_length;

                        airtrans = exp((-1.0f) * cam.cpara.camOpt.airattn * t);//从本目标到探测器的大气透过率，大气衰减系数*目标到相机的距离

                        //相机像元接收功率公式的固定值
                        fCamInceptPower_tmp = (airtrans * cam.cpara.camOpt.opttrans * cam.cpara.camDet.uthei * cam.cpara.camDet.utwid *1e-8f* pow(cam.cpara.camOpt.apt,2.0f)) / (4.0f * cam.cpara.camOpt.foc * cam.cpara.camOpt.foc);//将乘以verRad_tmp[i]的操作放到循环内进行，这里先算固定值

                        fCamInceptPower = fCamInceptPower_tmp * facetRad;
                        V = fCamInceptPower * V_tmp + cam.cpara.camDet.adbias;
                        IRScn.Gray[y*cam.cpara.camDet.wid + x] =V / Vlsb;
                        if (IRScn.Gray[y*cam.cpara.camDet.wid + x] > (uint32)(pow(2.0f, cam.cpara.camDet.adres)-1.0f))
                        {
                            IRScn.Gray[y*cam.cpara.camDet.wid + x] = (uint32)(pow(2.0f, cam.cpara.camDet.adres)-1.0f);//防止灰度溢出，本来是在IRRadiance里做的，为防止过采样后灰度均值为零放在这里了
                        }
                    }
                }
            }

            /*RayTracking();

            airtrans = exp((-1.0f) * cam.cpara.camOpt.airattn * t);//从本目标到探测器的大气透过率，大气衰减系数*目标到相机的距离

            //相机像元接收功率公式的固定值
            fCamInceptPower_tmp = (airtrans * cam.cpara.camOpt.opttrans * cam.cpara.camDet.uthei * cam.cpara.camDet.utwid *1e-8f* pow(cam.cpara.camOpt.apt,2.0f)) / (4.0f * cam.cpara.camOpt.foc * cam.cpara.camOpt.foc);//将乘以verRad_tmp[i]的操作放到循环内进行，这里先算固定值

            fCamInceptPower = fCamInceptPower_tmp * facetRad;
            V = fCamInceptPower * V_tmp + cam.cpara.camDet.adbias;
            IRScn.Gray[y*cam.cpara.camDet.wid + x] =V / Vlsb;*/

            StartX += cam.CamProjRect.xr;
        }

        StartY += cam.CamProjRect.yr;
    }


}

void IRImg_AR::RayTracking()
{
    float32 nearest_dist = 1e+6f;
    float32 dir_length;
    int32 nearest_face;
    int32 tmp_face;

    graphicirrad->Vec3GetLength(&dir_length, &dir);//求出相机目标之间距离，方便算辐射时用

    for(int i = pTgt[IRScn.AimTgtNo].vxNo; i < (pTgt[IRScn.AimTgtNo].vxNum+pTgt[IRScn.AimTgtNo].vxNo)/3.0f; i++)
    {
        if(intersect_triangle(IRScn.vxLst[3*i].p, IRScn.vxLst[3*i+1].p, IRScn.vxLst[3*i+2].p))
        {
            if(nearest_dist > t)
            {
                nearest_dist = t;
                tmp_face = i;
            }
            nearest_face = tmp_face;
            t = nearest_dist*dir_length;
            cout << "Find a interPoint!" << endl;
            cout << "t= " << t << endl;
        }
    }

}

int IRImg_AR::intersect_tgtbox(IRTARGET * pt)
{
    //if(intersect_triangle(pTgt[IRScn.AimTgtNo].tgtbox.bp[0], pTgt[IRScn.AimTgtNo].tgtbox.bp[2], pTgt[IRScn.AimTgtNo].tgtbox.bp[4]) || \
       intersect_triangle(pTgt[IRScn.AimTgtNo].tgtbox.bp[2], pTgt[IRScn.AimTgtNo].tgtbox.bp[4], pTgt[IRScn.AimTgtNo].tgtbox.bp[6]))

    if(intersect_triangle(pt->tgtbox.bp[0],pt->tgtbox.bp[2],pt->tgtbox.bp[4]) || \
       intersect_triangle(pt->tgtbox.bp[2],pt->tgtbox.bp[4],pt->tgtbox.bp[6]) || \
        intersect_triangle(pt->tgtbox.bp[0],pt->tgtbox.bp[1],pt->tgtbox.bp[2]) || \
        intersect_triangle(pt->tgtbox.bp[1],pt->tgtbox.bp[2],pt->tgtbox.bp[3]) || \
         intersect_triangle(pt->tgtbox.bp[0],pt->tgtbox.bp[1],pt->tgtbox.bp[4]) || \
         intersect_triangle(pt->tgtbox.bp[1],pt->tgtbox.bp[4],pt->tgtbox.bp[5]) || \
            intersect_triangle(pt->tgtbox.bp[2],pt->tgtbox.bp[3],pt->tgtbox.bp[6]) || \
            intersect_triangle(pt->tgtbox.bp[3],pt->tgtbox.bp[6],pt->tgtbox.bp[7]) || \
             intersect_triangle(pt->tgtbox.bp[4],pt->tgtbox.bp[5],pt->tgtbox.bp[6]) || \
             intersect_triangle(pt->tgtbox.bp[5],pt->tgtbox.bp[6],pt->tgtbox.bp[7]) || \
              intersect_triangle(pt->tgtbox.bp[1],pt->tgtbox.bp[3],pt->tgtbox.bp[5]) || \
              intersect_triangle(pt->tgtbox.bp[3],pt->tgtbox.bp[5],pt->tgtbox.bp[7]))
    {
        return 1;
    }
    else
        return 0;
}

int IRImg_AR::intersect_triangle(VECTOR3 vert0, VECTOR3 vert1, VECTOR3 vert2)
{
    VECTOR3 edge1, edge2, tvec, pvec, qvec;
    float32 det, inv_det;
    float32 u; float32 v;

    t = 0.0f;

    //find vectors for two edges sharing vert0
    graphicirrad->Vec3Subtract(&edge1,&vert1,&vert0);
    graphicirrad->Vec3Subtract(&edge2,&vert2,&vert0);

    // begin calculating determinant - also used to calculate U parameter
    graphicirrad->Vec3Cross(&pvec,&dir,&edge2);

    // if determinant is near zero, ray lies in plane of triangle
    graphicirrad->Vec3Dot(&det,&edge1,&pvec);

#ifdef TEST_CULL // define TEST_CULL if culling is desired
    if(det < EPSILON)
        return 0;

    // calculate distance from vert0 to ray origin
    graphicirrad->Vec3Subtract(&tvec, &eye, &vert0);

    // calculate U parameter and test bounds
    graphicirrad->Vec3Dot(&u,&tvec,&pvec);
    if (u < 0.0 || u > det)
        return 0;

    // prepare to test V parameter
    graphicirrad->Vec3Cross(&qvec, &tvec, &edge1);

    // calculate V parameter and test bounds
    graphicirrad->Vec3Dot(&v,&dir, &qvec);
    if (v < 0.0 || (u +v) > det)
        return 0;

    // calculate t, scale parameters, ray intersects triangle
    graphicirrad->Vec3Dot(&t,&edge2, &qvec);

    int_det = 1.0 / det;
    t *= inv_det;
    u *= inv_det;
    v *= inv_det;
#else           // the non-culling branch
    if (det > -EPSILON && det < EPSILON)
        return 0;
    inv_det = 1.0 / det;

    // calculate distance from vert0 to ray origin
    graphicirrad->Vec3Subtract(&tvec, &eye, &vert0);

    // calculate U parameter and test bounds
    graphicirrad->Vec3Dot(&u,&tvec, &pvec);
    u = u * inv_det;

    if (u < 0.0 || u > 1.0)
        return 0;

    // prepare to test V parameters
    graphicirrad->Vec3Cross(&qvec, &tvec, &edge1);

    // calculate V paremeter and test bounds
    graphicirrad->Vec3Dot(&v,&dir,&qvec);
    v = v * inv_det;

    if (v < 0.0 || (u + v) > 1.0)
        return 0;

    //calculate t, ray intersects triangle
    graphicirrad->Vec3Dot(&t,&edge2, &qvec);
    t = t * inv_det;

#endif
    return 1;

}



