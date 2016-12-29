//--------------------------------------------------------------------------------------
//Data:    		20160901
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------
#include "paraReader.h"

paraReader::paraReader()
{
    //参数初始化置0
    IRScn.tgtNum = 0;

    IRScn.vxNum = 0;
    IRScn.ftNum = 0;
    IRScn.mtNum = 0;
    IRScn.pmNum = 0;

    IRScn.ftgInfoLst = NULL;
    IRScn.mdl_vxLst = NULL;
    IRScn.vxLst = NULL;
    IRScn.mtLst = NULL;
    IRScn.pmLst = NULL;

    IRScn.Visible   = NULL;
    IRScn.facetGray = NULL;
    IRScn.Gray   = NULL;
    IRScn.ZDepth = NULL;
    IRScn.Alpha  = NULL;

    graph_irrad = new Graphics_IRrad();
    atmod       = new Atmod();
}

bool paraReader::DefScenePara(const char scncfgfn[])
{
    fpdebug = fopen("../para/fishdebug.txt","w");

    if(!LoadPara(scncfgfn)) return false;//导入光学系统，探测器参数，大气衰减率，读入场景总体目标描述文件，内含目标个数，每个目标的模型与轨迹文件名等

    calcCameraInnerPara();//计算相机的内参数

    if(!LoadCamMovData()) return false; //读入相机运动轨迹

    if(!loadTarget_Mov()) return false; //读入场景总体目标描述文件，内含目标个数，每个目标的模型与轨迹

#if ADD_BKG_FLAG //添加背景，根据光轴指向的目标反向移动背景
    if (!LoadBkg(scncfgfn))return false; //背景要最后导入
#endif//ADD_BKG_FLAG

#if ADD_ATMOD_FLAG
    if (!LoadAtm(scncfgfn))return false;
    if (!atmod->InitAtmod(&atm)) return false;//大气模块初始化
#endif

    //=========================立体视觉======================
    ParamsReader("../parameters.txt" );
    init_runParams(runParams);

    //计算出红外辐射通量密度的查找表
    graph_irrad->CalcBbRadianceLookupTable(BbRadiance,IR_TMPR_MIN,IR_TMPR_MAX,IR_TMPR_DELTA);//根据温度步长计算出一个红外辐射通量密度查找表

    delete graph_irrad;
    return true;
}

//----用于读入一个从开头起offset偏移地址处开始的字符串文件名到指定位置，字符串以分号结束。字符串最大长度小于MAX_FILENAME_CHAR----------
int32 paraReader::copystringfromfile(char * dest, char * src, int32 offset)
{
    int32 i=0;
    while((*(src+offset+i) != ';')&&(i <= MAX_FILENAME_CHAR))
    {
        dest[i] = *(src+offset+i);
        i++;
    }
    dest[i]='\0';
    return i;//返回读入字符的数目
}

//=========================从指定参数文件传入所有参数，包括相机，目标，及各自的轨迹，大气，背景等信息========================================
bool paraReader::LoadPara(const char argv[])
{
    FILE * file;
    char * buf = NULL;
    char * tmp = NULL;
    char * pt = NULL;
    long curpos, length;


    file = fopen(argv, "r");
    if (file == NULL)
    {
        return false;
    }
    curpos = ftell(file);
    fseek(file, 0L, SEEK_END);
    length = ftell(file);
    fseek(file,curpos, SEEK_SET);
    buf = (char *)malloc(sizeof(char) * (length+2));
    if (buf == NULL)
    {
        return false;
    }
    fread(buf, sizeof(char), length, file);
    buf[length+1]='\0';


    tmp = buf;
    pt= strstr(tmp, "airattn:");
    if(pt!= NULL)
    {
        sscanf((pt+8),"%f", &cam.cpara.camOpt.airattn);//读入大气衰减系数
    }
    tmp = buf;
    pt= strstr(tmp, "apt:");
    if(pt!= NULL)
    {
        sscanf((pt+4),"%f", &cam.cpara.camOpt.apt);//读入光学系统孔径(mm)
    }
    tmp = buf;
    pt= strstr(tmp, "foc:");
    if(pt!= NULL)
    {
        sscanf((pt+4),"%f", &cam.cpara.camOpt.foc);//读入光学系统焦距(mm)
    }
    tmp = buf;
    pt= strstr(tmp, "frmfrq:");
    if(pt!= NULL)
    {
        sscanf((pt+7),"%f", &cam.cpara.camOpt.frmfrq);//读入系统frame frequency(fps)
    }
    tmp = buf;
    pt= strstr(tmp, "opttrans:");
    if(pt!= NULL)
    {
        sscanf((pt+9),"%f", &cam.cpara.camOpt.opttrans);//读入光学系统效率
    }
    tmp = buf;
    pt= strstr(tmp, "adbias:");
    if(pt!= NULL)
    {
        sscanf((pt+7),"%f", &cam.cpara.camDet.adbias);//读入电子学系统偏置（V）
    }
    tmp = buf;
    pt= strstr(tmp, "adgain:");
    if(pt!= NULL)
    {
        sscanf((pt+7),"%f", &cam.cpara.camDet.adgain);//读入电子学系统增益
    }
    tmp = buf;
    pt= strstr(tmp, "adres:");
    if(pt!= NULL)
    {
        sscanf((pt+6),"%d", &cam.cpara.camDet.adres);//读入电子学系统精度(bit)
    }
    tmp = buf;
    pt= strstr(tmp, "advol1:");
    if(pt!= NULL)
    {
        sscanf((pt+7),"%f", &cam.cpara.camDet.advol1);//读入电子学系统最低电压(V)
    }
    tmp = buf;
    pt= strstr(tmp, "advol2:");
    if(pt!= NULL)
    {
        sscanf((pt+7),"%f", &cam.cpara.camDet.advol2);//读入电子学系统最高电压(V)
    }
    tmp = buf;
    pt= strstr(tmp, "detresp:");
    if(pt!= NULL)
    {
        sscanf((pt+8),"%f", &cam.cpara.camDet.detresp);//读入探测器响应率(V/W)
    }
    tmp = buf;
    pt= strstr(tmp, "detwl1:");
    if(pt!= NULL)
    {
        sscanf((pt+7),"%f", &cam.cpara.camDet.detwl1);//读入探测器工作波段下限(um)
    }
    tmp = buf;
    pt= strstr(tmp, "detwl2:");
    if(pt!= NULL)
    {
        sscanf((pt+7),"%f", &cam.cpara.camDet.detwl2);//读入探测器工作波段上限(um)
    }
    tmp = buf;
    pt= strstr(tmp, "detwldelta:");
    if(pt!= NULL)
    {
        sscanf((pt+11),"%f", &cam.cpara.camDet.detwldelta);//读入探测器工作波段积分步长(um)
    }
    tmp = buf;
    pt= strstr(tmp, "hei:");
    if(pt!= NULL)
    {
        sscanf((pt+4),"%d", &cam.cpara.camDet.hei);//读入探测器高度方向的像元个数
    }
    tmp = buf;
    pt= strstr(tmp, "wid:");
    if(pt!= NULL)
    {
        sscanf((pt+4),"%d", &cam.cpara.camDet.wid);//读入探测器宽度方向的像元个数
    }
    tmp = buf;
    pt= strstr(tmp, "uthei:");
    if(pt!= NULL)
    {
        sscanf((pt+6),"%d", &cam.cpara.camDet.uthei);//读入探测器像元高度尺寸(um)
    }
    tmp = buf;
    pt= strstr(tmp, "utwid:");
    if(pt!= NULL)
    {
        sscanf((pt+6),"%d", &cam.cpara.camDet.utwid);//读入探测器像元宽度尺寸(um)
    }

    //=================读入场景中的目标个数,并分配内存=====================================
    tmp = buf;
    pt= strstr(tmp, "tgtnum:");//比较文件中与tgtnum:相同的字符，得到这个字符的起始指针位置
    if(pt!= NULL)
    {
        sscanf((pt+7),"%d", &IRScn.tgtNum);//读入场景中的目标个数
    }

    pTgt = (IRTARGET *)malloc(sizeof(IRTARGET) * IRScn.tgtNum);//分配内存给目标,每个目标都按结构体大小分配内存
    if (pTgt == NULL)
    {
        return false;
    }

    memset(pTgt, 0,sizeof(IRTARGET) * IRScn.tgtNum);//初始化所有目标分配的内存为0

    //=================光轴瞄准第几个目标=====================================
    tmp = buf;
    pt = strstr(tmp, "aimat:");
    if (pt != NULL)
    {
        sscanf((pt+6),"%d",&IRScn.AimTgtNo);//光轴瞄准第几个目标
    }
    if (IRScn.AimTgtNo >= IRScn.tgtNum)//光轴瞄准的目标编号不能超出目标个数
    {
        printf("error: aim_at_target_No exceeds the total target number.\n ");
    }

    //===================读入每个目标的模型文件和轨迹文件=======================================
    int32 i = 0;//当前目标序号
    tmp = buf;
    while (pt != NULL)
    {
        //读入模型文件名称
        pt = strstr(tmp, "tgtmdlno");
        if (pt != NULL)
        {
            sscanf((pt+8),"%d",&i);
            tmp = pt + 8;
            pt = strstr(tmp,":");
            if(pt != NULL)
            {
                copystringfromfile(pTgt[i].m_model, pt, 1);
            }
        }

        //读入轨迹文件名称
        pt = strstr(tmp, "tgtmovno");
        if (pt != NULL)
        {
            sscanf((pt+8),"%d",&i);
            tmp = pt + 8;
            pt = strstr(tmp,":");
            if(pt != NULL)
            {
                copystringfromfile(pTgt[i].m_mov, pt, 1);

            }
        }
    }

    //===================相机的轨迹文件名称=======================================
    tmp = buf;
    pt = strstr(tmp, "cammov:");
    if (pt != NULL)
    {
        copystringfromfile(cam.cammovfilename, pt, 7);
    }

    //===================读入目标缺省温度(K)=======================================
    tmp = buf;
    pt= strstr(tmp, "tgttmpr:");
    if(pt!= NULL)
    {
        sscanf((pt+8),"%f", &TgtTmpr);//读入目标缺省温度(K)
    }

    tmp = buf;
    pt= strstr(tmp, "stemissivity:");
    if(pt!= NULL)
    {
        sscanf((pt+13),"%f", &stemi);//读入目标发射率
    }

    tmp = buf;
    pt= strstr(tmp, "sceneSNR:");
    if(pt!= NULL)
    {
        sscanf((pt+9),"%f", &scnSNR);//读入目标发射率
    }

    //===================读入待保存的dv2文件路径=======================================
    tmp = buf;
    pt= strstr(tmp, "dvfilename:");
    if(pt!= NULL)
    {
        copystringfromfile(DVFileName, pt, 11);
    }

    //==============关闭文件，释放内存============================================
    free(buf);
    buf = NULL;
    fclose(file);
    file = NULL;

    return true;
}

//=========================计算相机的内参数==========================================
bool paraReader::calcCameraInnerPara(void)
{
    //生成场景计算时不变的常量，这里是内参数矩阵的生成，主要用于投影矩阵的构建
    CamWidLen = cam.cpara.camDet.wid * cam.cpara.camDet.utwid * 1e-6f;//计算探测器像素宽度 x 每个像元的宽度=探测器总的宽度 单位m
    CamHeiLen = cam.cpara.camDet.hei * cam.cpara.camDet.uthei * 1e-6f;//计算探测器像素高度 x 每个像元的高度=探测器总的高度 单位m

    cam.CamProjRect.wid = cam.cpara.camDet.wid;//相机的原始投影平面宽度像素个数
    cam.CamProjRect.hei = cam.cpara.camDet.hei;//相机的原始投影平面高度像素个数
    cam.CamProjRect.x1 = -0.5f * CamWidLen;//x1y1最小值，x2y2最大值  这样就是设相机原始投影平面的原点在中间
    cam.CamProjRect.x2 = 0.5f * CamWidLen;//x1y1最小值，x2y2最大值
    cam.CamProjRect.y1 = -0.5f * CamHeiLen;//x1y1最小值，x2y2最大值
    cam.CamProjRect.y2 = 0.5f * CamHeiLen;//x1y1最小值，x2y2最大值
    cam.CamProjRect.z0 = cam.cpara.camOpt.foc * 1e-3f;//X3D中用的是负号，我没用，//焦距(米)乘以－1，

    cam.CamProjRect.xr = cam.cpara.camDet.utwid * 1e-6f;//相机原始投影平面上每像素代表的实际长度//(CamProjRect.x2 - CamProjRect.x1) / CamProjRect.wid;
    cam.CamProjRect.yr = cam.cpara.camDet.uthei * 1e-6f;//相机原始投影平面上每像素代表的实际长度//(CamProjRect.y2 - CamProjRect.y1) / CamProjRect.hei;

    dx = cam.CamProjRect.z0;//若令dx=焦距，则裁剪平面变为实际长宽度的一半
    dy = -cam.CamProjRect.z0;//因为光栅化时可能将y弄反了，所以这里的投影因子直接变为负号就可以了。

    return true;
}

//=================从文件中导入轨迹，目标和相机==============================
bool paraReader::LoadTrajFromFile(int32 argc,char argv[],int32 tgtno)
{//Desc: to load traj from *.traj files
    /*int32 num;

    FILE* tjFp = fopen(argv,"rb");
    if (tjFp == NULL)
    {
        printf("traj file is null\n");
        return false;
    }
    fread(&num, sizeof(int32), 1, tjFp);//读入步数


    if (num > MAX_TRAJ_NUM)
    {
        printf("warning: traj num in the file is larger than maximum traj num, only the first %d traj would be read.\n",MAX_TRAJ_NUM);
    }*/
    if(!ReadTrajconfig(argv)) return false;

    TrajStore();

    switch(argc)
    {
    case SCN_CAM_TRAJQRT:
        {
            cam.TrajCamNum = TrajNum;
            cam.m_pTraj = (TRAJECTORY *)malloc(sizeof(TRAJECTORY)*cam.TrajCamNum);//分配轨迹内存
            if (cam.m_pTraj == NULL)
            {
                printf("there isn't enough memory to allocate camera traj.\n");
            }
            cam.m_pTraj = m_pTraj;
            cam.TrajCamStoptime = cam.m_pTraj[cam.TrajCamNum-1].currenttime;//保存相机轨迹的结束时间
            /*cam.TrajCamNum = minimum(num,MAX_TRAJ_NUM);//读入目标轨迹步数
            cam.m_pTraj = (TRAJECTORY *)malloc(sizeof(TRAJECTORY)*cam.TrajCamNum);//分配轨迹内存
            if (cam.m_pTraj == NULL)
            {
                printf("there isn't enough memory to allocate camera traj.\n");
            }
            fread(&cam.m_pTraj[0],sizeof(TRAJECTORY),cam.TrajCamNum,tjFp);	//读入指定步数的轨迹

            cam.TrajCamStoptime = cam.m_pTraj[cam.TrajCamNum-1].currenttime;//保存相机轨迹的结束时间
            */
        }
        break;
    case SCN_TGT_TRAJQRT:
        {
            /*pTgt[tgtno].TrajTgtNum =minimum(num,MAX_TRAJ_NUM);
            pTgt[tgtno].m_pTraj = (TRAJECTORY *)malloc(sizeof(TRAJECTORY)*pTgt[tgtno].TrajTgtNum);
            if (pTgt[tgtno].m_pTraj == NULL)
            {
                printf("there isn't enough memory to allocate target%d traj.\n", tgtno);
            }
            fread(&pTgt[tgtno].m_pTraj[0],sizeof(TRAJECTORY),pTgt[tgtno].TrajTgtNum,tjFp);	//读入指定步数的轨迹

            //std::cout<< pTgt[tgtno].m_pTraj[1].xpos<<std::endl;

            pTgt[tgtno].TrajTgtStoptime = pTgt[tgtno].m_pTraj[pTgt[tgtno].TrajTgtNum-1].currenttime;//保存当前目标轨迹最后一步的的时间即仿真结束时间
            */
            pTgt[tgtno].TrajTgtNum = TrajNum;
            pTgt[tgtno].m_pTraj = (TRAJECTORY *)malloc(sizeof(TRAJECTORY)*pTgt[tgtno].TrajTgtNum);
            if (pTgt[tgtno].m_pTraj == NULL)
            {
                printf("there isn't enough memory to allocate target%d traj.\n", tgtno);
            }
            pTgt[tgtno].m_pTraj = m_pTraj;
            pTgt[tgtno].TrajTgtStoptime = pTgt[tgtno].m_pTraj[pTgt[tgtno].TrajTgtNum-1].currenttime;//保存当前目标轨迹最后一步的的时间即仿真结束时间
        }
        break;
    }

    //fclose(tjFp);

    return true;

}

//======================读入相机运动轨迹==================================================
bool paraReader::LoadCamMovData(void)
{    
    //读入普通Traj格式轨迹
    //读入相机轨迹
    if (!LoadTrajFromFile(SCN_CAM_TRAJQRT,cam.cammovfilename, 0))
    {
        printf("read camera traj false\n");
        return false;
    }

    return true;
}

//--------------读入每个目标的模型文件-------------------------------------------------
bool paraReader::LoadMDLFromFile(char scncfgfn[], int32 tgtno)
{

    FILE * file;
    int32 tvxNo,  tmtNo;//tobjNo,tftNo,, tpmNo

    file = fopen(scncfgfn, "rb");
    if (file == NULL)
    {
        printf("no%d target model file is null\n",tgtno);
        return false;
    }

    IRScn.ftgInfoLst = (FTG_INFO *)malloc(sizeof(FTG_INFO) * IRScn.tgtNum);//为每个目标的头信息分配内存
    if (IRScn.ftgInfoLst == NULL)
    {
        return false;
    }

    fread (&IRScn.ftgInfoLst[tgtno], sizeof(FTG_INFO),1,file);//读入模型文件头

    pTgt[tgtno].tgtNo = tgtno;
    //=======目标模型中顶点，面元，材料，尾焰的数目==========
    pTgt[tgtno].vxNum = IRScn.ftgInfoLst[tgtno].vxNum;
    pTgt[tgtno].mtNum = IRScn.ftgInfoLst[tgtno].mtNum;
    //=======目标在场景中对象，顶点，面元，材料，尾焰的起始值=======
    for(int i=0;i<=tgtno;i++)
    {
        pTgt[tgtno].vxNo = IRScn.ftNum*3;
        pTgt[tgtno].mtNo = IRScn.mtNum;

        IRScn.vxNum += IRScn.ftgInfoLst[tgtno].vxNum;
        IRScn.ftNum += IRScn.ftgInfoLst[i].ftNum;
        IRScn.mtNum += IRScn.ftgInfoLst[i].mtNum;
    }


    tvxNo = pTgt[tgtno].vxNo;
    tmtNo = pTgt[tgtno].mtNo;


    if (! MallocBufferForTgt())//根据文件头分配空间
    {
        printf("\nunable to malloc memory for target\n");
    }

    //==============读出目标的顶点列表信息===========================
    fread (&IRScn.mdl_vxLst[tvxNo], sizeof(VIRTEXF), pTgt[tgtno].vxNum, file);

    //==============读出目标的材料列表信息===========================
    fread (&IRScn.mtLst[tmtNo], sizeof(MATERIALF), pTgt[tgtno].mtNum, file);

    //========求出当前目标的包围盒(蒙皮的)==========================
    pTgt[tgtno].tgtbox.xmin = pTgt[tgtno].tgtbox.xmax = IRScn.mdl_vxLst[0].p.x;//设xmin,xmax初始值
    pTgt[tgtno].tgtbox.ymin = pTgt[tgtno].tgtbox.ymax = IRScn.mdl_vxLst[0].p.y;//设ymin,ymax初始值
    pTgt[tgtno].tgtbox.zmin = pTgt[tgtno].tgtbox.zmax = IRScn.mdl_vxLst[0].p.z;//设zmin,zmax初始值
    //=============找出包围盒的xmin,ymin,zmin,xmax,ymax,zmax的值,可得到包围盒的大小=================
    for (int i = tvxNo; i< tvxNo+pTgt[tgtno].vxNum; i++)
    {
        if (IRScn.mdl_vxLst[i].p.x < pTgt[tgtno].tgtbox.xmin)
        {
            pTgt[tgtno].tgtbox.xmin = IRScn.mdl_vxLst[i].p.x;
        }
        if (IRScn.mdl_vxLst[i].p.x > pTgt[tgtno].tgtbox.xmax)
        {
            pTgt[tgtno].tgtbox.xmax = IRScn.mdl_vxLst[i].p.x;
        }
        if (IRScn.mdl_vxLst[i].p.y < pTgt[tgtno].tgtbox.ymin)
        {
            pTgt[tgtno].tgtbox.ymin = IRScn.mdl_vxLst[i].p.y;
        }
        if (IRScn.mdl_vxLst[i].p.y >  pTgt[tgtno].tgtbox.ymax )
        {
            pTgt[tgtno].tgtbox.ymax  = IRScn.mdl_vxLst[i].p.y;
        }
        if (IRScn.mdl_vxLst[i].p.z < pTgt[tgtno].tgtbox.zmin)
        {
            pTgt[tgtno].tgtbox.zmin = IRScn.mdl_vxLst[i].p.z;
        }
        if (IRScn.mdl_vxLst[i].p.z >  pTgt[tgtno].tgtbox.zmax)
        {
            pTgt[tgtno].tgtbox.zmax = IRScn.mdl_vxLst[i].p.z;
        }

        //IRScn.mdl_vxLst[i].p.z *= -1.0f;//右手坐标系换到左手 ,20090310
    }
    //===========打印出包围盒的x,y,z最大值最小值=======================
    //printf("xmin is %f, xmax is %f, ymin is %f, ymax is %f, zmin is %f, zmax is %f.\n",\
    pTgt[tgtno].tgtbox.xmin, pTgt[tgtno].tgtbox.xmax, pTgt[tgtno].tgtbox.ymin, pTgt[tgtno].tgtbox.ymax, \
    pTgt[tgtno].tgtbox.zmin,pTgt[tgtno].tgtbox.zmax);

    return true;
}

//------------根据文件头为目标模型分配空间--------------------------------------------------
bool paraReader::MallocBufferForTgt()
{
    //==============为所有目标的模型空间中顶点列表分配内存===========================
    IRScn.mdl_vxLst = (VIRTEXF * ) realloc(IRScn.mdl_vxLst, IRScn.vxNum * sizeof(VIRTEXF));//realloc这里的vxNum是随着目标增加变化的
    if (IRScn.mdl_vxLst == NULL)
    {
        return false;
    }

    //==============为目标模型中所有顶点与其他矩阵进行转换计算后的目标顶点值存放列表===========================
    IRScn.vxLst = (VIRTEXF * ) realloc(IRScn.vxLst, IRScn.vxNum * sizeof(VIRTEXF));//这里的vxNum是随着目标增加变化的
    if (IRScn.vxLst == NULL)
    {
        return false;
    }

    //==============为目标模型中所有材料分配内存===========================每个材料对应一个对象
    IRScn.mtLst = (MATERIALF * ) realloc(IRScn.mtLst, IRScn.ftNum * sizeof(MATERIALF));//本句代码是与PC机代码不同的，PC机代码以后也要修改成这样，而且目标模型格式也需要更改，将材质由面元个数个转为对象个数个！
    if (IRScn.mtLst == NULL)
    {
        return false;
    }

    //==============为目标模型中所有尾焰和尾焰粒子分配内存===========================
    if(IRScn.pmNum != 0)
    {
        IRScn.pmLst = (PLUMEF * ) realloc(IRScn.pmLst, IRScn.pmNum * sizeof(PLUMEF));
        if (IRScn.pmLst == NULL)
        {
            return false;
        }
        /*IRScn.plmparLst = (PLUMEPAR *) realloc(IRScn.plmparLst, IRScn.pmNum * PLUME_PARNUM * sizeof(PLUMEPAR));
        if (IRScn.plmparLst == NULL)
        {
            return false;
        }*/

    }

    //==============为目标模型中所有面元的可见与否分配内存===========================
    IRScn.Visible = (int32 *) realloc(IRScn.Visible, IRScn.ftNum * sizeof (int32));
    if (IRScn.Visible == NULL)
    {
        return false;
    }

    //==============为目标模型中所有面元的灰度值分配内存===========================
    IRScn.facetGray = (float32 *) realloc(IRScn.facetGray, IRScn.ftNum * sizeof (float32));
    if (IRScn.facetGray == NULL)
    {
        return false;
    }

    //==============为为图像的灰度缓存和深度缓存分配内存===========================
    IRScn.Gray = (uint32 *)malloc(sizeof(uint32)*cam.cpara.camDet.hei*cam.cpara.camDet.wid);//为图像的灰度缓存分配内存
    IRScn.ZDepth = (float32 *)malloc(sizeof(float32)*cam.cpara.camDet.hei*cam.cpara.camDet.wid);//为图像的深度缓存分配内存
    if (IRScn.Gray == NULL || IRScn.ZDepth == NULL)
    {
        return false;
    }

    whitenoise = (int32 *)malloc(sizeof(int32)*cam.cpara.camDet.hei*cam.cpara.camDet.wid);//分配图像大小的白噪声内存
    if (whitenoise == NULL)
    {
        return false;
    }//加白噪声

    return true;
}

//================导入每个目标的模型和轨迹==========================================
bool paraReader::loadTarget_Mov(void)
{
    for (int i = 0; i < IRScn.tgtNum; i++)
    {

        if (!LoadTrajFromFile(SCN_TGT_TRAJQRT,pTgt[i].m_mov,i))//从文件中导入轨迹
        {
            printf("error reading traj of tgt %d.\n",i);
            return false;
        }


        if (!LoadMDLFromFile(pTgt[i].m_model,i))//读入每个目标的模型文件
        {
            printf("error reading model of tgt %d.\n",i);
            return false;
        }

    }

    return true;
}

//================导入目标或相机的初始轨迹========================================
bool paraReader::ReadTrajconfig(const char argv[]) //读入轨迹初始数据和运动数据
{
    FILE * file;
    char * buf = NULL;
    char * tmp = NULL;
    char * pt = NULL;
    //int32 i = 0;//当前目标序号
    //int32 j = 0;//目标文件名字数
    //int aim;
    long curpos, length;


    //=================读入文件内容=====================================
    file = fopen(argv, "r");//打开文件，就是总的配置文件
    if (file == NULL)
    {
        cout << "Cant find the file!" << endl;
        return false;
    }
    curpos = ftell(file);
    fseek(file, 0L, SEEK_END);
    length = ftell(file);//得到文件的长度
    fseek(file,curpos, SEEK_SET);
    buf = (char *)malloc(sizeof(char) * (length+2));//为文件内容分配内存
    if (buf == NULL)
    {
        return false;
    }
    fread(buf, sizeof(char), length, file);//把文件整体读入buf中
    buf[length+1]='\0';//buf的最后一位设为0

    tmp = buf;

    pt= strstr(tmp, "TrajNum:");//比较文件中与tgtnum:相同的字符，得到这个字符的起始指针位置
    if(pt!= NULL)
    {
        sscanf((pt+8),"%d", &TrajNum);//读入目标或相机的轨迹步数
    }

    pt= strstr(tmp, "xpos:");
    if(pt!= NULL)
    {
        sscanf((pt+5),"%f", &m_pTraj_tmp.xpos);//读入目标或相机的初始x坐标
    }

    pt= strstr(tmp, "ypos:");
    if(pt!= NULL)
    {
        sscanf((pt+5),"%f", &m_pTraj_tmp.ypos);//读入目标或相机的初始y坐标
    }

    pt= strstr(tmp, "zpos:");
    if(pt!= NULL)
    {
        sscanf((pt+5),"%f", &m_pTraj_tmp.zpos);//读入目标或相机的初始z坐标
    }

    pt= strstr(tmp, "velocity:");
    if(pt!= NULL)
    {
        sscanf((pt+9),"%f", &m_pTraj_tmp.velocity);//读入目标或相机的初始速度
    }

    pt= strstr(tmp, "acceleration:");
    if(pt!= NULL)
    {
        sscanf((pt+13),"%f", &m_pTraj_tmp.acceleration);//读入目标或相机的初始加速度
    }

    pt= strstr(tmp, "azimation:");
    if(pt!= NULL)
    {
        sscanf((pt+10),"%f", &m_pTraj_tmp.azimation);//读入目标或相机的初始偏航角
    }

    pt= strstr(tmp, "elevation:");
    if(pt!= NULL)
    {
        sscanf((pt+10),"%f", &m_pTraj_tmp.elevation);//读入目标或相机的初始俯仰角
    }

    pt= strstr(tmp, "banking:");
    if(pt!= NULL)
    {
        sscanf((pt+8),"%f", &m_pTraj_tmp.banking);//读入目标或相机的初始横滚角
    }

    pt= strstr(tmp, "steerspd:");
    if(pt!= NULL)
    {
        sscanf((pt+9),"%f", &m_pTraj_tmp.steerspd);//读入目标或相机的转向,偏航角的变化
    }

    pt= strstr(tmp, "raisespd:");
    if(pt!= NULL)
    {
        sscanf((pt+9),"%f", &m_pTraj_tmp.raisespd);//读入目标或相机的爬升，俯仰角的变化
    }

    pt= strstr(tmp, "rollspd:");
    if(pt!= NULL)
    {
        sscanf((pt+8),"%f", &m_pTraj_tmp.rollspd);//读入目标或相机的倾斜，横滚角的变化
    }

    pt= strstr(tmp, "currenttime:");
    if(pt!= NULL)
    {
        sscanf((pt+12),"%f", &m_pTraj_tmp.currenttime);//读入目标或相机的初始时间
    }

    pt= strstr(tmp, "interval:");
    if(pt!= NULL)
    {
        sscanf((pt+9),"%f", &m_pTraj_tmp.interval);//读入目标或相机的步数之间的时间间隔
    }

    pt= strstr(tmp, "stoptime:");
    if(pt!= NULL)
    {
        sscanf((pt+9),"%f", &TrajStopTime);//读入目标或相机的步数之间的时间间隔
    }

    fclose(file);
    file = NULL;
    free(buf);
    buf = NULL;

    return true;

}

bool paraReader::TrajStateStore(void)
{
    int i;
    float32 timeDelta=0;
    FILE* tjFp = fopen("../../para/Traj_state.tarj","wb");
    if (tjFp == NULL)
    {
        printf("traj file is Null!\n");
        return false;
    }

    TrajNum = (TrajStopTime - m_pTraj_tmp.currenttime)/m_pTraj_tmp.interval + 1;
    if (TrajNum > MAX_TRAJ_NUM)
    {
        TrajNum = MAX_TRAJ_NUM;
    }
    fwrite(&TrajNum, sizeof(int32), 1, tjFp);//写入步数

    m_pTraj = (TRAJECTORY *)malloc(sizeof(TRAJECTORY)*TrajNum);//分配轨迹内存
    if (m_pTraj == NULL)
    {
        printf("there isn't enough memory to allocate camera traj.\n");
    }

    m_pTraj[0] = m_pTraj_tmp;

    i=0;
    while(m_pTraj[i].currenttime<TrajStopTime+1)
    {
        i++;
        m_pTraj[i] = m_pTraj[i-1];
        m_pTraj[i].azimation = m_pTraj[i-1].azimation + m_pTraj_tmp.interval*m_pTraj[i-1].steerspd;
        m_pTraj[i].currenttime = m_pTraj[i-1].currenttime + m_pTraj_tmp.interval;
    }

    //fwrite(&m_pTraj[0],sizeof(TRAJECTORY),TrajNum,tjFp);	//读入指定步数的轨迹

    /*while (timeDelta <= TrajStopTime)
    {
        GetTrajStatus(m_pTraj, &m_stat, timeDelta, TrajNum);
        fwrite(&m_stat,sizeof(XSTATUS),1,tjFp);
        timeDelta += 10/10;
    }*/

    fclose(tjFp);
    tjFp = NULL;
    free(m_pTraj);
    m_pTraj = NULL;

    return true;
}

bool paraReader::TrajStore(void)
{
    int i;
    /*FILE* tjFp = fopen("../../para/Traj_new.traj","wb");
    if (tjFp == NULL)
    {
        printf("traj file is Null!\n");
        return false;
    }*/

    TrajNum = (TrajStopTime - m_pTraj_tmp.currenttime)/m_pTraj_tmp.interval + 1;
    if (TrajNum > MAX_TRAJ_NUM)
    {
        TrajNum = MAX_TRAJ_NUM;
    }
    //fwrite(&TrajNum, sizeof(int32), 1, tjFp);//写入步数

    m_pTraj = (TRAJECTORY *)malloc(sizeof(TRAJECTORY)*TrajNum);//分配轨迹内存
    if (m_pTraj == NULL)
    {
        printf("there isn't enough memory to allocate camera traj.\n");
    }

    m_pTraj[0] = m_pTraj_tmp;

    i = 0;
    while(m_pTraj[i].currenttime<TrajStopTime)
    {
        i++;
        m_pTraj[i] = m_pTraj[i-1];
        //m_pTraj[i].xpos = m_pTraj[i-1].xpos + m_pTraj_tmp.interval*1;
        //m_pTraj[i].ypos = m_pTraj[i-1].ypos + m_pTraj_tmp.interval*1;
        //m_pTraj[i].zpos = m_pTraj[i-1].zpos + m_pTraj_tmp.interval*1;
        //m_pTraj[i].elevation = m_pTraj[i-1].elevation + m_pTraj_tmp.interval*m_pTraj[i-1].raisespd;
        //m_pTraj[i].azimation = m_pTraj[i-1].azimation + m_pTraj_tmp.interval*m_pTraj[i-1].steerspd;
        //m_pTraj[i].banking = m_pTraj[i-1].banking + m_pTraj_tmp.interval*m_pTraj[i-1].rollspd;
        m_pTraj[i].currenttime = m_pTraj[i-1].currenttime + m_pTraj_tmp.interval;
    }

    //fwrite(&m_pTraj[0],sizeof(TRAJECTORY),TrajNum,tjFp);	//读入指定步数的轨迹

    //fclose(tjFp);
    //tjFp = NULL;
    //free(m_pTraj);
    //m_pTraj = NULL;

    return true;
}

//=====================读入背景数据====================================
bool paraReader::LoadBkg(const char scncfgfn[])
{
    FILE * file;
    char * buf = NULL;
    char * tmp = NULL;
    char * pt = NULL;
    int32 i = 0;//当前目标序号
    int32 j = 0;//目标文件名字数

    long curpos, length;

    file = fopen(scncfgfn, "r");
    if (file == NULL)
    {
        return false;
    }
    curpos = ftell(file);
    fseek(file, 0L, SEEK_END);
    length = ftell(file);
    fseek(file,curpos, SEEK_SET);
    buf = (char *)malloc(sizeof(char) * (length+2));
    if (buf == NULL)
    {
        return false;
    }
    fread(buf, sizeof(char), length, file);
    buf[length+1]='\0';


    tmp = buf;
    pt= strstr(tmp, "bkgfilename:");
    if(pt!= NULL)
    {
        copystringfromfile(bkg.filename, pt, 12);//背景数据指定文件pbkg->filenamelen=
    }

    pt=strstr(tmp,"bkgframeno:");
    if (pt != NULL)
    {
        sscanf((pt+11),"%d",&bkg.frameno);//背景数据是指定文件的第几帧
    }

    pt=strstr(tmp,"bkgbtmpr:");
    if (pt != NULL)
    {
        sscanf((pt+9),"%f",&bkg.btmpr);//背景图像中的平滑部分的平均温度(如天空）
    }
    pt=strstr(tmp,"bkgbems:");
    if (pt != NULL)
    {
        sscanf((pt+8),"%f",&bkg.bems);//背景图像中的平滑部分的发射率？(如天空）
    }

    pt=strstr(tmp,"bkgttmpr:");
    if (pt != NULL)
    {
        sscanf((pt+9),"%f",&bkg.ttmpr);//背景图像中的扰动部分的平均温度（如云）
    }
    pt=strstr(tmp,"bkgtems:");
    if (pt != NULL)
    {
        sscanf((pt+8),"%f",&bkg.tems);//背景图像中的扰动部分的发射率（如云）
    }
    pt=strstr(tmp,"bkgairtrans:");
    if (pt != NULL)
    {
        sscanf((pt+12),"%f",&bkg.airtrans);//从背景到相机的大气平均透过率
    }
    pt=strstr(tmp,"bkggrayres:");
    if (pt != NULL)
    {
        sscanf((pt+11),"%d",&bkg.grayres);//背景文件中的灰度精度(bit)
    }

    fclose(file);
    file = NULL;
    free(buf);
    buf = NULL;

    if(!ReadBkgFromFile(&bkg)) return false;

    return true;
}

//--------------------------------------------------------------------------------------
bool paraReader::ReadBkgFromFile(IRBKG * pbkg)//从指定DV2文件的指定frame中读取背景数据
{

    FILE* bkgfile;
    IRDVFILEHEADER bkgdvInfo;

    bkgfile = fopen(pbkg->filename,"rb");
    if(bkgfile== NULL)
    {
        return false;
    }

    fread(&bkgdvInfo, sizeof(IRDVFILEHEADER),1,bkgfile);//读一次背景文件头信息

    if (bkgdvInfo.length == 0)
    {
        printf("dv length is zero\n");
        return false;
    }

    if (pbkg->frameno >= bkgdvInfo.length)//length是图像的帧数
    {
        printf("bkg frameno > bkg dvfile length, reset frameno to the last frame in dv file.\n");
        pbkg->frameno = bkgdvInfo.length -1;
    }

    pbkg->bkgBuf = (unsigned short *)malloc(bkgdvInfo.height*bkgdvInfo.width*sizeof(unsigned short));//设置红外图像的缓存空间
    if (pbkg->bkgBuf == NULL )
    {
        return false;
    }

    int32 rsize;
    rsize = bkgdvInfo.height*bkgdvInfo.width;//红外图像的大小

    fseek(bkgfile,sizeof(IRDVFILEHEADER)+pbkg->frameno*bkgdvInfo.imgSize,SEEK_SET);//文件头信息后还有1941504个字节,不知道是什么鬼,
    fread(pbkg->bkgBuf,sizeof(short),rsize,bkgfile);//读入1264x768个字节图像。


    pbkg->wid = bkgdvInfo.width;//1264
    pbkg->hei = bkgdvInfo.height;//768

    pbkg->widstart = pbkg->wid/2-cam.cpara.camDet.wid/2;//左上角376x128，基本就是图像对称的中央区域
    pbkg->heistart = pbkg->hei/2-cam.cpara.camDet.hei/2;//暂时先设背景的初始选定区域位于背景的正中心

    fclose(bkgfile);
    bkgfile = NULL;

    return true;

}

//======================读入大气模块相关配置============================================
bool paraReader::LoadAtm(char scncfgfn[])
{
    FILE * file;
    char * buf = NULL;
    char * tmp = NULL;
    char * pt = NULL;

    long curpos, length;

    file = fopen(scncfgfn, "r");//argv
    if (file == NULL)
    {
        return false;
    }
    curpos = ftell(file);
    fseek(file, 0L, SEEK_END);
    length = ftell(file);
    fseek(file,curpos, SEEK_SET);
    buf = (char *)malloc(sizeof(char) * (length+2));
    if (buf == NULL)
    {
        return false;
    }
    fread(buf, sizeof(char), length, file);
    buf[length+1]='\0';


    tmp = buf;
    pt= strstr(tmp, "atmoddir:");
    if(pt!= NULL)
    {
        /*atm.AtmodDir=*/copystringfromfile(atm.AtmodDir, pt, 9);
    }

    fclose(file);
    file = NULL;
    free(buf);
    buf = NULL;


    return true;
}

void paraReader::ParamsReader( string filename="../parameters.txt" )
{
    ifstream fin( filename.c_str() ); //打开这个文件
    if (!fin)
    {
        cerr<<"parameter file does not exist."<<endl;
        return;
    }
    while(!fin.eof()) //判断是不是文件结尾
    {
        string str;
        getline( fin, str );
        if (str[0] == '#')
        {
            // 以‘＃’开头的是注释
            continue;
        }

        int pos = str.find("=");
        if (pos == -1)
            continue;
        string key = str.substr( 0, pos );
        string value = str.substr( pos+1, str.length() );
        data[key] = value;

        if ( !fin.good() )
            break;
    }
}
string paraReader::getData( string key )
{
    map<string, string>::iterator iter = data.find(key);
    if (iter == data.end())
    {
        cerr<<"Parameter name "<<key<<" not found!"<<endl;
        return string("NOT_FOUND");
    }
    return iter->second;
}

void paraReader::init_runParams(RunParams& runParams)
{
    //程序运行参数calib，rectify，reconstruct，process
    runParams.processType = getData("processType");
    cout << runParams.processType <<endl;

    //===================立体标定参数=====================================
    //图像序列列表#calib_image/imagelist.xml
    runParams.calib_imagepath = getData("imagepath");

    //立体标定参数保存文件目录
    runParams.stereocalib_path =  getData("stereocalib_path");

    //棋盘格长宽大小参数
    runParams.cornerX = atoi( getData("cornerX").c_str());
    runParams.cornerY = atoi( getData("cornerY").c_str());
    runParams.squareSize = atof( getData("squareSize").c_str());
    //====================================================================

    //=============立体校正参数=============================================
    //校正参数计算后测试图像
    runParams.image1_test =  getData("image1_test");
    runParams.image2_test =  getData("image2_test");

    //立体校正参数 RECTIFY_HARTLEY，RECTIFY_BOUGUET
    runParams.rectifymethod =  getData("rectifymethod");

    runParams.rectifyParams_path =  getData("rectifyParams_path");
   //=============立体校正参数============================================

   //================立体重建参数=========================================
   //起始与终止索引
   runParams.start_index  =   atoi(  getData( "start_index" ).c_str() );
   runParams.end_index =   atoi(  getData( "end_index"   ).c_str() );

   //数据所在目录，及图像名称前缀和后缀
   runParams.img_left= getData( "img_left" );
   runParams.left_extension= getData( "left_extension" );
   runParams.img_right= getData( "img_right" );
   runParams.right_extension= getData( "right_extension" );

   //视差计算算法  BM , SGBM, VAR, ELAS
   runParams.DisparityType =  getData( "DisparityType" );
   //================立体重建参数=========================================

   //================数据源==============================================
   //三种数据源：image，camera，video
    runParams.file_type =  getData( "file_type" );
    cout << runParams.file_type <<endl;

    runParams.image1_test =  getData("image1_dir");
    runParams.image2_test =  getData("image2_dir");

    runParams.camera1 = atoi( getData("camera1").c_str());
    runParams.camera2 = atoi( getData("camera2").c_str());

    runParams.video1_dir =  getData("video1_dir");
    runParams.video2_dir =  getData("video2_dir");

    runParams.detector =  getData( "detector" );
    runParams.descriptor =  getData( "descriptor" );
    runParams.good_match_threshold = atof(  getData( "good_match_threshold" ).c_str() );

    if(runParams.DisparityType == "SSCA")
    {
        ccName = getData( "ccName" );
        caName = getData( "caName" );
        ppName = getData( "ppName" );
        costAlpha = atof(getData( "costAlpha" ).c_str()); //0.3 for middllebury or 1.0 for kitti
        maxDis = atoi(getData( "maxDis" ).c_str()); //`60` for Middlebury and `256` for KITTI dataets.
        disSc = atoi(getData( "disSc" ).c_str());     //`4` for Middlebury and `1` for KITTI datasets
    }

}









