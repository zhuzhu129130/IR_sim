//--------------------------------------------------------------------------------------
//Data:    		20160901
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------
#ifndef PARAREADER_H
#define PARAREADER_H

#include "DataType.h"
#include "Variable.h"
#include "Graphics_IRrad.h"
#include "Atmod.h"

#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>

using namespace std;

class paraReader
{
public:
    paraReader();

    bool DefScenePara(const char scncfgfn[]);//Desc: to define all scene parameters needed for scene render calculation
    int32 copystringfromfile(char * dest, char * src, int32 offset);//用于读入一个从开头起offset偏移地址处开始的字符串文件名到指定位置，字符串以分号结束。字符串最大长度小于MAX_FILENAME_CHAR
    bool LoadPara(const char argv[]);//从指定相机参数文件传入相机参数
    bool calcCameraInnerPara(void);//计算相机的内参数
    bool LoadTrajFromFile(int32 argc,char argv[],int32 tgtno);//从文件中导入轨迹
    bool LoadCamMovData(void);//读入相机运动轨迹
    bool LoadMDLFromFile(char scncfgfn[], int32 tgtno);//读入每个目标的模型文件
    bool MallocBufferForTgt();//根据文件头为目标模型分配空间
    bool loadTarget_Mov(void);//导入每个目标的模型和轨迹

    bool ReadTrajconfig(const char argv[]);
    bool TrajStore(void);
    bool TrajStateStore(void);

    bool LoadBkg(const char scncfgfn[]);//读入背景数据
    bool ReadBkgFromFile(IRBKG * pbkg);//从指定DV2文件的指定frame中读取背景数据

    float32	CalcBlackbodyRadiance(const float32 tmpr);// 计算黑体的辐射强度(W/cm2.sr)
    float32 *	CalcBbRadianceLookupTable(float32 * rad, float32 IRTmprMin, float32 IRTmprMax, float32 IRTmprDelta);//计算黑体的红外辐射通量密度的查找表
    float32	SearchRadInLookupTable(float32 * radtbl, const float32 tmpr, float32 IRTmprMin, float32 IRTmprMax, float32 IRTmprDelta);//根据指定温度从查找表中找到黑体的红外辐射通量密度
    int32	RadToGray(const float32 rad);//将红外辐射映射为灰度

    bool LoadAtm(char scncfgfn[]);

    //===========================立体视觉========================
    void ParamsReader( string filename);
    string getData( string key );
    void init_runParams(RunParams& runParams);

public:
    Graphics_IRrad * graph_irrad;
    Atmod          * atmod;

    int32		TrajNum;//目标轨迹步数
    TRAJECTORY   	m_pTraj_tmp;//初始轨迹
    TRAJECTORY *	m_pTraj;//设置轨迹
    float32         TrajStopTime;

    //============================立体视觉==========================
    map<string, string> data;

};



#endif // PARAREADER_H
