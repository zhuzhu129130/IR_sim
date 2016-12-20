//--------------------------------------------------------------------------------------
//Data:    		20160505
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------
#include "Atmod.h"
#include <string.h>
//#include <stdlib.h>
//#include <stdio.h>
#include <math.h>

//=================读入大气参数文件中的数据=====================
bool Atmod::InitAtmod(ATMOD * patm)
{
    FILE * fp;

    //patm->AtmodDir=::GetCommandLine ();
    //char * strapp=::AfxGetAppName ();
    //strapp +=".exe";
    //char * temp3="atmod\\";
    //patm->AtmodDir.Replace(strapp,temp3);
    //char * temp1="\\";
    //char * temp2="\\\\";
    //patm->AtmodDir.Replace(temp1, temp2);
    //temp1="\"";
    //temp2="";
    //patm->AtmodDir.Replace(temp1, temp2);

    //::SetCurrentDirectory(patm->AtmodDir);//设置文件所在目录，这个对我没用，直接放到程序目录下即可


    patm->m_atmd = 8;//大气模式
    patm->m_aero = 4;//气溶胶
    patm->m_icld = 0;//云的种类
    patm->m_month = 1;//月份选择
    patm->m_himin = 0;//传输路径参数
    patm->m_himax = 100;//传输路径参数
    patm->m_gndalt = 0.1f;//传输路径参数//patm->m_gndalt = patm->m_dist = patm->m_theta = 0;
    patm->m_temp = patm->m_press = patm->m_rh = 0;//气象数据参数
    patm->m_visi = 10;//气象数据参数
    patm->m_rainrt = patm->m_cthick = patm->m_calt = patm->m_rextcld = 0;//云雨参数
    patm->m_wl1 = cam.cpara.camDet.detwl1;//波长参数,长波8-14
    patm->m_wl2 = cam.cpara.camDet.detwl2;//波长参数,中波4-8
    patm->m_wvstep = 0.02f;//波长参数，步长
    patm->m_met = 'N';//有无该高度的气象数据

    int32 cnum=0;
    while (patm->AtmodDir[cnum]!='\0')//配置文件的大气参数的目录
    {
        patm->m_fn[cnum]=patm->AtmodDir[cnum];
        cnum++;
    }
    patm->m_fn[cnum] = 'S';//大气模式文件
    patm->m_fn[cnum+1] = 'h';
    patm->m_fn[cnum+2] = 'a';
    patm->m_fn[cnum+3] = 'n';
    patm->m_fn[cnum+4] = 'g';
    patm->m_fn[cnum+5] = 'h';
    patm->m_fn[cnum+6] = 'a';
    patm->m_fn[cnum+7] = 'i';
    patm->m_fn[cnum+8] = '\0';

    patm->m_sh = 'N';//'Y';//是否计算斜程透过率

    patm->m_bTranChoice = false;//传输路径是否已输入
    patm->m_bMeteoChoice = false;//气象条件是否已输入
    patm->att=1;
    memset(patm->Taero,0,sizeof(patm->Taero));
    memset(patm->TH2O,0,sizeof(patm->TH2O));
    memset(patm->TH2OC,0,sizeof(patm->TH2OC));
    memset(patm->TN2,0,sizeof(patm->TN2));
    memset(patm->TCO2,0,sizeof(patm->TCO2));
    memset(patm->TO3,0,sizeof(patm->TO3));
    memset(patm->TN2O,0,sizeof(patm->TN2O));
    memset(patm->TCH4,0,sizeof(patm->TCH4));
    memset(patm->TRC,0,sizeof(patm->TRC));
    memset(patm->WL,0,sizeof(patm->WL));
    memset(patm->ch2o,0,sizeof(patm->ch2o));
    memset(patm->slf296,0,sizeof(patm->slf296));
    memset(patm->slf260,0,sizeof(patm->slf260));
    memset(patm->frh,0,sizeof(patm->frh));
    memset(patm->cn2,0,sizeof(patm->cn2));
    memset(patm->cco2,0,sizeof(patm->cco2));
    memset(patm->co3,0,sizeof(patm->co3));
    memset(patm->cch4,0,sizeof(patm->cch4));
    memset(patm->cn2o,0,sizeof(patm->cn2o));
    memset(patm->cco,0,sizeof(patm->cco));
    memset(patm->cnh3,0,sizeof(patm->cnh3));
    memset(patm->cno,0,sizeof(patm->cno));
    memset(patm->cso2,0,sizeof(patm->cso2));




    patm->nmax=0;
    patm->nmin=0;
    patm->IsCal=false;

    char str1[MAX_FILENAME_CHAR];
    cnum=0;
    while (patm->AtmodDir[cnum]!='\0')
    {
        str1[cnum]=patm->AtmodDir[cnum];
        cnum++;
    }
    str1[cnum]='P';
    str1[cnum+1]='A';
    str1[cnum+2]='R';
    str1[cnum+3]='A';
    str1[cnum+4]='.';
    str1[cnum+5]='D';
    str1[cnum+6]='A';
    str1[cnum+7]='T';
    str1[cnum+8]='\0';
    if ((fp=fopen(str1,"rt"))==NULL)
    {
        patm->IsCal=false;
        patm->att=1;
        printf(" PARA.DAT File could not be opened\n");
        return false;
    }
    char  fl[MAX_FILENAME_CHAR];
    int32 i;
    float32 kk=930;

    //读出文件中的数据，根据文件的定义分配给各个变量
    fscanf(fp,"%s",fl);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->ch2o[1][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=131;i++)
        fscanf(fp,"%f",&patm->ch2o[1][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=128;i++)
        fscanf(fp,"%f",&patm->ch2o[2][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->ch2o[3][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=178;i++)
        fscanf(fp,"%f",&patm->ch2o[3][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->ch2o[4][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=178;i++)
        fscanf(fp,"%f",&patm->ch2o[4][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->ch2o[5][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=178;i++)
        fscanf(fp,"%f",&patm->ch2o[5][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->ch2o[6][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=252;i++)
        fscanf(fp,"%f",&patm->ch2o[6][i]);
    fscanf(fp,"%s",fl);
    for (i=253;i<=368;i++)
        fscanf(fp,"%f",&patm->ch2o[6][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->ch2o[7][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=252;i++)
        fscanf(fp,"%f",&patm->ch2o[7][i]);
    fscanf(fp,"%s",fl);
    for (i=253;i<=370;i++)
        fscanf(fp,"%f",&patm->ch2o[7][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->ch2o[8][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=252;i++)
        fscanf(fp,"%f",&patm->ch2o[8][i]);
    fscanf(fp,"%s",fl);
    for (i=253;i<=323;i++)
        fscanf(fp,"%f",&patm->ch2o[8][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->ch2o[9][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=252;i++)
        fscanf(fp,"%f",&patm->ch2o[9][i]);
    fscanf(fp,"%s",fl);
    for (i=253;i<=385;i++)
        fscanf(fp,"%f",&patm->ch2o[9][i]);

    fscanf(fp,"%s",fl);
    for (i=1;i<=kk;i++)
        fscanf(fp,"%f",&patm->slf296[i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=kk;i++)
        fscanf(fp,"%f",&patm->slf260[i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=kk;i++)
        fscanf(fp,"%f",&patm->frh[i]);

    fscanf(fp,"%s",fl);
    fscanf(fp,"%s",fl);
    for (i=1;i<=133;i++)
        fscanf(fp,"%f",&patm->cn2[i]);

    fscanf(fp,"%s",fl);
    fscanf(fp,"%s",fl);
    for (i=1;i<=83;i++)
        fscanf(fp,"%f",&patm->cco2[1][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=121;i++)
        fscanf(fp,"%f",&patm->cco2[2][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->cco2[3][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=211;i++)
        fscanf(fp,"%f",&patm->cco2[3][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->cco2[4][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=138;i++)
        fscanf(fp,"%f",&patm->cco2[4][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=62;i++)
        fscanf(fp,"%f",&patm->cco2[5][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->cco2[6][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=171;i++)
        fscanf(fp,"%f",&patm->cco2[6][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->cco2[7][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=225;i++)
        fscanf(fp,"%f",&patm->cco2[7][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->cco2[8][i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=208;i++)
        fscanf(fp,"%f",&patm->cco2[8][i]);

    fscanf(fp,"%s",fl);
    fscanf(fp,"%s",fl);
    for (i=1;i<=153;i++)
        fscanf(fp,"%f",&patm->co3[1][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=134;i++)
        fscanf(fp,"%f",&patm->co3[2][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=36;i++)
        fscanf(fp,"%f",&patm->co3[3][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=83;i++)
        fscanf(fp,"%f",&patm->co3[4][i]);

    fscanf(fp,"%s",fl);
    fscanf(fp,"%s",fl);
    for (i=1;i<=493;i++)
        fscanf(fp,"%f",&patm->cch4[i]);

    fscanf(fp,"%s",fl);
    fscanf(fp,"%s",fl);
    for (i=1;i<=364;i++)
        fscanf(fp,"%f",&patm->cn2o[1][i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=315;i++)
        fscanf(fp,"%f",&patm->cn2o[2][i]);

    fscanf(fp,"%s",fl);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->cco[i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=137;i++)
        fscanf(fp,"%f",&patm->cco[i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=126;i++)
        fscanf(fp,"%f",&patm->cnh3[i]);
    fscanf(fp,"%s",fl);
    for (i=127;i<=252;i++)
        fscanf(fp,"%f",&patm->cnh3[i]);
    fscanf(fp,"%s",fl);
    for (i=253;i<=353;i++)
        fscanf(fp,"%f",&patm->cnh3[i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=62;i++)
        fscanf(fp,"%f",&patm->cno[i]);
    fscanf(fp,"%s",fl);
    for (i=1;i<=76;i++)
        fscanf(fp,"%f",&patm->cso2[i]);
    fscanf(fp,"%s",fl);
    for (i=77;i<=138;i++)
        fscanf(fp,"%f",&patm->cso2[i]);

    fclose(fp);


    return true;
}

//===================随着目标运动更新参数========================
void Atmod::UpdateAtmPara(ATMOD * patm, IRTARGET * pt)
{
    if (patm->m_sh == 'Y')//斜程路径
    {
        patm->m_himax = pt->m_stat.pos.y/1000.0f;
        patm->m_himin = cam.m_stat.pos.y/1000.0f;
        patm->m_gndalt = patm->m_himin;
        patm->m_theta = atan((patm->m_himax-patm->m_himin)*1000.0f/(sqrt(pt->m_stat.pos.z*pt->m_stat.pos.z+pt->m_stat.pos.x*pt->m_stat.pos.x)-sqrt(cam.m_stat.pos.z*cam.m_stat.pos.z+cam.m_stat.pos.x*cam.m_stat.pos.x)));
    }

    else// if (patm->m_sh == 'N')//水平路径
    {
        patm->m_dist = pt->CamTgtMod;//sqrt(pt->m_stat.pos.z*pt->m_stat.pos.z+pt->m_stat.pos.x*pt->m_stat.pos.x)-sqrt(cam.m_stat.pos.z*cam.m_stat.pos.z+cam.m_stat.pos.x*cam.m_stat.pos.x);
        patm->m_dist/=1000.0f;
        patm->m_himax = pt->m_stat.pos.y/1000.0f;
        patm->m_himin = patm->m_himax;
        patm->m_gndalt = patm->m_himax;
    }
}

float32 Atmod::OnCalculate(ATMOD * patm,IRTARGET * pt)
{
    UpdateAtmPara( patm,pt);//随着目标运动更新参数
    int32 nn;
    float32 T[ATMOD_N0+1],TF[ATMOD_N0+1];

    nn = ParaCalcu(1,patm);//计算各种分子吸收、气溶胶颗粒衰减和透过率
    /////////////////////////////////////////////////////////////////////////////////////
    //result
    int32 i;
    for (i=1;i<=nn;i++)
    {
        T[i]=patm->TH2O[i]*patm->TH2OC[i]*patm->TCO2[i]*patm->TO3[i]*patm->TRC[i]*patm->TN2O[i]*patm->TCH4[i]*patm->TN2[i];
        T[i]=T[i]*patm->Taero[i];
        TF[i]=patm->TH2O[i]*patm->TH2OC[i]*patm->TCO2[i];
    }
    float32 tt = 0,ttf=0;
    for(i=2;i<=nn;i++)
    {
        tt = tt+(T[i]+T[i-1])*(patm->WL[i]-patm->WL[i-1])/2;
        ttf = ttf+(TF[i]+TF[i-1])*(patm->WL[i]-patm->WL[i-1])/2;
    }
    float32 att = tt/(patm->WL[nn]-patm->WL[1]);
    return (att);
}

//计算各种分子吸收、气溶胶颗粒衰减和透过率
int32 Atmod::ParaCalcu(int32 cho,ATMOD * patm)
{// cho 选择是进行一次计算 or 批处理计算，来打开不同的参数文件 "ipt.par" or "iptbat.par"
    FILE * fp;
    const int32 WN=27;
    const int32 VN=4;//Oringal data for 27 wavelengths,4 visibilities,2 altitudes
    float32 WL0[WN+1],extc[VN+1+1][WN+1],VS[VN+1],exta[WN+1],scalh[WN+1];//scalh is desert aerosol scal highet
    float32 consta=0;
    int32 nn=0;
    int32 kr = 10;
    float32 sech=0;
    float32 T[ATMOD_N0+1],X[201+1],ERF[201+1];
    float32 tcld[ATMOD_N0+1],beta1[10+1+1][ATMOD_ITP+1+1];
    float32 ABDU[7+1];
    float32 z[ATMOD_NUM+1];//high(km)
    float32 pr[ATMOD_NUM+1];//pressure(mb)
    float32 tp[ATMOD_NUM+1];//temperture(K)
    float32 dena[ATMOD_NUM+1];//atmospheric density(g/cm-3)
    float32 denw[ATMOD_NUM+1];//water density
    float32 den03[ATMOD_NUM+1];//ozen density(g/cm-3)
    float32 FA[7+1][61+1],ASCA[7+1][61+1];
    float32 HZ1[ATMOD_ITP+1+1],HZ2[ATMOD_ITP+1+1],CLA[ATMOD_ITP+1+1];
    float32 TA1[ATMOD_ITP+1+1],evh[ATMOD_ITP+1+1];
    float32 wlaer[130+1],taer[130+1];
    float32 A=0,B=0;
    float32 za[ATMOD_ITP+1+1];
    float32 higha=0,himina=0;
    int32 nmina=0,nmaxa=0;
    float32 zzh1,zzh2,zzh,d11,d12,dd,t,dw;
    zzh1=zzh2=zzh=d11=d12=dd=t=dw=0;
    float32 evh1=0,evh2=0;
    float32 ha1=0,ha2=0;
    float32 aerc=0;
    float32 dzs=0,zh=0,nr1=0,a1=0,dzsa=0,zha=0;
    float32 tz=0,t1=0,t2=0,ttem=0;
    //////////////////////////////////////////////////////////////////////////
    memset(WL0,0,sizeof(WL0));
    memset(extc,0,sizeof(extc));
    memset(VS,0,sizeof(VS));
    memset(exta,0,sizeof(exta));
    memset(scalh,0,sizeof(scalh));
    memset(beta1,0,sizeof(beta1));
    memset(z,0,sizeof(z));
    memset(T,0,sizeof(T));
    memset(X,0,sizeof(X));
    memset(ERF,0,sizeof(ERF));
    memset(tcld,0,sizeof(tcld));
    memset(ABDU,0,sizeof(ABDU));
    memset(pr,0,sizeof(pr));
    memset(tp,0,sizeof(tp));
    memset(dena,0,sizeof(dena));
    memset(denw,0,sizeof(denw));
    memset(den03,0,sizeof(den03));
    memset(FA,0,sizeof(FA));
    memset(ASCA,0,sizeof(ASCA));
    memset(HZ1,0,sizeof(HZ1));
    memset(HZ2,0,sizeof(HZ2));
    memset(CLA,0,sizeof(CLA));
    memset(TA1,0,sizeof(TA1));
    memset(evh,0,sizeof(evh));
    memset(wlaer,0,sizeof(wlaer));
    memset(taer,0,sizeof(taer));
    memset(za,0,sizeof(za));
    patm->m_press = patm->m_press*760.0f/1013.4f;
    nn = (int32)((patm->m_wl2-patm->m_wl1)/patm->m_wvstep)+1;
    if(patm->m_theta!=90.0f)
        sech = 1.0f/cos(PI*patm->m_theta/180.0f);
    int32 NN0 = nn+1;

    /////////////////////////////////////////////////////////////////////////////
    //  打开大气模式文件，如"kuerle.3"
    //::SetCurrentDirectory(patm->AtmodDir);
    //char * str3, str4;
    char 	str1[MAX_FILENAME_CHAR];
    int32 i,j;


    //str4.Format("%s",patm->m_fn);

    fp=fopen(patm->m_fn,"rt");
    if(fp==NULL)
    {
        patm->IsCal=false;
        patm->att=1;
        printf("Can not open the file ");
        return -1;
    }
    fscanf(fp,"%s",str1);
    j=0;
    while (str1[j]!='\0')
    {
        patm->strAtmd[j]=str1[j];
        j++;
    }
    patm->strAtmd[j]='\0';

    for(i=1;i<7+1;i++)
    {
        fscanf(fp,"%f",&ABDU[i]);
    }

    int32 fll1 = 0;
    int32 k;
    for(k=1;k<ATMOD_ITP+1;k++)
    {
        i = ATMOD_ITP-k+1;
        fscanf(fp,"%f%E%f%E%E%E",&z[i],&pr[i],&tp[i],&dena[i],&denw[i],&den03[i]);
        if (z[i]==-1)
            fll1 = fll1 +1;
        //convert the unit g/m3 to g/cm3
        dena[i] = dena[i]*1E-6f;
        denw[i] = denw[i]*1E-6f;
        den03[i] = den03[i]*1E-6f;
    }
    fclose(fp);

    if(z[ATMOD_ITP+1-fll1]<patm->m_gndalt && fll1!=0)
    {
        printf("the error input of ground altitude");
        return -1;
    }
    ///////////////////////////////////////////////////////////////////////////////////
    // to calculate the aerosol extinction 气溶胶消光
    int32 cnum=0;
    while (patm->AtmodDir[cnum]!='\0')
    {
        str1[cnum]=patm->AtmodDir[cnum];
        cnum++;
    }
    str1[cnum]='a';
    str1[cnum+1]='e';
    str1[cnum+2]='r';
    str1[cnum+3]='o';
    str1[cnum+4]='s';
    str1[cnum+5]='o';
    str1[cnum+6]='l';
    str1[cnum+7]='1';
    str1[cnum+8]='.';
    str1[cnum+9]='d';
    str1[cnum+10]='a';
    str1[cnum+11]='t';
    str1[cnum+12]='\0';

    fp = fopen(str1,"rt");
    if(fp==NULL)
    {
        patm->IsCal=false;
        patm->att=1;
        printf("Can not open the file aerosol1.dat");

        return -1;
    }

    char  str[MAX_FILENAME_CHAR];
    for(i=1;i<7+1;i++)
    {
        fscanf(fp,"%s",str);

        j=0;
        while (str[j]!='\0')
        {
            patm->arsmodl[i][j]=str[j];
            j++;
        }
        patm->arsmodl[i][j]='\0';

        for(j=1;j<61+1;j++)
        {
            fscanf(fp,"%f%f%f",&FA[i][j],&A,&B);//消光系数，wavelength,absorption,scpatm->attering
            ASCA[i][j] = A+B;
        }
    }
    fclose(fp);

    if(patm->m_aero==8)//desert aerosol model
    {
        patm->arsmodl[8][0] = 'D';
        patm->arsmodl[8][1] = 'E';
        patm->arsmodl[8][2] = 'S';
        patm->arsmodl[8][3] = 'E';
        patm->arsmodl[8][4] = 'R';
        patm->arsmodl[8][5] = 'T';
        patm->arsmodl[8][6] = '\0';

        int32 cnum=0;
        while (patm->AtmodDir[cnum]!='\0')
        {
            str1[cnum]=patm->AtmodDir[cnum];
            cnum++;
        }
        str1[cnum]='a';
        str1[cnum+1]='e';
        str1[cnum+2]='r';
        str1[cnum+3]='o';
        str1[cnum+4]='s';
        str1[cnum+5]='o';
        str1[cnum+6]='l';
        str1[cnum+7]='2';
        str1[cnum+8]='.';
        str1[cnum+9]='d';
        str1[cnum+10]='a';
        str1[cnum+11]='t';
        str1[cnum+12]='\0';

        fp = fopen(str1,"rt");
        if(fp==NULL)
        {
            patm->IsCal=false;
            patm->att=1;
            printf("Can not open the file aerosol2.dat");
            return -1;
        }
        for(i=1;i<VN+1;i++)
        {
            fscanf(fp,"%f",&VS[i]);//4 KINDS OF DEFAULT VISIBILITY VS(I)=2,5,10,23
        }
        fscanf(fp,"%s",str);
        for(j=1;j<WN+1;j++)
        {
            fscanf(fp,"%f",&WL0[j]);//wavelength
        }
        for(i=1;i<VN+1+1;i++)
        {
            fscanf(fp,"%s",str);
            for(j=1;j<WN+1;j++)
            {
                fscanf(fp,"%f",&extc[i][j]);//extiction coefficient of different visibility and wavelength
            }
        }
        fclose(fp);
        //Interpolate  the extiction coeffient based on visibility
        //To asuuume the deast aerosol density decrease in the form:
        //              sigm(h)=sigm0*exp(-(h-1)/scalh)
        //where scalh can be figure out from ground and 2km altitude extiction
        //and assume the density of aerosol below 1 km is unformly.
        for(i=1;i<VN+1;i++)
        {
            if(patm->m_visi<=VS[i])
                break;
        }
        if(i==1)
            i = i+1;
        //for the case of visibility < 2km just use 2 and 5km to interpolate
        consta = (1/patm->m_visi-1/(VS[i-1]))/(1/VS[i]-1/VS[i-1]);
        for(j=1;j<WN+1;j++)
        {
            exta[j] = consta*(extc[i][j]-extc[i-1][j])+extc[i-1][j];
            scalh[j] = 1/(log(exta[j]/extc[VN+1][j]));
            if(scalh[j]<0)
                scalh[j] = 10000;
        }
    }


    cnum=0;
    while (patm->AtmodDir[cnum]!='\0')
    {
        str1[cnum]=patm->AtmodDir[cnum];
        cnum++;
    }
    str1[cnum]='a';
    str1[cnum+1]='e';
    str1[cnum+2]='r';
    str1[cnum+3]='o';
    str1[cnum+4]='s';
    str1[cnum+5]='o';
    str1[cnum+6]='l';
    str1[cnum+7]='3';
    str1[cnum+8]='.';
    str1[cnum+9]='d';
    str1[cnum+10]='a';
    str1[cnum+11]='t';
    str1[cnum+12]='\0';
    fp = fopen(str1,"rt");

    if(fp==NULL)
    {
        patm->IsCal=false;
        patm->att=1;
        printf("Can not open the file aerosol3,dat");
        return -1;
    }
    for(i=1;i<ATMOD_ITP+1;i++)
    {
        fscanf(fp,"%f",&HZ1[i]);
    }
    for(i=1;i<ATMOD_ITP+1;i++)
    {
        fscanf(fp,"%f",&HZ2[i]);
    }
    fclose(fp);
    //////////////////////////////////////////////////////
    //  high range
    if(patm->m_himax!=0)
    {
        if(patm->m_himin!=0)
        {
            for(i=1;i<ATMOD_ITP+1;i++)
            {
                if(z[i]<=patm->m_himin)
                {
                    patm->nmin = i;
                    break;
                }
            }
        }
        else
        {
            patm->nmin = ATMOD_ITP;
        }
        //102
        if(patm->m_himax!=patm->m_himin)
        {
            if(patm->m_himax<100)
            {
                for(i=ATMOD_ITP;i>=1;i--)
                {
                    if(z[i]>=patm->m_himax)
                    {
                        patm->nmax = i;
                        break;
                    }
                }
            }
            else
            {
                patm->nmax = 1;
            }
        }
        else
        {
            patm->nmax = patm->nmin;
        }
    }
    else
    {
        patm->nmax = ATMOD_ITP;
        patm->nmin = ATMOD_ITP;
    }
    //104
    for(i=patm->nmax;i<patm->nmin+1;i++)
    {
        if(patm->m_gndalt!=0 && z[i]<=6)
        {
            za[i] = (z[i]-patm->m_gndalt)/(1-patm->m_gndalt/6);
            if(za[i]<0)
                za[i] = 0;
        }
        else
        {
            za[i] = z[i];
        }
    }

    higha = patm->m_himax;
    himina = patm->m_himin;
    nmina = patm->nmin;
    nmaxa = patm->nmax;
    if(patm->m_himax<6 && patm->m_gndalt>0)
    {
        higha=(patm->m_himax-patm->m_gndalt)/(1-patm->m_gndalt/6);
        nmaxa=(int32)(ATMOD_ITP-za[patm->nmax]);
    }
    if(patm->m_himin<6 && patm->m_gndalt>0)
    {
        himina=(patm->m_himin-patm->m_gndalt)/(1-patm->m_gndalt/6);
        nmina=(int32)(ATMOD_ITP-za[patm->nmin]);
    }
    if(patm->m_himax!=patm->m_himin)
    {
        zzh1 = za[nmaxa];
        zzh2 = za[nmaxa+1];
        zzh = higha;
        d11 = HZ1[nmaxa];
        d12 = HZ1[nmaxa+1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);//exp insertion
        HZ1[nmaxa] = dd;

        d11 = HZ2[nmaxa];
        d12 = HZ2[nmaxa+1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        HZ2[nmaxa] = dd;
        if(HZ1[nmaxa]<=0)
            HZ1[nmaxa] = 1E-10f;
        if(HZ2[nmaxa]<=0)
            HZ2[nmaxa] = 1E-10f;

        zzh1 = za[nmina];
        zzh2 = za[nmina-1];
        zzh = himina;
        d11 = HZ1[nmina];
        d12 = HZ1[nmina-1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-10f;
        HZ1[nmina] = dd;

        d11 = HZ2[nmina];
        d12 = HZ2[nmina-1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-15f;
        HZ2[nmina] = dd;

        float32 dftz1,dftz2;

        dftz1 = (patm->m_himax-z[patm->nmax])/(z[patm->nmax]-z[patm->nmax+1]);
        dftz2 = (patm->m_himin-z[patm->nmin])/(z[patm->nmin]-z[patm->nmin-1]);
        //To calculate from the higher or lower layer, the vlaue of higher
        //or lower layer aer interpolated linearly between the high and z(patm->nmax),
        //himin and z(patm->nmin) respective
        zzh1 = z[patm->nmin];
        zzh2 = z[patm->nmin-1];
        zzh = patm->m_himin;
        d11 = dena[patm->nmin];
        d12 = dena[patm->nmin-1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-25f;
        dena[patm->nmin] = dd;

        d11 = denw[patm->nmin];
        d12 = denw[patm->nmin-1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-25f;
        denw[patm->nmin] = dd;

        d11 = den03[patm->nmin];
        d12 = den03[patm->nmin-1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-25f;
        den03[patm->nmin] = dd;

        d11 = pr[patm->nmin];
        d12 = pr[patm->nmin-1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-25f;
        pr[patm->nmin] = dd;

        zzh1 = z[patm->nmax];
        zzh2 = z[patm->nmax+1];
        zzh = patm->m_himax;
        d11 = dena[patm->nmax];
        d12 = dena[patm->nmax+1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-25f;
        dena[patm->nmax] = dd;

        d11 = denw[patm->nmax];
        d12 = denw[patm->nmax+1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-25f;
        denw[patm->nmax] = dd;

        d11 = den03[patm->nmax];
        d12 = den03[patm->nmax+1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-25f;
        den03[patm->nmax] = dd;

        d11 = pr[patm->nmax];
        d12 = pr[patm->nmax+1];
        dd = CalcuSub1(zzh1,zzh2,zzh,d11,d12);
        if(dd<=0)
            dd = 1E-25f;
        pr[patm->nmax] = dd;

        tp[patm->nmax] = tp[patm->nmax]+(tp[patm->nmax]-tp[patm->nmax+1])*dftz1;
        if (tp[patm->nmax]<=0)
            tp[patm->nmax]=1E-20f;
        tp[patm->nmin]=tp[patm->nmin]+(tp[patm->nmin]-tp[patm->nmin-1])*dftz2;
        if (tp[patm->nmin]<=0)
            tp[patm->nmin]=1E-20f;
        z[patm->nmax] = patm->m_himax;
        z[patm->nmin] = patm->m_himin;

        za[nmaxa] = higha;
        za[nmina] = himina;
    }
    //105
    int32 kl=0;

    z[patm->nmin+1]=z[patm->nmin];
    za[nmina+1]=za[nmina];
    pr[patm->nmin+1]=pr[patm->nmin];
    tp[patm->nmin+1]=tp[patm->nmin];
    dena[patm->nmin+1]=dena[patm->nmin];
    denw[patm->nmin+1]=denw[patm->nmin];
    den03[patm->nmin+1]=den03[patm->nmin];
    HZ1[nmina+1]=HZ1[nmina];
    HZ2[nmina+1]=HZ2[nmina];

    //below is for the defraction of low tenith angle
    const float32 rr0=6370.0f;     //the RADIUS of the earth in km
    float32 mwl2=0;
    mwl2 = pow((patm->m_wl2+patm->m_wl1)/2.0f,2.0f);
    float32 ph2o0=0;
    ph2o0 = denw[patm->nmin]*1E6f*8.31f*tp[patm->nmin]/18.0f;
    float32 nr0,AA,ph2o2,a2,beta;
    float32 nr[ATMOD_ITP+1],aer[ATMOD_ITP+1];
    nr0=AA=ph2o2=a2=beta=0;
    memset(nr,0,sizeof(nr));
    memset(aer,0,sizeof(aer));
    nr0 = 1.0f+((77.46f+0.459f/mwl2)*pr[patm->nmin]/tp[patm->nmin]-ph2o0/1.013E5f*(43.49f-0.347f/mwl2))*1E-6f;
    AA=nr0*(rr0+z[patm->nmin])*sin(patm->m_theta*ATMOD_PI/180.0f);
    for(i=patm->nmax;i<patm->nmin+1;i++)
    {
        ph2o2=denw[i]*8.31f*1E6f*tp[i]/18.0f;         // pressure of h2o  unit:pascal
        nr[i]=1+((77.46f+0.459f/mwl2)*pr[i]/tp[i]-ph2o2/1.013e5f*(43.49f-0.347f/mwl2))*1e-6f;
        a2=AA/(nr[i]*(rr0+z[i]));
        beta=atan(a2/sqrt(1-a2*a2));
    }
    //beta(0)=beta(1)
    //134
    for(i=patm->nmax;i<patm->nmin;i++) //II=patm->nmax TO patm->nmin-1
    {
        dzs=(z[i]-z[i+1])/kr;
        for (j=kr;j>=0;j--)// step -1
        {
            zh=z[i+1]+j*dzs;
            nr1=nr[i]+(nr[i+1]-nr[i])*(zh-z[i])/(z[i+1]-z[i]);
            a1=AA/(nr1*(rr0+z[i]+dzs*j));
            beta1[j][i]=atan(a1/sqrt(1-a1*a1));
        }
    }
    for(k=nmaxa;k<nmina+1;k++)
    {
        if(patm->m_himax!=patm->m_himin)
        {
            if(HZ1[k]!=HZ1[k+1])
            {
                ha1 = 1.0f/log(HZ1[k+1]/HZ1[k]);
                evh1 = ha1*(HZ1[k+1]-HZ1[k]);
            }
            else
            {//113
                evh1 = HZ1[k];
            }
            if(HZ2[k]!=HZ2[k+1])
            {
                ha2 = 1.0f/log(HZ2[k+1]/HZ2[k]);
                evh2 = ha2*(HZ2[k+1]-HZ2[k]);
            }
            else
            {//116
                evh2 = HZ2[k];
            }
        }
        else
        {//111
            evh1 = HZ1[nmina];
            evh2 = HZ2[nmina];
        }
        //117
        evh[k] = (evh2-evh1)*(1.0f/patm->m_visi-1.0f/23.0f)/(1.0f/5.0f-1.0f/23.0f)+evh1;
        if(evh[k]<0)
            evh[k] = 0;
        if(k<31)
            evh[k] = evh1;
    }
    /////////////////////////////////////////////////////////////////
    const float32 wvsa=(float32)0.20;
    //frequency loop
    kl = 0;
    float32 v=0;
    float32 walgh=0;
    float32 ta=0,sa=0,h=0;
    for(v=patm->m_wl1;v<=patm->m_wl2+0.5f;v=v+wvsa)
    {
        kl = kl+1;
        walgh = v;
        wlaer[kl] = v;
        for(j=1;j<61+1;j++)
        {
            if(v>=FA[1][j] && v<=FA[1][j+1])
                break;
        }
        if(j==62)
        {
            patm->IsCal=false;
            patm->att=1;
            printf("out of aerosol frequency range");
            return -1;
        }
        //109
        float32 fac=0;
        int32 lc=0,lh=0;
        fac = (v-FA[1][j])/(FA[1][j+1]-FA[1][j]);
        bool bconti=false;

        for(k=nmaxa;k<nmina+1;k++)
        {
            if(z[k]<=100 && z[k]>=30)
            {
                lc = 7;
                lh = 7;
            }
            else if(z[k]<30 && z[k]>=9)
            {
                lc = 5;
                lh = 6;
            }
            else if(z[k]<9 && z[k]>2)
            {
                lc = 2;
                lh = 2;
            }
            else if(z[k]<=2 && z[k]>=0)
            {
                lc = patm->m_aero;
                lh = patm->m_aero;
                if(patm->m_aero==8)
                {//desert aerosol model
                    for(j=WN;j>=1;j--)
                    {
                        if(walgh>=WL0[j])
                        {//126
                            consta = (walgh-WL0[j])/(WL0[j+1]-WL0[j]);
                            ta = consta*(exta[j+1]-exta[j])+exta[j];
                            sa = consta*(scalh[j+1]-scalh[j])+scalh[j];
                            h = z[k];
                            if(h<1)
                                h = 1;
                            TA1[k] = ta*exp(-(h-1)/sa);
                            bconti = true;
                            break;
                        }
                    }

                }
            }
            if(bconti)
                continue;
            if(j==0)
                j = 1;
            //127
            float32 akcl,akhz;
            akcl = (ASCA[lc][j+1]-ASCA[lc][j])*fac+ASCA[lc][j];
            if(lc!=lh)
                akhz = (ASCA[lh][j+1]-ASCA[lh][j])*fac+ASCA[lh][j];

            TA1[k] = evh[k]*akcl;
        }//k for
        aerc = 0;
        int32 iia1=0,iia2=0,ir=0;
        if(patm->m_himax!=patm->m_himin)
        {
            for(i=patm->nmax;i<patm->nmin;i++)
            {
                iia1 = i;
                iia2 = i+1;
                if(patm->m_gndalt>0 && z[i]<6)
                    iia1 = (int32)(ATMOD_ITP-za[i]);
                if(patm->m_gndalt>0 && z[i+1]<6)
                    iia2 = (int32)(ATMOD_ITP-za[i+1]);
                dzs = (z[i]-z[i+1])/kr;
                dzsa = (za[i]-za[i+1])/kr;
                for(ir=kr;ir>=1;ir--)
                {
                    zh = z[i+1]+ir*dzs;
                    zha = za[i+1]+ir*dzsa;
                    beta = beta1[ir][i]-beta1[ir+1][i];
                    tz = dzs*cos(beta/2)/fabs(cos(beta/2+beta1[ir+1][i]));
                    if(z[iia1]!=z[iia1+1])
                    {
                        t1 = TA1[iia1]+(TA1[iia1+1]-TA1[iia1])*(z[iia1]-za[i])/(z[iia1]-z[iia1+1]);
                    }
                    else
                    {
                        t1 = TA1[iia1];
                    }
                    if(z[iia2]!=z[iia2+1])
                    {
                        t2 = TA1[iia2]+(TA1[iia2+1]-TA1[iia2])*(z[iia2]-za[i+1])/(z[iia2]-z[iia2+1]);
                    }
                    else
                    {
                        t2 = TA1[iia2];
                    }
                    ttem =t1+(t2-t1)*(zh+dzs/2-z[i])/(z[i+1]-z[i]);
                    if(ttem<0)
                        ttem = t1;
                    //135
                    aerc = aerc+ttem*tz;
                }//ir for
            }//i for
            taer[kl] = exp(-aerc);
        }
        else
        {//131
            taer[kl] = exp(-TA1[patm->nmin]*patm->m_dist);
        }
    }//v for
    int32 nkl=0;
    float32 wl=0;
    int32 kj;
    nkl = (int32)((patm->m_wl2-patm->m_wl1)/wvsa+1);
    for(kl=1;kl<=NN0;kl++)
    {
        wl = patm->m_wl1+(kl-1)*patm->m_wvstep;
        for(kj=1;kj<=nkl;kj++)
        {
            if(wl<wlaer[kj])
            {
                patm->Taero[kl] = taer[kj-1]+(wl-wlaer[kj-1])*(taer[kj]-taer[kj-1])/(wlaer[kj]-wlaer[kj-1]);
                break;
            }
            else if(wl>wlaer[kj])
                continue;
            else if(wl==wlaer[kj])
            {
                patm->Taero[kl] = taer[kj];
                break;
            }
        }//141
        if(kj>nkl)
            patm->Taero[kl] = taer[kj-1]+(wl-wlaer[kj-1])*(taer[kj]-taer[kj-1])/(wlaer[kj]-wlaer[kj-1]);

    }//143

    float32 cld[8][17],rr[8][17];
    float32 extcld[8][48];
    memset(cld,0,sizeof(cld));
    memset(rr,0,sizeof(rr));
    memset(extcld,0,sizeof(extcld));

    float32 zcld[17]={0.0f,0.0f,0.16f,0.33f,0.66f,1.0f,1.5f,2.0f,2.4f,2.7f,3.0f,3.5f,4.0f,4.5f,5.0f,5.5f,6.0f};
    float32 calt1[6]={0.0f,0.66f,2.4f,0.33f,0.66f,0.16f};
    float32 calt2[6]={0.0f,2.7f,3.0f,10.0f,2.0f,0.66f};
    int32 fg[6]={0,4,8,3,4,3};
    float32 wl1[48],th,cona;
    memset(wl1,0,sizeof(wl1));

    float32 wcld=0,wrain=0,zmdl=0,zk=0;
    float32 kr1=0,kt=0,ss=0,fac=0;
    float32 cldatz,rratz,cldamt,zdif,rramt,train;
    float32 n2,h2oslf1,h2oslf2,h2ofrn;
    float32 es,eh2o,slf1,slf2,frn,wn2,ra,rw,ro3,m1o3,PR,TP,rp,rt;
    float32 tslf,tfrn,conjoe,conco2,cono3,wch4,conn2o,wco,wno,wnh3,wso2;
    float32 ah2o[10]={0.0f,1.1406f,0.9834f,1.0443f,0.9681f,0.9555f,0.9362f,0.9233f,0.8658f,0.8874f};
    float32 bh2o[10]={0.0f,-2.6343f,-2.5294f,-2.4359f,-1.9537f,-1.5378f,-1.6338f,-0.9398f,-0.1034f,-0.2576f};
    float32 wh2o[10],wco2[9],wo3[5],wn2o[3];
    cldatz=rratz=cldamt=zdif=rramt=train=0;
    n2=h2oslf1=h2oslf2=h2ofrn=0;
    es=eh2o=slf1=slf2=frn=wn2=patm->m_wh2o0=ra=rw=ro3=m1o3=PR=TP=rp=rt=0;
    tslf=tfrn=conjoe=conco2=cono3=wch4=conn2o=wco=wno=wnh3=wso2=0;
    memset(wh2o,0,sizeof(wh2o));
    memset(wco2,0,sizeof(wco2));
    memset(wo3,0,sizeof(wo3));
    memset(wn2o,0,sizeof(wn2o));
    //parameter for CO2 8 band model
    float32 aco2[9]={0.0f,0.6705f,0.7038f,0.7258f,0.6982f,0.8867f,0.7883f,0.6899f,0.6035f};
    float32 bco2[9]={0.0f,-2.2560f,-5.0768f,-1.6740f,-1.8107f,-0.5327f,-1.3244f,-0.8152f,0.6026f};
    float32 ao3[5]={0.0f,0.4221f,0.3739f,0.1770f,0.3921f};
    float32 bo3[5]={0.0f,0.7678f,0.1225f,0.9827f,0.1942f};
    float32 an2o[3]={0.0f,0.7203f,0.7764f};
    float32 bn2o[3]={0.0f,-0.1836f,1.1931f};
    float32 bndwvl[11]={0.0f,350.0f,1005.0f,1645.0f,2535.0f,3425.0f,4315.0f,6155.0f,8005.0f,9620.0f,0.0f};
    float32 bndwvh[11]={0.0f,1000.0f,1640.0f,2530.0f,3420.0f,4310.0f,6150.0f,8000.0f,9615.0f,11540.0f,0.0f};
    float32 ach2o[10]={0.0f,0.5299f,0.5416f,0.5479f,0.5495f,0.5464f,0.5454f,0.5474f,0.5579f,0.5621f};
    float32 acco2[11]={0.0f,0.6176f,0.6810f,0.6033f,0.6146f,0.6513f,0.6050f,1.6160f,0.7070f,0.7070f,0.7070f};
    float32 aco3[5]={0.0f,0.7593f,0.7819f,0.9175f,0.7703f};
    float32 acch4[5]={0.0f,0.5844f,0.5844f,0.5844f,0.5844f};
    float32 acn2o[11]={0.0f,0.7201f,0.7201f,0.7201f,0.7201f,0.7201f,0.6933f,0.6933f,0.6933f,0.6933f,0.6933f};

    const float32 CLDTP= (float32)6.0001f;
    const float32 DELZ= (float32)0.002f;
    int32 mc=0,mr=0,mk=0;
    const float32 CULWC = (float32)7.683E-03f;
    const float32 ASLWC = (float32)4.509E-03f;
    const float32 STLWC = (float32)5.272E-03f;
    const float32 SCLWC = (float32)4.177E-03f;
    const float32 SNLWC = (float32)7.518E-03f;
    const float32 TNLWC = (float32)3.446E-03f;
    const float32 TKLWC = (float32)5.811E-02f;

    const float32 MCO2 =(float32)330.0f;//the mixing ratio of  CO2(ppm)
    const float32 MCH4 =(float32)1.6f;//the mixing ratio of  CH4(ppm)
    const float32 MN2O =(float32)0.28f;//the mixing ratio of  N2O(ppm)
    const float32 MCO =(float32)1.4E-1f;//the mixing ratio of  CO(ppm)
    const float32 MNO =(float32)3.0E-4f;//the mixing ratio of  NO(ppm)
    const float32 MNH3 =(float32)5.0E-4f;//the mixing ratio of  NH3(ppm)
    const float32 MSO2 =(float32)3.0E-4f;//the mixing ratio of  SO2(ppm)
    const float32 MN2 =(float32)0.781f;//the mixing ratio of  N2(ppm)

    const float32 DEN = (float32)1.225E-06f;//unit:kg/cm^3
    const float32 NDENS = (float32)1E-20f;//1E-20 is the unit of h2o continum coefficient
    //PARAMETER FOR H2O SEL CONTINUM
    const float32 AFGDN=(float32)6.02E23f;
    const float32 CON=(float32)3.3429E21f;
    const float32 XLOSCH=(float32)2.6868E24f;
    const float32 RH0=(float32)(273.15f/296.0f);
    const float32 CONJO = (float32)3.7194E-21f;

    if(patm->m_icld>=1.0f)
    {
        int32 cnum=0;
        while (patm->AtmodDir[cnum]!='\0')
        {
            str1[cnum]=patm->AtmodDir[cnum];
            cnum++;
        }
        str1[cnum]='c';
        str1[cnum+1]='l';
        str1[cnum+2]='d';
        str1[cnum+3]='e';
        str1[cnum+4]='x';
        str1[cnum+5]='t';
        str1[cnum+6]='.';
        str1[cnum+7]='d';
        str1[cnum+8]='a';
        str1[cnum+9]='t';
        str1[cnum+10]='\0';
        fp = fopen(str1,"rt");//云模式消光系数
        if(fp==NULL)
        {
            patm->IsCal=false;
            patm->att=1;
            printf("Can not open the file cldext.dat");

            return -1;
        }
        for(j=1;j<=7;j++)
        {
            fscanf(fp,"%s",str1);
            for(i=1;i<=47;i++)
            {
                fscanf(fp,"%f",&extcld[j][i]);
            }
        }
        fclose(fp);

        cnum=0;
        while (patm->AtmodDir[cnum]!='\0')
        {
            str1[cnum]=patm->AtmodDir[cnum];
            cnum++;
        }
        str1[cnum]='c';
        str1[cnum+1]='l';
        str1[cnum+2]='d';
        str1[cnum+3]='r';
        str1[cnum+4]='r';
        str1[cnum+5]='.';
        str1[cnum+6]='d';
        str1[cnum+7]='a';
        str1[cnum+8]='t';
        str1[cnum+9]='\0';
        fp = fopen(str1,"rt");

        if(fp==NULL)
        {
            patm->IsCal=false;
            patm->att=1;
            printf("Can not open the file cldrr.dat");

            return -1;
        }
        for(j=1;j<=5;j++)
            for(i=1;i<=10;i++)
            {
                fscanf(fp,"%f",&cld[j][i]);
            }
            for(j=1;j<=5;j++)
                for(i=1;i<=11;i++)
                {
                    fscanf(fp,"%f",&rr[j][i]);
                }
                fclose(fp);

                cnum=0;
                while (patm->AtmodDir[cnum]!='\0')
                {
                    str1[cnum]=patm->AtmodDir[cnum];
                    cnum++;
                }
                str1[cnum]='w';
                str1[cnum+1]='l';
                str1[cnum+2]='1';
                str1[cnum+3]='.';
                str1[cnum+4]='d';
                str1[cnum+5]='a';
                str1[cnum+6]='t';
                str1[cnum+7]='\0';
                fp = fopen(str1,"rt");

                if(fp==NULL)
                {
                    patm->IsCal=false;
                    patm->att=1;
                    printf("Can not open the file WL1.dat ");
                    return -1;
                }
                for(i=1;i<=47;i++)
                    fscanf(fp,"%f",&wl1[i]);
                fclose(fp);

                if(patm->m_icld<=5)
                {
                    mc = patm->m_icld;
                    mr = 6;
                }
                else
                {//15
                    if(patm->m_icld==6)
                        mc = 3;
                    else if(patm->m_icld==7 || patm->m_icld==8)
                        mc =5;
                    else if(patm->m_icld>=8)
                        mc =1;
                    mr = patm->m_icld-5;
                }
                //14
                if(patm->m_calt>0 && patm->m_icld<11 && patm->m_cthick==0)
                {//use of cloud model with user input cloud altitude
                    th = zcld[fg[mc]];
                    for(i=1;i<=16;i++)
                        zcld[i] = zcld[i]-th+patm->m_calt;
                    calt1[mc] = patm->m_calt;
                    calt2[mc] = calt2[mc]+patm->m_calt;
                }
                switch(patm->m_icld)
                {
                case 1:
                    cona = CULWC;
                    break;
                case 2:
                    cona = ASLWC;
                    break;
                case 3:
                    cona = STLWC;
                    break;
                case 4:
                    cona = SCLWC;
                    break;
                case 5:
                    cona = SNLWC;
                    break;
                case 6:
                    cona = STLWC;
                    break;
                case 7:
                    cona = SNLWC;
                    break;
                case 8:
                    cona = SNLWC;
                    break;
                case 9:
                    cona = CULWC;
                    break;
                case 10:
                    cona = CULWC;
                    break;
                case 11:
                    cona = CULWC;
                    break;
                case 12:
                    cona = 1;
                    break;
                default:
                    cona = 1;
                    break;
                }
                if(patm->m_icld==12 && patm->m_cthick==0)
                    patm->m_cthick = 1;
                if(patm->m_icld==12 && patm->m_calt==0)
                {//for cirrus model
                    switch(patm->m_aero)
                    {
                    case 1:
                        patm->m_calt = 11;
                        break;
                    case 2:
                        patm->m_calt = 10;
                        break;
                    case 3:
                        patm->m_calt = 8;
                        break;
                    case 4:
                        patm->m_calt = 7;
                        break;
                    case 5:
                        patm->m_calt = 5;
                        break;
                    case 6:
                        patm->m_calt = 6;
                        break;
                    default:
                        break;
                    }
                }
                if(patm->m_icld==12)
                {
                    if(patm->m_rextcld==0)
                        patm->m_rextcld = 0.14f*patm->m_cthick;
                    patm->m_icld = 11;
                }

                wcld = 0;
                wrain = 0;
                kr1 = 400.0f;
                kt = kr1*(cos(patm->m_theta*PI/180.0f)+0.15f*pow((93.885f-patm->m_theta),(-1.253f)));
                ss = 0;
                for(i=patm->nmax;i<=patm->nmin;i++)
                {
                    tz = (z[i]-z[i+1])/kt;
                    dzs = (z[i]-z[i+1])/kr1;
                    if((z[i]-patm->m_gndalt)>6 && patm->m_icld!=12 && patm->m_icld!=11)
                        continue;
                    if((z[i]-patm->m_gndalt)<patm->m_calt && patm->m_rainrt==0 && (patm->m_icld<6 || patm->m_icld==11))
                        continue;
                    if(patm->m_icld==11)
                    {
                        if((z[i+1]-patm->m_gndalt)>(patm->m_calt+patm->m_cthick))
                            continue;
                    }
                    for(k=(int32)kr1-1;k>=0;k--)
                    {
                        zmdl = z[i+1]+k*dzs-patm->m_gndalt;
                        if(patm->m_himax==patm->m_himin)
                        {
                            tz = patm->m_dist/kr1;
                        }
                        //sub 12
                        zk = zmdl-patm->m_gndalt;
                        if (zk<=0)
                            zk = 0;
                        if (zmdl>6)
                            zk = zmdl;
                        if (zk<=CLDTP)
                        {
                            cldatz=0;
                            rratz=0;
                            if (zk<=10)
                                rratz = patm->m_rainrt;
                            if (mc>=1)
                            {
                                for (mk=1;mk<=15;mk++)
                                {
                                    if (zk>=zcld[mk+1])
                                        continue;
                                    if (zk<zcld[mk])
                                        continue;
                                    if (fabs(zk-zcld[mk])<DELZ)
                                    {
                                        cldatz=cld[mc][mk];
                                        rratz=rr[mr][mk];
                                        break;
                                    }
                                    else
                                    {
                                        zdif=zcld[mk+1]-zcld[mk];
                                        if (zdif<DELZ)
                                        {
                                            cldatz=cld[mc][mk];
                                            rratz=rr[mr][mk];
                                            break;
                                        }
                                        else
                                        {
                                            fac=(zcld[mk+1]-zk)/zdif;
                                            cldatz=cld[mc][mk+1]+fac*(cld[mc][mk]-cld[mc][mk+1]);
                                            rratz=rr[mr][mk+1]+fac*(rr[mr][mk]-rr[mr][mk+1]);
                                            break;
                                        }
                                    }
                                }
                            }
                            //29
                            cldamt=cldatz;
                            if (patm->m_rainrt>0)
                                rratz = patm->m_rainrt;
                            if (zk>calt2[mc])
                                rratz = 0;
                            rramt = rratz;
                        }
                        else
                        {//30
                            cldamt = 0.0;
                            rramt = 0.0;
                            cldatz = 0.0;
                            rratz = 0.0;
                        }

                        if (patm->m_icld==11)
                        {
                            if (zmdl>patm->m_calt && zmdl<(patm->m_calt+patm->m_cthick))
                                cldamt = 1;
                            else
                                cldamt = 0;
                        }
                        //17
                        wcld = wcld+cldamt*tz;
                        if (cldamt>0)
                            ss = ss+tz;
                        wrain=wrain+(float32)pow(rramt,0.63f*tz);
                    }//k for
                    if (patm->m_icld==11)
                        wrain=(float32)(pow(patm->m_rainrt,0.63f)*(z[patm->nmax]-z[patm->nmin])/kt*kr1);
                    if (patm->m_icld==11 && patm->m_himax==patm->m_himin)
                        wrain=(float32)pow(patm->m_rainrt,0.63f*patm->m_dist);
                }//i for
                //prinft("Wcloud:%f Wrain:%f range:%f",wcld,wrain,ss);
                wcld = wcld/cona;
                train = (float32)(exp(-0.365f*wrain));
                for (kl=1;kl<=NN0;kl++)
                {
                    wl=patm->m_wl1+(kl-1)*patm->m_wvstep;
                    if (patm->m_rextcld>0)
                    {
                        ta = patm->m_rextcld;
                        wcld = ss;
                    }
                    else
                    {
                        for (j=47;j>=1;j--)
                        {
                            if (wl>=wl1[j])
                                break;
                        }
                        if (j<1)
                            j=1;
                        //interpolate the extinction based on frequency
                        consta = (wl-wl1[j])/(wl1[j+1]-wl1[j]);
                        ta = consta*(extcld[mc][j+1]-extcld[mc][j])+extcld[mc][j];
                    }
                    //47
                    if (ta*wcld>8.0f)
                        tcld[kl] = 0;
                    else if (ta*wcld<1e-5f)
                        tcld[kl]=1;
                    else
                        tcld[kl]=(float32)(exp(-ta*wcld));

                    patm->Taero[kl]=patm->Taero[kl]*tcld[kl]*train;
                }
    }
    //138

    //20000 The molecular absorption /////////////////////////////////////////////////
    //char dt=""
    //l=dist*1000;
    z[0] = 120;
    n2 = 20;
    if(patm->m_himax==patm->m_himin)
        n2 = 1;

    h2oslf1 = 0;
    h2oslf2 = 0;
    h2ofrn = 0;

    if(patm->m_himax==patm->m_himin)
    {//calculate sea level transimittance
        if (patm->m_met=='Y' || patm->m_met=='y')
        {//use the input meteorological data
            pr[patm->nmax] = (float32)(patm->m_press*1013.4/760);
            pr[patm->nmax+1]=(float32)(patm->m_press*1013.4/760);
            tp[patm->nmax]=patm->m_temp+ATMOD_T0;
            tp[patm->nmax+1]=patm->m_temp+ATMOD_T0;
            es=(float32)(6.1078f*pow(10.0f,(7.45f*patm->m_temp/(235.0f+patm->m_temp)))*100.0f);//unit:pascal
            eh2o=(float32)(patm->m_rh*es/100.0f);
            denw[patm->nmax]=(float32)(eh2o*18.0f*1e-6f/(tp[patm->nmax]*8.314f));//unit:g/cm^3 using gas equation
            denw[patm->nmax+1]=denw[patm->nmax];
        }
    }
    //205
    //h2o self and foreign continum
    slf1 = 0;
    slf2 = 0;
    frn = 0;
    //for H2O
    for(i=1;i<=9;i++)
    {	wh2o[i] = 0;}
    //for CO2
    for(i=1;i<=8;i++)
    {	wco2[i] = 0;}
    for(i=1;i<=4;i++)
    {	wo3[i] = 0;}
    wn2 = 0;
    patm->m_wh2o0 = 0;
    wco = 0;
    wno = 0;
    wnh3 = 0;
    wso2 = 0;

    float32 ctk=2.0f;
    kr=(int32)(kr*ctk);
    for(i=patm->nmax;i<=patm->nmin;i++)
    {
        zzh1=z[i];
        zzh2=z[i+1];
        dzs=(z[i]-z[i+1])/kr;
        for(k=kr-1;k>=0;k--)
        {
            zh=z[i+1]+(k+0.5f)*dzs;
            zzh=zh;
            beta=beta1[(int32)(k/ctk)][i]-beta1[(int32)(k/ctk)+1][i];
            tz=dzs*cos(beta/2.0f)/fabs(cos(1.5f*beta+beta1[(int32)(k/ctk)][i]));
            if (patm->m_himax==patm->m_himin)
                tz=patm->m_dist/kr;
            d11=dena[i];
            d12=dena[i+1];
            dd=CalcuSub1(zzh1,zzh2,zzh,d11,d12);
            if (dd<=0)
                dd=1e-35f;
            ra = dd;
            d11=denw[i];
            d12=denw[i+1];
            dd=CalcuSub1(zzh1,zzh2,zzh,d11,d12);
            if (dd<=0)
                dd=1e-35f;
            rw=dd;
            d11=den03[i];
            d12=den03[i+1];
            dd=CalcuSub1(zzh1,zzh2,zzh,d11,d12);
            if (dd<=0)
                dd=1e-35f;
            ro3=dd;
            m1o3=0.6f*ro3/ra;
            d11=pr[i];
            d12=pr[i+1];
            dd=CalcuSub1(zzh1,zzh2,zzh,d11,d12);
            if (dd<=0)
                dd=1e-25f;
            PR = dd;
            TP = tp[i]+(tp[i+1]-tp[i])*(k+0.5f)/kr;
            if (TP<=0)
                TP=1e-25f;
            rp = PR/ATMOD_P0;
            rt = ATMOD_T0/TP;

            //h2o self dependance continum slf and foreign continum
            tslf = CON*rw*1e6f/XLOSCH;
            tfrn=rp*rt-tslf;
            slf1=slf1+tslf*tslf*tz;
            slf2=slf2+tslf*tslf*(296.0f-TP)/(296.0f-260.0f)*tz;
            frn=frn+tfrn*tslf*tz;

            //for h2o
            patm->m_wh2o0 = patm->m_wh2o0+rw*rp*rt*tz*1e5f;//unit:g/cm^2
            for (j=1;j<=9;j++)
                wh2o[j] = wh2o[j]+rw*pow(rp,ah2o[j])*pow(rt,bh2o[j])*tz*1e5f;//TO TZ KM TO CM
            conjoe = CONJO*6.02e23f*(ra/28.9f);//ra UNIT g/cm**3

            //for co2
            conco2 = conjoe*MCO2;
            for (j=1;j<=8;j++)
                wco2[j] = wco2[j]+conco2*pow(rp,aco2[j])*pow(rt,bco2[j])*tz;

            //for o3
            cono3 = 6.02e23f*1e6f*ro3*CONJO/48.0f;
            for (j=1;j<=4;j++)
                wo3[j] = wo3[j]+cono3*pow(rp,ao3[j])*pow(rt,bo3[j])*tz;

            //for ch4
            wch4 = wch4+conjoe*MCH4*pow(rp,0.7139f)*pow(rt,(-0.4185f))*tz;

            //for n2o
            conn2o = conjoe*MN2O;
            for (j=1;j<=2;j++)
                wn2o[j] = wn2o[j]+conn2o*pow(rp,an2o[j])*pow(rt,bn2o[j])*tz;

            //for trace gases
            wco = wco+conjoe*MCO*pow(rp,0.9267f)*pow(rt,0.1716f)*tz;
            wno = wno+conjoe*MNO*pow(rp,0.5265f)*pow(rt,(-0.4702f))*tz;
            wnh3 = wnh3+conjoe*MNH3*pow(rp,0.6968f)*pow(rt,(0.3377f))*tz;
            wso2 = wso2+conjoe*MSO2*pow(rp,0.2135f)*pow(rt,(0.0733f))*tz;
            wn2 = wn2+MN2*rp*rp*sqrt(rt)*tz;
        }//k for
    }//i for

    h2oslf1 = slf1;
    h2oslf2 = slf2;
    h2ofrn = frn;
    h2oslf1 = NDENS*h2oslf1*XLOSCH/RH0;
    h2oslf2 = NDENS*h2oslf2*XLOSCH/RH0;
    h2ofrn = NDENS*h2ofrn*XLOSCH/RH0;


    //p0=760;
    for(i=1;i<=NN0;i++)
    {
        nn=i;
        patm->WL[i]=patm->m_wl1+(i-1)*patm->m_wvstep;
        if (patm->WL[i]>=patm->m_wl2)
            break;
    }
    //208

    int32 iw=0,iw0=0,fl0=0,s=0;
    float32 wv,wl0c,cp;
    for (i=1;i<=nn;i++)
    {
        wv = patm->m_wl1+(i-1)*patm->m_wvstep;
        v = 10000/wv;
        for (j=1;j<=9;j++)
        {
            if (v>=bndwvl[j] && v<=bndwvh[j])
            {
                iw=j;
                break;
            }
        }
        if (j>9)
        {
            for (j=1;j<=8;j++)
            {
                if (v-bndwvh[j]<5 && bndwvl[j+1]-v<5)
                {
                    fl0=1;
                    iw=j;
                    break;
                }
            }
            if (j>8)
            {
                fl0=0;
                patm->TH2O[i]=1;
                continue;
            }
        }
        //611
        s=(int32)((v-bndwvl[iw])/5.0)+1;
        if (fl0==1)
            s=s-1;
        if (v>bndwvh[iw])
            s=s-2;
        wl0c = bndwvl[iw]+(s-1)*5.0f;
        if (v!=wl0c)
            cp=patm->ch2o[iw][s]+(v-wl0c)*(patm->ch2o[iw][s+1]-patm->ch2o[iw][s])/5.0f;
        else
            cp=patm->ch2o[iw][s];
        //613
        if (wh2o[iw]==0)
        {
            patm->TH2O[i]=1;
            continue;
        }
        ttem = log(wh2o[iw])/log(10.0f)+cp;
        ttem = ttem*ach2o[iw];
        if (ttem>1.0f)
        {
            patm->TH2O[i]=0;
            continue;
        }
        if (ttem<-6.0f)
            patm->TH2O[i]=1.0f;
        else
            patm->TH2O[i]=exp(-pow(10.0f,ttem));
    }

    //215 new model for the h2o continum
    float32 kk=930;

    //605 h2o self continum
    for (i=1;i<=nn;i++)
    {
        v=10000/patm->WL[i];

        //sub 602 subroutine of self continum
        int32 sp=10;
        float32 kv=(v-710)/sp+1;
        //float32 h2ot0=0.0f,h2ot1=0.0f,h2of=0.0f,vtem=0.0f,v1=0.0f,v2=0.0f,h2t0=0.0f;//,h2o0//未初始化
        float h2ot0,h2ot1,h2of,vtem,v1,v2,h2t0;//,h2o0
        if (kv==(int32)kv)
        {
            h2ot0=patm->slf296[(int32)kv];//???h2t0=patm->slf296[(int32)kv];
            h2ot1=patm->slf260[(int32)kv];
            h2of=patm->frh[(int32)kv];
        }
        else
        {
            k=(int32)kv;
            if (k==kk)
                k=k-1;
            v1=float(k*sp+710);
            v2=v1+sp;
            vtem=(v-v1)/sp;
            h2ot0=(patm->slf296[k+1]-patm->slf296[k])*vtem+patm->slf296[k];
            h2ot1=(patm->slf260[k+1]-patm->slf260[k])*vtem+patm->slf260[k];
            h2of=(patm->frh[k+1]-patm->frh[k])*vtem+patm->frh[k];
        }
        //604 correction from lowtran7
        a2=40000.0f;
        float32 xh2o=(float32)(1-0.2333f*(a2/((v-1050.0f)*(v-1050.0f)+a2)));
        h2ot0=h2ot0*xh2o;
        h2ot1=h2ot1*xh2o;
        //      TO CALCULATE H20 FAR WING CONTINUUM USING THE SUMS OF EXPONENTIALSFDG
        float32 y1=(float32)exp(log(1.025f*3.159e-8f)-(2.75e-4f)*v);
        float32 y2=(float32)exp(log(8.97e-6f)-(1.3e-3f)*v);
        float32 fdg=(float32)(1.0f/(1.0f/y1+1.0f/y2));

        float32 conh2o=h2ot0*h2oslf1+(h2ot1-h2ot0)*h2oslf2+(h2of+fdg)*h2ofrn;
        conh2o=conh2o*v;
        if (conh2o>9 || conh2o<0)
            patm->TH2OC[i]=0;
        else
            patm->TH2OC[i]=exp(-conh2o);
    }

    for (i=1;i<=133;i++)
        fscanf(fp,"%f",&patm->cn2[i]);
    for (i=1;i<=nn;i++)
    {
        wv=patm->m_wl1+(i-1)*patm->m_wvstep;
        v=10000.0f/wv;
        if (v<2080.0f)
        {
            patm->TN2[i]=1;
            continue;
        }
        if (v>2740.0f)
        {
            patm->TN2[i]=1;
            continue;
        }
        s=(int32)((v-2080.0f)/5.0f)+1;
        cp=patm->cn2[s];
        float32 tt=cp*wn2;
        if (tt<10e-6f)
        {
            patm->TN2[i]=1;
            continue;
        }
        if (tt>8.0f)
            patm->TN2[i]=0;
        else
            patm->TN2[i]=exp(-tt);
    }

    bndwvl[1]=425.0f;
    bndwvl[2]=840.0f;
    bndwvl[3]=1805.0f;
    bndwvl[4]=3070.0f;
    bndwvl[5]=3760.0f;
    bndwvl[6]=4530.0f;
    bndwvl[7]=5905.0f;
    bndwvl[8]=7395.0f;
    bndwvl[9]=8030.0f;
    bndwvl[10]=9340.0f;

    bndwvh[1]=835.0f;
    bndwvh[2]=1440.0f;
    bndwvh[3]=2855.0f;
    bndwvh[4]=3755.0f;
    bndwvh[5]=4065.0f;
    bndwvh[6]=5380.0f;
    bndwvh[7]=7025.0f;
    bndwvh[8]=7785.0f;
    bndwvh[9]=8335.0f;
    bndwvh[10]=9670.0f;

    for (i=1;i<=nn;i++)
    {
        wv=patm->m_wl1+(i-1)*patm->m_wvstep;
        v=10000/wv;
        patm->TCO2[i]=1;
        for (j=1;j<=10;j++)
        {
            if (v>=bndwvl[j] && v<=bndwvh[j])
            {
                iw=j;
                iw0=j;
                break;
            }
        }
        if (j>10)
        {
            for (j=1;j<=9;j++)
            {
                if (v-bndwvh[j]<5 && bndwvl[j+1]-v<5)
                {
                    fl0=1;
                    iw=j;
                    iw0=j;
                    break;
                }
            }
            if (j>9)
            {
                fl0=0;
                continue;
            }
        }
        //711
        if (iw==9 || iw==10)
            iw=8;
        s=(int32)((v-bndwvl[iw0])/5)+1;
        if (fl0==1)
            s=s-1;
        wl0c=bndwvl[iw0]+(s-1)*5;
        if (iw0==9)
            s=s+79;
        if (iw0==10)
            s=s+79+62;
        if (v!=wl0c)
            cp=patm->cco2[iw][s]+(v-wl0c)*(patm->cco2[iw][s+1]-patm->cco2[iw][s])/5;
        else
            cp=patm->cco2[iw][s];
        //713
        if (wco2[iw]!=0)
        {
            //			double tee;
            ttem=log(wco2[iw])/log(10.0f)+cp;
            ttem=ttem*acco2[iw0];
            if (ttem>1)
            {
                patm->TCO2[i]=0;
                continue;
            }

            if (ttem>=-6)
                patm->TCO2[i]=exp(-pow(10.0f,ttem));
            else
                patm->TCO2[i]=1;
        }
    }

    bndwvl[1]=515.0f;
    bndwvl[2]=1630.0f;
    bndwvl[3]=2670.0f;
    bndwvl[4]=2850.0f;

    bndwvh[1]=1275.0f;
    bndwvh[2]=2295.0f;
    bndwvh[3]=2845.0f;
    bndwvh[4]=3260.0f;

    for (i=1;i<=nn;i++)
    {
        wv=patm->m_wl1+(i-1)*patm->m_wvstep;
        v=10000.0f/wv;
        for (j=1;j<=4;j++)
        {
            if (v>=bndwvl[j] && v<=bndwvh[j])
            {
                iw=j;
                break;
            }
        }
        if (j>4)
        {
            for (j=1;j<=3;j++)
            {
                if (v-bndwvh[j]<5 && bndwvl[j+1]-v<5)
                {
                    fl0=1;
                    iw=j;
                    break;
                }
            }
            if (j>3)
            {
                fl0=0;
                patm->TO3[i]=1;
                continue;
            }
        }
        //911
        s=(int32)((v-bndwvl[iw])/5)+1;
        if (fl0==1)
            s=s-1;
        wl0c=bndwvl[iw]+(s-1)*5;
        if (v!=wl0c)
            cp=patm->co3[iw][s]+(v-wl0c)*(patm->co3[iw][s+1]-patm->co3[iw][s])/5;
        else
            cp=patm->co3[iw][s];
        //913
        if (wo3[iw]==0)
        {
            patm->TO3[i]=1;
            continue;
        }
        ttem=log(wo3[iw])/log(10.0f)+cp;
        ttem=ttem*aco3[iw];
        if (ttem>1.0f)
        {
            patm->TO3[i]=0;
            continue;
        }
        if (ttem>=-6.0f)
            patm->TO3[i]=exp(-pow(10.0f,ttem));
        else
            patm->TO3[i]=1;
    }

    //for ch4
    char * flch4;

    bndwvl[1]=1065.0f;
    bndwvl[2]=2345.0f;
    bndwvl[3]=4110.0f;
    bndwvl[4]=5865.0f;

    bndwvh[1]=1775.0f;
    bndwvh[2]=3230.0f;
    bndwvh[3]=4690.0f;
    bndwvh[4]=6135.0f;

    for (i=1;i<=nn;i++)
    {
        wv=patm->m_wl1+(i-1)*patm->m_wvstep;
        v=10000.0f/wv;
        for (j=1;j<=4;j++)
        {
            if (v>=bndwvl[j] && v<=bndwvh[j])
            {
                iw=j;
                break;
            }
        }
        if (j>4)
        {
            for (j=1;j<=3;j++)
            {
                if (v-bndwvh[j]<5 && bndwvl[j+1]-v<5)
                {
                    fl0=1;
                    iw=j;
                    break;
                }
            }
            if (j>3)
            {
                fl0=0;
                patm->TCH4[i]=1;
                continue;
            }
        }
        //301
        s=(int32)((v-bndwvl[iw])/5)+1;
        if (fl0==1)
            s=s-1;
        wl0c=bndwvl[iw]+(s-1)*5;
        if (iw==2)
            s=s+143;
        if (iw==3)
            s=s+143+178;
        if (iw==4)
            s=s+143+178+117;
        if (v!=wl0c)
            cp=patm->cch4[s]+(v-wl0c)*(patm->cch4[s+1]-patm->cch4[s])/5;
        else
            cp=patm->cch4[s];
        //1303
        if (wch4==0)
        {
            patm->TCH4[i]=1;
            continue;
        }
        ttem=log(wch4)/log(10.0f)+cp;
        ttem=ttem*acch4[iw];
        if (ttem>1)
        {
            patm->TCH4[i]=0;
            continue;
        }
        if (ttem>=-6)
            patm->TCH4[i]=exp(-pow(10.0f,ttem));
        else
            patm->TCH4[i]=1;
    }

    // for n2o
    char * flco;
    bndwvl[1]=490;
    bndwvl[2]=865;
    bndwvl[3]=1065;
    bndwvl[4]=1545;
    bndwvl[5]=2090;
    bndwvl[6]=2705;
    bndwvl[7]=3245;
    bndwvl[8]=4260;
    bndwvl[9]=4540;
    bndwvl[10]=4910;

    bndwvh[1]=775;
    bndwvh[2]=995;
    bndwvh[3]=1385;
    bndwvh[4]=2040;
    bndwvh[5]=2655;
    bndwvh[6]=2865;
    bndwvh[7]=3925;
    bndwvh[8]=4470;
    bndwvh[9]=4785;
    bndwvh[10]=5165;

    for (i=1;i<=nn;i++)
    {
        wv=patm->m_wl1+(i-1)*patm->m_wvstep;
        v=10000/wv;
        for (j=1;j<=10;j++)
        {
            if (v>=bndwvl[j] && v<=bndwvh[j])
            {
                iw=j;
                iw0=j;
                break;
            }
        }
        if (j>10)
        {
            for (j=1;j<=9;j++)
            {
                if (v-bndwvh[j]<5 && bndwvl[j+1]-v<5)
                {
                    fl0=1;
                    iw=j;
                    iw0=j;
                    break;
                }
            }
            if (j>9)
            {
                fl0=0;
                patm->TN2O[i]=1;
                continue;
            }
        }
        //321
        if (iw>5)
            iw=2;
        else
            iw=1;
        s=(int32)((v-bndwvl[iw0])/5)+1;
        if (fl0==1)
            s=s-1;
        wl0c=bndwvl[iw0]+(s-1)*5;
        switch (iw0)
        {
        case 2:
            s=s+58;
            break;
        case 3:
            s=s+58+27;
            break;
        case 4:
            s=s+58+27+65;
            break;
        case 5:
            s=s+58+27+65+100;
            break;
        case 7:
            s=s+33;
            break;
        case 8:
            s=s+33+137;
            break;
        case 9:
            s=s+33+137+43;
            break;
        case 10:
            s=s+33+137+43+50;
            break;
        default:
            break;
        }
        if (v!=wl0c)
            cp=patm->cn2o[iw][s]+(v-wl0c)*(patm->cn2o[iw][s+1]-patm->cn2o[iw][s])/5;
        else
            cp=patm->cn2o[iw][s];

        //323
        if (wn2o[iw]==0)
        {
            patm->TN2O[i]=1;
            continue;
        }
        ttem=log(wn2o[iw])/log(10.0f)+cp;
        ttem=ttem*acn2o[iw0];
        if (ttem>1)
        {
            patm->TN2O[i]=0;
            continue;
        }
        if (ttem>=-6)
            patm->TN2O[i]=exp(-pow(10.0f,ttem));
        else
            patm->TN2O[i]=1;
    }

    //For trace gases(co,nh3,no,so2)
    char * trc,flnh3,flno,flso2;

    int32 bcowl[3]={0,1940,4040};
    int32 bcowh[3]={0,2285,4370};
    int32 bnh3wl=390;
    int32 bnh3wh=2150;
    int32 bnowl=1700;
    int32 bnowh=2005;
    int32 bsowl[3]={0,950,2415};
    int32 bsowh[3]={0,1460,2580};

    float32 acco=0.6133f;
    float32 acnh3=0.6035f;
    float32 acno=0.6613f;
    float32 acso2=0.8466f;

    for (i=1;i<=nn;i++)
    {
        patm->TRC[i]=1;
        wv=patm->m_wl1+(i-1)*patm->m_wvstep;
        v=10000/wv;
        //for co
        float32 ttco;
        if ((v>=bcowl[1] && v<=bcowh[1]) || (v>=bcowl[2] && v<=bcowh[2]))
        {
            if (v>=bcowl[1] && v<=bcowh[1])
                iw=1;
            else
                iw=2;
            //811
            s=(int32)((v-bcowl[iw])/5)+1;
            wl0c=float32(bcowl[iw]+(s-1)*5);
            if (iw==2)
                s=s+70;
            if (v==wl0c)
                cp=patm->cco[s];
            else
                cp=patm->cco[s]+(v-wl0c)*(patm->cco[s+1]-patm->cco[s])/5;
            //813
            if (wco!=0)
            {
                ttem=log(wco)/log(10.0f)+cp;
                ttem=ttem*acco;
                if (ttem<=1)
                {
                    if (ttem>=-6)
                        ttco=exp(-pow(10.0f,ttem));
                    else
                        ttco=1;
                }
                else
                    ttco=0;
            }
            else
                ttco=1;
        }
        else
            ttco=1;
        //851
        patm->TRC[i]=patm->TRC[i]*ttco;

        //for nh3
        float32 ttnh3;
        if (v>=bnh3wl && v<=bnh3wh)
        {
            //821
            s=(int32)((v-bnh3wl)/5)+1;
            wl0c=float32(bnh3wl+(s-1)*5);
            if (v==wl0c)
                cp=patm->cnh3[s];
            else
                cp=patm->cnh3[s]+(v-wl0c)*(patm->cnh3[s+1]-patm->cnh3[s])/5;
            if (wnh3!=0)
            {
                ttem=log(wnh3)/log(10.0f)+cp;
                ttem=ttem*acnh3;
                if (ttem<=1)
                {
                    if (ttem>=-6)
                        ttnh3=exp(-pow(10.0f,ttem));
                    else
                        ttnh3=1;
                }
                else
                    ttnh3=0;
            }
            else
                ttnh3=1;
        }
        else
            ttnh3=1;
        //853
        patm->TRC[i]=patm->TRC[i]*ttnh3;

        //for NO
        float32 ttno;
        if (v>bnowl && v<bnowh)
        {
            //823

            s=(int32)((v-bnowl)/5)+1;
            wl0c=float32(bnowl+(s-1)*5);
            if (v==wl0c)
                cp=patm->cno[s];
            else
                cp=patm->cno[s]+(v-wl0c)*(patm->cno[s+1]-patm->cno[s])/5;
            if (wno!=0)
            {
                ttem=log(wno)/log(10.0f)+cp;
                ttem=ttem*acno;
                if (ttem<=1.0f)
                {
                    if (ttem>=-6.0f)
                        ttno=exp(-pow(10.0f,ttem));
                    else
                        ttno=1;
                }
                else
                    ttno=0;
            }
            else
                ttno=1;
        }
        else
            ttno=1;
        //854
        patm->TRC[i]=patm->TRC[i]*ttno;

        //for SO2
        float32 ttso2;
        if ((v<bsowl[1] || v>bsowh[1]) && (v<bsowl[2] || v>bsowh[2]))
            ttso2=1;
        else
        {
            if (v>=bsowl[1] && v<=bsowh[1])
                iw=1;
            else
                iw=2;
            //817
            s=(int32)((v-bsowl[iw])/5)+1;
            wl0c=float32(bsowl[iw]+(s-1)*5);
            if (iw==2)
                s=s+103;
            if (v==wl0c)
                cp=patm->cso2[s];
            else
                cp=patm->cso2[s]+(v-wl0c)*(patm->cso2[s+1]-patm->cso2[s])/5;
            //819
            if (wso2!=0)
            {
                ttem=log(wso2)/log(10.0f)+cp;
                ttem=ttem*acso2;
                if (ttem<=1.0f)
                {
                    if (ttem>=-6.0f)
                        ttso2=exp(-pow(10.0f,ttem));
                    else
                        ttso2=1;
                }
                else
                    ttso2=0;
            }
            else
                ttso2=1;
        }
        //857
        patm->TRC[i]=patm->TRC[i]*ttso2;
    }// i for

    return nn;
}

float32 Atmod::CalcuSub1(float32 zzh1, float32 zzh2, float32 zzh, float32 d11, float32 d12)
{
    float32 db, da, dd;

    if (d11==d12)
    {
        dd = d11;
        return dd;
    }
    if(zzh1==zzh2)
    {
        dd = d11;
        return dd;
    }
    if(d11<=0)
        d11 = 1e-20f;
    if(d12<=0)
        d12 = 1e-20f;
    db = (zzh2-zzh1)/log(d11/d12);
    da = d11*exp(zzh1/db);
    dd = da*exp(-zzh/db);
    return dd;

}
//linar insertion
float32 Atmod::CalcuSub2(float32 zzh1, float32 zzh2, float32 zzh, float32 d11, float32 d12)
{
    float32 temp=0,dd;

    if(d11==d12)
    {
        dd = d11;
        return dd;
    }
    if(zzh1==zzh2)
    {
        dd = d11;
        return dd;
    }
    temp = (zzh-zzh1)/(zzh1-zzh2);
    dd = d11+(d11-d12)*temp;
    return dd;
}
//to cal the water density from Rh(%) and t(C)
float32 Atmod::CalcuSub3(float32 rh, float32 t)
{
    float32 dw;
    float32 tp0=0,es=0,eh20=0;

    tp0 = t+ATMOD_T0;
    es = 6.1078f * pow(10.0f,(7.45f*t/(235.0f+t))) * 100.0f;//uint:pascal
    eh20 = rh * es /100.0f;
    dw = eh20*18.0f/(tp0*8.314f);//uint:g/cm**3 using gas equation

    return dw;

}
//to cal the RH(%) from water density denw(g/m**3) and t(C)
float32 Atmod::CalcuSub4(float32 denw, float32 t)
{
    float32 es,eh20,tp0;
    float32 rh0;

    tp0 = t+ATMOD_T0;
    es = 6.1078f * pow(10.0f,(7.45f*t/(235.0f+t))) *100.0f;
    eh20 = denw * tp0 * 8.314f / 18.0f;
    rh0 = eh20*100.0f/es;

    return rh0;
}
