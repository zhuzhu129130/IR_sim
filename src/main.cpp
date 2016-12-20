//--------------------------------------------------------------------------------------
//Data:    		20160901
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

#include <stdio.h>
#include <time.h>
#include <iostream>
#include "pthread.h"
#include <time.h>

#include "paraReader.h"
#include "DVFileStore.h"
#include "IRImg_AR.h"

#include "./stereo/basefunc/basefunc.h"
#include "./stereo/calib/stereocalib.h"
#include "./stereo/rectify/stereorectify.h"
#include "./stereo/reconstruct/stereoreconstruction.h"

using namespace std;

paraReader * parareader = new paraReader();
DVFileStore * dvfilestore = new DVFileStore();
IRImg_AR * irimg_ar = new IRImg_AR();

int th1=0;
int th2=0;

void *thread_readImg(void *arg)
{
    while(1)
    {
        if(readImg_mutex)
        {
        th1++;
        render_mutex = 0;
        dvfilestore->ReadIRimg();
        render_mutex = 1;
        readImg_mutex = 0;
        //cout <<"th1 = "<< th1 <<endl;
        }
    }
}

void *thread_readImg1(void *arg)
{
    while(1)
    {
        if(readImg_mutex1)
        {
        //th1++;
        dvfilestore->ReadIRimg_usb();
        readImg_mutex1 = 0;
        //cout <<"th1 = "<< th1 <<endl;
        }
    }
}


int main()
{
    int32 framecount=0;
    float32 timeDelta=0.0f;
    IRScn.ScnStopTime=0.0f;

    if(!parareader->DefScenePara("../para/IRSim_sw.txt")) //../para/IRSim6ScnConfig.txt
    {
        return false;
    }

    dvfilestore->DVFileStoreBegin(DVFileName);
    while(1)
    {
        dvfilestore->ReadIRUSB();
    }

#if ADD_IR_IMG
   int ret=pthread_create(&pthread_id,NULL,thread_readImg,NULL);
    if(ret)
    {
        cout << "Create thread failed!" <<endl;
        return false;
    }

    ret=pthread_create(&pthread_id1,NULL,thread_readImg1,NULL);
    if(ret)
    {
        cout << "Create thread failed!" <<endl;
        return false;
    }
    readImg_mutex = 1;
    readImg_mutex1 = 1;
    render_mutex = 0;
#else
    render_mutex = 1;
#endif
    double t=cv::getTickCount();


    while(timeDelta <= IRScn.ScnStopTime)
    {
        if(render_mutex)
        {
#if ADD_IR_IMG
        readImg_mutex = 0;
        readImg_mutex1 = 0;
        //memcpy(bkg.bkgBuf,buf_recv,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)+64);
        for (int i = 0; i < cam.cpara.camDet.wid * cam.cpara.camDet.hei; i++)
        {
#if ADD_IR_CAMERA
            bkg.bkgBuf[i] = buf_recv[cam.cpara.camDet.wid * cam.cpara.camDet.hei - i - 1];

#else
            bkg.bkgBuf[i] = buf_recv[i];            
#endif
        }
        readImg_mutex = 1;
        readImg_mutex1 = 1;
        render_mutex = 0;
#endif
        th2++;
        
        irimg_ar->ScnRender(timeDelta); //光栅化进行渲染
        //irimg_ar->RayRender(timeDelta); //光线追迹法进行渲染

        //cout << "th2 = " << th2 << endl;

        dvfilestore->DVFileStoreSeq();

        //timeDelta += 10/cam.cpara.camOpt.frmfrq;
        framecount ++;

        cout << "framecount is " << framecount << "now!" << endl;//每frame显示一下，总共影响了0.5fps的速度

        }

    }
    t = cv::getTickCount() - t;
    cout<<"Time elapsed: "<< (t/cv::getTickFrequency() )<<endl;

    irimg_ar->CleanScnBuffer();

    dvfilestore->DVFileStoreEnd();

    cin.get();
    delete parareader;
    delete dvfilestore;
    delete irimg_ar;

    return true;
}


