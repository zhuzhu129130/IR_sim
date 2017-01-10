//--------------------------------------------------------------------------------------
//Data:    		20160901
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

#ifndef DVFILESTORE_H
#define DVFILESTORE_H

#include "DataType.h"
#include "Variable.h"
#include <string.h>
#include <stdlib.h>
#include "opencv2/opencv.hpp"
#include "./readIR/ftcamif.h"

#include "./readIR/ftd2xx.h"
#include "./readIR/ftcamif.h"

//============立体视觉========================
#include "./stereo/Pose2Dto2D.h"
#include "./stereo/basefunc/basefunc.h"
#include "./stereo/calib/stereocalib.h"
#include "./stereo/rectify/stereorectify.h"
#include "./stereo/reconstruct/stereoreconstruction.h"

//===========网络接收红外图像==================
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<stdlib.h>
#include<memory.h>
#include<string.h>
#include<unistd.h>
#include<sys/time.h>

//============中波640*512图像====================
#define FRAME_PXCNT		640*512	// 640*512
#define PORT 2001
//=============网络接收红外图像================

//============短波320*256图像====================
#define FRAME_SIZE (320 * 256)
#define FRAME_WIDTH		320
#define FRAME_HEIGHT	256
//============短波320*256图像====================

#ifndef MAKEFOURCC
#define MAKEFOURCC(ch0, ch1, ch2, ch3)                              \
    ((uint32)(char)(ch0) | ((uint32)(char)(ch1) << 8) |   \
    ((uint32)(char)(ch2) << 16) | ((uint32)(char)(ch3) << 24 ))
#endif //defined(MAKEFOURCC)

//bright contrast
void myAutoContrast(int,void*);//对比度滑动条响应函数
void myAutoBright(int,void*);//对比度滑动条响应函数



class DVFileStore
{
public:
    DVFileStore();

    void DVFileStoreBegin(char argv[]);//创建IRDV文件且写入文件头

    //===========网络接收红外图像==================
    void ReadIRimg_init(IRBKG * pbkg);
    void ReadIRimg_init1();
    void ReadIRimg();
    void ReadIRimg_usb();
    //===========网络接收红外图像==================

    void DVFileStoreSeq(void);//逐frame写入IRDV文件图像

    void IRimg_Singlecorr(short data_d[],unsigned short buffer[],int width,int height);
    void myImage14to8(short data_d[],unsigned short buffer[],cv::Mat &image,int contrast,int bright,int width,int height);
    void myImage14toMat(unsigned short buffer[],cv::Mat &image,int contrast,int bright,int width,int height);
    void myImage14to8(unsigned short source[],unsigned char dst[],int contrast,int bright,int width,int height);

    void myread(int sock,unsigned char data_char[]);
    void mycorrection(int sock,short *data_d,int height,int width);

    void ReadIRUSB();
    void mycorrection_usb(short *data_d,int height,int width);
    void mycorrection_usb1(short *data_d,int height,int width);

    void DVFileStoreEnd(void);//结束关闭文件

    void ReadIRved(IRBKG * pbkg);//测试显示红外视频

    //=====================立体视觉===========================
    void readcolorimg(cv::Mat& img_left,cv::Mat& img_right);
    void processimg(cv::Mat& img_left,cv::Mat& img_right);

    void ProcessZbuf(unsigned short *buf_recv,cv::Mat& J);

    void detectNearObject(cv::Mat& image, cv::Mat& pointCloud, vector<ObjectInfo>& objectInfos);
    void imageDenoising( cv::Mat& img, int iters );
    void parseCandidates(cv::Mat& objects, cv::Mat& depthMap, vector<ObjectInfo>& objectInfos);
    void showObjectInfo(vector<ObjectInfo>& objectInfos, cv::Mat& outImage);

public:    
    cv::VideoWriter writer;
    cv::Mat I,J;
    int sock;
    struct sockaddr_in IR_addr_1;

    int len;
    unsigned char buf_send[5];

    //calcu frequency parameters

    int frequency_num;
    double frequency_t;

    FILE *file;

    short *data_d;//单点校正得到的结果
    short *data_d1;//单点校正得到的结果

    ftcamif *ftcam;
    ftcamif1 *ftcam1;
    cv::VideoCapture cap;
    cv::VideoCapture cap1;
    char image_name1[100];
    char image_name2[100];
    int j;

    cv::Mat img_left,img_right;
    cv::Mat imgGray;

    Pose2Dto2D* pose2dto2d;

    double m_ObjectWidth;
    double m_ObjectHeight;
    double m_ObjectDistance;
    double m_ObjectDisparity;

};

#endif // DVFILESTORE_H
