//--------------------------------------------------------------------------------------
//Data:    		20160901
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

#include "DVFileStore.h"

using namespace std;

DVFileStore::DVFileStore()
{
}

//========================立体视觉======================
stereoReconstruction *ss = new stereoReconstruction();
int startx, starty, endx, endy;
bool drawing = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 255, 255);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
    viewer->addCoordinateSystem ( 1.0 );
    viewer->initCameraParameters ();
    return viewer;
}

static void onMouse(int event, int x, int y, int flags, void*)
{
    if (event==CV_EVENT_LBUTTONDOWN)
    {
        drawing = true;
        startx = x; starty = y;
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        drawing = false;
        endx = x; endy = y;
        cv::Mat cropped = ss->disp8u(cv::Rect(startx, starty, abs(startx-endx), abs(starty-endy)));
        cout << "mean: " << mean(cropped)[0] <<"\n";
        cv::rectangle(ss->disp8u, cv::Point(startx, starty), cv::Point(x, y), cv::Scalar(0, 0, 255), 1);
        cv::imshow("disparity", ss->disp8u);
        ss->getDepth(startx, starty, endx, endy);
    }
}

void updateSgbmParameters(cv::StereoSGBM& sgbm,SgbmParams& sgbmParams)
{
    if(sgbmParams.P2 <= sgbmParams.P1)
    {
      sgbmParams.P2 = sgbmParams.P1 + 1;
      cout << "Warning: P2 too small! Using P2 = " << sgbmParams.P2 << "\n";
    }
    int residue = sgbmParams.number_of_disparities % 16;
    if(residue != 0)
    {
      sgbmParams.number_of_disparities += (16 - residue);
      std::cout << "Warning: number_of_disparities \% 16 != 0! Using number_of_disparities = " << sgbmParams.number_of_disparities << "\n";
    }
    sgbm.preFilterCap = sgbmParams.pre_filter_cap;
    sgbm.SADWindowSize = sgbmParams.sad_window_size;
    sgbm.P1 = sgbmParams.P1;
    sgbm.P2 = sgbmParams.P2;
    sgbm.minDisparity = sgbmParams.min_disparity;
    sgbm.numberOfDisparities = sgbmParams.number_of_disparities;
    sgbm.uniquenessRatio = sgbmParams.uniqueness_ratio;
    sgbm.speckleWindowSize = sgbmParams.speckle_window_size;
    sgbm.speckleRange = sgbmParams.speckle_range;
    sgbm.disp12MaxDiff = sgbmParams.disp12_max_diff;
    sgbm.fullDP = sgbmParams.full_dp;
}

void on_trackbar(int, void*)
{
    if(runParams.DisparityType == "SGBM")
        updateSgbmParameters(ss->sgbm,ss->sgbmParams);
    //else if()
}

void createBarSgbm(SgbmParams& sgbmParams)
{
    cv::namedWindow("Parameters");
    cv::createTrackbar("preFilterCap", "Parameters", &sgbmParams.pre_filter_cap, sgbmParams.pre_filter_cap_max, on_trackbar);
    cv::createTrackbar("SADWindowSize", "Parameters", &sgbmParams.sad_window_size, sgbmParams.sad_window_size_max, on_trackbar);
    cv::createTrackbar("P1", "Parameters", &sgbmParams.P1, sgbmParams.P1_max, on_trackbar);
    cv::createTrackbar("P2", "Parameters", &sgbmParams.P2, sgbmParams.P2_max, on_trackbar);
    cv::createTrackbar("minDisparity", "Parameters", &sgbmParams.min_disparity, sgbmParams.min_disparity_max, on_trackbar);
    cv::createTrackbar("numberOfDisparities", "Parameters", &sgbmParams.number_of_disparities, sgbmParams.number_of_disparities_max, on_trackbar);
    cv::createTrackbar("uniquenessRatio", "Parameters", &sgbmParams.uniqueness_ratio, sgbmParams.uniqueness_ratio_max, on_trackbar);
    cv::createTrackbar("speckleWindowSize", "Parameters", &sgbmParams.speckle_window_size, sgbmParams.speckle_window_size_max, on_trackbar);
    cv::createTrackbar("speckleRange", "Parameters", &sgbmParams.speckle_range, sgbmParams.speckle_range_max, on_trackbar);
    cv::createTrackbar("disp12MaxDiff", "Parameters", &sgbmParams.disp12_max_diff, sgbmParams.disp12_max_diff_max, on_trackbar);
    cv::createTrackbar("fullDP", "Parameters", &sgbmParams.full_dp, sgbmParams.full_dp_max, on_trackbar);
    cv::imshow("Parameters", cv::Mat::zeros(1, 1400, CV_8U));
}

//========================立体视觉=======================

//----------------------------创建IRDV文件且写入文件头---------------------------------------
void DVFileStore::DVFileStoreBegin(char argv[])
{//创建IRDV文件且写入文件头
    int32 i;

    IRScn.ScnStopTime = pTgt[IRScn.AimTgtNo].TrajTgtStoptime;

    DVFp = fopen(argv,"wb");
    if(DVFp== NULL)
    {
        return;
    }

    dvInfo.fccType= MAKEFOURCC('I','R','D','V');
    dvInfo.flags  = 0;
    dvInfo.width  = cam.cpara.camDet.wid;
    dvInfo.height = cam.cpara.camDet.hei;
    dvInfo.bitCount= 16;//32;//IRDV似乎只支持到16位，不支持32位
    dvInfo.imgSize = (dvInfo.width)*(dvInfo.height)*2/*sizeof(uint32)*/+ sizeof(IRDV_FMINFO); //这里的2应该要改为sizeof(short)
    dvInfo.length  = (uint32)(IRScn.ScnStopTime * cam.cpara.camOpt.frmfrq)+1; // +1

    if (dvInfo.length == 0)
    {
        printf("dv length is zero\n");
        return ;
    }
    dvInfo.rate =cam.cpara.camOpt.frmfrq;
    dvInfo.xRes = (cam.cpara.camDet.utwid)/(cam.cpara.camOpt.foc); //探测器单元宽度尺寸(um)除以焦距(mm)，单位：mrad
    dvInfo.yRes = (cam.cpara.camDet.uthei)/(cam.cpara.camOpt.foc); //探测器单元高度尺寸(um)除以焦距(mm)，单位：mrad

    for(i=0;i<16;i++) dvInfo.imgParam[i]= 0.0;
    strcpy(dvInfo.name, "IRSim6 DV");
    strcpy(dvInfo.calFile, "IRSim6.cal");

    fwrite(&dvInfo, sizeof(IRDVFILEHEADER),1,DVFp);

    m_nContrast=5;//5;
    m_nBright=110;
    contrast_max=50;
    bright_max=255;

    cv::namedWindow("IR_AR_img", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar( "contrast:","IR_AR_img", &m_nContrast,contrast_max,myAutoContrast );
    cv::createTrackbar( "bright:","IR_AR_img", &m_nBright,bright_max,myAutoBright );

    I.create(cam.cpara.camDet.hei,cam.cpara.camDet.wid,CV_8UC3);
    J.create(cam.cpara.camDet.hei,cam.cpara.camDet.wid,CV_8UC3);

    bool isColor = (I.type() == CV_8UC3);
    writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, cv::Size(640, 512),isColor);
    if (!writer.isOpened()) {
        cerr << "Could not open the output video file for write\n";
    }

    m_nContrast1=5;
    m_nBright1=110;
    cv::namedWindow("IR_AR_imgl", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar( "contrast:","IR_AR_imgl", &m_nContrast1,contrast_max,myAutoContrast );
    cv::createTrackbar( "bright:","IR_AR_imgl", &m_nBright1,bright_max,myAutoBright );

#if ADD_IR_IMG
    ftcam = new ftcamif();
    ftcam1 = new ftcamif1();
    //ReadIRimg_init(&bkg);
    //ReadIRimg_init1();
#endif //ADD_IR_IMG

    //========================立体视觉===================
    if(runParams.processType == "calib")
    {
        stereoCalib * sc = new stereoCalib();
        sc->calib_process(runParams);
        delete sc;
    }

    if(runParams.processType == "rectify")
    {
        stereoRectifyy  *sr = new stereoRectifyy();
        sr->rectify(runParams);
        cv::Mat imgleft = cv::imread(runParams.image1_test,1);
        cv::Mat imgright = cv::imread(runParams.image2_test,1);
        cv::imshow("l",imgleft);
        cv::imshow("r",imgright);
        cv::waitKey(1);
        sr->remapImage(imgleft, imgright, runParams.rectifymethod);

        /*cv::Mat imgleft = cv::imread(runParams.image1_test,1);
        cv::Mat imgright = cv::imread(runParams.image2_test,1);
        sr->UnCalibRectify(imgleft,imgright,runParams);
        sr->remapImage(imgleft, imgright, "RECTIFY_HARTLEY");*/
        delete sr;
    }

    if(runParams.processType == "reconstruct")
    {
        ss->loadRectifyDatas(runParams.rectifyParams_path);

       cv::namedWindow("disparity", CV_WINDOW_AUTOSIZE);
       cv::setMouseCallback("disparity", onMouse);

        viewer = createVisualizer( point_cloud_ptr );

        updateSgbmParameters(ss->sgbm,ss->sgbmParams);
        createBarSgbm(ss->sgbmParams);
        cv::Mat img_left = cv::imread(runParams.image1_test,1);
        cv::Mat img_right = cv::imread(runParams.image2_test,1);
        //ss->remapImage(img_left, img_right, ss->remapMat,runParams.rectifymethod);
        //while(1)
        //{
         //   processimg(img_left,img_right);
       // }
    }
    pose2dto2d = new Pose2Dto2D();

}

//================TCP/IP网络传输初始化===================
void DVFileStore::ReadIRimg_init(IRBKG * pbkg)
{
    buf_recv=(unsigned short*)malloc(cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)+64);//+64
    if(buf_recv == NULL)
    {
        exit(1);
    }
    memset(buf_recv,0,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(unsigned short)); //清理灰度缓冲为固定值

    pbkg->bkgBuf = (unsigned short *)malloc(cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(unsigned short)+64);//设置红外图像的缓存空间
    if (pbkg->bkgBuf == NULL)
    {
        exit(1);
    }
    
#if ADD_IR_CAMERA
    if((sock=socket(AF_INET,SOCK_STREAM,0))<0)
    {
        printf("socket create error!\n");
        exit(1);
    }
    IR_addr_1.sin_family=AF_INET;
    IR_addr_1.sin_port=htons(PORT);
    IR_addr_1.sin_addr.s_addr = inet_addr("192.168.1.10");

    if(connect(sock,(struct sockaddr*)&IR_addr_1,sizeof(struct sockaddr))<0)
    {
        printf("connect error!\n");
        exit(1);
    }
    printf("connected!\n");

#if SINGLE_CORRECT//单点校正
    //data_d=(short*)malloc(FRAME_PXCNT*sizeof(short));
    //mycorrection(sock,data_d,cam.cpara.camDet.hei,cam.cpara.camDet.wid);
#endif //SINGLE_CORRECT//单点校正

#elif ADD_IR_CAMERA_SW
    //system("(lsmod | grep \"ftdi_sio\") && (echo \"zhqr\" | sudo -S rmmod -f ftdi_sio)");
    //system("echo \"zhqr\" | sudo -S sh -c \"rmmod -f ftdi_sio\"");

    data_d=(short *)malloc(cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short));
    if (data_d == NULL)
    {
        exit(1);
    }

    int ret;
    ret = ftcam->opendev("CH2S01", strlen("CHES01"));
    if (ret != 0)
    {
        printf("opendev() error.\n");
    }

#if SINGLE_CORRECT//单点校正
    memset(data_d,0,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)); //清理灰度缓冲为固定值
    mycorrection_usb(data_d,cam.cpara.camDet.hei,cam.cpara.camDet.wid);
#endif
#elif ADD_COLOR_IMG
    cap.open(1);
    if(!cap.isOpened())
    {
        cout<<"Open cap0 failed!!!"<<endl;
        exit(1);
    }

#else
    file=fopen("../para/CH02521-1434-001.irv","r");
    if(file == NULL)
        cout << "File open failed!" << endl;

    fread(&dvInfo,1,sizeof(IRDVFILEHEADER),file);

#endif   //ADD_IR_CAMERA

}

void DVFileStore::ReadIRimg_init1()
{
#if ADD_IR_CAMERA_SW
    data_d1=(short *)malloc(cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short));
    if (data_d1 == NULL)
    {
        exit(1);
    }
    
    int ret = ftcam1->opendev("CH2S02", strlen("CHES02"));
    if (ret != 0)
    {
        printf("opendev() error.\n");
    }
    
    memset(data_d1,0,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)); //清理灰度缓冲为固定值
    mycorrection_usb1(data_d1,cam.cpara.camDet.hei,cam.cpara.camDet.wid);
#else
    /*cap1.open(1);
    if(!cap1.isOpened())
    {
        cout<<"Open cap1 failed!!!"<<endl;
        exit(1);
    }*/
#endif
    
}
//================TCP/IP网络传输初始化=====================

//====================网络接收红外图像======================
void DVFileStore::ReadIRimg()
{
#if ADD_IR_CAMERA
    buf_send[0]='r';
    len=write(sock,buf_send,5);
    if(len<0)
    {
        printf("send error!\n");
        exit(1);
    }
    //len=recv(sock,bkg.bkgBuf,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)+4,MSG_WAITALL);
    len=recv(sock,buf_recv,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)+64,MSG_WAITALL);
#if SINGLE_CORRECT //单点校正
    //IRimg_Singlecorr(data_d,buf_recv,dvInfo.width,dvInfo.height);//有单点校正
#endif  //单点校正

#elif ADD_IR_CAMERA_SW
    int ret = ftcam->getFrame((char *)&ftcam_frame, (cam.cpara.camDet.hei*cam.cpara.camDet.wid+5)*sizeof(short), 256);
    //ret = ftcam->getFrame((char *)&ftcam_frame, (cam.cpara.camDet.hei*cam.cpara.camDet.wid+5)*sizeof(short), 256);
    if (ret != 0)
    {
        printf("not all data recieved.\n");
        exit(1);
    }
    //memcpy(buf_recv,&ftcam_frame.frameBuffer,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(unsigned short));
#if SINGLE_CORRECT //单点校正
    IRimg_Singlecorr(data_d,ftcam_frame.frameBuffer,cam.cpara.camDet.wid,cam.cpara.camDet.hei);//有单点校正
#endif  //单点校正
    cv::Mat S0 = cv::Mat(cam.cpara.camDet.hei,cam.cpara.camDet.wid,CV_16UC1,&ftcam_frame.frameBuffer);
    cv::Mat D0 = cv::Mat(cam.cpara.camDet.hei,cam.cpara.camDet.wid,CV_16UC1,buf_recv);
    cv::medianBlur(S0,D0,3);

#elif ADD_COLOR_IMG
    cap.read(J);
    cap.read(J);
    cap.read(J);
    cap.read(J);
    cap.read(J);
    imgGray = J.clone();
    if(!img_left.empty())
    {
        pose2dto2d->computRT(img_left,J);
        //MATRIX16 RT =
    }

    img_left = J.clone();

    cv::cvtColor(J,imgGray,CV_BGR2GRAY);
    for (int y=0;y<imgGray.rows;y++)
    {
        uchar *output=imgGray.ptr<uchar>(y);
        for (int x=0;x<imgGray.cols;x++)
        {
            buf_recv[y*imgGray.cols + x] = (unsigned short)(*output++);
        }
    }
#else
    //fread(bkg.bkgBuf,1,dvInfo.imgSize,file);
    fread(buf_recv,1,dvInfo.imgSize,file);
#endif  //ADD_IR_CAMERA

    myImage14toMat(buf_recv,J,m_nContrast,m_nBright,cam.cpara.camDet.wid,cam.cpara.camDet.hei);

    /*cv::imshow("imgshow",I);
    cv::waitKey(1);
    */
}
//====================网络接收红外图像======================

//====================网络接收红外图像1======================
void DVFileStore::ReadIRimg_usb()
{
#if ADD_IR_CAMERA_SW
    int ret = ftcam1->getFrame((char *)&ftcam_frame1, (FRAME_WIDTH*FRAME_HEIGHT+5)*sizeof(short), 256);
    //ret = ftcam1->getFrame((char *)&ftcam_frame1, (FRAME_WIDTH*FRAME_HEIGHT+5)*sizeof(short), 256);
    if (ret != 0)
    {
        printf("not all data recieved.\n");
        exit(1);
    }
    //memcpy(buf_recv,&ftcam_frame.frameBuffer,FRAME_WIDTH*FRAME_HEIGHT*sizeof(unsigned short));
#if SINGLE_CORRECT //单点校正
    IRimg_Singlecorr(data_d1,ftcam_frame1.frameBuffer,FRAME_WIDTH,FRAME_HEIGHT);//有单点校正
#endif  //单点校正
    cv::Mat J1;
    J1 = J.clone();
    myImage14toMat((unsigned short*)ftcam_frame1.frameBuffer,J1,m_nContrast1,m_nBright1,FRAME_WIDTH,FRAME_HEIGHT);//无单点校正
    cv::medianBlur(J1,J,3);
//#else
    //fread(bkg.bkgBuf,1,dvInfo.imgSize,file);
//    fread(buf_recv,1,dvInfo.imgSize,file);

#elif ADD_COLOR_IMG
    /*readcolorimg(img_left,img_right);
    imgGray = img_left.clone();

    cv::cvtColor(img_left,imgGray,CV_BGR2GRAY);*/
    /*for (int y=0;y<imgGray.rows;y++)
    {
        uchar *output=imgGray.ptr<uchar>(y);
        for (int x=0;x<imgGray.cols;x++)
        {
            buf_recv[y*imgGray.cols + x] = (unsigned short)(*output++);
        }
    }*/

#endif  //ADD_IR_CAMERA

    /*myImage14toMat(pbkg->bkgBuf,I,m_nContrast,m_nBright,cam.cpara.camDet.wid,cam.cpara.camDet.hei);

    cv::imshow("imgshow",I);
    cv::waitKey(1);
    */
}
//====================网络接收红外图像1======================

//================USB短波传输初始化===================
void DVFileStore::ReadIRUSB()
{
    //system("(lsmod | grep \"ftdi_sio\") && (echo \"zhqr\" | sudo -S rmmod -f ftdi_sio)");
    //system("echo \"zhqr\" | sudo -S sh -c \"rmmod -f ftdi_sio\"");

    int ret;
    ret = ftcam->opendev("CH2S02", strlen("CHES01"));
    if (ret != 0)
    {
        printf("opendev() error.\n");
    }

    ret = ftcam1->opendev("CH2S01", strlen("CHES02"));
    if (ret != 0)
    {
        printf("opendev() error.\n");
    }

    //cv::namedWindow("InGaAs320x256r",CV_WINDOW_AUTOSIZE);
    //cv::namedWindow("InGaAs320x256l",CV_WINDOW_AUTOSIZE);

    cv::Mat J,K;
    J.create(FRAME_HEIGHT,FRAME_WIDTH,CV_8UC3);
    K.create(FRAME_HEIGHT,FRAME_WIDTH,CV_8UC3);

    //myAutoBright(0,0);
    //cv::createTrackbar( "contrast:","InGaAs320x256", &m_nContrast,contrast_max,myAutoContrast );
    //cv::createTrackbar( "bright:","InGaAs320x256", &m_nBright,bright_max,myAutoBright );

#if 1//单点校正
    short *data_d=(short*)malloc(FRAME_SIZE*sizeof(short));
    memset(data_d,0,FRAME_HEIGHT*FRAME_WIDTH*sizeof(short)); //清理灰度缓冲为固定值
    mycorrection_usb(data_d,FRAME_HEIGHT,FRAME_WIDTH);

    short *data_dl=(short*)malloc(FRAME_SIZE*sizeof(short));
    memset(data_dl,0,FRAME_HEIGHT*FRAME_WIDTH*sizeof(short)); //清理灰度缓冲为固定值
    mycorrection_usb1(data_dl,FRAME_HEIGHT,FRAME_WIDTH);
#endif

    while(1)
    {
        //memset(&ftcam_frame,0,(cam.cpara.camDet.hei*cam.cpara.camDet.wid+5)*sizeof(unsigned short)); //清理灰度缓冲为固定值
        ret = ftcam->getFrame((char *)&ftcam_frame, sizeof(FTCAM_FRAME), 255);
        ret = ftcam->getFrame((char *)&ftcam_frame, sizeof(FTCAM_FRAME), 255);
        if (ret != 0)
        {
            printf("not all data recieved.\n");
        }

        //memset(&ftcam_frame1,0,(cam.cpara.camDet.hei*cam.cpara.camDet.wid+5)*sizeof(unsigned short)); //清理灰度缓冲为固定值
        ret = ftcam1->getFrame((char *)&ftcam_frame1, sizeof(FTCAM_FRAME), 255);
        ret = ftcam1->getFrame((char *)&ftcam_frame1, sizeof(FTCAM_FRAME), 255);
        if (ret != 0)
        {
            printf("not all data recieved.\n");
        }

#if 1 //单点校正
        IRimg_Singlecorr(data_d,ftcam_frame.frameBuffer,FRAME_WIDTH,FRAME_HEIGHT);//有单点校正
        IRimg_Singlecorr(data_dl,ftcam_frame1.frameBuffer,FRAME_WIDTH,FRAME_HEIGHT);//有单点校正
#endif  //单点校正

         myImage14toMat((unsigned short*)ftcam_frame.frameBuffer,J,m_nContrast,m_nBright,FRAME_WIDTH,FRAME_HEIGHT);//无单点校正
         //myImage14to8(data_d,ftcam_frame.frameBuffer,J,contrast,bright,FRAME_WIDTH,FRAME_HEIGHT);//有单点校正

         myImage14toMat((unsigned short*)ftcam_frame1.frameBuffer,K,m_nContrast,m_nBright,FRAME_WIDTH,FRAME_HEIGHT);

         /*double globle_scale=2;
         cv::Mat M = J.clone();
         cv::resize(J,M,cv::Size(J.cols*globle_scale,J.rows*globle_scale));
         
         cv::Mat N = J.clone();
         cv::medianBlur(M,N,3);*/

         double globle_scale=2;
         cv::Mat M = J.clone();
         cv::Mat N = K.clone();
         cv::medianBlur(J,M,3);
         cv::medianBlur(K,N,3);
         //cv::Mat R = J.clone();
         //cv::Mat L = K.clone();
         //cv::resize(M,R,cv::Size(M.cols*globle_scale,M.rows*globle_scale));
         //cv::resize(N,L,cv::Size(N.cols*globle_scale,N.rows*globle_scale));

         cv::imshow("IR_AR_img",J);
         cv::imshow("IR_AR_imgl",K);

         //cv::waitKey(1);
         int k = cv::waitKey(2);
         if (k == 'q') { break; }
         else if(k == 's')
         {
             j++;
             sprintf(image_name1, "%s%d%s", "./left", j, ".jpg");//保存的图片名
             sprintf(image_name2, "%s%d%s", "./right", j, ".jpg");//保存的图片名
             cv::imwrite(image_name1,K);
             cv::imwrite(image_name2,J);
         }
         else if (k == 'c')
         {
     #if ADD_IR_CAMERA_SW
             memset(data_d,0,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)); //清理灰度缓冲为固定值
             mycorrection_usb(data_d,cam.cpara.camDet.hei,cam.cpara.camDet.wid);

             memset(data_d1,0,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)); //清理灰度缓冲为固定值
             mycorrection_usb1(data_d1,cam.cpara.camDet.hei,cam.cpara.camDet.wid);
     #endif
         }

         cout << "123" <<endl;

         //processimg(K,J);

         //ss->remapImage(K,J,ss->remapMat,"RECTIFY_BOUGUET");

    /* do something with data. */
         
    }

    ftcam->closedev();
    ftcam1->closedev();
    SAFE_FREE(data_d);
    SAFE_FREE(data_dl);

}
//================USB短波传输初始化=====================

//====================测试显示红外视频======================
void DVFileStore::ReadIRved(IRBKG * pbkg)
{
    pbkg->bkgBuf = (unsigned short *)malloc(cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(unsigned short)+4);//设置红外图像的缓存空间
    if (pbkg->bkgBuf == NULL)
    {
        exit(1);
    }

    file=fopen("../para/CH02521-1434-001.irv","r");
    if(file == NULL)
        cout << "File open failed!" << endl;

    fread(&dvInfo,1,sizeof(IRDVFILEHEADER),file);
    cout << "width = " << dvInfo.width <<endl;
    cout << "height = " << dvInfo.height <<endl;
    cout << "imgsize = " << dvInfo.imgSize <<endl;

    frequency_t=(double)cv::getTickCount();

    for(int i=0;i<dvInfo.length;i++)
    {
        fread(pbkg->bkgBuf,1,dvInfo.imgSize,file);//cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)
#if SINGLE_CORRECT //单点校正
        IRimg_Singlecorr(data_d,pbkg->bkgBuf,dvInfo.width,dvInfo.height);//有单点校正
#endif  //单点校正
        myImage14toMat(pbkg->bkgBuf,I,m_nContrast,m_nBright,dvInfo.width,dvInfo.height);//cam.cpara.camDet.wid,cam.cpara.camDet.hei
        cv::imshow("imgshow",I);
        cv::waitKey(1);

        if((frequency_num++)>=50)
        {
            double tmpT=((double)cv::getTickCount()-frequency_t)/cv::getTickFrequency();
            double frequency=frequency_num/tmpT;
            printf("frequency=%.2lf\n",frequency);
            frequency_num=0;
            frequency_t=(double)cv::getTickCount();
        }
    }
}
//====================测试显示红外视频======================

//-----------逐frame写入IRDV文件图像-----------------------------------------------------------
void DVFileStore::DVFileStoreSeq(void)
{//逐侦保存IRDV文件，先写入当前侦的图像，再写入IRDV_FMINFO信息

    IRDV_FMINFO fmInfo;
//	VECTOR3 cmLos_tmp;
//	float32 dd, az, el, bk;
    //short tmp[ cam.camDet.wid * cam.camDet.hei];
    unsigned short *tmp = (unsigned short *)malloc(sizeof(unsigned short)* cam.cpara.camDet.wid * cam.cpara.camDet.hei);
    if (tmp == NULL)
    {
        return ;
    }
    int32 i;

    for (i = 0; i < cam.cpara.camDet.wid * cam.cpara.camDet.hei; i++)
    {        
        //tmp[i] = buf_recv[cam.cpara.camDet.wid * cam.cpara.camDet.hei-i-1];//中波相机
        tmp[i] = (short) IRScn.Gray[i];
        //tmp[i] = buf_recv[i];//短波相机
    }
    fwrite(tmp, sizeof(short), cam.cpara.camDet.wid * cam.cpara.camDet.hei, DVFp);
    //fwrite(ImgGrayBuf, sizeof(uint32), CAMERA_HEI * CAMERA_WID, DVFp);
    /*
    Vec3Subtract(&cmLos_tmp, &TgtState.pos, &CamState.pos);//从归一化视线及模求回视线
    dd= sqrt(1.0f - (cmLos_tmp.z)*(cmLos_tmp.z));
    az= acos((cmLos_tmp.x)/dd);
    if(cmLos_tmp.y<0) az= 2*PI- az;
    el= acos(cmLos_tmp.z);
    bk= CamState.bnk;


    fmInfo.x= (int32) ((CamState.pos.x)*1e2);
    fmInfo.y= (int32) ((CamState.pos.y)*1e2);
    fmInfo.z= (int32) ((CamState.pos.z)*1e2);

    fmInfo.azm= (int32) (az*(1e12));
    fmInfo.elv= (int32) (el*(1e12));
    fmInfo.bnk= (int32) (bk*(1e12));*/

    fwrite(&fmInfo, sizeof(IRDV_FMINFO), 1, DVFp);

    myImage14toMat(tmp,I,m_nContrast,m_nBright,cam.cpara.camDet.wid,cam.cpara.camDet.hei);

    cv::imshow("IR_AR_img",I);

    cv::Mat J1 = I.clone();
    cv::medianBlur(J,J1,3);
    cv::imshow("IR_AR_imgl",J);
    //cv::waitKey(1);//auto
    writer.write(I);
    //cv::imshow("grayimg",imgGray);
    
    int k = cv::waitKey(1);//重新进行一点校正
    if (k == 'q') { }
    else if(k == 's')
    {
        j++;
        sprintf(image_name1, "%s%d%s", "./left", j, ".jpg");//保存的图片名
        sprintf(image_name2, "%s%d%s", "./right", j, ".jpg");//保存的图片名
        cv::imwrite(image_name1,I);
        cv::imwrite(image_name2,J);
    }
    else if (k == 'c')
    { 
#if ADD_IR_CAMERA_SW
        memset(data_d,0,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)); //清理灰度缓冲为固定值
        mycorrection_usb(data_d,cam.cpara.camDet.hei,cam.cpara.camDet.wid);
    
        memset(data_d1,0,cam.cpara.camDet.hei*cam.cpara.camDet.wid*sizeof(short)); //清理灰度缓冲为固定值
        mycorrection_usb1(data_d1,cam.cpara.camDet.hei,cam.cpara.camDet.wid);
#endif
    }   

    SAFE_FREE(tmp);
}

void DVFileStore::IRimg_Singlecorr(short data_d[],unsigned short buffer[],int width,int height)
{
    for ( int y=0;y<height;y++)
    {
        for ( int x=0;x<width;x++)
        {
            unsigned short tmp=0;
            //tmp=data_d[y*width+x];
            tmp=(unsigned short)buffer[y*width+x]/16+data_d[y*width+x];
            if(tmp<0)
                buffer[y*width+x]=0;
            else if(tmp>16383)
                buffer[y*width+x]=16383;
            else
                buffer[y*width+x]=tmp;
        }
    }
}

void DVFileStore::myImage14to8(short data_d[],unsigned short buffer[],cv::Mat &image,int contrast,int bright,int width,int height)
{
    for ( int y=0;y<height;y++)
    {
        for ( int x=0;x<width;x++)
        {
            unsigned short tmp=0;
            //tmp=data_d[y*width+x];
            tmp=(unsigned short)buffer[y*width+x]/16+data_d[y*width+x];
            if(tmp<0)
                buffer[y*width+x]=0;
            else if(tmp>16383)
                buffer[y*width+x]=16383;
            else
                buffer[y*width+x]=tmp;
        }
    }
    myImage14toMat(buffer,image,contrast,bright,width,height);
}

void DVFileStore::myImage14toMat(unsigned short buffer[],cv::Mat &image,int contrast,int bright,int width,int height)
{
    unsigned char *dst=(unsigned char*)malloc(height*width);
    myImage14to8(buffer,dst,contrast,bright,width,height);
    for (int y=0;y<height;y++)
    {
        uchar *output=image.ptr<uchar>(y);
        for (int x=0;x<width;x++)
        {
            unsigned char tmp=dst[y*width+x];
            *output++=tmp;
            *output++=tmp;
            *output++=tmp;
        }
    }
    SAFE_FREE(dst);
}

void DVFileStore::myImage14to8(unsigned short source[],unsigned char dst[],int contrast,int bright,int width,int height)
{
    if(contrast==0)
        contrast=1;
    double sum = 0;
    double mean;
    int t0 , t;
    for ( int y=0;y<height;y++ )
    {
        for ( int x=0;x<width;x++ )
        {
            t0= *(source+(y*width)+x );
            sum = sum + t0;
        }
    }
    mean = sum/(height*width);
    //printf("mean=%.2lf\n",mean);
    for (int y=0;y<height;y++)
    {
        for (int x=0;x<width;x++)
        {
            t0= *(source+(y*width)+x);
            //        t0 = ( t0 ) / contrast+bright;
            t0 = (t0-int(mean))/ contrast+bright;
            if (t0 > 255)
                t = 255;
            else if (t0 < 0)
                t = 0;
            else
                t = t0;
            dst[y*width+x]=t;
        }
    }
}

/*
 *flage 1:binary file  0:text file
 */
void DVFileStore::myread(int sock,unsigned char data_char[])
{
    int len=0;
    unsigned char buf_send[4];
    buf_send[0]='r';
    len=send(sock,buf_send,3,0);
    if(len<0){
        printf("send error!\n");
        exit(1);
    }else{
        //	printf("send succeed! send");
    }
    len=recv(sock,data_char,FRAME_PXCNT*2,MSG_WAITALL);
}
void DVFileStore::mycorrection(int sock,short *data_d,int height,int width)
{
    int img_num=5;
    int buf_sum[height*width];
    bzero(buf_sum,sizeof(buf_sum));
    unsigned short data[height*width];
    unsigned char data_char[height*width*2];
    int i,j;
    for(i=0;i<img_num;i++)
    {
        //bzero(data_char, PKG_SIZE);
        myread(sock,data_char);
        for(j=0;j<height*width;j++)
            data[j]=data_char[2*j]+data_char[2*j+1]*256;
        for(j=0;j<height*width;j++)
            buf_sum[j]+=data[j];
    }
    double sum=0;
    int ave_value=0;
    for(j=0;j<height*width;j++){
        buf_sum[j]=buf_sum[j]/img_num;
    }
    for(j=0;j<height*width;j++){
        sum+=buf_sum[j];
    }
    ave_value=sum/(width*height);
    printf("ave_value=%d\n",ave_value);
    for(j=0;j<height*width;j++){
        data_d[j]=ave_value-buf_sum[j];
    }
}

void DVFileStore::mycorrection_usb(short *data_d,int height,int width)
{
    int img_num=8;
    int buf_sum[height*width];
    bzero(buf_sum,sizeof(buf_sum));
    unsigned short data[height*width];
    int i,j,k;
    for(i=0;i<64;i++)
    {
        int ret = ftcam->getFrame((char *)&ftcam_frame, sizeof(FTCAM_FRAME), 256);
        if (ret != 0)
        {
            printf("not all data recieved.\n");
        }
    }
    for(i=0;i<img_num;i++)
    {
        memset(&ftcam_frame,0,sizeof(ftcam_frame));
        int ret = ftcam->getFrame((char *)&ftcam_frame, sizeof(FTCAM_FRAME), 256);
        if (ret != 0)
        {
            printf("not all data recieved.\n");
        }
        for(j=0;j<height*width;j++)
            data[j]=ftcam_frame.frameBuffer[j]/16;
        for(j=0;j<height*width;j++)
            buf_sum[j]+=data[j];
    }
    double sum=0;
    int ave_value=0;
    for(j=0;j<height*width;j++){
        buf_sum[j]=buf_sum[j]/img_num;
    }
    for(j=0;j<height*width;j++){
        sum+=buf_sum[j];
    }
    ave_value=sum/(width*height);
    printf("ave_value=%d\n",ave_value);
for(j=0;j<height*width;j++){
        data_d[j]=ave_value-buf_sum[j];
    }
}

void DVFileStore::mycorrection_usb1(short *data_d,int height,int width)
{
    int img_num=8;
    int buf_sum[height*width];
    bzero(buf_sum,sizeof(buf_sum));
    unsigned short data[height*width];
    int i,j,k;
    for(i=0;i<64;i++)
    {
        int ret = ftcam1->getFrame((char *)&ftcam_frame1, sizeof(FTCAM_FRAME), 256);
        if (ret != 0)
        {
            printf("not all data recieved.\n");
        }
    }
    for(i=0;i<img_num;i++)
    {
        memset(&ftcam_frame1,0,sizeof(ftcam_frame1));
        int ret = ftcam1->getFrame((char *)&ftcam_frame1, sizeof(FTCAM_FRAME), 256);
        if (ret != 0)
        {
            printf("not all data recieved.\n");
        }
        for(j=0;j<height*width;j++)
            data[j]=ftcam_frame1.frameBuffer[j]/16;
        for(j=0;j<height*width;j++)
            buf_sum[j]+=data[j];
    }
    double sum=0;
    int ave_value=0;
    for(j=0;j<height*width;j++){
        buf_sum[j]=buf_sum[j]/img_num;
    }
    for(j=0;j<height*width;j++){
        sum+=buf_sum[j];
    }
    ave_value=sum/(width*height);
    printf("ave_value=%d\n",ave_value);
for(j=0;j<height*width;j++){
        data_d[j]=ave_value-buf_sum[j];
    }
}

//-----------结束关闭文件-------------------------------------------------------------
void DVFileStore::DVFileStoreEnd(void)
{
    close(sock);
    fclose(DVFp);
    DVFp = NULL;
    ftcam->closedev();
    ftcam1->closedev();
    //cap1.open(0);
    cap.release();
    //cap1.release();

}

//创建对比度滑动条响应函数
void myAutoContrast(int,void*)
{
    if(m_nContrast==0)
        m_nContrast=1;
        printf("contrast=%d\n",m_nContrast);
}

//创建亮度滑动条响应函数
void myAutoBright(int,void*)
{
    printf("bright=%d\n",m_nBright);
}

//========================立体视觉========================
void DVFileStore::readcolorimg(cv::Mat&  img_left,cv::Mat&  img_right)
{
    if(runParams.file_type == "camera")
    {
        cv::Mat img_src;
        //cap1.open(runParams.camera1);
        cap1.set(CV_CAP_PROP_FRAME_WIDTH,1280);
        cap1.set(CV_CAP_PROP_FRAME_HEIGHT,480);
        if (!cap1.isOpened())
        {
            cout << "capture device failed to open!" << endl;
         }
         int width;
         int height;
         //ss->loadRectifyDatas(runParams.rectifyParams_path);
         //updateSgbmParameters(ss->sgbm,ss->sgbmParams);
         //createBarSgbm(ss->sgbmParams);
         //while(cap.read(img_src))
         {
             cap1.read(img_src);
             //------------------分割-----------------------------
             width  = img_src.size().width/2;
             height = img_src.size().height;

             //---------扩展ROI区域-----------------------------
             cv::Rect rect_L = cv::Rect(0,0,width,height);
             img_left = img_src(rect_L);
             cv::Rect rect_R = cv::Rect(width,0,width,height);
             img_right = img_src(rect_R);

             //cv::imshow("left",img_left);
             //cv::imshow("right",img_right);
             //cv::waitKey(1);

         }
    }

    else if(runParams.file_type == "video")
    {
        cv::VideoCapture cap1,cap2;
        cap1.open(runParams.video1_dir);
        cap2.open(runParams.video2_dir);
        while(1)
        {
            cap1.read(img_left);
            cap2.read(img_right);
            //cv::imshow("left",img_left);
            //cv::imshow("right",img_right);
            //cv::waitKey(1);
        }
    }

    else if(runParams.file_type == "image")
    {
            img_left = cv::imread(runParams.image1_test);
            img_right = cv::imread(runParams.image2_test);
            //cv::imshow("left",img_left);
            //cv::imshow("right",img_right);
            //cv::waitKey(1);
    }
}

void DVFileStore::ProcessZbuf(unsigned short *buf_recv,cv::Mat& J)
{
    memset(IRScn.ZDepth, 0.0f, cam.cpara.camDet.wid * cam.cpara.camDet.hei*sizeof(int32));//清理Z缓冲为固定值
    cv::Mat img_left,img_right;
    myImage14toMat(buf_recv,img_left,m_nContrast,m_nBright,cam.cpara.camDet.wid,cam.cpara.camDet.hei);//无单点校正
    img_right = J.clone();
    ss->Disp_compute(img_left,img_right,runParams);
    ss->reproject(ss->disp8u, img_left, point_cloud_ptr);

}

void DVFileStore::processimg(cv::Mat& img_left,cv::Mat& img_right)
{    
    //ss->remapImage(img_left,img_right, runParams.rectifymethod);
    ss->Disp_compute(img_left,img_right,runParams);
    ss->reproject(ss->disp8u, img_left, point_cloud_ptr);
    //cout << "PointCloud size: "<< point_cloud_ptr->size()<<"\n";

    imshow("disparity", ss->disp8u);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    viewer->updatePointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "reconstruction");
    viewer->spinOnce();

    cv::waitKey(1);
}



