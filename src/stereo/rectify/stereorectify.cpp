#include "stereorectify.h"

#define USE_EIGEN

#ifdef USE_EIGEN
#include <Eigen/Eigen>
#endif

#define DECOMPOSE_SVD
//#define EPSILON 0.0001


stereoRectifyy::stereoRectifyy()
{
}


//功能 :导入双目相机标定参数
bool stereoRectifyy::loadCalibDatas(string xmlFilePath)
{
    cv::FileStorage fs(xmlFilePath, cv::FileStorage::READ);
    if (fs.isOpened())
    {
        cv::FileNodeIterator it = fs["imageSize"].begin();
         it >> imageSize.width >> imageSize.height;

        fs["leftCameraMatrix"]				>> stereoParams.calib_M_L;
        fs["leftDistortCoefficients"]	    >> stereoParams.calib_D_L;
        fs["rightCameraMatrix"]		    >> stereoParams.calib_M_R;
        fs["rightDistortCoefficients"]	>> stereoParams.calib_D_R;
        cv::Mat_<float> rotMat;
        fs["rotationMatrix"] >> rotMat;
        cv::Rodrigues(rotMat,stereoParams.R);//由向量转换成3x3矩阵。
        fs["translationVector"] >>stereoParams.T;
    }
    fs.release();

    return 1;
}

int stereoRectifyy::rectify(RunParams& runParams)
{
    cout << "Rectify start..." << endl;

    loadCalibDatas(runParams.stereocalib_path);

    rectifyStereoCamera(runParams.rectifymethod);

    //rectifySingleCamera(stereoParams,remapMat);

    saveStereoDatas(runParams.rectifyParams_path,runParams.rectifymethod);

    cout << "Rectify END!" << endl;
    return 1;
}

//功能 : 生成单个摄像头的校正矩阵
int stereoRectifyy::rectifySingleCamera()
{
   cv::initUndistortRectifyMap(
       stereoParams.calib_M_L,
       stereoParams.calib_D_L,
       cv::Mat(),
       cv::getOptimalNewCameraMatrix(
           stereoParams.calib_M_L,
           stereoParams.calib_D_L,
           imageSize, 1, imageSize, 0),
           imageSize,
           CV_16SC2,
           remapMat.Calib_mX_L,
           remapMat.Calib_mY_L);

   return true;
}

//功能 : 执行双目摄像机校正，生成双目校正数据
int stereoRectifyy::rectifyStereoCamera(string method = "RECTIFY_BOUGUET")
{
   //初始化
   remapMat.Calib_mX_L = cv::Mat(imageSize, CV_32FC1);
   remapMat.Calib_mY_L = cv::Mat(imageSize, CV_32FC1);
   remapMat.Calib_mX_R = cv::Mat(imageSize, CV_32FC1);
   remapMat.Calib_mY_R = cv::Mat(imageSize, CV_32FC1);

   cv::Mat R1, R2, P1, P2, Q;
   cv::Rect roi1, roi2;

   //执行双目校正
   cv::stereoRectify(
       stereoParams.calib_M_L, stereoParams.calib_D_L,
       stereoParams.calib_M_R, stereoParams.calib_D_R,
       imageSize,
       stereoParams.R,stereoParams.T,
       R1,R2, P1, P2, Q,
       cv::CALIB_ZERO_DISPARITY, 1,
       imageSize,
       &roi1, &roi2);

   //使用HARTLEY方法的额外处理
   if (method == "RECTIFY_HARTLEY")
   {
     /*  vector<cv::Point2f> allimgpt[2];
       for(int  i = 0; i < cornerDatas.nImages; i++ )
       {
           copy(cornerDatas.imagePoints1[i].begin(), cornerDatas.imagePoints1[i].end(), back_inserter(allimgpt[0]));
           copy(cornerDatas.imagePoints2[i].begin(), cornerDatas.imagePoints2[i].end(), back_inserter(allimgpt[1]));
       }

       cv::Mat F, H1, H2;
       F = cv::findFundamentalMat(
                   cv::Mat(allimgpt[0]),
                   cv::Mat(allimgpt[1]),
                   cv::FM_8POINT, 0, 0);
       cv::stereoRectifyUncalibrated(
                   cv::Mat(allimgpt[0]),
                   cv::Mat(allimgpt[1]),
           F, cornerDatas.imageSize, H1, H2, 3);

       R1 = stereoParams.cameraParams1.M.inv() * H1 * stereoParams.cameraParams1.M;
       R2 = stereoParams.cameraParams2.M.inv() * H2 * stereoParams.cameraParams2.M;
       P1 = stereoParams.cameraParams1.M;
       P2 = stereoParams.cameraParams2.M;*/
   }

   //生成图像校正所需的像素映射矩阵
   cv::initUndistortRectifyMap(
       stereoParams.calib_M_L, stereoParams.calib_D_L,
       R1, P1,
       imageSize,
       CV_16SC2,
       remapMat.Calib_mX_L,remapMat.Calib_mY_L);

   cv::initUndistortRectifyMap(
       stereoParams.calib_M_R,stereoParams.calib_D_R,
       R2, P2,
       imageSize,
       CV_16SC2,
       remapMat.Calib_mX_R, remapMat.Calib_mY_R);

   //输出数据
   Q.copyTo(remapMat.Calib_Q);
   remapMat.Calib_Roi_L = roi1;
   remapMat.Calib_Roi_R = roi2;
   cout<<roi1<<endl;
   cout<<roi2<<endl;

   return 1;
}

/*----------------------------
 * 功能 : 保存双目校正参数
 *----------------------------
 * 函数 : stereoRectifyy::saveStereoDatas
 * 访问 : public
 * 返回 : 0 - 操作失败，1 - 操作成功
 *
 * 参数 : filename		[in]	保存路径/文件名
 * 参数 : method			[in]	双目校正方法
 * 参数 : stereoParams	[in]	双目标定结果
 * 参数 : remapMatrixs	[in]	图像校正像素映射矩阵
 */
int stereoRectifyy::saveStereoDatas(string filename, string method)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    if (fs.isOpened())
    {
        time_t rawtime;
        time(&rawtime);
        fs << "calibrationDate" << asctime(localtime(&rawtime));

        fs << "imageSize" << "[" << imageSize.width << imageSize.height << "]";

        fs << "leftCameraMatrix"			<< stereoParams.calib_M_L;
        fs << "leftDistortCoefficients"		<< stereoParams.calib_D_L;
        fs << "rightCameraMatrix"			<< stereoParams.calib_M_R;
        fs << "rightDistortCoefficients"	<< stereoParams.calib_D_R;
        fs << "rotationMatrix"				<< stereoParams.R;
        fs << "translationVector"			<< stereoParams.T;
        fs << "foundationalMatrix"			<< stereoParams.F;

        if (method == "RECTIFY_BOUGUET")
        {
            fs << "rectifyMethod" << "BOUGUET";
            fs << "leftValidArea" << "[:"
                << remapMat.Calib_Roi_L.x << remapMat.Calib_Roi_L.y
                << remapMat.Calib_Roi_L.width << remapMat.Calib_Roi_L.height << "]";
            fs << "rightValidArea" << "[:"
                << remapMat.Calib_Roi_R.x << remapMat.Calib_Roi_R.y
                << remapMat.Calib_Roi_R.width << remapMat.Calib_Roi_R.height << "]";
            fs << "QMatrix" << remapMat.Calib_Q;
        }
        else
            fs << "rectifyMethod" << "HARTLEY";

        fs << "remapX1" << remapMat.Calib_mX_L;
        fs << "remapY1" << remapMat.Calib_mY_L;
        fs << "remapX2" << remapMat.Calib_mX_R;
        fs << "remapY2" << remapMat.Calib_mY_R;

        fs.release();
        return 1;
    }
    else
    {
        return 0;
    }
}

int stereoRectifyy::UnCalibRectify(cv::Mat &rgb1,cv::Mat &rgb2,RunParams& runParams)
{    
    cout << "UnCalibRectify start..." << endl;

    loadCalibDatas(runParams.stereocalib_path);

    extractfeature(rgb1,rgb2);

    comput_F(rgb1,rgb2);

    FindCameraMatrices();

    rectify_param();
    //rectifyStereoCamera(runParams.rectifymethod);

    saveStereoDatas(runParams.rectifyParams_path,runParams.rectifymethod);

    cout << "UnCalibRectify END!" << endl;

    return 1;

}

void stereoRectifyy::extractfeature(cv::Mat &img_1,cv::Mat &img_2)//计算两幅图像的特征点，描述子
{
    imageSize = cv::Size(img_1.cols,img_1.rows);

    //cv::cvtColor(rgb1, rgb1, CV_BGR2GRAY);
    //cv::cvtColor(rgb2, rgb2, CV_BGR2GRAY);

    if( !img_1.data || !img_2.data)
    {
        cout<<"请先读取左右视点图像"<<endl;
    }

    _detector = cv::FeatureDetector::create("PyramidFAST");
    _extractor = cv::DescriptorExtractor::create("SURF");
    //int minHessian = 400;
    //_detector = cv::FeatureDetector::create( "GridSIFT" );
    //cv::SurfFeatureDetector _detector( minHessian );
    //_descriptor = cv::DescriptorExtractor::create( "SIFT" );
    //cv::SurfDescriptorExtractor _descriptor;

    _detector->detect( img_1, kp1 );  //提取关键点
    _detector->detect( img_2, kp2 );

    assert(kp1.size() > 0);
    assert(kp2.size() > 0);

    cout<<"Key points of two images: "<<kp1.size()<<", "<<kp2.size()<<endl;
    // 可视化， 显示关键点
    cv::Mat imgShow;
    cv::drawKeypoints( img_1, kp1, imgShow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    cv::imshow("keypoints", imgShow);
    //cv::imwrite( "./keypoints.png", imgShow );
    cv::waitKey(1); //暂停等待一个按键

    _extractor->compute(img_1, kp1, desc1);
    _extractor->compute(img_2, kp2, desc2);

    if(desc1.empty()){CV_Error(0,"desc1 is empty");}
    if(desc2.empty()){CV_Error(0,"desc2 is empty");}
    cout << "img1 has " << kp1.size() << " points (descriptors " << desc1.rows << ")" << endl;
    cout << "img2 has " << kp2.size() << " points (descriptors " << desc2.rows << ")" << endl;

}

void stereoRectifyy::distance_Match(cv::Mat &rgb1,cv::Mat &rgb2, cv::Mat &desp1, cv::Mat &desp2, vector<cv::DMatch> &matches) //四倍距离去除野点匹配
{
    cv::Mat imgMatches;
    vector<cv::DMatch>  dis_matches;
    matcher.match( desp1, desp2, dis_matches );
    // 筛选匹配，把距离太大的去掉
    // 这里使用的准则是去掉大于四倍最小距离的匹配
    double minDis = 9999;
    for ( size_t i=0; i<dis_matches.size(); i++ )
    {
        if ( dis_matches[i].distance < minDis )
            minDis = dis_matches[i].distance;
    }

    for ( size_t i=0; i<dis_matches.size(); i++ )
    {
        if (dis_matches[i].distance < 4*minDis)
            matches.push_back( dis_matches[i] );
    }

    // 显示 good matches
    cout<<"good matches="<<matches.size()<<endl;
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
    cv::imshow( "good matches", imgMatches );
    cv::imwrite( "./data/good_matches.png", imgMatches );
    cv::waitKey(0);
}

void stereoRectifyy::knnMatch(cv::Mat &rgb1,cv::Mat &rgb2, cv::Mat &desp1, cv::Mat &desp2, vector<cv::DMatch> &matches) //knn交叉匹配
{
    const float minRatio = 1.f / 1.5f;
    const int k = 2;

    cv::Mat imgMatches;
    vector<vector<cv::DMatch> > knnMatches;
    matcher.knnMatch(desp1, desp2, knnMatches, k);

    for (size_t i = 0; i < knnMatches.size(); i++) {
        const cv::DMatch& bestMatch = knnMatches[i][0];
        const cv::DMatch& betterMatch = knnMatches[i][1];

        float  distanceRatio = bestMatch.distance / betterMatch.distance;
        if (distanceRatio < minRatio)
            matches.push_back(bestMatch);
    }

    cout<<"knn good matches="<<matches.size()<<endl;
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matches, imgMatches );
    cv::imshow( "knn good matches", imgMatches );
    cv::imwrite( "./data/knn_good_matches.png", imgMatches );
    cv::waitKey(1);
}

void stereoRectifyy::comput_F(cv::Mat &rgb1,cv::Mat &rgb2) //RANSAC方法计算基础矩阵，并去除野点匹配。
{
    vector<cv::DMatch>  matches;
    //distance_Match(rgb1,rgb2, desp1, desp2, matches);
    knnMatch(rgb1,rgb2, desc1, desc2,matches);

   /* int ptCount = (int)matches.size();
    // Convert KeyPoint into Mat
    cv::Mat mKeypoint_1(ptCount, 2, CV_32F);
    cv::Mat mKeypoint_2(ptCount, 2, CV_32F);

    cv::Point2f pt;

    for(int i = 0; i < ptCount; i++)
    {
        pt = kp1[matches[i].queryIdx].pt;
        mKeypoint_1.at<float>(i, 0) = pt.x;
        mKeypoint_1.at<float>(i, 1) = pt.y;

        pt = kp2[matches[i].trainIdx].pt;
        mKeypoint_2.at<float>(i, 0) = pt.x;
        mKeypoint_2.at<float>(i, 1) = pt.y;
    }
    vector<uchar> RANSACStatus;    // 0 for outliers and 1 for the other points
    F = cv::findFundamentalMat(mKeypoint_1, mKeypoint_2, cv::FM_RANSAC, 3, 0.99, RANSACStatus);

    cout << "F= " << F << endl;

    // Calculate the outliers
    int OutlinerCount = 0;
    for( int i = 0; i < ptCount; i++)
    {
        if( RANSACStatus[i] == 0 )
        {
            OutlinerCount++;
        }
    }

    // Calculate the inliers
    int InlinerCount = ptCount - OutlinerCount;

    //vector<cv::Point2f> mInliner_1, mInliner_2;
    //std::vector<cv::DMatch> InlinerMatches;

    mInliner_1.resize(InlinerCount);
    mInliner_2.resize(InlinerCount);
    InlinerMatches.resize(InlinerCount);

    InlinerCount = 0;

    for(int i = 0; i < ptCount; i++)
    {
        if(RANSACStatus[i] != 0)
        {
            mInliner_1[InlinerCount].x = mKeypoint_1.at<float>(i, 0);
            mInliner_1[InlinerCount].y = mKeypoint_1.at<float>(i, 1);
            mInliner_2[InlinerCount].x = mKeypoint_2.at<float>(i, 0);
            mInliner_2[InlinerCount].y = mKeypoint_2.at<float>(i, 1);

            InlinerMatches[InlinerCount].queryIdx = InlinerCount;
            InlinerMatches[InlinerCount].trainIdx = InlinerCount;

            InlinerCount++;
        }
    }

    vector<cv::KeyPoint> Inliner_1,Inliner_2;

    cv::KeyPoint::convert(mInliner_1,Inliner_1);
    cv::KeyPoint::convert(mInliner_2,Inliner_2);

    // 显示计算F过后的内点匹配
    cout<<"Find total "<<InlinerMatches.size()<<" InlinerMatches."<<endl;
    cv::Mat OutImage;
    cv::drawMatches(rgb1, Inliner_1, rgb2, Inliner_2, InlinerMatches, OutImage);
    cv::imshow("Match features", OutImage);
    cv::waitKey( 1 );

*/

    //Align all points
    vector<cv::KeyPoint> alignedKps1, alignedKps2;
    for (size_t i = 0; i < matches.size(); i++)
    {
        alignedKps1.push_back(kp1[matches[i].queryIdx]);
        alignedKps2.push_back(kp2[matches[i].trainIdx]);
    }

    for (unsigned i = 0; i < alignedKps1.size(); i++)
        ps1.push_back(alignedKps1[i].pt);

    for (unsigned i = 0; i < alignedKps2.size(); i++)
        ps2.push_back(alignedKps2[i].pt);

    vector<uchar> RANSACStatus;
    F = cv::findFundamentalMat(
                cv::Mat(ps1),
                cv::Mat(ps2),
                RANSACStatus,
                cv::FM_RANSAC, 3, 0.99);

    cout << "F= " << F << endl;

    //优化匹配结果
    vector<cv::KeyPoint> leftInlier;
    vector<cv::KeyPoint> rightInlier;
    vector<cv::DMatch> inlierMatch;
    cv::DMatch  temp_matches;
    int index = 0;
    for (unsigned i = 0; i < matches.size(); i++) {
        if (RANSACStatus[i] != 0){
            leftInlier.push_back(alignedKps1[i]);
            rightInlier.push_back(alignedKps2[i]);
            temp_matches.trainIdx = index;
            temp_matches.queryIdx = index;
            inlierMatch.push_back(temp_matches);
            index++;
        }
    }

    ps1.clear();
    ps2.clear();

    for (unsigned i = 0; i < leftInlier.size(); i++)
        ps1.push_back(leftInlier[i].pt);

    for (unsigned i = 0; i < rightInlier.size(); i++)
        ps2.push_back(rightInlier[i].pt);

    kp1.clear();
    kp2.clear();
    kp1 = leftInlier;
    kp2 = rightInlier;

    goodMatches.clear();
    goodMatches = inlierMatch;

    // 显示计算F过后的内点匹配
    cout<<"Find total "<<goodMatches.size()<<" goodMatches."<<endl;
    cv::Mat OutImage;
    cv::drawMatches(rgb1, kp1, rgb2, kp2, goodMatches, OutImage);
    cv::imshow("Match features", OutImage);
    cv::waitKey( 1 );
}

bool stereoRectifyy::FindCameraMatrices()
{
    cv::Matx34d P(1,0,0,0,
                  0,1,0,0,
                  0,0,1,0),
                P1(1,0,0,0,
                   0,1,0,0,
                   0,0,1,0);

   /* cout << "Find camera matrices...\n";
    vector<uchar> RANSACStatus;
    F = cv::findFundamentalMat(
                cv::Mat(ps1),
                cv::Mat(ps2),
                RANSACStatus,
                cv::FM_RANSAC, 3, 0.99);

    cout << "F= " << F <<endl;

    cout << cv::countNonZero(RANSACStatus) << "matches of second F!" << endl;

    if(cv::countNonZero(RANSACStatus) < 100) { // || ((double)imgpts1_good.size() / (double)imgpts1.size()) < 0.25
        cout << "not enough inliers after F matrix" << endl;
        return false;
    }*/

    //Essential matrix: compute then extralct cameras [R|t]
    cv::Mat_<double> E = stereoParams.calib_M_R.t() * F * stereoParams.calib_M_L; //according to HZ (9.12)
    if(fabs(cv::determinant(E)) > 1e-07) {
        cout << "det(E) != 0 : " << determinant(E) << "\n";
        P1 = 0;
        return false;
    }

    cv::Mat_<double> R1(3,3);
    cv::Mat_<double> R2(3,3);
    cv::Mat_<double> t1(1,3);
    cv::Mat_<double> t2(1,3);

    if (!DecomposeEtoRandT(E,R1,R2,t1,t2)) return false;

    if(cv::determinant(R1)+1.0 < 1e-09)
    {
        //according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
        cout << "det(R) == -1 ["<<cv::determinant(R1)<<"]: flip E's sign" << endl;
        E = -E;
        DecomposeEtoRandT(E,R1,R2,t1,t2);
    }
    if(fabs(cv::determinant(R1))-1.0 > 1e-07)
    {
        cout << "det(R) != +-1.0, this is not a rotation matrix" << endl;
        P1 = 0;
        return false;
    }

    P1 = cv::Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	t1(0),
                 R1(1,0),	R1(1,1),	R1(1,2),	t1(1),
                 R1(2,0),	R1(2,1),	R1(2,2),	t1(2));
    cout << "Testing P1 " << endl << cv::Mat(P1) << endl;

    vector<CloudPoint> pcloud;
    double reproj_error1 = TriangulatePoints(kp1, kp2, P, P1,pcloud);
    vector<uchar> tmp_status;
    //check if pointa are triangulated --in front-- of cameras for all 4 ambiguations
    if (!TestTriangulation(pcloud,P1,tmp_status)  || reproj_error1 > 100.0)
    {
        P1 = cv::Matx34d(R1(0,0),	R1(0,1),	R1(0,2),	t2(0),
                     R1(1,0),	R1(1,1),	R1(1,2),	t2(1),
                     R1(2,0),	R1(2,1),	R1(2,2),	t2(2));
        cout << "Testing P1 "<< endl << cv::Mat(P1) << endl;

        pcloud.clear();
        reproj_error1 = TriangulatePoints(kp1, kp2, P, P1,pcloud);

        if (!TestTriangulation(pcloud,P1,tmp_status) || reproj_error1 > 100.0)
        {
            if(fabs(cv::determinant(R2))-1.0 > 1e-07)
            {
                cout << "det(R) != +-1.0, this is not a rotation matrix" << endl;
                P1 = 0;
                return false;
            }

            P1 = cv::Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	t1(0),
                         R2(1,0),	R2(1,1),	R2(1,2),	t1(1),
                         R2(2,0),	R2(2,1),	R2(2,2),	t1(2));
            cout << "Testing P1 "<< endl << cv::Mat(P1) << endl;

            pcloud.clear();
            reproj_error1 = TriangulatePoints(kp1, kp2, P, P1,pcloud);

            if (!TestTriangulation(pcloud,P1,tmp_status) || reproj_error1 > 100.0)
            {
                P1 = cv::Matx34d(R2(0,0),	R2(0,1),	R2(0,2),	t2(0),
                             R2(1,0),	R2(1,1),	R2(1,2),	t2(1),
                             R2(2,0),	R2(2,1),	R2(2,2),	t2(2));
                cout << "Testing P1 "<< endl << cv::Mat(P1) << endl;

                pcloud.clear();
                reproj_error1 = TriangulatePoints(kp1, kp2, P, P1,pcloud);

                if (!TestTriangulation(pcloud,P1,tmp_status) || reproj_error1 > 100.0)
                {
                    cout << "Shit." << endl;
                    return false;
                }
            }
        }
    }

    cout << "R1= " << R1 << endl;
    cout << "R2= " << R2 << endl;
    cout << "t1= " << t1 << endl;
    cout << "t2= " << t2 << endl;

    cout << "P1= " << P1 <<endl;
    /*stereoParams.R = cv::Mat(cv::Matx33d(P1(0,0),	P1(0,1),	P1(0,2),
                                 P1(1,0),	P1(1,1),	P1(1,2),
                                 P1(2,0),	P1(2,1),	P1(2,2)));
    stereoParams.T = cv::Mat(cv::Matx13d(P1(0,3),	P1(1,3),	P1(2,3)));*/
    //stereoParams.R = R1;
    //stereoParams.T = t1;

    return true;
}

bool stereoRectifyy::DecomposeEtoRandT(
    cv::Mat_<double>& E,
    cv::Mat_<double>& R1,
    cv::Mat_<double>& R2,
    cv::Mat_<double>& t1,
    cv::Mat_<double>& t2)
{
#ifdef DECOMPOSE_SVD
    //Using HZ E decomposition
    cv::Mat svd_u, svd_vt, svd_w;
    TakeSVDOfE(E,svd_u,svd_vt,svd_w);

    //check if first and second singular values are the same (as they should be)
    double singular_values_ratio = fabs(svd_w.at<double>(0) / svd_w.at<double>(1));
    if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
    if (singular_values_ratio < 0.7) {
        cout << "singular values are too far apart\n";
        return false;
    }

    cv::Matx33d W(0,-1,0,	//HZ 9.13
        1,0,0,
        0,0,1);
    cv::Matx33d Wt(0,1,0,
        -1,0,0,
        0,0,1);
    R1 = svd_u * cv::Mat(W) * svd_vt; //HZ 9.19
    R2 = svd_u * cv::Mat(Wt) * svd_vt; //HZ 9.19
    t1 = svd_u.col(2); //u3
    t2 = -svd_u.col(2); //u3
#else
    //Using Horn E decomposition
    DecomposeEssentialUsingHorn90(E[0],R1[0],R2[0],t1[0],t2[0]);
#endif
    return true;
}

void stereoRectifyy::DecomposeEssentialUsingHorn90(double _E[9], double _R1[9], double _R2[9], double _t1[3], double _t2[3])
{
    //from : http://people.csail.mit.edu/bkph/articles/Essential.pdf
#ifdef USE_EIGEN
    using namespace Eigen;

    Matrix3d E = Map<Matrix<double,3,3,RowMajor> >(_E);
    Matrix3d EEt = E * E.transpose();
    Vector3d e0e1 = E.col(0).cross(E.col(1)),e1e2 = E.col(1).cross(E.col(2)),e2e0 = E.col(2).cross(E.col(2));
    Vector3d b1,b2;

#if 1
    //Method 1
    Matrix3d bbt = 0.5 * EEt.trace() * Matrix3d::Identity() - EEt; //Horn90 (12)
    Vector3d bbt_diag = bbt.diagonal();
    if (bbt_diag(0) > bbt_diag(1) && bbt_diag(0) > bbt_diag(2)) {
        b1 = bbt.row(0) / sqrt(bbt_diag(0));
        b2 = -b1;
    } else if (bbt_diag(1) > bbt_diag(0) && bbt_diag(1) > bbt_diag(2)) {
        b1 = bbt.row(1) / sqrt(bbt_diag(1));
        b2 = -b1;
    } else {
        b1 = bbt.row(2) / sqrt(bbt_diag(2));
        b2 = -b1;
    }
#else
    //Method 2
    if (e0e1.norm() > e1e2.norm() && e0e1.norm() > e2e0.norm()) {
        b1 = e0e1.normalized() * sqrt(0.5 * EEt.trace()); //Horn90 (18)
        b2 = -b1;
    } else if (e1e2.norm() > e0e1.norm() && e1e2.norm() > e2e0.norm()) {
        b1 = e1e2.normalized() * sqrt(0.5 * EEt.trace()); //Horn90 (18)
        b2 = -b1;
    } else {
        b1 = e2e0.normalized() * sqrt(0.5 * EEt.trace()); //Horn90 (18)
        b2 = -b1;
    }
#endif

    //Horn90 (19)
    Matrix3d cofactors; cofactors.col(0) = e1e2; cofactors.col(1) = e2e0; cofactors.col(2) = e0e1;
    cofactors.transposeInPlace();

    //B = [b]_x , see Horn90 (6) and http://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
    Matrix3d B1; B1 <<	0,-b1(2),b1(1),
                        b1(2),0,-b1(0),
                        -b1(1),b1(0),0;
    Matrix3d B2; B2 <<	0,-b2(2),b2(1),
                        b2(2),0,-b2(0),
                        -b2(1),b2(0),0;

    Map<Matrix<double,3,3,RowMajor> > R1(_R1),R2(_R2);

    //Horn90 (24)
    R1 = (cofactors.transpose() - B1*E) / b1.dot(b1);
    R2 = (cofactors.transpose() - B2*E) / b2.dot(b2);
    Map<Vector3d> t1(_t1),t2(_t2);
    t1 = b1; t2 = b2;

    cout << "Horn90 provided " << endl << R1 << endl << "and" << endl << R2 << endl;
#endif
}

void stereoRectifyy::TakeSVDOfE(cv::Mat_<double>& E, cv::Mat& svd_u, cv::Mat& svd_vt, cv::Mat& svd_w)
{
#ifdef USE_EIGEN
    //Using OpenCV's SVD
    cv::SVD svd(E,cv::SVD::MODIFY_A);
    svd_u = svd.u;
    svd_vt = svd.vt;
    svd_w = svd.w;
#else
    //Using Eigen's SVD
    cout << "Eigen3 SVD..\n";
    Eigen::Matrix3f  e = Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >((double*)E.data).cast<float>();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(e, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf Esvd_u = svd.matrixU();
    Eigen::MatrixXf Esvd_v = svd.matrixV();
    svd_u = (cv::Mat_<double>(3,3) << Esvd_u(0,0), Esvd_u(0,1), Esvd_u(0,2),
                          Esvd_u(1,0), Esvd_u(1,1), Esvd_u(1,2),
                          Esvd_u(2,0), Esvd_u(2,1), Esvd_u(2,2));
    cv::Mat_<double> svd_v = (cv::Mat_<double>(3,3) << Esvd_v(0,0), Esvd_v(0,1), Esvd_v(0,2),
                          Esvd_v(1,0), Esvd_v(1,1), Esvd_v(1,2),
                          Esvd_v(2,0), Esvd_v(2,1), Esvd_v(2,2));
    svd_vt = svd_v.t();
    svd_w = (cv::Mat_<double>(1,3) << svd.singularValues()[0] , svd.singularValues()[1] , svd.singularValues()[2]);
#endif

    cout << "----------------------- SVD ------------------------\n";
    cout << "U:\n"<<svd_u<<"\nW:\n"<<svd_w<<"\nVt:\n"<<svd_vt<<endl;
    cout << "----------------------------------------------------\n";
}

//Triagulate points
double stereoRectifyy::TriangulatePoints(const vector<cv::KeyPoint>& pt_set1,
                        const vector<cv::KeyPoint>& pt_set2,
                        const cv::Matx34d& P,
                        const cv::Matx34d& P1,
                        vector<CloudPoint>& pointcloud)
{
    cv::Matx44d P1_(P1(0,0),P1(0,1),P1(0,2),P1(0,3),
                P1(1,0),P1(1,1),P1(1,2),P1(1,3),
                P1(2,0),P1(2,1),P1(2,2),P1(2,3),
                0,		0,		0,		1);
    cv::Matx44d P1inv(P1_.inv());

    cout << "Triangulating...\n";
    vector<double> reproj_error;
    unsigned int pts_size = pt_set1.size();

#if 0
    //Using OpenCV's triangulation
    //convert to Point2f
    vector<Point2f> _pt_set1_pt,_pt_set2_pt;
    KeyPointsToPoints(pt_set1,_pt_set1_pt);
    KeyPointsToPoints(pt_set2,_pt_set2_pt);

    //undistort
    Mat pt_set1_pt,pt_set2_pt;
    undistortPoints(_pt_set1_pt, pt_set1_pt, K, distcoeff);
    undistortPoints(_pt_set2_pt, pt_set2_pt, K, distcoeff);

    //triangulate
    Mat pt_set1_pt_2r = pt_set1_pt.reshape(1, 2);//通道数变为1，行数为2，为什么这么干？
    Mat pt_set2_pt_2r = pt_set2_pt.reshape(1, 2);
    Mat pt_3d_h(1,pts_size,CV_32FC4);
    cv::triangulatePoints(P,P1,pt_set1_pt_2r,pt_set2_pt_2r,pt_3d_h);

    //calculate reprojection
    vector<Point3f> pt_3d;
    convertPointsHomogeneous(pt_3d_h.reshape(4, 1), pt_3d);
    cv::Mat_<double> R = (cv::Mat_<double>(3,3) << P(0,0),P(0,1),P(0,2), P(1,0),P(1,1),P(1,2), P(2,0),P(2,1),P(2,2));
    Vec3d rvec; Rodrigues(R ,rvec);
    Vec3d tvec(P(0,3),P(1,3),P(2,3));
    vector<Point2f> reprojected_pt_set1;
    projectPoints(pt_3d,rvec,tvec,K,distcoeff,reprojected_pt_set1);

    for (unsigned int i=0; i<pts_size; i++) {
        CloudPoint cp;
        cp.pt = pt_3d[i];
        pointcloud.push_back(cp);
        reproj_error.push_back(norm(_pt_set1_pt[i]-reprojected_pt_set1[i]));
    }
#else
    cv::Mat_<double> KP1 = stereoParams.calib_M_R * cv::Mat(P1);

    for (int i=0; i<pts_size; i++) {//对每一个关键点，先转化成齐次坐标模式，然后在左边乘以Kinv，得到带标定参数的点。
        cv::Point2f kp = pt_set1[i].pt;
        cv::Point3d u(kp.x,kp.y,1.0);//齐次坐标
        cv::Mat_<double> um = stereoParams.calib_M_L.inv() * cv::Mat_<double>(u);
        u.x = um(0); u.y = um(1); u.z = um(2);

        cv::Point2f kp1 = pt_set2[i].pt;
        cv::Point3d u1(kp1.x,kp1.y,1.0);
        cv::Mat_<double> um1 = stereoParams.calib_M_R.inv() * cv::Mat_<double>(u1);
        u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);

        cv::Mat_<double> X = IterativeLinearLSTriangulation(u,P,u1,P1);//进行迭代线性重构

//		cout << "3D Point: " << X << endl;
//		Mat_<double> x = Mat(P1) * X;
//		cout <<	"P1 * Point: " << x << endl;
//		Mat_<double> xPt = (Mat_<double>(3,1) << x(0),x(1),x(2));
//		cout <<	"Point: " << xPt << endl;
        cv::Mat_<double> xPt_img = KP1 * X;				//reproject
//		cout <<	"Point * K: " << xPt_img << endl;
        cv::Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));

        double reprj_err = norm(xPt_img_-kp1);
        reproj_error.push_back(reprj_err);

        CloudPoint cp;
        cp.pt = cv::Point3d(X(0),X(1),X(2));
        cp.reprojection_error = reprj_err;

        pointcloud.push_back(cp);
    }
#endif

    cv::Scalar mse = cv::mean(reproj_error);
    cout << "Done. ("<<"mean reproj err = " << mse[0] << ")"<< endl;

    //show "range image"
#ifdef __SFM__DEBUG__
    {
        double minVal,maxVal;
        minMaxLoc(depths, &minVal, &maxVal);
        Mat tmp(240,320,CV_8UC3,Scalar(0,0,0)); //cvtColor(img_1_orig, tmp, CV_BGR2HSV);
        for (unsigned int i=0; i<pointcloud.size(); i++) {
            double _d = MAX(MIN((pointcloud[i].z-minVal)/(maxVal-minVal),1.0),0.0);
            circle(tmp, correspImg1Pt[i].pt, 1, Scalar(255 * (1.0-(_d)),255,255), CV_FILLED);
        }
        cvtColor(tmp, tmp, CV_HSV2BGR);
        imshow("Depth Map", tmp);
        waitKey(0);
        destroyWindow("Depth Map");
    }
#endif

    return mse[0];
}

cv::Mat_<double> stereoRectifyy::IterativeLinearLSTriangulation(cv::Point3d u,	//homogenous image point (u,v,1)
                                            cv::Matx34d P,			//camera 1 matrix
                                            cv::Point3d u1,			//homogenous image point in 2nd camera
                                            cv::Matx34d P1			//camera 2 matrix
                                            )
{
    double wi = 1, wi1 = 1;
    cv::Mat_<double> X(4,1);

    cv::Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
    X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;

    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most

        //recalculate weights
        double p2x = cv::Mat_<double>(cv::Mat_<double>(P).row(2)*X)(0);
        double p2x1 = cv::Mat_<double>(cv::Mat_<double>(P1).row(2)*X)(0);

        //breaking point
        if(fabs(wi - p2x) <= EPSILON && fabs(wi1 - p2x1) <= EPSILON) break;

        wi = p2x;
        wi1 = p2x1;

        //reweight equations and solve
        cv::Matx43d A((u.x*P(2,0)-P(0,0))/wi,		(u.x*P(2,1)-P(0,1))/wi,			(u.x*P(2,2)-P(0,2))/wi,
                  (u.y*P(2,0)-P(1,0))/wi,		(u.y*P(2,1)-P(1,1))/wi,			(u.y*P(2,2)-P(1,2))/wi,
                  (u1.x*P1(2,0)-P1(0,0))/wi1,	(u1.x*P1(2,1)-P1(0,1))/wi1,		(u1.x*P1(2,2)-P1(0,2))/wi1,
                  (u1.y*P1(2,0)-P1(1,0))/wi1,	(u1.y*P1(2,1)-P1(1,1))/wi1,		(u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        cv::Mat_<double> B = (cv::Mat_<double>(4,1) <<	  -(u.x*P(2,3)	-P(0,3))/wi,
                                                  -(u.y*P(2,3)	-P(1,3))/wi,
                                                  -(u1.x*P1(2,3)	-P1(0,3))/wi1,
                                                  -(u1.y*P1(2,3)	-P1(1,3))/wi1
                          );

        cv::solve(A,B,X_,cv::DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
    }
    return X;
}

cv::Mat_<double> stereoRectifyy::LinearLSTriangulation(cv::Point3d u,		//homogenous image point (u,v,1)
                                   cv::Matx34d P,		//camera 1 matrix
                                   cv::Point3d u1,		//homogenous image point in 2nd camera
                                   cv::Matx34d P1		//camera 2 matrix
                                   )
{

    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    //	cout << "u " << u <<", u1 " << u1 << endl;
    //	Matx<double,6,4> A; //this is for the AX=0 case, and with linear dependence..
    //	A(0) = u.x*P(2)-P(0);
    //	A(1) = u.y*P(2)-P(1);
    //	A(2) = u.x*P(1)-u.y*P(0);
    //	A(3) = u1.x*P1(2)-P1(0);
    //	A(4) = u1.y*P1(2)-P1(1);
    //	A(5) = u1.x*P(1)-u1.y*P1(0);
    //	Matx43d A; //not working for some reason...
    //	A(0) = u.x*P(2)-P(0);
    //	A(1) = u.y*P(2)-P(1);
    //	A(2) = u1.x*P1(2)-P1(0);
    //	A(3) = u1.y*P1(2)-P1(1);
    cv::Matx43d A(u.x*P(2,0)-P(0,0),	u.x*P(2,1)-P(0,1),		u.x*P(2,2)-P(0,2),
              u.y*P(2,0)-P(1,0),	u.y*P(2,1)-P(1,1),		u.y*P(2,2)-P(1,2),
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),	u1.x*P1(2,2)-P1(0,2),
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),	u1.y*P1(2,2)-P1(1,2)
              );
    cv::Matx41d B(-(u.x*P(2,3)	-P(0,3)),
              -(u.y*P(2,3)	-P(1,3)),
              -(u1.x*P1(2,3)	-P1(0,3)),
              -(u1.y*P1(2,3)	-P1(1,3)));

    cv::Mat_<double> X;
    cv::solve(A,B,X,cv::DECOMP_SVD);

    return X;
}

std::vector<cv::Point3d> stereoRectifyy::CloudPointsToPoints(const std::vector<CloudPoint> cpts)
{
    std::vector<cv::Point3d> out;
    for (unsigned int i=0; i<cpts.size(); i++)
    {
        out.push_back(cpts[i].pt);
    }
    return out;
}

bool stereoRectifyy::TestTriangulation(const vector<CloudPoint>& pcloud, const cv::Matx34d& P, vector<uchar>& status)
{
    vector<cv::Point3d> pcloud_pt3d = CloudPointsToPoints(pcloud);//把CloudPoint转化成Point3d
    vector<cv::Point3d> pcloud_pt3d_projected(pcloud_pt3d.size());

    cv::Matx44d P4x4 = cv::Matx44d::eye();
    for(int i=0;i<12;i++) P4x4.val[i] = P.val[i];//逐元素复制

    cv::perspectiveTransform(pcloud_pt3d, pcloud_pt3d_projected, P4x4);//

    status.resize(pcloud.size(),0);
    for (int i=0; i<pcloud.size(); i++) {
        status[i] = (pcloud_pt3d_projected[i].z > 0) ? 1 : 0;
    }
    int count = cv::countNonZero(status);

    double percentage = ((double)count / (double)pcloud.size());
    cout << count << "/" << pcloud.size() << " = " << percentage*100.0 << "% are in front of camera" << endl;
    if(percentage < 0.75)
        return false; //less than 75% of the points are in front of the camera

    //check for coplanarity of points
    if(false) //not
    {
        cv::Mat_<double> cldm(pcloud.size(),3);
        for(unsigned int i=0;i<pcloud.size();i++) {
            cldm.row(i)(0) = pcloud[i].pt.x;
            cldm.row(i)(1) = pcloud[i].pt.y;
            cldm.row(i)(2) = pcloud[i].pt.z;
        }
        cv::Mat_<double> mean;
        cv::PCA pca(cldm,mean,CV_PCA_DATA_AS_ROW);

        int num_inliers = 0;
        cv::Vec3d nrm = pca.eigenvectors.row(2); nrm = nrm / norm(nrm);
        cv::Vec3d x0 = pca.mean;
        double p_to_plane_thresh = sqrt(pca.eigenvalues.at<double>(2));

        for (int i=0; i<pcloud.size(); i++) {
            cv::Vec3d w = cv::Vec3d(pcloud[i].pt) - x0;
            double D = fabs(nrm.dot(w));
            if(D < p_to_plane_thresh) num_inliers++;
        }

        cout << num_inliers << "/" << pcloud.size() << " are coplanar" << endl;
        if((double)num_inliers / (double)(pcloud.size()) > 0.85)
            return false;
    }

    return true;
}

void stereoRectifyy::rectify_param()
{
    //初始化
    remapMat.Calib_mX_L = cv::Mat(imageSize, CV_32FC1);
    remapMat.Calib_mY_L = cv::Mat(imageSize, CV_32FC1);
    remapMat.Calib_mX_R = cv::Mat(imageSize, CV_32FC1);
    remapMat.Calib_mY_R = cv::Mat(imageSize, CV_32FC1);

    double r1[3][3],r2[3][3],p1[3][4],p2[3][4],q[4][4];
    cv::Mat R1 = cv::Mat(3,3,CV_64F,r1);
    cv::Mat R2 = cv::Mat(3,3,CV_64F,r2);
    cv::Mat P1 = cv::Mat(3,4,CV_64F,p1);
    cv::Mat P2 = cv::Mat(3,4,CV_64F,p2);
    cv::Mat Q   = cv::Mat(4,4,CV_64F,q);

    double _H1[3][3], _H2[3][3], _iM[3][3];
    H1 = cv::Mat(3, 3, CV_64F, _H1);
    H2 = cv::Mat(3, 3, CV_64F, _H2);
    iM = cv::Mat(3, 3, CV_64F, _iM);

    cv::stereoRectifyUncalibrated( cv::Mat(ps1), cv::Mat(ps2), F, imageSize, H1, H2, 3);
    //cv::stereoRectifyUncalibrated( mInliner_1, mInliner_2, F, imageSize, H1, H2, 3);

    /*cv::Mat Mrect = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat invMrect =Mrect.inv(cv::DECOMP_SVD);
    R1 = invMrect * H1 * Mrect;
    R2 = invMrect * H2 * Mrect;
    P1 = Mrect;
    P2 = Mrect;*/

    cv::invert(stereoParams.calib_M_L, iM);
    //R1 = iM * H1* stereoParams.calib_M_L;
    cv::invert(stereoParams.calib_M_R, iM);
    //R2 = iM * H2* stereoParams.calib_M_R;
    P1 = stereoParams.calib_M_L;
    P2 = stereoParams.calib_M_R;

    R1 = stereoParams.calib_M_L.inv(cv::DECOMP_SVD) * H1* stereoParams.calib_M_L;
    R2 = stereoParams.calib_M_R.inv(cv::DECOMP_SVD) * H2* stereoParams.calib_M_R;

   // cv::Mat distCoeffs;
    //生成图像校正所需的像素映射矩阵
    cv::initUndistortRectifyMap(
        stereoParams.calib_M_L, stereoParams.calib_D_L,
        R1, P1,
        imageSize,
        CV_32FC1,
        remapMat.Calib_mX_L,remapMat.Calib_mY_L);

    cv::initUndistortRectifyMap(
        stereoParams.calib_M_R, stereoParams.calib_D_R,
        R2, P2,
        imageSize,
        CV_32FC1,
        remapMat.Calib_mX_R, remapMat.Calib_mY_R);
}

//功能 : 对图像进行校正
int stereoRectifyy::remapImage(cv::Mat& imgleft, cv::Mat& imgright, string method = "RECTIFY_BOUGUET")
{
   cv::Mat imgleft_c = imgleft.clone();
   cv::Mat imgright_c = imgright.clone();

#if DEBUG_SHOW
   cv::Mat canvas;
   double sf;
   int w, h;
   sf = 320./MAX(imageSize.width, imageSize.height);
   w = cvRound(imageSize.width*sf);
   h = cvRound(imageSize.height*sf);
   canvas.create(h, w*2, CV_8UC3);
#endif

   if ( !remapMat.Calib_mX_L.empty() && !remapMat.Calib_mY_L.empty() )
   {
       cv::remap( imgleft_c, imgleft, remapMat.Calib_mX_L, remapMat.Calib_mY_L, cv::INTER_LINEAR );

#if DEBUG_SHOW
       cv::Mat canvasPart = canvas(cv::Rect(0, 0, w, h));
       cv::resize(imgleft, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
       if(method != "RECTIFY_BOUGUET")
       {
           cv::Rect vroi(cvRound(remapMat.Calib_Roi_L.x*sf), cvRound(remapMat.Calib_Roi_L.y*sf),
                                         cvRound(remapMat.Calib_Roi_L.width*sf), cvRound(remapMat.Calib_Roi_L.height*sf));
            cv::rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);

            //imgleft_c = imgleft(vroi);
            //imgleft = imgleft_c.clone();
            //cv::resize(imgleft, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
       }
#endif
   }
   if ( !remapMat.Calib_mX_R.empty() && !remapMat.Calib_mY_R.empty() )
   {
       cv::remap( imgright_c, imgright, remapMat.Calib_mX_R, remapMat.Calib_mY_R, cv::INTER_LINEAR );

#if DEBUG_SHOW
       cv::Mat canvasPart = canvas(cv::Rect(w, 0, w, h));
       cv::resize(imgright, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
       if(method != "RECTIFY_BOUGUET")
       {
           cv::Rect vroi(cvRound(remapMat.Calib_Roi_R.x*sf), cvRound(remapMat.Calib_Roi_R.y*sf),
                                         cvRound(remapMat.Calib_Roi_R.width*sf), cvRound(remapMat.Calib_Roi_R.height*sf));
            cv::rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
            //imgright_c = imgright(vroi);
            //imgright = imgright_c.clone();
            //cv::resize(imgright, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
       }
#endif
   }

#if DEBUG_SHOW
   for(int j = 0; j < canvas.rows; j += 16 )
        cv::line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);

   cv::imshow("rectify_left",imgleft);
   cv::imshow("rectify_right",imgright);
   cv::imshow("rectified_image", canvas);
   cv::waitKey(0);
#endif

   return 1;
}
