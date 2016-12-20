#ifndef STEREORECTIFY_H
#define STEREORECTIFY_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/nonfree/nonfree.hpp>
#include "../basefunc/basefunc.h"
#include "../../DataType.h"
#include "../../Variable.h"

using namespace std;

/***
 *	双目校正的输入参数
 */
typedef struct _StereoParams_r
{
    cv::Size		imageSize;				// 图像分辨率
    cv::Mat       calib_M_L;
    cv::Mat       calib_D_L;
    cv::Mat       calib_M_R;
    cv::Mat       calib_D_R;

    cv::Mat			R;		// 旋转矩阵
    cv::Mat			T;	// 平移向量
    cv::Mat			E;		// 本质矩阵
    cv::Mat			F;	// 基础矩阵
}StereoParams_r;

/***
 *	双目校正的输出参数
 */
typedef struct _RemapMatrixs_r
{
    cv::Size		imageSize;				// 图像分辨率

    cv::Mat		Calib_mX_L;	// 左视图 X 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_mY_L;	// 左视图 Y 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_mX_R;	// 右视图 X 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_mY_R;	// 右视图 Y 方向畸变校正像素坐标映射矩阵
    cv::Mat		Calib_Q;		    // 用于计算三维点云的 Q 矩阵
    cv::Rect	Calib_Roi_L;	    // 左视图校正后的有效区域的矩形
    cv::Rect	Calib_Roi_R;	    // 右视图校正后的有效区域的矩形
    cv::Mat     Calib_Mask_Roi;		// 左视图校正后的有效区域
}RemapMatrixs_r;

struct CloudPoint {
    cv::Point3d pt;
    std::vector<int> imgpt_for_img;
    double reprojection_error;
};


class stereoRectifyy
{
public:
    stereoRectifyy();

    StereoParams_r stereoParams;
    RemapMatrixs_r remapMat;

    cv::Size imageSize;

    bool loadCalibDatas(string xmlFilePath);//功能 :导入双目相机标定参数

    int rectify(RunParams& runParams);
    int rectifySingleCamera();//功能 : 生成单个摄像头的校正矩阵
    int rectifyStereoCamera(string method);//功能 : 执行双目摄像机校正，生成双目校正数据

    int UnCalibRectify(cv::Mat& rgb1, cv::Mat& rgb2, RunParams& runParams);

    void extractfeature(cv::Mat &img_1,cv::Mat &img_2);//计算两幅图像的特征点，描述子

    void distance_Match(cv::Mat &rgb1,cv::Mat &rgb2, cv::Mat &desp1, cv::Mat &desp2, vector<cv::DMatch> &matches);

    void knnMatch(cv::Mat &rgb1,cv::Mat &rgb2, cv::Mat &desp1, cv::Mat &desp2,vector<cv::DMatch> &matches);

    void comput_F(cv::Mat &rgb1,cv::Mat &rgb2);

    bool FindCameraMatrices();

    bool DecomposeEtoRandT(cv::Mat_<double>& E,cv::Mat_<double>& R1,cv::Mat_<double>& R2,cv::Mat_<double>& t1,cv::Mat_<double>& t2);

    void DecomposeEssentialUsingHorn90(double _E[9], double _R1[9], double _R2[9], double _t1[3], double _t2[3]);

    void TakeSVDOfE(cv::Mat_<double>& E, cv::Mat& svd_u, cv::Mat& svd_vt, cv::Mat& svd_w);

    double TriangulatePoints(const vector<cv::KeyPoint>& pt_set1,
                            const vector<cv::KeyPoint>& pt_set2,
                            const cv::Matx34d& P,
                            const cv::Matx34d& P1,
                            vector<CloudPoint>& pointcloud);

    cv::Mat_<double> IterativeLinearLSTriangulation(cv::Point3d u,cv::Matx34d P,cv::Point3d u1,cv::Matx34d P1);

    cv::Mat_<double> LinearLSTriangulation(cv::Point3d u,cv::Matx34d P,cv::Point3d u1,cv::Matx34d P1);

    bool TestTriangulation(const vector<CloudPoint>& pcloud, const cv::Matx34d& P, vector<uchar>& status);

    std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts);

    void rectify_param();

    int remapImage(cv::Mat& img1, cv::Mat& img2, string method);//功能 : 对图像进行校正

    int saveStereoDatas(string filename, string method);//功能 :保存双目校正参数

public:

    // 声明特征提取器与描述子提取器
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _extractor;

    vector< cv::KeyPoint > kp1, kp2; //关键点
    cv::Mat desc1, desc2; // 计算描述子

    cv::FlannBasedMatcher matcher;
    //cv::BFMatcher matcher (cv::NORM_L2);

    vector<cv::Point2f> ps1, ps2;//Keypoints to points

    cv::Mat F, H1, H2, iM;

    vector<cv::Point2f> mInliner_1, mInliner_2;
    std::vector<cv::DMatch> InlinerMatches;

    vector<cv::DMatch> goodMatches;


};

#endif // STEREORECTIFY_H
