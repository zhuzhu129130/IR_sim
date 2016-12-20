#ifndef POSE2DTO2D_H
#define POSE2DTO2D_H

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Pose2Dto2D
{
public:
    Pose2Dto2D();

    void decomposeEssentialMat( InputArray _E, OutputArray _R1, OutputArray _R2, OutputArray _t );

    int recoverPose( InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                         OutputArray _t, double focal, Point2d pp=Point2d(0, 0), InputOutputArray _mask=noArray());

    cv::Mat findEssentialMat( InputArray _points1, InputArray _points2, double focal, Point2d pp);

    void find_feature_matches (
        const Mat& img_1, const Mat& img_2,
        std::vector<KeyPoint>& keypoints_1,
        std::vector<KeyPoint>& keypoints_2,
        std::vector< DMatch >& matches );

    void pose_estimation_2d2d (
        std::vector<KeyPoint> keypoints_1,
        std::vector<KeyPoint> keypoints_2,
        std::vector< DMatch > matches,
        Mat& R, Mat& t );

    // 像素坐标转相机归一化坐标
    Point2d pixel2cam ( const Point2d& p, const Mat& K );

    void triangulation (
        const vector<KeyPoint>& keypoint_1,
        const vector<KeyPoint>& keypoint_2,
        const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t,
        vector<Point3d>& points
    );

    int computRT(cv::Mat img_1,cv::Mat img_2);

public:
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    Mat R,t;
    vector<Point3d> points;
};

#endif // POSE2DTO2D_H
