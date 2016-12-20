#ifndef BASEFUNC_H
#define BASEFUNC_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include "opencv2/opencv.hpp"
#include "../../DataType.h"
#include "../../Variable.h"
using namespace std;

class basefunc
{
public:
    basefunc();
    bool loadImages(vector<string> fileList, vector<cv::Mat> &images);
    bool loadImageList(string file, vector<string> &list);

    void readFrame( int index, cv::Mat& img_left,cv::Mat& img_right,RunParams& runParams);

    void ConvertRaw12toRaw8(const cv::Mat& raw12, cv::Mat& raw8);

    void ConvertRGB12toBGR8(const cv::Mat& rgb12, cv::Mat& bgr8);

    void FixGroundtruth(cv::Mat& img_gt);
};

#endif // BASEFUNC_H
