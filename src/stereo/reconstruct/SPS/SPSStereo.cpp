/*
    Copyright (C) 2014  Koichiro Yamaguchi

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SPSStereo.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <float.h>
#include "SGMStereo.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// Default parameters
const double SPSSTEREO_DEFAULT_OUTPUT_DISPARITY_FACTOR = 1024.0;
const int SPSSTEREO_DEFAULT_OUTER_ITERATION_COUNT = 10;
const int SPSSTEREO_DEFAULT_INNER_ITERATION_COUNT = 10;
const double SPSSTEREO_DEFAULT_POSITION_WEIGHT = 500.0;
const double SPSSTEREO_DEFAULT_DISPARITY_WEIGHT = 2000.0;
const double SPSSTEREO_DEFAULT_BOUNDARY_LENGTH_WEIGHT = 1500.0;
const double SPSSTEREO_DEFAULT_SMOOTHNESS_WEIGHT = 400.0;
const double SPSSTEREO_DEFAULT_INLIER_THRESHOLD = 3.0;
const double SPSSTEREO_DEFAULT_HINGE_PENALTY = 5.0;
const double SPSSTEREO_DEFAULT_OCCLUSION_PENALTY = 15.0;
const double SPSSTEREO_DEFAULT_IMPOSSIBLE_PENALTY = 30.0;

// Pixel offsets of 4- and 8-neighbors
const int fourNeighborTotal = 4;
const int fourNeighborOffsetX[4] = { -1, 0, 1, 0 };
const int fourNeighborOffsetY[4] = { 0, -1, 0, 1 };
const int eightNeighborTotal = 8;
const int eightNeighborOffsetX[8] = { -1, -1, 0, 1, 1, 1, 0, -1 };
const int eightNeighborOffsetY[8] = { 0, -1, -1, -1, 0, 1, 1, 1 };


SPSStereo::SPSStereo() : outputDisparityFactor_(SPSSTEREO_DEFAULT_OUTPUT_DISPARITY_FACTOR),
						 outerIterationTotal_(SPSSTEREO_DEFAULT_OUTER_ITERATION_COUNT),
						 innerIterationTotal_(SPSSTEREO_DEFAULT_INNER_ITERATION_COUNT),
						 positionWeight_(SPSSTEREO_DEFAULT_POSITION_WEIGHT),
						 disparityWeight_(SPSSTEREO_DEFAULT_DISPARITY_WEIGHT),
						 boundaryLengthWeight_(SPSSTEREO_DEFAULT_BOUNDARY_LENGTH_WEIGHT),
						 inlierThreshold_(SPSSTEREO_DEFAULT_INLIER_THRESHOLD),
						 hingePenalty_(SPSSTEREO_DEFAULT_HINGE_PENALTY),
						 occlusionPenalty_(SPSSTEREO_DEFAULT_OCCLUSION_PENALTY),
						 impossiblePenalty_(SPSSTEREO_DEFAULT_IMPOSSIBLE_PENALTY)
{
	smoothRelativeWeight_ = SPSSTEREO_DEFAULT_SMOOTHNESS_WEIGHT/SPSSTEREO_DEFAULT_DISPARITY_WEIGHT;
}

void SPSStereo::setOutputDisparityFactor(const double outputDisparityFactor) {
	if (outputDisparityFactor < 1) {
		throw std::invalid_argument("[SPSStereo::setOutputDisparityFactor] disparity factor is less than 1");
	}

	outputDisparityFactor_ = outputDisparityFactor;
}

void SPSStereo::setIterationTotal(const int outerIterationTotal, const int innerIterationTotal) {
	if (outerIterationTotal < 1 || innerIterationTotal < 1) {
		throw std::invalid_argument("[SPSStereo::setIterationTotal] the number of iterations is less than 1");
	}

	outerIterationTotal_ = outerIterationTotal;
	innerIterationTotal_ = innerIterationTotal;
}

void SPSStereo::setWeightParameter(const double positionWeight, const double disparityWeight, const double boundaryLengthWeight, const double smoothnessWeight) {
	if (positionWeight < 0 || disparityWeight < 0 || boundaryLengthWeight < 0 || smoothnessWeight < 0) {
		throw std::invalid_argument("[SPSStereo::setWeightParameter] weight value is nagative");
	}

	positionWeight_ = positionWeight;
	disparityWeight_ = disparityWeight;
	boundaryLengthWeight_ = boundaryLengthWeight;
	smoothRelativeWeight_ = smoothnessWeight/disparityWeight;
}

void SPSStereo::setInlierThreshold(const double inlierThreshold) {
	if (inlierThreshold <= 0) {
		throw std::invalid_argument("[SPSStereo::setInlierThreshold] threshold of inlier is less than zero");
	}

	inlierThreshold_ = inlierThreshold;
}

void SPSStereo::setPenaltyParameter(const double hingePenalty, const double occlusionPenalty, const double impossiblePenalty) {
	if (hingePenalty <= 0 || occlusionPenalty <= 0 || impossiblePenalty < 0) {
		throw std::invalid_argument("[SPSStereo::setPenaltyParameter] penalty value is less than zero");
	}
	if (hingePenalty >= occlusionPenalty) {
		throw std::invalid_argument("[SPSStereo::setPenaltyParameter] hinge penalty is larger than occlusion penalty");
	}

	hingePenalty_ = hingePenalty;
	occlusionPenalty_ = occlusionPenalty;
	impossiblePenalty_ = impossiblePenalty;
}

void SPSStereo::compute(const int superpixelTotal,
		const cv::Mat &leftImage,
		const cv::Mat &rightImage,
		const cv::Mat &segmentImage,
		 cv::Mat &disparityImage,
		std::vector<std::vector<double> > &disparityPlaneParameters,
		std::vector<std::vector<int> > &boundaryLabels)
{
	if (superpixelTotal < 2) {
		throw std::invalid_argument("[SPSStereo::compute] the number of superpixels is less than 2");
	}

	width_ = static_cast<int>(leftImage.cols);
	height_ = static_cast<int>(leftImage.rows);

    //给变量分配内存空间，包括inputLabImage_（w*h*3f），initialDisparityImage_（初始视差图像w*h f），labelImage_（每个像素位置表示所属的分割块），outlierFlagImage_（每个像素的局外点标志0,1），boundaryFlagImage_
	allocateBuffer();


    //对输入图像对使用SGM计算初始视差图initialDisparityImage_
	setInputData(leftImage, rightImage);

    //求出最佳的分割块，边缘标志图，局外点图
	initializeSegment(superpixelTotal);

    // 找到边缘，并判断边缘的类型，并根据这个类型对分割块的视差平面再次进行平滑
	performSmoothingSegmentation();

    //根据求出的视差平面估计每一个像素的视差
	makeOutputImage(segmentImage, disparityImage);

    //得出边缘数据
	makeSegmentBoundaryData(disparityPlaneParameters, boundaryLabels);

    freeBuffer();//清除分配的空间
}


void SPSStereo::allocateBuffer() {
	inputLabImage_ = reinterpret_cast<float*>(malloc(width_*height_*3*sizeof(float)));
	initialDisparityImage_ = reinterpret_cast<float*>(malloc(width_*height_*sizeof(float)));
	labelImage_ = reinterpret_cast<int*>(malloc(width_*height_*sizeof(int)));
	outlierFlagImage_ = reinterpret_cast<unsigned char*>(malloc(width_*height_*sizeof(unsigned char)));
	boundaryFlagImage_ = reinterpret_cast<unsigned char*>(malloc(width_*height_*sizeof(unsigned char)));
}

void SPSStereo::freeBuffer() {
	free(inputLabImage_);
	free(initialDisparityImage_);
	free(labelImage_);
	free(outlierFlagImage_);
	free(boundaryFlagImage_);
}

void SPSStereo::setInputData(const cv::Mat& leftImage, const cv::Mat& rightImage) {

    setLabImage(leftImage);//把RGB转换成LAB空间，只转化了左图
    computeInitialDisparityImage(leftImage, rightImage);//使用SGM计算了立体图像对的初始视差图
}

//把RGB转换成LAB空间，只转化了左图
void SPSStereo::setLabImage( const cv::Mat& leftImage) {
	std::vector<float> sRGBGammaCorrections(256);
	for (int pixelValue = 0; pixelValue < 256; ++pixelValue) {
		double normalizedValue = pixelValue/255.0;
		double transformedValue = (normalizedValue <= 0.04045) ? normalizedValue/12.92 : pow((normalizedValue+0.055)/1.055, 2.4);

		sRGBGammaCorrections[pixelValue] = static_cast<float>(transformedValue);
	}

	for (int y = 0; y < height_*width_*3; y=y+3) {



			float correctedR = sRGBGammaCorrections[leftImage.data[y]];
			float correctedG = sRGBGammaCorrections[leftImage.data[y+1]];
			float correctedB = sRGBGammaCorrections[leftImage.data[y+2]];

			float xyzColor[3];
			xyzColor[0] = correctedR*0.412453f + correctedG*0.357580f + correctedB*0.180423f;
			xyzColor[1] = correctedR*0.212671f + correctedG*0.715160f + correctedB*0.072169f;
			xyzColor[2] = correctedR*0.019334f + correctedG*0.119193f + correctedB*0.950227f;

			const double epsilon = 0.008856;
			const double kappa = 903.3;
			const double referenceWhite[3] = { 0.950456, 1.0, 1.088754 };

			float normalizedX = static_cast<float>(xyzColor[0]/referenceWhite[0]);
			float normalizedY = static_cast<float>(xyzColor[1]/referenceWhite[1]);
			float normalizedZ = static_cast<float>(xyzColor[2]/referenceWhite[2]);
			float fX = (normalizedX > epsilon) ? static_cast<float>(pow(normalizedX, 1.0/3.0)) : static_cast<float>((kappa*normalizedX + 16.0)/116.0);
			float fY = (normalizedY > epsilon) ? static_cast<float>(pow(normalizedY, 1.0/3.0)) : static_cast<float>((kappa*normalizedY + 16.0)/116.0);
			float fZ = (normalizedZ > epsilon) ? static_cast<float>(pow(normalizedZ, 1.0/3.0)) : static_cast<float>((kappa*normalizedZ + 16.0)/116.0);

			inputLabImage_[y] = static_cast<float>(116.0*fY - 16.0);
			inputLabImage_[y+1] = static_cast<float>(500.0*(fX - fY));
			inputLabImage_[y+2] = static_cast<float>(200.0*(fY - fZ));

	}
}

//使用SGM计算了立体图像对的初始视差图
void SPSStereo::computeInitialDisparityImage( const cv::Mat& leftImage, const cv::Mat& rightImage) {
	SGMStereo sgm;
	sgm.compute(leftImage, rightImage, initialDisparityImage_,height_,width_);
}

//求出最佳的分割块，边缘标志图，局外点图
void SPSStereo::initializeSegment(const int superpixelTotal) {
    //把整幅图进行超像素分割，划分成superpixelTotal份，把每个像素所属的分割块下标赋给每个像素，
    //得到初始labelImage_（就是每个像素位置表示所属的分割块），每个segments_的像素值（所有像素三通道各自的和）
	makeGridSegment(superpixelTotal);

    //求出最佳的分割块，以及最佳的边缘像素标志图
	assignLabel();

    //求出每个分割块的最佳视差平面系数，初始化局外点图
	initialFitDisparityPlane();
}

//把整幅图进行超像素分割，划分成superpixelTotal份，把每个像素所属的分割块下标赋给每个像素，
//得到初始labelImage_（就是每个像素位置表示所属的分割块）
void SPSStereo::makeGridSegment(const int superpixelTotal) {
	int imageSize = width_*height_;
	double gridSize = sqrt(static_cast<double>(imageSize)/superpixelTotal);
	stepSize_ = static_cast<int>(gridSize + 2.0);

    int segmentTotalX = static_cast<int>(ceil(width_/gridSize));//取大于等于参数的整数
	int segmentTotalY = static_cast<int>(ceil(height_/gridSize));

	segmentTotal_ = segmentTotalX*segmentTotalY;
	segments_.resize(segmentTotal_);
	for (int y = 0; y < height_; ++y) {
		int segmentIndexY = static_cast<int>(y/gridSize);
		for (int x = 0; x < width_; ++x) {
			int segmentIndexX = static_cast<int>(x/gridSize);
			int segmentIndex = segmentTotalX*segmentIndexY + segmentIndexX;

			labelImage_[width_*y + x] = segmentIndex;
			segments_[segmentIndex].addPixel(x, y, inputLabImage_[width_*3*y + 3*x], inputLabImage_[width_*3*y + 3*x + 1], inputLabImage_[width_*3*y + 3*x + 2]);
		}
	}

	memset(outlierFlagImage_, 0, width_*height_);
}

//求出最佳的分割块，以及最佳的边缘像素标志图
void SPSStereo::assignLabel() {
	std::stack<int> boundaryPixelIndices;
    //通过检查相邻点是否位于同一个分割块中，提取边缘像素.把边缘像素所在的位置放在boundaryPixelIndices栈中
    extractBoundaryPixel(boundaryPixelIndices);//同时把整幅图像每个像素的是不是边缘的标志（1,0）写进boundaryFlagImage_

	while (!boundaryPixelIndices.empty()) {
        int pixelIndex = boundaryPixelIndices.top();//从栈中取出一个边缘像素
        boundaryPixelIndices.pop();//把顶部像素弹出
        int pixelX = pixelIndex%width_;//得到初始边缘像素的位置横坐标
        int pixelY = pixelIndex/width_;//得到初始边缘像素的位置纵坐标
		boundaryFlagImage_[width_*pixelY + pixelX] = 0;// not a boundary

        //利用八邻域来判断当前边缘是不是真的边缘，是则跳过，不是则判断当前像素所属的最好的分割块
        if (isUnchangeable(pixelX, pixelY)) continue;//如果当前像素的八邻域中一个像素与自己是同一个分割块，
                                                                                   //下一个邻域的像素不在同一个分割块，则就是不可改变的。

        //这里是分别比较当前像素到四邻域像素所在的分割块的距离能量，找到当前像素所属的最佳分割块，以及最佳能量值
		int bestSegmentIndex = findBestSegmentLabel(pixelX, pixelY);
        if (bestSegmentIndex == labelImage_[width_*pixelY + pixelX]) continue;//若边缘点的最佳分割块号与初始分割块号一致，则continue

        //若像素换了新的分割块，则原来的分割块移除该像素，新的分割块增加该像素
		changeSegmentLabel(pixelX, pixelY, bestSegmentIndex);
        //若像素换了新的分割块，判断当前边缘像素的邻域是不是边缘像素
		addNeighborBoundaryPixel(pixelX, pixelY, boundaryPixelIndices);
	}
}

//通过四邻域像素和当前像素是否在同一个分割块中判断是否边缘像素，并把所有像素是不是边缘写入boundaryFlagImage_
void SPSStereo::extractBoundaryPixel(std::stack<int>& boundaryPixelIndices) {
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
            if (isBoundaryPixel(x, y)) {//通过四邻域像素和当前像素是否在同一个分割块中判断是否边缘像素
				boundaryPixelIndices.push(width_*y + x);
				boundaryFlagImage_[width_*y + x] = 1;
			} else {
				boundaryFlagImage_[width_*y + x] = 0;
			}
		}
	}
}

//通过四邻域像素和当前像素是否在同一个分割块中判断是否边缘像素
bool SPSStereo::isBoundaryPixel(const int x, const int y) const {
	int pixelSegmentIndex = labelImage_[width_*y + x];
	for (int neighborIndex = 0; neighborIndex < fourNeighborTotal; ++neighborIndex) {
		int neighborX = x + fourNeighborOffsetX[neighborIndex];
		if (neighborX < 0 || neighborX >= width_) continue;
		int neighborY = y + fourNeighborOffsetY[neighborIndex];
		if (neighborY < 0 || neighborY >= height_) continue;

		if (labelImage_[width_*neighborY + neighborX] != pixelSegmentIndex) return true;
	}

	return false;
}

//如果当前像素的八邻域中一个像素与自己是同一个分割块，
//下一个邻域的像素不在同一个分割块，则就是不可改变的。
bool SPSStereo::isUnchangeable(const int x, const int y) const {
    int pixelSegmentIndex = labelImage_[width_*y + x];//当前边缘像素所属的分割块

	int outCount = 0;
	for (int neighborIndex = 0; neighborIndex < eightNeighborTotal; ++neighborIndex) {
		int neighborX = x + eightNeighborOffsetX[neighborIndex];
		if (neighborX < 0 || neighborX >= width_) continue;
		int neighborY = y + eightNeighborOffsetY[neighborIndex];
		if (neighborY < 0 || neighborY >= height_) continue;

		if (labelImage_[width_*neighborY + neighborX] != pixelSegmentIndex) continue;

		int nextNeighborIndex = neighborIndex + 1;
		if (nextNeighborIndex >= eightNeighborTotal) nextNeighborIndex = 0;

		int nextNeighborX = x + eightNeighborOffsetX[nextNeighborIndex];
		if (nextNeighborX < 0 || nextNeighborX >= width_) { ++outCount; continue; }
		int nextNeighborY = y + eightNeighborOffsetY[nextNeighborIndex];
		if (nextNeighborY < 0 || nextNeighborY >= height_) { ++outCount; continue; }

		if (labelImage_[width_*nextNeighborY + nextNeighborX] != pixelSegmentIndex) ++outCount;
	}
	if (outCount > 1) return true;

	return false;
}

//这里是分别比较当前像素到四邻域像素所在的分割块的距离能量，找到当前像素所属的最佳分割块，以及最佳能量值
int SPSStereo::findBestSegmentLabel(const int x, const int y) const {
	int bestSegmentIndex = -1;
	double bestEnergy = DBL_MAX;

    //这里是分别比较当前像素到四邻域像素所在的分割块的距离能量，找到当前像素所属的最佳分割块，以及最佳能量值
    std::vector<int> neighborSegmentIndices = getNeighborSegmentIndices(x, y);//得到四邻域像素所属的分割块
	for (int neighborIndex = 0; neighborIndex < static_cast<int>(neighborSegmentIndices.size()); ++neighborIndex) {
		int segmentIndex = neighborSegmentIndices[neighborIndex];

        double pixelEnergy = computePixelEnergy(x, y, segmentIndex);//计算像素颜色位置视差加权能量
        double boundaryLengthEnergy = computeBoundaryLengthEnergy(x, y, segmentIndex);//求出像素八邻域的像素与当前像素不在同一个分割块的数量

        double totalEnergy = pixelEnergy + boundaryLengthEnergy;//像素能量与边缘能量相加
		if (totalEnergy < bestEnergy) {
			bestSegmentIndex = segmentIndex;
			bestEnergy = totalEnergy;
		}
	}
	//on retourne le segment qui a le moins d'energie ( calculé avec la diff entre l'inputLabImage et l'image du segment )
	return bestSegmentIndex;
}

//得到四邻域像素所属的分割块
std::vector<int> SPSStereo::getNeighborSegmentIndices(const int x, const int y) const {
	std::vector<int> neighborSegmentIndices(1);
	neighborSegmentIndices[0] = labelImage_[width_*y + x];
	for (int neighborIndex = 0; neighborIndex < fourNeighborTotal; ++neighborIndex) {
		int neighborX = x + fourNeighborOffsetX[neighborIndex];
		if (neighborX < 0 || neighborX >= width_) continue;
		int neighborY = y + fourNeighborOffsetY[neighborIndex];
		if (neighborY < 0 || neighborY >= height_) continue;

		bool newSegmentFlag = true;
		for (int listIndex = 0; listIndex < static_cast<int>(neighborSegmentIndices.size()); ++listIndex) {
			if (labelImage_[width_*neighborY + neighborX] == neighborSegmentIndices[listIndex]) {
				newSegmentFlag = false;
				break;
			}
		}

		if (newSegmentFlag) neighborSegmentIndices.push_back(labelImage_[width_*neighborY + neighborX]);
	}

	return neighborSegmentIndices;
}

//求出像素颜色位置视差与所属分割块距离的能量
double SPSStereo::computePixelEnergy(const int x, const int y, const int segmentIndex) const {
	const double normalizedPositionWeight = positionWeight_/(stepSize_*stepSize_);
	const double inlierThresholdSquare = inlierThreshold_*inlierThreshold_;

    //像素颜色距离分割块颜色的平方和
	double segmentL = segments_[segmentIndex].color(0);
	double segmentA = segments_[segmentIndex].color(1);
	double segmentB = segments_[segmentIndex].color(2);
	double distanceColor = (inputLabImage_[width_*3*y + 3*x] - segmentL)*(inputLabImage_[width_*3*y + 3*x] - segmentL)
		+ (inputLabImage_[width_*3*y + 3*x + 1] - segmentA)*(inputLabImage_[width_*3*y + 3*x + 1] - segmentA)
		+ (inputLabImage_[width_*3*y + 3*x + 2] - segmentB)*(inputLabImage_[width_*3*y + 3*x + 2] - segmentB);

    //像素位置距离分割块中心位置的平方和
	double segmentX = segments_[segmentIndex].position(0);
	double segmentY = segments_[segmentIndex].position(1);
	double distancePosition = (x - segmentX)*(x - segmentX) + (y - segmentY)*(y - segmentY);

    //视差距离
	double distanceDisparity = inlierThresholdSquare;
	double estimatedDisparity = segments_[segmentIndex].estimatedDisparity(x, y);
	if (estimatedDisparity > 0) {
		distanceDisparity = (initialDisparityImage_[width_*y + x] - estimatedDisparity)*(initialDisparityImage_[width_*y + x] - estimatedDisparity);
		if (distanceDisparity > inlierThresholdSquare) distanceDisparity = inlierThresholdSquare;
	}

    //求出像素加权能量
	double pixelEnergy = distanceColor + normalizedPositionWeight*distancePosition + disparityWeight_*distanceDisparity;

	return pixelEnergy;
}

//求出像素八邻域的像素与当前像素不在同一个分割块的数量
double SPSStereo::computeBoundaryLengthEnergy(const int x, const int y, const int segmentIndex) const {
	int boundaryCount = 0;
	for (int neighborIndex = 0; neighborIndex < eightNeighborTotal; ++neighborIndex) {
		int neighborX = x + eightNeighborOffsetX[neighborIndex];
		if (neighborX < 0 || neighborX >= width_) continue;
		int neighborY = y + eightNeighborOffsetY[neighborIndex];
		if (neighborY < 0 || neighborY >= height_) continue;

		if (labelImage_[width_*neighborY + neighborX] != segmentIndex) ++boundaryCount;
	}

	return boundaryLengthWeight_*boundaryCount;
}

//若像素换了新的分割块，则原来的分割块移除该像素，新的分割块增加该像素，若边缘像素在新的分割块中的估计视差与初始视差相差太多，就是局外点。
void SPSStereo::changeSegmentLabel(const int x, const int y, const int newSegmentIndex) {
	int previousSegmentIndex = labelImage_[width_*y + x];
	labelImage_[width_*y + x] = newSegmentIndex;

	double estimatedDisparity = segments_[newSegmentIndex].estimatedDisparity(x, y);
	double disparityError = fabs(initialDisparityImage_[width_*y + x] - estimatedDisparity);
	if (disparityError > inlierThreshold_) outlierFlagImage_[width_*y + x] = 255;
	else outlierFlagImage_[width_*y + x] = 0;

	float pixelL = inputLabImage_[width_*y + x];
	float pixelA = inputLabImage_[width_*y + x + 1];
	float pixelB = inputLabImage_[width_*y + x + 2];

	segments_[previousSegmentIndex].removePixel(x, y, pixelL, pixelA, pixelB);
	segments_[newSegmentIndex].addPixel(x, y, pixelL, pixelA, pixelB);
}

//若像素换了新的分割块，判断当前边缘像素的邻域是不是边缘像素
void SPSStereo::addNeighborBoundaryPixel(const int x, const int y, std::stack<int>& boundaryPixelIndices) const {
	for (int neighorPixelIndex = 0; neighorPixelIndex < fourNeighborTotal; ++neighorPixelIndex) {
		int neighborX = x + fourNeighborOffsetX[neighorPixelIndex];
		if (neighborX < 0 || neighborX >= width_) continue;
		int neighborY = y + fourNeighborOffsetY[neighorPixelIndex];
		if (neighborY < 0 || neighborY >= height_) continue;

		if (boundaryFlagImage_[width_*neighborY + neighborX] > 0) continue;

		if (isBoundaryPixel(neighborX, neighborY)) {
			boundaryPixelIndices.push(width_*neighborY + neighborX);
			boundaryFlagImage_[width_*neighborY + neighborX] = 1;
		}
	}
}

//求出每个分割块的最佳视差平面系数，初始化局外点图
void SPSStereo::initialFitDisparityPlane() {
    //使用RANSAC方法求出所有分割块视差平面系数。逐渐的逼近，使内点最多，局外点更少d=Ax+By+C
	estimateDisparityPlaneRANSAC(initialDisparityImage_);

	float* interpolatedDisparityImage = reinterpret_cast<float*>(malloc(width_*height_*sizeof(float)));

    //对初始视差平面进行插值，//解决像素视差小于等于0的问题，取水平线上一段像素视差的最小值
    interpolateDisparityImage(interpolatedDisparityImage);

    //使用RANSAC方法求出所有分割块插值后的视差平面系数。逐渐的逼近，使内点最多，局外点更少d=Ax+By+C
	estimateDisparityPlaneRANSAC(interpolatedDisparityImage);
	free(interpolatedDisparityImage);

    //初始化局外点图，初始视差为0的点是局外点，估计的视差与初始视差差异大的也为局外点
	initializeOutlierFlagImage();
}

//使用RANSAC方法求出所有分割块平面系数。逐渐的逼近，使内点最多，局外点更少d=Ax+By+C
void SPSStereo::estimateDisparityPlaneRANSAC(const float* disparityImage) {
	const double confidenceLevel = 0.99;
	const double initialInlierThreshold = 1.0;

	std::vector< std::vector<int> > segmentPixelXs(segmentTotal_);
	std::vector< std::vector<int> > segmentPixelYs(segmentTotal_);
	std::vector< std::vector<float> > segmentPixelDisparities(segmentTotal_);
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
			if (disparityImage[width_*y + x] == 0) continue;

			int pixelSegmentIndex = labelImage_[width_*y + x];
			segmentPixelXs[pixelSegmentIndex].push_back(x);
			segmentPixelYs[pixelSegmentIndex].push_back(y);
			segmentPixelDisparities[pixelSegmentIndex].push_back(disparityImage[width_*y + x]);
		}
	}

	for (int segmentIndex = 0; segmentIndex < segmentTotal_; ++segmentIndex) {
        if (segments_[segmentIndex].hasDisparityPlane()) continue;//若计算过分割块视差平面系数就跳过

		int segmentPixelTotal = static_cast<int>(segmentPixelXs[segmentIndex].size());
		if (segmentPixelTotal < 3) continue;

		int bestInlierTotal = 0;
		std::vector<bool> bestInlierFlags(segmentPixelTotal);
		int samplingTotal = segmentPixelTotal;
		int samplingCount = 0;
		while (samplingCount < samplingTotal) {
			int drawIndices[3];
			drawIndices[0] = rand()%segmentPixelTotal;
			drawIndices[1] = rand()%segmentPixelTotal;
			while (drawIndices[1] == drawIndices[0]) drawIndices[1] = rand()%segmentPixelTotal;
			drawIndices[2] = rand()%segmentPixelTotal;
			while (drawIndices[2] == drawIndices[0] || drawIndices[2] == drawIndices[1]) drawIndices[2] = rand()%segmentPixelTotal;

			std::vector<double> planeParameter;
			solvePlaneEquations(segmentPixelXs[segmentIndex][drawIndices[0]], segmentPixelYs[segmentIndex][drawIndices[0]], 1, segmentPixelDisparities[segmentIndex][drawIndices[0]],
								segmentPixelXs[segmentIndex][drawIndices[1]], segmentPixelYs[segmentIndex][drawIndices[1]], 1, segmentPixelDisparities[segmentIndex][drawIndices[1]],
								segmentPixelXs[segmentIndex][drawIndices[2]], segmentPixelYs[segmentIndex][drawIndices[2]], 1, segmentPixelDisparities[segmentIndex][drawIndices[2]],
								planeParameter);

			// Count the number of inliers
			int inlierTotal = 0;
			std::vector<bool> inlierFlags(segmentPixelTotal);
			for (int pixelIndex = 0; pixelIndex < segmentPixelTotal; ++pixelIndex) {
				double estimateDisparity = planeParameter[0]*segmentPixelXs[segmentIndex][pixelIndex]
					+ planeParameter[1]*segmentPixelYs[segmentIndex][pixelIndex]
					+ planeParameter[2];
				if (fabs(estimateDisparity - segmentPixelDisparities[segmentIndex][pixelIndex]) <= initialInlierThreshold) {
					++inlierTotal;
					inlierFlags[pixelIndex] = true;
				} else {
					inlierFlags[pixelIndex] = false;
				}
			}

			if (inlierTotal > bestInlierTotal) {
				bestInlierTotal = inlierTotal;
				bestInlierFlags = inlierFlags;

				samplingTotal = computeRequiredSamplingTotal(3, bestInlierTotal, segmentPixelTotal, samplingTotal, confidenceLevel);
			}

			++samplingCount;
		}

		double sumXSqr = 0, sumYSqr = 0, sumXY = 0, sumX = 0, sumY = 0;
		double sumXD = 0, sumYD = 0, sumD = 0;
		int inlierIndex = 0;
		for (int pixelIndex = 0; pixelIndex < segmentPixelTotal; ++pixelIndex) {
			if (bestInlierFlags[pixelIndex]) {
				int x = segmentPixelXs[segmentIndex][pixelIndex];
				int y = segmentPixelYs[segmentIndex][pixelIndex];
				float d = segmentPixelDisparities[segmentIndex][pixelIndex];

				sumXSqr += x*x;
				sumYSqr += y*y;
				sumXY += x*y;
				sumX += x;
				sumY += y;
				sumXD += x*d;
				sumYD += y*d;
				sumD += d;
				++inlierIndex;
			}
		}
		std::vector<double> planeParameter(3);
		solvePlaneEquations(sumXSqr, sumXY, sumX, sumXD,
							sumXY, sumYSqr, sumY, sumYD,
							sumX, sumY, inlierIndex, sumD,
							planeParameter);

		segments_[segmentIndex].setDisparityPlane(planeParameter[0], planeParameter[1], planeParameter[2]);
	}
}

void SPSStereo::solvePlaneEquations(const double x1, const double y1, const double z1, const double d1,
									const double x2, const double y2, const double z2, const double d2,
									const double x3, const double y3, const double z3, const double d3,
									std::vector<double>& planeParameter) const
{
	const double epsilonValue = 1e-10;

	planeParameter.resize(3);

	double denominatorA = (x1*z2 - x2*z1)*(y2*z3 - y3*z2) - (x2*z3 - x3*z2)*(y1*z2 - y2*z1);
	if (denominatorA < epsilonValue) {
		planeParameter[0] = 0.0;
		planeParameter[1] = 0.0;
		planeParameter[2] = -1.0;
		return;
	}

	planeParameter[0] = ((z2*d1 - z1*d2)*(y2*z3 - y3*z2) - (z3*d2 - z2*d3)*(y1*z2 - y2*z1))/denominatorA;

	double denominatorB = y1*z2 - y2*z1;
	if (denominatorB > epsilonValue) {
		planeParameter[1] = (z2*d1 - z1*d2 - planeParameter[0]*(x1*z2 - x2*z1))/denominatorB;
	} else {
		denominatorB = y2*z3 - y3*z2;
		planeParameter[1] = (z3*d2 - z2*d3 - planeParameter[0]*(x2*z3 - x3*z2))/denominatorB;
	}
	if (z1 > epsilonValue) {
		planeParameter[2] = (d1 - planeParameter[0]*x1 - planeParameter[1]*y1)/z1;
	} else if (z2 > epsilonValue) {
		planeParameter[2] = (d2 - planeParameter[0]*x2 - planeParameter[1]*y2)/z2;
	} else {
		planeParameter[2] = (d3 - planeParameter[0]*x3 - planeParameter[1]*y3)/z3;
	}
}

int SPSStereo::computeRequiredSamplingTotal(const int drawTotal, const int inlierTotal, const int pointTotal, const int currentSamplingTotal, const double confidenceLevel) const {
	double ep = 1 - static_cast<double>(inlierTotal)/static_cast<double>(pointTotal);
	if (ep == 1.0) {
		ep = 0.5;
	}

	int newSamplingTotal = static_cast<int>(log(1 - confidenceLevel)/log(1 - pow(1 - ep, drawTotal)) + 0.5);
	if (newSamplingTotal < currentSamplingTotal) {
		return newSamplingTotal;
	} else {
		return currentSamplingTotal;
	}
}

//对初始视差平面进行插值，//解决像素视差小于等于0的问题，取水平线上一段像素视差的最小值
void SPSStereo::interpolateDisparityImage(float* interpolatedDisparityImage) const {
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
			interpolatedDisparityImage[width_*y + x] = initialDisparityImage_[width_*y + x];
		}
	}

	for (int y = 0; y < height_; ++y) {
		int count = 0;
		for (int x = 0; x < width_; ++x) {
            if (interpolatedDisparityImage[width_*y + x] > 0) {
                if (count >= 1) {
					int startX = x - count;
					int endX = x - 1;

					if (startX > 0 && endX < width_ - 1) {
						float interpolationDisparity = std::min(interpolatedDisparityImage[width_*y + startX - 1], interpolatedDisparityImage[width_*y + endX + 1]);
						for (int interpolateX = startX; interpolateX <= endX; ++interpolateX) {
							interpolatedDisparityImage[width_*y + interpolateX] = interpolationDisparity;
						}
					}
				}

				count = 0;
			} else {
				++count;
			}
		}

		for (int x = 0; x < width_; ++x) {
			if (interpolatedDisparityImage[width_*y + x] > 0) {
				for (int interpolateX = 0; interpolateX < x; ++interpolateX) {
					interpolatedDisparityImage[width_*y + interpolateX] = interpolatedDisparityImage[width_*y + x];
				}
				break;
			}
		}

		for (int x = width_ - 1; x >= 0; --x) {
			if (interpolatedDisparityImage[width_*y + x] > 0) {
				for (int interpolateX = x + 1; interpolateX < width_; ++interpolateX) {
					interpolatedDisparityImage[width_*y + interpolateX] = interpolatedDisparityImage[width_*y + x];
				}
				break;
			}
		}
	}

	for (int x = 0; x < width_; ++x) {
		for (int y = 0; y < height_; ++y) {
			if (interpolatedDisparityImage[width_*y + x] > 0) {
				for (int interpolateY = 0; interpolateY < y; ++interpolateY) {
					interpolatedDisparityImage[width_*interpolateY + x] = interpolatedDisparityImage[width_*y + x];
				}
				break;
			}
		}

        // 从下方开始
		for (int y = height_ - 1; y >= 0; --y) {
			if (interpolatedDisparityImage[width_*y + x] > 0) {
				for (int interpolateY = y+1; interpolateY < height_; ++interpolateY) {
					interpolatedDisparityImage[width_*interpolateY + x] = interpolatedDisparityImage[width_*y + x];
				}
				break;
			}
		}
	}
}

//初始化局外点图，初始视差为0的点是局外点，估计的视差与初始视差差异大的也为局外点
void SPSStereo::initializeOutlierFlagImage() {
	memset(outlierFlagImage_, 0, width_*height_);
	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
			if (initialDisparityImage_[width_*y + x] == 0) {
				outlierFlagImage_[width_*y + x] = 255;
				continue;
			}

			int pixelSegmentIndex = labelImage_[width_*y + x];
			double estimatedDisparity = segments_[pixelSegmentIndex].estimatedDisparity(x, y);
			if (fabs(initialDisparityImage_[width_*y + x] - estimatedDisparity) > inlierThreshold_) {
				outlierFlagImage_[width_*y + x] = 255;
			}
		}
	}
}

void SPSStereo::performSmoothingSegmentation() {
	for (int outerIterationCount = 0; outerIterationCount < outerIterationTotal_; ++outerIterationCount) {
        //求出最佳的分割块，以及最佳的边缘像素标志图
		assignLabel();

		//pour chaque pixel, on regarde les boundary et on les assignes aux segments.
		buildSegmentConfiguration();

        //设置边缘类型，并对视差平面进行再次平滑
		planeSmoothing();
	}
}

void SPSStereo::buildSegmentConfiguration() {
	//init config of each segments
	for (int segmentIndex = 0; segmentIndex < segmentTotal_; ++segmentIndex) {
		segments_[segmentIndex].clearConfiguration();
	}
	//init again
	boundaries_.clear();
    boundaryIndexMatrix_.resize(segmentTotal_);//分配分割块总数的容器空间
	for (int i = 0; i < segmentTotal_; ++i) {
        boundaryIndexMatrix_[i].resize(segmentTotal_);//并对容器内嵌套的容器分配分割块总数个int，并初始化为-1
		for (int j = 0; j < segmentTotal_; ++j) boundaryIndexMatrix_[i][j] = -1;
	}//init done

	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
            int pixelSegmentIndex = labelImage_[width_*y + x];//获得当前像素所属的分割块号
            segments_[pixelSegmentIndex].appendSegmentPixel(x, y);//多项式系数为所有点（x，y）的加和
			//si le pixel a une disparité et n'est pas un outlier, on remplit les coeff polynomiale subatomique euclidien non gaussien
            if (initialDisparityImage_[width_*y + x] > 0 && outlierFlagImage_[width_*y + x] == 0) {//若初始视差大于0且不是局外点
                segments_[pixelSegmentIndex].appendSegmentPixelWithDisparity(x, y, initialDisparityImage_[width_*y + x]);//另一个多项式参数的求解加和
			}
			//on ajoute les boundaryIndex aux segments concernés et dans boundaries on met des coeff
            if (isHorizontalBoundary(x, y)) {//如果是水平边缘，就是当前像素与水平方向+1的像素的所属分割块不同。
                int neighborSegmentIndex = labelImage_[width_*y + x + 1];//水平相邻像素+1位置所属的分割块号
				int boundaryIndex = appendBoundary(pixelSegmentIndex, neighborSegmentIndex);
                boundaries_[boundaryIndex].appendBoundaryPixel(x + 0.5, y);//这个边缘增加一个像素，这个像素位置在原来边缘加0.5像素
			}
			if (isVerticalBoundary(x, y)) {
				int neighborSegmentIndex = labelImage_[width_*(y + 1) + x];
				int boundaryIndex = appendBoundary(pixelSegmentIndex, neighborSegmentIndex);
				boundaries_[boundaryIndex].appendBoundaryPixel(x, y + 0.5);
			}
		}
	}
}

bool SPSStereo::isHorizontalBoundary(const int x, const int y) const {
	if (x >= width_ - 1) return false;

	if (labelImage_[width_*y + x] != labelImage_[width_*y + x + 1]) return true;
	return false;
}

bool SPSStereo::isVerticalBoundary(const int x, const int y) const {
	if (y >= height_ - 1) return false;

	if (labelImage_[width_*y + x] != labelImage_[width_*(y + 1) + x]) return true;
	return false;
}

//
int SPSStereo::appendBoundary(const int firstSegmentIndex, const int secondSegmentIndex) {
	if (boundaryIndexMatrix_[firstSegmentIndex][secondSegmentIndex] >= 0) return boundaryIndexMatrix_[firstSegmentIndex][secondSegmentIndex];

    boundaries_.push_back(Boundary(firstSegmentIndex, secondSegmentIndex));//感觉像是两个相邻分割块的标号
	int newBoundaryIndex = static_cast<int>(boundaries_.size()) - 1;
    boundaryIndexMatrix_[firstSegmentIndex][secondSegmentIndex] = newBoundaryIndex;//这是赋给这俩相邻分割块的边缘的标号
    boundaryIndexMatrix_[secondSegmentIndex][firstSegmentIndex] = newBoundaryIndex;//意思是这俩分割块拥有这个标号的边缘

    segments_[firstSegmentIndex].appendBoundaryIndex(newBoundaryIndex);//这个分割块拥有这个边缘
	segments_[secondSegmentIndex].appendBoundaryIndex(newBoundaryIndex);

    return newBoundaryIndex;//返回这个当前边缘标号
}

void SPSStereo::planeSmoothing() {
	for (int innerIterationCount = 0; innerIterationCount < innerIterationTotal_; ++innerIterationCount) {
        //判断边缘类型，遮挡点，或是其他( 1 2 ou 3 )
        estimateBoundaryLabel();
        //利用上面判断的边缘类型再次对分割块的视差平面的参数进行平滑
        estimateSmoothFitting();
	}
}

void SPSStereo::estimateBoundaryLabel() {
	int boundaryTotal = static_cast<int>(boundaries_.size());
	for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
		Boundary& currentBoundary = boundaries_[boundaryIndex];
		int firstSegmentIndex = currentBoundary.segmentIndex(0);
		int secondSegmentIndex = currentBoundary.segmentIndex(1);
		Segment& firstSegment = segments_[firstSegmentIndex];
		Segment& secondSegment = segments_[secondSegmentIndex];

		double ai = firstSegment.planeParameter(0);
		double bi = firstSegment.planeParameter(1);
		double ci = firstSegment.planeParameter(2);
		double aj = secondSegment.planeParameter(0);
		double bj = secondSegment.planeParameter(1);
		double cj = secondSegment.planeParameter(2);

		std::vector<double> boundaryEnergies(4);

		// Hinge
		double hingeSquaredError = 0;
		double hingeError = 0;
		hingeError = (ai - aj)*currentBoundary.polynomialCoefficient(3)
			+ (bi - bj)*currentBoundary.polynomialCoefficient(4)
			+ (ci - cj)*currentBoundary.polynomialCoefficient(5);
		hingeSquaredError = currentBoundary.polynomialCoefficient(0)*(ai*ai + aj*aj - 2*ai*aj)
			+ currentBoundary.polynomialCoefficient(1)*(bi*bi + bj*bj - 2*bi*bj)
			+ currentBoundary.polynomialCoefficient(2)*(2*ai*bi + 2*aj*bj - 2*ai*bj -2*aj*bi)
			+ currentBoundary.polynomialCoefficient(3)*(2*ai*ci + 2*aj*cj - 2*ai*cj -2*aj*ci)
			+ currentBoundary.polynomialCoefficient(4)*(2*bi*ci + 2*bj*cj - 2*bi*cj -2*bj*ci)
			+ currentBoundary.polynomialCoefficient(5)*(ci*ci + cj*cj - 2*ci*cj);
		hingeSquaredError /= currentBoundary.boundaryPixelTotal();
		boundaryEnergies[2] = hingePenalty_ + hingeSquaredError;

		// Occlusion
		if (hingeError > 0) {
			boundaryEnergies[0] = occlusionPenalty_;
			boundaryEnergies[1] = occlusionPenalty_ + impossiblePenalty_;
		} else {
			boundaryEnergies[0] = occlusionPenalty_ + impossiblePenalty_;
			boundaryEnergies[1] = occlusionPenalty_;
		}

		// Coplanar
		double coplanarSquaredError = 0;
		coplanarSquaredError = firstSegment.polynomialCoefficientAll(0)*(ai*ai + aj*aj - 2*ai*aj)
			+ firstSegment.polynomialCoefficientAll(1)*(bi*bi + bj*bj - 2*bi*bj)
			+ firstSegment.polynomialCoefficientAll(2)*(2*ai*bi + 2*aj*bj - 2*ai*bj -2*aj*bi)
			+ firstSegment.polynomialCoefficientAll(3)*(2*ai*ci + 2*aj*cj - 2*ai*cj -2*aj*ci)
			+ firstSegment.polynomialCoefficientAll(4)*(2*bi*ci + 2*bj*cj - 2*bi*cj -2*bj*ci)
			+ firstSegment.polynomialCoefficientAll(5)*(ci*ci + cj*cj - 2*ci*cj);
		coplanarSquaredError += secondSegment.polynomialCoefficientAll(0)*(ai*ai + aj*aj - 2*ai*aj)
			+ secondSegment.polynomialCoefficientAll(1)*(bi*bi + bj*bj - 2*bi*bj)
			+ secondSegment.polynomialCoefficientAll(2)*(2*ai*bi + 2*aj*bj - 2*ai*bj -2*aj*bi)
			+ secondSegment.polynomialCoefficientAll(3)*(2*ai*ci + 2*aj*cj - 2*ai*cj -2*aj*ci)
			+ secondSegment.polynomialCoefficientAll(4)*(2*bi*ci + 2*bj*cj - 2*bi*cj -2*bj*ci)
			+ secondSegment.polynomialCoefficientAll(5)*(ci*ci + cj*cj - 2*ci*cj);
		coplanarSquaredError /= (firstSegment.pixelTotal() + secondSegment.pixelTotal());
		boundaryEnergies[3] = coplanarSquaredError;

		int minBoundaryLabel = 0;
		if (boundaryEnergies[1] < boundaryEnergies[minBoundaryLabel]) minBoundaryLabel = 1;
		if (boundaryEnergies[2] < boundaryEnergies[minBoundaryLabel]) minBoundaryLabel = 2;
		if (boundaryEnergies[3] < boundaryEnergies[minBoundaryLabel]) minBoundaryLabel = 3;

		boundaries_[boundaryIndex].setType(minBoundaryLabel);
	}
}

void SPSStereo::estimateSmoothFitting() {
	for (int segmentIndex = 0; segmentIndex < segmentTotal_; ++segmentIndex) {
		Segment currentSegment = segments_[segmentIndex];
		int segmentPixelTotal = currentSegment.pixelTotal();
		int disparityPixelTotal = 0;

		double sumXSqr = 0, sumYSqr = 0, sumXY = 0, sumX = 0, sumY = 0;
		double sumXD = 0, sumYD = 0, sumD = 0;
		double pointTotal = 0;

		sumXSqr += currentSegment.polynomialCoefficient(0);
		sumYSqr += currentSegment.polynomialCoefficient(1);
		sumXY += currentSegment.polynomialCoefficient(2);
		sumX += currentSegment.polynomialCoefficient(3);
		sumY += currentSegment.polynomialCoefficient(4);
		sumXD += currentSegment.polynomialCoefficient(5);
		sumYD += currentSegment.polynomialCoefficient(6);
		sumD += currentSegment.polynomialCoefficient(7);
		pointTotal += currentSegment.polynomialCoefficient(8);

		disparityPixelTotal += static_cast<int>(currentSegment.polynomialCoefficient(8));

		for (int neighborIndex = 0; neighborIndex < currentSegment.boundaryTotal(); ++neighborIndex) {
			int boundaryIndex = currentSegment.boundaryIndex(neighborIndex);
			int boundaryLabel = boundaries_[boundaryIndex].type();
			if (boundaryLabel < 2) continue;

			Boundary& currentBoundary = boundaries_[boundaryIndex];
			int neighborSegmentIndex = currentBoundary.segmentIndex(0);
			if (neighborSegmentIndex == segmentIndex) neighborSegmentIndex = currentBoundary.segmentIndex(1);
			Segment& neighborSegment = segments_[neighborSegmentIndex];

			if (boundaryLabel == 2) {
				// Hinge
				int boundaryPixelTotal = currentBoundary.boundaryPixelTotal();
				double weightValue = smoothRelativeWeight_/boundaryPixelTotal*stepSize_*stepSize_;

				sumXSqr += weightValue*currentBoundary.polynomialCoefficient(0);
				sumYSqr += weightValue*currentBoundary.polynomialCoefficient(1);
				sumXY += weightValue*currentBoundary.polynomialCoefficient(2);
				sumX += weightValue*currentBoundary.polynomialCoefficient(3);
				sumY += weightValue*currentBoundary.polynomialCoefficient(4);
				pointTotal += weightValue*currentBoundary.polynomialCoefficient(5);

				sumXD += weightValue*(neighborSegment.planeParameter(0)*currentBoundary.polynomialCoefficient(0)
									  + neighborSegment.planeParameter(1)*currentBoundary.polynomialCoefficient(2)
									  + neighborSegment.planeParameter(2)*currentBoundary.polynomialCoefficient(3));
				sumYD += weightValue*(neighborSegment.planeParameter(0)*currentBoundary.polynomialCoefficient(2)
									  + neighborSegment.planeParameter(1)*currentBoundary.polynomialCoefficient(1)
									  + neighborSegment.planeParameter(2)*currentBoundary.polynomialCoefficient(4));
				sumD += weightValue*(neighborSegment.planeParameter(0)*currentBoundary.polynomialCoefficient(3)
									 + neighborSegment.planeParameter(1)*currentBoundary.polynomialCoefficient(4)
									 + neighborSegment.planeParameter(2)*currentBoundary.polynomialCoefficient(5));

				disparityPixelTotal += static_cast<int>(currentBoundary.polynomialCoefficient(5));

			} else {
				// Coplanar
				int neighborSegmentPixelTotal = neighborSegment.pixelTotal();
				double weightValue = smoothRelativeWeight_/(segmentPixelTotal + neighborSegmentPixelTotal)*stepSize_*stepSize_;

				sumXSqr += weightValue*currentSegment.polynomialCoefficientAll(0);
				sumYSqr += weightValue*currentSegment.polynomialCoefficientAll(1);
				sumXY += weightValue*currentSegment.polynomialCoefficientAll(2);
				sumX += weightValue*currentSegment.polynomialCoefficientAll(3);
				sumY += weightValue*currentSegment.polynomialCoefficientAll(4);
				pointTotal += weightValue*currentSegment.polynomialCoefficientAll(5);

				sumXD += weightValue*(neighborSegment.planeParameter(0)*currentSegment.polynomialCoefficientAll(0)
									  + neighborSegment.planeParameter(1)*currentSegment.polynomialCoefficientAll(2)
									  + neighborSegment.planeParameter(2)*currentSegment.polynomialCoefficientAll(3));
				sumYD += weightValue*(neighborSegment.planeParameter(0)*currentSegment.polynomialCoefficientAll(2)
									  + neighborSegment.planeParameter(1)*currentSegment.polynomialCoefficientAll(1)
									  + neighborSegment.planeParameter(2)*currentSegment.polynomialCoefficientAll(4));
				sumD += weightValue*(neighborSegment.planeParameter(0)*currentSegment.polynomialCoefficientAll(3)
									 + neighborSegment.planeParameter(1)*currentSegment.polynomialCoefficientAll(4)
									 + neighborSegment.planeParameter(2)*currentSegment.polynomialCoefficientAll(5));

				disparityPixelTotal += static_cast<int>(currentSegment.polynomialCoefficientAll(5));

				sumXSqr += weightValue*neighborSegment.polynomialCoefficientAll(0);
				sumYSqr += weightValue*neighborSegment.polynomialCoefficientAll(1);
				sumXY += weightValue*neighborSegment.polynomialCoefficientAll(2);
				sumX += weightValue*neighborSegment.polynomialCoefficientAll(3);
				sumY += weightValue*neighborSegment.polynomialCoefficientAll(4);
				pointTotal += weightValue*neighborSegment.polynomialCoefficientAll(5);

				sumXD += weightValue*(neighborSegment.planeParameter(0)*neighborSegment.polynomialCoefficientAll(0)
									  + neighborSegment.planeParameter(1)*neighborSegment.polynomialCoefficientAll(2)
									  + neighborSegment.planeParameter(2)*neighborSegment.polynomialCoefficientAll(3));
				sumYD += weightValue*(neighborSegment.planeParameter(0)*neighborSegment.polynomialCoefficientAll(2)
									  + neighborSegment.planeParameter(1)*neighborSegment.polynomialCoefficientAll(1)
									  + neighborSegment.planeParameter(2)*neighborSegment.polynomialCoefficientAll(4));
				sumD += weightValue*(neighborSegment.planeParameter(0)*neighborSegment.polynomialCoefficientAll(3)
									 + neighborSegment.planeParameter(1)*neighborSegment.polynomialCoefficientAll(4)
									 + neighborSegment.planeParameter(2)*neighborSegment.polynomialCoefficientAll(5));

				disparityPixelTotal += static_cast<int>(neighborSegment.polynomialCoefficientAll(5));

			}
		}

		if (disparityPixelTotal >= 3) {
			std::vector<double> planeParameter(3);
			solvePlaneEquations(sumXSqr, sumXY, sumX, sumXD,
								sumXY, sumYSqr, sumY, sumYD,
								sumX, sumY, pointTotal, sumD,
								planeParameter);
			segments_[segmentIndex].setDisparityPlane(planeParameter[0], planeParameter[1], planeParameter[2]);
		}
	}
}

void SPSStereo::makeOutputImage(const cv::Mat& segmentImage, cv::Mat& segmentDisparityImage) const {
	//segmentImage.create(height_, width_, CV_16UC1);
	segmentDisparityImage.create(height_, width_, CV_16UC1);


	for (int y = 0; y < height_; ++y) {
		for (int x = 0; x < width_; ++x) {
			int pixelSegmentIndex = labelImage_[width_*y + x];
			//segmentImage.at<unsigned short>(x, y) =  pixelSegmentIndex;
			double estimatedDisparity = segments_[pixelSegmentIndex].estimatedDisparity(x, y);
			if (estimatedDisparity <= 0.0 || estimatedDisparity > 255.0) {
				segmentDisparityImage.at<unsigned short>(y,x) = 0;

			} else {
				//printf("--%f--and--%d--",estimatedDisparity*outputDisparityFactor_ + 0.5,static_cast<unsigned short>(estimatedDisparity*outputDisparityFactor_ + 0.5));
				segmentDisparityImage.at<unsigned short>(y,x) = static_cast<unsigned short>(estimatedDisparity*outputDisparityFactor_  + 0.5);

			}
		}
	}

}

void SPSStereo::makeSegmentBoundaryData(std::vector< std::vector<double> >& disparityPlaneParameters, std::vector< std::vector<int> >& boundaryLabels) const {
	int segmentTotal = static_cast<int>(segments_.size());
	disparityPlaneParameters.resize(segmentTotal);
	for (int segmentIndex = 0; segmentIndex < segmentTotal; ++segmentIndex) {
		disparityPlaneParameters[segmentIndex].resize(3);
		disparityPlaneParameters[segmentIndex][0] = segments_[segmentIndex].planeParameter(0);
		disparityPlaneParameters[segmentIndex][1] = segments_[segmentIndex].planeParameter(1);
		disparityPlaneParameters[segmentIndex][2] = segments_[segmentIndex].planeParameter(2);
	}

	int boundaryTotal = static_cast<int>(boundaries_.size());
	boundaryLabels.resize(boundaryTotal);
	for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
		boundaryLabels[boundaryIndex].resize(3);
		boundaryLabels[boundaryIndex][0] = boundaries_[boundaryIndex].segmentIndex(0);
		boundaryLabels[boundaryIndex][1] = boundaries_[boundaryIndex].segmentIndex(1);
		boundaryLabels[boundaryIndex][2] = boundaries_[boundaryIndex].type();
	}
}
