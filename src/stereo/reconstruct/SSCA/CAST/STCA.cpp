#include "STCA.h"
#include "StereoDisparity.h"
#include "StereoHelper.h"
#include "SegmentTree.h"

//
// Segment-Tree Cost Aggregation
//
void STCA::aggreCV( const Mat& lImg, const Mat& rImg, const int maxDis, Mat* costVol )
{
    //printf( "\n\t\tSegment Tree cost aggregation" );
    //printf( "\n\t\tCost volume need to be recompute" );

	int hei = lImg.rows;
	int wid = lImg.cols;
	float* pCV = NULL;
	// image format must convert
	Mat lSgImg, rSgImg;
	lImg.convertTo( lSgImg, CV_8U, 255 );
	rImg.convertTo( rSgImg, CV_8U, 255 );
	cvtColor( lSgImg, lSgImg, CV_RGB2BGR );
	cvtColor( rSgImg, rSgImg, CV_RGB2BGR );
	Mat sgLCost;
	CDisparityHelper dispHelper;
	// init segmentation tree cost volume
	sgLCost = Mat::zeros(1, hei * wid * maxDis, CV_32F);
	CV_Assert(lSgImg.type() == CV_8UC3 && rSgImg.type() == CV_8UC3);
#ifdef RE_COMPUTE_COST
	// recompute cost volume
	sgLCost = 
		dispHelper.GetMatchingCost( lSgImg, rSgImg, maxDis );
#else
	// my cost to st
	// !!! mine start from 1
    // just used for cencus cost 只使用census匹配代价
	pCV = ( float* )sgLCost.data;
	for( int y = 0; y < hei; y ++ ) {
		for( int x = 0; x < wid; x ++ ) {
			for( int d = 0; d < maxDis; d ++ ) {
				double* cost   = ( double* ) costVol[ d ].ptr<double>( y );
				*pCV = cost[ x ];
				pCV ++;
			}
		}
	}
#endif
	// build tree
	CSegmentTree stree;
	CColorWeight cWeight( lSgImg );
	stree.BuildSegmentTree( lSgImg, 0.1, 1200, cWeight);
    // filter cost volume前面只是得到每个像素点在每个视差d的匹配代价。经过分割树代价聚合后，完成每个像素在视差d的代价聚合，类似于局部窗算法，不过抛弃了窗，换作分割树
    stree.Filter(sgLCost, maxDis);//从叶节点到根节点，从根节点到叶节点，对每个像素进行代价聚合。

    // st cost to my  再把分割树之后的代价聚合值给回匹配代价Mat
	pCV = ( float* )sgLCost.data;
	for( int y = 0; y < hei; y ++ ) {
		for( int x = 0; x < wid; x ++ ) {
			for( int d = 0; d < maxDis; d ++ ) {
				double* cost   = ( double* ) costVol[ d ].ptr<double>( y );
                cost[ x ]  = *pCV;//与上面反向，给回去
				pCV ++;
			}
		}
	}
}

