#include "CenCC.h"


void CenCC::buildCV( const Mat& lImg, const Mat& rImg, const int maxDis, Mat* costVol )
{
    // for TAD + Grd input image must be CV_64FC3//输入的图像必须是64FC3
	CV_Assert( lImg.type() == CV_64FC3 && rImg.type() == CV_64FC3 );

    int hei = lImg.rows;//图像宽高
	int wid = lImg.cols;
	Mat lGray, rGray;
	Mat tmp;
    lImg.convertTo( tmp, CV_32F );//64FC3图像数据转化成32F
    cvtColor( tmp, lGray, CV_RGB2GRAY );//灰度化
    lGray.convertTo( lGray, CV_8U, 255 );//灰度图像转换成8U数据格式，并把开始除的255乘回来
	rImg.convertTo( tmp, CV_32F );
	cvtColor( tmp, rGray, CV_RGB2GRAY );
	rGray.convertTo( rGray, CV_8U, 255 );
	// prepare binary code 
	int H_WD = CENCUS_WND / 2;
	bitset<CENCUS_BIT>* lCode = new bitset<CENCUS_BIT>[ wid * hei ];
	bitset<CENCUS_BIT>* rCode = new bitset<CENCUS_BIT>[ wid * hei ];
	bitset<CENCUS_BIT>* pLCode = lCode;
	bitset<CENCUS_BIT>* pRCode = rCode;
    for( int y = 0; y < hei; y ++ ) { //求得中心像素的CENSUS码
		uchar* pLData = ( uchar* ) ( lGray.ptr<uchar>( y ) );
		uchar* pRData = ( uchar* ) ( rGray.ptr<uchar>( y ) );
		for( int x = 0; x < wid; x ++ ) {
			int bitCnt = 0;
			for( int wy = - H_WD; wy <= H_WD; wy ++ ) {
				int qy = ( y + wy + hei ) % hei;
				uchar* qLData = ( uchar* ) ( lGray.ptr<uchar>( qy ) );
				uchar* qRData = ( uchar* ) ( rGray.ptr<uchar>( qy ) );
				for( int wx = - H_WD; wx <= H_WD; wx ++ ) {
					if( wy != 0 || wx != 0 ) {
						int qx = ( x + wx + wid ) % wid;
                        ( *pLCode )[ bitCnt ] = ( pLData[ x ] > qLData[ qx ] );//这里是对窗口内的每个像素与中心像素比较，小于则为1,大于则为0
						( *pRCode )[ bitCnt ] = ( pRData[ x ] > qRData[ qx ] );
						bitCnt ++;
					}

				}
			}
			pLCode ++;
			pRCode ++;
		}
	}
	// build cost volume
	bitset<CENCUS_BIT> lB;
	bitset<CENCUS_BIT> rB;
	pLCode = lCode;
	for( int y = 0; y < hei; y ++ ) {
        int index = y * wid;
		for( int x = 0; x < wid; x ++ ) {
			lB = *pLCode;
			for( int d = 0; d < maxDis; d ++ ) {
                double* cost   = ( double* ) costVol[ d ].ptr<double>( y );//costVol是视差为d时的匹配代价Mat集合
				cost[ x ] = CENCUS_BIT;
				if( x - d >= 0 ) {
					rB = rCode[ index + x - d ];
                    cost[ x ] = ( lB ^ rB ).count();//这里是求左右图的census值的相似性，作为视差为d时，当前像素的匹配代价。
				}

			}
			pLCode ++;
		}
	}
	delete [] lCode;
	delete [] rCode;
}
#ifdef COMPUTE_RIGHT
void CenCC::buildRightCV( const Mat& lImg, const Mat& rImg, const int maxDis, Mat* rCostVol )//这就是从右图到左图的匹配代价计算。
{
	// for TAD + Grd input image must be CV_64FC3
	CV_Assert( lImg.type() == CV_64FC3 && rImg.type() == CV_64FC3 );

	int hei = lImg.rows;
	int wid = lImg.cols;
	Mat lGray, rGray;
	Mat tmp;
	lImg.convertTo( tmp, CV_32F );
	cvtColor( tmp, lGray, CV_RGB2GRAY );
	lGray.convertTo( lGray, CV_8U, 255 );
	rImg.convertTo( tmp, CV_32F );
	cvtColor( tmp, rGray, CV_RGB2GRAY );
	rGray.convertTo( rGray, CV_8U, 255 );
	// prepare binary code 
	int H_WD = CENCUS_WND / 2;
	bitset<CENCUS_BIT>* lCode = new bitset<CENCUS_BIT>[ wid * hei ];
	bitset<CENCUS_BIT>* rCode = new bitset<CENCUS_BIT>[ wid * hei ];
	bitset<CENCUS_BIT>* pLCode = lCode;
	bitset<CENCUS_BIT>* pRCode = rCode;
	for( int y = 0; y < hei; y ++ ) {
		uchar* pLData = ( uchar* ) ( lGray.ptr<uchar>( y ) );
		uchar* pRData = ( uchar* ) ( rGray.ptr<uchar>( y ) );
		for( int x = 0; x < wid; x ++ ) {
			int bitCnt = 0;
			for( int wy = - H_WD; wy <= H_WD; wy ++ ) {
				int qy = ( y + wy + hei ) % hei;
				uchar* qLData = ( uchar* ) ( lGray.ptr<uchar>( qy ) );
				uchar* qRData = ( uchar* ) ( rGray.ptr<uchar>( qy ) );
				for( int wx = - H_WD; wx <= H_WD; wx ++ ) {
					if( wy != 0 || wx != 0 ) {
						int qx = ( x + wx + wid ) % wid;
						( *pLCode )[ bitCnt ] = ( pLData[ x ] > qLData[ qx ] );
						( *pRCode )[ bitCnt ] = ( pRData[ x ] > qRData[ qx ] );
						bitCnt ++;
					}

				}
			}
			pLCode ++;
			pRCode ++;
		}
	}
	// build cost volume
	bitset<CENCUS_BIT> lB;
	bitset<CENCUS_BIT> rB;
	pRCode = rCode;
	for( int y = 0; y < hei; y ++ ) {
		int index = y * wid;
		for( int x = 0; x < wid; x ++ ) {
			rB = *pRCode;
			for( int d = 0; d < maxDis; d ++ ) {
				double* cost   = ( double* ) rCostVol[ d ].ptr<double>( y );
				cost[ x ] = CENCUS_BIT;
				if( x + d < wid ) {
					lB = lCode[ index + x + d ];
					cost[ x ] = ( rB ^ lB ).count();
				}

			}
			pRCode ++;
		}
	}
	delete [] lCode;
	delete [] rCode;
}
#endif
