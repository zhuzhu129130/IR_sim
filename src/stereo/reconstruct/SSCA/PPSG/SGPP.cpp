#include "SGPP.h"
#include <cstdio>
#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "pnmfile.h"
#include "segment-image.h"

namespace SGPP_FUNC {
    void lrCheck( Mat& lDis, Mat& rDis, int* lValid, int* rValid, const int disSc )  //左右一致性检查，就是判断左视差与右视差是否一致，一致则该像素视差有效。
	{
		int hei = lDis.rows;
		int wid = lDis.cols;
		int imgSize = hei * wid;
		memset( lValid, 0, imgSize * sizeof( int ) );
		memset( rValid, 0, imgSize * sizeof( int ) );
		int* pLValid = lValid;
		int* pRValid = rValid;
		for( int y = 0; y < hei; y ++ ) {
			uchar* lDisData = ( uchar* ) lDis.ptr<uchar>( y );
			uchar* rDisData = ( uchar* ) rDis.ptr<uchar>( y );
			for( int x = 0; x < wid; x ++ ) {
				// check left image
                int lDep = lDisData[ x ] / disSc; //因为在WTA的时候视差乘以了disSc
				// assert( ( x - lDep ) >= 0 && ( x - lDep ) < wid );
				int rLoc = ( x - lDep + wid ) % wid;
				int rDep = rDisData[ rLoc ] / disSc;
				// disparity should not be zero
				if( lDep == rDep && lDep > 3 && lDep < 150 ) {
                    *pLValid = 1;  //如果左视差等于右视差， 且视差值在3-150之间，这个视差就是有效的。
				}
				// check right image
				rDep = rDisData[ x ] / disSc;
				// assert( ( x + rDep ) >= 0 && ( x + rDep ) < wid );
				int lLoc = ( x + rDep + wid ) % wid;
				lDep = lDisData[ lLoc ] / disSc;
				// disparity should not be zero
				if( lDep == rDep  && rDep > 3 && rDep < 150 ) {
					*pRValid = 1;
				}
				pLValid ++;
				pRValid ++;
			}
		}
	}
	void fillInv( Mat& lDis, Mat& rDis, int* lValid, int* rValid ) //这里就是对左右一致性检查的视差不可靠点，进行左右查找有效点，选择比较小的那个有效点，填充当前视差点
	{
		int hei = lDis.rows;
		int wid = lDis.cols;
		// fill left dep
		int* pLValid = lValid;
		for( int y = 0; y < hei; y ++ ) {
			int* yLValid = lValid + y * wid;
			uchar* lDisData = ( uchar* ) lDis.ptr<uchar>( y );
			for( int x = 0; x < wid; x ++ ) { 
				if( *pLValid == 0 ) {
					// find left first valid pixel
					int lFirst = x;
					int lFind = 0;
					while( lFirst >= 0 ) {
						if( yLValid[ lFirst ] ) {
							lFind = 1;
							break;
						}
						lFirst --;
					}
					int rFind = 0;
					// find right first valid pixel
					int rFirst = x;
					while( rFirst < wid ) {
						if( yLValid[ rFirst ] ) {
							rFind = 1;
							break;
						}
						rFirst ++;
					}
					// set x's depth to the lowest one
					if( lFind && rFind ) {
						if( lDisData[ lFirst ] <= lDisData[ rFirst ] ) {
							lDisData[ x ] = lDisData[ lFirst ];
						} else {
							lDisData[ x ] = lDisData[ rFirst ];
						}
					} else if( lFind ) {
						lDisData[ x ] = lDisData[ lFirst ];
					} else if ( rFind ) {
						lDisData[ x ] = lDisData[ rFirst ];
					}

				}
				pLValid ++;
			}
		}
		// fill right dep
		int* pRValid = rValid;
		for( int y = 0; y < hei; y ++ ) {
			int* yRValid = rValid + y * wid;
			uchar* rDisData = ( uchar* ) ( rDis.ptr<uchar>( y ) );
			for( int x = 0; x < wid; x ++ ) {
				if( *pRValid == 0 ) {
					// find left first valid pixel
					int lFirst = x;
					int lFind = 0;
					while( lFirst >= 0 ) {
						if( yRValid[ lFirst ] ) {
							lFind = 1;
							break;
						}
						lFirst --;
					}
					// find right first valid pixel
					int rFirst = x;
					int rFind = 0;
					while( rFirst < wid ) {
						if( yRValid[ rFirst ] ) {
							rFind = 1;
							break;
						}
						rFirst ++;
					}
					if( lFind && rFind ) {
						// set x's depth to the lowest one
						if( rDisData[ lFirst ] <= rDisData[ rFirst ] ) {
							rDisData[ x ] = rDisData[ lFirst ];
						} else {
							rDisData[ x ] = rDisData[ rFirst ];
						}
					} else if( lFind ) {
						rDisData[ x ] = rDisData[ lFirst ];
					} else if ( rFind )  {
						rDisData[ x ] = rDisData[ rFirst ];
					}

				}
				pRValid ++;
			}
		}
	}
	void wgtMedian( const Mat& lImg, const Mat& rImg, Mat& lDis, Mat& rDis, int* lValid, int* rValid, const int maxDis, const int disSc )
	{
		int hei = lDis.rows;
		int wid = lDis.cols;
		int wndR = MED_SZ / 2; //选择35的窗口
		double* disHist = new double[ maxDis ];

		// filter left
		int* pLValid = lValid;
		for( int y = 0; y < hei; y ++  ) {
			uchar* lDisData = ( uchar* ) lDis.ptr<uchar>( y );
			float* pL = ( float* ) lImg.ptr<float>( y );
			for( int x = 0; x < wid; x ++ ) {
				if( *pLValid == 0 ) {
					// just filter invalid pixels
					memset( disHist, 0, sizeof( double ) * maxDis );
					double sumWgt = 0.0f;
					// set disparity histogram by bilateral weight
					for( int wy = - wndR; wy <= wndR; wy ++ ) {
						int qy = ( y + wy + hei ) % hei;
						// int* qLValid = lValid + qy * wid;
						float* qL = ( float* ) lImg.ptr<float>( qy );
						uchar* qDisData = ( uchar* ) lDis.ptr<uchar>( qy );
						for( int wx = - wndR; wx <= wndR; wx ++ ) {
							int qx = ( x + wx + wid ) % wid;
							// invalid pixel also used
							// if( qLValid[ qx ] && wx != 0 && wy != 0 ) {
							int qDep = qDisData[ qx ] / disSc;
							if( qDep != 0 ) {

								double disWgt = wx * wx + wy * wy; //这是双边滤波的距离权重
								// disWgt = sqrt( disWgt );
								double clrWgt = ( pL[ 3 * x ] - qL[ 3 * qx ] ) * ( pL[ 3 * x ] - qL[ 3 * qx ] ) +
									( pL[ 3 * x + 1 ] - qL[ 3 * qx + 1 ] ) * ( pL[ 3 * x + 1 ] - qL[ 3 * qx + 1 ] ) +
									( pL[ 3 * x + 2 ] - qL[ 3 * qx + 2 ] ) * ( pL[ 3 * x + 2 ] - qL[ 3 * qx + 2 ] );//这是颜色权重
								// clrWgt = sqrt( clrWgt );
								double biWgt = exp( - disWgt / ( SIG_DIS * SIG_DIS ) - clrWgt / ( SIG_CLR * SIG_CLR ) );//综合权重
								disHist[ qDep ] += biWgt;//窗口内每个像素的视差的直方图加权重
								sumWgt += biWgt;//窗口内所有像素视差的权重之和
							}
							// }
						}
					}
					double halfWgt = sumWgt / 2.0f;
					sumWgt = 0.0f;
					int filterDep = 0;
                    for( int d = 0; d < maxDis; d ++ ) {//如果视差d的直方图累加直到大于所有视差权重直方图之和的一半，就认为此视差d为此像素的视差。
						sumWgt += disHist[ d ];
						if( sumWgt >= halfWgt ) {
							filterDep = d;
							break;
						}
					}
					// set new disparity
                    lDisData[ x ] = filterDep * disSc; //求的左图经加权中值滤波的视差
				}
				pLValid ++;
			}
		}
		// filter right depth
		int* pRValid = rValid;
		for( int y = 0; y < hei; y ++  ) {
			uchar* rDisData = ( uchar* ) rDis.ptr<uchar>( y );
			float* pR = ( float* ) rImg.ptr<float>( y );
			for( int x = 0; x < wid; x ++ ) {
				if( *pRValid == 0 ) {
					// just filter invalid pixels
					memset( disHist, 0, sizeof( double ) * maxDis );
					double sumWgt = 0.0f;
					// set disparity histogram by bilateral weight
					for( int wy = - wndR; wy <= wndR; wy ++ ) {
						int qy = ( y + wy + hei ) % hei;
						// int* qRValid = rValid + qy * wid;
						float* qR = ( float* ) rImg.ptr<float>( qy );
						uchar* qDisData = ( uchar* ) rDis.ptr<uchar>( qy );
						for( int wx = - wndR; wx <= wndR; wx ++ ) {
							int qx = ( x + wx + wid ) % wid;
							// if( qRValid[ qx ] && wx != 0 && wy != 0 ) {
							int qDep = qDisData[ qx ] / disSc;
							if( qDep != 0 ) {

								double disWgt = wx * wx + wy * wy;
								disWgt = sqrt( disWgt );
								double clrWgt =
									( pR[ 3 * x ] - qR[ 3 * qx ] ) * ( pR[ 3 * x ] - qR[ 3 * qx ] ) +
									( pR[ 3 * x + 1 ] - qR[ 3 * qx + 1 ] ) * ( pR[ 3 * x + 1 ] - qR[ 3 * qx + 1 ] ) +
									( pR[ 3 * x + 2 ] - qR[ 3 * qx + 2 ] ) * ( pR[ 3 * x + 2 ] - qR[ 3 * qx + 2 ] );
								clrWgt = sqrt( clrWgt );
								double biWgt = exp( - disWgt / ( SIG_DIS * SIG_DIS ) - clrWgt / ( SIG_CLR * SIG_CLR ) );
								disHist[ qDep ] += biWgt;
								sumWgt += biWgt;
							}
							// }
						}
					}
					double halfWgt = sumWgt / 2.0f;
					sumWgt = 0.0f;
					int filterDep = 0;
					for( int d = 0; d < maxDis; d ++ ) {
						sumWgt += disHist[ d ];
						if( sumWgt >= halfWgt ) {
							filterDep = d;
							break;
						}
					}
					// set new disparity
					rDisData[ x ] = filterDep * disSc;
				}
				pRValid ++;
			}
		}

		delete [] disHist;
	}
    void saveChk( const int hei, const int wid,  int* lValid, int* rValid, Mat& lChk )//保存最终经左右一致性检查后的有效视差图，有效为255,无效为0
	{
		Mat rChk = Mat::zeros( hei, wid, CV_8UC1 );
		int* pLV = lValid;
		int* pRV = rValid;
		for( int y = 0; y < hei; y ++ ) {
			uchar* lChkData = ( uchar* )( lChk.ptr<uchar>( y ) );
			uchar* rChkData = ( uchar* )( rChk.ptr<uchar>( y ) );
			for( int x = 0; x < wid; x ++ ) {
				if( *pLV ) {
					lChkData[ x ] = 0;
				} else{
					lChkData[ x ] = 255;
				}

				if( *pRV ) {
					rChkData[ x ] = 0;
				} else{
					rChkData[ x ] = 255;
				}
				pLV ++;
				pRV ++;
			}
		}
#ifdef _DEBUG
		imwrite( "l_chk.png", lChk );
		imwrite( "r_chk.png", rChk );
#endif
	}
	//
    // Convert Mat to Image  把Mat格式转化成我们定义的Image格式
	//
	void matToImage( const Mat& mat, image<rgb>*& input )
	{
		int hei = mat.rows;
		int wid = mat.cols;
		rgb c;
		for( int y = 0;y < hei; y ++ ) {
			uchar* matData = ( uchar* )mat.ptr<uchar>( y );
			for( int x = 0; x < wid; x ++ ) {
				// mat must be RGB !!!
				c.r = matData[ 3 * x ];
				c.g = matData[ 3 * x + 1 ];
				c.b = matData[ 3 * x + 2 ];
				imRef( input, x, y ) = c;
			}
		}
	}
	void imageToMat( image<rgb>*& input, Mat& mat )
	{
		int hei = mat.rows;
		int wid = mat.cols;
		rgb c;
		for( int y = 0;y < hei; y ++ ) {
			uchar* matData = ( uchar* )mat.ptr<uchar>( y );
			for( int x = 0; x < wid; x ++ ) {
				// RGB -> BGR
				c = imRef( input, x, y );
				matData[ 3 * x ] = c.b;
				matData[ 3 * x + 1 ] = c.g;
				matData[ 3 * x + 2 ] = c.r;
			}
		}

	}
	void histFitOneSeg( MySegment& curS, const Mat& lDis, const Mat& lDGrdX, const Mat& lDGrdY, int* lValid )
	{
		int hei = lDis.rows;
		int wid = lDis.cols;
		vector<double> param[ 3 ];
		double minParam[ 3 ] = { 1e10, 1e10, 1e10 };
		double maxParam[ 3 ] = { -1e10, -1e10, -1e10 };
		for( int i = 0; i < curS.xIdx.size(); i ++ ) {
			// only valid pixel can be used
			int curY = curS.yIdx[ i ];
			int curX = curS.xIdx[ i ];
			if( lValid[ curY * wid + curX ] ) {
				double curParam[ 3 ] = {0};
				curParam[ 0 ] = lDGrdX.at<double>( curY,
					curX );
				curParam[ 1 ]= lDGrdY.at<double>(curY,
					curX );
				double curD = ( double ) lDis.at<uchar>( curY,
					curX );
				curParam[ 2 ] = curD - curParam[ 0 ] * curX 
					- curParam[ 1 ] * curY;
				for( int p = 0; p < 3; p ++ ) {
					if( curParam[ p ] < minParam[ p ] ) {
						minParam[ p ] = curParam[ p ];
					}
					if( curParam[ p ] > maxParam[ p ] ) {
						maxParam[ p ] = curParam[ p ];
					}
					param[ p ].push_back( curParam[ p ] );
				}
			}
		}
		// build parameter histogram
		int hist[ 3 ][ SEG_HIST + 1 ];
		memset( &( hist[ 0 ][ 0 ] ), 0, 3 * SEG_HIST * sizeof( int ) );
		double itv[ 3 ] = {0};
		for( int p = 0; p < 3; p ++ ) {
			itv[ p ] = ( maxParam[ p ] - minParam[ p ] ) / SEG_HIST;
		}
		for( int p = 0; p < 3; p ++ ) {
			for( int i = 0; i < param[ p ].size(); i ++ ) {
				int hIdx = 0;
				if( itv[ p ] != 0 ) {
					hIdx = ( int )( ( param[ p ][ i ] - minParam[ p ] ) /
						itv[ p ] );
				}
				if( hIdx < 0 || hIdx >= SEG_HIST + 1 ) {
					printf( "error " );
				}
				hist[ p ][ hIdx ] ++;
			}
		}
		// find max in histotram
		int histMax[ 3 ] = {0};
		int histLoc[ 3 ] = {0};
		for( int p = 0; p < 3; p ++ ) {
			for( int h = 0; h < SEG_HIST + 1; h ++ ) {
				if( hist[ p ][ h ] > histMax[ p ] ) {
					histMax[ p ] = hist[ p ][ h ];
					histLoc[ p ] = h;
				}
			}
		}
		// save paramter
		curS.a = ( histLoc[ 0 ] + 0.5 ) * itv[ 0 ] + minParam[ 0 ];
		curS.b = ( histLoc[ 1 ] + 0.5 ) * itv[ 1 ] + minParam[ 1 ];
		curS.c = ( histLoc[ 2 ] + 0.5 ) * itv[ 2 ] + minParam[ 2 ];
	}
	void histRefineOneSeg( MySegment& curS, Mat& lDis, int* lValid )
	{
		int hei = lDis.rows;
		int wid = lDis.cols;
		for( int i = 0; i < curS.xIdx.size(); i ++ ) {
			// only invalid pixels need to refine
			int curY = curS.yIdx[ i ];
			int curX = curS.xIdx[ i ];
			if( !lValid[ curY * wid + curX ] ) {
				int curD = ( int )( curS.a * curX + curS.b * curY + curS.c );
				lDis.at<uchar>( curY, curX ) = curD;
			}
		}
	}
	void histPlaneFit( Mat& lDis, Mat& rDis, int* lValid, int* rValid, MySegment*& mySeg )
	{
		// compute disparity gradient
		Mat lDGrdX, lDGrdY;
		Mat tmp;
		lDis.convertTo( tmp, CV_32F );
		// X Gradient
		// sobel size must be 1
		Sobel( tmp, lDGrdX, CV_64F, 1, 0 );
		Sobel( tmp, lDGrdY, CV_64F, 0, 1 );

		int hei = lDis.rows;
		int wid = lDis.cols;
		MySegment* pSeg = mySeg;
		int prcSegCnt = 0;
		for( int y = 0;y < hei; y ++ ) {
			for( int x = 0; x < wid; x ++ ) {
				// process each segment
				// segment must > 20 pixels;
				if( (*pSeg).xIdx.size() > 1000 ) {
					printf( "Seg %d ", prcSegCnt );
					prcSegCnt ++;
					// d( x, y ) = ax + by + c
					MySegment curS = *pSeg;
					int valCnt = 0;
					for( int i = 0; i < curS.xIdx.size(); i ++ ) {
						// only valid pixel can be used
						int curY = curS.yIdx[ i ];
						int curX = curS.xIdx[ i ];
						if( lValid[ curY * wid + curX ] ) {
							valCnt ++;
						}
					}
					if( valCnt >= 200 ) {
						// only handle regions with large valid count
						histFitOneSeg( curS, lDis, lDGrdX, lDGrdY, lValid );
						// refine current segment
						histRefineOneSeg( curS, lDis, lValid );
					}
				}
				pSeg ++;
			}
		}
	}
	void lsFitOneSeg( MySegment& curS, const Mat& curDis, int* curValid, const int valCnt )
	{
		int hei = curDis.rows;
		int wid = curDis.cols;
        Mat A = Mat::zeros( valCnt, 3, CV_64F );//分割块各像素的位置
        Mat Y = Mat::zeros( valCnt, 1, CV_64F ); //分割块各像素的视差值
		Mat X = Mat::zeros( 3, 1, CV_64F );
		Mat S = Mat::zeros( valCnt, 1, CV_64F ); 
		int v = 0;
		for( int i = 0; i < curS.xIdx.size(); i ++ ) {
			// only valid pixel can be used
			int curY = curS.yIdx[ i ];
			int curX = curS.xIdx[ i ];
			if( curValid[ curY * wid + curX ] ) {
				A.at<double>( v, 0 ) = curX;
				A.at<double>( v, 1 ) = curY;
				A.at<double>( v, 2 ) = 1.0;
				Y.at<double>( v, 0 ) = ( double)( curDis.at<uchar>( curY, curX ) );
				v ++;
			}
		}
		// itetrative L1 minimization
		// min( || Y - AX - S ||_2 + lammbda * || S ||_1 )
		const double lambda = 1;
		const int ITER_NUM = 10;
		for( int i = 0 ; i < ITER_NUM; i ++ ) {
            solve( A, Y - S, X, DECOMP_QR ); //使用QR原理进行因式分解，求解使AX - （Y - S）最小的X值
            S =  Y - A * X; //求出最小X后，求解S
			for( int v = 0; v < valCnt; v ++ ) {
                double tmpS = S.at<double>( v, 0 ); //对每一个有效像素，判断其S值，从而得到signS的值。
				double signS = 0;
				if( tmpS > 0 ) {
					signS = 1;
				} else if( tmpS == 0 ) {
					signS = 0;
				} else {
					signS = - 1;
				}
				double tmp = ( fabs( tmpS ) - lambda );
                S.at<double>( v, 0 ) = signS * ( tmp > 0 ? tmp : 0 );//求出最后S的值。
			}
#ifdef _DEBUG
			printf( "\n\t\t iter:%d\n", i );
			PrintMat<double>( X );
#endif
		}
		
        // save paramter  求出的X就是这个分割块的平面参数，各像素视差就是该平面参数与平面坐标的相乘
		curS.a = X.at<double>( 0, 0 );
		curS.b = X.at<double>( 1, 0 );
		curS.c = X.at<double>( 2, 0 );
	}
    void lsRefineOneSeg( MySegment& curS, Mat& curDis, int* curValid )//根据求出的平面参数，求出该分割块的各点的坐标。
	{
		int hei = curDis.rows;
		int wid = curDis.cols;
		for( int i = 0; i < curS.xIdx.size(); i ++ ) {
			// only invalid pixels need to refine
			int curY = curS.yIdx[ i ];
			int curX = curS.xIdx[ i ];
			// if( !curValid[ curY * wid + curX ] ) {
				int curD = ( int )( curS.a * curX + curS.b * curY + curS.c );
				curDis.at<uchar>( curY, curX ) = curD;
			//}
		}
	}
	void lsPlaneFit( Mat& lDis, Mat& rDis, int* lValid, int* rValid, MySegment*& lMySeg, MySegment*& rMySeg )
	{
		int hei = lDis.rows;
		int wid = lDis.cols;
		//
		// left plane fit
		//
		MySegment* pSeg = lMySeg;
		int prcSegCnt = 0;
		for( int y = 0;y < hei; y ++ ) {
			for( int x = 0; x < wid; x ++ ) {
				// process each segment
                // segment must > 20 pixels; 分割块必须大于20个像素
				if( (*pSeg).xIdx.size() > MIN_FIT_SZIE ) {
                    //printf( "Seg %d ", prcSegCnt );
					prcSegCnt ++;
					// d( x, y ) = ax + by + c
					MySegment curS = *pSeg;
					int valCnt = 0;
					for( int i = 0; i < curS.xIdx.size(); i ++ ) {
						// only valid pixel can be used
						int curY = curS.yIdx[ i ];
						int curX = curS.xIdx[ i ];
#ifdef _DEBUG
						if( curY == 352 && curX == 745 ) {
							printf( "haha" );
						}
#endif
						if( lValid[ curY * wid + curX ] ) {
							valCnt ++;
						}
					}
                    int totalSize = (*pSeg).xIdx.size();//如果这个分割块中的像素数大于DIFF_FIT_CUT200,就总数*0.1,反之*0.4
					int valFlag = 0;
					if( totalSize > DIFF_FIT_CUT ) {
						valFlag = totalSize * SMALL_PERC;
					} else {
						valFlag = totalSize * BIG_PERC;
					}
                    if( valCnt >= valFlag ) { //如果有效像素数大于valFlag，就进行处理这个分割块
                        // only handle regions with large valid count  对有效像素多的分割块进行计算平面参数
						lsFitOneSeg( curS, lDis, lValid, valCnt );
                        // refine current segment  //根据计算的平面参数求解该分割块每个像素的视差。
						lsRefineOneSeg( curS, lDis, lValid );
					}
				}
				pSeg ++;
			}
		}
		//
		// right plane fit
		//
		pSeg = rMySeg;
		prcSegCnt = 0;
		for( int y = 0;y < hei; y ++ ) {
			for( int x = 0; x < wid; x ++ ) {
				// process each segment
				// segment must > 20 pixels;
				if( (*pSeg).xIdx.size() > MIN_FIT_SZIE ) {
					printf( "Seg %d ", prcSegCnt );
					prcSegCnt ++;
					// d( x, y ) = ax + by + c
					MySegment curS = *pSeg;
					int valCnt = 0;
					for( int i = 0; i < curS.xIdx.size(); i ++ ) {
						// only valid pixel can be used
						int curY = curS.yIdx[ i ];
						int curX = curS.xIdx[ i ];
#ifdef _DEBUG
						if( curY == 352 && curX == 745 ) {
							printf( "haha" );
						}
#endif
						if( rValid[ curY * wid + curX ] ) {
							valCnt ++;
						}
					}
					int totalSize = (*pSeg).xIdx.size();
					int valFlag = 0;
					if( totalSize > DIFF_FIT_CUT ) {
						valFlag = totalSize * SMALL_PERC;
					} else {
						valFlag = totalSize * BIG_PERC;
					}
					if( valCnt >= valFlag ) {
						// only handle regions with large valid count
						lsFitOneSeg( curS, rDis, rValid, valCnt );
						// refine current segment
						lsRefineOneSeg( curS, rDis, rValid );
					}
				}
				pSeg ++;
			}
		}
	}
}


void SGPP::postProcess( const Mat& lImg, const Mat& rImg, const int maxDis, const int disSc, Mat& lDis, Mat& rDis,
	Mat& lSeg, Mat& lChk )
{
    // color image should be 3x3 median filtered 彩色图像必须经过3x3的中值滤波
    // according to weightedMedianMatlab.m from CVPR11 根据加权中值
	Mat lFloat, rFloat;
	Mat lUchar, rUchar;
    lImg.convertTo( lUchar, CV_8U,  255 ); //转换成8U的，图像数据恢复成原来的值
	rImg.convertTo( rUchar, CV_8U,  255 );
	lImg.convertTo( lFloat, CV_32F );
	rImg.convertTo( rFloat, CV_32F );
    int hei = lDis.rows; //视差图的高宽
	int wid = lDis.cols;
	int imgSize = hei * wid;
    int* lValid = new int[ imgSize ]; //最后生成的有效的图像
	int* rValid = new int[ imgSize ];
	


    // qualise hist of lUchar 对8U的图像先灰度化，直方图均衡化，再恢复RGB三通道。
	Mat lGray;
    cvtColor( lUchar, lGray, CV_RGB2GRAY ); //灰度化
    equalizeHist( lGray, lGray ); //提高灰度图像的对比度
    cvtColor( lGray, lUchar, CV_GRAY2RGB ); //
	Mat rGray;
	cvtColor( rUchar, rGray, CV_RGB2GRAY );
	equalizeHist( rGray, rGray );
	cvtColor( rGray, rUchar, CV_GRAY2RGB );
	//
	// Graph Segmentation
	//
	image<rgb>* input = new image<rgb>( wid, hei );
    // convert mat to image //转换Mat格式到我们定义的Image格式
	SGPP_FUNC::matToImage( lUchar, input );
	// sgement image
    //printf( "\n\t\tSegmentation..." );
	int num_ccs;
	MySegment* lMySeg = new MySegment[ wid * hei ];
	image<rgb> *seg = segment_image(input, SEG_SIGMA, SEG_K, 
        SEG_MIN, &num_ccs, lMySeg );  //得到分割后的图像seg，另外得到IMySeg，指明每个分割块中所含的像素
	// get segment image
	SGPP_FUNC::imageToMat( seg, lSeg );
	MySegment* rMySeg = new MySegment[ wid * hei ];
	// convert mat to image
	SGPP_FUNC::matToImage( rUchar, input );
	seg = segment_image(input, SEG_SIGMA, SEG_K, 
		SEG_MIN, &num_ccs, rMySeg ); 
#ifdef _DEBUG
	savePPM(seg, "seg.ppm" );
#endif
	// for( int iter = 0; iter < 3; iter ++ ) {
		// weight median refinement
        printf( "\n\t\tWeight median refinement..." );//中值滤波求精。
		SGPP_FUNC::lrCheck( lDis, rDis, lValid, rValid, disSc );
		SGPP_FUNC::fillInv( lDis, rDis, lValid, rValid );
		SGPP_FUNC::wgtMedian( lFloat, rFloat, lDis, rDis, lValid, rValid, maxDis, disSc );
		//SGPP_FUNC::lrCheck( lDis, rDis, lValid, rValid, disSc );

#ifdef _DEBUG
		imwrite( "before.png", lDis );
#endif
		// plane-fit to refine disparity
		printf( "\n\t\tPlane-fit to refine..." );
        SGPP_FUNC::lsPlaneFit( lDis, rDis, lValid, rValid, lMySeg, rMySeg );//计算分割块平面参数，然后对视差进行求精。


	// }

	// get check image
    SGPP_FUNC::lrCheck( lDis, rDis, lValid, rValid, disSc );//最后再进行一次左右一致性检查，
    SGPP_FUNC::saveChk( hei, wid, lValid, rValid, lChk );//然后可以输出不一致点的图。

	delete input;
	delete [] lMySeg;
	delete [] rMySeg;
	delete [] lValid;
	delete [] rValid;
}
