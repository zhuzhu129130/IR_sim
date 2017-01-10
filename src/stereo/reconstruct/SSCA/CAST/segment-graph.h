/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

//modified by X.Sun, 2012

#ifndef SEGMENT_GRAPH
#define SEGMENT_GRAPH

#define PENALTY_CROSS_SEG 5

#include <vector>
using namespace std;

#include <algorithm>
#include <cmath>
#include "disjoint-set.h"
#include "SegmentTree.h"

// threshold function
#define THRESHOLD(size, c) (c/size)
#define MIN_SIZE_SEG 50

/*
 * Segment a graph
 *
 * Returns a disjoint-set forest representing the segmentation.
 *
 * num_vertices: number of vertices in graph.
 * num_edges: number of edges in graph
 * edges: array of edges.
 * c: constant for threshold function.
 */

int cnt = 0;
universe *segment_graph(int num_vertices, int num_edges, edge *edges, 
			float c, unsigned char *edges_mask) { 
  // sort edges by weight  按边的权重大小升序排列边
  std::sort(edges, edges + num_edges);

  // make a disjoint-set forest   产生一个并查集森林，每棵树中的成员都可由根结点所代表，意思是将图中的点分割成子树，每个子树的根节点代表子树。然后再把各子树连接起来
  universe *u = new universe(num_vertices);

  // init thresholds
  float *threshold = new float[num_vertices];
  for (int i = 0; i < num_vertices; i++) //对每个像素点，初始化阈值为c/1
    threshold[i] = THRESHOLD(1,c);
  
  // for each edge, in non-decreasing weight order...对每个边
  //这里就是从第0个像素开始，若边权重小于阈值，就划为同一个分割块，若不小于，则不是同一个分割块，这里就得到整幅图像的若干个分割块。
  //每个分割块的任意像素的elts[].p都是最开始不是上一个分割块的那个像素的坐标一维值。size记录当前分割块所包含的像素的数量。
  //当一个边的一个端点首次加入比较时，此时的边有个mask值为255,分割块之间的边的mask都为0，重复比较的两点之间的边的mask也为0。这里是避免树上节点的重复连接。
  for (int i = 0; i < num_edges; i++) {
    edge *pedge = &edges[i];
    
    // components connected by this edge找到这个边连接的两个像素
    int a = u->find(pedge->a);
    int b = u->find(pedge->b);
    if (a != b) //如果这两个像素不在同一个分割块内
	{
        if (pedge->w <= threshold[a] && pedge->w <= threshold[b])//如果这两个像素连的边的权值小于等于这两个像素的阈值
        {//就把这两个像素合并，a作为b的根节点，b作为子节点，代表a，以后u->find(a)得到的就是b的属性p这样就可以每个节点只有两个子节点
            edges_mask[i]=255;
			u->join(a, b);
			a = u->find(a);	
			
            threshold[a]  = pedge->w + THRESHOLD(u->size(a), c);//THRESHOLD(u->size(a)得到此时这个分块的大小，其实是b的大小。。。
		}
    }
  }

  //cv::Mat sg_img = cv::Mat(256,320,CV_8UC3);
  char sg_img_1[256*320*3];
  cv::Mat sg_img = cv::Mat(256,320,CV_8UC3,sg_img_1);

  if((cnt++)%3 == 0)
  {
      for(int i=0;i<num_edges;i++)
      {
          int a = u->find(edges[i].a);
          int b = u->find(edges[i].b);

          sg_img_1[edges[i].a * 3] = a%(256*256);
          sg_img_1[edges[i].a * 3+1 ] =256 -  a%256;
          sg_img_1[edges[i].a * 3+2 ] = a;

          sg_img_1[edges[i].b * 3] = b%(256*256);
          sg_img_1[edges[i].b * 3+1 ] =256 - b%256;
          sg_img_1[edges[i].b * 3+2 ] = b;

      }

      cv::imshow("sg_img",sg_img);
      cv::waitKey(1);
  }



  //added by X. Sun: re-organizing the structures to be a single tree
  for (int i = 0; i < num_edges; i++)
  {
		int a = u->find(edges[i].a);
		int b = u->find(edges[i].b);
		if (a != b)
		{
            int size_min = MIN(u->size(a), u->size(b));//求分块a和b的最少大小
			u->join(a, b);

			//record
			edges_mask[i]=255;
			if(size_min > MIN_SIZE_SEG) 
				edges[i].w += PENALTY_CROSS_SEG;
		}
  }



  // free up
  delete []threshold;
  return u;
}

#endif
