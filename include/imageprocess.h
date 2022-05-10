#ifndef INCLUDE_IMAGE_H
#define INCLUDE_IMAGE_H

#include "polyfit.h"
#include "utility.h"
#include "data_io.h"
#include <numeric>
#include <algorithm>

//pcl
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h> 

//boost
#include <boost/filesystem.hpp>


//opencv
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

namespace roadmarking
{
	class Imageprocess
	{
	  public:
		struct str{
		bool operator() ( Point a, Point b ){
			if ( a.y != b.y ) 
				return a.y < b.y;
			return a.x <= b.x ;
			}
		}comp;

		  void pcgrid(Bounds &boundingbox, float resolution);                   
		  
		  void savepcgrid(Bounds &boundingbox, float resolution, const pcXYZIPtr &c);   
		  void savepcgrid(Bounds &boundingbox, float resolution,  pcXYZIPtr &c,  pcXYZIPtr &gc, pcXYZIPtr &ngc);   
		  void pc2imgI(const pcXYZIPtr &cloud, int whatcloud,  Mat &img, float times_std);         //whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud]; //times_std: maxI = meanI+ times_std*stdI
		  void pc2imgZ(const pcXYZIPtr &cloud, int whatcloud,  Mat &img);                          //whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud]
		  void pc2imgD(const pcXYZIPtr &cloud, int whatcloud,  Mat &img ,float k);                 //whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud] k:expected max point number in a pixel
		 
		  void img2pc_g(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr & outcloud);  
		  void img2pc_c(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr & outcloud);  

		  void img2pclabel_g(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ);   
		  void img2pclabel_c(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ); 

		  Mat Sobelboundary(Mat img0);                                                  //Sobel
		  
		  float caculateCurrentEntropy(Mat hist, int threshold);                      
		  Mat maxEntropySegMentation(Mat inputImage);                                 
		  Mat ExtractRoadPixelIZD(const Mat & _binI,const Mat & _binZ,const Mat & _binD); //(for MLS)
		  Mat ExtractRoadPixelIZ (const Mat & _binI,const Mat & _binZ);                   //(for ALS)

		  void RemoveSmallRegion(const Mat &_binImg, Mat &_binfilter, int AreaLimit);   
		  void CcaByTwoPass(const Mat & _binfilterImg, Mat & _labelImg);                
		  void CcaBySeedFill(const Mat& _binfilterImg, Mat & _lableImg);                
		  
		  void ImgReverse(const Mat &img, Mat &img_reverse);                            
		  void ImgFilling(const Mat &img, Mat &img_fill);
		  void ImgFilling(const Mat &img, Mat &img_fill, HoughConfig HC);                            

		  void LabelColor(const Mat & _labelImg, Mat & _colorImg);                     
		  Scalar GetRandomColor();                                                      

		  void DetectCornerHarris(const Mat & src, const Mat & colorlabel, Mat & cornershow, Mat & cornerwithimg, int threshold);                                //Harris
		  void DetectCornerShiTomasi(const Mat & src, const Mat & colorlabel, Mat & cornerwithimg, int minDistance, double mincorenerscore);                     //Shi-Tomasi

		  void Truncate( Mat & Img, Mat & TImg);                                   

		  //Save images
		  void saveimg(const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label, const Mat &Corner);
		  void saveimg(std::string base_folder, int file_index, const Mat &ProjI, const Mat &ProjZ, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label);
		  void saveimg(std::string base_folder, int file_index, const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label); 
		  void saveimg(const Mat &ProjI, const Mat &ProjImf, const Mat &GI, const Mat &BI, const Mat &BIF, const Mat &Label);

		  //Get the transformation
		  void applyPrespectiveTransform(const Mat &img, Bounds& bounds);

		  // Extract Pixel indices for polynomial fitting
		  vector<vector<Point>> getNonZeroIdx(vector<Vec2f> houghLines, Mat img_h, int off_y, int off_x);

		  int nx, ny; //pixel number
		  int timin, tjmin; //truncated pixel no.
		  float minX, minY, minZ;  //bounding box minimum value
		  float res;  //resolution
		  vector<vector<vector<int>>> CMatrixIndice;    //cloud
		  vector<vector<vector<int>>> GCMatrixIndice;   //ground cloud
		  vector<vector<vector<int>>> NGCMatrixIndice;  //non-ground cloud
		  int totallabel; 
		  vector<vector <int>>  labelx;  
		  vector<vector <int>>  labely;  

		protected:
	
		private:
			void visualizeIntensityHistogram(const Mat & imgI);
			void intensityLineSegmentDetector(const Mat & imgI);
			vector<Vec2f> intensityHoughLineDetector(const Mat & imgI, HoughConfig HC);
			void visualizeHoughResults(const Mat &img, const string & condition, const vector<Vec2f> & lines, int marking_width);
			Mat generateHoughMask(const Mat &img, const vector<Vec2f> & lines, int marking_width);
			void testPolyFit(const Mat &img, vector<Point> lane_idx);
			vector<Point> returnHoughWindowContour(const Size& imgBounds, const Vec2f & line, const size_t & houghWinOffset);
			vector<Point> calcLineToImgBounds(const Size& imgBounds, const Vec2f & line);
			vector<Point> returnHoughLineBound(const Size& imgBounds, const Vec2f &line, int window_width);
			Mat rotateFrame(Mat img, const Vec2f & trajectory_line, const size_t & num_line);			
			// Re-ordering the contor
			template< class T >
			void reorder(vector<T> &v, vector<size_t> const &order )  {   
				for ( int s = 1, d; s < order.size(); ++ s ) {
					for ( d = order[s]; d < s; d = order[d] ) ;
					if ( d == s ) while ( d = order[d], d != s ) swap( v[s], v[d] );
				}
			}

	};

}

#endif
