#ifndef INCLUDE_IMAGE_H
#define INCLUDE_IMAGE_H

#include "utility.h"
#include "data_io.h"
#include <numeric>
#include <algorithm>

//pcl
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h> 
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

//boost
#include <boost/filesystem.hpp>

//ROOT
//#include <TLinearFitter.h>
//#include <TMatrixT.h>
#include <Math/RootFinder.h>
#include <Math/Polynomial.h>
// #include <SpecFuncCephes.h>

//opencv
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>

using namespace std;

namespace roadmarking
{
	class Imageprocess
	{
	  public:
		struct str{
		bool operator() ( cv::Point a, cv::Point b ){
			if ( a.y != b.y ) 
				return a.y < b.y;
			return a.x <= b.x ;
			}
		}comp;

		  void pcgrid(Bounds &boundingbox, float resolution);                   
		  
		  void savepcgrid(Bounds &boundingbox, float resolution, const pcXYZIPtr &c);   
		  void savepcgrid(Bounds &boundingbox, float resolution,  pcXYZIPtr &c,  pcXYZIPtr &gc, pcXYZIPtr &ngc, const pcXYZRGBPtr& GT);
		  void savepcgrid(Bounds &boundingbox, float resolution,  pcXYZIPtr &c,  pcXYZIPtr &gc, pcXYZIPtr &ngc);   
		  void pc2imgI(const pcXYZIPtr &cloud, int whatcloud,  cv::Mat &img, float times_std);         //whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud]; //times_std: maxI = meanI+ times_std*stdI
		  void pc2imgZ(const pcXYZIPtr &cloud, int whatcloud,  cv::Mat &img);                          //whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud]
		  void pc2imgD(const pcXYZIPtr &cloud, int whatcloud,  cv::Mat &img ,float k);                 //whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud] k:expected max point number in a pixel
		 
		  void img2pc_g(const cv::Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr & outcloud);  
		  void img2pc_c(const cv::Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr & outcloud);  

		  void img2pclabel_g(const cv::Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ);   
		  void img2pclabel_c(const cv::Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ); 

		  cv::Mat Sobelboundary(cv::Mat img0);                                                  //Sobel
		  
		  float caculateCurrentEntropy(cv::Mat hist, int threshold);                      
		  cv::Mat maxEntropySegMentation(cv::Mat inputImage); 
		  cv::Mat OtsuSegMentation(cv::Mat inputImage);                                
		  cv::Mat ExtractRoadPixelIZD(const cv::Mat & _binI,const cv::Mat & _binZ,const cv::Mat & _binD); //(for MLS)
		  cv::Mat ExtractRoadPixelIZ (const cv::Mat & _binI,const cv::Mat & _binZ);                   //(for ALS)

		  void RemoveSmallRegion(const cv::Mat &_binImg, cv::Mat &_binfilter, int AreaLimit);   
		  void CcaByTwoPass(const cv::Mat & _binfilterImg, cv::Mat & _labelImg);                
		  void CcaBySeedFill(const cv::Mat& _binfilterImg, cv::Mat & _lableImg);                
		  
		  void ImgReverse(const cv::Mat &img, cv::Mat &img_reverse);                            
		  void ImgFilling(const cv::Mat &img, cv::Mat &img_fill);
		  void ImgFilling(const cv::Mat &img, cv::Mat &img_fill, HoughConfig HC, bool IS_SPARSE);                            

		  void LabelColor(const cv::Mat & _labelImg, cv::Mat & _colorImg);                     
		  cv::Scalar GetRandomColor();                                                      

		  void DetectCornerHarris(const cv::Mat & src, const cv::Mat & colorlabel, cv::Mat & cornershow, cv::Mat & cornerwithimg, int threshold);                                //Harris
		  void DetectCornerShiTomasi(const cv::Mat & src, const cv::Mat & colorlabel, cv::Mat & cornerwithimg, int minDistance, double mincorenerscore);                     //Shi-Tomasi

		  void Truncate( cv::Mat & Img, cv::Mat & TImg);                                   

		  //Save images
		  void saveimg(const cv::Mat &ProjI, const cv::Mat &ProjZ, const cv::Mat &ProjD, const cv::Mat &ProjImf, const cv::Mat &GI, const cv::Mat &GZ, const cv::Mat &BZ, const cv::Mat &BD, const cv::Mat &GIR, const cv::Mat &BI, const cv::Mat &BIF, const cv::Mat &Label, const cv::Mat &Corner);
		  void saveimg(std::string base_folder, int file_index, const cv::Mat &ProjI, const cv::Mat &ProjZ, const cv::Mat &ProjImf, const cv::Mat &GI, const cv::Mat &GZ, const cv::Mat &BZ, const cv::Mat &GIR, const cv::Mat &BI, const cv::Mat &BIF, const cv::Mat &Label);
		  void saveimg(std::string base_folder, int file_index, const cv::Mat &ProjI, const cv::Mat &ProjZ, const cv::Mat &ProjD, const cv::Mat &ProjImf, const cv::Mat &GI, const cv::Mat &GZ, const cv::Mat &BZ, const cv::Mat &BD, const cv::Mat &GIR, const cv::Mat &BI, const cv::Mat &BIF, const cv::Mat &Label); 
		  void saveimg(const cv::Mat &ProjI, const cv::Mat &ProjImf, const cv::Mat &GI, const cv::Mat &BI, const cv::Mat &BIF, const cv::Mat &Label);

		  //Get the transformation
		  void applyPrespectiveTransform(const cv::Mat &img, Bounds& bounds);

		  // Extract Pixel indices for polynomial fitting
		  cv::Mat getNonZeroIdx(vector<cv::Vec2f> houghLines, cv::Mat img_h, int off_y, int off_x);

		  // Label PC in Filttered intensity image 2 for marking and 0 eitherwise
		  void EvaluateLaneMarkings(const cv::Mat & imgFilled, pcXYZRGBPtr& pcGT, bool VISUALIZE);
		  void generatePredictionPC(const cv::Mat & imgFilled, const pcXYZIPtr &cloud, pcXYZRGBPtr& pcPred);
		  int nx, ny; //pixel number
		  int timin, tjmin; //truncated pixel no.
		  float minX, minY, minZ;  //bounding box minimum value
		  float res;  //resolution
		  vector<vector<vector<int>>> CMatrixIndice;    //cloud
		  vector<vector<vector<int>>> GCMatrixIndice;   //ground cloud
		  vector<vector<vector<int>>> NGCMatrixIndice;  //non-ground cloud
		  vector<vector<vector<int>>> CMatrixIndiceWithGT; //ground truth labels for the point cloud
		  int totallabel; 
		  vector<vector <int>>  labelx;  
		  vector<vector <int>>  labely;

		  pcXYZRGBPtr pcGT;

		protected:
	
		private:
			void visualizeIntensityHistogram(const cv::Mat & imgI);
			void intensityLineSegmentDetector(const cv::Mat & imgI);
			vector<cv::Vec2f> intensityHoughLineDetector(const cv::Mat & imgI, HoughConfig HC);
			void visualizeHoughResults(const cv::Mat &img, const string & condition, const vector<cv::Vec2f> & lines, int marking_width);
			cv::Mat generateHoughMask(const cv::Mat &img, const vector<cv::Vec2f> & lines, int marking_width);
			void testPolyFit(const cv::Mat &img, vector<cv::Point> lane_idx);
			vector<cv::Point> returnHoughWindowContour(const cv::Size& imgBounds, const cv::Vec2f & line, const size_t & houghWinOffset);
			vector<cv::Point> calcLineToImgBounds(const cv::Size& imgBounds, const cv::Vec2f & line);
			vector<cv::Point> returnHoughLineBound(const cv::Size& imgBounds, const cv::Vec2f &line, int window_width);
			cv::Mat rotateFrame(const cv::Mat &img, vector<cv::Vec2f> & houghLines);
			cv::Mat recoverImg(const cv::Mat &img, const cv::Size &orig_dims, const double &rot_angle_in_degrees);			
			// Re-ordering the contor
			template< class T >
			void reorder(vector<T> &v, vector<size_t> const &order )  {   
				for ( int s = 1, d; s < order.size(); ++ s ) {
					for ( d = order[s]; d < s; d = order[d] ) ;
					if ( d == s ) while ( d = order[d], d != s ) swap( v[s], v[d] );
				}
			}
			template <typename T>
			std::vector<T> linspace(T a, T b, size_t N) {
				T h = (b - a) / static_cast<T>(N-1);
				std::vector<T> xs(N);
				typename std::vector<T>::iterator x;
				T val;
				for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
					*x = val;
				return xs;
			}
			vector<cv::Point> robustFitting(vector<cv::Point> data_points, cv::Size img_bounds);
			// vector<cv::Vec2f> adjustLines(const vector<Vec2f> &houghLines, const double& rot_angle);
			void visualizePredToGT (const pcXYZRGBPtr & IoU);

	};

}

#endif
