#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "utility.h"
#include "Hungarian.h"
#include <cfloat>

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>  // To save the data

using namespace std;

namespace roadmarking
{
	struct {
        bool operator()(pcl::PointXYZI p1, pcl::PointXYZI p2) const {
			if (p1.x != p2.x)
				return p1.x > p2.x;
			else if (p1.y != p2.y)
				return  p1.y > p2.y;
			else
				return p1.z > p2.z;
		}
    } comparePoint;

	struct {
        bool operator()(pcl::PointXYZI p1, pcl::PointXYZI p2) const {
   		 if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
        	return true;
    	return false;
		}
    } equalPoint;

	class Csegmentation
	{
	public:
		
		Csegmentation(float res){ resolution = res; };

		void GroundFilter_PMF(const pcXYZIPtr &cloud, pcXYZIPtr &gcloud, pcXYZIPtr &ngcloud);   // PMF 
		void GroundFilter_PMF(const pcXYZIPtr &cloud, pcXYZIPtr &gcloud, pcXYZIPtr &ngcloud, int max_window_size, float slope, float initial_distance, float max_distance);   // PMF �������˲� ��������;
		
		pcXYZIPtr planesegRansac(const pcXYZIPtr &cloud, float threshold);   //Ransac 
		pcXYZIPtr groundprojection(const pcXYZIPtr &cloud);                 
		void cloudFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outclouds, int N, int MeanK, double std);  //Otsu Intensity Thresholding + SOR Filter 
		void SORFilter(const pcXYZI &inclouds, pcXYZI &outclouds, int MeanK, double std); //SOR (Statisics Outliers Remover)  
		void NFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outclouds, int K); 

		void BoundingInformation(const vector<pcXYZI> &clouds, vector<vector<pcl::PointXYZI>> & boundingdatas); //bounding 4 points 
		void BoundingFeatureCalculation(const vector<vector<pcl::PointXYZI>> & boundingdatas, vector<BoundingFeature> & boundingfeatures); //bounding 4 points
		void BoundaryExtraction(const vector<pcXYZI> &clouds, vector<pcXYZI> &boundaryclouds, pcXYZRGBPtr pcGT, int down_rate=1, float alpha_value_scale = 0.8, bool VISUALIZE = true);                 
		void BoundaryExtraction(const vector<pcXYZI> &clouds, vector<pcXYZI> &boundaryclouds, int down_rate=1, float alpha_value_scale = 0.8);                 
		void CornerExtraction(const vector<pcXYZI> &boundaryclouds, vector<pcXYZI> &cornerclouds, bool UseRadius, int K, float radius, float dis_threshold, float maxcos);     
		
		void CategoryJudgementBox_highway(const vector<BoundingFeature> & boundingfeatures, RoadMarkings & roadmarkings);
		void CategoryJudgementBox_cityroad(const vector<BoundingFeature> & boundingfeatures, RoadMarkings & roadmarkings);

		void CombineSideLines(const RoadMarkings & roadmarkings, double combine_length, RoadMarkings & combine_sideline_markings); 
		void Find_head2tail(int head_index, const vector<vector<double>> & d_head_tail, vector<bool> & line_used, vector<int> & combineline, double combine_length);   
		void Find_tail2head(int tail_index, const vector<vector<double>> & d_tail_head, vector<bool> & line_used, vector<int> & combineline, double combine_length);   
		
		void MarkingVectorization_highway(const vector<pcXYZI> &clouds, const vector<vector<pcl::PointXYZI>> &boundingdatas,RoadMarkings & roadmarkings,  double line_sample_dl, double ambiguousRatio);  
		void MarkingVectorization_cityroad(const vector<pcXYZI> &clouds, const vector<vector<pcl::PointXYZI>> &boundingdatas, RoadMarkings & roadmarkings, double line_sample_dl, double ambiguousRatio);  
		void GetRoadmarkingsForVect(RoadMarkings & roadmarkings, RoadMarkings & roadmarkings_sideline, RoadMarkings & roadmarkings_vect);

		pcXYZI alphashape(const pcXYZI &cloud, float alpha_value);   //Concave Hull Generation
		
		pcXYZI CornerpointKNN(const pcXYZI &boundarycloud, int K, float disthreshold, float maxcos);                   //KNN corner point extraction 
		pcXYZI CornerpointRadius(const pcXYZI &boundarycloud, float radius, float disthreshold, float maxcos);         //Radius corner point extraction

		void visualizeConcaveHullBoundries(pcXYZRGBPtr pcGT, const vector<pcXYZI> & boundaryclouds);  // Show the extracted boundry of the point cloud
		void VisualizeStart_EndBB(vector<vector<pcl::PointXYZI>> & boundingdatas,const std::vector<int> &dash_idx, pcXYZRGBPtr pcGT);

		//pcXYZI CornerClusteringKMeans(const pcXYZI &cornercloud, int K);
		// https://stackoverflow.com/questions/34481190/removing-duplicates-of-3d-points-in-a-vector-in-c

		void getClassificationResult(pcXYZRGBPtr pcGT, const vector<pcXYZI> &outclouds);
		vector<DashMarking> EstimateEndPoints(pcXYZRGBPtr pcGT, const vector<pcXYZI> & boundaryclouds, bool VISUALIZE);
		vector<DashMarking> EstimateEndPointsGT(pcXYZRGBPtr pcGT, const pcXYZIPtr &cloud, const vector<int> &gtLabels, bool VISUALIZE);
		void mapMatch(const vector<DashMarking> & gtMarks, const vector<DashMarking> & predMarks, bool SHOW_DISTANCE, pcXYZRGBPtr pcGT, bool VISUALIZE);

		
	protected:
	
	
	private:
		float resolution;
		double estimateSinAngleVec3D(const DashMarking& predInst, const DashMarking& GtInst);
		void outputError(const vector<vector<DashMarkProps>>& propsMatrix, const vector<int>& assignment);
		void writeCostFile(const vector<double>& costList, string fileName);
		double estimateOrthogonalCost(const pcl::PointXYZI& predPoint, const DashMarking& gtSegment);
		double estimateHeadingCost(double side, double hypotenuse);
		
		
	};
}
#endif