#ifndef UTILITY_H
#define UTILITY_H


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>


using namespace std;

typedef  pcl::PointCloud<pcl::PointXYZI>::Ptr      pcXYZIPtr;
typedef  pcl::PointCloud<pcl::PointXYZI>           pcXYZI;

typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr      pcXYZPtr;
typedef  pcl::PointCloud<pcl::PointXYZ>           pcXYZ;

typedef  pcl::PointCloud<pcl::PointXY>::Ptr       pcXYPtr;
typedef  pcl::PointCloud<pcl::PointXY>            pcXY;

typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr      pcXYZRGBPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGB>           pcXYZRGB;

typedef  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr       pcXYZRGBAPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGBA>            pcXYZRGBA;

typedef pcl::PointCloud<pcl::Normal>::Ptr NormalsPtr;
typedef pcl::PointCloud<pcl::Normal> Normals;


typedef  pcl::PointCloud<pcl::PointXYZI>::Ptr      CloudXYZI_Ptr;
typedef  pcl::PointCloud<pcl::PointXYZI>           CloudXYZI;

typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr      CloudXYZ_Ptr;
typedef  pcl::PointCloud<pcl::PointXYZ>           CloudXYZ;

typedef  pcl::PointCloud<pcl::PointXY>::Ptr      CloudXY_Ptr;
typedef  pcl::PointCloud<pcl::PointXY>           CloudXY;

typedef  pcl::PointCloud<pcl::Normal>::Ptr        Normal_Ptr;
typedef  pcl::PointCloud<pcl::Normal>             Normal;

typedef  pcl::PointCloud<pcl::PointNormal>::Ptr  Point_Normal_Ptr;
typedef  pcl::PointCloud<pcl::PointNormal>       Point_Normal;

typedef  pcl::PointCloud<pcl::PointXYZINormal>::Ptr  PointXYZI_Normal_Ptr;
typedef  pcl::PointCloud<pcl::PointXYZINormal>      PointXYZI_Normal;

namespace roadmarking
{

	struct DashMarkProps{
		double cost;
		double startCost;
		double endCost;
		double orthoAvgCost;
		double HeadAvgCost;
		double angle;
	};

	struct CenterPoint
	{
		double x;
		double y;
		double z;
		CenterPoint(double x = 0, double y = 0, double z = 0) :
			x(x), y(y), z(z)
		{
			z = 0.0;
			x = y = 0.0;
		}

	};

	struct DashMarking
	{
		pcl::PointXYZI startPoint;
		pcl::PointXYZI endPoint;
	};

	struct Bounds
	{
		double min_x;
		double min_y;
		double min_z;
		double max_x;
		double max_y;
		double max_z;
		Bounds()
		{
			min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
		}
	};

	struct RoadMarking
	{
		int category;  //Category,from 1 to 8

		std::vector<pcl::PointXYZI> polyline; //For vectorization

		Eigen::Matrix4f localization_tranmat_m2s;

		bool is_right_direction;

		//Other semantic or topological information

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	typedef std::vector<RoadMarking, Eigen::aligned_allocator<RoadMarking>> RoadMarkings;

	struct BoundingFeature
	{
		double corner; 
		std::vector<double> sortingEdges; 
	};
	
	struct Voxel
	{
		std::vector<int> point_id;
		float min_z;
		float max_z;
		float dertaz;
		float min_z_x;
		float min_z_y;
		float NeighborMin_z;
		int PointsNumber;
		float mean_z;
		Voxel()
		{
			min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
			PointsNumber = 1;
			dertaz = 0.0;
		}
	};

	class StructOperator
	{
	public:
		void getCloudCenterPoint(const pcl::PointCloud<pcl::PointXYZI> & cloud, CenterPoint & centerPoint)
		{
			double cx = 0, cy = 0, cz = 0;

			for (int i = 0; i < cloud.size(); i++)
			{
				cx += cloud.points[i].x / cloud.size();
				cy += cloud.points[i].y / cloud.size();
				cz += cloud.points[i].z / cloud.size();
			}
			centerPoint.x = cx;
			centerPoint.y = cy;
			centerPoint.z = cz;
		}
		void getCloudBound(const pcl::PointCloud<pcl::PointXYZ> & cloud, Bounds & bound)
		{
			double min_x = cloud[0].x;
			double min_y = cloud[0].y;
			double min_z = cloud[0].z;
			double max_x = cloud[0].x;
			double max_y = cloud[0].y;
			double max_z = cloud[0].z;

			for (int i = 0; i < cloud.size(); i++)
			{
				if (min_x > cloud.points[i].x)
					min_x = cloud.points[i].x;
				if (min_y > cloud.points[i].y)
					min_y = cloud.points[i].y;
				if (min_z > cloud.points[i].z)
					min_z = cloud.points[i].z;
				if (max_x < cloud.points[i].x)
					max_x = cloud.points[i].x;
				if (max_y < cloud.points[i].y)
					max_y = cloud.points[i].y;
				if (max_z < cloud.points[i].z)
					max_z = cloud.points[i].z;
			}
			bound.min_x = min_x;
			bound.max_x = max_x;
			bound.min_y = min_y;
			bound.max_y = max_y;
			bound.min_z = min_z;
			bound.max_z = max_z;
		}

		void getCloudBound(const pcl::PointCloud<pcl::PointXYZI> & cloud, Bounds & bound)
		{
			double min_x = cloud[0].x;
			double min_y = cloud[0].y;
			double min_z = cloud[0].z;
			double max_x = cloud[0].x;
			double max_y = cloud[0].y;
			double max_z = cloud[0].z;

			for (int i = 0; i < cloud.size(); i++)
			{
				if (min_x > cloud.points[i].x)
					min_x = cloud.points[i].x;
				if (min_y > cloud.points[i].y)
					min_y = cloud.points[i].y;
				if (min_z > cloud.points[i].z)
					min_z = cloud.points[i].z;
				if (max_x < cloud.points[i].x)
					max_x = cloud.points[i].x;
				if (max_y < cloud.points[i].y)
					max_y = cloud.points[i].y;
				if (max_z < cloud.points[i].z)
					max_z = cloud.points[i].z;
			}
			bound.min_x = min_x;
			bound.max_x = max_x;
			bound.min_y = min_y;
			bound.max_y = max_y;
			bound.min_z = min_z;
			bound.max_z = max_z;
		}
		void getBoundAndCenter(const pcl::PointCloud<pcl::PointXYZI> & cloud, Bounds & bound, CenterPoint& centerPoint)
		{
			double min_x = cloud[0].x;
			double min_y = cloud[0].y;
			double min_z = cloud[0].z;
			double max_x = cloud[0].x;
			double max_y = cloud[0].y;
			double max_z = cloud[0].z;

			double cx = 0, cy = 0, cz = 0;

			for (int i = 0; i < cloud.size(); i++)
			{
				if (min_x > cloud.points[i].x)
					min_x = cloud.points[i].x;
				if (min_y > cloud.points[i].y)
					min_y = cloud.points[i].y;
				if (min_z > cloud.points[i].z)
					min_z = cloud.points[i].z;
				if (max_x < cloud.points[i].x)
					max_x = cloud.points[i].x;
				if (max_y < cloud.points[i].y)
					max_y = cloud.points[i].y;
				if (max_z < cloud.points[i].z)
					max_z = cloud.points[i].z;
				cx += cloud.points[i].x / cloud.size();
				cy += cloud.points[i].y / cloud.size();
				cz += cloud.points[i].z / cloud.size();
			}
			bound.min_x = min_x;
			bound.max_x = max_x;
			bound.min_y = min_y;
			bound.max_y = max_y;
			bound.min_z = min_z;
			bound.max_z = max_z;


			centerPoint.x = cx;
			centerPoint.y = cy;
			centerPoint.z = cz;
		}

		void GetSubsetBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr & plane_wall_cloud,
			vector<int> & index, Bounds & bound)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < index.size(); i++)
			{
				temp_cloud->push_back(plane_wall_cloud->points[index[i]]);
			}
			getCloudBound(*temp_cloud, bound);
		}
	protected:

	private:
	};
}

#endif


