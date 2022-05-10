#include "imageprocess.h"

using namespace std;
using namespace cv;

namespace roadmarking
{

	void Imageprocess::pcgrid(Bounds &boundingbox, float resolution)
	{
		//boundingbox order: Xmin,Ymin,Zmin,Xmax,Ymax,Zmax
		float lx, ly, lz;
		int ix, iy;
		lx = boundingbox.max_x - boundingbox.min_x;
		ly = boundingbox.max_y - boundingbox.min_y;
		lz = boundingbox.max_z - boundingbox.min_z;

		minX = -0.5 * lx;
		minY = -0.5 * ly;
		minZ = -0.5 * lz;

		nx = lx / resolution + 1;
		ny = ly / resolution + 1;

		res = resolution;

		cout << "Image Size: " << nx << " * " << ny << endl;
	}

	void Imageprocess::savepcgrid(Bounds &boundingbox, float resolution, pcXYZIPtr &c, pcXYZIPtr &gc, pcXYZIPtr &ngc)
	{
		float lx, ly, lz;
		int ix, iy;
		lx = boundingbox.max_x - boundingbox.min_x;
		ly = boundingbox.max_y - boundingbox.min_y;
		lz = boundingbox.max_z - boundingbox.min_z;

		minX = -0.5 * lx;
		minY = -0.5 * ly;
		minZ = -0.5 * lz;

		// cout << lx << "," << ly << "," << lz << endl;
		// cout << boundingbox.max_x << "," << boundingbox.max_y << "," << boundingbox.max_z << endl;

		nx = lx / resolution + 1;
		ny = ly / resolution + 1;

		cout << "Image Size: " << nx << " x " << ny << endl;

		if(nx * ny > 2e7)
			cout<<"Warning: Too much grid map, the system may run out of memory."<<endl;

		CMatrixIndice.resize(nx, vector<vector<int>>(ny, vector<int>(0)));

		//Saving point indices 
		for (int i = 0; i < c->points.size(); i++)
		{
			ix = (int)((c->points[i].x - minX) / resolution);
			iy = (int)((c->points[i].y - minY) / resolution);
			CMatrixIndice[ix][iy].push_back(i);
		}
        c.reset(new pcXYZI()); //free the memory
        
	    GCMatrixIndice.resize(nx, vector<vector<int>>(ny, vector<int>(0)));
		NGCMatrixIndice.resize(nx, vector<vector<int>>(ny, vector<int>(0)));

		for (int i = 0; i < gc->points.size(); i++)
		{
			ix = (int)((gc->points[i].x - minX) / resolution);
			iy = (int)((gc->points[i].y - minY) / resolution);
			GCMatrixIndice[ix][iy].push_back(i);
		}
		for (int i = 0; i < ngc->points.size(); i++)
		{
			ix = (int)((ngc->points[i].x - minX) / resolution);
			iy = (int)((ngc->points[i].y - minY) / resolution);
			NGCMatrixIndice[ix][iy].push_back(i);
		}
	}

	void Imageprocess::savepcgrid(Bounds &boundingbox, float resolution, const pcXYZIPtr &c)
	{
		float lx, ly, lz;
		int ix, iy;
		lx = boundingbox.max_x - boundingbox.min_x;
		ly = boundingbox.max_y - boundingbox.min_y;
		lz = boundingbox.max_z - boundingbox.min_z;

		minX = -0.5 * lx;
		minY = -0.5 * ly;
		minZ = -0.5 * lz;

		nx = lx / resolution + 1;
		ny = ly / resolution + 1;

		res = resolution;

		cout << "Image Size: " << nx << " * " << ny << endl;

		CMatrixIndice.resize(nx);

		for (size_t i = 0; i < nx; i++)
		{
			CMatrixIndice[i].resize(ny);
		}

		//Saving point indices 
		for (size_t i = 0; i < c->points.size(); i++)
		{
			ix = (c->points[i].x - minX) / res;
			iy = (c->points[i].y - minY) / res;
			CMatrixIndice[ix][iy].push_back(i);
		}
	}

	void Imageprocess::pc2imgI(const pcXYZIPtr &cloud, int whatcloud, Mat &img, float times_std)
	{

		double mini, maxi;	//min and max Intensity
		double meani, stdi; //mean and standard deviation of Intensity
		int Number_non_zero_pixel;
		int max_per_cell = INT_MIN, min_per_cell = INT_MAX;

		meani = 0;
		stdi = 0;
		Number_non_zero_pixel = 0;

		img.create(nx, ny, CV_8UC1); 

		vector<vector<vector<float>>> matrixi;
		vector<vector<float>> ave;

		matrixi.resize(nx);
		ave.resize(nx);

		for (int i = 0; i < nx; i++)
		{
			matrixi[i].resize(ny);
			ave[i].resize(ny);
			for (int j = 0; j < ny; j++)
			{
				matrixi[i][j].push_back(0);
			
			}
		}

		switch (whatcloud)
		{
		case 0: //Original Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = CMatrixIndice[i][j].begin(); k != CMatrixIndice[i][j].end(); ++k)
						matrixi[i][j].push_back(cloud->points[*k].intensity);
				}
			}
			break;
		case 1: //Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					int num_i = 0;
					for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k) 
						{
							matrixi[i][j].push_back(cloud->points[*k].intensity);
							num_i++;
						}			
						if(max_per_cell < num_i)
							max_per_cell = num_i;
						if(min_per_cell > num_i)
							min_per_cell = num_i;
				}
			}
			cout << "=========CELL GRID STATS.=========" << endl;
			cout << "Maximum number of PC per cell : " << max_per_cell << endl;
			cout << "Minimum number of PC per cell : " << min_per_cell << endl;
			cout << "====================================" << endl;
			break;
		case 2: //Non-Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = NGCMatrixIndice[i][j].begin(); k != NGCMatrixIndice[i][j].end(); ++k) 
						matrixi[i][j].push_back(cloud->points[*k].intensity);
				}
			}
			break;
		}

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				ave[i][j] = (0.001 + accumulate(begin(matrixi[i][j]), end(matrixi[i][j]), 0.0)) / matrixi[i][j].size();
			}
		}

		// Visualizing the intensity matrix
		cv::Mat matIntensities(ave.size(), ave.at(0).size(), CV_32FC1);
		//Initialize m
		double minVal; 
		double maxVal; 
		Point minLoc; 
		Point maxLoc;

		for(int i=0; i<matIntensities.rows; ++i)
			for(int j=0; j<matIntensities.cols; ++j)
				matIntensities.at<float>(i, j) = ave[i][j];

		minMaxLoc(matIntensities, &minVal, &maxVal, &minLoc, &maxLoc );
		cout << "Min Val of intensities: " << minVal << endl;
		cout << "Max Val of intensities: " << maxVal << endl;
		resize(matIntensities, matIntensities, Size(), 0.75, 0.75, INTER_LINEAR);
		// imshow("Intensity Image Plot ", matIntensities);

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				// if (ave[i][j] > 1) 
				// {
					meani += ave[i][j];
					Number_non_zero_pixel++;
				// }
			}
		}
		meani /= Number_non_zero_pixel;
		std::cout << "============meani /= Number_non_zero_pixel;===============" << std::endl;
		std::cout << "the number of non zero pixel : " << Number_non_zero_pixel << std::endl;
		std::cout << "the mean value : " << meani << std::endl;

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				// if (ave[i][j] > 1)
				// {
					stdi += (ave[i][j] - meani) * (ave[i][j] - meani);
				// }
			}
		}

		stdi /= Number_non_zero_pixel;
		stdi = sqrt(stdi);
		std::cout << "============stdi = sqrt(stdi);===============" << std::endl;
		std::cout << stdi << std::endl;
		maxi = meani + times_std * stdi; 
		std::cout << " The maximum intensity value of the frame is "
				  << maxi
				  << std::endl;
		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				if (ave[i][j] > maxi)
					ave[i][j] = maxi;
				img.at<uchar>(i, j) = 255 * ave[i][j] / maxi ;
			}
		}
		// imshow("Intensity image after scaling", img);
		// visualizeIntensityHistogram(img);
		// waitKey();
	}

	void Imageprocess::pc2imgZ(const pcXYZIPtr &cloud, int whatcloud, Mat &img)
	{

		float minaz, maxaz;			 
		img.create(nx, ny, CV_8UC1); 

		minaz = FLT_MAX;
		maxaz = -FLT_MAX;

		vector<vector<vector<float>>> matrixz;
		vector<vector<float>> ave;
		matrixz.resize(nx);
		ave.resize(nx);

		for (int i = 0; i < nx; i++)
		{
			matrixz[i].resize(ny);
			ave[i].resize(ny);
			for (int j = 0; j < ny; j++)
			{
				matrixz[i][j].push_back(minZ); 
			}
		}

		switch (whatcloud)
		{
		case 0: //Original Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = CMatrixIndice[i][j].begin(); k != CMatrixIndice[i][j].end(); ++k) 
						matrixz[i][j].push_back(cloud->points[*k].z);
				}
			}
			break;
		case 1: //Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k)
						matrixz[i][j].push_back(cloud->points[*k].z);
				}
			}
			break;
		case 2: //Non-Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = NGCMatrixIndice[i][j].begin(); k != NGCMatrixIndice[i][j].end(); ++k) 
						matrixz[i][j].push_back(cloud->points[*k].z);
				}
			}
			break;
		}

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
				ave[i][j] = (0.1 + accumulate(begin(matrixz[i][j]), end(matrixz[i][j]), 0.0)) / matrixz[i][j].size();
		}

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				maxaz = max(maxaz, ave[i][j]);
				minaz = min(minaz, ave[i][j]);
			}
		} 

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
				img.at<uchar>(i, j) = 255 * (ave[i][j] - minaz) / (maxaz - minaz);
		}
	}

	void Imageprocess::pc2imgD(const pcXYZIPtr &cloud, int whatcloud, Mat &img, float expected_max_point_num_in_a_pixel)
	{

		img.create(nx, ny, CV_8UC1); 
		Eigen::MatrixXi Matrixnum;
		int maxnum, maxelement;
		Matrixnum.resize(nx, ny);
		Matrixnum = Eigen::MatrixXi::Zero(nx, ny);

		switch (whatcloud)
		{
		case 0: //Original Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
					Matrixnum(i, j) = CMatrixIndice[i][j].size();
			}
			break;
		case 1: //Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
					Matrixnum(i, j) = GCMatrixIndice[i][j].size();
			}
			break;
		case 2: //Non-Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
					Matrixnum(i, j) = NGCMatrixIndice[i][j].size();
			}
			break;
		}

		//maxelement = Matrixnum.maxCoeff();
		//if (maxelement < expectedmaxnum) maxnum = maxelement;
		//else maxnum = expectedmaxnum;
		maxnum = expected_max_point_num_in_a_pixel;

		cout << "max point number: "<<maxnum<<endl;
		for (size_t i = 0; i < nx; i++)
		{
			for (size_t j = 0; j < ny; j++)
			{
				int out;
				if (Matrixnum(i, j) < maxnum)
					out = 255 * Matrixnum(i, j) / maxnum;
				else
					out = 255;
				img.at<uchar>(i, j) = out;
			}
		}
	}

	void Imageprocess::img2pc_g(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr &outcloud)
	{

		Mat grayImage, binImage;
		cvtColor(img, grayImage, COLOR_BGR2GRAY);
		threshold(grayImage, binImage, 0, 1, THRESH_BINARY);

		for (size_t i = timin; i < timin + img.rows; i++)
		{
			for (size_t j = tjmin; j < tjmin + img.cols; j++)
			{
				if (binImage.at<uchar>(i - timin, j - tjmin) == 1)
				{
					for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k)
						outcloud->points.push_back(incloud->points[*k]);
				}
			}
		}
	}

	void Imageprocess::img2pc_c(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr &outcloud)
	{

		Mat grayImage, binImage;
		cvtColor(img, grayImage, COLOR_BGR2GRAY);
		threshold(grayImage, binImage, 0, 1, THRESH_BINARY);

		for (size_t i = timin; i < timin + img.rows; i++)
		{
			for (size_t j = tjmin; j < tjmin + img.cols; j++)
			{
				if (binImage.at<uchar>(i - timin, j - tjmin) == 1)
				{
					for (auto k = CMatrixIndice[i][j].begin(); k != CMatrixIndice[i][j].end(); ++k)
						outcloud->points.push_back(incloud->points[*k]);
				}
			}
		}
	}

	void Imageprocess::img2pclabel_g(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ)
	{
		int classNo = 0;

		outclouds.resize(totallabel - 1);

		timin = 0;
		tjmin = 0;

		for (int i = timin; i < timin + img.rows; i++)
		{
			const int *data_src = (int *)img.ptr<int>(i - timin);
			for (int j = tjmin; j < tjmin + img.cols; j++)
			{
				int pixelValue = data_src[j - tjmin];

				if (pixelValue > 1)
				{
					double max_z, min_z, disz;
					max_z = -DBL_MAX;
					min_z = DBL_MAX;

					for (int k = 0; k < GCMatrixIndice[i][j].size(); k++)
					{

						if (max_z < incloud->points[GCMatrixIndice[i][j][k]].z)
							max_z = incloud->points[GCMatrixIndice[i][j][k]].z;
						if (min_z > incloud->points[GCMatrixIndice[i][j][k]].z)
							min_z = incloud->points[GCMatrixIndice[i][j][k]].z;
					}
					disz = max_z - min_z;
					if (disz < dZ)
					{
						for (auto g = GCMatrixIndice[i][j].begin(); g != GCMatrixIndice[i][j].end(); ++g)
							outclouds[pixelValue - 2].points.push_back(incloud->points[*g]);
					}
					else
					{
						if (outclouds[pixelValue - 2].size() == 0)
							outclouds[pixelValue - 2].points.push_back(incloud->points[GCMatrixIndice[i][j][0]]); 
					}
				}
			}
		}

		for (int k = 0; k <= totallabel - 2; k++)
		{
			if (outclouds[k].size() > 0)
				classNo++;
		}
		//cout << "Cloud Number: " << classNo << endl;
	}

	void Imageprocess::img2pclabel_c(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ)
	{
		int classNo = 0;

		outclouds.resize(totallabel - 1);

		for (int i = timin; i < timin + img.rows; i++)
		{
			const int *data_src = (int *)img.ptr<int>(i - timin);
			for (int j = tjmin; j < tjmin + img.cols; j++)
			{
				int pixelValue = data_src[j - tjmin];

				if (pixelValue > 1)
				{
					double max_z, min_z, disz;
					max_z = -DBL_MAX;
					min_z = DBL_MAX;

					for (int k = 0; k < CMatrixIndice[i][j].size(); k++)
					{

						if (max_z < incloud->points[CMatrixIndice[i][j][k]].z)
							max_z = incloud->points[CMatrixIndice[i][j][k]].z;
						if (min_z > incloud->points[CMatrixIndice[i][j][k]].z)
							min_z = incloud->points[CMatrixIndice[i][j][k]].z;
					}
					disz = max_z - min_z;
					if (disz < dZ)
					{
						for (auto g = CMatrixIndice[i][j].begin(); g != CMatrixIndice[i][j].end(); ++g)
							outclouds[pixelValue - 2].points.push_back(incloud->points[*g]);
					}
					else
					{
						if (outclouds[pixelValue - 2].size() == 0)
							outclouds[pixelValue - 2].points.push_back(incloud->points[CMatrixIndice[i][j][0]]); //������ֿյ�����֮���ٰ������˳�
					}
				}
			}
		}

		for (int k = 0; k <= totallabel - 2; k++)
		{
			if (outclouds[k].size() > 0)
				classNo++;
		}
		cout << "Cloud Number: " << classNo << endl;
	}

	Mat Imageprocess::Sobelboundary(Mat img0)
	{
		//Using Sobel Operation
		Mat grad_xg, grad_yg, abs_grad_xg, abs_grad_yg, dstg;

		Sobel(img0, grad_xg, CV_16S, 1, 0, 3, 1, 1, BORDER_DEFAULT);
		convertScaleAbs(grad_xg, abs_grad_xg);

		Sobel(img0, grad_yg, CV_16S, 0, 1, 3, 1, 1, BORDER_DEFAULT);
		convertScaleAbs(grad_yg, abs_grad_yg);

		addWeighted(abs_grad_xg, 0.5, abs_grad_yg, 0.5, 0, dstg);
		return dstg;
	}

	float Imageprocess::caculateCurrentEntropy(Mat hist, int threshold)
	{
		float BackgroundSum = 0, targetSum = 0;
		const float *pDataHist = (float *)hist.ptr<float>(0);
		for (int i = 0; i < 256; i++)
		{
			if (i < threshold)
				BackgroundSum += pDataHist[i];
			else
				targetSum += pDataHist[i];
		}
		float BackgroundEntropy = 0, targetEntropy = 0;
		for (int i = 0; i < 256; i++)
		{
			if (i < threshold)
			{
				if (pDataHist[i] == 0)
					continue;
				float ratio1 = pDataHist[i] / BackgroundSum;
				BackgroundEntropy += -ratio1 * logf(ratio1);
			}
			else
			{
				if (pDataHist[i] == 0)
					continue;
				float ratio2 = pDataHist[i] / targetSum;
				targetEntropy += -ratio2 * logf(ratio2);
			}
		}
		return (targetEntropy + BackgroundEntropy); 
	}

	Mat Imageprocess::maxEntropySegMentation(Mat inputImage)
	{
		// Max Entropy Binarization
		// Using the distribution of histogram to calculate the threshold leading to the max entropy.
		const int channels[1] = {0};
		const int histSize[1] = {256};
		float pranges[2] = {0, 256};
		const float *ranges[1] = {pranges};
		MatND hist; // No difference to the normal Mat constructor and is about to be absolote
		calcHist(&inputImage, 1, channels, Mat(), hist, 1, histSize, ranges);
		float maxentropy = 0;
		int max_index = 0;
		Mat result;
		for (int i = 0; i < 256; i++) 
		{
			float cur_entropy = caculateCurrentEntropy(hist, i);
			if (cur_entropy > maxentropy)
			{
				maxentropy = cur_entropy;
				max_index = i;
			}
		}
		threshold(inputImage, result, max_index, 1, THRESH_BINARY); // > max_index assign as 1   < max_index assign as 0
		return result;
	}

	Mat Imageprocess::ExtractRoadPixelIZD(const Mat &_imgI, const Mat &_binZ, const Mat &_binD)
	{
		Mat result;
		_imgI.convertTo(result, CV_8UC1);
		//int mini, minj, maxi, maxj;
		//vector <int> arrayi, arrayj;
		for (int i = 0; i < _imgI.rows; i++)
		{
			for (int j = 0; j < _imgI.cols; j++)
			{
				if (_binZ.at<uchar>(i, j) == 1 || _binD.at<uchar>(i, j) == 0)
				{
					result.at<uchar>(i, j) = 0;
				}
				else
				{
					result.at<uchar>(i, j) = _imgI.at<uchar>(i, j);
				}
			}
		}
		return result;
	}
	Mat Imageprocess::ExtractRoadPixelIZ(const Mat &_imgI, const Mat &_binZ)
	{
		Mat result;
		_imgI.convertTo(result, CV_8UC1);
		for (int i = 0; i < _imgI.rows; i++)
		{
			for (int j = 0; j < _imgI.cols; j++)
			{
				if (_binZ.at<uchar>(i, j) == 1)
				{
					result.at<uchar>(i, j) = 0;
				}
				else
				{
					result.at<uchar>(i, j) = _imgI.at<uchar>(i, j);
				}
			}
		}
		return result;
	}
	void Imageprocess::CcaByTwoPass(const Mat &_binImg, Mat &_labelImg)
	{
		// connected component analysis (8-component)
		// use two-pass algorithm 
		// 1. first pass: label each foreground pixel with a label
		// 2. second pass: visit each labeled pixel and merge neighbor labels
		//
		// foreground pixel: _binImg(x,y) = 1
		// background pixel: _binImg(x,y) = 0

		if (_binImg.empty() ||
			_binImg.type() != CV_8UC1)
		{
			return;
		}

		// 1. first pass

		_labelImg.release();
		_binImg.convertTo(_labelImg, CV_32SC1); // _labelImg -> _binImg  32 Signed 

		int label = 1;		   // start by 2
		vector<int> labelSet;  
		labelSet.push_back(0); // background: 0
		labelSet.push_back(1); // foreground: 1

		int rows = _binImg.rows - 1;
		int cols = _binImg.cols - 1;
		for (int i = 1; i < rows; i++) 
		{
			int *data_preRow = _labelImg.ptr<int>(i - 1); 
			int *data_curRow = _labelImg.ptr<int>(i);	  
			for (int j = 1; j < cols; j++)				  
			{
				if (data_curRow[j] == 1) 
				{
					vector<int> neighborLabels;			
					neighborLabels.reserve(2);			
					int leftPixel = data_curRow[j - 1]; 
					int upPixel = data_preRow[j];		
					//int leftupPixel = data_preRow[j - 1];                 

					if (leftPixel > 1) //
					{
						neighborLabels.push_back(leftPixel); 
					}
					if (upPixel > 1) 
					{
						neighborLabels.push_back(upPixel); 
					}

					if (neighborLabels.empty()) //
					{
						labelSet.push_back(++label); // assign to a new label
						data_curRow[j] = label;	
						labelSet[label] = label;
					}
					else
					{
						sort(neighborLabels.begin(), neighborLabels.end());
						int smallestLabel = neighborLabels[0];
						data_curRow[j] = smallestLabel; 

						// save equivalence
						for (size_t k = 1; k < neighborLabels.size(); k++) 
						{
							int tempLabel = neighborLabels[k]; //
							int &oldSmallestLabel = labelSet[tempLabel];
							if (oldSmallestLabel > smallestLabel)
							{
								labelSet[oldSmallestLabel] = smallestLabel;
								oldSmallestLabel = smallestLabel;
							}
							else if (oldSmallestLabel < smallestLabel)
							{
								labelSet[smallestLabel] = oldSmallestLabel;
							}
						}
					}
				}
			}
		}

		// update equivalent labels
		// assigned with the smallest label in each equivalent label set
		for (size_t i = 2; i < labelSet.size(); i++) 
		{
			int curLabel = labelSet[i];
			int preLabel = labelSet[curLabel];
			while (preLabel != curLabel)
			{
				curLabel = preLabel;
				preLabel = labelSet[preLabel];
			}
			labelSet[i] = curLabel;
		}

		// 2. second pass
		for (int i = 0; i < rows; i++)
		{
			int *data = _labelImg.ptr<int>(i);
			for (int j = 0; j < cols; j++)
			{
				int &pixelLabel = data[j];
				pixelLabel = labelSet[pixelLabel];
			}
		}
		totallabel = label;
		//cout << "Number label: " << totallabel << endl;
	}

	void Imageprocess::CcaBySeedFill(const Mat &_binImg, Mat &_lableImg)
	{
		// connected component analysis (8-component)
		// use seed filling algorithm
		// 1. begin with a foreground pixel and push its foreground neighbors into a stack;
		// 2. pop the top pixel on the stack and label it with the same label until the stack is empty
		//
		// foreground pixel: _binImg(x,y) = 1
		// background pixel: _binImg(x,y) = 0

		if (_binImg.empty() ||
			_binImg.type() != CV_8UC1)
		{
			cout << "Wrong type" << endl;
			return;
		}

		_lableImg.release();
		_binImg.convertTo(_lableImg, CV_32SC1); 

		int label = 1; // start by 2
		//vector<vector<pair<int, int>>> labeledPixel;
		//labeledPixel.resize(10000);

		int rows = _binImg.rows;
		int cols = _binImg.cols;
		for (int i = 1; i < rows - 1; i++)
		{
			int *data = _lableImg.ptr<int>(i);
			for (int j = 1; j < cols - 1; j++)
			{
				if (data[j] == 1)
				{
					std::stack<std::pair<int, int>> neighborPixels;
					neighborPixels.push(std::pair<int, int>(i, j)); // pixel position: <i,j>
					++label;										// begin with a new label
					while (!neighborPixels.empty())
					{
						// get the top pixel on the stack and label it with the same label
						std::pair<int, int> curPixel = neighborPixels.top();
						int curX = curPixel.first;
						int curY = curPixel.second;
						_lableImg.at<int>(curX, curY) = label;

						//pair<int, int> pixelcor(curX, curY);
						//labeledPixel[label].push_back(pixelcor);

						// pop the top pixel
						neighborPixels.pop();

						// push the 8-neighbors (foreground pixels)
						if (_lableImg.at<int>(curX, curY - 1) == 1)
						{ // left pixel
							neighborPixels.push(std::pair<int, int>(curX, curY - 1));
						}
						if (_lableImg.at<int>(curX, curY + 1) == 1)
						{ // right pixel
							neighborPixels.push(std::pair<int, int>(curX, curY + 1));
						}
						if (_lableImg.at<int>(curX - 1, curY) == 1)
						{ // up pixel
							neighborPixels.push(std::pair<int, int>(curX - 1, curY));
						}
						if (_lableImg.at<int>(curX + 1, curY) == 1)
						{ // down pixel
							neighborPixels.push(std::pair<int, int>(curX + 1, curY));
						}
						if (_lableImg.at<int>(curX - 1, curY - 1) == 1)
						{ // left up pixel
							neighborPixels.push(std::pair<int, int>(curX - 1, curY - 1));
						}
						if (_lableImg.at<int>(curX - 1, curY + 1) == 1)
						{ // left down pixel
							neighborPixels.push(std::pair<int, int>(curX - 1, curY + 1));
						}
						if (_lableImg.at<int>(curX + 1, curY - 1) == 1)
						{ // right up pixel
							neighborPixels.push(std::pair<int, int>(curX + 1, curY - 1));
						}
						if (_lableImg.at<int>(curX + 1, curY + 1) == 1)
						{ // right down pixel
							neighborPixels.push(std::pair<int, int>(curX + 1, curY + 1));
						}
					}
				}
			}
		}
		totallabel = label;
	}

	void Imageprocess::RemoveSmallRegion(const Mat &Src, Mat &Dst, int AreaLimit)
	{
		int RemoveCount = 0;
		Src.convertTo(Dst, CV_8UC1);
		int CheckMode = 1;	 
		int NeihborMode = 1; 
		Mat PointLabel = Mat::zeros(Src.size(), CV_8UC1);
		if (CheckMode == 1) 
		{
			for (int i = 0; i < Src.rows; i++)
			{
				for (int j = 0; j < Src.cols; j++)
				{
					if (Src.at<uchar>(i, j) < 1)
						PointLabel.at<uchar>(i, j) = 3; 
				}
			}
		}
		else 
		{
			for (int i = 0; i < Src.rows; i++)
			{
				for (int j = 0; j < Src.cols; j++)
				{
					if (Src.at<uchar>(i, j) > 10)
						PointLabel.at<uchar>(i, j) = 3; 
				}
			}
		}

		vector<Point2i> NeihborPos;
		NeihborPos.push_back(Point2i(-1, 0));
		NeihborPos.push_back(Point2i(1, 0));
		NeihborPos.push_back(Point2i(0, -1));
		NeihborPos.push_back(Point2i(0, 1));
		if (NeihborMode == 1)
		{
			NeihborPos.push_back(Point2i(-1, -1));
			NeihborPos.push_back(Point2i(-1, 1));
			NeihborPos.push_back(Point2i(1, -1));
			NeihborPos.push_back(Point2i(1, 1));
		}
		else
			int a = 0; 
		int NeihborCount = 4 + 4 * NeihborMode;
		int CurrX = 0, CurrY = 0;
		for (int i = 0; i < Src.rows; i++)
		{
			for (int j = 0; j < Src.cols; j++)
			{
				if (PointLabel.at<uchar>(i, j) == 0) 
				{									
					vector<Point2i> GrowBuffer;		 
					GrowBuffer.push_back(Point2i(j, i));
					PointLabel.at<uchar>(i, j) = 1;
					int CheckResult = 0;

					for (int z = 0; z < GrowBuffer.size(); z++)
					{
						for (int q = 0; q < NeihborCount; q++)
						{
							CurrX = GrowBuffer.at(z).x + NeihborPos.at(q).x;
							CurrY = GrowBuffer.at(z).y + NeihborPos.at(q).y;
							if (CurrX >= 0 && CurrX < Src.cols && CurrY >= 0 && CurrY < Src.rows) 
							{
								if (PointLabel.at<uchar>(CurrY, CurrX) == 0)
								{
									GrowBuffer.push_back(Point2i(CurrX, CurrY)); 
									PointLabel.at<uchar>(CurrY, CurrX) = 1;		
								}
							}
						}
					}
					if (GrowBuffer.size() > AreaLimit)
						CheckResult = 2;
					else
					{
						CheckResult = 1;
						RemoveCount++; 
					}
					for (int z = 0; z < GrowBuffer.size(); z++)
					{
						CurrX = GrowBuffer.at(z).x;
						CurrY = GrowBuffer.at(z).y;
						PointLabel.at<uchar>(CurrY, CurrX) += CheckResult; 
					}
				}
			}
		}
		CheckMode = 255 * (1 - CheckMode);
		for (int i = 0; i < Src.rows; ++i)
		{
			for (int j = 0; j < Src.cols; ++j)
			{
				if (PointLabel.at<uchar>(i, j) == 2)
				{
					Dst.at<uchar>(i, j) = 0;
				}
				else
				{
					Dst.at<uchar>(i, j) = Src.at<uchar>(i, j);
				}
			}
		}
	}

	Scalar Imageprocess::GetRandomColor()
	{
		uchar r = 255 * (rand() / (1.0 + RAND_MAX)); // rand() / (1.0+ RAND_MAX) : a random float number between 0 and 1 (can't be equal to 1)
		uchar g = 255 * (rand() / (1.0 + RAND_MAX)); 
		uchar b = 255 * (rand() / (1.0 + RAND_MAX));
		return Scalar(b, g, r);
	}

	void Imageprocess::LabelColor(const Mat &_labelImg, Mat &_colorLabelImg)
	{
		if (_labelImg.empty() ||
			_labelImg.type() != CV_32SC1)
		{
			return;
		}

		std::map<int, Scalar> colors;

		int rows = _labelImg.rows;
		int cols = _labelImg.cols;

		_colorLabelImg.release();
		_colorLabelImg.create(rows, cols, CV_8UC3);
		_colorLabelImg = Scalar::all(0);

		for (int i = 0; i < rows; i++)
		{
			const int *data_src = (int *)_labelImg.ptr<int>(i);
			uchar *data_dst = _colorLabelImg.ptr<uchar>(i);
			for (int j = 0; j < cols; j++)
			{
				int pixelValue = data_src[j];
				if (pixelValue > 1)
				{
					//if(j%100==0) cout << pixelValue << endl;
					//labelx[pixelValue - 2].push_back(i);
					//labely[pixelValue - 2].push_back(j);

					if (colors.count(pixelValue) <= 0)
					{
						colors[pixelValue] = GetRandomColor();
					}
					Scalar color = colors[pixelValue];
					*data_dst++ = color[0];
					*data_dst++ = color[1];
					*data_dst++ = color[2];
				}
				else
				{
					data_dst++;
					data_dst++;
					data_dst++;
				}
			}
		}
	}
	void Imageprocess::Truncate(Mat &Img, Mat &TruncatedImg)
	{
		int mini, minj, maxi, maxj, di, dj;
		mini = INT_MAX;
		minj = INT_MAX;
		maxi = 0;
		maxj = 0;

		for (int i = 0; i < Img.rows; i++)
		{
			Img.at<uchar>(i, 0) = 0;
			Img.at<uchar>(i, Img.cols - 1) = 0;
		}
		for (int j = 0; j < Img.cols; j++)
		{
			Img.at<uchar>(0, j) = 0;
			Img.at<uchar>(Img.rows - 1, j) = 0;
		}

		for (int i = 0; i < Img.rows; i++)
		{
			for (int j = 0; j < Img.cols; j++)
			{
				if (Img.at<uchar>(i, j) != 0)
				{
					if (i < mini)
						mini = i;
					if (i > maxi)
						maxi = i;
					if (j < minj)
						minj = j;
					if (j > maxj)
						maxj = j;
				}
			}
		}

		timin = mini - 1;
		tjmin = minj - 1;
		di = maxi - mini + 3;
		dj = maxj - minj + 3;
		cv::Rect rect(tjmin, timin, dj, di);
		TruncatedImg = Img(rect);

		/*TruncatedImg.create(di, dj, CV_8UC1);
	for (int i = 0; i < di; i++)
	{
		for (int j = 0; j < dj; j++)
		{
			TruncatedImg.at<uchar>(i, j) = Img.at<uchar>(timin + i, tjmin + j);
		}
	}*/

		//cout << "X: " << tjmin << "  Y: " << timin << "  dX: " << dj << "  dY: " << di<< endl;

		//imshow("Truncated", 255*TruncatedImg);
	}

	void Imageprocess::DetectCornerHarris(const Mat &src, const Mat &colorlabel, Mat &cornershow, Mat &cornerwithimg, int threshold)
	{
		Mat corner, corner8u, imageGray;
		src.convertTo(imageGray, CV_8UC1);
		corner = Mat::zeros(src.size(), CV_32FC1);
		cornerHarris(imageGray, corner, 3, 3, 0.04, BORDER_DEFAULT);
		normalize(corner, corner8u, 0, 255, NORM_MINMAX); 
		convertScaleAbs(corner8u, cornershow);
		cornerwithimg = colorlabel.clone();
		for (int i = 0; i < src.rows; i++)
		{
			for (int j = 0; j < src.cols; j++)
			{
				if (cornershow.at<uchar>(i, j) > threshold)
				{
					circle(cornerwithimg, Point(j, i), 2, Scalar(255, 255, 255), 2); 
				}
			}
		}
	}
	void Imageprocess::ImgReverse(const Mat &img, Mat &img_reverse)
	{
		img.convertTo(img_reverse, CV_8UC1);

		for (int i = 1; i < img.rows - 1; i++)
		{
			for (int j = 1; j < img.cols - 1; j++)
			{
				if (img.at<uchar>(i, j) == 1)
					img_reverse.at<uchar>(i, j) = 0;
				if (img.at<uchar>(i, j) == 0)
					img_reverse.at<uchar>(i, j) = 1;
			}
		}
		//imshow("imgReverse", 255*img_reverse);
	}
	void Imageprocess::ImgFilling(const Mat &img, Mat &img_fill)
	{
		img.convertTo(img_fill, CV_8UC1);

		Mat img_reverse, img_reverse_label;
		//threshold(img, img_reverse, 1, 1, CV_THRESH_BINARY);
		ImgReverse(img, img_reverse);
		CcaBySeedFill(img_reverse, img_reverse_label);

		for (int i = 1; i < img.rows - 1; i++)
		{
			for (int j = 1; j < img.cols - 1; j++)
			{
				if (img_reverse_label.at<int>(i, j) == 2)
					img_reverse.at<uchar>(i, j) = 0;
				img_fill.at<uchar>(i, j) = img.at<uchar>(i, j) + img_reverse.at<uchar>(i, j);
			}
		}

		// imshow("imgfill", 255 * img_fill);
		// waitKey();
		// intensityLineSegmentDetector(255 * img_fill);
	}

	void Imageprocess::ImgFilling(const Mat &img, Mat &img_fill, HoughConfig HC)
	{
		img.convertTo(img_fill, CV_8UC1);

		Mat img_reverse, img_reverse_label;
		//threshold(img, img_reverse, 1, 1, CV_THRESH_BINARY);
		ImgReverse(img, img_reverse);
		CcaBySeedFill(img_reverse, img_reverse_label);

		for (int i = 1; i < img.rows - 1; i++)
		{
			for (int j = 1; j < img.cols - 1; j++)
			{
				if (img_reverse_label.at<int>(i, j) == 2)
					img_reverse.at<uchar>(i, j) = 0;
				img_fill.at<uchar>(i, j) = img.at<uchar>(i, j) + img_reverse.at<uchar>(i, j);
			}
		}

		vector<Vec2f> houghLines = intensityHoughLineDetector(255 * img_fill,HC);

		// Transform the image into a horizontal Orientation
		Vec2f trajectory_line = houghLines[0];
		int off_y = 30, off_x = 5;
		Mat img_h;
		cout << "Vehicle trajectory line : " << trajectory_line << endl;
		img_h = rotateFrame(255 * img_fill, trajectory_line, 0);
		
		// Only line instances should be rotated
		houghLines = intensityHoughLineDetector(img_h,HC);
		vector<vector<Point>> polyList_idx = getNonZeroIdx(houghLines, img_h, off_y, off_x);

		// //fit polynomial to per line
		// for(size_t line_i = 0; polyList_idx.size(); line_i++){
		// 	testPolyFit(img_h, polyList_idx[line_i]);
		// }
	}

	void Imageprocess::DetectCornerShiTomasi(const Mat &src, const Mat &colorlabel, Mat &cornerwithimg, int minDistance, double qualityLevel)
	{
		Mat imageGray;
		src.convertTo(imageGray, CV_8UC1);
		vector<Point2f> corners;
		int maxCorners = INT_MAX;
		int blockSize = 3;
		bool useHarrisDetector = false;
		double k = 0.04;

		cornerwithimg = colorlabel.clone();
		/// Apply corner detection :Determines strong corners on an image.
		goodFeaturesToTrack(imageGray, corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k);

		// Draw corners detected
		for (int i = 0; i < corners.size(); i++)
		{
			circle(cornerwithimg, corners[i], 3, Scalar(255, 255, 255), 1, 8, 0);
		}
	}

	void Imageprocess::saveimg(const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label, const Mat &Corner)
	{

		imwrite("1_Intensity Projection Image.jpg", ProjI);
		imwrite("2_Elevation Projection Image.jpg", ProjZ);
		imwrite("3_Density Projection Image.jpg", ProjD);
		imwrite("4_Intensity Projection Image after Median Filter.jpg", ProjImf);
		imwrite("5_Intensity Gradient Image.jpg", GI);
		imwrite("6_Slope Image.jpg", GZ);
		imwrite("7_Slope Binary Image.jpg", 255 * BZ);
		imwrite("8_Density Binary Image.jpg", 255 * BD);
		imwrite("9_Road Intensity Gradient Image.jpg", GIR);
		imwrite("10_Road Intensity Binary Image.jpg", 255 * BI);
		imwrite("11_CCA Filter Road Intensity Binary Image.jpg", 255 * BIF);
		imwrite("12_RoadMarkings.jpg", Label);
		imwrite("13_Marking Corners.jpg", Corner);

		//cout << "Image Output Done." << endl;
	}

	void Imageprocess::saveimg(std::string outputFolder,
							   int file_index, const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label)
	{
		
		if (!boost::filesystem::exists(outputFolder))
		{
			boost::filesystem::create_directory(outputFolder);
		}

		string img1, img2, img3, img4, img5, img6, img7, img8, img9, img10, img11, img12;

		img1 = outputFolder + "/" + "1_Intensity Projection Image.jpg";
		img2 = outputFolder + "/" + "2_Elevation Projection Image.jpg";
		img3 = outputFolder + "/" + "3_Density Projection Image.jpg";
		img4 = outputFolder + "/" + "4_Intensity Projection Image after Median Filter.jpg";
		img5 = outputFolder + "/" + "5_Intensity Gradient Image.jpg";
		img6 = outputFolder + "/" + "6_Slope Image.jpg";
		img7 = outputFolder + "/" + "7_Slope Binary Image.jpg";
		img8 = outputFolder + "/" + "8_Density Binary Image.jpg";
		img9 = outputFolder + "/" + "9_Road Intensity Gradient Image.jpg";
		img10 = outputFolder + "/" + "10_Road Intensity Binary Image.jpg";
		img11 = outputFolder + "/" + "11_CCA Filter Road Intensity Binary Image.jpg";
		img12 = outputFolder + "/" + "12_RoadMarkings.jpg";

		imwrite(img1, ProjI);
		imwrite(img2, ProjZ);
		imwrite(img3, ProjD);
		imwrite(img4, ProjImf);
		imwrite(img5, GI);
		imwrite(img6, GZ);
		imwrite(img7, 255 * BZ);
		imwrite(img8, 255 * BD);
		imwrite(img9, GIR);
		imwrite(img10, 255 * BI);
		imwrite(img11, 255 * BIF);
		imwrite(img12, Label);

		cout << "Image Output Done." << endl;
	}

	//hyj0728 without density
	void Imageprocess::saveimg(std::string outputFolder_base,
							   int file_index, const Mat &ProjI, const Mat &ProjZ, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label)
	{
		std::string outputFolder;

		ostringstream oss;
		// oss << "\\Images_"<< file_index;
		oss << "/Images_" << setw(4) << setfill('0') << file_index;
		// outputFolder = outputFolder_base + "\\Geo-reference Images";

		if (!boost::filesystem::exists(outputFolder_base))
		{
			boost::filesystem::create_directory(outputFolder_base);
		}

		outputFolder = outputFolder_base + oss.str();

		if (!boost::filesystem::exists(outputFolder))
		{
			boost::filesystem::create_directory(outputFolder);
		}

		string img1, img2, img3, img4, img5, img6, img7, img8, img9, img10;

		img1 = outputFolder + "/" + "1_Intensity Projection Image.jpg";
		img2 = outputFolder + "/" + "2_Elevation Projection Image.jpg";
		// img3 = outputFolder + "/" + "3_Density Projection Image.jpg";
		img3 = outputFolder + "/" + "3_Intensity Projection Image after Median Filter.jpg";
		img4 = outputFolder + "/" + "4_Intensity Gradient Image.jpg";
		img5 = outputFolder + "/" + "5_Slope Image.jpg";
		img6 = outputFolder + "/" + "6_Slope Binary Image.jpg";
		// img8 = outputFolder + "/" + "8_Density Binary Image.jpg";
		img7 = outputFolder + "/" + "7_Road Intensity Gradient Image.jpg";
		img8 = outputFolder + "/" + "8_Road Intensity Binary Image.jpg";
		img9 = outputFolder + "/" + "9_CCA Filter Road Intensity Binary Image.jpg";
		img10 = outputFolder + "/" + "10_RoadMarkings.jpg";

		imwrite(img1, ProjI);
		imwrite(img2, ProjZ);
		// imwrite(img3, ProjD);
		imwrite(img3, ProjImf);
		imwrite(img4, GI);
		imwrite(img5, GZ);
		imwrite(img6, 255 * BZ);
		// imwrite(img8, 255 * BD);
		imwrite(img7, GIR);
		imwrite(img8, 255 * BI);
		imwrite(img9, 255 * BIF);
		imwrite(img10, Label);

		cout << "Image Output Done." << endl;
	}

	void Imageprocess::saveimg(const Mat &ProjI, const Mat &ProjImf, const Mat &GI, const Mat &BI, const Mat &BIF, const Mat &Label)
	{
		imwrite("1_Intensity Projection Image.jpg", ProjI);
		imwrite("2_Intensity Projection Image after Median Filter.jpg", ProjImf);
		imwrite("3_Intensity Gradient Image.jpg", GI);
		imwrite("4_Road Intensity Binary Image.jpg", 255 * BI);
		imwrite("5_CCA Filter Road Intensity Binary Image.jpg", 255 * BIF);
		imwrite("6_RoadMarkings.jpg", Label);

		//cout << "Image Output Done." << endl;
	}

	void Imageprocess::visualizeIntensityHistogram(const Mat &imgI){
		const int channels[1] = {0};
		const int histSize[1] = {255};
		float pranges[2] = {1, 256};
		const float *ranges[1] = {pranges};
		Mat hist;
		int WIDTH = 400, HEIGHT = 400;
		int bin_w = cvRound( (double) WIDTH/histSize[0]);
		Mat histImage( HEIGHT, WIDTH, CV_8UC1, Scalar(0)), imgI_down;

		calcHist( &imgI, 1, 0, Mat(), hist, 1, histSize, ranges);
		normalize(hist, hist, 0, imgI.rows, NORM_MINMAX, -1, Mat() );
		for( int i = 1; i < histSize[0]; i++ )
    	{
        line( histImage, Point( bin_w*(i-1), HEIGHT - cvRound(hist.at<float>(i-1)) ),
              Point( bin_w*(i), HEIGHT - cvRound(hist.at<float>(i)) ),
              Scalar(255), 2, 8, 0  );
   		}
		
		// Resize the image so we would be able to visualize it
		resize(imgI, imgI_down, Size(WIDTH, HEIGHT), INTER_LINEAR);

		imwrite("Intensity_scales_hist.png", histImage );
		waitKey();
	}
	void Imageprocess::intensityLineSegmentDetector(const Mat & imgI){

    bool useRefine = true;
    bool useCanny = false;
    bool overlay = false;

    if (useCanny)
    {
        Canny(imgI, imgI, 50, 200, 3); // Apply Canny edge detector
    }

    // Create and LSD detector with standard or no refinement.
    Ptr<LineSegmentDetector> ls = useRefine ? createLineSegmentDetector(LSD_REFINE_ADV) : createLineSegmentDetector(LSD_REFINE_NONE);

    double start = double(getTickCount());
    vector<Vec4f> lines_std;
	vector<float> width_lines, prec_lines, nfa;
	Mat imgI_down;

	imgI.convertTo(imgI_down, CV_8UC1);

    // Detect the lines
    ls->detect(imgI_down, lines_std, width_lines, prec_lines, nfa);

    double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
    std::cout << "It took " << duration_ms << " ms." << std::endl;

	for (int i = 0; i < lines_std.size(); i++)
	{
		if (width_lines[i] < 5)
		{
			lines_std.erase(lines_std.begin()+i);
			width_lines.erase(width_lines.begin()+i);
			i--;
		}
	}

    // Show found lines
    if (!overlay || useCanny)
    {
        imgI_down = Scalar(0);
    }

    ls->drawSegments(imgI_down, lines_std);

    String window_name = useRefine ? "Result - standard refinement" : "Result - no refinement";
    window_name += useCanny ? " - Canny edge detector used" : "";
	
	imwrite("LSD_Intensity_Image.jpg", imgI_down);

	}

	vector<Vec2f> Imageprocess::intensityHoughLineDetector(const Mat & imgI, HoughConfig HC){

		Mat dst;
		vector<Vec2f> lines, filtered_lines; // will hold the results of the detection
		float trajectory_ang_rad = HC.trajectory_ang_rad;
		float fuse_thres = HC.fuse_thres;
		float fuse_factor = HC.fuse_factor;
		double rho_res = HC.rho_res;
		double theta_res = HC.theta_res;
		double decimal_tol = pow(10.0,HC.decimal_tol);

		int vote_thres = HC.vote_thres;

		// Edge detection
		Canny(imgI, dst, 50, 200, 3);
		cout << "=========AFTER CANNY=========" << endl;
		// Standard Hough Line Transform
		HoughLines(dst, lines, rho_res, theta_res * CV_PI/180, vote_thres, 0, 0 ); // runs the actual detection
		
		visualizeHoughResults(imgI, "after_hough_detect", lines, HC.marking_width);
		cout << "=========AFTER HOUGH=========" << endl;
		// Getting the most dominant theta 
		map<float,vector<Vec2f>> parallel_dict;

		// Filtering according to the trajectory direction
			// for( size_t i = 0; i < lines.size(); i++ )
			// {
			// 	if(abs(lines[i][1] - trajectory_ang_rad) < 0.09)
			// 		{
			// 			filtered_lines.push_back(lines[i]);
			// 		}
			// }
		cout << "=========BEFORE PARALLEL FILTERING =========" << endl;
		// Searching for the maximum of parallal lines
		for( size_t i = 0; i < lines.size(); i++)
		{
			float theta_group = round(lines[i][1] * decimal_tol) / decimal_tol;
			cout << " THETA GROUP: " << theta_group << endl;
			parallel_dict[theta_group].push_back(lines[i]);
		}

		map<float,vector<Vec2f>>::iterator it;
		filtered_lines.clear(); // Store only the most dominent theta group (the angle that has the maximum number of parallal lines)
		auto max_parallal = parallel_dict.begin()->second.size();
		float dominantTheta = parallel_dict.begin()->first;

		for (it = parallel_dict.begin(); it != parallel_dict.end(); it++)
		{
			if (it->second.size() > max_parallal)
			{
				max_parallal = it->second.size();
				dominantTheta = it->first;
				cout << "Group # " << max_parallal << " ,theta # " << dominantTheta << endl;
			}
		}
		filtered_lines = parallel_dict[dominantTheta];

		visualizeHoughResults(imgI, "after_parallel_filtering", filtered_lines, HC.marking_width);
		cout << "=========AFTER PARALLEL FILTERING =========" << endl;
		cout << "=========BEFORE FUSING =========" << endl;
		// Fusing close lines
		if (filtered_lines.size() > 0){
			for( size_t i = 0; i < filtered_lines.size()-1; i++)
			{
				for(size_t j = i+1; j < filtered_lines.size(); j++)
				{
					float fused_off = sqrt((filtered_lines[i][0] - filtered_lines[j][0]) * (filtered_lines[i][0] - filtered_lines[j][0]));
					
					if(fused_off <= fuse_thres)
					{
						float fused_rho = filtered_lines[i][0] * fuse_factor + filtered_lines[j][0] * (1 - fuse_factor);
						float fused_theta = filtered_lines[i][1] * fuse_factor + filtered_lines[j][1] * (1 - fuse_factor);
						Vec2f fused_line = {fused_rho, fused_theta};
						filtered_lines.erase(filtered_lines.begin()+j);
						filtered_lines.erase(filtered_lines.begin()+i);
						filtered_lines.push_back(fused_line);
						i--;
						break;
					}

				}
			}
		}

		visualizeHoughResults(imgI, "after_line_fusing", filtered_lines, HC.marking_width);
		cout << "=========AFTER FUSING =========" << endl;
		return filtered_lines;
	}

	void Imageprocess::visualizeHoughResults(const Mat &img, const string & condition, const vector<Vec2f> & lines, int marking_width){
		
		Mat cdst;

		// Copy edges to the images that will display the results in BGR
		cvtColor(img, cdst, COLOR_GRAY2BGR);

		//Print the filtered lines
		std::cout << "Number  of lines " + condition + " : " << lines.size() << std::endl;
		for( size_t i = 0; i < lines.size(); i++)
		{
			cout << "Line # " << i << " : " << lines[i] << endl;
			vector<Point> lineImgBounds = returnHoughLineBound(img.size(), lines[i], marking_width);
			Point Pt1 = lineImgBounds[0], Pt2 = lineImgBounds[1];
			line( cdst, Pt1, Pt2, Scalar(0,0,255), marking_width, LINE_AA);
		}
		imwrite("Hough_transform_img_" + condition + ".jpg",cdst);
	}

	Mat Imageprocess::generateHoughMask(const Mat &img, const vector<Vec2f> & lines, int marking_width){
		
		Mat cdst = Mat::zeros(img.size(), CV_8UC1), seg_Mat;

		// Get an extreme boundry of the image to guarantee that lines will cut the end
		int max_dim = 2*max(img.rows,img.cols); 
		for( size_t i = 0; i < lines.size(); i++)
		{
			cout << "Line # " << i << " : " << lines[i] << endl;
			vector<Point> lineImgBounds = returnHoughLineBound(img.size(), lines[i], marking_width);
			Point Pt1 = lineImgBounds[0], Pt2 = lineImgBounds[1];
			line( cdst, Pt1, Pt2, Scalar(255), marking_width, LINE_AA);
		}

		cv::bitwise_and(img, cdst, seg_Mat);
		return seg_Mat;
	}

	vector<Point> Imageprocess::calcLineToImgBounds(const Size& imgBounds, const Vec2f & line){
		vector<Point> lineBound;
		int WIDTH = imgBounds.width, HEIGHT = imgBounds.height;
		cout << "WIDTH X HEIGHT : " << WIDTH << "  " << HEIGHT << endl;
		float rho = line[0], theta = line[1];
		cout << "RHO Value: " << rho << "   " << "THETA value: " << theta  << endl;
		float x, y, a, b; // for testing boundry point
		Point boundPoint;

		// Check for y limits
		y = 0;
		b = rho / sin(theta), a = -1 * (cos(theta) / sin(theta));
		x = a * y + b;
		cout << "A: " << a << endl;
		cout << "B: " << b << endl;
		cout << "X: " << x << endl;
		cout << "CV_ROUND X: " << cvRound(x) << endl;
		if (x >= 0 && x <= HEIGHT){
			boundPoint.y = cvRound(x);
			boundPoint.x = cvRound(y);
			lineBound.push_back(boundPoint);
			cout << "[y=0] A found boundry point : " << boundPoint << endl;
		}

		y = WIDTH;
		b = rho / sin(theta), a = -1 * (cos(theta) / sin(theta));
		x = a * y + b;
		cout << "A: " << a << endl;
		cout << "B: " << b << endl;
		cout << "X: " << x << endl;
		cout << "CV_ROUND X: " << cvRound(x) << endl;
		if (x >= 0 && x <= HEIGHT){
			boundPoint.y = cvRound(x);
			boundPoint.x = cvRound(y);
			lineBound.push_back(boundPoint);
			cout << "[y = WIDTH] A found boundry point : " << boundPoint << endl;
		}

		//Check for x limits
		x = 0;
		b = rho / sin(theta), a = -1 * (cos(theta) / sin(theta));
		y = (x - b) * pow(a, -1);
		cout << "A: " << a << endl;
		cout << "B: " << b << endl;
		cout << "Y: " << y << endl;
		cout << "CV_ROUND Y: " << cvRound(y) << endl;
		if (y >= 0 && y <= WIDTH){
			boundPoint.y = cvRound(x);
			boundPoint.x = cvRound(y);
			lineBound.push_back(boundPoint);
			cout << "[x = 0;] A found boundry point : " << boundPoint << endl;
		}

		x = HEIGHT;
		b = rho / sin(theta), a = -1 * (cos(theta) / sin(theta));
		y = (x - b) * pow(a, -1);
		cout << "A: " << a << endl;
		cout << "B: " << b << endl;
		cout << "Y: " << y << endl;
		cout << "CV_ROUND Y: " << cvRound(y) << endl;
		if (y >= 0 && y <= WIDTH){
			boundPoint.y = cvRound(x);
			boundPoint.x = cvRound(y);
			lineBound.push_back(boundPoint);
			cout << "[x = HEIGHT;] A found boundry point : " << boundPoint << endl;
		}

		//TODO: Handle the case for the offset outside the image boundry
		return lineBound;
	}

	//TODO: It fails when the lines are vertical
	//TODO: Indices should be switched !!
	vector<Point> Imageprocess::returnHoughLineBound(const Size& imgBounds, const Vec2f &line, int window_width){
		vector<Point> lineBound;
		int WIDTH = imgBounds.width, HEIGHT = imgBounds.height;
		cout << "WIDTH X HEIGHT : " << WIDTH << "  " << HEIGHT << endl;
		float rho = line[0], theta = line[1];
		// TODO: What should happen if the angle is 0: 1/sin(0) == inf
		if (theta == 0)
			theta = 0.0001;
		cout << "RHO Value: " << rho << "   " << "THETA value: " << theta  << endl;
		float x, y, a, b; // for testing boundry point
		Point boundPoint;

		// Check for y limits
		y = 0;
		b = rho / sin(theta), a = -1 * (cos(theta) / sin(theta));
		x = a * y + b;
		if (x >= 0 && x <= HEIGHT){
			boundPoint.y = cvRound(x);
			boundPoint.x = cvRound(y);
			lineBound.push_back(boundPoint);
			cout << "[y = 0] A found boundry point : " << boundPoint << endl;
		}

		y = WIDTH;
		b = rho / sin(theta), a = -1 * (cos(theta) / sin(theta));
		x = a * y + b;
		if (x >= 0 && x <= HEIGHT){
			boundPoint.y = cvRound(x);
			boundPoint.x = cvRound(y);
			lineBound.push_back(boundPoint);
			cout << "[y = WIDTH] A found boundry point : " << boundPoint << endl;
		}

		//Check for x limits
		x = 0;
		b = rho / sin(theta), a = -1 * (cos(theta) / sin(theta));
		cout << "A: " << a << endl;
		y = (x - b) * pow(a, -1);
		if (y >= 0 && y <= WIDTH){
			boundPoint.y = cvRound(x);
			boundPoint.x = cvRound(y);
			lineBound.push_back(boundPoint);
			cout << "[x = 0;] A found boundry point : " << boundPoint << endl;
		}

		x = HEIGHT;
		b = rho / sin(theta), a = -1 * (cos(theta) / sin(theta));
		cout << "A: " << a << endl;
		y = (x - b) * pow(a, -1);
		if (y >= 0 && y <= WIDTH){
			boundPoint.y = cvRound(x);
			boundPoint.x = cvRound(y);
			lineBound.push_back(boundPoint);
			cout << "[x = HEIGHT;] A found boundry point : " << boundPoint << endl;
		}

		cout << "=======================================" << endl;
		cout << "Line bound Point # :"; 
		for(size_t i = 0; i < lineBound.size(); i++){
			cout << " " << lineBound[i];
		}
		cout << endl << "=======================================" << endl;


		//TODO: Handle the case for the offset outside the image boundry
		return lineBound;
	}

	Mat Imageprocess::rotateFrame(Mat img, const Vec2f & trajectory_line, const size_t &num_line){
		// TODO: Should be a user defined input instead !!
		double theta = trajectory_line[1];
		double rot_angle_in_degrees = (theta - CV_PI / 2.0)  * (180 / CV_PI);
		cout << " Trajectory Angle: " << (theta *(180 / CV_PI)) << endl;
		cout << " Rotation Angle : " << rot_angle_in_degrees << endl;
		// get rotation matrix for rotating the image around its center in pixel coordinates
		cv::Point2f center((img.cols-1)/2.0, (img.rows-1)/2.0);
		cv::Mat rot = cv::getRotationMatrix2D(center, rot_angle_in_degrees, 1.0);
		// determine bounding rectangle, center not relevant
		cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), img.size(), rot_angle_in_degrees).boundingRect2f();
		// adjust transformation matrix
		rot.at<double>(0,2) += bbox.width/2.0 - img.cols/2.0;
		rot.at<double>(1,2) += bbox.height/2.0 - img.rows/2.0;

		Mat dst;
		img.copyTo(dst);
		cv::warpAffine(img, dst, rot, bbox.size());
		imwrite("rotated_img_"+to_string(num_line)+".png", dst);
		return dst;
		//TODO: Revert it back to the original orientation
	}

	vector<vector<Point>> Imageprocess::getNonZeroIdx(vector<Vec2f> houghLines, Mat img_h, int off_y, int off_x){
		
		vector<vector<Point>> idxList; 

		// Initialize the vector of line indices
		idxList.resize(houghLines.size());

		// Parameters of your slideing window
		int windows_n_rows = 2 * off_x + 1; 
		int windows_n_cols = 2 * off_y + 1; 

		// Slidding step if the center of mass is the same
		int StepSlide = off_y;

		Mat src = img_h.clone();
		Mat disp_src;
		cvtColor(img_h, disp_src, COLOR_GRAY2BGR);

		//TODO: Put limit of x moves
		for(size_t line_i = 0; line_i < houghLines.size(); line_i++){
			vector<Point> lineImgBounds = returnHoughLineBound(src.size(), houghLines[line_i], 5);
			Point line_start = lineImgBounds[0], line_end = lineImgBounds[1];
			line(disp_src, line_start, line_end, Scalar(0, 0, 255), 2);
			
			int win_start_x, win_start_y;

			// Window boundry initilaization
			win_start_x = max(0, line_start.y - off_x);
			win_start_y = max(0, line_start.x);

			cout << "===============CURR WIN===============" << endl;
			cout << "X, Y, WIDTH & HEIGHT: " << win_start_x << "," << win_start_y << "," << windows_n_cols << "," << windows_n_rows << endl;
			cout << "============================================="<< endl;

			while (win_start_x < (src.rows - windows_n_rows) && win_start_y < (src.cols - windows_n_cols)){
			
				Point start_circle, end_circle, mass_center;
				start_circle.x = win_start_y;
				start_circle.y = win_start_x;
				end_circle = start_circle;
				end_circle.x += windows_n_cols;
				end_circle.y += windows_n_rows;

				circle(disp_src, start_circle, 2, Scalar(255, 0, 0), 2);
				circle(disp_src, end_circle, 2, Scalar(255, 0, 255), 2);

				cout << "===============CURR WIN===============" << endl;
				cout << "X, Y, WIDTH & HEIGHT: " << win_start_x << "," << win_start_y << "," << windows_n_cols << "," << windows_n_rows << endl;
				cout << "============================================="<< endl;
				Rect marking_win(win_start_y, win_start_x, windows_n_cols, windows_n_rows);
				Mat roi = src(marking_win);
				// cout << "===============DISP CURR ROI===============" << endl;
				// cout << "Window Info. : " << marking_win << endl;
				// // Draw only rectangle
				// rectangle(disp_src, marking_win, Scalar(0,255,0), 1);
				// imshow("current roi", disp_src);
				// waitKey();
				// cout << "============================================="<< endl;
				vector<Point> nonzero_region;
				findNonZero(roi, nonzero_region);
				// cout << "===============BEFORE ACCUMM: NON ZERO POINT===============" << endl;
				// for (size_t idx = 0; idx < nonzero_region.size(); idx++)
				// 	cout << nonzero_region[idx] << endl;
				// cout << "============================================="<< endl;
				// cout << "===============AFTER ACCUMM: NON ZERO POINT===============" << endl;
				// for (size_t idx = 0; idx < nonzero_region.size(); idx++){
				// 	nonzero_region[idx].x += win_start_y;
				// 	nonzero_region[idx].y += win_start_x;
				// 	cout << nonzero_region[idx] << endl;
				// }
				// cout << "============================================="<< endl;
				// idxList[line_i].insert(idxList[line_i].end(), nonzero_region.begin(), nonzero_region.end());
				
				// find moments of the image
				Moments m = moments(roi,true);
				Point p(m.m10/m.m00, m.m01/m.m00);

				// coordinates of centroid
				cout << "===============CURR CENTROID===============" << endl;
				cout<< "Coordinates of the centroid is : " << p << endl;
				cout << "============================================="<< endl;
				
				//Check if the centroid is already the center of this round.
				//TODO: What would happen if the blob is black
				win_start_y += windows_n_cols;
				
				if(p.y != 0 || p.x != 0){
					int off_shift = p.y - off_x; //Shift accordingly upwards if the diff is negative, vice versa if downwards
					win_start_x += off_shift;
					mass_center.x = win_start_y - off_y;
					mass_center.y = win_start_x + off_x;
					circle(disp_src, mass_center, 2, Scalar(100, 255, 0), 2);
					rectangle(disp_src, marking_win, Scalar(0,255,0), 1);
					Mat disp_tmp;
					// resize(disp_src, disp_tmp, Size(), 0.25, 0.25, INTER_LINEAR);
					imshow("Center of Mass", disp_src);
					waitKey();
					// Append only the center of mass
					idxList[line_i].push_back(mass_center);
				}
				src(marking_win) = 0; disp_src(marking_win) = 0; // Set already processed region to zero so we won't accumlate them again.	
			}
			testPolyFit(img_h, idxList[line_i]);
			polylines(disp_src, idxList[line_i], false, Scalar(0,255,0),2);
			Mat disp_tmp;
			// resize(disp_src, disp_tmp, Size(), 0.25, 0.25, INTER_LINEAR);
			imshow("PolyLine result", disp_src);
			waitKey();
		}
		return idxList;
	}

	// Is the error proportional to significance of the y or not ?
	void Imageprocess::testPolyFit(const Mat & img, vector<Point> lane_idx){

		Mat cdst(Size(img.rows,img.cols), CV_8UC3, Scalar(0,0,0));
		cout << "The image boundries : " << img.rows << " , " << img.cols << endl;
		size_t n = lane_idx.size();
		double erry[n] = {0.0};
		for(size_t i = 0; i < lane_idx.size(); i++)
		{
			cout << "Point : " << lane_idx[i] <<  " , " << erry[i] << endl;
			circle(cdst, lane_idx[i], 10, Scalar(255,255,255), 8,0);
		}
		cout << "The size of the samples: " << lane_idx.size() << endl;
		// resize(cdst, cdst, Size(), 0.5, 0.5, INTER_LINEAR);
		// imwrite("poly_line_img.jpg",cdst);
		
		// for(size_t i=0; i < lane_idx.size(); i++){
		// 	cout << "Display point: " << lane_idx[i] << endl;
		// }

		// vector<Point> out = polyfit::PolyFitCV(lane_idx, erry, 2, std::pair<size_t,size_t>(img.rows, img.cols));
		// polylines(cdst, out, false, Scalar(0,255,0), 3);
		// // resize(cdst, cdst, Size(), 0.5, 0.5, INTER_LINEAR);
		imwrite("poly_line_img.jpg",cdst);
	}

	void Imageprocess::applyPrespectiveTransform(const Mat &img, Bounds& bounds){
		Mat img_tmp(img.size(), CV_8UC1), tansform_mat;
		vector<Vec2f> src, dst;
		Vec2f vertex_src, vertex_dst;

		// Get the source boundry points
		
		// Up Left
		vertex_src[0] = bounds.min_x;
		vertex_src[1] = bounds.min_y;
		src.push_back(vertex_src);
		
		//Up Right
		vertex_src[0] = bounds.min_x;
		vertex_src[1] = bounds.max_y;
		src.push_back(vertex_src);

		//Down Right
		vertex_src[0] = bounds.max_x;
		vertex_src[1] = bounds.max_y;
		src.push_back(vertex_src);

		//Down Left
		vertex_src[0] = bounds.max_x;
		vertex_src[1] = bounds.min_y;
		src.push_back(vertex_src);

		// Get the destination boundry points
		
		// Up Left
		vertex_dst[0] = 0;
		vertex_dst[1] = 0;
		dst.push_back(vertex_dst);
		
		//Up Right
		vertex_dst[0] = 0;
		vertex_dst[1] = bounds.max_y;
		dst.push_back(vertex_dst);

		//Down Right
		vertex_dst[0] = bounds.max_x;
		vertex_dst[1] = bounds.max_y;
		dst.push_back(vertex_dst);

		//Down Left
		vertex_dst[0] = bounds.max_x;
		vertex_dst[1] = 0;
		dst.push_back(vertex_dst);

		// Compute the transformation matrix
		tansform_mat = getPerspectiveTransform(src,dst);
		warpPerspective(img, img_tmp, tansform_mat, img.size());

		// Display the result
		imshow("After Prespective Transformation", img_tmp);
		// Display the result
		imshow("Before Prespective Image", img);
		waitKey(0);
	}

	vector<Point> Imageprocess::returnHoughWindowContour(const Size& imgBounds, const Vec2f & line, const size_t & houghWinOffset){

		vector<Point> lineBounds;
		Vec2f boundingLine;

		int WIDTH = imgBounds.width, HEIGHT = imgBounds.height;
		//Get lower bounding line end points
		boundingLine = {line[0] - houghWinOffset, line[1]};
		vector<Point> lowerBounds = calcLineToImgBounds(imgBounds, boundingLine);
		lineBounds.insert(lineBounds.end(), lowerBounds.begin(), lowerBounds.end());

		//Get upper bounding line end points
		boundingLine = {line[0] + houghWinOffset, line[1]};
		vector<Point> upperBounds = calcLineToImgBounds(imgBounds, boundingLine);
		lineBounds.insert(lineBounds.end(), upperBounds.begin(), upperBounds.end());

		cout << "AFTER: =======================================" << endl;
		cout << "Contour bound:"; 
		for(size_t i = 0; i < lineBounds.size(); i++){
			cout << " " << lineBounds[i];
		}
		cout << endl << "=======================================" << endl;

		return lineBounds;
		
	}
}