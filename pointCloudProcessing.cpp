
#include "pointCloudProcessing.h"



int pointCloudProcessing::generateRandomRotationMatrix(cv::Mat &R)
{
	const float pi = 3.14159265;
	srand(time(NULL));
	double u1 = ((double)rand() / (RAND_MAX));
	double u2 = ((double)rand() / (RAND_MAX));
	double u3 = ((double)rand() / (RAND_MAX));

	double thetaInRad = acos(2 * u1 - 1);
	double phiInRad = 2 * pi * u2;


	double e1 = sin(thetaInRad) * cos(phiInRad);
	double e2 = sin(thetaInRad) * sin(phiInRad);
	double e3 = cos(thetaInRad);

	double signInRad = 2 * pi * u3;
	double cosSignInRad = cos(signInRad);
	double sinSignInRad = sin(signInRad);

	cv::Mat rotationMatrix = cv::Mat(cv::Size(3, 3), CV_32FC1);

	rotationMatrix.at<float>(0, 0) = (1 - cosSignInRad) * e1 * e1 + cosSignInRad;
	rotationMatrix.at<float>(0, 1) = (1 - cosSignInRad) * e1 * e2 - e3 * sinSignInRad;
	rotationMatrix.at<float>(0, 2) = (1 - cosSignInRad) * e1 * e3 + e2 * sinSignInRad;
	rotationMatrix.at<float>(1, 0) = (1 - cosSignInRad) * e2 * e1 + e3 * sinSignInRad;
	rotationMatrix.at<float>(1, 1) = (1 - cosSignInRad) * e2 * e2 + cosSignInRad;
	rotationMatrix.at<float>(1, 2) = (1 - cosSignInRad) * e2 * e3 - e1 * sinSignInRad;
	rotationMatrix.at<float>(2, 0) = (1 - cosSignInRad) * e3 * e1 - e2 * sinSignInRad;
	rotationMatrix.at<float>(2, 1) = (1 - cosSignInRad) * e3 * e2 + e1 * sinSignInRad;
	rotationMatrix.at<float>(2, 2) = (1 - cosSignInRad) * e3 * e3 + cosSignInRad;

	R = rotationMatrix;
	return 0;
}

int pointCloudProcessing::generate4X4MatrixFromRotation(const cv::Mat &R, cv::Mat &transform4x4)
{
	transform4x4 = cv::Mat(cv::Size(4, 4), CV_32FC1);

	transform4x4.at<float>(0, 0) = R.at<float>(0, 0);
	transform4x4.at<float>(0, 1) = R.at<float>(0, 1);
	transform4x4.at<float>(0, 2) = R.at<float>(0, 2);
	transform4x4.at<float>(1, 0) = R.at<float>(1, 0);
	transform4x4.at<float>(1, 1) = R.at<float>(1, 1);
	transform4x4.at<float>(1, 2) = R.at<float>(1, 2);
	transform4x4.at<float>(2, 0) = R.at<float>(2, 0);
	transform4x4.at<float>(2, 1) = R.at<float>(2, 1);
	transform4x4.at<float>(2, 2) = R.at<float>(2, 2);

	transform4x4.at<float>(3, 0) = 0;
	transform4x4.at<float>(3, 1) = 0;
	transform4x4.at<float>(3, 2) = 0;

	transform4x4.at<float>(0, 3) = 0;
	transform4x4.at<float>(1, 3) = 0;
	transform4x4.at<float>(2, 3) = 0;

	transform4x4.at<float>(3, 3) = 1;
	return 0;
}

int pointCloudProcessing::generate4X4MatrixFromTranslation(const cv::Mat &T, cv::Mat &transform4x4)
{
	transform4x4 = cv::Mat(cv::Size(4, 4), CV_32FC1);

	transform4x4.at<float>(0, 0) = 1;
	transform4x4.at<float>(0, 1) = 0;
	transform4x4.at<float>(0, 2) = 0;
	transform4x4.at<float>(1, 0) = 0;
	transform4x4.at<float>(1, 1) = 1;
	transform4x4.at<float>(1, 2) = 0;
	transform4x4.at<float>(2, 0) = 0;
	transform4x4.at<float>(2, 1) = 0;
	transform4x4.at<float>(2, 2) = 1;

	transform4x4.at<float>(3, 0) = 0;
	transform4x4.at<float>(3, 1) = 0;
	transform4x4.at<float>(3, 2) = 0;

	transform4x4.at<float>(0, 3) = T.at<float>(0, 0);
	transform4x4.at<float>(1, 3) = T.at<float>(1, 0);
	transform4x4.at<float>(2, 3) = T.at<float>(2, 0);

	transform4x4.at<float>(3, 3) = 1;
	return 0;
}

int pointCloudProcessing::generate4X4MatrixFromRotationNTranslation(const cv::Mat &R, const cv::Mat &T, cv::Mat &transform4x4)
{
	transform4x4 = cv::Mat(cv::Size(4, 4), CV_32FC1);

	transform4x4.at<float>(0, 0) = R.at<float>(0, 0);
	transform4x4.at<float>(0, 1) = R.at<float>(0, 1);
	transform4x4.at<float>(0, 2) = R.at<float>(0, 2);
	transform4x4.at<float>(1, 0) = R.at<float>(1, 0);
	transform4x4.at<float>(1, 1) = R.at<float>(1, 1);
	transform4x4.at<float>(1, 2) = R.at<float>(1, 2);
	transform4x4.at<float>(2, 0) = R.at<float>(2, 0);
	transform4x4.at<float>(2, 1) = R.at<float>(2, 1);
	transform4x4.at<float>(2, 2) = R.at<float>(2, 2);

	transform4x4.at<float>(3, 0) = 0;
	transform4x4.at<float>(3, 1) = 0;
	transform4x4.at<float>(3, 2) = 0;

	transform4x4.at<float>(0, 3) = T.at<float>(0, 0);
	transform4x4.at<float>(1, 3) = T.at<float>(1, 0);
	transform4x4.at<float>(2, 3) = T.at<float>(2, 0);

	transform4x4.at<float>(3, 3) = 1;
	return 0;
}
int  pointCloudProcessing::getTranslationMatrixByAlignCenterOfMassToOrigin(const cv::Mat &pointCloud, cv::Mat &T)
{
	double cumuX = 0;
	double cumuY = 0;
	double cumuZ = 0;

	for (int i = 0; i < pointCloud.rows; ++i)
	{
		cumuX += pointCloud.row(i).at<float>(0);
		cumuY += pointCloud.row(i).at<float>(1);
		cumuZ += pointCloud.row(i).at<float>(2);
	}

	double centerX = cumuX / pointCloud.rows;
	double centerY = cumuY / pointCloud.rows;
	double centerZ = cumuZ / pointCloud.rows;
	
	T = cv::Mat(3, 1, CV_32FC1);
	T.at<float>(0, 0) = -centerX;
	T.at<float>(1, 0) = -centerY;
	T.at<float>(2, 0) = -centerZ;

	return 0;
}

int pointCloudProcessing::transformPointCloud(const cv::Mat &inPointCloud, const cv::Mat &transform4x4, cv::Mat &outPointCloud)
{
	outPointCloud = cv::Mat(inPointCloud.size(), CV_32FC1);
	for (int i = 0; i < inPointCloud.rows; ++i)
	{
		cv::Mat pt = cv::Mat(1,3,CV_32FC1);

		cv::Mat homopt = cv::Mat(4, 1, CV_32FC1);
		homopt.at<float>(0, 0) = inPointCloud.row(i).at<float>(0);
		homopt.at<float>(1, 0) = inPointCloud.row(i).at<float>(1);
		homopt.at<float>(2, 0) = inPointCloud.row(i).at<float>(2);
		homopt.at<float>(3, 0) = 1;

		cv::Mat homoptTrans = cv::Mat(4, 1, CV_32FC1);
		homoptTrans = transform4x4 * homopt;
		outPointCloud.row(i).at<float>(0) = homoptTrans.at<float>(0, 0);
		outPointCloud.row(i).at<float>(1) = homoptTrans.at<float>(1, 0);
		outPointCloud.row(i).at<float>(2) = homoptTrans.at<float>(2, 0);
	}
	return 0;
}

int pointCloudProcessing::maxNminPointInPointCloud(const cv::Mat &inPointCloud, cv::Point3f &maxPoint, cv::Point3f &minPoint)
{
	float maxValue = -99999;
	cv::Point3f tmpMaxPoint;
	float minValue = 99999;
	cv::Point3f tmpMinPoint;

	for (int i = 0; i < inPointCloud.rows; ++i)
	{
		bool istmpMax = false;
		if (inPointCloud.row(i).at<float>(0) > maxValue )
		{
			maxValue = inPointCloud.row(i).at<float>(0);
			istmpMax = true;
		}
		if (inPointCloud.row(i).at<float>(1) > maxValue)
		{
			maxValue = inPointCloud.row(i).at<float>(1);
			istmpMax = true;
		}
		if (inPointCloud.row(i).at<float>(2) > maxValue)
		{
			maxValue = inPointCloud.row(i).at<float>(2);
			istmpMax = true;
		}

		bool istmpMin = false;
		if (inPointCloud.row(i).at<float>(0) < minValue)
		{
			minValue = inPointCloud.row(i).at<float>(0);
			istmpMin = true;
		}
		if (inPointCloud.row(i).at<float>(1) < minValue)
		{
			minValue = inPointCloud.row(i).at<float>(1);
			istmpMin = true;
		}
		if (inPointCloud.row(i).at<float>(2) < minValue)
		{
			minValue = inPointCloud.row(i).at<float>(2);
			istmpMin = true;
		}

		if (istmpMax)
		{
			tmpMaxPoint.x = inPointCloud.row(i).at<float>(0);
			tmpMaxPoint.y = inPointCloud.row(i).at<float>(1);
			tmpMaxPoint.z = inPointCloud.row(i).at<float>(2);
		}
		if (istmpMin)
		{
			tmpMinPoint.x = inPointCloud.row(i).at<float>(0);
			tmpMinPoint.y = inPointCloud.row(i).at<float>(1);
			tmpMinPoint.z = inPointCloud.row(i).at<float>(2);
		}
	}

	maxPoint = tmpMaxPoint;
	minPoint = tmpMinPoint;
	return 0;
}


int pointCloudProcessing::scalePointCloud(const cv::Mat &inPointCloud, float scaling_factor, cv::Mat &outPointCloud)
{
	inPointCloud.convertTo(outPointCloud, CV_32F);
	for (int i = 0; i < outPointCloud.rows; ++i)
	{
		outPointCloud.row(i).at<float>(0) *= scaling_factor;
		outPointCloud.row(i).at<float>(1) *= scaling_factor;
		outPointCloud.row(i).at<float>(2) *= scaling_factor;
	}
	return 0;
}

int pointCloudProcessing::normalizedPointCloud(const cv::Mat &inPointCloud, cv::Mat &outPointCloud)
{
	cv::Mat tempPC;
	centralizedPointCloudToOrigin(inPointCloud, tempPC);

	cv::Point3f maxPoint;
	cv::Point3f minPoint;
	maxNminPointInPointCloud(tempPC, maxPoint, minPoint);

	float maxDimension = -99999;

	float d;
	d = fabs(maxPoint.x);
	if (d > maxDimension)
	{
		maxDimension = d;
	}
	d = fabs(maxPoint.y);
	if (d > maxDimension)
	{
		maxDimension = d;
	}
	d = fabs(maxPoint.z);
	if (d > maxDimension)
	{
		maxDimension = d;
	}
	d = fabs(minPoint.x);
	if (d > maxDimension)
	{
		maxDimension = d;
	}
	d = fabs(minPoint.y);
	if (d > maxDimension)
	{
		maxDimension = d;
	}
	d = fabs(minPoint.z);
	if (d > maxDimension)
	{
		maxDimension = d;
	}

	scalePointCloud(tempPC, 1.0 / maxDimension, outPointCloud);
	return 0;
}

int pointCloudProcessing::centralizedPointCloudToOrigin(const cv::Mat &inPointCloud, cv::Mat &outPointCloud)
{
	cv::Mat T;
	pointCloudProcessing::getTranslationMatrixByAlignCenterOfMassToOrigin(inPointCloud, T);
	cv::Mat translate44;
	pointCloudProcessing::generate4X4MatrixFromTranslation(T, translate44);
	pointCloudProcessing::transformPointCloud(inPointCloud, translate44, outPointCloud);
	return 0;
}

#ifdef USE_PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

int pointCloudProcessing::downSamplePointCloud(const cv::Mat &inPointCloud, int downSampleTargetNumber, cv::Mat &outPointCloud)
{
	if (inPointCloud.empty())
	{
		return -1;
	}
	if (inPointCloud.cols != 3)
	{
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the CloudIn data
	cloud_in->width = 1;
	cloud_in->height = inPointCloud.rows;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);

	#pragma omp parallel for
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		//cloud_in->
		cloud_in->points[i].x = (float)inPointCloud.at<float>(i, 0);
		cloud_in->points[i].y = (float)inPointCloud.at<float>(i, 1);
		cloud_in->points[i].z = (float)inPointCloud.at<float>(i, 2);
	}

	//Downsample input and target point cloud
	pcl::RandomSample<pcl::PointXYZ> Pcl_RandomSample;

	Pcl_RandomSample.setInputCloud(cloud_in);
	Pcl_RandomSample.setSample(downSampleTargetNumber);
	pcl::PointCloud<pcl::PointXYZ> cloud_in_sample;
	Pcl_RandomSample.filter(cloud_in_sample);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_DN(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_in_DN = cloud_in_sample;

	outPointCloud = cv::Mat(cloud_in_DN->points.size(), 3, CV_32FC1);

#pragma omp parallel for
	for (size_t i = 0; i < cloud_in_DN->points.size(); ++i)
	{
		outPointCloud.at<float>(i, 0) = cloud_in_DN->points[i].x;
		outPointCloud.at<float>(i, 1) = cloud_in_DN->points[i].y;
		outPointCloud.at<float>(i, 2) = cloud_in_DN->points[i].z;
	}
	return 0;
}

int pointCloudProcessing::normalEstimation(const cv::Mat &inPointCloud, cv::Mat &outNormal)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the CloudIn data
	cloud_in->width = 1;
	cloud_in->height = inPointCloud.rows;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);

#pragma omp parallel for
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		//cloud_in->
		cloud_in->points[i].x = (float)inPointCloud.at<float>(i, 0);
		cloud_in->points[i].y = (float)inPointCloud.at<float>(i, 1);
		cloud_in->points[i].z = (float)inPointCloud.at<float>(i, 2);
	}

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud_in);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	//ne.setRadiusSearch(0.03);
	ne.setKSearch(10);
	// Compute the features
	ne.compute(*cloud_normals);

	outNormal = cv::Mat(cloud_in->points.size(), 3, CV_32FC1);
#pragma omp parallel for
	for (size_t i = 0; i < cloud_normals->points.size(); ++i)
	{
		outNormal.at<float>(i, 0) = cloud_normals->points[i].normal_x;
		outNormal.at<float>(i, 1) = cloud_normals->points[i].normal_y;
		outNormal.at<float>(i, 2) = cloud_normals->points[i].normal_z;
	}
	return 0;
}

#else
int pointCloudProcessing::downSamplePointCloud(const cv::Mat &inPointCloud, int downSampleTargetNumber, cv::Mat &outPointCloud)
{
	std::cout << "Please build code with PCL.\n";
	return -1;
}
int pointCloudProcessing::normalEstimation(const cv::Mat &inPointCloud, cv::Mat &outNormal)
{
	std::cout << "Please build code with PCL.\n";
	return -1;
}
#endif