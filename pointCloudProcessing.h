#pragma once
#include <opencv2\opencv.hpp>

class pointCloudProcessing
{
public:
	static int getTranslationMatrixByAlignCenterOfMassToOrigin(const cv::Mat &pointCloud, cv::Mat &T);
	static int transformPointCloud(const cv::Mat &inPointCloud, const cv::Mat &transform4x4, cv::Mat &outPointCloud);
	static int generateRandomRotationMatrix(cv::Mat &R);
	static int generate4X4MatrixFromRotation(const cv::Mat &R, cv::Mat &transform4x4);
	static int generate4X4MatrixFromTranslation(const cv::Mat &T, cv::Mat &transform4x4);
	static int generate4X4MatrixFromRotationNTranslation(const cv::Mat &R, const cv::Mat &T, cv::Mat &transform4x4);
	static int normalizedPointCloud(const cv::Mat &inPointCloud, cv::Mat &outPointCloud);
	static int centralizedPointCloudToOrigin(const cv::Mat &inPointCloud, cv::Mat &outPointCloud);

	static int maxNminPointInPointCloud(const cv::Mat &inPointCloud, cv::Point3f &maxPoint, cv::Point3f &minPoint);
	static int scalePointCloud(const cv::Mat &inPointCloud, float scaling_factor, cv::Mat &outPointCloud);

	static int downSamplePointCloud(const cv::Mat &inPointCloud, int downSampleTargetNumber, cv::Mat &outPointCloud);
	static int normalEstimation(const cv::Mat &inPointCloud, cv::Mat &outNormal);
};
