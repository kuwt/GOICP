/********************************************************************
Main Function for point cloud registration with Go-ICP Algorithm
Last modified: Feb 13, 2014

"Go-ICP: Solving 3D Registration Efficiently and Globally Optimally"
Jiaolong Yang, Hongdong Li, Yunde Jia
International Conference on Computer Vision (ICCV), 2013

Copyright (C) 2013 Jiaolong Yang (BIT and ANU)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/

#include <time.h>
#include <iostream>
#include <fstream>
using namespace std;

#include "jly_goicp.h"
#include "ConfigMap.hpp"
#include "BasicVisualizer.h"
#include "pointCloudProcessing.h"

#define DEFAULT_OUTPUT_FNAME "../demo/output.txt"
#define DEFAULT_CONFIG_FNAME "../demo/config.txt"
#define DEFAULT_MODEL_FNAME "../demo/model_bunny.txt"
#define DEFAULT_DATA_FNAME "../demo/data_bunny.txt"
#define DEFAULT_DOWNSAMPLE (500)

void parseInput(int argc, char **argv, string & modelFName, string & dataFName, int & NdDownsampled, string & configFName, string & outputFName);
void readConfig(string FName, GoICP & goicp);
int loadPointCloud(string FName, int & N, POINT3D **  p);

int main(int argc, char** argv)
{
	int Nm, Nd, NdDownsampled;
	clock_t  clockBegin, clockEnd;
	string modelFName, dataFName, configFName, outputFname;
	POINT3D * pModel, * pData;
	GoICP goicp;

	parseInput(argc, argv, modelFName, dataFName, NdDownsampled, configFName, outputFname);
	readConfig(configFName, goicp);

	// Load model and data point clouds
	loadPointCloud(modelFName, Nm, &pModel);
	loadPointCloud(dataFName, Nd, &pData);
	
	goicp.pModel = pModel;
	goicp.Nm = Nm;
	goicp.pData = pData;
	goicp.Nd = Nd;

	// Build Distance Transform
	cout << "Building Distance Transform..." << flush;
	clockBegin = clock();
	goicp.BuildDT();
	clockEnd = clock();
	cout << (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC << "s (CPU)" << endl;

	// Run GO-ICP
	if(NdDownsampled > 0)
	{
		goicp.Nd = NdDownsampled; // Only use first NdDownsampled data points (assumes data points are randomly ordered)
	}
	cout << "Model ID: " << modelFName << " (" << goicp.Nm << "), Data ID: " << dataFName << " (" << goicp.Nd << ")" << endl;
	cout << "Registering..." << endl;
	clockBegin = clock();
	goicp.Register();
	clockEnd = clock();
	double time = (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC;
	cout << "Optimal Rotation Matrix:" << endl;
	cout << goicp.optR << endl;
	cout << "Optimal Translation Vector:" << endl;
	cout << goicp.optT << endl;
	cout << "Finished in " << time << endl;

	
	ofstream ofile;
	ofile.open(outputFname.c_str(), ofstream::out);
	ofile << time << endl;
	ofile << goicp.optR << endl;
	ofile << goicp.optT << endl;
	ofile.close();
	
	cv::Mat transform4x4;
	cv::Mat R = cv::Mat(cv::Size(3, 3), CV_32FC1);
	cv::Mat T = cv::Mat(cv::Size(1, 3), CV_32FC1);
	T.at<float>(0, 0) = goicp.optT.val[0][0];
	T.at<float>(1, 0) = goicp.optT.val[1][0];
	T.at<float>(2, 0) = goicp.optT.val[2][0];

	R.at<float>(0, 0) = goicp.optR.val[0][0]; R.at<float>(0, 1) = goicp.optR.val[0][1]; R.at<float>(0, 2) = goicp.optR.val[0][2];
	R.at<float>(1, 0) = goicp.optR.val[1][0]; R.at<float>(1, 1) = goicp.optR.val[1][1]; R.at<float>(1, 2) = goicp.optR.val[1][2];
	R.at<float>(2, 0) = goicp.optR.val[2][0]; R.at<float>(2, 1) = goicp.optR.val[2][1]; R.at<float>(2, 2) = goicp.optR.val[2][2];

	pointCloudProcessing::generate4X4MatrixFromRotationNTranslation(R, T, transform4x4);

	cv::Mat inPointCloud = cv::Mat(cv::Size(3, Nm), CV_32FC1);
	cv::Mat outPointCloud = cv::Mat(cv::Size(3, Nd), CV_32FC1);;

	for (int i = 0; i < Nm; ++i)
	{
		inPointCloud.at<float>(i, 0) = pModel[i].x;
		inPointCloud.at<float>(i, 1) = pModel[i].y;
		inPointCloud.at<float>(i, 2) = pModel[i].z;
	}
	for (int i = 0; i < Nd; ++i)
	{
		outPointCloud.at<float>(i, 0) = pData[i].x;
		outPointCloud.at<float>(i, 1) = pData[i].y;
		outPointCloud.at<float>(i, 2) = pData[i].z;
	}
	cv::Mat outPointCloudTrans;
	pointCloudProcessing::transformPointCloud(outPointCloud,transform4x4, outPointCloudTrans);
	
	/************ display ***************/
	{
		BasicVisualizer::BasicVisualizer vis;

		cv::Mat PointCloud;
		vis.addPointCloud(outPointCloudTrans, "pointcloudTarget");
		vis.addText("pointcloudTarget", 0, 12, 12, 1, 1, 1, "pointcloudTarget_text");
		vis.setPointCloudRenderingProperties(BasicVisualizer::COLOR,
			1.0,
			1.0,
			1.0,
			"pointcloudTarget");

		vis.addPointCloud(inPointCloud, "pointcloudSrc");
		vis.addText("pointcloudSrc", 0, 0, 12, 1, 1, 1, "pointcloudSrc_text");
		vis.setPointCloudRenderingProperties(BasicVisualizer::COLOR,
			0,
			1.0,
			0,
			"pointcloudSrc");
		vis.setBackgroundColor(0.0, 0.0, 0);
		vis.Render();
		vis.clearVisualizer();
	}

	delete(pModel);
	delete(pData);

	cout << "Press enter to continue... " << endl;
	getchar();
	return 0;
}

void parseInput(int argc, char **argv, string & modelFName, string & dataFName, int & NdDownsampled, string & configFName, string & outputFName)
{
	// Set default values
	modelFName = DEFAULT_MODEL_FNAME;
	dataFName = DEFAULT_DATA_FNAME;
	configFName = DEFAULT_CONFIG_FNAME;
	outputFName = DEFAULT_OUTPUT_FNAME;
	NdDownsampled = DEFAULT_DOWNSAMPLE; // No downsampling

	//cout << endl;
	//cout << "USAGE:" << "./GOICP <MODEL FILENAME> <DATA FILENAME> <NUM DOWNSAMPLED DATA POINTS> <CONFIG FILENAME> <OUTPUT FILENAME>" << endl;
	//cout << endl;

	if(argc > 5)
	{
		outputFName = argv[5];
	}
	if(argc > 4)
	{
		configFName = argv[4];
	}
	if(argc > 3)
	{
		NdDownsampled = atoi(argv[3]);
	}
	if(argc > 2)
	{
		dataFName = argv[2];
	}
	if(argc > 1)
	{
		modelFName = argv[1];
	}

	cout << "INPUT:" << endl;
	cout << "(modelFName)->(" << modelFName << ")" << endl;
	cout << "(dataFName)->(" << dataFName << ")" << endl;
	cout << "(NdDownsampled)->(" << NdDownsampled << ")" << endl;
	cout << "(configFName)->(" << configFName << ")" << endl;
	cout << "(outputFName)->(" << outputFName << ")" << endl;
	cout << endl;
}

void readConfig(string FName, GoICP & goicp)
{
	// Open and parse the associated config file
	ConfigMap config(FName.c_str());

	goicp.MSEThresh = config.getF("MSEThresh");
	goicp.initNodeRot.a = config.getF("rotMinX");
	goicp.initNodeRot.b = config.getF("rotMinY");
	goicp.initNodeRot.c = config.getF("rotMinZ");
	goicp.initNodeRot.w = config.getF("rotWidth");
	goicp.initNodeTrans.x = config.getF("transMinX");
	goicp.initNodeTrans.y = config.getF("transMinY");
	goicp.initNodeTrans.z = config.getF("transMinZ");
	goicp.initNodeTrans.w = config.getF("transWidth");
	goicp.trimFraction = config.getF("trimFraction");
	// If < 0.1% trimming specified, do no trimming
	if(goicp.trimFraction < 0.001)
	{
		goicp.doTrim = false;
	}
	goicp.dt.SIZE = config.getI("distTransSize");
	goicp.dt.expandFactor = config.getF("distTransExpandFactor");

	cout << "CONFIG:" << endl;
	config.print();
	//cout << "(doTrim)->(" << goicp.doTrim << ")" << endl;
	cout << endl;
}

int loadPointCloud(string FName, int & N, POINT3D ** p)
{
	int i;
	ifstream ifile;

	ifile.open(FName.c_str(), ifstream::in);
	if(!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	ifile >> N; // First line has number of points to follow
	*p = (POINT3D *)malloc(sizeof(POINT3D) * N);
	for(i = 0; i < N; i++)
	{
		ifile >> (*p)[i].x >> (*p)[i].y >> (*p)[i].z;
	}

	ifile.close();

	return 0;
}
