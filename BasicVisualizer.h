#pragma once
/**************************************************

A Very Basic VTK Visualizer for visualizing 3D pointclouds

* If you plan to use Point Cloud Library(PCL) as input point cloud format, 
add USE_MY_PCL to preprocessor

* If you plan to use opencv cvmat  as input point cloud format,
add USE_OPENCV to preprocessor

* See test.cpp for examples. 

Author: kuwingto @ Feb 8, 2019
*****************************************************/

#include <vtkSmartPointer.h>

#include <vtkPolyDataAlgorithm.h>
#include <vtkMatrix4x4.h>
#include <vtkRenderer.h>
#include <vtkActor.h>
#include <map>
#include <vector>


#ifdef USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif

#ifdef USE_OPENCV
#include <opencv2\opencv.hpp>
#endif
namespace BasicVisualizer
{
	/** \brief Set of rendering properties. */
	enum RenderProp
	{
		POINT_SIZE,            /**< integer starting from 1 */
		OPACITY,               /**< Float going from 0.0 (transparent) to 1.0 (opaque) */
		LINE_WIDTH,            /**< Integer starting from 1 */
		COLOR,                 /**< 3 floats (R, G, B) going from 0.0 (dark) to 1.0 (light) */
	};
}

namespace BasicVisualizer
{
	struct TRANS_PARA
	{
		// Rigid body has 6 degree of freedom
		float dx;
		float dy;
		float dz;
		float alpha;
		float beta;
		float gamma;
		float scale;


		TRANS_PARA::TRANS_PARA() : dx(0), dy(0), dz(0), alpha(0), beta(0), gamma(0), scale(1) {}
		TRANS_PARA::TRANS_PARA(float dx, float dy, float dz, float alpha, float beta, float gamma, float scale) : dx(dx), dy(dy), dz(dz), alpha(alpha), beta(beta), gamma(gamma), scale(scale) {}
	};

	class BasicVisualizer
	{
	public:
		BasicVisualizer();
		~BasicVisualizer();

		/************************************************************
		setPointCloudRenderingProperties for object in the renderer
		Depending on RenderProperty, the number of arguments is different
		RenderProp defines RenderProperty
		The last argument is the id of the object currently in the renderer
		*******************************************************/
		void setPointCloudRenderingProperties(int RenderProperty,
			double value1,
			double value2,
			double value3,
			const std::string &id);

		void setPointCloudRenderingProperties(int RenderProperty,
			double value1,
			double value2,
			const std::string &id);

		void setPointCloudRenderingProperties(int RenderProperty,
			double value,
			const std::string &id);

		void setPointCloudRandomColor(const std::string &id);
		/***************************
		set Renderer Background Color rgb
		e.g. 1, 0.5, 1
		*************************/
		void setBackgroundColor(double value1,
			double value2,
			double value3);

		/***************************
		add PointCloud by different format
		*************************/
#ifdef USE_PCL
		void addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, const std::string &id);
		void addPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud, const std::string &id);
#endif

#ifdef USE_OPENCV
		void addPointCloud(const cv::Mat &pointCloud, const std::string &id);
		void addPointCloudWTextureColor(const cv::Mat &pointCloud, const cv::Mat &Color, const std::string &id);
		/******* vector scale controls the size of the arrow representing the vector. If the pointcloud is in a greater scale, use a greater number. e.g. 1 ********/
		void addPointCloudWVectorField(const cv::Mat &pointCloud, const cv::Mat &field, const float vectorScale, const std::string &id);
		void addPointCloud2DGridWTextureColor(const cv::Mat &pointCloud, const cv::Mat &Color, const cv::Mat &isValid, const std::string &id);
		
#endif
		void addPointCloud(vtkSmartPointer<vtkPolyDataAlgorithm> polydataAlg, const std::string &id);
		void addPointCloud(const std::string& path, const std::string& pointCloudType, const std::string &id);
		/***************************
		add text
		*************************/
		void addText(const std::string &text, const int locationX, const int locationY, float fontsize, double r, double g, double b, const std::string &id);
	
		/**********************************
		add a line
		r,g,b from 0 to 1
		***********************************/
		void addLine(const float x1, const float y1, const float z1,
					const float x2, const float y2, const float z2,
					double r, double g, double b, const std::string &id);

		/**********************************
		add a line
		r,g,b from 0 to 1
		***********************************/
		void addArrowLine(const float x1, const float y1, const float z1,
			const float x2, const float y2, const float z2,
			double r, double g, double b, const std::string &id);

		/**********************************
		add a point
		r,g,b from 0 to 1
		***********************************/
		void addPoint(const float x1, const float y1, const float z1, double r, double g, double b, const std::string &id);

		/**********************************
		add upright square prism 
		r,g,b from 0 to 1
		***********************************/
		void addSquarePrism(
			const float TLx, const float TLy,
			const float BRx, const float BRy,
			const float zMin, const float zMax,
			double r, double g, double b, const std::string &id);

		/**********************************
		Add origin
		***********************************/
		void BasicVisualizer::addOrigin(
			const float scale, double r, double g, double b, const std::string &id);
		/******************************************************
		Transform object by differetn transformation format
		**************************************************/
		void TransformObject(const TRANS_PARA &Trans, const std::string &id);

#ifdef USE_OPENCV
		void TransformObject(const cv::Mat &Trans, const std::string &id);
#endif
		
		void TransformObject(const vtkSmartPointer<vtkMatrix4x4> pMat, const std::string &id);

		/******************************************************
		Delete object currently in the renderer
		**************************************************/
		void deleteObject(const std::string &id);

		/******************************************************
		Get renderer, Use external render window , e.g. qt
		**************************************************/
		vtkSmartPointer<vtkRenderer> GetRenderer();

		/******************************************************
		render directly by calling a new window, if use external render window, never call this
		**************************************************/
		void Render();

		/******************************************************
		clear the render window and all the objects currently in visualizer
		**************************************************/
		void clearVisualizer();

	private:
		void ConstructRenderer();
		void CleanDisplay();
		bool ActorsContains(const std::string &id);
		bool Actors2DContains(const std::string &id);
		bool IDInOrderContains(const std::string &id);

		vtkSmartPointer<vtkRenderer> m_pRenderer;

		std::vector<std::string>  m_vIDInOrder;
		std::map<std::string, vtkSmartPointer<vtkActor>>  m_MapActors;
		std::map<std::string, vtkSmartPointer<vtkActor2D>> m_MapActors2D;

		float m_BackgroundColor[3] = { 0,0,0 };
	};
}

