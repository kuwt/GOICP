#ifdef USE_PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/conversions.h>
#endif

#include "BasicVisualizer.h"
#include "vtkAutoInit.h"

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkCenterOfMass.h>
#include <vtkTransform.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkProperty.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPLYReader.h>
#include <vtkSTLReader.h>
#include <vtkSimplePointsReader.h>
#include <string>
#include <algorithm>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkDataSet.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkRenderWindow.h>
#include <vtkArrowSource.h>
#include <vtkGlyph3D.h>
#include <vtkDoubleArray.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>

namespace BasicVisualizer
{

	BasicVisualizer::BasicVisualizer()
	{
		VTK_MODULE_INIT(vtkRenderingOpenGL2); // VTK was built with vtkRenderingOpenGL2
		VTK_MODULE_INIT(vtkInteractionStyle);
		VTK_MODULE_INIT(vtkRenderingFreeType);

		m_pRenderer = vtkSmartPointer<vtkRenderer>::New();
	}


	BasicVisualizer::~BasicVisualizer()
	{
	}

	void BasicVisualizer::setPointCloudRandomColor(const std::string &id)
	{
		if (ActorsContains(id)) // found
		{
			float r = ((double)rand() / (RAND_MAX));
			float g = ((double)rand() / (RAND_MAX));
			float b = ((double)rand() / (RAND_MAX));

			vtkSmartPointer<vtkActor> pActor = m_MapActors.at(id);
			pActor->GetProperty()->SetColor(r, g, b);
		}
	}

	void BasicVisualizer::setPointCloudRenderingProperties(int RenderProperty,
		double value1,
		double value2,
		double value3,
		const std::string &id)
	{
		switch (RenderProperty)
		{
		case COLOR:
			if (ActorsContains(id)) // found
			{
				vtkSmartPointer<vtkActor> pActor = m_MapActors.at(id);
				pActor->GetProperty()->SetColor(value1, value2, value3);
			}
			break;
		default:
			break;
		}
	}

	void BasicVisualizer::setPointCloudRenderingProperties(int RenderProperty,
		double value1,
		double value2,
		const std::string &id)
	{


	}

	void BasicVisualizer::setPointCloudRenderingProperties(int RenderProperty,
		double value,
		const std::string &id)
	{
		switch (RenderProperty) {
		case POINT_SIZE:
			if (ActorsContains(id)) // found
			{
				vtkSmartPointer<vtkActor> pActor = m_MapActors.at(id);
				pActor->GetProperty()->SetPointSize(value);
			}
			break;
		case OPACITY:
			if (ActorsContains(id)) // found
			{
				vtkSmartPointer<vtkActor> pActor = m_MapActors.at(id);
				pActor->GetProperty()->SetOpacity(value);
			}
			break;
		case LINE_WIDTH:
			if (ActorsContains(id)) // found
			{
				vtkSmartPointer<vtkActor> pActor = m_MapActors.at(id);
				pActor->GetProperty()->SetLineWidth(value);
			}
			break;
		default:
			;
		}

	}

	void BasicVisualizer::setBackgroundColor(double value1,
		double value2,
		double value3)
	{
		m_BackgroundColor[0] = value1;
		m_BackgroundColor[1] = value2;
		m_BackgroundColor[2] = value3;

		if (m_pRenderer)
		{
			m_pRenderer->SetBackground(value1, value2, value3);
		}
	}

#ifdef USE_PCL
	void BasicVisualizer::addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}
		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		{
			pcl::io::pointCloudTovtkPolyData(*pCloud, polydata);
		}
		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(polydata);
		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(mapper);
		m_MapActors[id] = pActor;
		m_vIDInOrder.push_back(id);
	}
	void BasicVisualizer::addPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloud, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		{
			pcl::io::pointCloudTovtkPolyData(*pCloud, polydata);
		}
		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(polydata);
		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(mapper);
		m_MapActors[id] = pActor;

		vtkSmartPointer<vtkPolyDataAlgorithm> dummy = vtkSmartPointer<vtkPolyDataAlgorithm>::New();
		m_vIDInOrder.push_back(id);
	}
#endif

#ifdef USE_OPENCV
	void BasicVisualizer::addPointCloud(const cv::Mat &pointCloud, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}
		if (pointCloud.empty())
		{
			std::cout << "pointCloud empty." << "\n";
			return;
		}

		cv::Mat Model;
		pointCloud.convertTo(Model, CV_32F);

		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		for (int j = 0; j < Model.size().height; ++j)
		{
			double x = Model.at<float>(j, 0);
			double y = Model.at<float>(j, 1);
			double z = Model.at<float>(j, 2);
			points->InsertNextPoint(x, y, z);
		}
		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->SetPoints(points);

		vtkSmartPointer<vtkVertexGlyphFilter> pVertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();

		pVertexGlyphFilter->AddInputData(polydata);
		pVertexGlyphFilter->Update();
		addPointCloud(pVertexGlyphFilter, id);
	}

	void BasicVisualizer::addPointCloudWTextureColor(const cv::Mat &pointCloud, const cv::Mat &Color, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}
		if (pointCloud.empty())
		{
			std::cout << "pointCloud empty." << "\n";
			return;
		}
		if (Color.empty())
		{
			std::cout << "color empty" << "\n";
			return;
		}
		if (pointCloud.size() != Color.size())
		{
			std::cout << "pointCloud color size not align." << "\n";
			return;
		}

		cv::Mat Model;
		pointCloud.convertTo(Model, CV_32F);
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		for (int j = 0; j < Model.size().height; ++j)
		{
			float x = Model.at<float>(j, 0);
			float y = Model.at<float>(j, 1);
			float z = Model.at<float>(j, 2);
			points->InsertNextPoint(x, y, z);
		}

		vtkSmartPointer<vtkPolyData> pointspolydata = vtkSmartPointer<vtkPolyData>::New();
		pointspolydata->SetPoints(points);

		vtkSmartPointer<vtkVertexGlyphFilter> pVertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
		pVertexGlyphFilter->SetInputData(pointspolydata);
		pVertexGlyphFilter->Update();

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->ShallowCopy(pVertexGlyphFilter->GetOutput());
		/*****************color *************************/

		vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
		colors->SetNumberOfComponents(3);
		colors->SetName("Colors");

		for (int j = 0; j < Color.size().height; ++j)
		{
			unsigned char color[3];
			for (unsigned int n = 0; n < 3; n++)
			{
				color[n] = static_cast<unsigned char>(Color.at<cv::Vec3b>(j)[n]);
			}
			colors->InsertNextTypedTuple(color);
		}

		polydata->GetPointData()->SetScalars(colors);

		vtkSmartPointer<vtkPolyDataMapper> pMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pMapper->SetInputData(polydata);

		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(pMapper);

		m_MapActors[id] = pActor;
		m_vIDInOrder.push_back(id);
	}

	void BasicVisualizer::addPointCloudWVectorField(const cv::Mat &pointCloud, const cv::Mat &field, const float vectorScale, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}
		if (pointCloud.empty())
		{
			std::cout << "pointCloud empty." << "\n";
			return;
		}

		if (field.empty())
		{
			std::cout << "field empty" << "\n";
			return;
		}
		if (pointCloud.size() != field.size())
		{
			std::cout << "pointCloud field size not align." << "\n";
			return;
		}
		/***************************** load points *******************************/
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		{
			cv::Mat pointCloud32F;
			pointCloud.convertTo(pointCloud32F, CV_32F);

			for (int j = 0; j < pointCloud32F.size().height; ++j)
			{
				float x = pointCloud32F.at<float>(j, 0);
				float y = pointCloud32F.at<float>(j, 1);
				float z = pointCloud32F.at<float>(j, 2);
				points->InsertNextPoint(x, y, z);
			}
		}
		vtkSmartPointer<vtkPolyData> pointspolydata = vtkSmartPointer<vtkPolyData>::New();
		pointspolydata->SetPoints(points);
		
		/***************************** load normals *******************************/
		vtkSmartPointer<vtkDoubleArray> pointNormalsArray = vtkSmartPointer<vtkDoubleArray>::New();
		{
			pointNormalsArray->SetNumberOfComponents(3); //3d normals (ie x,y,z)
			
			cv::Mat field32F;
			field.convertTo(field32F, CV_32F);

			int NumOfData = field32F.size().height;
			pointNormalsArray->SetNumberOfTuples(NumOfData);

			for (int i = 0; i < field32F.size().height; ++i)
			{
				float x = field32F.at<float>(i, 0);
				float y = field32F.at<float>(i, 1);
				float z = field32F.at<float>(i, 2);
				double pN[3] = { x, y, z };
				pointNormalsArray->SetTuple(i, pN);
			}
		}
		/*****************************
		// Set normals to point clouds
		*******************************/
		pointspolydata->GetPointData()->SetNormals(pointNormalsArray);

		/*****************************
		// Create arrows for the point normal
		*******************************/
		vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
		{
			vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();
			glyph3D->SetSourceConnection(arrowSource->GetOutputPort());
			glyph3D->OrientOn();
			glyph3D->SetVectorModeToUseNormal();
			glyph3D->SetScaleFactor(vectorScale);
			glyph3D->SetInputData(pointspolydata);
			glyph3D->Update();
		}
		/*****************************
		// Visualize
		*******************************/
		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(glyph3D->GetOutputPort());

		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(mapper);

		m_MapActors[id] = pActor;
		m_vIDInOrder.push_back(id);
	}

	void BasicVisualizer::addPointCloud2DGridWTextureColor(const cv::Mat &pointCloud, const cv::Mat &Color, const cv::Mat &isValid, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}
		if (pointCloud.empty())
		{
			std::cout << "pointCloud empty." << "\n";
			return;
		}
		if (Color.empty())
		{
			std::cout << "color empty" << "\n";
			return;
		}
		if (isValid.empty())
		{
			std::cout << "isValid empty" << "\n";
			return;
		}
		if (pointCloud.size() != Color.size() || pointCloud.size() != isValid.size())
		{
			std::cout << "pointCloud color isValid size not align." << "\n";
			return;
		}

		cv::Mat Model;
		pointCloud.convertTo(Model, CV_32F);
		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		for (int h = 0; h < Model.rows; h++)
		{
			for (int w = 0; w < Model.cols; w++)
			{
				if (isValid.at<unsigned char>(h, w) == 1)
				{
					double x = Model.at<cv::Vec3f>(h, w)[0];
					double y = Model.at<cv::Vec3f>(h, w)[1];
					double z = Model.at<cv::Vec3f>(h, w)[2];
					points->InsertNextPoint(x, y, z);
				}
			}
		}

		vtkSmartPointer<vtkPolyData> pointspolydata = vtkSmartPointer<vtkPolyData>::New();
		pointspolydata->SetPoints(points);

		vtkSmartPointer<vtkVertexGlyphFilter> pVertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
		pVertexGlyphFilter->SetInputData(pointspolydata);
		pVertexGlyphFilter->Update();

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->ShallowCopy(pVertexGlyphFilter->GetOutput());
		/*****************color *************************/

		vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
		colors->SetNumberOfComponents(3);
		colors->SetName("Colors");

		for (int h = 0; h < Color.rows; h++)
		{
			for (int w = 0; w < Color.cols; w++)
			{
				if (isValid.at<unsigned char>(h, w) == 1)
				{
					unsigned char color[3];
					for (unsigned int n = 0; n < 3; n++)
					{
						color[n] = static_cast<unsigned char>(Color.at<cv::Vec3b>(h, w)[n]);
					}
					colors->InsertNextTypedTuple(color);
				}
			}
		}

		polydata->GetPointData()->SetScalars(colors);

		vtkSmartPointer<vtkPolyDataMapper> pMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pMapper->SetInputData(polydata);

		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(pMapper);

		m_MapActors[id] = pActor;
		m_vIDInOrder.push_back(id);
	}
#endif

	void BasicVisualizer::addPointCloud(const std::string& path,
		const std::string& pointCloudType,
		const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}

		if (pointCloudType == "stl")
		{
			vtkSmartPointer<vtkSTLReader> pReader = vtkSmartPointer<vtkSTLReader>::New();
			const char * c = path.c_str();
			pReader->SetFileName(c);
			addPointCloud(pReader, id);
		}
		else if (pointCloudType == "ply")
		{
			vtkSmartPointer<vtkPLYReader> pReader = vtkSmartPointer<vtkPLYReader>::New();
			const char * c = path.c_str();
			pReader->SetFileName(c);
			addPointCloud(pReader, id);
		}
		else if (pointCloudType == "ply_noFace")
		{
			vtkSmartPointer<vtkPLYReader> pReader = vtkSmartPointer<vtkPLYReader>::New();
			const char * c = path.c_str();
			pReader->SetFileName(c);

			vtkSmartPointer<vtkVertexGlyphFilter> pVertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
			pVertexGlyphFilter->SetInputConnection(pReader->GetOutputPort());
			addPointCloud(pVertexGlyphFilter, id);
		}
		else if (pointCloudType == "raw")
		{
			vtkSmartPointer<vtkSimplePointsReader> pReader = vtkSmartPointer<vtkSimplePointsReader>::New();
			const char * c = path.c_str();
			pReader->SetFileName(c);
			addPointCloud(pReader, id);
		}
	}

	void BasicVisualizer::addPointCloud(vtkSmartPointer<vtkPolyDataAlgorithm> polydataAlg,
		const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}

		vtkSmartPointer<vtkPolyDataMapper> pMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pMapper->SetInputConnection(polydataAlg->GetOutputPort());
		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(pMapper);
		m_MapActors[id] = pActor;

		m_vIDInOrder.push_back(id);
	}

	void BasicVisualizer::addText(const std::string &text, const int locationX, const int locationY, float fontsize, double r, double g, double b, const std::string &id)
	{
		// Setup the text and add it to the renderer
		vtkSmartPointer<vtkTextActor> ptextActor = vtkSmartPointer<vtkTextActor>::New();
		ptextActor->SetInput(text.c_str());
		ptextActor->SetPosition(locationX, locationY);
		ptextActor->GetTextProperty()->SetFontSize(fontsize);
		ptextActor->GetTextProperty()->SetColor(r, g, b);
		m_MapActors2D[id] = ptextActor;

		m_vIDInOrder.push_back(id);
	}

	void BasicVisualizer::addLine(const float x1, const float y1, const float z1,
		const float x2, const float y2, const float z2,
		double r, double g, double b, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}

		double p0[3] = { x1,y1,z1 };
		double p1[3] = { x2,y2,z2 };

		vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
		pts->InsertNextPoint(p0);
		pts->InsertNextPoint(p1);
		linesPolyData->SetPoints(pts);

		vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
		line0->GetPointIds()->SetNumberOfIds(2);
		line0->GetPointIds()->SetId(0, 0); // the second 0 is the index of the Origin in linesPolyData's points
		line0->GetPointIds()->SetId(1, 1); // the second 1 is the index of P0 in linesPolyData's points

		vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
		lines->InsertNextCell(line0);

		// Add the lines to the polydata container
		linesPolyData->SetLines(lines);

		unsigned char theColor[3] = { r * 255, g * 255, b * 255 };
		vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
		colors->SetNumberOfComponents(3);
		colors->SetName("Colors");
		colors->InsertNextTypedTuple(theColor);
		linesPolyData->GetCellData()->SetScalars(colors);

		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(linesPolyData);

		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(mapper);

		m_MapActors[id] = pActor;

		m_vIDInOrder.push_back(id);
	}

	void BasicVisualizer::addPoint(const float x1, const float y1, const float z1, 
		double r, double g, double b, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}

		double x = x1;
		double y = y1;
		double z = z1;

		vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
		points->InsertNextPoint(x, y, z);

		vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
		pointsPolydata->SetPoints(points);

		vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
		vertexFilter->SetInputData(pointsPolydata);
		vertexFilter->Update();

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
		polydata->ShallowCopy(vertexFilter->GetOutput());

		unsigned char theColor[3] = { r * 255, g * 255, b * 255 };
		vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
		colors->SetNumberOfComponents(3);
		colors->SetName("Colors");
		colors->InsertNextTypedTuple(theColor);
		polydata->GetPointData()->SetScalars(colors);

		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(polydata);

		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(mapper);

		m_MapActors[id] = pActor;
		m_vIDInOrder.push_back(id);
	}

	void BasicVisualizer::addArrowLine(
		const float x1, const float y1, const float z1,
		const float x2, const float y2, const float z2,
		double r, double g, double b, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}

		double startPoint[3], endPoint[3];
		startPoint[0] = x1;
		startPoint[1] = y1;
		startPoint[2] = z1;
		endPoint[0] = x2;
		endPoint[1] = y2;
		endPoint[2] = z2;

		vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

		// Compute a basis
		double normalizedX[3];
		double normalizedY[3];
		double normalizedZ[3];

		// The X axis is a vector from start to end
		vtkMath::Subtract(endPoint, startPoint, normalizedX);
		double length = vtkMath::Norm(normalizedX);
		vtkMath::Normalize(normalizedX);

		// The Z axis is an arbitrary vector cross X
		double arbitrary[3];
		arbitrary[0] = vtkMath::Random(-10, 10);
		arbitrary[1] = vtkMath::Random(-10, 10);
		arbitrary[2] = vtkMath::Random(-10, 10);
		vtkMath::Cross(normalizedX, arbitrary, normalizedZ);
		vtkMath::Normalize(normalizedZ);

		// The Y axis is Z cross X
		vtkMath::Cross(normalizedZ, normalizedX, normalizedY);
		vtkSmartPointer<vtkMatrix4x4> matrix =
			vtkSmartPointer<vtkMatrix4x4>::New();

		// Create the direction cosine matrix
		matrix->Identity();
		for (unsigned int i = 0; i < 3; i++)
		{
			matrix->SetElement(i, 0, normalizedX[i]);
			matrix->SetElement(i, 1, normalizedY[i]);
			matrix->SetElement(i, 2, normalizedZ[i]);
		}

		// Apply the transforms
		vtkSmartPointer<vtkTransform> transform =
			vtkSmartPointer<vtkTransform>::New();
		transform->Translate(startPoint);
		transform->Concatenate(matrix);
		transform->Scale(length, length, length);

		// Transform the polydata
		vtkSmartPointer<vtkTransformPolyDataFilter> transformPD =
			vtkSmartPointer<vtkTransformPolyDataFilter>::New();
		transformPD->SetTransform(transform);

		vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();
		arrowSource->SetShaftRadius(0.005);
		arrowSource->SetTipLength(0.05);
		arrowSource->SetTipRadius(0.03);
		transformPD->SetInputConnection(arrowSource->GetOutputPort());

		vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		vtkSmartPointer<vtkActor> pActor =
			vtkSmartPointer<vtkActor>::New();
		mapper->SetInputConnection(transformPD->GetOutputPort());
		pActor->SetMapper(mapper);

		m_MapActors[id] = pActor;
		m_vIDInOrder.push_back(id);
	}

	void BasicVisualizer::addOrigin(
		const float scale, double r, double g, double b, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}
		double p0[3] = { 0, 0 ,0 };
		double p1[3] = { 1 * scale, 0, 0 };
		double p2[3] = { 0, 1 * scale, 0 };
		double p3[3] = { 0, 0, 1* scale};

		vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
		vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
		{
			pts->InsertNextPoint(p0);
			pts->InsertNextPoint(p1);

			pts->InsertNextPoint(p2);
			pts->InsertNextPoint(p3);
		}
		linesPolyData->SetPoints(pts);
		{
			vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
			line0->GetPointIds()->SetNumberOfIds(2);
			line0->GetPointIds()->SetId(0, 0);
			line0->GetPointIds()->SetId(1, 1);
			lines->InsertNextCell(line0);

			vtkSmartPointer<vtkLine> line1 = vtkSmartPointer<vtkLine>::New();
			line1->GetPointIds()->SetNumberOfIds(2);
			line1->GetPointIds()->SetId(0, 0);
			line1->GetPointIds()->SetId(1, 2);
			lines->InsertNextCell(line1);

			vtkSmartPointer<vtkLine> line2 = vtkSmartPointer<vtkLine>::New();
			line2->GetPointIds()->SetNumberOfIds(2);
			line2->GetPointIds()->SetId(0, 0);
			line2->GetPointIds()->SetId(1, 3);
			lines->InsertNextCell(line2);

			vtkSmartPointer<vtkLine> line3 = vtkSmartPointer<vtkLine>::New();
			line3->GetPointIds()->SetNumberOfIds(2);
			line3->GetPointIds()->SetId(0, 0);
			line3->GetPointIds()->SetId(1, 4);
			lines->InsertNextCell(line3);
		}

		// Add the lines to the polydata container
		linesPolyData->SetLines(lines);

		unsigned char theColor[3] = { r * 255, g * 255, b * 255 };
		vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
		colors->SetNumberOfComponents(3);
		colors->SetName("Colors");
		for (int i = 0; i < 4; ++i)
		{
			colors->InsertNextTypedTuple(theColor);
		}
		linesPolyData->GetCellData()->SetScalars(colors);

		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(linesPolyData);

		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(mapper);


		m_MapActors[id] = pActor;

		m_vIDInOrder.push_back(id);
	}

	void BasicVisualizer::addSquarePrism(
		const float TLx, const float TLy,
		const float BRx, const float BRy,
		const float zMin, const float zMax,
		double r, double g, double b, const std::string &id)
	{
		if (ActorsContains(id))
		{
			return;
		}

		
		double p0_down[3] = { TLx,TLy,zMin };
		double p0_up[3] = { TLx,TLy,zMax };

		double p1_down[3] = { BRx,TLy,zMin };
		double p1_up[3] = { BRx,TLy,zMax };

		double p2_down[3] = { TLx,BRy,zMin };
		double p2_up[3] = { TLx,BRy,zMax };

		double p3_down[3] = { BRx,BRy,zMin };
		double p3_up[3] = { BRx,BRy,zMax };

		vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
		vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
		{
			pts->InsertNextPoint(p0_down);
			pts->InsertNextPoint(p0_up);

			pts->InsertNextPoint(p1_down);
			pts->InsertNextPoint(p1_up);

			pts->InsertNextPoint(p2_down);
			pts->InsertNextPoint(p2_up);

			pts->InsertNextPoint(p3_down);
			pts->InsertNextPoint(p3_up);
		}
		linesPolyData->SetPoints(pts);
		{
			vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
			line0->GetPointIds()->SetNumberOfIds(2);
			line0->GetPointIds()->SetId(0, 0);
			line0->GetPointIds()->SetId(1, 1);
			lines->InsertNextCell(line0);

			vtkSmartPointer<vtkLine> line0_b1 = vtkSmartPointer<vtkLine>::New();
			line0_b1->GetPointIds()->SetNumberOfIds(2);
			line0_b1->GetPointIds()->SetId(0, 0);
			line0_b1->GetPointIds()->SetId(1, 2);
			lines->InsertNextCell(line0_b1);

			vtkSmartPointer<vtkLine> line0_b2 = vtkSmartPointer<vtkLine>::New();
			line0_b2->GetPointIds()->SetNumberOfIds(2);
			line0_b2->GetPointIds()->SetId(0, 0);
			line0_b2->GetPointIds()->SetId(1, 4);
			lines->InsertNextCell(line0_b2);

			vtkSmartPointer<vtkLine> line0_t1 = vtkSmartPointer<vtkLine>::New();
			line0_t1->GetPointIds()->SetNumberOfIds(2);
			line0_t1->GetPointIds()->SetId(0, 1);
			line0_t1->GetPointIds()->SetId(1, 3);
			lines->InsertNextCell(line0_t1);

			vtkSmartPointer<vtkLine> line0_t2 = vtkSmartPointer<vtkLine>::New();
			line0_t2->GetPointIds()->SetNumberOfIds(2);
			line0_t2->GetPointIds()->SetId(0, 1);
			line0_t2->GetPointIds()->SetId(1, 5);
			lines->InsertNextCell(line0_t2);

			vtkSmartPointer<vtkLine> line1 = vtkSmartPointer<vtkLine>::New();
			line1->GetPointIds()->SetNumberOfIds(2);
			line1->GetPointIds()->SetId(0, 2); 
			line1->GetPointIds()->SetId(1, 3); 
			lines->InsertNextCell(line1);


			vtkSmartPointer<vtkLine> line2 = vtkSmartPointer<vtkLine>::New();
			line2->GetPointIds()->SetNumberOfIds(2);
			line2->GetPointIds()->SetId(0, 4);
			line2->GetPointIds()->SetId(1, 5);
			lines->InsertNextCell(line2);

			vtkSmartPointer<vtkLine> line3 = vtkSmartPointer<vtkLine>::New();
			line3->GetPointIds()->SetNumberOfIds(2);
			line3->GetPointIds()->SetId(0, 6);
			line3->GetPointIds()->SetId(1, 7);
			lines->InsertNextCell(line3);

			vtkSmartPointer<vtkLine> line3_b1 = vtkSmartPointer<vtkLine>::New();
			line3_b1->GetPointIds()->SetNumberOfIds(2);
			line3_b1->GetPointIds()->SetId(0, 6);
			line3_b1->GetPointIds()->SetId(1, 4);
			lines->InsertNextCell(line3_b1);

			vtkSmartPointer<vtkLine> line3_b2 = vtkSmartPointer<vtkLine>::New();
			line3_b2->GetPointIds()->SetNumberOfIds(2);
			line3_b2->GetPointIds()->SetId(0, 6);
			line3_b2->GetPointIds()->SetId(1, 2);
			lines->InsertNextCell(line3_b2);

			vtkSmartPointer<vtkLine> line3_t1 = vtkSmartPointer<vtkLine>::New();
			line3_t1->GetPointIds()->SetNumberOfIds(2);
			line3_t1->GetPointIds()->SetId(0, 7);
			line3_t1->GetPointIds()->SetId(1, 3);
			lines->InsertNextCell(line3_t1);

			vtkSmartPointer<vtkLine> line3_t2 = vtkSmartPointer<vtkLine>::New();
			line3_t2->GetPointIds()->SetNumberOfIds(2);
			line3_t2->GetPointIds()->SetId(0, 7);
			line3_t2->GetPointIds()->SetId(1, 5);
			lines->InsertNextCell(line3_t2);
		}
		// Add the lines to the polydata container
		linesPolyData->SetLines(lines);

		unsigned char theColor[3] = { r * 255, g * 255, b * 255 };
		vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
		colors->SetNumberOfComponents(3);
		colors->SetName("Colors");
		for (int i = 0; i < 12; ++i)
		{
			colors->InsertNextTypedTuple(theColor);
		}
		linesPolyData->GetCellData()->SetScalars(colors);

		vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(linesPolyData);

		vtkSmartPointer<vtkActor> pActor = vtkSmartPointer<vtkActor>::New();
		pActor->SetMapper(mapper);

	
		m_MapActors[id] = pActor;

		m_vIDInOrder.push_back(id);
	}


	void BasicVisualizer::TransformObject(const TRANS_PARA &Trans, const std::string &id)
	{
		std::map<std::string, vtkSmartPointer<vtkActor>>::iterator  iter = m_MapActors.find(id);
		if (iter != m_MapActors.end()) //found
		{
			vtkSmartPointer<vtkCenterOfMass> COMFilter = vtkSmartPointer<vtkCenterOfMass>::New();
			COMFilter->SetInputData(vtkPolyData::SafeDownCast(iter->second->GetMapper()->GetInput()));
			COMFilter->SetUseScalarsAsWeights(false);
			COMFilter->Update();

			double center[3];
			COMFilter->GetCenter(center);


			vtkSmartPointer<vtkTransform> ptrans = vtkSmartPointer<vtkTransform>::New();
			ptrans->PostMultiply();
			ptrans->Translate(-center[0], -center[1], -center[2]);
			const float pi = 3.141592658;
			ptrans->RotateX(Trans.alpha * 180.0 / pi);
			ptrans->RotateY(Trans.beta * 180.0 / pi);
			ptrans->RotateZ(Trans.gamma * 180.0 / pi);
			ptrans->Scale(Trans.scale, Trans.scale, Trans.scale);
			ptrans->Translate(center[0], center[1], center[2]);

			ptrans->Translate(Trans.dx, Trans.dy, Trans.dz);
			iter->second->SetUserTransform(ptrans);
		}

		m_pRenderer->ResetCamera();

	}

#ifdef USE_OPENCV
	void BasicVisualizer::TransformObject(const cv::Mat &Trans, const std::string &id)
	{
		std::map<std::string, vtkSmartPointer<vtkActor>>::iterator  iter = m_MapActors.find(id);
		if (iter != m_MapActors.end()) //found
		{
			vtkSmartPointer<vtkTransform> ptrans = vtkSmartPointer<vtkTransform>::New();
			{
				cv::Mat transMat;
				Trans.convertTo(transMat, CV_64F);

				vtkSmartPointer<vtkMatrix4x4> m = vtkSmartPointer<vtkMatrix4x4>::New();

				m->SetElement(0, 0, transMat.at<double>(0, 0));
				m->SetElement(0, 1, transMat.at<double>(0, 1));
				m->SetElement(0, 2, transMat.at<double>(0, 2));
				m->SetElement(0, 3, transMat.at<double>(0, 3));
				m->SetElement(1, 0, transMat.at<double>(1, 0));
				m->SetElement(1, 1, transMat.at<double>(1, 1));
				m->SetElement(1, 2, transMat.at<double>(1, 2));
				m->SetElement(1, 3, transMat.at<double>(1, 3));
				m->SetElement(2, 0, transMat.at<double>(2, 0));
				m->SetElement(2, 1, transMat.at<double>(2, 1));
				m->SetElement(2, 2, transMat.at<double>(2, 2));
				m->SetElement(2, 3, transMat.at<double>(2, 3));
				m->SetElement(3, 0, transMat.at<double>(3, 0));
				m->SetElement(3, 1, transMat.at<double>(3, 1));
				m->SetElement(3, 2, transMat.at<double>(3, 2));
				m->SetElement(3, 3, transMat.at<double>(3, 3));
				
				ptrans->SetMatrix(m);
			}
			
			iter->second->SetUserTransform(ptrans);
		}

		m_pRenderer->ResetCamera();



	}
#endif
	void BasicVisualizer::TransformObject(const vtkSmartPointer<vtkMatrix4x4> pMat, const std::string &id)
	{
		std::map<std::string, vtkSmartPointer<vtkActor>>::iterator  iter = m_MapActors.find(id);
		if (iter != m_MapActors.end()) //found
		{
			vtkSmartPointer<vtkTransform> ptrans = vtkSmartPointer<vtkTransform>::New();

			ptrans->SetMatrix(pMat);
			iter->second->SetUserTransform(ptrans);
		}
		m_pRenderer->ResetCamera();

	}

	void BasicVisualizer::deleteObject(const std::string &id)
	{
		if (ActorsContains(id)) // found
		{
			m_MapActors.erase(id);
		}
		if (Actors2DContains(id)) // found
		{
			m_MapActors2D.erase(id);
		}
		if (IDInOrderContains(id)) // found
		{
			m_vIDInOrder.erase(std::remove(m_vIDInOrder.begin(), m_vIDInOrder.end(), id), m_vIDInOrder.end());
		}
	}

	void BasicVisualizer::ConstructRenderer()
	{
		m_pRenderer->RemoveAllViewProps();

		for (int i = 0; i < m_vIDInOrder.size(); ++i)
		{
			std::string id = m_vIDInOrder.at(i);
			if (m_MapActors.find(id) != m_MapActors.end()) // found
			{
				m_pRenderer->AddActor(m_MapActors[id]);
			}

			if (m_MapActors2D.find(id) != m_MapActors2D.end()) // found
			{
				m_pRenderer->AddActor(m_MapActors2D[id]);
			}
		}
		m_pRenderer->SetBackground(m_BackgroundColor[0], m_BackgroundColor[1], m_BackgroundColor[2]);

		m_pRenderer->ResetCamera();
	}
	bool BasicVisualizer::ActorsContains(const std::string &id)
	{
		if (m_MapActors.find(id) != m_MapActors.end()) // found
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool BasicVisualizer::Actors2DContains(const std::string &id)
	{
		if (m_MapActors2D.find(id) != m_MapActors2D.end()) // found
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool BasicVisualizer::IDInOrderContains(const std::string &id)
	{
		if (std::find(m_vIDInOrder.begin(), m_vIDInOrder.end(), id) != m_vIDInOrder.end())  // found
		{
			return true;
		}
		else
		{
			return false;
		}

	}


	vtkSmartPointer<vtkRenderer> BasicVisualizer::GetRenderer()
	{
		ConstructRenderer();
		return m_pRenderer;
	}

	void BasicVisualizer::Render()
	{
		ConstructRenderer();

		vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
		renderWindow->AddRenderer(m_pRenderer);

		vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
		renderWindowInteractor->SetRenderWindow(renderWindow);

		renderWindow->Render();
		renderWindowInteractor->Start();
	}

	void BasicVisualizer::CleanDisplay()
	{
		m_pRenderer->RemoveAllViewProps();

	}

	void BasicVisualizer::clearVisualizer()
	{
		CleanDisplay();
		m_MapActors.clear();
		m_MapActors2D.clear();
		m_vIDInOrder.clear();
	}

}