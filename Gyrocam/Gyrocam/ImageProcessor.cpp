#include "ImageProcessor.h"
#include "IoProvider.h"
#include "Shared.h"
#include "RansacClusterizer.h"
#include "Geometry.h"
#include "TimeCounter.h"

#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace gyrocam
{
	ImageProcessor::ImageProcessor(const SingleRunConfig &config, const Settings &settings)
	{
		this->config = config;
		this->settings = settings;
	}
	void ImageProcessor::loadConfig(const SingleRunConfig &config)
	{
		this->config = config;
	}

	SingleRunResult ImageProcessor::process()
	{
		SingleRunResult result;
		TimeCounter timeCounter;
		timeCounter.StartCounter();

		initColors();
		loadImage();
		loadCalibrationMatrices();

		extractLineSegments();

		std::vector<cv::Point3d> vps;
		for (int i = 0; i < 3; i++)
		{
			if (notUsedSegments.size() < 2)
				break;

			cv::Point3d refinedVpNormalized = findVanishingPoint(vps);
			vps.push_back(refinedVpNormalized);
		}

		result.vpBasis = getRotationMatrix(vps);
		refineVpBasis(result.vpBasis);
		result.orthoVpBasis = getNearestOrthogonalMatrix(result.vpBasis);
		reorderColumn(result.orthoVpBasis, 1);
		reorderColumn(result.orthoVpBasis, 0);
		reorderColumn(result.orthoVpBasis);		
		
		result.eulerAngles = getEulerAngles(result.orthoVpBasis.t());
		result.runTime = timeCounter.GetCounter();
		saveResults(result);
		drawProcessedImage(getEulerAnglesString(result.eulerAngles));

		return result;
	}
	void ImageProcessor::refineVpBasis(cv::Mat &vpBasis)
	{
		cv::Mat c = vpBasis;
		cv::Mat r = vpBasis.t();
		//std::cout << c << std::endl;
		
		for (int i = 0; i < 3; i++)
		{
			auto yi = (i+1)%3;
			auto zi = (i+2)%3;
			double a1 = abs(angleBetween(r.row(i), c.col(yi)) - CV_PI/2);
			double a2 = abs(angleBetween(r.row(i), c.col(zi)) - CV_PI/2);
			if (min(a1, a2) > 0.2)
			{
				vpBasis.col(i) = 0 * vpBasis.col(i);
				break;
			}			
			if (max(a1, a2) > 1.0)
			{
				int col = a1 >= a2 ? yi : zi;
				vpBasis.col(col) = 0 * vpBasis.col(col);
				break;
			}
			
		}
	}
	cv::Point3d ImageProcessor::smartVpRefinement(const std::vector<LineSegment> &originCluster, int i)
	{
		int steps = 2;
		cv::Point3d vp, normalizedVp;
		std::vector<int> indices;
		std::vector<LineSegment> cluster = originCluster;
		do
		{
			normalizedVp = refineVanishingPoint(cluster);
			cv::Point3d t(normalizedVp);
			normalizeZ(t);
			vp = fromNormalized(cv::Mat(t));
			normalizeZ(vp);
		
			indices = ransac::getInducedSegments(segments, notUsedSegments, vp, ANGLE_EPSILON / (4 - steps));
			cluster = ransac::resolveIndices(segments, indices);
			
			--steps;
		}
		while (steps > 0 && cluster.size() > 3);
			
		ransac::markInducedSegmentsAsUsed(notUsedSegments, indices);
		drawInducedCluster(vp, cluster, colors[i], settings.BUILD_IMAGE);

		return normalizedVp;
	}

	cv::Point3d ImageProcessor::findVanishingPoint(std::vector<cv::Point3d> &vps)
	{
		int i = vps.size() + 1;
		cv::Point3d originVp, refinedVp;
		std::vector<LineSegment> originCluster = ransac::nextCluster(segments, notUsedSegments, originVp);
		
		drawInducedCluster(originVp, originCluster, colors[i]/2, settings.BUILD_IMAGE && settings.DRAW_RAW_SEGMENTS);
		
		if (originCluster.size() < 3)
		{
			ransac::markInducedSegmentsAsUsed(notUsedSegments, ransac::getInducedSegments(segments, notUsedSegments, originVp, ANGLE_EPSILON));
			return refinedVp;
		}
		
		refinedVp = smartVpRefinement(originCluster, i);
		ransac::markInducedSegmentsAsUsed(notUsedSegments, ransac::getInducedSegments(segments, notUsedSegments, originVp, ANGLE_EPSILON));
		
		if (settings.TRACE_ENABLED)
			std::cout << "OriginVP" << i << std::endl << originVp << std::endl << "RefinedVP" << i << std::endl << refinedVp << std::endl;
		return refinedVp;
	}

	cv::Point3d ImageProcessor::refineVanishingPoint(const std::vector<LineSegment> &cluster)
	{
		cv::Mat A = cv::Mat::zeros(cluster.size(), 3, CV_64FC1);
		for (int i = 0; i < cluster.size(); i++)
			setRow(A, i, toNormalized(cluster[i]));

		//std::cout << A << std::endl;

		cv::Mat res = cv::Mat::zeros(3, 1, CV_64FC1);
		solveZ(A, res);
		res = res.t();

		if (settings.TRACE_ENABLED)
		{
			cv::Mat res1 = cv::Mat::zeros(3, 1, CV_64FC1);
			cv::SVD::solveZ(A, res1);

			std::cout << "SolveZ Manually: " << std::endl << res << std::endl;
			std::cout << "SolveZ: " <<std::endl << res1 << std::endl;
			std::cout << "SolveZ diff: " << norm(res - res1) << std::endl;
		}

		return cv::Point3d(res);
	}

	void ImageProcessor::solveZ(const cv::Mat &a, cv::Mat &res)
	{
		cv::SVD svd(a, cv::SVD::FULL_UV);

		if (settings.TRACE_ENABLED)
		{
			std::cout << "SVD results:" << std::endl;
			std::cout << "W: " << svd.w << std::endl;
			std::cout << "Vt: " << svd.vt << std::endl;
		}

		/*double maxSigma = svd.w.rows > 0 ? svd.w.at<double>(0, 0) : 1;
		for (int i = 0; i < svd.w.rows; i++)
		{
			double d = svd.w.at<double>(i, 0);
			if (d / maxSigma < INFINITE_POINT_Z_EPSILON)
			{
				res = svd.vt.row(i == 0 ? i : i - 1);
				return;
			}
		}*/

		res = svd.vt.row(svd.w.rows - 1);
	}

	void ImageProcessor::initColors()
	{
		if (colors.empty())
			colors = getColors();
	}
	void ImageProcessor::loadCalibrationMatrices()
	{
		if (calibrationMatrix.rows > 0)
			return;

		auto path = config.getCalibrationMatrixPath();
		calibrationMatrix = readCalibrationMatrix(path);

		if (path.empty())
		{
			calibrationMatrix.at<double>(0, 2) = image.cols / 2;
			calibrationMatrix.at<double>(1, 2) = image.rows / 2;
		}
		inversedCalibrationMatrix = calibrationMatrix.inv();
	}

	void ImageProcessor::loadImage()
	{
		image = cv::imread(config.getInputImagePath());
		image = resizeImage(image, settings.POCKET_SIZE);
	}

	void ImageProcessor::extractLineSegments()
	{
		// to grayscale
		cv::Mat grayImage;
		cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

		// Detect the lines
		std::vector<cv::Vec4i> lines_std;
		cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(cv::LSD_REFINE_STD);
		ls->detect(grayImage, lines_std);
		
		segments = LineSegment::toLineSegments(lines_std, findMinAllowedLineSegmentLength(image));
		notUsedSegments.clear();
		for (int i = 0; i < segments.size(); i++)
			notUsedSegments.insert(i);
		
		if (settings.BUILD_IMAGE && settings.DRAW_RAW_SEGMENTS)
		{
			drawFoundSegments(segments, image, colors[0], 1);
		}
		if (settings.TRACE_ENABLED)
		{
			std::cout << "Raw segments: " << lines_std.size() << "; passed length filter: " << segments.size() << std::endl;
		}
	}
	
	cv::Mat ImageProcessor::getRotationMatrix(const std::vector<cv::Point3d> &vps)
	{
		cv::Mat A = cv::Mat::zeros(3, 3, CV_64FC1);
		for (int i = 0; i < vps.size(); i++)
		{
			double d = norm(vps[i]);
			if (d > 1e-10)
				setRow(A, i, cv::Mat(vps[i] / d));
		}

		return A.t();
	}

	void ImageProcessor::drawInducedCluster(const cv::Point3d &vp, const std::vector<LineSegment> &cluster, const cv::Scalar &color, bool isDrawNeeded)
	{
		if (isDrawNeeded)
		{
			drawFoundSegments(cluster, image, color);
			drawPointMarkerIfVisibleOnImage(vp, image, color);
		}
	}

	void ImageProcessor::drawProcessedImage(std::string title)
	{
		if (settings.BUILD_IMAGE)
		{
			imwrite(config.getOutputImagePath(), image);		
			if (settings.SHOW_IMAGE)
				imshow(title, image);
		}
	}

	void ImageProcessor::saveResults(const SingleRunResult &result)
	{
		saveMatrix(config.getOutputNonOrthogonalVpsPath(), result.vpBasis);
		saveMatrix(config.getOutputOrthogonalVpsPath(), result.orthoVpBasis);
		saveMatrix(config.getOutputAnglesPath(), result.eulerAngles);
		
		if (settings.TRACE_ENABLED)
		{
			std::cout << result.vpBasis << std::endl;
			std::cout << result.eulerAngles << std::endl;
		}
		if (settings.TRACE_ENABLED || settings.SHOW_IMAGE)
		{
			std::cout << result.orthoVpBasis << std::endl;
		}
		if (settings.BUILD_IMAGE)
		{
			drawEulerAngles(image, result.eulerAngles);
			//drawAxes(image, 20 * calibrationMatrix * result.orthoVpBasis.t());
		}
	}
	
	cv::Mat ImageProcessor::toNormalized(const cv::Mat &line)
	{
		return inversedCalibrationMatrix * line;
	}

	cv::Mat ImageProcessor::toNormalized(const LineSegment &s)
	{
		cv::Mat from(s.from);
		cv::Mat to(s.to);
		//std::cout << from << std::endl << to << std::endl;
		from = toNormalized(from);
		to = toNormalized(to);
		//std::cout << from << std::endl << to << std::endl;

		cv::Mat line = from.cross(to);
		if (norm(line) > 1e-12)
			line /= norm(line);
		//std::cout << line << std::endl;
		return line;
	}
	cv::Point3d ImageProcessor::fromNormalized(const cv::Mat &line)
	{
		cv::Mat t = calibrationMatrix * line;
		return cv::Point3d(t);
	}


	ImageProcessor::~ImageProcessor(void)
	{
	}
}