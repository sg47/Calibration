
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <android/log.h>

#include "CataCameraParameters.h"
#include "Chessboard.h"
#include "MutualCalibration.h"




MutualCalibration::MutualCalibration(size_t heightImage, size_t widthImage, size_t heightBoard, size_t widthBoard, 
									 bool useOpenCVCorner, bool useOpenCVCalibration)
	: mImageSize(cv::Size(widthImage, heightImage)), 
	  mBoardSize(cv::Size(widthBoard, heightBoard)), 
	  mUseOpenCVCorner(useOpenCVCorner), 
	  mUseOpenCVCalibration(useOpenCVCalibration)
{
	mSquareSize = 1.0f; 
}

size_t 
MutualCalibration::getNumberOfImages() const
{
	return mImagePoints.size(); 
}

void 
MutualCalibration::addIMUData(double r0, double r1, double r2)
{
	cv::Mat rvec(3, 1, CV_64F); 
	rvec.at<double>(0) = r0; 
	rvec.at<double>(1) = r1; 
	rvec.at<double>(2) = r2; 
	double a = cv::norm(rvec); 
	a = 2 * asin(a); 
	rvec = rvec / cv::norm(rvec) * a; 

	mrvecsIMU.push_back(rvec); 
/*	cv::Mat R; 
	cv::Rodrigues(rvec, R); 
	R = R.t(); 

	__android_log_print(
	ANDROID_LOG_INFO, "IMUR", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf", 
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), 
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), 
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
*/		
}

bool 
MutualCalibration::tryAddingChessboardImage(cv::Mat & inputImage, cv::Mat & outputImage)
{
	vcharge::Chessboard chessboard(mBoardSize, inputImage); 
	chessboard.findCorners(mUseOpenCVCorner); 
	chessboard.getSketch().copyTo(outputImage); 
	if (!chessboard.cornersFound())
		return false; 
	else 
	{
		mImagePoints.push_back(chessboard.getCorners()); 
		return true; 
	}
}

void 
MutualCalibration::calibrateCamera()
{
    std::vector< std::vector<cv::Point3f> > objectPoints;
    for (int i = 0; i < mImagePoints.size(); ++i)
    {
    	std::vector<cv::Point3f> objectPointsInView;
    	for (int j = 0; j < mBoardSize.height; ++j)
    	{
    		for (int k = 0; k < mBoardSize.width; ++k)
    		{
    			objectPointsInView.push_back(cv::Point3f(j * mSquareSize, k * mSquareSize, 0.0));
    		}
    	}
    	objectPoints.push_back(objectPointsInView);
    }

	if (1)
	{
		cv::Mat cameraMatrix, distCoeffs; 
		cv::calibrateCamera(objectPoints, mImagePoints, mImageSize, cameraMatrix, distCoeffs, mrvecsCamera, mtvecsCamera); 
	}
	
}


void 
MutualCalibration::mutualCalibrate()
{
	std::vector<cv::Mat> qsDiff; 
	
	cv::Mat averQuat(4, 1, CV_64F); 
	for (size_t i = 0; i < mrvecsCamera.size(); i++)
	{
		cv::Mat RCamera(3, 3, CV_64F), RIMU(3, 3, CV_64F); 
		cv::Rodrigues(mrvecsCamera[i], RCamera); 
		cv::Rodrigues(mrvecsIMU[i], RIMU); 
		RIMU = RIMU.t(); 

		__android_log_print(
		ANDROID_LOG_INFO, "RCamera", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf \n", 
			RCamera.at<double>(0, 0), RCamera.at<double>(0, 1), RCamera.at<double>(0, 2), 
			RCamera.at<double>(1, 0), RCamera.at<double>(1, 1), RCamera.at<double>(1, 2), 
			RCamera.at<double>(2, 0), RCamera.at<double>(2, 1), RCamera.at<double>(2, 2));
		__android_log_print(
		ANDROID_LOG_INFO, "RIMU", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf \n", 
			RIMU.at<double>(0, 0), RIMU.at<double>(0, 1), RIMU.at<double>(0, 2), 
			RIMU.at<double>(1, 0), RIMU.at<double>(1, 1), RIMU.at<double>(1, 2), 
			RIMU.at<double>(2, 0), RIMU.at<double>(2, 1), RIMU.at<double>(2, 2));



		cv::Mat RDiff = RIMU.inv() * RCamera; 
		cv::Mat rDiff; 
		cv::Rodrigues(RDiff, rDiff); 

		double dp = RIMU.col(2).dot(RCamera.col(2)); 
		__android_log_print(
		ANDROID_LOG_INFO, "dotproduct", "%lf", dp);

		cv::Mat qDiff(4, 1, CV_64F); 
		double a, x, y, z; 
		a = cv::norm(rDiff); 
		x = rDiff.at<double>(0) / a; 
		y = rDiff.at<double>(1) / a; 
		z = rDiff.at<double>(2) / a;  
		qDiff.at<double>(0) = cv::cos(a / 2.0); 
		qDiff.at<double>(1) = -cv::sin(a / 2.0) * x; 
		qDiff.at<double>(2) = -cv::sin(a / 2.0) * y; 
		qDiff.at<double>(3) = -cv::sin(a / 2.0) * z; 

		averQuat += qDiff; 
		qsDiff.push_back(qDiff); 
		std::cout << qDiff << std::endl; 

		__android_log_print(
		ANDROID_LOG_INFO, "qDiff", "%lf %lf %lf %lf", qDiff.at<double>(0), qDiff.at<double>(1), qDiff.at<double>(2), qDiff.at<double>(3));
		


/*		a = cv::norm(mrvecsCamera[i]); 
		x = mrvecsCamera[i].at<double>(0) / a; 
		y = mrvecsCamera[i].at<double>(1) / a; 
		z = mrvecsCamera[i].at<double>(2) / a; 
		boost::math::quaternion<double> q1(
				cos(a / 2.0), 
				-sin(a / 2.0) * x, 
				-sin(a / 2.0) * y, 
				-sin(a / 2.0) * z); 


		a = cv::norm(mrvecsIMU[i]); 
		x = mrvecsIMU[i].at<double>(0) / a; 
		y = mrvecsIMU[i].at<double>(1) / a; 
		z = mrvecsIMU[i].at<double>(2) / a; 
		boost::math::quaternion<double> q2(
				cos(a / 2.0), 
				-sin(a / 2.0) * x, 
				-sin(a / 2.0) * y, 
				-sin(a / 2.0) * z); 
		std::cout << q1 / q2 << std::endl; 
		std::cout << q2 / q1 << std::endl; 
*/

	}
	cv::normalize(averQuat, averQuat); 

	for (size_t i = 0; i < qsDiff.size(); i++)
	{
		rotationTest.push_back(qsDiff[i].at<double>(0)); 
		rotationTest.push_back(qsDiff[i].at<double>(1)); 
		rotationTest.push_back(qsDiff[i].at<double>(2)); 
		rotationTest.push_back(qsDiff[i].at<double>(3)); 
	}
	pRotationTest = &rotationTest[0]; 	
}

