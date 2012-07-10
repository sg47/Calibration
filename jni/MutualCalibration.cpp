
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

cv::Mat cameraMatrix, distCoeffs; 
	if (1)
	{
		
		cv::calibrateCamera(objectPoints, mImagePoints, mImageSize, cameraMatrix, distCoeffs, mrvecsCamera, mtvecsCamera); 
	}

/*	for (size_t i = 0; i < objectPoints.size(); i++)
		for (size_t j = 0; j < objectPoints[i].size(); j++)
		{
			cv::Mat X(3, 1, CV_64F); 
			X.at<double>(0) = objectPoints[i][j].x; 
			X.at<double>(1) = objectPoints[i][j].y; 
			X.at<double>(2) = 0; 

			cv::Mat R, t, x; 
			cv::Rodrigues(mrvecsCamera[i], R); 
			t = mtvecsCamera[i]; 
			x = cameraMatrix * (R * X + t); 
			float xx = x.at<double>(0) / x.at<double>(2); 
			float yy = x.at<double>(1) / x.at<double>(2); 
			x = cameraMatrix * (R.t() * X + t); 
			float xxx = x.at<double>(0) / x.at<double>(2); 
			float yyy = x.at<double>(1) / x.at<double>(2); 
	
				__android_log_print(
	ANDROID_LOG_INFO, "project", "%lf %lf | %lf %lf | %lf %lf", 
	xx, yy, xxx, yyy, mImagePoints[i][j].x, mImagePoints[i][j].y); 

		}
*/
	
}


void 
MutualCalibration::mutualCalibrate()
{
	std::vector<cv::Mat> qsDiff; 
	
		cv::Mat P(3, 3, CV_64F); 
		P.at<double>(0, 0) = 0; 
		P.at<double>(0, 1) = -1; 
		P.at<double>(0, 2) = 0; 
		P.at<double>(1, 0) = -1; 
		P.at<double>(1, 1) = 0; 
		P.at<double>(1, 2) = 0; 
		P.at<double>(2, 0) = 0; 
		P.at<double>(2, 1) = 0; 
		P.at<double>(2, 2) = -1; 

	cv::Mat averQuat(4, 1, CV_64F); 
	for (size_t i = 0; i < mrvecsCamera.size(); i++)
	{
		cv::Mat RCamera(3, 3, CV_64F), RIMU(3, 3, CV_64F); 
		cv::Rodrigues(mrvecsCamera[i], RCamera); 
		cv::Rodrigues(mrvecsIMU[i], RIMU); 
		RIMU = RIMU.t(); 		

		cv::Mat Q(3, 3, CV_64F); 

		__android_log_print(
		ANDROID_LOG_INFO, "ri", "%lf %lf %lf", RIMU.at<double>(0, 2), RIMU.at<double>(1, 2), RIMU.at<double>(2, 2));
		__android_log_print(
		ANDROID_LOG_INFO, "rc", "%lf %lf %lf", RCamera.at<double>(0, 2), RCamera.at<double>(1, 2), RCamera.at<double>(2, 2));
	

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

