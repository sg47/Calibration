
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

std::vector<size_t>
MutualCalibration::randPerm(size_t n) const 
{
	std::vector<size_t> perm; 
	for(size_t i = 0; i < n; i++) perm.push_back(i);

	for(size_t i = 0; i < n; i++) 
	{	
		size_t j = rand() % (n - i) + i;
		size_t t = perm[j];
		perm[j] = perm[i];
		perm[i] = t;
	}
	return perm; 
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

	cv::Mat RIMU; 
	cv::Rodrigues(rvec, RIMU); 
	RIMU = RIMU.t(); 
	mRsIMU.push_back(RIMU * 1.0); 
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
	std::vector<cv::Mat> rvecs; 
	if (1)
	{	
		cv::calibrateCamera(objectPoints, mImagePoints, mImageSize, cameraMatrix, distCoeffs, rvecs, mtsCamera); 
		for (size_t i = 0; i < rvecs.size(); i++)
		{
			cv::Mat RCamera; 
			cv::Rodrigues(rvecs[i], RCamera); 
			mRsCamera.push_back(RCamera * 1.0); 
		}
	}
		
}


void 
MutualCalibration::mutualCalibrate()
{
	if (mRsCamera.size() != mRsIMU.size()) exit(-1); 

	cv::Mat vsCamera(3, mRsCamera.size(), CV_64F); 
	cv::Mat vsIMU(3, mtsCamera.size(), CV_64F); 
	for (size_t i = 0; i < mRsCamera.size(); i++)
	{
		vsCamera.col(i) = mRsCamera[i].col(3); 
		vsIMU.col(i) = mRsIMU[i].col(3); 
	}

	// RANSAC
	size_t it = 0; 
	size_t max_iter = 100; 
	cv::Mat R_best; 
	size_t max_inliers = 0; 

	while (it < max_iter)
	{
		std::vector<size_t> perm = randPerm(vsCamera.size()); 
		cv::Mat VsCamera(3, 3, CV_64F); 
		VsCamera.col(0) = vsCamera.col(perm[0]); 
		VsCamera.col(1) = vsCamera.col(perm[1]); 
		VsCamera.col(2) = vsCamera.col(perm[2]); 
		cv::Mat VsIMU(3, 3, CV_64F); 
		VsIMU.col(0) = vsIMU.col(perm[0]); 
		VsIMU.col(1) = vsIMU.col(perm[1]); 
		VsIMU.col(2) = vsIMU.col(perm[2]); 

		// Kabsch algorihm from wikipedia
		cv::Mat A = VsIMU * VsCamera.t(); 
		cv::Mat S, U, Vt; 
		cv::svd::compute(A, S, U, Vt); 
		cv::Mat R = U * Vt; 			

		cv::Mat Vs = R * VsCamera; 
		size_t inliers = 0; 
		for (size_t i = 0; i < Vs.cols; i++)
		{
			if (cv::norm(Vs.col(i) - VsIMU.col(i)) < 0.05) inliers++; 
		}
		if (inliers > max_inlers) 
		{
			R.copyTo(R_best); 
			max_inliers = inliers; 
		}
	}
	__android_log_print(
			ANDR_bestOID_LOG_INFO, "R_best", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf",
			R_best.at<double>(0, 0), R_best.at<double>(0, 1), R_best.at<double>(0, 2),
			R_best.at<double>(1, 0), R_best.at<double>(1, 1), R_best.at<double>(1, 2),
			R_best.at<double>(2, 0), R_best.at<double>(2, 1), R_best.at<double>(2, 2));



}

