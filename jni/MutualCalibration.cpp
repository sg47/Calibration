#include <algorithm>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <android/log.h>

#include "CataCameraParameters.h"
#include "Chessboard.h"
#include "Cas1DVanishingPoint.h"
#include "RansacVanishingPoint.h"
#include "MutualCalibration.h"

void showMat(cv::Mat R, const char* s)
{
	__android_log_print(
			ANDROID_LOG_INFO, s, "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf\n",
			R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
}





MutualCalibration::MutualCalibration(size_t heightImage, size_t widthImage, size_t heightBoard, size_t widthBoard, 
									 bool useOpenCVCorner, bool useOpenCVCalibration, 
									 bool useOnlyIMUGravity, bool useChessboardHorizontal, 
									 bool useRANSAC)
	: mImageSize(cv::Size(widthImage, heightImage)), 
	  mBoardSize(cv::Size(widthBoard, heightBoard)), 
	  mUseOpenCVCorner(useOpenCVCorner), 
	  mUseOpenCVCalibration(useOpenCVCalibration), 
	  mUseOnlyIMUGravity(useOnlyIMUGravity), 
	  mUseChessboardHorizontal(useChessboardHorizontal), 
	  mUseRANSAC(useRANSAC), 
	  mChessboardMeasured(false), 
	  mChessboardImages(0), 
	  mVanishingPointImages(0)
{
	mSquareSize = 1.0f; 
}

size_t 
MutualCalibration::getNumberOfImages() const
{
	return std::max(mChessboardImages, mVanishingPointImages); 
}

void
MutualCalibration::getRotationMatrix(double p[]) const
{
	p[0] = mCamera2IMU.at<double>(0, 0); 
	p[1] = mCamera2IMU.at<double>(0, 1); 
	p[2] = mCamera2IMU.at<double>(0, 2); 
	p[3] = mCamera2IMU.at<double>(1, 0); 
	p[4] = mCamera2IMU.at<double>(1, 1); 
	p[5] = mCamera2IMU.at<double>(1, 2); 
	p[6] = mCamera2IMU.at<double>(2, 0); 
	p[7] = mCamera2IMU.at<double>(2, 1); 
	p[8] = mCamera2IMU.at<double>(2, 2); 


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

//	__android_log_print(
//				ANDROID_LOG_INFO, "perm", "%d, %d, %d", perm[0], perm[1], perm[2]);

	return perm; 
}

void 
MutualCalibration::addFullIMURotationByQuaternion(double r0, double r1, double r2)
{
	cv::Mat rvec(3, 1, CV_64F); 
	rvec.at<double>(0) = r0; 
	rvec.at<double>(1) = r1; 
	rvec.at<double>(2) = r2; 
	double a = cv::norm(rvec); 

	__android_log_print(
			ANDROID_LOG_INFO, "full", "%lf, %lf, %lf -- a = %lf", r0, r1, r2, a);



	a = 2 * asin(a); 
	rvec = rvec / cv::norm(rvec) * a; 

	cv::Mat RIMU; 
	cv::Rodrigues(rvec, RIMU); 

	RIMU = RIMU.t(); 
	mRsIMU.push_back(RIMU * 1.0); 
}

void
MutualCalibration::addIMUGravityVector(double g1, double g2, double g3)
{
	cv::Mat g(3, 1, CV_64F); 
	g.at<double>(0) = g1; 
	g.at<double>(1) = g2; 
	g.at<double>(2) = g3; 
	mgsIMU.push_back(g * 1.0); 
	cv::normalize(mgsIMU.back(), mgsIMU.back()); 
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
		mChessboardImages++; 
		return true; 
	}
}

bool
MutualCalibration::tryAddingVanishingPointImage(cv::Mat & inputImage, cv::Mat & outputImage)
{
/*	Cas1DVanishingPoint vanishingPoint(inputImage); 
	vanishingPoint.findOrthogonalVanishingPts(); 
	vanishingPoint.getSketch().copyTo(outputImage); 
	if (vanishingPoint.threeDetected())
	{
		cv::Mat R = vanishingPoint.getRotation(); 
		size_t idx = 0; 
		for (size_t i = 0; i < 3; i++)
		{
			__android_log_print(
				ANDROID_LOG_INFO, "asdf", "%lf, %lf", fabs(R.at<double>(1, idx)), fabs(R.at<double>(1, i)));
		
			if (fabs(R.at<double>(1, idx) < fabs(R.at<double>(1, i))) )
			{
		
				idx = i; 
			}
		}
		__android_log_print(
				ANDROID_LOG_INFO, "g", "%lf, %lf, %lf", R.at<double>(0, idx), R.at<double>(1, idx), R.at<double>(2, idx));
		if (R.at<double>(1, idx) > 0) 
			mgsCamera.push_back(R.col(idx) * 1.0); 
		else mgsCamera.push_back(R.col(idx) * -1.0); 

		mVanishingPointImages++; 
		return true; 
	}
	else return false; 
*/
	RansacVanishingPoint vanishingPoint(inputImage); 
	vanishingPoint.findOrthogonalVanishingPts(); 
	vanishingPoint.getSketch().copyTo(outputImage); 
	if (vanishingPoint.orthogonalityDetected())
	{
		cv::Mat R = vanishingPoint.getRotation(); 
		__android_log_print(
				ANDROID_LOG_INFO, "g", "%lf, %lf, %lf", R.at<double>(0, 2), R.at<double>(1, 2), R.at<double>(2, 2));
		mgsCamera.push_back(-R.col(2) * 1.0); 
		mVanishingPointImages++; 
		return true; 
	}
	else return false; 
}

void 
MutualCalibration::calibrateCamera()
{
	if (mImagePoints.empty()) return; 
	
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
			__android_log_print(
		ANDROID_LOG_INFO, "focal", "%lf", cameraMatrix.at<double>(0, 0));
		for (size_t i = 0; i < rvecs.size(); i++)
		{
			cv::Mat RCamera; 
			cv::Rodrigues(rvecs[i], RCamera); 
			mRsCamera.push_back(RCamera * 1.0); 
			mgsCamera.push_back(RCamera.col(2) * 1.0); 
		}
	}
		
}

void 
MutualCalibration::lsMutualCalibrateWithHorizontalChessboard()
{
	cv::Mat S(3, mgsCamera.size(), CV_64F); 
	cv::Mat T(3, mgsCamera.size(), CV_64F); 
	for (size_t i = 0; i < mgsCamera.size(); i++)
	{
		if (mUseOnlyIMUGravity)
		{
			S.col(i) = -mgsCamera[i] * 1.0; 
			T.col(i) = mgsIMU[i] * 1.0; 
			__android_log_print(
			ANDROID_LOG_INFO, "ls", "1", S.cols, S.rows); 
		}
		else
		{
			S.col(i) = mgsCamera[i] * 1.0; 
			T.col(i) = mRsIMU[i].col(2) * 1.0; 
		}
	}
__android_log_print(
			ANDROID_LOG_INFO, "R", "%d %d", S.cols, S.rows); 

	showMat(S, "S"); 
	showMat(T, "T");

	// Kabsch algorihm from wikipedia
	cv::Mat A = T * S.t();
	cv::Mat D, U, Vt; 
	cv::SVD::compute(A, D, U, Vt);
	cv::Mat R = U * Vt; 			
	__android_log_print(
			ANDROID_LOG_INFO, "R", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf\n",
			R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

	R.copyTo(mCamera2IMU); 
}

void 
MutualCalibration::ransacMutualCalibrateWithHorizontalChessboard()
{
	if (!mUseOnlyIMUGravity && mRsCamera.size() != mRsIMU.size()) exit(-1); 
	if (mUseOnlyIMUGravity && mRsCamera.size() != mgsIMU.size()) exit(-1); 

	// RANSAC
	size_t it = 0; 
	size_t max_iter = 20;
	cv::Mat R_best; 
	size_t max_inliers = 0; 
	
	while (it < max_iter)
	{
		std::vector<size_t> perm = randPerm(mRsCamera.size());
		cv::Mat S(3, 3, CV_64F); 
		cv::Mat T(3, 3, CV_64F); 
		if (mUseOnlyIMUGravity)
		{
			// R * S = T
			// S = R_c * [0; 0; -1] 
			// T = g
			S.col(0) = -mgsCamera[perm[0]] * 1.0; 
			S.col(1) = -mgsCamera[perm[1]] * 1.0; 
			S.col(2) = -mgsCamera[perm[2]] * 1.0; 
			T.col(0) = mgsIMU[perm[0]] * 1.0; 
			T.col(1) = mgsIMU[perm[1]] * 1.0; 
			T.col(2) = mgsIMU[perm[2]] * 1.0;
		}
		else
		{
			// R * S = T
			// S = R_c * [0; 0; 1]
			// T = R_i * [0; 0; 1]
			S.col(0) = mgsCamera[perm[0]] * 1.0; 
			S.col(1) = mgsCamera[perm[1]] * 1.0; 
			S.col(2) = mgsCamera[perm[2]] * 1.0; 
			T.col(0) = mRsIMU[perm[0]].col(2) * 1.0; 
			T.col(1) = mRsIMU[perm[1]].col(2) * 1.0; 
			T.col(2) = mRsIMU[perm[2]].col(2) * 1.0; 
		}

		// Kabsch algorihm from wikipedia
		cv::Mat A = T * S.t();
		cv::Mat D, U, Vt; 
		cv::SVD::compute(A, D, U, Vt);
		cv::Mat R = U * Vt; 			

		cv::Mat V(3, mRsCamera.size(), CV_64F); 
		if (mUseOnlyIMUGravity)
		{
			for (size_t i = 0; i < mgsCamera.size(); i++)
				V.col(i) = -mgsCamera[i] * 1.0; 
		}
		else
		{
			for (size_t i = 0; i < mgsCamera.size(); i++)
				V.col(i) = mgsCamera[i] * 1.0; 
		}

		cv::Mat Vs = R * V; 
		size_t inliers = 0; 
		for (size_t i = 0; i < Vs.cols; i++)
		{
			if (mUseOnlyIMUGravity)
			{
				if (cv::norm(Vs.col(i) - mgsIMU[i]) < 0.05) inliers++; 
			}
			else
			{
				if (cv::norm(Vs.col(i) - mRsIMU[i].col(2)) < 0.05) inliers++; 
			}
		}
		if (inliers > max_inliers)
		{
			R.copyTo(R_best); 
			max_inliers = inliers; 
		}
		it++;


		__android_log_print(
				ANDROID_LOG_INFO, "S", "%lf %lf \n %lf %lf \n %lf %lf ",
				S.at<double>(0, 0), S.at<double>(0, 1), 
				S.at<double>(1, 0), S.at<double>(1, 1), 
				S.at<double>(2, 0), S.at<double>(2, 1)); 
		__android_log_print(
				ANDROID_LOG_INFO, "T", "%lf %lf \n %lf %lf \n %lf %lf ",
				T.at<double>(0, 0), T.at<double>(0, 1), 
				T.at<double>(1, 0), T.at<double>(1, 1), 
				T.at<double>(2, 0), T.at<double>(2, 1)); 
		__android_log_print(
				ANDROID_LOG_INFO, "R", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf\n %d",
				R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
				R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
				R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), inliers);

	}

	if (max_inliers > 0)
		__android_log_print(
			ANDROID_LOG_INFO, "R_best", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf\n %d",
			R_best.at<double>(0, 0), R_best.at<double>(0, 1), R_best.at<double>(0, 2),
			R_best.at<double>(1, 0), R_best.at<double>(1, 1), R_best.at<double>(1, 2),
			R_best.at<double>(2, 0), R_best.at<double>(2, 1), R_best.at<double>(2, 2), max_inliers);
	else 
		__android_log_print(
			ANDROID_LOG_INFO, "R_best", "wierd"); 

	R_best.copyTo(mCamera2IMU); 

}

void
MutualCalibration::mutualCalibrate()
{
	if (mUseRANSAC)
		ransacMutualCalibrateWithHorizontalChessboard(); 
	else 
		lsMutualCalibrateWithHorizontalChessboard(); 
}
