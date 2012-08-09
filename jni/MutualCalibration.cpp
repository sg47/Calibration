#include <cassert>
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
									 bool useOpenCVCorner, bool useOnlyIMUGravity, bool useRANSAC)
	: mImageSize(cv::Size(widthImage, heightImage)), 
	  mBoardSize(cv::Size(widthBoard, heightBoard)), 
	  mUseOpenCVCorner(useOpenCVCorner),
	  mUseOnlyIMUGravity(useOnlyIMUGravity),
	  mUseOpenCVCalibration(false),
	  mUseChessboardHorizontal(false),
	  mUseRANSAC(useRANSAC),
	  mChessboardMeasured(false), 
	  mChessboardImages(0), 
	  mVanishingPointImages(0),
	  mSquareSize(1.0f)
{
}

size_t 
MutualCalibration::getNumberOfImages() const
{
	assert(mChessboardImages == 0 || mVanishingPointImages == 0);
	return std::max(mChessboardImages, mVanishingPointImages); 
}

void
MutualCalibration::getRotationMatrix(double p[]) const
{
	// store columnwise
	cv::Mat mCamera2IMU2;
	cv::transpose(mCamera2IMU, mCamera2IMU2);
	p[0] = mCamera2IMU2.at<double>(0, 0);
	p[1] = mCamera2IMU2.at<double>(0, 1);
	p[2] = mCamera2IMU2.at<double>(0, 2);
	p[3] = mCamera2IMU2.at<double>(1, 0);
	p[4] = mCamera2IMU2.at<double>(1, 1);
	p[5] = mCamera2IMU2.at<double>(1, 2);
	p[6] = mCamera2IMU2.at<double>(2, 0);
	p[7] = mCamera2IMU2.at<double>(2, 1);
	p[8] = mCamera2IMU2.at<double>(2, 2);
}

void
MutualCalibration::getCameraMatrix(double p[]) const
{
	// store columnwise
	cv::Mat mKCamera2;
	cv::transpose(mKCamera, mKCamera2);
	p[0] = mKCamera2.at<double>(0, 0);
	p[1] = mKCamera2.at<double>(0, 1);
	p[2] = mKCamera2.at<double>(0, 2);
	p[3] = mKCamera2.at<double>(1, 0);
	p[4] = mKCamera2.at<double>(1, 1);
	p[5] = mKCamera2.at<double>(1, 2);
	p[6] = mKCamera2.at<double>(2, 0);
	p[7] = mKCamera2.at<double>(2, 1);
	p[8] = mKCamera2.at<double>(2, 2);
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
MutualCalibration::addFullIMURotationByQuaternion(double r0, double r1, double r2)
{
	assert(!mUseOnlyIMUGravity);
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
	mgsIMU.push_back(RIMU.col(2) * 1.0);
}

void
MutualCalibration::addIMUGravityVector(double g1, double g2, double g3)
{
	assert(mUseOnlyIMUGravity);
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
	RansacVanishingPoint vanishingPoint(inputImage); 
	vanishingPoint.findOrthogonalVanishingPts(); 
	vanishingPoint.getSketch().copyTo(outputImage); 
	if (vanishingPoint.orthogonalityDetected())
	{
		cv::Mat R = vanishingPoint.getRotation(); 
		std::vector<cv::Point2f> ovpts = vanishingPoint.selectOrthogonalVanishingPts();

		
		mRsCamera.push_back(R * 1.0);
		mfsCamera.push_back(vanishingPoint.getFocal());
		mVanishingPointImages++; 
		return true; 
	}
	else return false; 
}

void 
MutualCalibration::calibrateCamera()
{
	if (mImagePoints.empty())
	{
		float focal = 0;
		for (size_t i = 0; i < mfsCamera.size(); i++)
			focal += mfsCamera[i];
		focal / mfsCamera.size();
		cv::Mat cameraMatrix(3, 3, CV_64F);
		cameraMatrix.at<double>(0, 0) = focal;
		cameraMatrix.at<double>(0, 1) = 0;
		cameraMatrix.at<double>(0, 2) = (float)mImageSize.width / 2;
		cameraMatrix.at<double>(1, 0) = 0;
		cameraMatrix.at<double>(1, 1) = focal;
		cameraMatrix.at<double>(1, 2) = (float)mImageSize.height / 2;
		cameraMatrix.at<double>(2, 0) = 0;
		cameraMatrix.at<double>(2, 1) = 0;
		cameraMatrix.at<double>(2, 2) = 1;
		cameraMatrix.copyTo(mKCamera);
		return;
	}
	
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
		cameraMatrix.copyTo(mKCamera);
		for (size_t i = 0; i < rvecs.size(); i++)
		{
			cv::Mat RCamera; 
			cv::Rodrigues(rvecs[i], RCamera); 
			mRsCamera.push_back(RCamera * 1.0); 
		}
	}
}

cv::Mat
MutualCalibration::createAlignmentMatrix() const
{
	cv::Mat ideal(3, 3, CV_64F);
	ideal.at<double>(0, 0) = 0;
	ideal.at<double>(0, 1) = -1;
	ideal.at<double>(0, 2) = 0;
	ideal.at<double>(1, 0) = -1;
	ideal.at<double>(1, 1) = 0;
	ideal.at<double>(1, 2) = 0;
	ideal.at<double>(2, 0) = 0;
	ideal.at<double>(2, 1) = 0;
	ideal.at<double>(2, 2) = -1;
	return ideal * 1.0;
}

std::vector<cv::Mat>
MutualCalibration::findCameraGravity(const std::vector<cv::Mat> & cameraRotations, const std::vector<cv::Mat> & imuGravity) const
{
	cv::Mat ideal = createAlignmentMatrix();
	std::vector<cv::Mat> cameraGravity;
	for (size_t i = 0; i < cameraRotations.size(); i++)
	{
		cv::Mat R_c2i = ideal * cameraRotations[i];

		cv::Mat gCamera = R_c2i.col(0) * 1.0;

		if (cv::norm(imuGravity[i] + R_c2i.col(0)) < cv::norm(imuGravity[i] - gCamera))
			gCamera = -R_c2i.col(0) * 1.0;

		if (cv::norm(imuGravity[i] - R_c2i.col(1)) < cv::norm(imuGravity[i] - gCamera))
			gCamera = R_c2i.col(1) * 1.0;

		if (cv::norm(imuGravity[i] + R_c2i.col(1)) < cv::norm(imuGravity[i] - gCamera))
			gCamera = -R_c2i.col(1) * 1.0;

		if (cv::norm(imuGravity[i] - R_c2i.col(2)) < cv::norm(imuGravity[i] - gCamera))
			gCamera = R_c2i.col(2) * 1.0;

		if (cv::norm(imuGravity[i] + R_c2i.col(2)) < cv::norm(imuGravity[i] - gCamera))
			gCamera = -R_c2i.col(2) * 1.0;

		cameraGravity.push_back(ideal.t() * gCamera * 1.0);
	}
	return cameraGravity;

}

bool
MutualCalibration::lsMutualCalibrateWithHorizontalChessboard(const std::vector<cv::Mat> & cameraGravity, const std::vector<cv::Mat> & imuGravity, cv::Mat & ouputRotation) const
{

	cv::Mat S(3, cameraGravity.size(), CV_64F);
	cv::Mat T(3, cameraGravity.size(), CV_64F);
	for (size_t i = 0; i < cameraGravity.size(); i++)
	{
		S.col(i) = cameraGravity[i] * 1.0;
		T.col(i) = imuGravity[i] * 1.0;
	}

	// Kabsch algorihm from wikipedia
	cv::Mat A = T * S.t();
	cv::Mat D, U, Vt;
	cv::SVD::compute(A, D, U, Vt);
	cv::Mat R = U * Vt;
/*	__android_log_print(
			ANDROID_LOG_INFO, "R", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf\n",
			R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
			R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));*/
	R.copyTo(ouputRotation);
	if (D.at<double>(1) / D.at<double>(0) < 0.01) return false;
	else return true;
}

bool
MutualCalibration::ransacMutualCalibrateWithHorizontalChessboard(const std::vector<cv::Mat> & cameraGravity, const std::vector<cv::Mat> & imuGravity, cv::Mat & outputRotation) const
{
	// RANSAC
	size_t it = 0;
	size_t max_iter = 100;
	cv::Mat R_best;
	size_t max_inliers = 0;

	while (it < max_iter)
	{
		std::vector<size_t> perm = randPerm(mRsCamera.size());

		std::vector<cv::Mat> sampledCameraGravity, sampledImuGravity;
		for (size_t i = 0; i < 3; i++)
		{
			sampledCameraGravity.push_back(cameraGravity[perm[i]]);
			sampledImuGravity.push_back(imuGravity[perm[i]]);
		}

		cv::Mat R;
		bool wellPosed = lsMutualCalibrateWithHorizontalChessboard(sampledCameraGravity, sampledImuGravity, R);
		if (!wellPosed) continue;

		cv::Mat U(3, cameraGravity.size(), CV_64F);
		for (size_t i = 0; i < cameraGravity.size(); i++)
			U.col(i) = cameraGravity[i] * 1.0;

		cv::Mat V = R * U;
		size_t inliers = 0;
		for (size_t i = 0; i < V.cols; i++)
		{
			__android_log_print(
							ANDROID_LOG_INFO, "R_err", "%lf", cv::norm(V.col(i) - imuGravity[i]));
			if (cv::norm(V.col(i) - imuGravity[i]) < 0.05) inliers++;
		}
		if (inliers > max_inliers)
		{
			R.copyTo(R_best);
			max_inliers = inliers;
		}
		it++;

		__android_log_print(
				ANDROID_LOG_INFO, "R", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf\n %d %d",
				R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
				R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
				R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), inliers, it);

	}

	if (max_inliers > 0)
	{
		__android_log_print(
			ANDROID_LOG_INFO, "R_best", "%lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf\n %d",
			R_best.at<double>(0, 0), R_best.at<double>(0, 1), R_best.at<double>(0, 2),
			R_best.at<double>(1, 0), R_best.at<double>(1, 1), R_best.at<double>(1, 2),
			R_best.at<double>(2, 0), R_best.at<double>(2, 1), R_best.at<double>(2, 2), max_inliers);

		R_best.copyTo(outputRotation);
	}
	else
		__android_log_print(
			ANDROID_LOG_INFO, "R_best", "Weird");


	return max_inliers >= 3;

}

bool
MutualCalibration::mutualCalibrate()
{
	std::vector<cv::Mat> cameraGravity = findCameraGravity(mRsCamera, mgsIMU);
	bool wellPosed;
	if (mUseRANSAC)
	{
		wellPosed = ransacMutualCalibrateWithHorizontalChessboard(cameraGravity, mgsIMU, mCamera2IMU);
	}
	else
	{
		wellPosed = lsMutualCalibrateWithHorizontalChessboard(cameraGravity, mgsIMU, mCamera2IMU);
	}

	return wellPosed;
}
