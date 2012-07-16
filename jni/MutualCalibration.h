#include <iostream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <boost/math/quaternion.hpp>

#include "CataCameraParameters.h"
#include "Chessboard.h"

class MutualCalibration
{
	cv::Size mImageSize; 
	cv::Size mBoardSize; 
	float mSquareSize; 
//	std::vector<cv::Mat> mChessboardImages; 
	std::vector<std::vector<cv::Point2f> > mImagePoints; 
	
	vcharge::CataCameraParameters mCameraParam; 
	std::vector<cv::Mat> mRsCamera, mtsCamera, mgsCamera; 
	std::vector<float> mfsCamera; 
	std::vector<cv::Mat> mRsIMU, mgsIMU; 

	cv::Mat mCamera2IMU; 

	bool mUseOpenCVCorner; 
	bool mUseOpenCVCalibration; 
	bool mUseOnlyIMUGravity; 
	bool mUseChessboardHorizontal; 
	bool mUseRANSAC; 

	bool mChessboardMeasured; 
	double mZChessboardNormal; 

	size_t mChessboardImages, mVanishingPointImages; 

protected:

public:

	MutualCalibration(size_t heightImage, size_t widthImage, size_t heightBoard, size_t widthBoard, 
					  bool useOpenCVCorner = false, bool useOpenCVCalibration = true, 
					  bool useOnlyIMUGravity = false, bool useChessboardHorizontal = true, 
					  bool useRANSAC = false); 
	bool tryAddingChessboardImage(cv::Mat & inputImage, cv::Mat & outputImage); 
	bool tryAddingVanishingPointImage(cv::Mat & inputImage, cv::Mat & outputImage); 
	void addFullIMURotationByQuaternion(double r0, double r1, double r2); 
	void addIMUGravityVector(double g1, double g2, double g3); 
	size_t getNumberOfImages() const; 
	void getRotationMatrix(double p[]) const; 
	std::vector<size_t> randPerm(size_t n) const; 
	void calibrateCamera(); 
	void ransacMutualCalibrateWithHorizontalChessboard(); 
	void lsMutualCalibrateWithHorizontalChessboard(); 
	void mutualCalibrate(); 

}; 
