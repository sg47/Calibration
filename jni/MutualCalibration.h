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
	std::vector<cv::Mat> mRsCamera, mtsCamera; 
	std::vector<cv::Mat> mRsIMU; 

	cv::Mat mCamera2IMU; 

	bool mUseOpenCVCorner; 
	bool mUseOpenCVCalibration; 

protected:

public:
	std::vector<double> rotationTest; 
	double * pRotationTest; 

	MutualCalibration(size_t heightImage, size_t widthImage, size_t heightBoard, size_t widthBoard, 
					  bool useOpenCVCorner = false, bool useOpenCVCalibration = true); 
	bool tryAddingChessboardImage(cv::Mat & inputImage, cv::Mat & outputImage); 
	void addIMUData(double r0, double r1, double r2); 
	size_t getNumberOfImages() const; 
	std::vector<size_t> randPerm(size_t n) const; 
	void calibrateCamera(); 
	void mutualCalibrate(); 

}; 
