#include "VanishingPoint.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
int main()
{
	cv::Mat image = cv::imread("b.jpg"); 
//	cv::Mat image = cv::imread("cb.png");
	Cas1DVanishingPoint vp(image); 
	vp.findOrthogonalVanishingPts(); 
	std::cout << cv::Mat(vp.getVanishingPts()) << std::endl; 
	vp.showVanishing(vp.getVanishingPts()); 
	return 0; 
}
