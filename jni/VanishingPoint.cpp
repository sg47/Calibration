#include "VanishingPoint.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

Cas1DVanishingPoint::Cas1DVanishingPoint(const cv::Mat & image)
{
	if (image.channels() > 1)	
		cv::cvtColor(image, mImage, CV_BGR2GRAY); 
	else image.copyTo(image, mImage); 
	cv::cvtColor(mImage, mSketch, CV_GRAY2BGR); 
	
	mInteriorRadius = hypot(mImage.cols / 2, mImage.rows / 2); 

	detectLines(); 
	for (size_t i = 0; i < mLines.size(); i++)
	{
		mLines[i][0] -= mImage.cols / 2; 
		mLines[i][1] -= mImage.rows / 2; 
		mLines[i][2] -= mImage.cols / 2; 
		mLines[i][3] -= mImage.rows / 2; 
	}
	showLines(mLines); 
}

bool 
Cas1DVanishingPoint::focalAvailable() const
{
	return mMessage == THREE_DETECTED_WITH_ONE_INFINITE ||
		   mMessage == THREE_DETECTED_WITH_REFINERY ||
		   mMessage == THREE_DETECTED_WITHOUT_REFINERY; 
}

bool 
Cas1DVanishingPoint::threeDetected() const
{
	return mMessage == THREE_DETECTED_WITH_ONE_INFINITE ||
		   mMessage == THREE_DETECTED_WITH_TWO_INFINITE ||
		   mMessage == THREE_DETECTED_WITH_REFINERY ||
		   mMessage == THREE_DETECTED_WITHOUT_REFINERY; 
}

float
Cas1DVanishingPoint::getFocal() const
{
	return mFocal; 
}

std::vector<cv::Point2f>
Cas1DVanishingPoint::getVanishingPts() const
{
	return mVanishingPts; 
}

cv::Mat 
Cas1DVanishingPoint::getSketch() const
{
	mSketch.copyTo(m); 
	for (size_t i = 0; i < vanishingPts.size(); i++)
	{
		cv::Point2f v; 
		v = vanishingPts[i];  
		cv::line(m, cv::Point(m.cols/2, m.rows/2), 
				cv::Point(v.x + m.cols/2, v.y + m.rows/2), cv::Scalar(255, 255, 0), 3, 8);
	}
	return m * 1.0; 
}


void 
Cas1DVanishingPoint::showLines(const std::vector<cv::Vec4i> & lines) const
{
		cv::namedWindow("lines", CV_WINDOW_KEEPRATIO); 
		cv::Mat m; 
		mSketch.copyTo(m); 
		for (size_t i = 0; i < lines.size(); i++)
			cv::line(m, cv::Point(lines[i][0] + m.cols/2, lines[i][1] + m.rows/2), 
					cv::Point(lines[i][2] + m.cols/2, lines[i][3] + m.rows/2), cv::Scalar(255, 255, 0), 1, 8);
		cv::imshow("lines", m); 
		cv::waitKey(); 
}

void
Cas1DVanishingPoint::showVanishing(const std::vector<cv::Point2f> & vanishingPts) const
{
		cv::namedWindow("v", CV_WINDOW_KEEPRATIO); 
		cv::Mat m; 
		mSketch.copyTo(m); 
		for (size_t i = 0; i < vanishingPts.size(); i++)
		{
			cv::Point2f v; 
			v = vanishingPts[i];  
			cv::line(m, cv::Point(m.cols/2, m.rows/2), 
					cv::Point(v.x + m.cols/2, v.y + m.rows/2), cv::Scalar(255, 255, 0), 3, 8);
		}
		cv::imshow("v", m); 
		cv::waitKey(); 
}

double 
Cas1DVanishingPoint::mod(double x, double d) const
{
	int i = x / d; 	
	while (i * d > x) i--; 
	return x - i*d; 
}

void
Cas1DVanishingPoint::detectLines()
{
	float min_length = 30.0f / 800 * hypot(mImage.cols, mImage.rows); 
	int min_vote = 80.0 / 800 * hypot(mImage.cols, mImage.rows); 
	int blurRadius = 1.5 / 800 * hypot(mImage.cols, mImage.rows); 
	std::cout << blurRadius << std::endl; 
	cv::Mat blured; 
	cv::GaussianBlur(mImage, blured, cv::Size(2 * blurRadius + 1, 2 * blurRadius + 1), blurRadius); 
	cv::Mat edge; 
	cv::Canny(blured, edge, 50, 100, 3); 
	cv::namedWindow("edge"); 
	cv::imshow("edge", blured); 
	cv::waitKey(); 
	cv::HoughLinesP(edge, mLines, 1, CV_PI/180, min_vote, min_length, 10);
	std::cout << mLines.size() << " lines detected. " << std::endl; 
/*	for (size_t i = 0; i < mLines.size(); i++)
	{
		cv::line(mSketch, cv::Point2f(mLines[i][0], mLines[i][1]), cv::Point2f(mLines[i][2], mLines[i][3]), cv::Scalar(0, 0, 244), 3, 8); 
	}
	cv::namedWindow("line"); 
	cv::imshow("line", mSketch); 
	cv::waitKey(); */
}

cv::Point2f
Cas1DVanishingPoint::convergeLines(const std::vector<cv::Vec4i> & lines) const
{
	cv::Mat A = cv::Mat::zeros(2, 2, CV_64F); 
	cv::Mat B = cv::Mat::zeros(2, 1, CV_64F); 
	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::Mat n(2, 1, CV_64F); 
		n.at<double>(0) = lines[i][3] - lines[i][1]; 
		n.at<double>(1) = lines[i][0] - lines[i][2]; 
		cv::normalize(n, n); 
		cv::Mat p(2, 1, CV_64F); 
		p.at<double>(0) = lines[i][0]; 
		p.at<double>(1) = lines[i][1]; 
		A += n * n.t(); 
		B += n * n.t() * p; 
	}
	cv::Mat x = A.inv() * B; 
	return cv::Point2f(x.at<double>(0), x.at<double>(1)); 
	 
//	std::cout << cv::Mat(vec) << linesSupport(vec, mLines).size() << std::endl; 
//	showLines(lines); 
}

std::vector<cv::Point2f>
Cas1DVanishingPoint::intersectLines(const std::vector<cv::Vec4i> & lines) const
{
	std::vector<cv::Point2f> intersections; 
	for (size_t i = 0; i < lines.size(); i++)
		for (size_t j = i + 1; j < lines.size(); j++)
		{
			std::vector<cv::Vec4i> two_lines; 
			two_lines.push_back(lines[i]); 
			two_lines.push_back(lines[j]); 
			intersections.push_back(convergeLines(two_lines)); 
		}
	return intersections; 
}

float 
Cas1DVanishingPoint::distance(float theta, float rho, cv::Vec4f line) const
{
//	std::cout << cv::Mat(line)  << std::endl; 
	float mid_x = 0.5f * (line[0] + line[2]); 
	float mid_y = 0.5f * (line[1] + line[3]); 

	float vp_x = rho * cos(theta); 
	float vp_y = rho * sin(theta); 

	float v_x = vp_x - mid_x; 
	float v_y = vp_y - mid_y; 
	float r = hypot(v_x, v_y); 
	v_x /= r; 
	v_y /= r; 

	float l_x = line[0] - (line[0] + line[2]) / 2.0f; 
	float l_y = line[1] - (line[1] + line[3]) / 2.0f; 

	float proj = v_x * l_x + v_y * l_y; 
	std::cout << sqrt(l_x * l_x + l_y * l_y - proj * proj) << std::endl; 
	return sqrt(l_x * l_x + l_y * l_y - proj * proj); 
}

std::vector<cv::Vec4i>
Cas1DVanishingPoint::linesSupport(float theta, float rho, const std::vector<cv::Vec4i> & lines) const
{
	std::vector<cv::Vec4i> support; 
	for (size_t i = 0; i < lines.size(); i++)
		if (distance(theta, rho, lines[i]) < 2.0) support.push_back(lines[i]); 
	return support; 
}

void 
Cas1DVanishingPoint::removeVanishingLines(float theta, float rho, std::vector<cv::Vec4i> & lines) const
{
	std::vector<cv::Vec4i> residual_lines; 
	for (size_t i = 0; i < lines.size(); i++)
		if (distance(theta, rho, lines[i]) > 2.0) residual_lines.push_back(lines[i]); 
	lines = residual_lines; 
}

float 
Cas1DVanishingPoint::findTheta(const std::vector<float> & thetas, std::vector<size_t> & supportIndex) const
{
	float theta_threshold = 5.0f / 360 * CV_PI; 

	int channel = 0; 	
	int histSize = thetas.size() < 360 ? 90 : 360; 
	float histRange[] = {-CV_PI, CV_PI}; 
	const float *histRanges[] = {histRange}; 
	cv::Mat data(thetas); 
	cv::Mat hist; 
	cv::calcHist(&data, 1, &channel, cv::Mat(), hist, 1, &histSize, histRanges, true, false); 

	cv::Point max_loc; 
	cv::minMaxLoc(hist, NULL, NULL, NULL, &max_loc); 
//	std::cout << hist << std::endl; 
	float theta = histRange[0] + max_loc.y * (histRange[1] - histRange[0]) / histSize; 

	for (size_t i = 0; i < thetas.size(); i++)
	{
		if (fabs(thetas[i] - theta) < theta_threshold |
			2.0f * CV_PI - fabs(thetas[i] - theta) < theta_threshold)
		{
			supportIndex.push_back(i); 
		}
	}

	return theta; 
}

float 
Cas1DVanishingPoint::findRho(const std::vector<float> & rhos, const std::vector<size_t> & supportIndex) const
{
	float z = std::min(mImage.cols, mImage.rows); 
	std::vector<float> phis; 
	for (size_t i = 0; i < supportIndex.size(); i++)
		phis.push_back(atan2(rhos[supportIndex[i]], z)); 

	int channel = 0; 	
	int histSize = rhos.size() < 90 ? 23 : 90;  
	float histRange[] = {0.0f, 0.5f * CV_PI}; 
	const float *histRanges[] = {histRange}; 
	cv::Mat data(phis); 
	cv::Mat hist; 
	cv::calcHist(&data, 1, &channel, cv::Mat(), hist, 1, &histSize, histRanges, true, false); 

//	std::cout << hist << std::endl; 
	cv::Point max_loc; 
	cv::minMaxLoc(hist, NULL, NULL, NULL, &max_loc); 
	float phi = histRange[0] + max_loc.y * (histRange[1] - histRange[0]) / histSize; 
	float rho = z * tan(phi); 
	return rho; 
}

void 
Cas1DVanishingPoint::findExteriorVanishingPt(const std::vector<cv::Point2f> & pts, float & extVPTheta, float & extVPRho) const
{
	std::vector<float> extThetas, extRhos; 
	for (size_t i = 0; i < pts.size(); i++)
	{
		float rho = hypot(pts[i].x, pts[i].y); 
		if (rho < mInteriorRadius) continue; 
		extThetas.push_back(atan2(pts[i].y, pts[i].x)); 
		extRhos.push_back(rho); 
	}
	std::vector<size_t> idx; 
	extVPTheta = findTheta(extThetas, idx); 
	extVPRho = findRho(extRhos, idx); 
}

void 
Cas1DVanishingPoint::findInteriorVanishingPt(const std::vector<cv::Point2f> & pts, float & intVPTheta, float & intVPRho) const
{
	float binSize = 10.0f; 
	float binInf = -2.0 * mInteriorRadius; 
	float binSup = 2.0 * mInteriorRadius; 
	size_t histSize = (size_t)((binSup - binInf) / binSize) + 1;  
	std::vector<float> histRange; 
	for (size_t i = 0; i <= histSize; i++)
		histRange.push_back(-2.0f * mInteriorRadius + i * binSize); 

	std::vector<std::vector<cv::Point2f> > xBins(histSize), yBins(histSize); 
	for (size_t i = 0; i < pts.size(); i++)
	{
		if (hypot(pts[i].x, pts[i].y) > 2.0 * mInteriorRadius) continue; 
		size_t xIndex = (size_t)((pts[i].x - binInf) / binSize); 
		size_t yIndex = (size_t)((pts[i].y - binInf) / binSize); 
		xBins[xIndex].push_back(pts[i]); 
		yBins[yIndex].push_back(pts[i]); 
	}

	size_t histThresh = std::max(pts.size() / histSize, (size_t)6); 
	std::vector<float> xWeights(histSize), yWeights(histSize); 
	for (size_t i = 0; i < histSize; i++)
	{
		if (xBins[i].size() > histThresh)
		{
			float sum = 0, sqrSum = 0; 		
			for (size_t j = 0; j < xBins[i].size(); j++)
			{
				sum += xBins[i][j].y; 
				sqrSum += xBins[i][j].y * xBins[i][j].y; 
			}
			float mean = sum / xBins[i].size(); 
			float var = sqrSum / xBins[i].size() - mean * mean; 
			xWeights[i] = (float)xBins[i].size() / var; 
		}
		else
			xWeights[i] = 0.0f; 

		if (yBins[i].size() > histThresh)
		{
			float sum = 0, sqrSum = 0; 		
			for (size_t j = 0; j < yBins[i].size(); j++)
			{
				sum += yBins[i][j].x; 
				sqrSum += yBins[i][j].x * yBins[i][j].x; 
			}
			float mean = sum / yBins[i].size(); 
			float var = sqrSum / yBins[i].size() - mean * mean; 
			yWeights[i] = (float)yBins[i].size() / var; 
		}
		else
			yWeights[i] = 0.0f; 

	}
	size_t xIndex = 0, yIndex = 0; 
	for (size_t i = 0; i < histSize; i++)
	{
		xIndex = (xWeights[xIndex] > xWeights[i]) ? xIndex : i; 
		yIndex = (yWeights[yIndex] > yWeights[i]) ? yIndex : i; 
	}
	cv::Point2f intVP; 
	intVP.x = binInf + xIndex * binSize; 
	intVP.y = binInf + yIndex * binSize; 
	intVPTheta = atan2(intVP.y, intVP.x); 
	intVPRho = hypot(intVP.x, intVP.y); 
}

void 
Cas1DVanishingPoint::find3rdDegenVanishingPt(const std::vector<cv::Point2f> & pts, 
											 const std::vector<float> & vanishingThetas, 
											 const std::vector<float> & vanishingRhos,
											 float & theta3rd, float & rho3rd) const
{
	float theta_threshold = 10.0f/180 * CV_PI; 
	std::vector<float> rhos; 
	std::vector<size_t> index; 
	size_t support = 0; 
	for (size_t vi = 0; vi < 2; vi++)
	{
		float theta_cand = vanishingThetas[vi] + CV_PI; 
		if (theta_cand > CV_PI) theta_cand -= 2.0f * CV_PI; 
		for (size_t i = 0; i < pts.size(); i++)
		{
			float theta = atan2(pts[i].y, pts[i].x); 
			std::cout << theta << " ";  
			if (fabs(theta_cand - theta) < theta_threshold |
				2.0f * CV_PI - fabs(theta_cand - theta) < theta_threshold)
			{
				index.push_back(rhos.size()); 
				rhos.push_back(hypot(pts[i].x, pts[i].y)); 
			}
		}
		float rho_cand = findRho(rhos, index); 	
		size_t s = linesSupport(theta_cand, rho_cand, mLines).size(); 
		if (s > support) 
		{
			theta3rd = theta_cand; 
			rho3rd = rho_cand; 
			support = s; 
		}
	}
}

void 
Cas1DVanishingPoint::findOrthogonalVanishingPts()
{
	size_t min_support = 4; 
	float min_rho = 10; 

	std::vector<cv::Vec4i> remaining_lines(mLines); 
	bool interiorUsed = false; 
	
	std::vector<cv::Point2f> vanishingPts; 
	std::vector<float> vanishingThetas, vanishingRhos; 
	std::vector<size_t> vanishingSupports; 
	for (size_t i = 0; i < 2; i++)
	{

//		showLines(remaining_lines); 
		std::vector<cv::Point2f> pts = intersectLines(remaining_lines); 
		std::vector<cv::Point2f> intPts, extPts; 
		std::vector<float> intThetas, intRhos, extThetas, extRhos; 
		for (size_t j = 0; j < pts.size(); j++)
		{
			float rho = hypot(pts[j].x, pts[j].y); 
			if (rho < 2.0f * mInteriorRadius) intPts.push_back(pts[j]); 
			if (rho > mInteriorRadius) extPts.push_back(pts[j]); 
		}

		cv::Point2f intVP; 
		float intVPTheta, intVPRho; 
		size_t intSupport = 0; 
		if (!interiorUsed && intPts.size() > 0)
		{
			findInteriorVanishingPt(intPts, intVPTheta, intVPRho); 
			intVP.x = intVPRho * cos(intVPTheta); 
			intVP.y = intVPRho * sin(intVPTheta); 
			intSupport = linesSupport(intVPTheta, intVPRho, remaining_lines).size(); 
		}

		if (extPts.size() == 0) continue; 

		float extVPTheta, extVPRho; 
		findExteriorVanishingPt(extPts, extVPTheta, extVPRho); 
		cv::Point2f extVP; 
	   	extVP.x = extVPRho * cos(extVPTheta); 
	   	extVP.y = extVPRho * sin(extVPTheta); 
		size_t extSupport = linesSupport(extVPTheta, extVPRho, remaining_lines).size(); 

		if (extSupport < intSupport)
		{
			vanishingPts.push_back(intVP); 
			vanishingThetas.push_back(intVPTheta); 
			vanishingRhos.push_back(intVPRho); 
			vanishingSupports.push_back(intSupport); 
			interiorUsed = true; 
		}
		else 
		{
			vanishingPts.push_back(extVP); 
			vanishingThetas.push_back(extVPTheta); 
			vanishingRhos.push_back(extVPRho); 
			vanishingSupports.push_back(extSupport); 
		}
		
		removeVanishingLines(vanishingThetas.back(), vanishingRhos.back(), remaining_lines); 
	}

//	showLines(remaining_lines); 
	if (vanishingPts.size() > 1 && vanishingSupports[0] < vanishingSupports[1]) 
	{
		std::swap(vanishingPts[0], vanishingPts[1]); 
		std::swap(vanishingThetas[0], vanishingThetas[1]); 
		std::swap(vanishingRhos[0], vanishingRhos[1]); 
		std::swap(vanishingSupports[0], vanishingSupports[1]);
	}
	if (vanishingPts.size() == 0 || vanishingSupports[0] < min_support)
	{
		mMessage = NOTHING_DETECTED; 
		return; 
	}
	else if (vanishingPts.size() == 1 || vanishingSupports[1] < min_support)
	{
		mVanishingPts.push_back(vanishingPts[0]); 
		mMessage = ONE_DETECTED; 
		return; 
	}

	float angle = mod(vanishingThetas[0] - vanishingThetas[1], 2.0f * CV_PI); 
	if (angle > CV_PI) angle = 2.0f * CV_PI - angle; 
	float threshold = CV_PI / 180.0f; 
	// 
	// non-orthogonal
	if (angle < CV_PI / 2.0f - 10.0f * threshold) mMessage = TWO_NON_ORTHOGONAL_DETECTED; 	
	// 
	// degen
	else if (angle >= CV_PI / 2.0f - 10.0f * threshold && angle < CV_PI / 2.0f + 3.0 * threshold)
	{
		if (mod(vanishingThetas[1] - vanishingThetas[0], 2.0f * CV_PI) < CV_PI)
			vanishingThetas[1] = vanishingThetas[0] + CV_PI / 2.0f; 
		else vanishingThetas[1] = vanishingThetas[0] - CV_PI / 2.0f; 
		float theta3rd, rho3rd; 
		find3rdDegenVanishingPt(intersectLines(remaining_lines), vanishingThetas, vanishingRhos, theta3rd, rho3rd); 
		if (linesSupport(theta3rd, rho3rd, remaining_lines).size() > min_support)
		{
			vanishingThetas.push_back(theta3rd); 
			vanishingRhos.push_back(rho3rd); 
			vanishingPts.push_back(cv::Point2f(rho3rd * cos(theta3rd), rho3rd * sin(theta3rd))); 
			if (*std::min_element(vanishingRhos.begin(), vanishingRhos.end()) < min_rho)
			{
				mMessage = THREE_DETECTED_WITH_TWO_INFINITE; 
			}
			else 
			{
				std::sort(vanishingRhos.begin(), vanishingRhos.end()); 
				mFocal = sqrt(vanishingRhos[0] * vanishingRhos[1]); 
				mMessage = THREE_DETECTED_WITH_ONE_INFINITE; 
			}
		}
		else
		{
			mMessage = TWO_DETECTED_WITH_ONE_INFINITE; 
		}
	}
	// Normal
	else
	{
		float focal = sqrt(-vanishingRhos[0] * vanishingRhos[1] * cos(vanishingThetas[0] - vanishingThetas[1])); 
		cv::Mat vp(2, 2, CV_64F); 
		vp.at<double>(0, 0) = vanishingPts[0].x; 
		vp.at<double>(0, 1) = vanishingPts[0].y; 
		vp.at<double>(1, 0) = vanishingPts[1].x; 
		vp.at<double>(1, 1) = vanishingPts[1].y; 
		cv::Mat guess = vp.inv() * cv::Mat(2, 1, CV_64F, -focal * focal); 

		float intTheta, intRho, extTheta, extRho; 
		size_t intSupport = 0, extSupport = 0;  
		if (cv::norm(guess) < mInteriorRadius * 2 && !interiorUsed)
		{
			findInteriorVanishingPt(intersectLines(remaining_lines), intTheta, intRho); 
			intSupport = linesSupport(intTheta, intRho, remaining_lines).size(); 
		}
		else 
		{
			findExteriorVanishingPt(intersectLines(remaining_lines), extTheta, extRho); 
			extSupport = linesSupport(extTheta, extRho, remaining_lines).size(); 
		}

		if (std::max(intSupport, extSupport) > min_support)
		{
			float theta = intSupport > extSupport ? intTheta : extTheta; 
			float rho = intSupport > extSupport ? intRho : extRho; 
			vanishingThetas.push_back(theta); 
			vanishingRhos.push_back(rho); 
			bool success = refineVanishingPts(intersectLines(mLines), vanishingThetas, vanishingRhos, mFocal); 
			if (success) 
			{
				vanishingPts.clear(); 
				for (size_t i = 0; i < 3; i++)
					vanishingPts.push_back(cv::Point2f(vanishingRhos[i] * cos(vanishingThetas[i]), vanishingRhos[i] * sin(vanishingThetas[i]))); 
				mMessage = THREE_DETECTED_WITH_REFINERY; 
			}
			else 
			{
				vanishingPts.push_back(cv::Point2f(guess.at<double>(0), guess.at<double>(1))); 
				mFocal = focal; 
				mMessage = THREE_DETECTED_WITHOUT_REFINERY; 
			}
		}
		else
		{
			vanishingPts.push_back(cv::Point2f(guess.at<double>(0), guess.at<double>(1))); 
			mFocal = focal; 
			mMessage = THREE_DETECTED_WITHOUT_REFINERY; 
		}
	}
	mVanishingPts = vanishingPts; 
/*	std::cout << cv::Mat(vanishingPts) << std::endl; 
	showVanishing(vanishingPts); */
}

bool
Cas1DVanishingPoint::refineVanishingPts(const std::vector<cv::Point2f> & pts, const std::vector<float> & vanishingThetas, std::vector<float> & vanishingRhos, float & focal) const
{
	float theta_threshold = 2.5f /180 * CV_PI; 
	float z = std::min(mImage.cols, mImage.rows); 

	if (cos(vanishingThetas[0] - vanishingThetas[1]) > 0 || 
		cos(vanishingThetas[1] - vanishingThetas[2]) > 0 ||
		cos(vanishingThetas[2] - vanishingThetas[0]) > 0)
		return false; 

	float c1 = cos(vanishingThetas[0]); 
	float c2 = cos(vanishingThetas[1]); 
	float c3 = cos(vanishingThetas[2]); 

	float s1 = sin(vanishingThetas[0]); 
	float s2 = sin(vanishingThetas[1]); 
	float s3 = sin(vanishingThetas[2]); 

	float d0 = sqrt(fabs((-c2*c3-s2*s3)/(-c1*c2-s1*s2)/(-c1*c3-s1*s3)));
	float d1 = sqrt(fabs((-c1*c3-s1*s3)/(-c2*c1-s2*s1)/(-c2*c3-s2*s3)));
	float d2 = sqrt(fabs((-c1*c2-s1*s2)/(-c1*c3-s1*s3)/(-c2*c3-s2*s3)));
	
	std::vector<float> focals; 
	for (size_t i = 0; i < pts.size(); i++)
	{
		float rho = hypot(pts[i].x, pts[i].y); 
		if (rho < mInteriorRadius) continue; 

		float theta = atan2(pts[i].y, pts[i].x); 
		if (fabs(theta - vanishingThetas[0]) < theta_threshold || 
			fabs(theta - vanishingThetas[0]) > 2.0 * CV_PI - theta_threshold)
			focals.push_back(rho / d0); 
		
		if (fabs(theta - vanishingThetas[1]) < theta_threshold || 
			fabs(theta - vanishingThetas[1]) > 2.0 * CV_PI - theta_threshold)
			focals.push_back(rho / d1); 
		
		if (fabs(theta - vanishingThetas[2]) < theta_threshold || 
			fabs(theta - vanishingThetas[2]) > 2.0 * CV_PI - theta_threshold)
			focals.push_back(rho / d2); 
	}

	std::vector<float> phis; 
	for (size_t i = 0; i < focals.size(); i++)
		phis.push_back(atan2(focals[i], z)); 

	int channel = 0; 	
	int histSize = phis.size() < 90 ? 45 : 90;  
	float histRange[] = {0.0f, 0.5f * CV_PI}; 
	const float *histRanges[] = {histRange}; 
	cv::Mat data(phis); 
	cv::Mat hist; 
	cv::calcHist(&data, 1, &channel, cv::Mat(), hist, 1, &histSize, histRanges, true, false); 

//	std::cout << hist << std::endl; 
	cv::Point max_loc; 
	cv::minMaxLoc(hist, NULL, NULL, NULL, &max_loc); 
	float phi = histRange[0] + max_loc.y * (histRange[1] - histRange[0]) / histSize; 

	focal = z * tan(phi); 
	vanishingRhos[0] = focal * d0; 
	vanishingRhos[1] = focal * d1; 
	vanishingRhos[2] = focal * d2; 
	return true; 

}
