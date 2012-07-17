#include "RansacRansacVanishingPoint.h"

#include <cmath>
#include <utility>
#include <algorithm>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

RansacVanishingPoint::RansacVanishingPoint(const cv::Mat & image, cv::Point2f pp, float focal)
{
	if (pp.x > 0 && pp.y > 0)
	{
		mFixPriciplePt = true; 
		mPrinciplePt = pp; 
	}
	else mFixPriciplePt = false; 

	if (focal > 0)
	{
		mFixFocal = true; 
		mFocal = focal; 
	}
	else mFixFocal = false; 

	if (image.channels() > 1)	
		cv::cvtColor(image, mImage, CV_BGR2GRAY); 
	else image.copyTo(image, mImage); 
	cv::cvtColor(mImage, mSketch, CV_GRAY2BGR); 
	
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

void 
RansacVanishingPoint::findVanishingPts()
{
	size_t min_support = 5; 
	size_t max_count = 5; 
	std::vector<cv::Vec4i> lines(mLines); 
	mVanishingPts.clear(); 
	for (size_t i = 0; i < max_count; i++)
	{
		cv::Point2f vpt = ransac2Lines(lines); 
		std::vector<cv::Vec4i> support = linesSupport(vpt, lines); 

//		std::cout << "-----------" << cv::Mat(vpt) << std::endl; 
		showLines(support); 
		mVanishingPts.push_back(vpt); 
		if (support.size() >= min_support) 
		{
			removeVanishingLines(vpt, lines); 
		}
	}
}

void
RansacVanishingPoint::findOrthogonalVanishingPts()
{
	findVanishingPts(); 
	mOrthogonalVanishingPts = selectOrthogonalRansacVanishingPoints(); 
	if (!mOrthogonalVanishingPts.empty())
	{
		float err; 
		selectOrthogonalRansacVanishingPointsHelper(mOrthogonalVanishingPts, mFocal, err); 
	}
}

bool
RansacVanishingPoint::orthogonalityDetected() const
{
	return !mOrthogonalVanishingPts.empty(); 
}


cv::Mat 
Cas1DVanishingPoint::getSketch() const
{
	cv::Mat m; 
	mSketch.copyTo(m); 
	if (orthogonalityDetected())
	{
		for (size_t i = 0; i < mOrthogonalVanishingPts.size(); i++)
		{
			cv::Point2f v; 
			v = mOrthogonalVanishingPts[i];  
			cv::line(m, cv::Point(m.cols/2, m.rows/2), 
					cv::Point(v.x + m.cols/2, v.y + m.rows/2), cv::Scalar(255, 255, 0), 3, 8);
		}
	}
	return m * 1.0; 
}

cv::Mat 
RansacVanishingPoint::getRotation() const
{
	if (!orthogonalityDetected()) return cv::Mat(); 

	cv::Vec3f x, y, z; 
	x[0] = mOrthogonalVanishingPts[0].x; 
	x[1] = mOrthogonalVanishingPts[0].y; 
	x[2] = mFocal; 
	cv::normalize(x, x); 
	y[0] = mOrthogonalVanishingPts[1].x; 
	y[1] = mOrthogonalVanishingPts[1].y; 
	y[2] = mFocal; 
	cv::normalize(y, y); 
	z[0] = mOrthogonalVanishingPts[2].x; 
	z[1] = mOrthogonalVanishingPts[2].y; 
	z[2] = mFocal; 
	cv::normalize(z, z); 

	if (fabs(x[0]) < fabs(z[0])) std::swap(x, z); 
	if (fabs(y[0]) < fabs(z[0])) std::swap(y, z); 
	if (z[1] < 0.0) z = -z;

	if (x.cross(y).dot(z) < 0) std::swap(x, y); 

	cv::Mat R(3, 3, CV_64F); 
	cv::Mat(x * 1.0).copyTo(R.col(0)); 
	cv::Mat(y * 1.0).copyTo(R.col(1)); 
	cv::Mat(z * 1.0).copyTo(R.col(2)); 
	return R * 1.0; 
	
}

std::vector<cv::Point2f>
RansacVanishingPoint::selectOrthogonalVanishingPts() const
{
	std::vector<cv::Point2f> triplet; 
	float min_err = std::numeric_limits<float>::max(); 
	for (size_t i = 0; i < mVanishingPts.size(); i++)
		for (size_t j = i + 1; j < mVanishingPts.size(); j++)
			for (size_t k = j + 1; k < mVanishingPts.size(); k++)
			{
				std::vector<cv::Point2f> vpts; 
				vpts.push_back(mVanishingPts[i]); 
				vpts.push_back(mVanishingPts[j]); 
				vpts.push_back(mVanishingPts[k]); 
				float focal, error; 
				selectOrthogonalRansacVanishingPointsHelper(vpts, focal, error); 	
				if (error < min_err)
				{
					triplet = vpts; 
					min_err = error; 
				}
			}
	if (min_err < 0.1) return triplet; 
	else return std::vector<cv::Point2f>(); 
}

void
RansacVanishingPoint::selectOrthogonalVanishingPtsHelper(const std::vector<cv::Point2f> & vanishingPts, float & focal, float & err) const
{
	float len0 = hypot(vanishingPts[0].x, vanishingPts[0].y); 
	float len1 = hypot(vanishingPts[1].x, vanishingPts[1].y); 
	float len2 = hypot(vanishingPts[2].x, vanishingPts[2].y); 
	cv::Point2f vp0, vp1; 
	if (len0 < len2 && len1 < len2)
	{
		vp0 = vanishingPts[0]; 
		vp1 = vanishingPts[1]; 
	}
	else if (len0 < len1 && len2 < len1)
	{
		vp0 = vanishingPts[0]; 
		vp1 = vanishingPts[2]; 
	}
	else
	{
		vp0 = vanishingPts[1]; 
		vp1 = vanishingPts[2]; 
	}

	float sqr_focal = -vp0.x * vp1.x - vp0.y * vp1.y; 
	if (sqr_focal < 0) 
	{
		focal = -1; 
		err = std::numeric_limits<float>::max(); 
	}
	else
	{
		focal = sqrt(sqr_focal); 
		cv::Mat A(3, 3, CV_64F); 
		for (size_t i = 0; i < 3; i++)
		{
			float len = hypot(focal, hypot(vanishingPts[i].x, vanishingPts[i].y)); 
			A.at<double>(0, i) = vanishingPts[i].x / len; 
			A.at<double>(1, i) = vanishingPts[i].y / len; 
			A.at<double>(2, i) = focal / len; 
		}
		cv::Mat U, D, Vt; 
		cv::SVD::compute(A, D, U, Vt); 
		err = cv::norm(U * Vt - A); 
	}
	
}

std::vector<cv::Point2f>
RansacVanishingPoint::getVanishingPts() const
{
	return mVanishingPts; 
}

void 
RansacVanishingPoint::showLines(const std::vector<cv::Vec4i> & lines) const
{

		cv::namedWindow("lines"); 
		cv::Mat m; 
		mSketch.copyTo(m); 
		for (size_t i = 0; i < lines.size(); i++)
			cv::line(m, cv::Point(lines[i][0], lines[i][1]) + cv::Point(m.cols/2, m.rows/2),cv::Point(lines[i][2], lines[i][3]) + cv::Point(m.cols/2, m.rows/2), cv::Scalar(255, 255, 0), 3, 8);
		cv::imshow("lines", m); 
		cv::waitKey(); 
}

void
RansacVanishingPoint::showVanishing(const std::vector<cv::Point2f> & vanishingPts) const
{
		cv::namedWindow("v", CV_WINDOW_KEEPRATIO); 
		cv::Mat m; 
		mSketch.copyTo(m); 
		for (size_t i = 0; i < vanishingPts.size(); i++)
		{
			cv::Point2f v; 
			v = vanishingPts[i];  
			cv::line(m, cv::Point(m.cols/2, m.rows/2), 
					cv::Point(v.x, v.y) + cv::Point(m.cols/2, m.rows/2), cv::Scalar(255, 255, 0), 3, 8);
		}
		cv::imshow("v", m); 
		cv::waitKey(); 
}
void
RansacVanishingPoint::detectLines()
{
	int blurRadius = 1.5 / 800 * hypot(mImage.cols, mImage.rows); 
	float min_length = 30.0f / 800 * hypot(mImage.cols, mImage.rows); 
	int min_vote = 30.0 / 800 * hypot(mImage.cols, mImage.rows); 
//	std::cout << blurRadius << std::endl; 
	cv::Mat blured; 
	cv::GaussianBlur(mImage, blured, cv::Size(2 * blurRadius + 1, 2 * blurRadius + 1), blurRadius); 
	cv::equalizeHist(blured, blured); 

	cv::Mat edge; 
	cv::Canny(blured, edge, 50, 100, 3); 
	cv::HoughLinesP(edge, mLines, 1, CV_PI/180, min_vote, min_length, 2);
	cv::namedWindow("blured"); 
	cv::imshow("blured", edge); 
	cv::waitKey(); 

/*	for (size_t i = 0; i < mLines.size(); i++)
	{
		cv::line(mSketch, cv::Point2f(mLines[i][0], mLines[i][1]), cv::Point2f(mLines[i][2], mLines[i][3]), cv::Scalar(0, 0, 244), 3, 8); 
	}
	cv::namedWindow("line"); 
	cv::imshow("line", mSketch); 
	cv::waitKey(); */
}

std::vector<size_t>
RansacVanishingPoint::randPerm(size_t n) const 
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

cv::Point2f 
RansacVanishingPoint::intersectLines(const std::vector<cv::Vec4i> & lines) const
{
/*	cv::Mat N(std::max((int)lines.size(), 3), 3, CV_64F); 
	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::Mat n; 
		// OpenCV solveZ cannot solve 2x3 matrix correctly. 
		// This bug seems fixed after r8827, Jun 28, 2012. 
		// Add an extra row to X to make 3x3 matrix for this bug. 
		cv::Mat X(3, 3, CV_64F); 
		X.at<double>(0, 0) = lines[i][0]; 
		X.at<double>(0, 1) = lines[i][1]; 
		X.at<double>(0, 2) = 501.0; 
		X.at<double>(1, 0) = lines[i][2]; 
		X.at<double>(1, 1) = lines[i][3]; 
		X.at<double>(1, 2) = 501.0; 
		X.row(1).copyTo(X.row(2)); 

		cv::SVD::solveZ(X, n); 
		N.row(i) = n.t() * 1.0;  
//		std::cout << "X=" << X << std::endl; 
//		std::cout << "X=" << n << std::endl; 
	}
	if (lines.size() < 3)
		N.row(2).copyTo(N.row(2)); 
	cv::Mat v; 
//	std::cout << "N = " << N << std::endl; 
	cv::SVD::solveZ(N, v); 
	cv::Vec3f vec; 
	vec[0] = v.at<double>(0); 
	vec[1] = v.at<double>(1); 
	vec[2] = v.at<double>(2); 
	return vec; 
*/
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
//	std::cout << cv::Mat(vec) << linesSupport(vec, mLines).size() << std::endl; 
//	showLines(lines); 
	return cv::Point2f(x.at<double>(0), x.at<double>(1)); 
}

cv::Point2f
RansacVanishingPoint::sampleVanishingPt(const std::vector<cv::Vec4i> & lines) const
{
	std::vector<size_t> perm = randPerm(lines.size()); 
	std::vector<cv::Vec4i> sampleLines; 
	sampleLines.push_back(lines[perm[0]]); 
	sampleLines.push_back(lines[perm[1]]); 
//	showLines(sampleLines); 
	return intersectLines(sampleLines); 
}

float 
RansacVanishingPoint::distance(cv::Point2f pt, cv::Vec4f line) const
{
//	std::cout << cv::Mat(line)  << std::endl; 
	float mid_x = 0.5f * (line[0] + line[2]); 
	float mid_y = 0.5f * (line[1] + line[3]); 
	
	float v_x = pt.x; 
	float v_y = pt.y; 

	v_x = v_x - mid_x; 
	v_y = v_y - mid_y; 
	float r = hypot(v_x, v_y); 
	v_x /= r; 
	v_y /= r; 

	float l_x = line[0] - (line[0] + line[2]) / 2.0f; 
	float l_y = line[1] - (line[1] + line[3]) / 2.0f; 

	float proj = v_x * l_x + v_y * l_y; 
	return sqrt(l_x * l_x + l_y * l_y - proj * proj); 
}

std::vector<cv::Vec4i>
RansacVanishingPoint::linesSupport(cv::Point2f vpt, const std::vector<cv::Vec4i> & lines) const
{
	std::vector<cv::Vec4i> support; 
	for (size_t i = 0; i < lines.size(); i++)
		if (distance(vpt, lines[i]) < 2.0) support.push_back(lines[i]); 
	return support; 
}

void 
RansacVanishingPoint::removeVanishingLines(cv::Point2f vpt, std::vector<cv::Vec4i> & lines) const
{
	std::vector<cv::Vec4i> residual_lines; 
	for (size_t i = 0; i < lines.size(); i++)
		if (distance(vpt, lines[i]) > 2.0) residual_lines.push_back(lines[i]); 
	lines = residual_lines; 
}

cv::Point2f
RansacVanishingPoint::ransac2Lines(const std::vector<cv::Vec4i> & lines) const
{
	float p = 0.995f; 
	float r = 2.0f / lines.size(); 
	float k = log(1.0f - p) / log(1.0f - r * r); 
	size_t max_iter = 1000; 
	size_t it = 0; 
	size_t max_inliers = 2; 
	cv::Point2f vanishingPt; 
	while (it < k && it < max_iter)
	{
		cv::Point2f guess = sampleVanishingPt(lines); 
		size_t inliers = linesSupport(guess, lines).size(); 
//		std::cout << cv::Mat(guess) << inliers << std::endl; 
//		showLines(linesSupport(guess)); 
		if (inliers > max_inliers)
		{
			max_inliers = inliers; 
			vanishingPt = guess; 

			r = (float)max_inliers / lines.size(); 
			k = log(1.0f - p) / log(1.0f - r * r); 
			it = 0; 
		}
//		std::cout << k << " " << it  << std::endl; 
		it++; 
	}
//	std::cout << "==========" << cv::Mat(vanishingPt) << "  " << max_inliers << std::endl; 
	showLines(linesSupport(vanishingPt, lines)); 
//	vanishingPt = intersectLines(linesSupport(vanishingPt, lines)); 
//	std::cout << "=====" << cv::Mat(vanishingPt) << "  " << max_inliers << std::endl; 

	return vanishingPt; 

}

