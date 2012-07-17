#include <opencv2/core/core.hpp>

class RansacVanishingPoint
{
	cv::Mat mImage; 
	cv::Mat mSketch; 
	std::vector<cv::Vec4i> mLines; 
	std::vector<cv::Point2f> mVanishingPts; 
	std::vector<cv::Point2f> mOrthogonalVanishingPts; 
	float mFocal; 
	cv::Point2f mPrinciplePt; 

	bool mFixFocal; 
	bool mFixPriciplePt; 

public: 
	RansacVanishingPoint(const cv::Mat & image, cv::Point2f pp = cv::Point2f(-1.0f, -1.0f), float focal = -1.0f); 
	float getFocal() const; 
	cv::Mat getSketch() const; 
	void findVanishingPts(); 
	void findOrthogonalVanishingPts();
	bool orthogonalityDetected() const; 	
	std::vector<cv::Point2f> selectOrthogonalVanishingPts() const; 
	cv::Mat getRotation() const; 
	void showVanishing(const std::vector<cv::Point2f> & vanishingPts) const; 
	std::vector<cv::Point2f> getVanishingPts() const; 

protected:
	// No orthogonality
	cv::Point2f ransac2Lines(const std::vector<cv::Vec4i> & lines) const; 

	void detectLines(); 
	std::vector<size_t> randPerm(size_t n) const; 
	cv::Point2f intersectLines(const std::vector<cv::Vec4i> & lines) const; 
	cv::Point2f sampleVanishingPt(const std::vector<cv::Vec4i> & lines) const; 
	float distance(cv::Point2f point, cv::Vec4f line) const; 
	std::vector<cv::Vec4i> linesSupport(cv::Point2f point, const std::vector<cv::Vec4i> & lines) const; 
	void removeVanishingLines(cv::Point2f vanishingPt, std::vector<cv::Vec4i> & lines) const; 

	void selectOrthogonalVanishingPtsHelper(const std::vector<cv::Point2f> & vanishingPts, float & focal, float & err) const; 

	void showLines(const std::vector<cv::Vec4i> & lines) const; 
}; 
