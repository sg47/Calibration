#include <opencv2/core/core.hpp>

#define NOTHING_DETECTED 0
#define ONE_DETECTED 1
#define TWO_DETECTED 2
#define TWO_NON_ORTHOGONAL_DETECTED 3
#define TWO_DETECTED_WITH_ONE_INFINITE 4
#define THREE_DETECTED_WITH_ONE_INFINITE 5
#define THREE_DETECTED_WITH_TWO_INFINITE 6
#define THREE_DETECTED_WITH_REFINERY 7
#define THREE_DETECTED_WITHOUT_REFINERY 8

class Cas1DVanishingPoint
{
	cv::Mat mImage; 
	cv::Mat mSketch; 
	std::vector<cv::Vec4i> mLines; 
	std::vector<cv::Point2f> mVanishingPts; 
	float mInteriorRadius; 
	float mFocal; 
	cv::Point2f mPrinciplePt; 
	int mMessage; 

public: 
	Cas1DVanishingPoint(const cv::Mat & image); 

	bool focalAvailable() const; 
	float getFocal() const; 

	bool threeDetected() const; 

	cv::Mat getSketch() const; 

	void findOrthogonalVanishingPts(); 
	std::vector<cv::Point2f> getVanishingPts() const; 

	void 
		showLines(const std::vector<cv::Vec4i> & lines) const; 

	void 
		showVanishing(const std::vector<cv::Point2f> & vanishingPts) const;
protected:
	void detectLines(); 

	double 
		mod(double x, double d) const; 
	cv::Point2f 
		convergeLines(const std::vector<cv::Vec4i> & lines) const; 
	std::vector<cv::Point2f> 
		intersectLines(const std::vector<cv::Vec4i> & lines) const; 
	float 
		distance(float theta, float rho, cv::Vec4f line) const; 
	std::vector<cv::Vec4i> 
		linesSupport(float theta, float rho, const std::vector<cv::Vec4i> & lines) const; 
	void 
		removeVanishingLines(float theta, float rho, std::vector<cv::Vec4i> & lines) const; 

	float 
		findTheta(const std::vector<float> & thetas, std::vector<size_t> & idx) const; 
	float 
		findRho(const std::vector<float> & rhos, const std::vector<size_t> & idx) const; 
	void 	
		findExteriorVanishingPt(const std::vector<cv::Point2f> & pts, float & extVPTheta, float & extVPRho) const; 
	void 	
		findInteriorVanishingPt(const std::vector<cv::Point2f> & pts, float & intVPTheta, float & intVPRho) const; 
	void
		find3rdDegenVanishingPt(const std::vector<cv::Point2f> & pts, 
				const std::vector<float> & thetas, const std::vector<float> & rhos, 
				float & theta3, float & rho3) const; 
	bool 
		refineVanishingPts(const std::vector<cv::Point2f> & pts, const std::vector<float> & vanishingThetas, std::vector<float> & vanishingRhos, float & focal) const; 
};

