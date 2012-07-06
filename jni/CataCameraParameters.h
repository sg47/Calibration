#ifndef CATACAMERAPARAMETERS_H
#define CATACAMERAPARAMETERS_H

#include <string>

namespace vcharge
{

class CataCameraParameters
{
public:
	CataCameraParameters();
	CataCameraParameters(
			int w_, int h_, 
			double xi_, 
			double k1_, double k2_, double p1_, double p2_, 
			double gamma1_, double gamma2_, double u0_, double v0_); 

	std::string& cameraName(void);
	int& imageWidth(void);
	int& imageHeight(void);
	double& xi(void);
	double& k1(void);
	double& k2(void);
	double& p1(void);
	double& p2(void);
	double& gamma1(void);
	double& gamma2(void);
	double& u0(void);
	double& v0(void);

	const std::string& cameraName(void) const;
	int imageWidth(void) const;
	int imageHeight(void) const;
	double xi(void) const;
	double k1(void) const;
	double k2(void) const;
	double p1(void) const;
	double p2(void) const;
	double gamma1(void) const;
	double gamma2(void) const;
	double u0(void) const;
	double v0(void) const;

	bool read(const std::string& filename);
	void write(const std::string& filename) const;

	friend std::ostream& operator<< (std::ostream& out, CataCameraParameters& params);

private:
	std::string m_cameraName;
	int m_imageWidth;
	int m_imageHeight;
	double m_xi;
	double m_k1;
	double m_k2;
	double m_p1;
	double m_p2;
	double m_gamma1;
	double m_gamma2;
	double m_u0;
	double m_v0;
};

}

#endif

