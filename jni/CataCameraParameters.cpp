#include "CataCameraParameters.h"

#include <opencv2/core/core.hpp>
#include <iomanip>

namespace vcharge
{

CataCameraParameters::CataCameraParameters()
 : m_imageWidth(0)
 , m_imageHeight(0)
 , m_xi(0.0)
 , m_k1(0.0)
 , m_k2(0.0)
 , m_p1(0.0)
 , m_p2(0.0)
 , m_gamma1(0.0)
 , m_gamma2(0.0)
 , m_u0(0.0)
 , m_v0(0.0)
{

}

CataCameraParameters::CataCameraParameters(
			int w_, int h_, 
			double xi_, 
			double k1_, double k2_, double p1_, double p2_, 
			double gamma1_, double gamma2_, double u0_, double v0_)
 : m_imageWidth(w_)
 , m_imageHeight(h_)
 , m_xi(xi_)
 , m_k1(k1_)
 , m_k2(k2_)
 , m_p1(p1_)
 , m_p2(p2_)
 , m_gamma1(gamma1_)
 , m_gamma2(gamma2_)
 , m_u0(u0_)
 , m_v0(v0_)
{
}
std::string&
CataCameraParameters::cameraName(void)
{
	return m_cameraName;
}

int&
CataCameraParameters::imageWidth(void)
{
	return m_imageWidth;
}

int&
CataCameraParameters::imageHeight(void)
{
	return m_imageHeight;
}

double&
CataCameraParameters::xi(void)
{
	return m_xi;
}

double&
CataCameraParameters::k1(void)
{
	return m_k1;
}

double&
CataCameraParameters::k2(void)
{
	return m_k2;
}

double&
CataCameraParameters::p1(void)
{
	return m_p1;
}

double&
CataCameraParameters::p2(void)
{
	return m_p2;
}

double&
CataCameraParameters::gamma1(void)
{
	return m_gamma1;
}

double&
CataCameraParameters::gamma2(void)
{
	return m_gamma2;
}

double&
CataCameraParameters::u0(void)
{
	return m_u0;
}

double&
CataCameraParameters::v0(void)
{
	return m_v0;
}

const std::string&
CataCameraParameters::cameraName(void) const
{
	return m_cameraName;
}

int
CataCameraParameters::imageWidth(void) const
{
	return m_imageWidth;
}

int
CataCameraParameters::imageHeight(void) const
{
	return m_imageHeight;
}

double
CataCameraParameters::xi(void) const
{
	return m_xi;
}

double
CataCameraParameters::k1(void) const
{
	return m_k1;
}

double
CataCameraParameters::k2(void) const
{
	return m_k2;
}

double
CataCameraParameters::p1(void) const
{
	return m_p1;
}

double
CataCameraParameters::p2(void) const
{
	return m_p2;
}

double
CataCameraParameters::gamma1(void) const
{
	return m_gamma1;
}

double
CataCameraParameters::gamma2(void) const
{
	return m_gamma2;
}

double
CataCameraParameters::u0(void) const
{
	return m_u0;
}

double
CataCameraParameters::v0(void) const
{
	return m_v0;
}

bool
CataCameraParameters::read(const std::string& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);

	if (!fs.isOpened())
	{
		return false;
	}

	m_imageWidth = static_cast<int>(fs["image_width"]);
	m_imageHeight = static_cast<int>(fs["image_height"]);
	fs["camera_name"] >> m_cameraName;

	cv::FileNode n = fs["mirror_parameters"];
	m_xi = static_cast<double>(n["xi"]);

	n = fs["distortion_parameters"];
	m_k1 = static_cast<double>(n["k1"]);
	m_k2 = static_cast<double>(n["k2"]);
	m_p1 = static_cast<double>(n["p1"]);
	m_p2 = static_cast<double>(n["p2"]);

	n = fs["projection_parameters"];
	m_gamma1 = static_cast<double>(n["gamma1"]);
	m_gamma2 = static_cast<double>(n["gamma2"]);
	m_u0 = static_cast<double>(n["u0"]);
	m_v0 = static_cast<double>(n["v0"]);

	return true;
}

void
CataCameraParameters::write(const std::string& filename) const
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	fs << "image_width" << m_imageWidth;
	fs << "image_height" << m_imageHeight;
	fs << "camera_name" << m_cameraName;

	// mirror: xi
	fs << "mirror_parameters";
	fs << "{" << "xi" << m_xi << "}";

	// radial distortion: k1, k2
	// tangential distortion: p1, p2
	fs << "distortion_parameters";
	fs << "{" << "k1" << m_k1
			  << "k2" << m_k2
			  << "p1" << m_p1
			  << "p2" << m_p2 << "}";

	// projection: xi, gamma1, gamma2, u0, v0
	fs << "projection_parameters";
	fs << "{" << "gamma1" << m_gamma1
			  << "gamma2" << m_gamma2
			  << "u0" << m_u0
			  << "v0" << m_v0 << "}";

	fs.release();
}

std::ostream&
operator<< (std::ostream& out, CataCameraParameters& params)
{
	out << "Mirror Parameters" << std::endl;
	out << std::fixed << std::setprecision(10);
	out << "      xi " << params.m_xi << std::endl;

	// radial distortion: k1, k2
	// tangential distortion: p1, p2
	out << "Distortion Parameters" << std::endl;
	out << "      k1 " << params.m_k1 << std::endl
		<< "      k2 " << params.m_k2 << std::endl
		<< "      p1 " << params.m_p1 << std::endl
		<< "      p2 " << params.m_p2 << std::endl;

	// projection: xi, gamma1, gamma2, u0, v0
	out << "Projection Parameters" << std::endl;
	out << "  gamma1 " << params.m_gamma1 << std::endl
		<< "  gamma2 " << params.m_gamma2 << std::endl
		<< "      u0 " << params.m_u0 << std::endl
		<< "      v0 " << params.m_v0 << std::endl;

	return out;
}

}
