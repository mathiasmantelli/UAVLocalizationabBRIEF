//******************************************************************************/
// Created by Vitor Jorge July 2009
//
// CGaussian, CGaussian1D, CCircular and CEliptical are odd and the mask size 
// is calculated, and not passed as a parameter
//
// In the case of CGaussianM, CEpanechnikovM, CElipticalM, the kernel size can 
// be odd or even and some of the spcific kernel parameters are calculated later
//
// CGaussianC, CCircular, CAntiEllipsoid and CGaussianIsotropic mask size comes
// from radius
//
//******************************************************************************/
#ifndef __SOMEKERNELS_H__
#define __SOMEKERNELS_H__

#include "Kernel.h"
#include <iomanip>

#ifndef PI
#define PI 	3.1415926535897932384626433832795
#endif
#define CGaussian1H CGaussian1D
#define CGaussian1HM CGaussian1DM 

//Laplacian kernel class
class CLaplacian:public CKernel
{
public:
	CLaplacian(void);
	virtual void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	virtual void calculateKernel(void);
};



//Gaussian Kernel Class
class CGaussian:public CKernel
{
public:
	CGaussian(void);
	double getStdev(void);
	double getMean(void);
	//To be polymorfed by other gaussians
	//given the stdev and mean, calculate the 2D circular mask size
	virtual void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	virtual void calculateKernel(void);
protected:
	double m_stdev, m_mean;
};
//1D Gaussian Kernel Class
class CGaussian1D:public CGaussian
{
public:
	CGaussian1D(void);
	//given the stdev and mean, calculate the 1D mask size
	virtual void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	void calculateKernel(void);
};
class CGaussian1DM:public CGaussian1D
{
public:
	CGaussian1DM(void);
	//given the stdev and mean, calculate the 1D mask size
	void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
};
class CGaussian1V:public CGaussian
{
public:
	CGaussian1V(void);
	//given the stdev and mean, calculate the vertical 1D mask size
	virtual void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	void calculateKernel(void);
};
class CGaussian1VM:public CGaussian1V
{
public:
	CGaussian1VM(void);
	//given the stdev and mean, calculate the vertical 1D mask size
	virtual void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
};
//Circular kernel
class CCircular:public CKernel
{
public:
	CCircular(void);
	//given the radius, calculate the mask size
	void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	virtual void calculateKernel(void);
	int getRadius(void) const;
protected:
	int m_radius;
};
//The circular kernel approximation using the Gaussian kernel
class CGaussianC:public CCircular
{
public:
	CGaussianC(void);
	//given the radius, calculate the mask size using CCircular method
	//given the radius, calculate the mask size
	void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	void calculateKernel(void);
	double getStdev(void);
	double getMean(void);

protected:
	//stdev will always be equal to r and mean equal to zero
	double m_stdev, m_mean;

};

class CGaussianIsotropic:public CGaussianC
{
public:
	CGaussianIsotropic(void);
	//given the radius, calculate the mask size but zeroing the responses outside the circle
	void calculateKernel(void);
};

//Epanechnikov Kernel Class
class CEpanechnikov:public CCircular
{
public:
	CEpanechnikov(void);
	//given the radius, calculate the mask size using CCircular method
	void calculateKernel(void);
};
//Fixed coordinate Eliptical Kernel Class
class CEliptical:public CKernel
{
public:
	CEliptical(void);
	//given the two semi-axis dimensions, calculate the mask size
	void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	void calculateKernel(void);
	int getSemiAxisX(void);
	int getSemiAxisY(void);
protected:
	int m_radiusX, m_radiusY;
};
/*********** Kernels that pass the Mask dimensions as a parameter *********/
class CEpanechnikovM:public CKernel
{
public:
	CEpanechnikovM(void);
	//given the mask dimensions,calculate the size of the two semi-axis
	void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	void calculateKernel(void);
	double getSemiAxisX(void);
	double getSemiAxisY(void);
protected:
	double m_radiusX, m_radiusY;
};
class CGaussianM:public CKernel
{
public:
	CGaussianM(void);
	void calculateKernel(void);
	//given the mask dimensions,calculate the size of the two semi-axis
	virtual void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	double getStdevX(void);
	double getMeanX(void);
	double getStdevY(void);
	double getMeanY(void);
protected:
	double m_stdevX, m_meanX, m_stdevY, m_meanY;
};

class CAntiEllipsoid:public CCircular
{
public:
	CAntiEllipsoid(void);
	//given the radius, calculate the mask size
	virtual void calculateKernel(void);
	double getSemiAxis(void);
	double getConstantHeight(void);
};
/***************************** Composite Kernels ************************/
//class CCompositeGaussian2D
//{
//public:
//	CCompositeGaussian2D(void);
//	~CCompositeGaussian2D(void);
//	void initializeKernel(double *params);
//	void gpuConvolution(CTexture *frame);
//	double getStdevX(void);
//	double getMeanX(void);
//	double getStdevY(void);
//	double getMeanY(void);
//	int getMaskHeight(void);
//	int getMaskWidth(void);
//	void ResetFBO(void);
//	CTexture *backbuffer;
//
//protected:
//	CGaussian1H gaussian1H;
//	CGaussian1V gaussian1V;
//	double m_stdevX, m_meanX, m_stdevY, m_meanY;
//	//FBO stuff
//	CFramebufferObject *m_fbo;
//	CRenderbufferObject *m_rbo;
//
//};
//class CGaussianCircularProfile
// {
//public:
//	CGaussianCircularProfile(void);
//	~CGaussianCircularProfile(void);
//	void initializeKernel(double *params);
//	virtual void gpuConvolution(CTexture *frame, CTexture *backbuffer, CFramebufferObject *fbo);
//	double getStdevX(void);
//	double getMeanX(void);
//	double getStdevY(void);
//	double getMeanY(void);
//	int getMaskHeight(void);
//	int getMaskWidth(void);
//	void ResetFBO(void);
//
//protected:
//	CGaussian1HM gaussian1H;
//	CGaussian1VM gaussian1V;
//	double m_stdevX, m_meanX, m_stdevY, m_meanY;
//	CFramebufferObject *m_fbo;
//};


#endif
