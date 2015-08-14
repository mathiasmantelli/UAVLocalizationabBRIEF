#include "SomeKernels.h"

#include <iostream>
#include <cmath>
int roundNumber(double num)
{
	return (((num-int(num))>=.5)?int(ceil(num)):int(floor(num)));
}
int round(float num)
{
	return (((num-int(num))>=.5)?int(ceil(num)):int(floor(num)));
}
int round(long double num)
{
	return (((num-int(num))>=.5)?int(ceil(num)):int(floor(num)));
}

//Laplacian Constructor and  Methods
CLaplacian::CLaplacian(void){}

void CLaplacian::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	*maskSizeX = 3;
	*maskSizeY = 3;
}
void CLaplacian::calculateKernel(void)
{
	//Set values
	m_kernelMask[0]=1.0f;
	m_kernelMask[1]=2.0f;
	m_kernelMask[2]=1.0f;
	m_kernelMask[3]=2.0f;
	m_kernelMask[4]=-12.0f;
	m_kernelMask[5]=2.0f;
	m_kernelMask[6]=1.0f;
	m_kernelMask[7]=2.0f;
	m_kernelMask[8]=1.0f;

	m_normalizationConstant = 4.0f;
	
	//Normalize it
	normalizeMask();
}
//Class Gaussian:Kernel constructor and methods
CGaussian::CGaussian()
{
}

void CGaussian::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	m_mean = params[0];
	m_stdev = params[1];

	float gaussian_value = 0;
	int xValue = 3*m_stdev;
	int yValue = 3*m_stdev;

	//Four digits after comma are considered to be enough
	while(gaussian_value <= .0001)
	{
		//leave loop if the value is smaller than 1
		if(xValue <= 1 && yValue<=1)
		{
			xValue = 1; yValue = 1;
			break;
		}
		if(xValue>1) xValue--;
		else xValue = 1;
		if(yValue>1) yValue--;
		else yValue = 1;
		gaussian_value = (1/(2*(m_stdev*m_stdev)*PI))*exp(-((xValue-m_mean)*(xValue-m_mean))/(2*(m_stdev*m_stdev))-((yValue-m_mean)*(yValue-m_mean))/(2*(m_stdev*m_stdev)));
	}
	*maskSizeX = (xValue)*2+1;
	*maskSizeY = (yValue)*2+1;
}
double CGaussian::getMean()
{
	return m_mean;
}

double CGaussian::getStdev(void)
{
	return m_stdev;
}
void CGaussian::calculateKernel(void)
{
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Calculating kernel
			double xValue = (x-(m_maskSizeX-1)/2.0f);
			double yValue = y-(m_maskSizeY-1)/2.0f;
			double gaussian_value = (1/(2*(m_stdev*m_stdev)*PI))*exp(-((xValue-m_mean)*(xValue-m_mean))/(2*(m_stdev*m_stdev))-((yValue-m_mean)*(yValue-m_mean))/(2*(m_stdev*m_stdev)));
			//Set values
			m_kernelMask[x+m_maskSizeX*y+0]=gaussian_value;
			m_normalizationConstant += gaussian_value;
			//std::cout << circular_value << " ";
		}
		//std::cout << "" << std::endl;
	}
	
	//Normalize it
	normalizeMask();
}
//Class Circular:Kernel constructor and methods
CCircular::CCircular()
{
}

void CCircular::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	//Set spcific parameters
	double r = *params;
	m_radius = int(r);

	*maskSizeX = m_radius*2+1;
	*maskSizeY = m_radius*2+1;
}
void CCircular::calculateKernel(void)
{
	//Calculate the not normalized kernel and the normalization constant
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Check if it is inside the circle
			float norma = norm(0, 0,x-m_radius,y-m_radius);
			float circular_value = ((norma<=m_radius+.5) ? 1.0f : 0.0f);
			m_kernelMask[x+m_maskSizeX*y]=circular_value;
			m_normalizationConstant += circular_value;
			std::cout << circular_value << " ";
		}
		std::cout << "" << std::endl;
	}
	//Normalize it
	normalizeMask();
}

int CCircular::getRadius(void) const
{
	return m_radius;
}

//Class CGaussianC: Kernel constructor and methods
CGaussianC::CGaussianC(void){}

void CGaussianC::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	//Set specific parameters

	double r = *params;
	m_radius = int(r);
    m_stdev = *params;
	*maskSizeX = m_radius*2+1;
	*maskSizeY = m_radius*2+1;
}
void CGaussianC::calculateKernel(void)
{

	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Calculating kernel
			double xValue = x-(m_maskSizeX-1)/2.0f;
			double yValue = y-(m_maskSizeY-1)/2.0f;
			double gaussian_value = (1/(2*(m_stdev*m_stdev)*PI))*exp(-((xValue)*(xValue))/(2*(m_stdev*m_stdev))-((yValue)*(yValue))/(2*(m_stdev*m_stdev)));
			//Set values
			m_kernelMask[x+m_maskSizeX*y]=gaussian_value;
			m_normalizationConstant += gaussian_value;
			//std::cout << circular_value << " ";
		}
		//std::cout << "" << std::endl;
	}
	
	//Normalize it
	normalizeMask();
}
double CGaussianC::getStdev(void)
{
	return m_stdev;
}
double CGaussianC::getMean(void)
{
	return m_mean;
}

//Class CGaussianIsotropic: Kernel constructor and methods
CGaussianIsotropic::CGaussianIsotropic(void){}

void CGaussianIsotropic::calculateKernel(void)
{
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Check if it is inside the circle
            double norma = norm(0, 0,x-m_radius,y-m_radius);
			//Calculating kernel
			double xValue = x-(m_maskSizeX-1)/2.0f;
			double yValue = y-(m_maskSizeY-1)/2.0f;
			double gaussian_value = ((norma<=m_radius+.5) 
				? (1/(2*(m_stdev*m_stdev)*PI))*exp(-((xValue)*(xValue))/(2*(m_stdev*m_stdev))-((yValue)*(yValue))/(2*(m_stdev*m_stdev))) 
				: 0.0f);
			//Set values
			m_kernelMask[x+m_maskSizeX*y]=gaussian_value;
			m_normalizationConstant += gaussian_value;
			//std::cout << circular_value << " ";
		}
		//std::cout << "" << std::endl;
	}
	
	//Normalize it
	normalizeMask();
}


//Class CEpanechnikov:Kernel constructor and methods
CEpanechnikov::CEpanechnikov()
{
}

void CEpanechnikov::calculateKernel(void)
{
	//Calculate the not normalized kernel and the normalization constant
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Check if it is inside the circle
            double norma = norm(0, 0,x-m_radius,y-m_radius);
            double normalized_value = (0.75f)*(1-(norma/m_radius)*(norma/m_radius));
            double value = (norma < double(m_radius) +.5) ? normalized_value : 0.0f;
			m_kernelMask[x+m_maskSizeX*y]=value;
			m_normalizationConstant += value;
			//std::cout << value << " ";
		}
		//std::cout << "" << std::endl;
	}
	//Normalize it
	normalizeMask();
}

//Class CCircular:Kernel constructor and methods
CEliptical::CEliptical()
{
}

void CEliptical::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	//Set spcific parameters Y-axis radius and X-axis radius
	m_radiusX = params[0];
	m_radiusY = params[1];

	*maskSizeX = m_radiusX*2+1;
	*maskSizeY = m_radiusY*2+1;
}
void CEliptical::calculateKernel(void)
{
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Interpolating all the window values to values between -1 and 1
            double eliptical_value, xcoord;
			xcoord = float(x);
			//Check if it is inside the elipse
            double xDistance = xcoord-m_radiusX;
            double yDistance = y-m_radiusY;
			eliptical_value = (((xDistance)/m_radiusX)*((xDistance)/m_radiusX)+((yDistance)/m_radiusY)*((yDistance)/m_radiusY)<=1) ? 1.0f : 0.0f;
			//Set values
			m_kernelMask[x+m_maskSizeX*y+0]=eliptical_value;
			m_normalizationConstant += eliptical_value;
			//std::cout << circular_value << " ";
		}
		//std::cout << "" << std::endl;
	}

	//Normalize it
	normalizeMask();
}
//1D Horizontal Gaussian stuff 
CGaussian1D::CGaussian1D()
{
}

void CGaussian1D::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	m_mean = params[0];
	m_stdev = params[1];

	double gaussian_value = 0;
	int xValue = 3.0*m_stdev;

	//Four digits after comma are considered to be enough
	while(gaussian_value <= .0001)
	{
		if(xValue <= 1)
		{
			xValue = 1;
			break;
		}
		xValue--;
		//Calculate border value
		gaussian_value = (1/(m_stdev*sqrt(2*PI)))*exp(-((xValue-m_mean)*(xValue-m_mean))/(2*(m_stdev*m_stdev)));
	}
	*maskSizeX = (xValue)*2+1;
	*maskSizeY = 1;
}
void CGaussian1D::calculateKernel(void)
{
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Calculating kernel
			double xValue = (x-(m_maskSizeX-1)/2.0f);
			//double yValue = y-(m_maskSizeY-1)/2.0f;
			double gaussian_value = (1/(m_stdev*sqrt(2*PI)))*exp(-((xValue-m_mean)*(xValue-m_mean))/(2*(m_stdev*m_stdev)));;
			//Set values
			m_kernelMask[x+m_maskSizeX*y]=gaussian_value;
			m_normalizationConstant += gaussian_value;
			//std::cout << gaussian_value << " ";
		}
		//std::cout << "" << std::endl;
	}
	
	//Normalize it
	normalizeMask();
}
//1D Vertical Gaussian stuff
CGaussian1V::CGaussian1V()
{
}

void CGaussian1V::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	m_mean = params[0];
	m_stdev = params[1];

	double gaussian_value = 0;
	int yValue = 3.0*m_stdev;

	//Four digits after comma are considered to be enough
	while(gaussian_value <= .0001)
	{
		if(yValue <= 1)
		{
			yValue = 1;
			break;
		}
		yValue--;
		//Calculate border value
		gaussian_value = (1/(m_stdev*sqrt(2*PI)))*exp(-((yValue-m_mean)*(yValue-m_mean))/(2*(m_stdev*m_stdev)));
	}
	*maskSizeX = 1;
	*maskSizeY = (yValue)*2+1;
}
void CGaussian1V::calculateKernel(void)
{
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Calculating kernel
			//double xValue = (x-(m_maskSizeX-1)/2.0f);
			double yValue = y-(m_maskSizeY-1)/2.0f;
			double gaussian_value = (1/(m_stdev*sqrt(2*PI)))*exp(-((yValue-m_mean)*(yValue-m_mean))/(2*(m_stdev*m_stdev)));;
			//Set values
			m_kernelMask[x+m_maskSizeX*y]=gaussian_value;
			m_normalizationConstant += gaussian_value;
			//std::cout << gaussian_value << " ";
		}
		//std::cout << "" << std::endl;
	}
	
	//Normalize it
	normalizeMask();
}


//Gaussian with parameters different in x and y coordinates, mask size defined by parameters
CGaussianM::CGaussianM()
{
}

void CGaussianM::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	m_meanX = params[0];
	m_stdevX = params[1];
	m_meanY = params[2];
	m_stdevY = params[3];
	*maskSizeX = params[4];
	*maskSizeY = params[5];
}
void CGaussianM::calculateKernel(void)
{
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Calculating kernel
			double xValue = (x-(m_maskSizeX-1)/2.0f);
			double yValue = y-(m_maskSizeY-1)/2.0f;
			double gaussian_value = (1/(2*(m_stdevX*m_stdevY)*PI))*exp(-((xValue-m_meanX)*(xValue-m_meanX))/(2*(m_stdevX*m_stdevX))-((yValue-m_meanY)*(yValue-m_meanY))/(2*(m_stdevY*m_stdevY)));
			//Set values
			m_kernelMask[x+m_maskSizeX*y]=gaussian_value;
			m_normalizationConstant += gaussian_value;
			//std::cout << circular_value << " ";
		}
		//std::cout << "" << std::endl;
	}
	
	//Normalize it
	normalizeMask();
    // //Display
	//for(int y=0; y < m_maskSizeY; y++)
	//{
	//	for(int x=0; x < m_maskSizeX; x++)
	//	{
	//		std::cout << m_kernelMask[x+m_maskSizeX*y+0] << " ";
	//	}
	//	std::cout << "" << std::endl;
	//}
}
double CGaussianM::getStdevX(void)
{
	return m_stdevX;
}
double CGaussianM::getMeanX(void)
{
	return m_meanX;
}
double CGaussianM::getStdevY(void)
{
	return m_stdevY;
}
double CGaussianM::getMeanY(void)
{
	return m_meanY;
}


//Epanechnikov set with mask size specific
CEpanechnikovM::CEpanechnikovM()
{
}

void CEpanechnikovM::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{

	//Set spcific parameters Y-axis radius and X-axis radius
	*maskSizeX = params[0];
	*maskSizeY = params[1];
    m_radiusX = params[0]/2.0;
    m_radiusY = params[1]/2.0;

}
void CEpanechnikovM::calculateKernel()
{
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Interpolating all the window values to values between -1 and 1
            double value, xcoord;
            xcoord = double(x);
			//Check if it is inside the elipse
            double xDistance = xcoord-m_radiusX;
            double yDistance = y-m_radiusY;
            double temp = ((xDistance)/m_radiusX)*((xDistance)/m_radiusX)+((yDistance)/m_radiusY)*((yDistance)/m_radiusY);
			value = (temp<=1) ? 0.75*(1.0f-temp): 0.0f;
			//Set values
			m_kernelMask[x+m_maskSizeX*y]=value;
			m_normalizationConstant += value;
			//std::cout << value << " ";
		}
		//std::cout << "" << std::endl;
	}

	//Normalize it
	normalizeMask();
}
double CEpanechnikovM::getSemiAxisX()
{
	return m_radiusX;
}
double CEpanechnikovM::getSemiAxisY()
{
	return m_radiusY;
}
//1D Circular Profile Gaussian parameters
CGaussian1DM::CGaussian1DM()
{
}

void CGaussian1DM::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	m_mean = params[0];
	m_stdev = params[1];
	//circular profile with a gaussian mask
	int size = round(2.0f*m_stdev);
	//ensure odd mask
	*maskSizeX = ((size%2)==0) ? size+1 : size;
	*maskSizeY = 1;
}
CGaussian1VM::CGaussian1VM()
{
}

void CGaussian1VM::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	m_mean = params[0];
	m_stdev = params[1];

	//circular profile with a gaussian mask
	int size = round(2.0f*m_stdev);
	//ensure odd mask
	*maskSizeX = 1;
	*maskSizeY = ((size%2)==0) ? size+1 : size;
}

/******************** AntiEllipsoid ******************************/
CAntiEllipsoid::CAntiEllipsoid(void){}

void CAntiEllipsoid::calculateKernel(void)
{
	//values that zeroes the ellipsoid and cilinder volume sum
	const double constantheight = (.125f +.851427f)*m_radius;
	const double semiaxis = .851427f*m_radius;

	//Calculate the not normalized kernel and the normalization constant
	for(int y=0; y < m_maskSizeY; y++)
	{
		for(int x=0; x < m_maskSizeX; x++)
		{
			//Check if it is inside the circle
            double norma = norm(0, 0,x-m_radius,y-m_radius);
			//The .851427Rradius isthe ellipsoid semiaxis equals the Edge and CEnter line situations
            double circular_value = ((norma<=m_radius) ? constantheight - semiaxis/m_radius*sqrt(m_radius*m_radius-norma*norma) : 0.0f);
			if(sqrt((norma-m_radius)*(norma-m_radius)) <=.5 && sqrt((norma-m_radius)*(norma-m_radius)) > 0)
				circular_value = constantheight;
			m_kernelMask[x+m_maskSizeX*y]=circular_value;
			m_normalizationConstant += circular_value;
			//std::cout << circular_value << " ";
			std::cout << std::setprecision(3) << std::fixed << circular_value<< " ";
		}
		std::cout << std::endl;
	}
	//Normalize it
	normalizeMask();

}

///******************* Compsite Kernels **************************/
//CCompositeGaussian2D::CCompositeGaussian2D():
//m_fbo(0), m_rbo(0), backbuffer(0)
//{
//}
//CCompositeGaussian2D::~CCompositeGaussian2D()
//{
//	//Deleting the framebuffer
//	if(m_fbo != NULL)
//		if(glIsFramebuffer(m_fbo->GetName()))
//		{
//			delete m_fbo;
//		}
//	m_fbo = NULL;
//	//Deleting the framebuffer
//	if(m_rbo != NULL)
//		if(glIsRenderbuffer(m_rbo->GetName()))
//		{
//			delete m_rbo;
//		}
//	m_rbo = NULL;
//
//	//Deleting the intermediate texture
//	if(backbuffer != NULL)
//		if(glIsTexture(backbuffer->GetName()))
//		{
//			delete backbuffer;
//		}
//	backbuffer = NULL;
//	//gaussian1H._cleanMaskMem();
//	//gaussian1V._cleanMaskMem();
//}
//void CCompositeGaussian2D::initializeKernel(double *params)
//{
//	//Starting Horizontal Gaussian
//	m_meanX = params[0]; m_stdevX = params[1];
//	double Hparams[]={params[0], params[1]};
//	gaussian1H.initializeKernel(Hparams);
//	//Starting Vertical Gaussian
//	m_meanY = params[2]; m_stdevY = params[3];
//	double Vparams[]={params[2], params[3]};
//	gaussian1V.initializeKernel(Vparams);
//	
//	ResetFBO();
//}
//void CCompositeGaussian2D::gpuConvolution(CTexture *frame)
//{
//	//Enabling first pass
//	m_fbo->Bind();
//	gaussian1H.gpuConvolution(frame);
//	GLint lasbuffer;
//	glDrawBuffer(GL_COLOR_ATTACHMENT0);
//	m_fbo->Disable();
//	gaussian1V.gpuConvolution(backbuffer);	
//}
//double CCompositeGaussian2D::getStdevX(void)
//{
//	return gaussian1H.getStdev();
//}
//double CCompositeGaussian2D::getMeanX(void)
//{
//	return gaussian1H.getMean();
//}
//double CCompositeGaussian2D::getStdevY(void)
//{
//	return gaussian1V.getStdev();
//}
//double CCompositeGaussian2D::getMeanY(void)
//{
//	return gaussian1V.getMean();
//}
//void CCompositeGaussian2D::ResetFBO(void)
//{
//	//Deleting the framebuffer
//	if(m_fbo != NULL)
//		if(glIsFramebuffer(m_fbo->GetName()))
//		{
//			delete m_fbo;
//		}
//	m_fbo = NULL;
//
//	//Deleting the framebuffer
//	if(m_rbo != NULL)
//		if(glIsRenderbuffer(m_rbo->GetName()))
//		{
//			delete m_rbo;
//		}
//	m_rbo = NULL;
//
//	//Deleting the intermediate texture
//	if(backbuffer != NULL)
//		if(glIsTexture(backbuffer->GetName()))
//		{
//			delete backbuffer;
//		}
//	backbuffer = NULL;
//
//	//Grabbing the viewport size
//	GLint currentViewport[]={0,0,0,0};
//	glGetIntegerv(GL_VIEWPORT, currentViewport);
//
//	//Setting the new texture
//	backbuffer = new CTexture;
//	glEnable(GL_TEXTURE_RECTANGLE_ARB);
//	//The most perfect calculation should be done with float GL_RGB32F, however GL_RGB is faster
//	backbuffer->Initialize(GL_TEXTURE_RECTANGLE_ARB, GL_RGB);
//	backbuffer->Bind();
//	backbuffer->SetParameter(GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//	backbuffer->SetParameter(GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//	backbuffer->SetParameter(GL_TEXTURE_WRAP_S, GL_CLAMP);
//	backbuffer->SetParameter(GL_TEXTURE_WRAP_T, GL_CLAMP);
//	backbuffer->SetImage(currentViewport[2], currentViewport[3], 0, GL_BGR, GL_FLOAT, NULL);
//	glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
//	glDisable(GL_TEXTURE_RECTANGLE_ARB);
//
//	//setting the framebuffer
//	m_fbo = new CFramebufferObject;
//	m_fbo->Initialize();
//	m_fbo->AttachTexture(GL_COLOR_ATTACHMENT0, backbuffer);
//	m_rbo = new CRenderbufferObject;
//	m_rbo->Initialize(GL_DEPTH_COMPONENT24, currentViewport[2], currentViewport[3]);
//	m_fbo->AttachRenderbuffer(m_rbo);
//
//}
//int CCompositeGaussian2D::getMaskHeight(void)
//{
//	return gaussian1V.getMaskHeight();
//}
//int CCompositeGaussian2D::getMaskWidth(void)
//{
//	return gaussian1H.getMaskWidth();
//}
//
//CGaussianCircularProfile::CGaussianCircularProfile()
//{
//}
//CGaussianCircularProfile::~CGaussianCircularProfile()
//{
//}
//void CGaussianCircularProfile::initializeKernel(double *params)
//{
//	//Starting Horizontal Gaussian
//	m_meanX = params[0]; m_stdevX = params[1];
//	double Hparams[]={params[0], params[1]};
//	gaussian1H.initializeKernel(Hparams);
//	//Starting Vertical Gaussian
//	m_meanY = params[2]; m_stdevY = params[3];
//	double Vparams[]={params[2], params[3]};
//	gaussian1V.initializeKernel(Vparams);
//}
//void CGaussianCircularProfile::gpuConvolution(CTexture *frame, CTexture *backbuffer, CFramebufferObject *fbo)
//{
//	//Enabling first pass
//	fbo->Bind();
//	fbo->AttachTexture(GL_COLOR_ATTACHMENT0, backbuffer);
//	gaussian1H.gpuConvolution(frame);
//	glDrawBuffer(GL_COLOR_ATTACHMENT0);
//	fbo->Disable();
//	gaussian1V.gpuConvolution(backbuffer);	
//}
//double CGaussianCircularProfile::getStdevX(void)
//{
//	return gaussian1H.getStdev();
//}
//double CGaussianCircularProfile::getMeanX(void)
//{
//	return gaussian1H.getMean();
//}
//double CGaussianCircularProfile::getStdevY(void)
//{
//	return gaussian1V.getStdev();
//}
//double CGaussianCircularProfile::getMeanY(void)
//{
//	return gaussian1V.getMean();
//}
//int CGaussianCircularProfile::getMaskHeight(void)
//{
//	return gaussian1V.getMaskHeight();
//}
//int CGaussianCircularProfile::getMaskWidth(void)
//{
//	return gaussian1H.getMaskWidth();
//}
