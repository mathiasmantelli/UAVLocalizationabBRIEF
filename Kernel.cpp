#include "Kernel.h" 

#include <iostream>
//#ifndef CMATH_H
#include <cmath>
//endif

//Constructor and destructor of the abstract class CKernel
CKernel::CKernel():
m_kernelMask(NULL), m_maskSizeX(0), m_maskSizeY(0), m_maskLength(0), m_normalizationConstant(0.0)
{
}
CKernel::~CKernel()
{
	_cleanMaskMem();
}
//Norm
double CKernel::norm(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
//Alloc memory to the mask
void CKernel::_allocMaskMem(int w, int h)
{
	//Cleaning mask memory
	_cleanMaskMem();

	//Setting mask size parameters
	m_maskSizeX = w;
	m_maskSizeY = h;

	//Calculating mask length
	m_maskLength = m_maskSizeX*m_maskSizeY;

	//Alloc memory to the mask
	m_kernelMask = new double[m_maskLength];
}
void CKernel::_cleanMaskMem(void)
{
	//Cleaning mask memory
	if(m_kernelMask != NULL)
		delete [] m_kernelMask;
	m_kernelMask=NULL;

	//Setting mask values to zero
	m_maskLength = 0;
	m_maskSizeX = 0;
	m_maskSizeY = 0;
}
void CKernel::_checkDimensions(int w, int h)
{
	//Check if dimensions are valid
	if(w == 0)
	{
		std::cerr << "Warning: mask WIDTH with size 0. Setting the value to one" << std::endl;
		w = 1;
	}
	if(h == 0)
	{
		std::cerr << "Warning: mask HEIGHT with size 0. Setting the value to one" << std::endl;
		h = 1;
	}
}

void CKernel::normalizeMask(void)
{
	if(m_normalizationConstant==0)
		std::cout << "Warning: normalization constant is equal to zero." << std::endl;
	else
	{
		double sum=0;
		for(int s = 0; s < m_maskLength; s++)
		{
			double temp = m_kernelMask[s];
			temp=temp/m_normalizationConstant;

            m_kernelMask[s]=temp;
			//Three channels normalized xhould sum to 3
			sum+=temp;
		}
		
        double epsilon = 0.00001;
        if(!(sqrt((1.0-sum)*(1.0-sum))<=epsilon))
            std::cout << "Error: Values of the kernel were not normalized correctly." <<std::endl;
	}
}
void CKernel::setMaskSize(int *maskSizeX, int *maskSizeY, double *params)
{
	*maskSizeX = params[0];
	*maskSizeY = params[1];
}
void CKernel::calculateKernel(void)
{
	m_normalizationConstant = 1;
}

void CKernel::initializeKernel(double *params)
{
	//Temporary nask sizes
	int maskSizeX=1, maskSizeY=1;

	//Polymorfism will be used to overwrite this method
	setMaskSize(&maskSizeX, &maskSizeY, params);

	//Check mask dimensions
	_checkDimensions(maskSizeX, maskSizeY);

	//Alloc memory to the mask
	_allocMaskMem(maskSizeX, maskSizeY);

	//Calculating Mask according to each kernel - polymorfism again
	calculateKernel();

}

int CKernel::width(void) const
{
	return m_maskSizeX;
}
int CKernel::height(void) const
{
	return m_maskSizeY;
}
int CKernel::size(void) const
{
	return m_maskLength;
}
