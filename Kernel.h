//******************************************************************************/
// Created by Vitor Jorge July 2009
//
// Abstract class with two methods that are changed by subclasses using 
// polymorfism to differentiate them.
//
// Re-implement setMaskSize and calculateKernel to generate different types 
// of kernel class.
//
// There is no need to normalize the kernel mask, it will be done automatically.
//
//******************************************************************************/

#ifndef __KERNEL_H__
#define __KERNEL_H__


// Abstract kernel class
class CKernel
{
public:
	CKernel(void);
	virtual ~CKernel(void);
	double norm(double x1, double y1, double x2, double y2);
	void initializeKernel(double *params);
	void normalizeMask(void);
	int width(void) const;
	int height(void) const;
	int size(void) const;
    double *m_kernelMask;
	void _cleanMaskMem(void);
protected:
	virtual void setMaskSize(int *maskSizeX, int *maskSizeY, double *params);
	virtual void calculateKernel(void);

	int m_maskSizeX;
    int m_maskSizeY;
    int m_maskLength;
	double m_normalizationConstant;
	//Shader stuff

private:
	void _allocMaskMem(int w, int h);

	void _checkDimensions(int w, int h);
	void _initTexture(void);

};

#endif
