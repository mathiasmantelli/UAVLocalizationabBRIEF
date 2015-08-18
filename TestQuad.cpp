#include "TestQuad.h"

TestQuad::TestQuad(void)
{
}
TestQuad::~TestQuad(void)
{
}
bool TestQuad::initialize(GLenum target, GLenum usage, GLuint w, GLuint h)
{
	//Cleaning data
	if(!m_cpudata.empty())
		m_cpudata.clear();

	// Fill system memory buffer to the size of the input data and make things faster
	m_cpudata.resize(4); //Four 2D float Vertices, four 3D float colors and four 2D float texcoord colors

	//According to the struct, assign buffer info
	setVertexArrayBuffer  (2, GL_FLOAT, 7*4, BUFFER_OFFSET(0));
	setColorArrayBuffer   (3, GL_FLOAT, 7*4, BUFFER_OFFSET(2*4), true);
	setTexcoordArrayBuffer(2, GL_FLOAT, 7*4, BUFFER_OFFSET(5*4), true);


	//Vertice 0
	m_cpudata[0].x=0.0f;
	m_cpudata[0].y=0.0f;
	//Color 0 BLACK
	m_cpudata[0].r=0.0f;
	m_cpudata[0].g=0.0f;
	m_cpudata[0].b=0.0f;
	//Texcoord LowerLeft
	m_cpudata[0].s=0.0f;
	m_cpudata[0].t=0.0f;

	//Vertice 1;
	m_cpudata[1].x=float(w);
	m_cpudata[1].y=0.0f;
	//Color 1 White
	m_cpudata[1].r=1.0f;
	m_cpudata[1].g=1.0f;
	m_cpudata[1].b=1.0f;
	//Texcoord LowerRight
	m_cpudata[1].s=float(w);
	m_cpudata[1].t=0.0f;

	//Vertice 2;
	m_cpudata[2].x=float(w);
	m_cpudata[2].y=float(h);
	//Color 2 Green
	m_cpudata[2].r=0.0f;
	m_cpudata[2].g=1.0f;
	m_cpudata[2].b=0.0f;
	//Texcoord UpperRight
	m_cpudata[2].s=float(w);
	m_cpudata[2].t=float(h);

	//Vertice 3;
	m_cpudata[3].x=0.0f;
	m_cpudata[3].y=float(h);
	//Color 3 Red
	m_cpudata[3].r=1.0f;
	m_cpudata[3].g=0.0f;
	m_cpudata[3].b=0.0f;
	//Texcoord UpperLeft
	m_cpudata[3].s=0.0f;
	m_cpudata[3].t=float(h);

	//Initialize VBO
	_initialize(target, usage);

	//free data
	m_cpudata.clear();

	return true;
}
void TestQuad::Draw(void)
{
	//Video using a vertex buffer object
	Bind();
	glDrawArrays(GL_QUADS, 0, 4*3);
	//glDrawRangeElements is faster but more difficult to handle, because we need indexes
	Unbind();
}
QuadofPoints::QuadofPoints(void)
{
}
QuadofPoints::~QuadofPoints(void)
{
}
bool QuadofPoints::initialize(GLenum target, GLenum usage, GLuint w, GLuint h)
{
	if(!m_cpudata.empty())
		m_cpudata.clear();

	// Create system memory buffer - Vertex XYZW is on the first, texcoord STRV on the second
	// Fill system memory buffer to the size of the input data and make things faster
	m_cpudata.resize((w)*(h)*4);
	// Setting the array information
	setVertexArrayBuffer(2, GL_FLOAT, 4*4, BUFFER_OFFSET(0));
	setTexcoordArrayBuffer(2, GL_FLOAT, 4*4, BUFFER_OFFSET(2*4), true);

	int address=0;
	for(int height = 0; height < h; height++)
	{
		for(int width = 0; width < w; width++)
		{

			m_cpudata[address]=width+.5f;
			m_cpudata[address+1]=height+.5f;
			m_cpudata[address+2]=width+.5f;
			m_cpudata[address+3]=height+.5f;
			address+=4;
		}
	}
	//Initialize VBO
	_initialize(target, usage);

	//free data
	m_cpudata.clear();

	return true;
}
//Draw method
void QuadofPoints::Draw()
{
	//Capture current viewport
	int currentViewport[4];
	glGetIntegerv(GL_VIEWPORT, currentViewport);

	//Video using a vertex buffer object
	Bind();
	glDrawArrays(GL_POINTS, 0, 2*(currentViewport[2])*(currentViewport[3]));
	//glDrawRangeElements is faster but more difficult to handle, because we need indexes
	Unbind();
}

SimpleQuad::SimpleQuad()
{
}
SimpleQuad::~SimpleQuad()
{
}
void SimpleQuad::drawQuad(CTexture &frame)
{
	//Saving current viewport
	int currentViewport[4];
	glGetIntegerv(GL_VIEWPORT, currentViewport);

	//Changing the viewport
	glViewport(currentViewport[0], currentViewport[1], frame.GetWidth(), frame.GetHeight());

	//Setting new projection
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, 1.0f, 0, 1.0f, 1.0f, -1.0f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	
	frame.Bind(0);

	//Position the quad 
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(0.0f, 0.0f);
	glTexCoord2f(frame.GetWidth(), 0.0f);
	glVertex2f(1.0f, 0.0f);
	glTexCoord2f(frame.GetWidth(), frame.GetHeight());
	glVertex2f(1.0f, 1.0f);
	glTexCoord2f(0.0f, frame.GetHeight());
	glVertex2f(0.0f, 1.0f);
	glEnd();

	frame.DisableTextureUnit(0);
	glDisable(GL_TEXTURE_RECTANGLE_ARB);

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	//Reloading the previous viewport
	glViewport(currentViewport[0], currentViewport[1], currentViewport[2], currentViewport[3]);
}
void SimpleQuad::drawFloatTextureFromVec3(std::vector<vec3> &doubleCpuTex, int width, int height)
{
	//Generating CPU texture and setting the size
	std::vector<float> cpuTexture;
	int size = width*height*3;
	cpuTexture.resize(size);

	float farray[3]; 
	for(int s = 0; s < size; s=s+3)
	{
		farray[0]=float(doubleCpuTex[s/3].r/*/100.0f*/);
		farray[1]=float(doubleCpuTex[s/3].g/*/100.0f*/);
		farray[2]=float(doubleCpuTex[s/3].b/*/100.0f*/);
		memcpy(&cpuTexture[s], &farray[0], sizeof(float)*3);
	}
	glDrawPixels(width, height, GL_RGB, GL_FLOAT, &cpuTexture[0]);
}
/**
*/
void SimpleQuad::drawImage(unsigned char* image, int width, int height)
{
	GLboolean test=GL_FALSE;
	glGetBooleanv(GL_CURRENT_RASTER_POSITION_VALID, &test);
	if(test!=GL_FALSE)
	{
		if(image!=NULL)
		{
			glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
			getDrawImageError();

		}
		else			
			std::cerr << "Error: Trying to print empty image!\n";

	}
	else
	{		
		std::cerr << "RASTER POSITION INVÁLIDA!\n";
	}
}
void SimpleQuad::drawImage(unsigned int* image, int width, int height)
{
	GLboolean test=GL_FALSE;
	glGetBooleanv(GL_CURRENT_RASTER_POSITION_VALID, &test);
	if(test==GL_FALSE)
		std::cout << "Fuuu!!!!\n";

	glDrawPixels(width, height, GL_RGB, GL_UNSIGNED_INT, image);
	getDrawImageError();
	
}

void SimpleQuad::getDrawImageError()
{
	GLenum error = glGetError();
	switch(error)
	{
	case GL_INVALID_VALUE:
		std::cout << "Draw Image Error: either width or height is negative." <<std::endl;
		break;
	case GL_INVALID_ENUM:
		std::cout << "Draw Image Error: format or type is not one of the accepted values." <<std::endl;
		break;
	case GL_INVALID_OPERATION:
		std::cout << "Draw Image Error: format is GL_RED, GL_GREEN, GL_BLUE, GL_ALPHA, GL_RGB, GL_RGBA, GL_LUMINANCE, or GL_LUMINANCE_ALPHA, and the GL is in color index mode." <<std::endl;
		break;
	case GL_NO_ERROR:
		//std::cout << "Yei!!!\n";
		break;
	default:
		std::cout << "nknown draw image error!" << error << std::endl;
		break;
	};
}