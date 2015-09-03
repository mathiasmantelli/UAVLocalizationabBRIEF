#include "ColorCPU.h"

CPUColorConverter::CPUColorConverter(void)
{
}

CPUColorConverter::~CPUColorConverter(void)
{
}

//double CPUColorConverter::MAX(double a, double b, double c)
//{
//	double max;
//	if (a > b)
//		max = a;
//	else
//		max = b;
//	if(c > max )
//		max = c;

//	return max;
//}
//double CPUColorConverter::MIN(double a, double b, double c)
//{
//	double min;
//	if (a < b)
//		min = a;
//	else
//		min = b;
//	if(c < min )
//		min = c;

//	return min;
//}
vec3 CPUColorConverter::RGB255toNormalizedRGB(const vec3& colorRGB255)
{
	return vec3(colorRGB255/255.0);
}
vec3 CPUColorConverter::sRGBtorgb(const vec3& colorRGB)
{
	//sRGB factor, gamma power and threshold
	const double factor=.055f;
	const double gammaPower=2.4f;
	const double threshold=.04045f;
	//sRGB has two separated functions
	vec3 result;
	if(colorRGB.r > threshold)
		result.r=pow((colorRGB.r+factor)/1.055f, gammaPower);
	else
		result.r=colorRGB.r/12.92f;
	if(colorRGB.g > threshold)
		result.g=pow((colorRGB.g+factor)/1.055f, gammaPower);
	else
		result.g=colorRGB.g/12.92f;
	if(colorRGB.b > threshold)
		result.b=pow((colorRGB.b+factor)/1.055f, gammaPower);
	else
		result.b=colorRGB.b/12.92f;

	//result.x = pow(colorRGB.x, double(2.2f));
	//result.y = pow(colorRGB.y, double(2.2f));
	//result.z = pow(colorRGB.z, double(2.2f));

	//return color1 if mixBoolean is false and color2 if mix boolean is true
	return result;
}
vec3 CPUColorConverter::rgbXYZ(const vec3& rgbColor)
{
	//Conversion Matrix from rgb to XYZ D65 illuminant
	mat3x3 rgbtoXYZ(   
		vec3(0.4124564f,0.2126729f,0.0193339f), 
		vec3(0.3575761f,0.7151522f,0.1191920f), 
		vec3(0.1804375f,0.0721750f,0.9503041f));

	return vec3(rgbtoXYZ*rgbColor);
}

vec3 CPUColorConverter::XYZCIELAB(const vec3& colorXYZ, vec3 illuminant)
{
	double Ka = 24389.0f/27.0f;
	double Epsilon = 216.0f/24389.0f;

	//making XYZ relative to the illuminant D65
	vec3 oldcolorXYZ = colorXYZ/illuminant;
	//XYZ factors, cubic root and threshold
	const double Kfactor=Ka;
	const double cubicRoot=1.0f/3.0f;
	const double threshold=Epsilon;

	//conversion has two separated functions
	vec3 result;
	if(oldcolorXYZ.r > threshold)
		result.r=pow(oldcolorXYZ.r,cubicRoot);
	else
		result.r=(oldcolorXYZ.r*Kfactor+16.0f)/116.0f;
	if(oldcolorXYZ.g > threshold)
		result.g=pow(oldcolorXYZ.g,cubicRoot);
	else
		result.g=(oldcolorXYZ.g*Kfactor+16.0f)/116.0f;
	if(oldcolorXYZ.b > threshold)
		result.b=pow(oldcolorXYZ.b,cubicRoot);
	else
		result.b=(oldcolorXYZ.b*Kfactor+16.0f)/116.0f;

	double L = (result.y*116.0f-16.0f);
	double a = 500.0f*(result.x-result.y);
	double b = 200.0f*(result.y-result.z);
	L = isZero(L);
	a = isZero(a);
	b = isZero(b);

	//Calculating the Lab component values
	return vec3(L, a, b); 
}
// sRGB to L*a*b
vec3 CPUColorConverter::sRGBCIELAB(const vec3& colorRGB)
{
	// it is not scaled to one
	return XYZCIELAB(rgbXYZ(colorRGB));
}
vec3 CPUColorConverter::RGB255toCIELAB(const vec3& colorRGB255)
{
	return XYZCIELAB(rgbXYZ(sRGBtorgb(RGB255toNormalizedRGB(colorRGB255))));
}
vec3 CPUColorConverter::CIELABLCH(const vec3 &labcolor)
{	
	double H = degrees(atan2(labcolor.z, labcolor.y));
	return vec3(
		labcolor.x,
		sqrt(pow(labcolor.y, 2)+pow(labcolor.z, 2)),
		H>=0 ? H : H+360.0f);
}

vec3 CPUColorConverter::RGB255toCIELCH(const vec3& colorRGB255)
{
	return CIELABLCH(XYZCIELAB(rgbXYZ(RGB255toNormalizedRGB(colorRGB255))));
}

vec3 CPUColorConverter::RGB255toGrayscale(const vec3& colorRGB255)
{
	vec3 grayFactors = vec3(0.2989, 0.5870, 0.1140);
	vec3 color = RGB255toNormalizedRGB(colorRGB255);
    double gray = vec3::dot(color, grayFactors);
	return vec3(gray, gray, gray); //works
}
void CPUColorConverter::cpuRGBtoCIELABConversion(const unsigned char *frame, std::vector<vec3> &output)
{
	size_t size = output.size()*3;
	for(size_t i=0; i < size; i=i+3)
	{
		vec3 color(frame[i], frame[i+1], frame[i+2]);
		color =  RGB255toCIELAB(color);
		memcpy(&output[i/3], &color, sizeof(double)*3);
		//testing stuff
	}

}
void CPUColorConverter::cpuRGBtoCIELCHConversion(const unsigned char *frame, std::vector<vec3> &output)
{
	size_t size = output.size()*3;
	for(size_t i=0; i < size; i=i+3)
	{
		unsigned char tempData[3];
		memcpy(&tempData[0], &frame[i], sizeof(unsigned char)*3);
		vec3 color(tempData[0], tempData[1], tempData[2]);
		color =  RGB255toCIELCH(color);	
		output[i/3]=color;
	}
}
void CPUColorConverter::cpuRGBtoGrayscaleConversion(const unsigned char *frame, std::vector<vec3> &output)
{
	size_t size = output.size()*3;
	for(unsigned int i=0; i < size; i=i+3)
	{
		unsigned char tempData[3];
		memcpy(&tempData[0], &frame[i], sizeof(unsigned char)*3);
		vec3 color(tempData[0], tempData[1], tempData[2]);
		color =  RGB255toGrayscale(color);	
		output[i/3]=color;
	}
}
void CPUColorConverter::cpuRGBtoRGB(const unsigned char *frame, std::vector<vec3> &output)
{
	size_t size = output.size()*3;
	for(unsigned int i=0; i < size; i=i+3)
	{
		unsigned char tempData[3];
		memcpy(&tempData[0], &frame[i], sizeof(unsigned char)*3);
		vec3 color(tempData[0], tempData[1], tempData[2]);
		color =  RGB255toNormalizedRGB(color);
		output[i/3]=color;
	}
}
void CPUColorConverter::cpuColorConversion(const unsigned char *frame, std::vector<vec3> &output, int conversion)
{
	switch(conversion)
	{
	case RGBTOCIELCH:
		cpuRGBtoCIELCHConversion(frame, output);
		break;
	case RGBTOCIELAB:
		cpuRGBtoCIELABConversion(frame, output);
		break;
	case RGBTOGRAYSCALE:
		cpuRGBtoGrayscaleConversion(frame, output);
		break;
	case RGBTORGB:
		cpuRGBtoRGB(frame, output);
		break;
	default:
		std::cout << "unknown color conversion\n";
		break;
	};
}

double CPUColorConverter::DeltaECIE1976(vec3 colorLab1, vec3 colorLab2)
{
	return vec3::distance(colorLab1, colorLab2);
}
double CPUColorConverter::DeltaECIE1994(vec3 colorLab1, vec3 colorLab2)
{
	//Graphic Arts
	double KL = 1.0f, KC = 1.0f, KH = 1.0f;
	//for Textiles KL = 2.0f
	double K1 = 0.045f, K2 = 0.015f;

	vec3 colorLCH1 = CIELABLCH(colorLab1);
	vec3 colorLCH2 = CIELABLCH(colorLab2);
	//Calculating DeltaL
   	double deltaL = colorLab1.x - colorLab2.x;
	double deltaA = colorLab1.y - colorLab2.y;
	double deltaB = colorLab1.z - colorLab2.z;
	double deltaC = colorLCH1.y - colorLCH2.y;
	double deltaH2 = deltaA*deltaA+deltaB*deltaB-deltaC*deltaC;

	double SL =1.0f;
	double SC =1.0f+K1*colorLCH1.y;
	double SH =1.0f+K2*colorLCH1.y;

	//Checking if the square root will be negative
	if(deltaH2>=0)
	{
		double deltaH = sqrt(deltaH2);
		return sqrt(pow(deltaL/(KL*SL),2) + pow(deltaC/(KC*SC),2) + pow(deltaH/(KH*SH), 2));
	}
	else
		return sqrt(deltaL*deltaL+deltaA*deltaA+deltaB*deltaB);
}
double CPUColorConverter::DeltaECIE2000(vec3 colorLab1, vec3 colorLab2, double KL, double KC, double KH)
{
	//Delta-E CIE 2000
	//Delta L stuff
	double Lbar = (colorLab1.x + colorLab2.x)/2.0f;
	double Lbar50squared = pow(Lbar-50.0f, 2);
	double DeltaL = colorLab2.x - colorLab1.x;

	//Delta C stuff
	double C1 = vec2::length(vec2(colorLab1.y, colorLab1.z)); 
	double C2 = vec2::length(vec2(colorLab2.y, colorLab2.z));
	double Cbar7 = pow((C1+C2)/2.0f, 7);
	double G = (1.0f-sqrt(Cbar7/(Cbar7+25.0f)))/2.0f;
	double aL1 = colorLab1.y*(1.0f + G);
	double aL2 = colorLab2.y*(1.0f + G);
	double CL1 = length(vec2(aL1, colorLab1.z));
	double CL2 = length(vec2(aL2, colorLab2.z));
	double CLbar = (CL1+CL2)/2.0f;
	double CLbar7 = pow(CLbar,7);
	double CLbar25_7 = pow(CLbar/25.0, 7.0);
	double DeltaCL = CL2 - CL1;

	//delta H stuff
	double HL1 = degrees(atan2(colorLab1.z, aL1));
	double HL2 = degrees(atan2(colorLab2.z, aL2));
	if(HL1 < 0.0f)
		HL1 = HL1+360.0f;
	if(HL2 < 0.0f)
		HL2 = HL2+360.0f;
	double HLbar = (abs(HL1-HL2) > 180.0f) ? (HL1+HL2+360.0f)/2.0f : (HL1+HL2)/2.0f;
	double T = 1.0f-0.17f*cos(radians(HLbar-30.0f))+0.24f*cos(radians(2.0f*HLbar))+0.32f*cos(radians(3.0f*HLbar+6.0f))-0.20f*cos(radians(4.0f*HLbar-63.0f));
	double deltaHL = abs(HL2 - HL1);
	if(deltaHL <=180.0f)
		deltaHL = HL2 - HL1;
	else
	{
		if(HL2>HL1)
			deltaHL = HL2 - HL1 - 360.0f;
		else
			deltaHL = HL2 - HL1 + 360.0f;
	}
	double DeltaH = 2.0*sqrt(CL2*CL1)*sin(radians(deltaHL/2.0f));

	//Parameters for the calculation
	double SL = 1.0f+(0.015f*Lbar50squared)/sqrt(20.0f+Lbar50squared);
	double SC = 1.0f+0.045f*CLbar;
	double SH = 1.0f+0.015f*CLbar*T;
	double DeltaTetha = 30.0f*exp(-pow((HLbar-275.0f)/25.0f, 2));
	double RC = sqrt(CLbar25_7/(CLbar25_7+1.0f));
	double RT = -2.0f*RC*sin(radians(2.0f*DeltaTetha));

	//Result
	double cie1976 = DeltaECIE1976(colorLab1, colorLab2);
	double result = pow(DeltaL/(KL*SL),2)+pow(DeltaCL/(KC*SC),2)+pow(DeltaH/(KH*SH),2)+RT*(DeltaCL/(KC*SC))*DeltaH/(KH*SH);
	if(result> 0.0f/* && cie1976 < 5.0f*/)
		return sqrt(result);
	else
		return cie1976;
}

double CPUColorConverter::DeltaECMC1984(vec3 colorLab1, vec3 colorLab2, double l, double c)
{
	vec3 colorLCH1 = CIELABLCH(colorLab1);
   	vec3 colorLCH2 = CIELABLCH(colorLab2);
	
	//Calculating DeltaL
   	double deltaL = colorLab1.x - colorLab2.x;
	double deltaA = colorLab1.y - colorLab2.y;
	double deltaB = colorLab1.z - colorLab2.z;
	double deltaC = colorLCH1.y - colorLCH2.y;
    double dh = deltaA*deltaA+deltaB*deltaB-deltaC*deltaC;

    // avoid negative square-root
    if(dh >=0)
    {
        double deltaH = sqrt(dh);

        //Calculating SL
        double SL = (colorLab1.x < 16.0f)
                ? 0.511f
                : 0.040975f*colorLab1.x/(1.0f+0.01765f*colorLab1.x);
        //Calculating SC
        double SC = .0638f*colorLCH1.y/(1.0f+0.0131f*colorLCH1.y)+0.638f;
        /******* reach SH **********/
        //Calculate T
        double T = (164.0f <= colorLCH1.z && colorLCH1.z < 345.0f)
                ? (0.56f + abs(0.2f*cos(radians(colorLCH1.z+168.0f))))
                : (0.36f + abs(0.4f*cos(radians(colorLCH1.z+ 35.0f))));
        // Calculating C1^4 and F
        double Cab4 = pow(colorLCH1.y, 4);
        double F = sqrt(Cab4/(Cab4+1900.0f));
        //finally SH
        double SH = SC*(F*T+1.0f-F);

        //Checking if the square root will be negative
        double squaredValue = pow(deltaL/(l*SL), 2) + pow(deltaC/(c*SC), 2) + pow(deltaH/SH, 2);
        return sqrt(squaredValue);

    }
	else
        return sqrt(deltaL*deltaL+deltaA*deltaA+deltaB*deltaB);
}

double CPUColorConverter::DeltaEMixCIE2000(vec3 colorLab1, vec3 colorLab2, double KL, double KC, double KH)
{
	//The reference Color is color 1
	colorLab2 = colorLab2*0.75f+colorLab1*0.25f;

	//Delta-E CIE 2000
	//Delta L stuff
	double Lbar = (colorLab1.x + colorLab2.x)/2.0f;
	double Lbar50squared = pow(Lbar-50.0f, 2);
	double DeltaL = colorLab2.x - colorLab1.x;

	//Delta C stuff
	double C1 = vec2::length(vec2(colorLab1.y, colorLab1.z));
	double C2 = vec2::length(vec2(colorLab2.y, colorLab2.z));
	double Cbar7 = pow((C1+C2)/2.0f, 7);
	double G = (1.0f-sqrt(Cbar7/(Cbar7+25.0f)))/2.0f;
	double aL1 = colorLab1.y*(1.0f + G);
	double aL2 = colorLab2.y*(1.0f + G);
	double CL1 = length(vec2(aL1, colorLab1.z));
	double CL2 = length(vec2(aL2, colorLab2.z));
	double CLbar = (CL1+CL2)/2.0f;
	double CLbar7 = pow(CLbar,7);
	double CLbar25_7 = pow(CLbar/25.0, 7.0);
	double DeltaCL = CL2 - CL1;

	//delta H stuff
	double HL1 = degrees(atan2(colorLab1.z, aL1));
	double HL2 = degrees(atan2(colorLab2.z, aL2));
	if(HL1 < 0.0f)
		HL1 = HL1+360.0f;
	if(HL2 < 0.0f)
		HL2 = HL2+360.0f;
	double HLbar = (abs(HL1-HL2) > 180.0f) ? (HL1+HL2+360.0f)/2.0f : (HL1+HL2)/2.0f;
	double T = 1.0f-0.17f*cos(radians(HLbar-30.0f))+0.24f*cos(radians(2.0f*HLbar))+0.32f*cos(radians(3.0f*HLbar+6.0f))-0.20f*cos(radians(4.0f*HLbar-63.0f));
	double deltaHL = abs(HL2 - HL1);
	if(deltaHL <=180.0f)
		deltaHL = HL2 - HL1;
	else
	{
		if(HL2>HL1)
			deltaHL = HL2 - HL1 - 360.0f;
		else
			deltaHL = HL2 - HL1 + 360.0f;
	}
	double DeltaH = 2.0*sqrt(CL2*CL1)*sin(radians(deltaHL/2.0f));

	//Parameters for the calculation
	double SL = 1.0f+(0.015f*Lbar50squared)/sqrt(20.0f+Lbar50squared);
	double SC = 1.0f+0.045f*CLbar;
	double SH = 1.0f+0.015f*CLbar*T;
	double DeltaTetha = 30.0f*exp(-pow((HLbar-275.0f)/25.0f, 2));
	double RC = sqrt(CLbar25_7/(CLbar25_7+1.0f));
	double RT = -2.0f*RC*sin(radians(2.0f*DeltaTetha));

	//Result
	return sqrt(
	pow(DeltaL/(KL*SL),2)+pow(DeltaCL/(KC*SC),2)+pow(DeltaH/(KH*SH),2)+RT*(DeltaCL/(KC*SC))*DeltaH/(KH*SH));
}
double CPUColorConverter::DeltaEMixCIE1994(vec3 colorLab1, vec3 colorLab2)
{
		double kL = 1.0; double k1 = .045; double k2 = .015;
		colorLab2 =  colorLab1*0.25+colorLab2*0.75;
		vec3 colorLCH1 = CIELABLCH(colorLab1);
		vec3 colorLCH2 = CIELABLCH(colorLab2);
		//Calculating DeltaL
		double deltaL = colorLCH1.x - colorLCH2.x;
		//calculating DeltaC
		double deltaC = sqrt(pow(colorLCH2.y, 2.0)+pow(colorLCH2.z, 2.0))-sqrt(pow(colorLCH1.y, 2.0)+pow(colorLCH1.z, 2.0));
		//Calculating deltaH
		double DEab1976 = sqrt(pow(colorLab1.x-colorLab2.x, 2.0) + pow(colorLab1.y-colorLab2.y, 2.0) + pow(colorLab1.z-colorLab2.z, 2.0));
		double deltaH = sqrt(pow(DEab1976, 2.0)-pow(deltaL, 2.0)-pow(deltaC, 2.0));
		double DEab1994 =	pow((deltaL)/kL, 2.0)+
							pow((deltaC)/(1.0+k1*colorLCH1.y), 2.0)+
							pow((deltaH)/(1.0+k2*colorLCH1.y), 2.0);
		return (DEab1976 < 5.0 && deltaH >= 0.0) ? DEab1994 : DEab1976;

}
