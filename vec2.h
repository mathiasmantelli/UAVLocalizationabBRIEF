//-----------------------------------------------------------------------------
//           Name: vec2.h
//         Author: Vitor Jorge
//  Last Modified: 02/01/05
//    Description: OpenGL compatible utility class for a 2D vector of doubles
//                 NOTE: This class has been left unoptimized for readability.
//-----------------------------------------------------------------------------

#ifndef __VEC2H__
#define __VEC2H__


#include <float.h>
#include <iostream>
using namespace std;
#include "vec3.h"
union vec2 {

	struct { double r; double g;};
	struct { double x; double y;};

    vec2() { }
    vec2(double x_, double y_);
    vec2(const double scalar ) : x( scalar ), y( scalar ) {}
    vec2(const vec2& v): x( v.x ), y( v.y ) {}
    
    void set(double x_, double y_);
    double length(void);
    
    vec2 normalize( ){
		double fLength = length();
		x = x / fLength;
		y = y / fLength;
		return *this;
	}

	// Static utility methods
	static double distance(const vec2 &v1, const vec2 &v2);
    static double dot(const vec2 &v1,  const vec2 &v2 );
    static vec3 cross(const vec2 &v1, const vec2 &v2);
	static double length(const vec2 &v);

	//peseudo vector info
	//vec2 convert1DTo2DCoord(const size_t& coord, const size_t& width, const size_t& height, const size_t& numChannels);
	//size_t convert2DTo1DCoord(const vec2& p, const size_t& width, const size_t& height, const size_t& numChannels);
    
    // Operators...
    vec2 operator+ (const vec2& other)const;
    vec2 operator- (const vec2& other)const;
    vec2 operator/ (const vec2& other)const;
    vec2 operator* (const vec2& other)const;
    vec2 operator* (const double scalar);
    friend vec2 operator* (const double scalar, const vec2& other);
    
    vec2& operator = (const vec2& other);

    vec2 operator + () const;
    vec2 operator - () const;

	bool operator > (const double f) { return x > f && y >f; }
    
//________________________________________________________________LNN MDF

 	vec2 operator = ( const double right ) {
		r = g = right;
		return *this;
	} // end operator
	
	vec2 operator += (const vec3& other){
		r += other.r;
		g += other.g;
		return *this;
	}
	vec2 operator -= (const vec2& other){
		r -= other.r;
		g -= other.g;
		return *this;
	}
	bool operator==(const vec2& right)
	{
		if((sqrt(pow((right.x-this->x),2))<ALMOSTZERO) && (sqrt(pow((right.y-this->y),2))<ALMOSTZERO))
			return true;
		else
			return false;
	}
	bool operator!=(const vec2& right)
	{
		if((sqrt(pow((right.x-this->x),2))<ALMOSTZERO) && (sqrt(pow((right.y-this->y),2))<ALMOSTZERO))
			return false;
		else
			return true;
	}

	vec2 operator / (const double r);
	
    void rotateY(double fAnguloEmRadianos);
//    void glVertex()const{glVertex2f(x, y);}
//    void glTranslate(){glTranslatef(x, y, 0);}
//    void glVectors(){glVertex2f(0, 0); glVertex2f(x, y);}
    
    vec2 abs(){ return vec2(std::abs(x), std::abs(y)); }
	
//	vec2  MAX(const vec3& r){
//		vec2 result;
//        result.x = std::max(x, r.x); result.y = std::max(y, r.y);
//		return result;
//	}
//	vec2  MIN(const vec2& r){
//		vec2 result;
//		result.x = min(x, r.x); result.y = min(y, r.y);
//		return result;
//	}

	void  maxEq(const vec2& r){
		x = max(x, r.x); y = max(y, r.y);
	}
	void  minEq(const vec2& r){
		x = min(x, r.x); y = min(y, r.y);
	}
	void flipXandY(void){
		double temp = x;
		x=y; y=temp;
	}
};
//_____________________________________________________________OVERLOAD COUT
std::ostream &operator<<( std::ostream &out, const vec2& f );
double length(const vec2& u);
vec2 pow(const vec2& number, const vec2& power);
vec2 pow(const vec2& number, const double power);
//peseudo vector info
vec2 convert1DTo2DCoord(const int& coord, const int& width, const int& height, const int& numChannels);
int convert2DTo1DCoord(const vec2& p, const int& width, const int& height, const int& numChannels);

#endif // _vec2_H_
