//-----------------------------------------------------------------------------
//           Name: vec3.h
//         Author: Kevin Harris
//  Last Modified: 02/01/05
//    Description: OpenGL compatible utility class for a 3D vector of doubles
//                 NOTE: This class has been left unoptimized for readability.
//-----------------------------------------------------------------------------

#ifndef __VEC3_H__
#define __VEC3_H__


#include <cmath>
#include <algorithm>
#include <memory.h>
#include <ostream>

#ifndef PI
#define PI 	3.1415926535897932384626433832795
#endif

#ifndef ALMOSTZERO
#define ALMOSTZERO 	0.00000000000000000000000000000001
#endif

static double degrees(double rad){
    return rad*180.0f/PI;
}
static double radians(double degrees){
    return degrees*PI/180.0f;
}

union vec3{

		struct { double r; double g; double b;};
		struct { double x; double y; double z;};

    vec3() { }
    vec3(double x_, double y_, double z_);
    vec3(const double scalar ) : x( scalar ), y( scalar ), z( scalar ) {}
    vec3(const vec3& v): x( v.x ), y( v.y ), z( v.z ) {}
    
    void set(double x_, double y_, double z_);
    double length(void);
    
    vec3 normalize( ){
		double fLength = length();
		x = x / fLength;
		y = y / fLength;
		z = z / fLength;
		return *this;
	}
	
	vec3 znormal();

    // Static utility methods
    static double distance(const vec3 &v1, const vec3 &v2);
    static double dot(const vec3 &v1,  const vec3 &v2 );
    static vec3 cross(const vec3 &v1, const vec3 &v2);
    
    // Operators...
    vec3 operator+ (const vec3& other)const;
    vec3 operator- (const vec3& other)const;
    vec3 operator/ (const vec3& other)const;
    vec3 operator* (const vec3& other)const;
    vec3 operator* (const double scalar);
    friend vec3 operator* (const double scalar, const vec3& other);
    
    vec3& operator = (const vec3& other);

    vec3 operator + () const;
    vec3 operator - () const;

	bool operator > (const double f) { return x > f && y >f && z > f; }
    
//________________________________________________________________LNN MDF

 	vec3 operator = ( const double right ) {
		r = g = b = right;
		return *this;
	} // end operator
	
	vec3 operator += (const vec3& other){
		r += other.r;
		g += other.g;
		b += other.b;
		return *this;
	}
	vec3 operator -= (const vec3& other){
		r -= other.r;
		g -= other.g;
		b -= other.b;
		return *this;
	}

    bool operator == (const vec3& other){
        if(r==other.r && g==other.g && b==other.b)
            return true;
        return false;
    }
    bool operator != (const vec3& other){
        if(r!=other.r && g!=other.g && b!=other.b)
            return true;
        return false;
    }
	vec3 operator / (const double r);
	
    void rotateY(double fAnguloEmRadianos);
//    void glVertex()const{glVertex3f(x, y, z);}
//    void glVertexN(vec3 n){glNormal3f(n.x, n.y, n.z); glVertex3f(x, y, z);}
//	void glNormal(){glNormal3f(x, y, z);}
//    void glTranslate(){glTranslatef(x, y, z);}
//    void glVectors(){glVertex3f(0, 0, 0); glVertex3f(x, y, z);}

	vec3 inv(){ return vec3(x, y, -z);}
    
    vec3 abs(){ return vec3(std::abs(x), std::abs(y), std::abs(z)); }
	
//	vec3  MAX(const vec3& r){
//		vec3 result;
//        result.x = std::max(x, r.x); result.y = std::max(y, r.y); result.z = std::max(z, r.z);
//		return result;
//	}
//	vec3  MIN(const vec3& r){
//		vec3 result;
//        result.x = std::min(this->x, r.x); result.y = std::min(y, r.y); result.z = std::min(z, r.z);
//		return result;
//	}

	void  maxEq(const vec3& r){
        x = std::max(x, r.x); y = std::max(y, r.y); z = std::max(z, r.z);
	}
	void  minEq(const vec3& r){
        x = std::min(x, r.x); y = std::min(y, r.y); z = std::min(z, r.z);
	}
};
//_____________________________________________________________OVERLOAD COUT
std::ostream &operator<<( std::ostream &out, const vec3& f );
// ----------------------------------------------------------------------------
double length(const vec3& u);
vec3 pow(const vec3& number, const vec3& power);
vec3 pow(const vec3& number, const double power);
#endif // _vec3_H_
