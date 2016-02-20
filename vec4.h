#ifndef __VEC4H__
#define __VEC4H__
#include <float.h>
#include <iostream>
using namespace std;
#include <limits>
using std::numeric_limits;

//#include <iostream>
//  using std::ostream;
//	using std::cerr;
//	using std::cout;
//#include <string>
//	using std::string;
//#include <iomanip>
//	using std::setprecision;
//#include <cmath>
#include "vec3.h"
//#include <QtGLWidget>

//#define printme( e ) { cout << #e << " = " << ( e ) << "\n"; }

#define FLOAT_MAX 99999.0f
#define FLOAT_MIN -99999.0f

union vec4 {

        struct { double r; double g; double b; double a; };
        struct { double x; double y; double z; double w; };
        //struct { double x; double y; double z; double mass; };

	vec4(): x( 0 ), y( 0 ), z( 0 ), w( 0 )  {}
	vec4( const double scalar ): x( scalar ), y( scalar ), z( scalar ), w( scalar ) {}
    vec4( const double scalarr, const double scalarg, const double scalarb, const double scalara )
			: x( scalarr ), y( scalarg ), z( scalarb ), w( scalara ) {}	
    
   vec4( const vec3& v ): x(v.x), y(v.y), z(v.z){}
   vec4( const vec3& v, double w ): x(v.x), y(v.y), z(v.z) , w(1){}

   vec3 v3f()const {return vec3(x, y, z);}
   
   void set(double _x, double _y, double _z, double _w){x= _x; y = _y; z = _z; w = _w;}

	
	vec4 operator*( const vec4& right ) const {
		return vec4( r * right.r, g * right.g, b * right.b, a * right.a );
	} // end operator
	
	vec4 operator*( const double alpha ) const {
		return vec4( r * alpha, g * alpha, b * alpha, a * alpha );
	} // end operator

    vec4 operator/( const double alpha ) const {
		return vec4( r / alpha, g / alpha, b / alpha, a / alpha );
	} // end operator

	vec4 operator/( const vec4& right ) const {
		return vec4( r / right.r, g / right.g, b / right.b, a / right.a );
	} // end operator
	
	vec4 operator+( const vec4& right ) const {
		return vec4( r + right.r, g + right.g, b + right.b, a + right.a );
	} // end operator

	vec4 operator-( const vec4& right ) const {
		return vec4( r - right.r, g - right.g, b - right.b, a - right.a );
	} // end operator

	vec4 operator-() const {
		return vec4( -r, -g, -b, -a );
	} // end operator

    void operator+=( const vec4& right ) {
		r += right.r;
		g += right.g;
		b += right.b;
		a += right.a;
	} // end operator

    void operator-=( const vec4& right ) {
		r -= right.r;
		g -= right.g;
		b -= right.b;
		a -= right.a;
	} // end operator
	
    void operator-=( const double& right ) {
		r -= right;
		g -= right;
		b -= right;
		a -= right;
	} // end operator
	
    void operator*=( const vec4& right ) {
		r *= right.r;
		g *= right.g;
		b *= right.b;
		a *= right.a;
	} // end operator
	
    void operator*=( const double& right ) {
		r *= right;
		g *= right;
		b *= right;
		a *= right;
	} // end operator
	
	vec4 operator=( const double right ) {
		r = g= b= a = right;
		return *this;
	} // end operator

    double &operator[]( size_t i ) { return ( ( double * )( this ) )[ i ]; }
    const double &operator[]( size_t i ) const {  return ( ( double * )( this ) )[ i ]; }

	static vec4 cross( const vec4& v1, const vec4& v2){
		vec4 vCrossProduct;
		vCrossProduct.x =  v1.y * v2.z - v1.z * v2.y;
		vCrossProduct.y = -v1.x * v2.z + v1.z * v2.x;
		vCrossProduct.z =  v1.x * v2.y - v1.y * v2.x;
		vCrossProduct.w = 1;
		return vCrossProduct;
	}

	double length()const{return sqrt( pow(x,2) + pow(y,2) + pow(z,2) );}
	vec4 abs();
	
	vec4 normalize(){
		double aux = sqrt( pow(x,2) + pow(y,2) + pow(z,2) );
		x = x/aux; y = y/aux; z= z/aux; 
		return *this;
	}
//	static inline bool dot(vec4 a, vec4 b);
	static double dot( const vec4& v1,  const vec4& v2 ){
		return( v1.x * v2.x + v1.y * v2.y + v1.z * v2.z );
	}
	
	static double dot( const vec4& v1,  const vec3& v2 ){
		return( v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w  );
	}

//    void glVertex(){glVertex3f(x, y, z);}
//    void glNormal(){glNormal3f(x, y, z);}
//    void glVertexN(vec3 n){glNormal3f(n.x, n.y, n.z); glVertex3f(x, y, z);}
//    void glTranslate(){glTranslatef(x, y, z);}
//    void glVectors(){glVertex3f(0, 0, 0); glVertex3f(x, y, z);}
//    void glColor(){glColor4f(r, g, b, a);}

	/*string toString() const {
		return "( " + double2str( r ) + ", " + double2str( g )
			+ ", " +  double2str( b ) + ", " + double2str( a ) + " )";
	} // end function*/
}; // end class

ostream &operator<<( ostream &out, const vec4& f );
bool nearEqual( const double a, const double b );
bool nearEqual( const vec4& u, const vec4& v,
	const bool er = true,
	const bool eg = true,
	const bool eb = true,
	const bool ea = true );
vec4 pow(const vec4& number, const vec4& power);
vec4 pow(const vec4& number, const double power);
#endif
