#include "vec3.h"

#include <float.h>
#include <iostream>
using namespace std;
#include <limits>
using std::numeric_limits;

vec3::vec3( double x_, double y_, double z_ ){
    x = x_;
    y = y_;
    z = z_;
}

void vec3::set( double x_, double y_, double z_ ){
    x = x_;
    y = y_;
    z = z_;
}

double vec3::length( void ){
    return( (double)sqrt( x * x + y * y + z * z ) );
}

vec3 vec3::znormal(){
	x = x/z;
	y = y/z;
	z = 1;
	return *this;
}

// Static utility methods...

double vec3::distance( const vec3 &v1,  const vec3 &v2  ){
    double dx = v1.x - v2.x;
    double dy = v1.y - v2.y;
    double dz = v1.z - v2.z;

    return sqrt( dx * dx + dy * dy + dz * dz );
}

 double vec3::dot( const vec3 &v1,  const vec3 &v2 ){
    return( v1.x * v2.x + v1.y * v2.y + v1.z * v2.z  );
}

 vec3 vec3::cross( const vec3 &v1,  const vec3 &v2 ){
    vec3 vCrossProduct;

    vCrossProduct.x =  v1.y * v2.z - v1.z * v2.y;
    vCrossProduct.y = -v1.x * v2.z + v1.z * v2.x;
    vCrossProduct.z =  v1.x * v2.y - v1.y * v2.x;

    return vCrossProduct;
}

// Operators...

vec3 vec3::operator + ( const vec3 &other )const
{
    vec3 vResult(0.0f, 0.0f, 0.0f);

    vResult.x = x + other.x;
    vResult.y = y + other.y;
    vResult.z = z + other.z;

    return vResult;
}

vec3 vec3::operator + ( void ) const{
    return *this;
}

vec3 vec3::operator - ( const vec3 &other )const
{
    vec3 vResult(0.0f, 0.0f, 0.0f);

    vResult.x = x - other.x;
    vResult.y = y - other.y;
    vResult.z = z - other.z;

    return vResult;
}

vec3 vec3::operator - (  ) const{
    vec3 vResult(-x, -y, -z);
    return vResult;
}

vec3 vec3::operator * ( const vec3 &other )const{
    vec3 vResult(0.0f, 0.0f, 0.0f);

    vResult.x = x * other.x;
    vResult.y = y * other.y;
    vResult.z = z * other.z;

    return vResult;
}

vec3 vec3::operator * ( const double scalar ){
    vec3 vResult(0.0f, 0.0f, 0.0f);

    vResult.x = x * scalar;
    vResult.y = y * scalar;
    vResult.z = z * scalar;

    return vResult;
}

vec3 operator * ( const double scalar, const vec3 &other ){
    vec3 vResult(0.0f, 0.0f, 0.0f);

    vResult.x = other.x * scalar;
    vResult.y = other.y * scalar;
    vResult.z = other.z * scalar;

    return vResult;
}

vec3 vec3::operator / ( const vec3 &other )const
{
    vec3 vResult(0.0f, 0.0f, 0.0f);

    vResult.x = x / other.x;
    vResult.y = y / other.y;
    vResult.z = z / other.z;

    return vResult;
}

vec3& vec3::operator = ( const vec3 &other ){
    x = other.x;
    y = other.y;
    z = other.z;

    return *this;
}

//-----------------------------------------------------------------------------
// rotaciona o vetor 3D em torno do eixo Y
//-----------------------------------------------------------------------------
void vec3::rotateY( double fAnguloEmRadianos) {
  double fSeno, fCosseno;

  fSeno    = sin(fAnguloEmRadianos);
  fCosseno = cos(fAnguloEmRadianos);

  x = x * fCosseno + z * fSeno;
  z = z * fCosseno - x * fSeno;
}

//-----------------------------------------------------------------------------
// rotaciona o vetor 3D em torno do eixo Y
//-----------------------------------------------------------------------------
//void vec3::rotate( double X, double Y, doubleZ, double alphaRad) {
	
//}
//-----------------------------------------------------------------------------OPERATORS
vec3 vec3::operator / (const double r){ 
    vec3 vResult(0.0, 0.0, 0.0);

    vResult.x = x / r;
    vResult.y = y / r;
    vResult.z = z / r;
    
    return vResult;
}
std::ostream &operator<<( std::ostream &out, const vec3& f ) {
	return out << "( " << f.x << ", " << f.y << ", " <<  f.z << " )";
} // end oeprator
double length(const vec3& u)
{
	return sqrt( pow(u.x,2) + pow(u.y,2) + pow(u.z,2) );
}
vec3 pow(const vec3& number, const vec3& power)
{
	return vec3(pow(number.x, power.x),pow(number.y, power.y),pow(number.z, power.z));
}
vec3 pow(const vec3& number, const double power)
{
	return vec3(pow(number.x, power),pow(number.y, power),pow(number.z, power));
}
