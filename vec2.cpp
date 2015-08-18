#include "vec2.h"
#include <limits>
#undef max
#undef min
using std::numeric_limits;

vec2::vec2( double x_, double y_){
    x = x_;
    y = y_;
}

void vec2::set( double x_, double y_){
    x = x_;
    y = y_;
}

double vec2::length( void ){
    return( (double)sqrt( x * x + y * y));
}

// Static utility methods...
double vec2::length(const vec2 &v){
    return sqrt( v.x * v.x + v.y * v.y);
}

double vec2::distance( const vec2 &v1,  const vec2 &v2  ){
    double dx = v1.x - v2.x;
    double dy = v1.y - v2.y;

    return (double)sqrt( dx * dx + dy * dy);
}

 double vec2::dot( const vec2 &v1,  const vec2 &v2 ){
    return( v1.x * v2.x + v1.y * v2.y);
}

 vec3 vec2::cross( const vec2 &v1,  const vec2 &v2 ){
    vec3 vCrossProduct;

    vCrossProduct.x =  0;
    vCrossProduct.y =  0;
    vCrossProduct.z =  v1.x * v2.y - v1.y * v2.x;

    return vCrossProduct;
}

// Operators...

vec2 vec2::operator + ( const vec2 &other )const
{
    vec2 vResult(0.0f, 0.0f);

    vResult.x = x + other.x;
    vResult.y = y + other.y;

    return vResult;
}

vec2 vec2::operator + ( void ) const{
    return *this;
}

vec2 vec2::operator - ( const vec2 &other )const
{
    vec2 vResult(0.0f, 0.0f);

    vResult.x = x - other.x;
    vResult.y = y - other.y;

    return vResult;
}

vec2 vec2::operator - (  ) const{
    vec2 vResult(-x, -y);
    return vResult;
}

vec2 vec2::operator * ( const vec2 &other )const{
    vec2 vResult(0.0f, 0.0f);

    vResult.x = x * other.x;
    vResult.y = y * other.y;

    return vResult;
}

vec2 vec2::operator * ( const double scalar ){
    vec2 vResult(0.0f, 0.0f);

    vResult.x = x * scalar;
    vResult.y = y * scalar;

    return vResult;
}

vec2 operator * ( const double scalar, const vec2 &other ){
    vec2 vResult(0.0f, 0.0f);

    vResult.x = other.x * scalar;
    vResult.y = other.y * scalar;

    return vResult;
}

vec2 vec2::operator / ( const vec2 &other )const
{
    vec2 vResult(0.0f, 0.0f);

    vResult.x = x / other.x;
    vResult.y = y / other.y;

    return vResult;
}

vec2& vec2::operator = ( const vec2 &other ){
    x = other.x;
    y = other.y;
    return *this;
}

//-----------------------------------------------------------------------------
// rotaciona o vetor 3D em torno do eixo Y
//-----------------------------------------------------------------------------
//void vec3::rotateY( double fAnguloEmRadianos) {
//  double fSeno, fCosseno;
//
//  fSeno    = sin(fAnguloEmRadianos);
//  fCosseno = cos(fAnguloEmRadianos);
//
//  x = x * fCosseno + z * fSeno;
//  z = z * fCosseno - x * fSeno;
//}

//-----------------------------------------------------------------------------
// rotaciona o vetor 3D em torno do eixo Y
//-----------------------------------------------------------------------------
//void vec3::rotate( double X, double Y, doubleZ, double alphaRad) {
	
//}
//-----------------------------------------------------------------------------OPERATORS
vec2 vec2::operator / (const double r){ 
	vec2 vResult(0.0f, 0.0f);

    vResult.x = x / r;
    vResult.y = y / r;
    
    return vResult;
}

//functions
double length(const vec2& u){
	return sqrt( pow(u.x,2) + pow(u.y,2) );
}
vec2 pow(const vec2& number, const vec2& power)
{
	return vec2(pow(number.x, power.x),pow(number.y, power.y));
}
vec2 pow(const vec2& number, const double power)
{
	return vec2(pow(number.x, power),pow(number.y, power));
}
vec2 convert1DTo2DCoord(const int& coord, const int& width, const int& height, const int& numChannels)
{
	if(coord>=width*height*numChannels || coord<0)
	{
		std::cerr << "Error the subscript is out of bounds to convert it to 2D vector.\n";
		//qDebug("Error: invalid 1D coordinate. ", coord%(width*numChannels)/numChannels," ",coord/(width*numChannels));
		return vec2( std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
	}
	return vec2(coord%(width*numChannels)/numChannels, coord/(width*numChannels));

}
int convert2DTo1DCoord(const vec2& p, const int& width, const int& height, const int& numChannels)
{
	if(p.x>=width || p.y >=height || p.x<0 || p.y < 0)
	{
		cout << "Error: invalid 2D coordinate, conversion to 1D impossible.\n";
		return std::numeric_limits<int>::max();
	}
	return size_t((p.y*width+p.x)*numChannels);
}
std::ostream &operator<<( std::ostream &out, const vec2& f ) {
	return out << "( " << f.x << ", " << f.y << ")\n";
} // end oeprator