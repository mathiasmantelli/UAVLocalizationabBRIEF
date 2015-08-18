#include "vec4.h"

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
vec4 vec4::abs(){
	vec4 u;
	u.x = fabs( x );
	u.y = fabs( y );
	u.z = fabs( z );
	u.w = fabs( w );
	return u;
} 
// end class vec4
// ----------------------------------------------------------------------------
//start functions
ostream &operator<<( ostream &out, const vec4& f ) {
	return out << "( " << f.r << ", " << f.g << ", " <<  f.b << ", " << f.a << " )";
} // end oeprator
bool nearEqual( const double a, const double b ) {
    const double erro = 0.1;
	return fabs( a - b ) <= erro;
} 
bool nearEqual( const vec4& u, const vec4& v,
	const bool er,
	const bool eg,
	const bool eb,
	const bool ea) {

	return
		( !er || nearEqual( u.x, v.x ) ) &&
		( !eg || nearEqual( u.y, v.y ) ) &&
		( !eb || nearEqual( u.z, v.z ) ) &&
		( !ea || nearEqual( u.w, v.w ) );
} 
bool lessEqual( const vec4& u, const vec4& v,
	const bool er = true,
	const bool eg = true,
	const bool eb = true,
	const bool ea = true ) {

	return
		( !er || u.x <= v.x ) &&
		( !eg || u.y <= v.y ) &&
		( !eb || u.z <= v.z ) &&
		( !ea || u.w <= v.w );
} 
vec4 pow(const vec4& number, const vec4& power)
{
	return vec4(pow(number.x, power.x),pow(number.y, power.y),pow(number.z, power.z),pow(number.w, power.w));
}
vec4 pow(const vec4& number, const double power)
{
	return vec4(pow(number.x, power),pow(number.y, power), pow(number.z, power), pow(number.w, power));
}
// end functions
// ----------------------------------------------------------------------------
