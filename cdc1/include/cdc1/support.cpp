#include "support.h"

////////////////////////////////////////////////////////////////////////////////////
int cdc_value_near(double val, double goal, double tol)
{
	// returns 1 if val > goal-tol  and val < goal+tol
	if (val < goal+tol && val > goal-tol)
	return 1 ;
	else
	return 0 ;
}
////////////////////////////////////////////////////////////////////////////////////
double cdc_mag(double vec[3])
{
	// returns the magnitude of a length 3 vector
	return (sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2])) ;
}
////////////////////////////////////////////////////////////////////////////////////
void cdc_cross(double ansvec[3], double vec1[3], double vec2[3])
{
	// inputs -
	//       vec1, vec2 - two length 3 vectors
	// outputs -
	//       ansvec - vec1 cross vec2
	ansvec[0] = vec1[1]*vec2[2] - vec1[2]*vec2[1] ;
	ansvec[1] = vec1[2]*vec2[0] - vec1[0]*vec2[2] ;
	ansvec[2] = vec1[0]*vec2[1] - vec2[0]*vec1[1] ;
}
////////////////////////////////////////////////////////////////////////////////////
double cdc_dot(double vec1[3], double vec2[3])
{
	// inputs - 
	//      vec1, vec2 - two length 3 vectors
	// outputs -
	//      none
	// return -
	//      returns vec1 dot vec2
	return(vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2]) ;
}

////////////////////////////////////////////////////////////////////////////////////
int cdc_sgn(double v) {
  return (v < 0) ? -1 : ((v > 0) ? 1 : 0);
}

////////////////////////////////////////////////////////////////////////////////////
int cdc_get_intersection_of_two_lines(double int_pt[3], double S1[3], double SOL1[3], double S2[3], double SOL2[3])
{
	// inputs -
	//       S1, SOL1 - coordinates of line 1  (note that S1 must be perpendicular to SOL1)
	//       S2, SOL2 - coordinates of line 2  (same condition as for line 1)
	// outputs -
	//       int_pt   - the point of intersection ; garbage value if the lines do not intersect or are parallel
	// returns 1 if lines do intersect, 0 if no intersection, and 999 if they intersect at infinity (parallel lines)
	
	int j ;
	double mag1, mag2, numer, denom, test1, temp[3], val3 ;
	mag1 = cdc_mag(S1) ;
	mag2 = cdc_mag(S2) ;
	
	for (j=0 ; j<3 ; ++j)
	{
		S1[j]   = S1[j]/mag1 ;
		SOL1[j] = SOL1[j]/mag1 ;
		S2[j]   = S2[j]/mag2 ;
		SOL2[j] = SOL2[j]/mag2 ;
	}
	
	double MM ;
	MM = cdc_dot(S1, SOL2) + cdc_dot(S2, SOL1) ;
	
	if (!cdc_value_near(MM, 0.0, 0.001))
	{
		// the lines don't intersect
		return 0 ;
	}
	
	test1 = cdc_dot(S1, S2) ;  // if this equals 1, the lines are parallel
	if (cdc_value_near(abs(test1), 1.0, 0.001))
	{
		return 999 ;
	}
	denom = 1-test1*test1 ;
	
	double term1[3], term2[3], term3[3] ;
	
	for (j=0 ; j<3 ; ++j)
	{
		temp[j] = -cdc_dot(S1,S2)*S1[j] ;
	}
	
	cdc_cross(term1, S2, SOL2) ;
	cdc_cross(term2, temp, SOL2) ;
	cdc_cross(term3, S1, SOL1) ;
	val3 = cdc_dot(term3, S2) ;
	
	for(j=0 ; j<3 ; ++j)
	{
		int_pt[j] = (term1[j] + term2[j] + val3*S2[j])/denom ;
	}
	return 1 ;
}
////////////////////////////////////////////////////////////////////////////////////