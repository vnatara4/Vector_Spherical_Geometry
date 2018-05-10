#include "geometry.h"
#include <math.h>
#include <string.h>
#include <arm_neon.h>
#include <arm_acle.h>

#define PI 3.14159265f
#define PI_OVER_180 (0.017453293f) // (3.1415927/180.0)



extern const PT_T waypoints[];
extern float v_lat[164],v_lon[164],v_sinlat[164],v_coslat[164];
float32x4_t vec_cos_fn(float32x4_t);

#define VER 12 

/*
10: radians in table, precalc'd sin, cos
11: Calc_Closeness
12: Don't do bearing
*/

// Table holds precalculated sin/cos for p2. Table Lat/Lon values are in radians

#if VER==10
float Calc_Distance( PT_T * p1,  const PT_T * p2) { 
  // calculates distance in kilometers between locations (represented in radians)

  return acosf(p1->SinLat * p2->SinLat + 
	       p1->CosLat * p2->CosLat*
	       cosf(p2->Lon - p1->Lon))* 6371;
}
#endif

float Calc_Bearing( PT_T * p1,  const PT_T * p2){
  // calculates bearing in degrees between locations (represented in radians)	
  float term1, term2;
  float angle;
	
  term1 = sinf(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat - 
    p1->SinLat*p2->CosLat*cosf(p1->Lon - p2->Lon);
  angle = atan2f(term1, term2) * (180/PI);
  if (angle < 0.0)
    angle += 360;
  return angle;
}

#if (VER==11) || (VER==12)
float Calc_Closeness( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat + 
    p1->CosLat * p2->CosLat*
    cosf(p2->Lon - p1->Lon);
}
#endif


void Find_Nearest_Waypoint(float cur_pos_lat, float cur_pos_lon, float * distance, float * bearing, 
			   char  * * name) {
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees
		
  int i=0, closest_i=0;
  PT_T ref;
  float d, b, c, max_c=0;

  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';

  ref.Lat = cur_pos_lat*PI/180;
  ref.Lon = cur_pos_lon*PI/180;
  ref.SinLat = sinf(ref.Lat);
  ref.CosLat = cosf(ref.Lat);
		
  strcpy(ref.Name, "Reference");

  while (strcmp(waypoints[i].Name, "END")) {
#if VER==10
    d = Calc_Distance(&ref, &(waypoints[i]) );
    // if we found a closer waypoint, remember it and display it
    if (d<closest_d) {
      closest_d = d;
      closest_i = i;
    }
#else
    c = Calc_Closeness(&ref, &(waypoints[i]) );
    if (c>max_c) {
      max_c = c;
      closest_i = i;
    }
#endif

#if (VER==10) || (VER==11)
    b = Calc_Bearing(&ref, &(waypoints[i]) ); // Deletable!
#endif

    i++;
  }

  // Finish calcuations for the closest point

#if VER==10
  d = closest_d; 
#else
  d = acosf(max_c)*6371; // finish distance calcuation
#endif	

#if VER==12
  b = Calc_Bearing(&ref, &(waypoints[closest_i]) );
#endif

  // return information to calling function about closest waypoint 
  *distance = d;
  *bearing = b;
  *name = (char * ) (waypoints[closest_i].Name);
}


float32x4_t vec_cos_fn(float32x4_t val)
{
  float32x4_t c1,c2,c3,c4,c5,c6,c7,x2; 

  c1=vdupq_n_f32(0.99999999999925182);
  c2=vdupq_n_f32(-0.49999999997024012);
  c3=vdupq_n_f32(0.041666666473384543);
  c4=vdupq_n_f32(-0.001388888418000423);
  c5=vdupq_n_f32(0.0000248010406484558);
  c6=vdupq_n_f32(-0.0000002752469638432);
  c7=vdupq_n_f32(0.0000000019907856854);
  x2=vmulq_f32(val,val);

  return vmlaq_f32(c1,x2,vmlaq_f32(c2,x2,vmlaq_f32(c3,x2,vmlaq_f32(c4,x2,vmlaq_f32(c5,x2,vmlaq_f32(c6,c7,x2))))));
}

void Find_Nearest_Waypoint_opt(float cur_pos_lat, float cur_pos_lon, float * distance, float * bearing, 
			   char  * * name) 
{
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees
		
  int closest_i=0, i = 0, j = 0;
  int q=0;
  PT_T ref;
  float d, b;
  float32x4_t vec_sinlat;
  float32x4_t vec_coslat;
  float32x4_t vec_lon;
  float32x4_t vec_close;
  float32x4_t cos_vec;
  
  float max_val = 0.0;
  float current_max = -1e30;
  float32x4_t vec_closemax;
  float32x2_t v2_zero = vdup_n_f32(0.0);
  float32x4_t vec_refsinlat,vec_refcoslat,vec_reflon;
  
  float32x4_t vec_a,vec_b,vec_c,vec_d;
  float32x2_t v2_u, v2_l;
  
  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';

  ref.Lat = cur_pos_lat*PI/180;
  ref.Lon = cur_pos_lon*PI/180;
  ref.SinLat = sinf(ref.Lat);
  ref.CosLat = cosf(ref.Lat);
  
  vec_refsinlat = vld1q_dup_f32(&ref.SinLat);
  vec_refcoslat = vld1q_dup_f32(&ref.CosLat);
  vec_reflon = vld1q_dup_f32(&ref.Lon);
  
  vec_closemax = vld1q_dup_f32(&max_val);
  

  for (i=0,j=0; j<41; i+=4,j++ )
  {
  	vec_sinlat = vld1q_f32 (&v_sinlat[i]);	
  	vec_coslat = vld1q_f32 (&v_coslat[i]);
  	vec_lon = vld1q_f32 (&v_lon[i]);
  	
  	vec_a = vmulq_f32(vec_sinlat , vec_refsinlat);
  	vec_b = vmulq_f32(vec_coslat , vec_refcoslat);
  	vec_c = vsubq_f32(vec_lon , vec_reflon);
  	
  	cos_vec = vec_cos_fn(vec_c);
  	
  	vec_d = vmulq_f32(vec_b, cos_vec);
  	
  	vec_close = vaddq_f32 (vec_a,vec_d);
  	
  	vec_closemax = vmaxq_f32 (vec_close, vec_closemax);
  	
    v2_u = vget_high_f32(vec_closemax);
    v2_l = vget_low_f32(vec_closemax);
    v2_u = vpmax_f32(v2_u, v2_l);
    v2_u = vpmax_f32(v2_u, v2_zero);
    current_max = vget_lane_f32(v2_u, 0);
    
    if(current_max == vec_closemax[0])
    	q = 0;
    else if(current_max == vec_closemax[1])
    	q = 1;
    else if(current_max == vec_closemax[2])
    	q = 2;
    else if(current_max == vec_closemax[3])
    	q = 3;
    	
   if(current_max > max_val)
   {
     max_val = current_max;	
     closest_i = ( j * 4) + q;
   } 
 }

#if VER==10
  d = closest_d; 
#else
  d = acosf(max_val)*6371; // finish distance calcuation
#endif	

#if VER==12
  b = Calc_Bearing(&ref, &(waypoints[closest_i]) );
#endif

  *distance = d;
  *bearing = b;
  *name = (char * ) (waypoints[closest_i].Name);
}


