/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "geometry.h"

#define TEST1_LAT (45.0)
#define TEST1_LON (79.0)

#define LAT_MIN -90.0f
#define LAT_MAX 90.0f
#define LON_MIN 0.0f
#define LON_MAX 180.0f

#define N_TESTS (1000)

int rand_range(int lower, int upper) {
	return (rand() + lower) % (upper + 1);
}
extern const PT_T waypoints[];
float v_sinlat[164],v_coslat[164],v_lon[164],v_lat[164];
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	float dist, bearing, cur_pos_lat, cur_pos_lon, dist_opt, bearing_opt;
	char * name, * name_opt;
	struct timespec start, end, start_opt, end_opt;
	unsigned long diff, total=0, min=1234567890, diff_opt, total_opt=0, min_opt=1234567890;
	int n=0;

	
	cur_pos_lat = rand_range(LAT_MIN, LAT_MAX);
	cur_pos_lon = rand_range(LON_MIN, LON_MAX);


	for(int i=0;(strcmp(waypoints[i].Name, "END"));i++)
	{
		v_sinlat[i] = waypoints[i].SinLat;
		v_coslat[i] = waypoints[i].CosLat;
		v_lon[i] = waypoints[i].Lon;
		v_lat[i] = waypoints[i].Lat;
	}

	/*	printf("Current location is %f deg N, %f deg W\n", cur_pos_lat,
	       cur_pos_lon);
	*/
	for (n=0; n<N_TESTS; n++) {
	  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
	  Find_Nearest_Waypoint(cur_pos_lat, cur_pos_lon,
				&dist, &bearing, &name);
	  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
	  
	  diff = 1000000000 * (end.tv_sec - start.tv_sec) +
	    end.tv_nsec - start.tv_nsec;
	  //	  printf("%2d: %8lu ns\n", n, diff);
	  total += diff;
	  if (diff < min)
	    min = diff;

	  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start_opt);
	  Find_Nearest_Waypoint_opt(cur_pos_lat, cur_pos_lon,
				&dist_opt, &bearing_opt, &name_opt);
	  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end_opt);
	  
	  diff_opt = 1000000000 * (end_opt.tv_sec - start_opt.tv_sec) +
	    end_opt.tv_nsec - start_opt.tv_nsec;
	  //	  printf("%2d: %8lu ns\n", n, diff);
	  total_opt += diff_opt;
	  if (diff_opt < min_opt)
	    min_opt = diff_opt;

	}

	printf("Closest waypoint is %s. %f km away at bearing %f degrees\n",
	       name, dist, bearing);

	printf("Closest waypoint opt is %s. %f km away at bearing %f degrees\n",
	       name_opt, dist_opt, bearing_opt);


	printf("Average %10.3f us\n", total/(1000.0*N_TESTS));
	printf("Minimum %10.3f us\n",  min/1000.0);

	printf("Average_opt %10.3f us\n", total_opt/(1000.0*N_TESTS));
	printf("Minimum_opt %10.3f us\n",  min_opt/1000.0);

    printf("Distance difference: %f\n", dist - dist_opt);
    printf("Bearing difference: %f\n", bearing - bearing_opt);

	exit(0);
}

