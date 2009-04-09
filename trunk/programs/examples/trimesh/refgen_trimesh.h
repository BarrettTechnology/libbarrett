/* This is a custom refgen that does trimeshes from
 * Solidworks-generated VRML 1.0 ascii files*/

#include <libbarrett/refgen.h>

#include <gsl/gsl_interp.h>

struct refgen_trimesh_triangle
{
   /* Indices of the three points */
   int p_index[3];

   /* For getting 3d position of a two-vector */
   gsl_vector * p0; /* 3x1 */
   gsl_matrix * ij; /* 3x2 */
   
   /* js matrix (one column each j) */
   gsl_matrix * js;

   /* For getting the heights from the three sides */
   gsl_vector * hs_const; /* 3x1 */
   gsl_matrix * hs_mult;  /* 3x2 */
   
   /* For converting to each neighbor's frame */
   struct refgen_trimesh_triangle * neighbor[3];
   gsl_vector * conv_const[3];
   gsl_matrix * conv_mult[3];
};

struct refgen_trimesh
{
   struct bt_refgen base;
   gsl_vector * cpos; /* current, saved */
   
   /* Collection of points */
   gsl_vector ** points;
   int points_num;
   
   /* Collection of triangles */
   struct refgen_trimesh_triangle ** triangles;
   int triangles_num;
   
   /* Current state */
   struct refgen_trimesh_triangle * cur;
   gsl_vector * pos; /* 2x1 */
   gsl_vector * hs;
   
   gsl_vector * temp;
   gsl_vector * guess;
   gsl_vector * start;
   
};

struct refgen_trimesh * refgen_trimesh_create(char * filename,gsl_vector * cpos);
