/* This is a custom refgen that does trimeshes from
 * Solidworks-generated VRML 1.0 ascii files*/

#include <libbarrett/refgen.h>

#include <gsl/gsl_interp.h>

enum refgen_trimesh_triangle_side
{
   REFGEN_TRIMESH_TRIANGLE_SIDE_LEFT,
   REFGEN_TRIMESH_TRIANGLE_SIDE_RIGHT,
   REFGEN_TRIMESH_TRIANGLE_SIDE_TOP
};

struct refgen_trimesh_triangle
{
   int pb_i;
   int pl_i;
   int pr_i;

   gsl_vector * pb; /* bottom */
   gsl_vector * pl; /* left */
   gsl_vector * pr; /* right */
   
   struct refgen_trimesh_triangle * tl; /* left coupled triangle */
   enum refgen_trimesh_triangle_side tl_side; /* side of the lct that borders */
   
   struct refgen_trimesh_triangle * tr; /* right coupled triangle */
   enum refgen_trimesh_triangle_side tr_side; /* side of the rct that borders */
   
   struct refgen_trimesh_triangle * tt; /* top coupled triangle */
   enum refgen_trimesh_triangle_side tt_side; /* side of the tct that borders */
   
   /* Unit vectors in the two directions */
   double v_left_len;
   double v_right_len;
   gsl_vector * v_left_unit;
   gsl_vector * v_right_unit;
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
   double left;
   double right;
   
   gsl_vector * temp;
   
};

struct refgen_trimesh * refgen_trimesh_create(char * filename,gsl_vector * cpos);

