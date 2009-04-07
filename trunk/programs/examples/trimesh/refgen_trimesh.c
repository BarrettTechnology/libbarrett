
#include <stdlib.h>

#include <string.h>
#include <math.h>

#define _GNU_SOURCE /* For getline() #include <stdio.h>*/
#include <stdio.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include <syslog.h>

#include "refgen_trimesh.h"
#include "libbarrett/gsl.h"

#define RADIUS_LEFT (0.0010)
#define RADIUS_RIGHT (0.0010)
#define BINARY_DEPTH (8)

static int destroy(struct bt_refgen * base);
static int get_start(struct bt_refgen * base, gsl_vector ** start);
static int get_total_time(struct bt_refgen * base, double * time);
static int get_num_points(struct bt_refgen * base, int * points);
static int start(struct bt_refgen * base);
static int eval(struct bt_refgen * base, gsl_vector * ref);
static int trigger(struct bt_refgen * base);
static const struct bt_refgen_type refgen_trimesh_type = {
   "trimesh",
   &destroy,
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval,
   &trigger
};
const struct bt_refgen_type * refgen_trimesh = &refgen_trimesh_type;

/* Generic parameterization */
void func(struct refgen_trimesh * r, double left, double right, gsl_vector * pos)
{
   /* Start at the bottom point */
   gsl_vector_memcpy(pos, r->cur->pb);
   
   /* Add in the left vector */
   gsl_blas_daxpy( left, r->cur->v_left_unit, pos );
   
   /* Add in the right vector */
   gsl_blas_daxpy( right, r->cur->v_right_unit, pos );
   
   return;
}

struct edge
{
   struct refgen_trimesh_triangle * t1;
   enum refgen_trimesh_triangle_side t1_side;
   
   struct refgen_trimesh_triangle * t2;
   enum refgen_trimesh_triangle_side t2_side;
};

/* Functions */
struct refgen_trimesh * refgen_trimesh_create(char * filename,gsl_vector * cpos)
{
   struct refgen_trimesh * r;
   FILE * fp;
   char * line;
   int line_len;
   int i, j;
   enum STATE {
      STATE_PREPOINTS,
      STATE_POINTS,
      STATE_PRETRIS,
      STATE_TRIS,
      STATE_END
   } state;
   
   /* Create */
   r = (struct refgen_trimesh *) malloc(sizeof(struct refgen_trimesh));
   if (!r) return 0;
   
   /* save the cartesian position,
    * set on refgen creation */
   r->cpos = cpos;
   
   /* Initialize */
   r->base.type = refgen_trimesh;
   r->points = 0;
   r->points_num = 0;
   r->triangles = 0;
   r->triangles_num = 0;
   r->cur = 0;
   r->left = 0.0;
   r->right = 0.0;
   
   /* Read from a file to fill points and triangles */
   fp = fopen(filename,"r");
   if (!fp)
   {
      syslog(LOG_ERR,"%s: Could not open file %s.",__func__,filename);
      free(r);
      return 0;
   }
   
   /* Read from the file */
   state = STATE_PREPOINTS;
   line = 0;
   line_len = 0;
   while (1)
   {
      int err;
      err = getline(&line,&line_len,fp);
      if (err == -1)
         break;
      switch (state)
      {
         case STATE_PREPOINTS:
            if (strncmp(line,"point [",strlen("point ["))==0)
               state = STATE_POINTS;
            break;
         case STATE_POINTS:
            {
               char * pointstr;
               int points_read;
               
               if (strncmp(line,"]",1)==0)
               {
                  state = STATE_PRETRIS;
                  break;
               }
               
               points_read = 0;
               while ((pointstr = strtok(points_read ? 0 : line, ",")))
               {
                  double a, b, c;
                  gsl_vector * point;
                  
                  points_read++;
                  if (sscanf(pointstr," %lf %lf %lf",&a,&b,&c) != 3)
                  {
                     break;
                  }
                  point = gsl_vector_alloc(3);
                  if (!point)
                  {
                     syslog(LOG_ERR,"%s: Out of memory.",__func__);
                     /* We should be better about freeing */
                     free(r);
                     return 0;
                  }
                  /* Move all the points the same way */
                  gsl_vector_set(point,0,10*a+0.5);
                  gsl_vector_set(point,1,10*b);
                  gsl_vector_set(point,2,10*c);
                  if (!r->points_num)
                     r->points = (gsl_vector **)
                                 malloc(sizeof(gsl_vector *));
                  else
                     r->points = (gsl_vector **)
                                 realloc(r->points, (r->points_num+1)*
                                         sizeof(gsl_vector *));
                  if (!r->points)
                  {
                     syslog(LOG_ERR,"%s: Out of memory.",__func__);
                     /* We should be better about freeing */
                     gsl_vector_free(point);
                     free(r);
                     return 0;
                  }
                  r->points[r->points_num] = point;
                  r->points_num++;
               }
            }
            break;
         case STATE_PRETRIS:
            if (strncmp(line,"coordIndex [",strlen("coordIndex ["))==0)
               state = STATE_TRIS;
            break;
         case STATE_TRIS:
            {
               char * trisstr;
               int tris_read;
               
               if (strncmp(line,"]",1)==0)
               {
                  state = STATE_END;
                  break;
               }
               
               tris_read = 0;
               while ((trisstr = strtok(tris_read ? 0 : line, "-")))
               {
                  int a, b, c;
                  struct refgen_trimesh_triangle * triangle;
                  
                  if (tris_read)
                     trisstr += 2;
                  if (strcmp(trisstr,"\n")==0)
                     break;
                  
                  tris_read++;
                  if (sscanf(trisstr," %d, %d, %d, ",&a,&b,&c) != 3)
                  {
                     break;
                  }
                  triangle = (struct refgen_trimesh_triangle *)
                             malloc(sizeof(struct refgen_trimesh_triangle));
                  if (!triangle)
                  {
                     syslog(LOG_ERR,"%s: Out of memory.",__func__);
                     /* We should be better about freeing */
                     free(r);
                     return 0;
                  }
                  if (   a >= r->points_num
                      || b >= r->points_num
                      || c >= r->points_num
                  ) {
                     syslog(LOG_ERR,"%s: Point %d, %d, or %d bigger than %d.",
                            __func__, a, b, c, r->points_num);
                     free(triangle);
                     continue;
                  }
                  triangle->pb_i = a;
                  triangle->pl_i = b;
                  triangle->pr_i = c;
                  triangle->pb = r->points[a];
                  triangle->pl = r->points[b];
                  triangle->pr = r->points[c];
                  if (!r->triangles_num)
                     r->triangles = (struct refgen_trimesh_triangle **)
                                 malloc(sizeof(struct refgen_trimesh_triangle *));
                  else
                     r->triangles = (struct refgen_trimesh_triangle **)
                                 realloc(r->triangles, (r->triangles_num+1)*
                                         sizeof(struct refgen_trimesh_triangle *));
                  if (!r->triangles)
                  {
                     syslog(LOG_ERR,"%s: Out of memory.",__func__);
                     /* We should be better about freeing */
                     free(triangle);
                     free(r);
                     return 0;
                  }
                  r->triangles[r->triangles_num] = triangle;
                  r->triangles_num++;
               }
            }
            break;
         case STATE_END:
            break;
      }
      if (state == STATE_END)
         break;
   }
   
   /* Fill in the triangle-to-triangle links
    * by building an EDGE DATABASE */
   {
      struct edge *** edges;
      
      /* Zero the triangles' links to adjacent triangles
       * (this is what we'll be filling in here) */
      for (i=0; i<r->triangles_num; i++)
      {
         r->triangles[i]->tl = 0;
         r->triangles[i]->tr = 0;
         r->triangles[i]->tt = 0;
      }
      
      /* Allocate space for weird lookup thing */
      edges = (struct edge ***) malloc( r->points_num * sizeof(struct edge **) );
      for (i=0; i<r->points_num; i++)
         edges[i] = (struct edge **) malloc( r->points_num * sizeof(struct edge *) );
      /* Make space for a bunch of edges */
      for (i=0; i<r->points_num; i++)
      for (j=i+1; j<r->points_num; j++)
      {
         edges[i][j] = 0;
         edges[j][i] = 0;
      }
      
      /* Add all the triangles to the edge database */
      for (i=0; i<r->triangles_num; i++)
      {
         struct edge * edge;
         /* Left edge */
         edge = edges[ r->triangles[i]->pl_i ][ r->triangles[i]->pb_i ];
         if (!edge)
         {
            edge = (struct edge *) malloc(sizeof(struct edge));
            edge->t1 = 0;
            edge->t2 = 0;
            edges[ r->triangles[i]->pl_i ][ r->triangles[i]->pb_i ] = edge;
            edges[ r->triangles[i]->pb_i ][ r->triangles[i]->pl_i ] = edge;
         }
         if (!edge->t1)
         {
            edge->t1 = r->triangles[i];
            edge->t1_side = REFGEN_TRIMESH_TRIANGLE_SIDE_LEFT;
         }
         else if (!edge->t2)
         {
            edge->t2 = r->triangles[i];
            edge->t2_side = REFGEN_TRIMESH_TRIANGLE_SIDE_LEFT;
         }
         else
            printf( "Error: Edge %d, %d has more than two attached triangles!\n",
                    r->triangles[i]->pl_i, r->triangles[i]->pb_i );
         /* Right edge */
         edge = edges[ r->triangles[i]->pr_i ][ r->triangles[i]->pb_i ];
         if (!edge)
         {
            edge = (struct edge *) malloc(sizeof(struct edge));
            edge->t1 = 0;
            edge->t2 = 0;
            edges[ r->triangles[i]->pr_i ][ r->triangles[i]->pb_i ] = edge;
            edges[ r->triangles[i]->pb_i ][ r->triangles[i]->pr_i ] = edge;
         }
         if (!edge->t1)
         {
            edge->t1 = r->triangles[i];
            edge->t1_side = REFGEN_TRIMESH_TRIANGLE_SIDE_RIGHT;
         }
         else if (!edge->t2)
         {
            edge->t2 = r->triangles[i];
            edge->t2_side = REFGEN_TRIMESH_TRIANGLE_SIDE_RIGHT;
         }
         else
            printf( "Error: Edge %d, %d has more than two attached triangles!\n",
                    r->triangles[i]->pl_i, r->triangles[i]->pb_i );
         /* Top edge */
         edge = edges[ r->triangles[i]->pl_i ][ r->triangles[i]->pr_i ];
         if (!edge)
         {
            edge = (struct edge *) malloc(sizeof(struct edge));
            edge->t1 = 0;
            edge->t2 = 0;
            edges[ r->triangles[i]->pl_i ][ r->triangles[i]->pr_i ] = edge;
            edges[ r->triangles[i]->pr_i ][ r->triangles[i]->pl_i ] = edge;
         }
         if (!edge->t1)
         {
            edge->t1 = r->triangles[i];
            edge->t1_side = REFGEN_TRIMESH_TRIANGLE_SIDE_TOP;
         }
         else if (!edge->t2)
         {
            edge->t2 = r->triangles[i];
            edge->t2_side = REFGEN_TRIMESH_TRIANGLE_SIDE_TOP;
         }
         else/* Start at the bottom point */
            printf( "Error: Edge %d, %d has more than two attached triangles!\n",
                    r->triangles[i]->pl_i, r->triangles[i]->pb_i );
      }
      
      /* Go through the edge database, linking triangles together */
      for (i=0; i<r->points_num; i++)
      for (j=i+1; j<r->points_num; j++)
      {
         struct edge * edge;
         edge = edges[i][j];
         if (!edge)
            continue;
         if (!edge->t1 || !edge->t2)
         {
            printf( "Error: Edge %d, %d has fewer than two attached triangles!\n",
                   i, j);
            continue;
         }
         /* Let t1 know about t2 */
         switch (edge->t1_side)
         {
            case REFGEN_TRIMESH_TRIANGLE_SIDE_LEFT:
               edge->t1->tl = edge->t2;
               edge->t1->tl_side = edge->t2_side;
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_RIGHT:
               edge->t1->tr = edge->t2;
               edge->t1->tr_side = edge->t2_side;
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_TOP:
               edge->t1->tt = edge->t2;
               edge->t1->tt_side = edge->t2_side;
               break;
         }
         /* Let t2 know about t1 */
         switch (edge->t2_side)
         {
            case REFGEN_TRIMESH_TRIANGLE_SIDE_LEFT:
               edge->t2->tl = edge->t1;
               edge->t2->tl_side = edge->t1_side;
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_RIGHT:
               edge->t2->tr = edge->t1;
               edge->t2->tr_side = edge->t1_side;
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_TOP:
               edge->t2->tt = edge->t1;
               edge->t2->tt_side = edge->t1_side;
               break;
         }
      }
      
      /* De-allocate the edge database */
      for (i=0; i<r->points_num; i++)
      for (j=i+1; j<r->points_num; j++)
         free(edges[i][j]);
      for (i=0; i<r->points_num; i++)
         free(edges[i]);
      free(edges);
   }
   
   /* Make temporary vector */
   r->temp = gsl_vector_alloc(3);
   
   /* Compute triangle unit vectors */
   for (i=0; i<r->triangles_num; i++)
   {
      struct refgen_trimesh_triangle * t;
      t = r->triangles[i];
      
      t->v_left_unit = gsl_vector_alloc(3);
      gsl_vector_memcpy(t->v_left_unit, t->pl);
      gsl_vector_sub(t->v_left_unit, t->pb);
      t->v_left_len = gsl_blas_dnrm2(t->v_left_unit);
      gsl_blas_dscal(1.0/t->v_left_len, t->v_left_unit);
      
      t->v_right_unit = gsl_vector_alloc(3);
      gsl_vector_memcpy(t->v_right_unit, t->pr);
      gsl_vector_sub(t->v_right_unit, t->pb);
      t->v_right_len = gsl_blas_dnrm2(t->v_right_unit);
      gsl_blas_dscal(1.0/t->v_right_len, t->v_right_unit);
   }
   
   /* OK, triangles should be done! */
   r->cur = 0;
   r->left = 0.0;
   r->right = 0.0;
   
   return r;
}

static int destroy(struct bt_refgen * base)
{
   int i;
   struct refgen_trimesh * r = (struct refgen_trimesh *) base;
   
   /* Free points */
   for (i=0; i<r->points_num; i++)
   {
      gsl_vector_free(r->points[i]);
   }
   free(r->points);
   
   /* Free triangles */
   for (i=0; i<r->triangles_num; i++)
   {
      gsl_vector_free(r->triangles[i]->v_left_unit);
      gsl_vector_free(r->triangles[i]->v_right_unit);
      free(r->triangles[i]);
   }
   free(r->triangles);
   
   gsl_vector_free(r->temp);
   
   free(r);
   return 0;
}

static int get_start(struct bt_refgen * base, gsl_vector ** start)
{
   struct refgen_trimesh * r = (struct refgen_trimesh *) base;
   
   /* Use the first triangle's bottom point */
   (*start) = r->triangles[0]->pb;
   
   return 0;
}

static int get_total_time(struct bt_refgen * base, double * time)
{ return 0; }
static int get_num_points(struct bt_refgen * base, int * points)
{ return 0; }

static int start(struct bt_refgen * base)
{
   struct refgen_trimesh * r = (struct refgen_trimesh *) base;
   r->cur = r->triangles[0];
   r->left = 0.0;
   r->right = 0.0;
}

static int eval(struct bt_refgen * base, gsl_vector * ref)
{
   int i;
   double left, right;
   double error;
   double new_left_error, new_right_error;
   double radius_left, radius_right;
   double rad;
   struct refgen_trimesh * r = (struct refgen_trimesh *) base;
   /* We have r->cpos, the current position */
   
   /* Our initial best guess is left, right. */
   left = r->left;
   right = r->right;
   
   radius_left = RADIUS_LEFT;
   radius_right = RADIUS_RIGHT;
   
   /* Do a binary search through parameters (left,right) */
   for (i=0; i<BINARY_DEPTH; i++)
   {
      /* Evaluate current error */
      func(r,left,right, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      error = gsl_blas_dnrm2( r->temp );
      
      /* Test left */
      func(r,left+radius_left,right, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_left_error = gsl_blas_dnrm2( r->temp );
      
      /* Test right */
      func(r,left,right+radius_right, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_right_error = gsl_blas_dnrm2( r->temp );
      
      radius_left /= 2;
      radius_right /= 2;
      
      /* Adjust parameters */
      if (new_left_error < error) left += radius_left;
      else                        left -= radius_left;
      
      if (new_right_error < error) right += radius_right;
      else                         right -= radius_right;
   }
   
   /* If we've gone over an edge, switch triangles */
   {
      struct refgen_trimesh_triangle * new;
      double l_perc, r_perc;
      double new_l_perc, new_r_perc;
      l_perc = left / r->cur->v_left_len;
      r_perc = right / r->cur->v_right_len;
      new = 0;
      if (l_perc < 0)
      {
         /* We've gone over the right edge */
         new = r->cur->tr;
         switch (r->cur->tr_side)
         {
            case REFGEN_TRIMESH_TRIANGLE_SIDE_LEFT:
               new_l_perc = r_perc;
               new_r_perc = 0; /* This is far from ideal */
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_RIGHT:
               new_r_perc = 1 - r_perc;
               new_l_perc = 0; /* This is far from ideal */
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_TOP:
               new_r_perc = r_perc;
               new_l_perc = 1 - r_perc;
               break;
         }
      }
      else if (r_perc < 0)
      {
         /* We've gone over the left edge */
         new = r->cur->tl;
         switch (r->cur->tl_side)
         {
            case REFGEN_TRIMESH_TRIANGLE_SIDE_RIGHT:
               new_r_perc = l_perc;
               new_l_perc = 0;
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_LEFT:
               new_l_perc = 1 - l_perc;
               new_r_perc = 0;
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_TOP:
               new_l_perc = l_perc;
               new_r_perc = 1 - l_perc;
               break;
         }
      }
      else if ( l_perc + r_perc > 1)
      {
         /* We've gone over the top edge */
         new = r->cur->tt;
         switch (r->cur->tt_side)
         {
            case REFGEN_TRIMESH_TRIANGLE_SIDE_TOP:
               new_l_perc = r_perc;
               new_r_perc = 1 - r_perc;
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_RIGHT:
               new_l_perc = 0;
               new_r_perc = r_perc;
               break;
            case REFGEN_TRIMESH_TRIANGLE_SIDE_LEFT:
               new_l_perc = l_perc;
               new_r_perc = 0;
               break;
         }
      }
      /* If changed, make the switch */
      if (new)
      {
         r->cur = new;
         left = new_l_perc * new->v_left_len;
         right = new_r_perc * new->v_right_len;
      }
   }
   
   /* Calculate the new values*/
   func(r,left,right,ref);
   
   /* Save the new parameters */
   r->left = left;
   r->right = right;
   
   return 0;
}

static int trigger(struct bt_refgen * base)
{ return -1; }


