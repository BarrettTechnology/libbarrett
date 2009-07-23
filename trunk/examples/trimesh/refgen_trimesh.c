
#define _GNU_SOURCE /* For getline() #include <stdio.h>*/
#include <stdlib.h>
#include <stdio.h>
#undef _GNU_SOURCE

#include <string.h>
#include <math.h>

#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include <syslog.h>

#include "refgen_trimesh.h"
#include "libbarrett/gsl.h"

#define RADIUS (0.0050)
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


/* Make ij vectors for a triangle */
int mk_ij(gsl_vector * i, gsl_vector * j,
          gsl_vector * p_root, gsl_vector * p_i, gsl_vector * p_j)
{
   double val;
   
   /* i = hat(p_i-p_root) */
   gsl_vector_memcpy(i,p_i);
   gsl_vector_sub(i,p_root);
   val = gsl_blas_dnrm2(i);
   gsl_blas_dscal(1.0/val,i);
   
   /* j = hat((p_j-p_root) - dot(p_j-p_root,i) i) */
   gsl_vector_memcpy(j,p_j);
   gsl_vector_sub(j,p_root);
   gsl_blas_ddot(j,i,&val);
   gsl_blas_daxpy(-val,i,j);
   val = gsl_blas_dnrm2(j);
   gsl_blas_dscal(1.0/val,j);
   
   return 0;
}


struct edge
{
   struct refgen_trimesh_triangle * t1;
   int t1_side;
   struct refgen_trimesh_triangle * t2;
   int t2_side;
};

static void edge_add_triangle(struct edge *** edges, int i, int j,
                              struct refgen_trimesh_triangle * triangle,
                              int side_index)
{
   struct edge * edge;
   /* Get potential edge */
   edge = edges[i][j];
   if (!edge)
   {
      edge = (struct edge *) malloc(sizeof(struct edge));
      edge->t1 = 0;
      edge->t2 = 0;
      edges[i][j] = edge;
      edges[j][i] = edge;
   }
   if (!edge->t1)
   {
      edge->t1 = triangle;
      edge->t1_side = side_index;
   }
   else if (!edge->t2)
   {
      edge->t2 = triangle;
      edge->t2_side = side_index;
   }
   else
      syslog(LOG_ERR,"Error: Edge %d, %d has more than two attached triangles!",
              i, j );
}

/* Functions */
struct refgen_trimesh * refgen_trimesh_create(char * filename,gsl_vector * cpos)
{
   struct refgen_trimesh * r;
   FILE * fp;
   char * line;
   unsigned int line_len;
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
                  gsl_vector_set(point,0,b+0.5);
                  gsl_vector_set(point,1,a);
                  gsl_vector_set(point,2,c);
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
                  /* Note: order doesn't matter! */
                  triangle->p_index[0] = a;
                  triangle->p_index[1] = b;
                  triangle->p_index[2] = c;
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
   
   /* Make temporary vector */
   r->temp = gsl_vector_alloc(3);
   r->start = gsl_vector_alloc(3);
   
   r->pos = gsl_vector_alloc(2);
   r->guess = gsl_vector_alloc(2);
   r->hs = gsl_vector_alloc(3);
   
   /* For each triangle, generate ij, and the two temp ijs */
   for (i=0; i<r->triangles_num; i++)
   {
      struct refgen_trimesh_triangle * t;
      gsl_vector_view i_view;
      gsl_vector_view j_view;
      gsl_vector_view j0_view;
      gsl_vector_view j1_view;
      gsl_vector_view j2_view;
      
      t = r->triangles[i];
      
      /* Save the root point */
      t->p0 = r->points[t->p_index[0]];
      
      /* Make the ij matrix */
      t->ij = gsl_matrix_alloc(3,2);
      i_view = gsl_matrix_column(t->ij,0);
      j_view = gsl_matrix_column(t->ij,1);
      mk_ij(&i_view.vector, &j_view.vector,
            r->points[t->p_index[0]],
            r->points[t->p_index[1]],
            r->points[t->p_index[2]]);
      
      /* Make the js matrix */
      t->js = gsl_matrix_alloc(3,3);
      j0_view = gsl_matrix_column(t->js,0);
      j1_view = gsl_matrix_column(t->js,1);
      j2_view = gsl_matrix_column(t->js,2);
      
      /* -- Copy in the j0 vector */
      gsl_vector_memcpy(&j0_view.vector, &j_view.vector);
      
      /* -- Make the j1 vector */
      mk_ij(r->temp, &j1_view.vector,
            r->points[t->p_index[1]],
            r->points[t->p_index[2]],
            r->points[t->p_index[0]]);
      
      /* -- Make the j2 vector */
      mk_ij(r->temp, &j2_view.vector,
            r->points[t->p_index[2]],
            r->points[t->p_index[0]],
            r->points[t->p_index[1]]);
      
      /* Make the hs_const vector */
      t->hs_const = gsl_vector_alloc(3);
      
      /* -- Set the first element */
      gsl_vector_set(t->hs_const,0, 0.0);
      
      /* -- Set the second element */
      gsl_vector_memcpy(r->temp, r->points[t->p_index[0]]);
      gsl_vector_sub(r->temp,r->points[t->p_index[1]]);
      gsl_blas_ddot(r->temp,&j1_view.vector,gsl_vector_ptr(t->hs_const,1));
      
      /* -- Set the third element */
      gsl_vector_memcpy(r->temp, r->points[t->p_index[0]]);
      gsl_vector_sub(r->temp,r->points[t->p_index[2]]);
      gsl_blas_ddot(r->temp,&j2_view.vector,gsl_vector_ptr(t->hs_const,2));
      
      /* Make the hs_mult matrix */
      t->hs_mult = gsl_matrix_alloc(3,2);
      
      /* -- Set the first row */
      gsl_matrix_set(t->hs_mult,0,0, 0.0);
      gsl_matrix_set(t->hs_mult,0,1, 1.0);
      
      /* -- Set the second row */
      gsl_blas_ddot(&i_view.vector, &j1_view.vector, gsl_matrix_ptr(t->hs_mult,1,0));
      gsl_blas_ddot(&j_view.vector, &j1_view.vector, gsl_matrix_ptr(t->hs_mult,1,1));
      
      /* -- Set the third row */
      gsl_blas_ddot(&i_view.vector, &j2_view.vector, gsl_matrix_ptr(t->hs_mult,2,0));
      gsl_blas_ddot(&j_view.vector, &j2_view.vector, gsl_matrix_ptr(t->hs_mult,2,1));
   }
   
   /* Fill in the triangle-to-triangle links
    * by building an EDGE DATABASE */
   {
      struct edge *** edges;
      
      /* Zero the triangles' links to neighbor triangles
       * (this is what we'll be filling in here) */
      for (i=0; i<r->triangles_num; i++)
      {
         r->triangles[i]->neighbor[0] = 0;
         r->triangles[i]->neighbor[1] = 0;
         r->triangles[i]->neighbor[2] = 0;
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
         edge_add_triangle(edges, r->triangles[i]->p_index[0], r->triangles[i]->p_index[1],
                              r->triangles[i], 0);
         edge_add_triangle(edges, r->triangles[i]->p_index[1], r->triangles[i]->p_index[2],
                              r->triangles[i], 1);
         edge_add_triangle(edges, r->triangles[i]->p_index[2], r->triangles[i]->p_index[0],
                              r->triangles[i], 2);
      }
      
      /* Go through the edge database, linking triangles together */
      for (i=0; i<r->points_num; i++)
      for (j=i+1; j<r->points_num; j++)
      {
         struct edge * edge;
         
         /* Get the edge, make sure it's good */
         edge = edges[i][j];
         if (!edge)
            continue;
         if (!edge->t1 || !edge->t2)
         {
            syslog(LOG_ERR,"Error: Edge %d, %d has fewer than two attached triangles!",
                   i, j);
            continue;
         }

         /* Let t1 know about t2 */
         {
            gsl_vector * conv_const;
            gsl_matrix * conv_mult;
            
            /* Make the conv_const vector */
            conv_const = gsl_vector_alloc(2);
            
            /* conv_const = ij_2 ^T (p0_1 - p0_2) */
            gsl_vector_memcpy(r->temp, edge->t1->p0);
            gsl_vector_sub(r->temp, edge->t2->p0);
            gsl_blas_dgemv( CblasTrans, 1.0, edge->t2->ij, r->temp, 0.0, conv_const );
            
            /* Make the conv_mult vector */
            conv_mult = gsl_matrix_alloc(2,2);
            
            /* conv_mult = ij_2 ^T ij_1 */
            gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, edge->t2->ij, edge->t1->ij,
                           0.0, conv_mult);
            
            /* Set conv_const and conv_mult in t1 in the right place */
            edge->t1->neighbor[edge->t1_side] = edge->t2;
            edge->t1->conv_const[edge->t1_side] = conv_const;
            edge->t1->conv_mult[edge->t1_side] = conv_mult;
         }
         
         /* Let t2 know about t1 */
         {
            gsl_vector * conv_const;
            gsl_matrix * conv_mult;
            
            /* Make the conv_const vector */
            conv_const = gsl_vector_alloc(2);
            
            /* conv_const = ij_1 ^T (p0_2 - p0_1) */
            gsl_vector_memcpy(r->temp, edge->t2->p0);
            gsl_vector_sub(r->temp, edge->t1->p0);
            gsl_blas_dgemv( CblasTrans, 1.0, edge->t1->ij, r->temp, 0.0, conv_const );
            
            /* Make the conv_mult vector */
            conv_mult = gsl_matrix_alloc(2,2);
            
            /* conv_mult = ij_2 ^T ij_1 */
            gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, edge->t1->ij, edge->t2->ij,
                           0.0, conv_mult);
            
            /* Set conv_const and conv_mult in t1 in the right place */
            edge->t2->neighbor[edge->t2_side] = edge->t1;
            edge->t2->conv_const[edge->t2_side] = conv_const;
            edge->t2->conv_mult[edge->t2_side] = conv_mult;
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
      int j;
      for (j=0; j<3; j++)
      if (r->triangles[i]->neighbor[j])
      {
         gsl_vector_free(r->triangles[i]->conv_const[j]);
         gsl_matrix_free(r->triangles[i]->conv_mult[j]);
      }
      gsl_matrix_free(r->triangles[i]->ij);
      gsl_matrix_free(r->triangles[i]->js);
      gsl_vector_free(r->triangles[i]->hs_const);
      gsl_matrix_free(r->triangles[i]->hs_mult);
      free(r->triangles[i]);
   }
   free(r->triangles);
   
   gsl_vector_free(r->temp);
   gsl_vector_free(r->guess);
   gsl_vector_free(r->start);
   gsl_vector_free(r->pos);
   gsl_vector_free(r->hs);
   
   free(r);
   return 0;
}


/* Generic parameterization */
void func(struct refgen_trimesh_triangle * tri, gsl_vector * pos, gsl_vector * res)
{
   /* Start at the root point */
   gsl_vector_memcpy(res, tri->p0);
   
   /* Multiply in the ij component */
   gsl_blas_dgemv( CblasNoTrans, 1.0, tri->ij, pos, 1.0, res );
   
   return;
}

static int get_start(struct bt_refgen * base, gsl_vector ** start)
{
   struct refgen_trimesh * r = (struct refgen_trimesh *) base;
   
   /* Start at the first triangle, zero position */
   r->cur = r->triangles[0];
   gsl_vector_set_zero(r->pos);
   
   /* Use the first triangle's bottom point */
   func(r->cur, r->pos, r->start);
   (*start) = r->start;
   
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
   gsl_vector_set_zero(r->pos);
   return 0;
}

static int eval(struct bt_refgen * base, gsl_vector * ref)
{
   int i;
   double error;
   double new_left_error, new_right_error;
   double radius;
   struct refgen_trimesh * r = (struct refgen_trimesh *) base;
   /* We have r->cpos, the current position */
   
   /* Our initial best guess is the current position. */
   gsl_vector_memcpy(r->guess,r->pos);
   
   /* Start the binary search with +/- RADIUS */
   radius = RADIUS;
   
   /* Do a binary search through parameters (left,right) */
   for (i=0; i<BINARY_DEPTH; i++)
   {
      /* Evaluate current error */
      func(r->cur, r->guess, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      error = gsl_blas_dnrm2( r->temp );
      
      /* Test left */
      *(gsl_vector_ptr(r->guess,0)) += radius;
      func(r->cur, r->guess, r->temp);
      *(gsl_vector_ptr(r->guess,0)) -= radius;
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_left_error = gsl_blas_dnrm2( r->temp );
      
      /* Test right */
      *(gsl_vector_ptr(r->guess,1)) += radius;
      func(r->cur, r->guess, r->temp);
      *(gsl_vector_ptr(r->guess,1)) -= radius;
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_right_error = gsl_blas_dnrm2( r->temp );
      
      radius /= 2;
      
      /* Adjust parameters */
      if (new_left_error < error) *(gsl_vector_ptr(r->guess,0)) += radius;
      else                        *(gsl_vector_ptr(r->guess,0)) -= radius;
      
      if (new_right_error < error) *(gsl_vector_ptr(r->guess,1)) += radius;
      else                         *(gsl_vector_ptr(r->guess,1)) -= radius;
   }

   gsl_vector_memcpy(r->pos,r->guess);
   
   /* Calculate the new 3d reference */
   func(r->cur, r->pos, ref);

   /* Calculate hs */
   gsl_vector_memcpy(r->hs, r->cur->hs_const);
   gsl_blas_dgemv( CblasNoTrans, 1.0, r->cur->hs_mult, r->pos, 1.0, r->hs );

#if 0
   /* Eliminate positive (good) heights */
   for (i=0; i<3; i++)
      if (gsl_vector_get(r->hs,i)>0)
         gsl_vector_set(r->hs,i,0.0);
   
   /* Add in the correction vector to stay inside triangle */
   gsl_blas_dgemv( CblasNoTrans, -1.0, r->cur->js, r->hs, 1.0, ref );
#endif

   {
      int i;
      
      /* Get the smallest height */
      i = gsl_vector_min_index(r->hs);
      
      /* If the height is negative, switch triangles across side i! */
      if (gsl_vector_get(r->hs,i) < 0.0)
      {
         if (!r->cur->neighbor[i])
         {
            /* Error! going out of bounds ... */
            return 0;
         }
         /* Switch the position */
         gsl_vector_memcpy(r->guess, r->pos); /* guess is the old position */
         gsl_vector_memcpy(r->pos, r->cur->conv_const[i]);
         gsl_blas_dgemv( CblasNoTrans, 1.0, r->cur->conv_mult[i], r->guess, 1.0, r->pos );
         /* Switch the current triangle */
         r->cur = r->cur->neighbor[i];
         /* Re-calculate reference */
         func(r->cur, r->pos, ref);
      }
   }
   
   return 0;
}

static int trigger(struct bt_refgen * base)
{ return -1; }


