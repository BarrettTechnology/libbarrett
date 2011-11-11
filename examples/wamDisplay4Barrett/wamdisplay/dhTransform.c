/***************************************************************************
 *   Copyright (C) 2007 by Pedro Nobre                                     *
 *   pedrognobre@gmail.com                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <math.h>

#include "dhTransform.h"

void get_DHmatrix(gsl_matrix *A, double a, double alpha, double d, double theta){
  gsl_matrix_set_identity(A);
  
  gsl_matrix_set (A, 0, 0, cos(theta));
  gsl_matrix_set (A, 0, 1, (-sin(theta)*cos(alpha)));
  gsl_matrix_set (A, 0, 2, (sin(theta)*sin(alpha)));
  gsl_matrix_set (A, 0, 3, (a*cos(theta)));
  
  gsl_matrix_set (A, 1, 0, sin(theta));
  gsl_matrix_set (A, 1, 1, (cos(theta)*cos(alpha)));
  gsl_matrix_set (A, 1, 2, (-cos(theta)*sin(alpha)));
  gsl_matrix_set (A, 1, 3, (a*sin(theta)));
  
  gsl_matrix_set (A, 2, 1, (sin(alpha)));
  gsl_matrix_set (A, 2, 2, (cos(alpha)));
  gsl_matrix_set (A, 2, 3, d);
}

void get_T0END(gsl_matrix *result, int numDOF, DHparam *param){
  int i;
  gsl_matrix * transform = gsl_matrix_alloc (4, 4);
  gsl_matrix * temp = gsl_matrix_alloc (4, 4);

  gsl_matrix_set_identity(temp);

  for(i=0; i<numDOF ; i++){
    get_DHmatrix(transform, param[i].a, param[i].alpha, param[i].d, param[i].theta);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, temp, transform, 0.0, result);    
    gsl_matrix_memcpy (temp, result);
  }

  gsl_matrix_free (transform);
  gsl_matrix_free (temp);

}

void get_ENDtranslate(int numDOF, DHparam *param, double *x, double *y, double *z){
  gsl_matrix * m = gsl_matrix_alloc (4, 4);
  get_T0END(m, numDOF, param);
  *x=gsl_matrix_get (m, 0, 3);
  *y=gsl_matrix_get (m, 1, 3);
  *z=gsl_matrix_get (m, 2, 3);
}
