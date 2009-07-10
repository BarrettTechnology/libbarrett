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




/*! \file dhTransform.h 
    \author Pedro Nobre <pedrognobre@gmail.com>
    
    \brief Computes generalized DH transform matrix
*/

#include <gsl/gsl_blas.h>

 #ifdef __cplusplus
 extern "C" {
 #endif

#ifndef DHTRANSFORM_H
#define DHTRANSFORM_H

typedef struct {
  double a;
  double alpha;
  double d;
  double theta;
}DHparam;

/*! \brief Computes DH generalized transform matrix to each frame

*/
void get_DHmatrix(
gsl_matrix *A,
double a,
double alpha,
double d,
double theta
);

/*! \brief Transform matrix from the base to end

*/
void get_T0END(gsl_matrix *result, int numDOF, DHparam *param);

/*! \brief Get the translation components

*/
void get_ENDtranslate(int numDOF, DHparam *param, double *x, double *y, double *z);

#endif

 #ifdef __cplusplus
 }
 #endif
