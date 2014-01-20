/* --------------------------------------------------------------  */
/* (C)Copyright 2001,2007,                                         */
/* International Business Machines Corporation,                    */
/* Sony Computer Entertainment, Incorporated,                      */
/* Toshiba Corporation,                                            */
/*                                                                 */
/* All Rights Reserved.                                            */
/*                                                                 */
/* Redistribution and use in source and binary forms, with or      */
/* without modification, are permitted provided that the           */
/* following conditions are met:                                   */
/*                                                                 */
/* - Redistributions of source code must retain the above copyright*/
/*   notice, this list of conditions and the following disclaimer. */
/*                                                                 */
/* - Redistributions in binary form must reproduce the above       */
/*   copyright notice, this list of conditions and the following   */
/*   disclaimer in the documentation and/or other materials        */
/*   provided with the distribution.                               */
/*                                                                 */
/* - Neither the name of IBM Corporation nor the names of its      */
/*   contributors may be used to endorse or promote products       */
/*   derived from this software without specific prior written     */
/*   permission.                                                   */
/*                                                                 */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND          */
/* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,     */
/* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF        */
/* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE        */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR            */
/* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
/* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT    */
/* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)        */
/* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN       */
/* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR    */
/* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,  */
/* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              */
/* --------------------------------------------------------------  */
/* PROLOG END TAG zYx                                              */


#ifndef _MPM_MOD_EXP_H_
#define _MPM_MOD_EXP_H_		1

#include <spu_intrinsics.h>
#include "mpm_defines.h"
#include "mpm_mod_exp2.h"
#include "mpm_mod_exp2_2.h"
#include "mpm_div2.h"

/* mpm_mod_exp
 * -----------
 * Generic routine that computes modular exponentiation. This routine computes:
 *
 *	c = (b ^ e) % m
 *
 * where b, e, and m are large multi-precision numbers of msize, esize, 
 * and msize quadwords, respectively. The result, c, is of msize quadwords.
 *
 * This implementation uses a window-based modular exponentiation algorithm.
 * The maximum window size is specified by the input parameter k. 
 */

// Was:
// static vector unsigned int _two_pow_n_mpm_mod_exp[2*MPM_MAX_SIZE+1] = {((vector unsigned int) { 0,0,0,1})};
static vector unsigned int _two_pow_n_mpm_mod_exp[2*MPM_MAX_SIZE+1] = { { 0,0,0,0} };

static __inline void _mpm_mod_exp(vector unsigned int *c, const vector unsigned int *b, const vector unsigned int *e, int esize, const vector unsigned int *m, int msize, int k)
{
  int usize;
  vector unsigned int u[2*MPM_MAX_SIZE+1];
  vector unsigned int one = { 0, 0, 0, 1 };

  /* Pre-compute the constant u used by the fixed modulus reduction 
   * algorithm. 
   */
  _two_pow_n_mpm_mod_exp[0] = one;
  usize = 2*msize+1;
  _mpm_div2(u, _two_pow_n_mpm_mod_exp, usize, m, msize);
  _mpm_mod_exp2(c, b, e, esize, m, msize, k, &u[msize-1]);
}

static __inline void _mpm_mod_exp_2(vector unsigned int *c, const vector unsigned int *e, int esize, const vector unsigned int *m, int msize, int k)
{
  int usize;
  vector unsigned int u[2*MPM_MAX_SIZE+1];
  vector unsigned int one = { 0, 0, 0, 1 };

  /* Pre-compute the constant u used by the fixed modulus reduction 
   * algorithm. 
   */
  _two_pow_n_mpm_mod_exp[0] = one;
  usize = 2*msize+1;
  _mpm_div2(u, _two_pow_n_mpm_mod_exp, usize, m, msize);
  _mpm_mod_exp2_2(c, e, esize, m, msize, k, &u[msize-1]);
}

#endif /* _MPM_MOD_EXP_H_ */
