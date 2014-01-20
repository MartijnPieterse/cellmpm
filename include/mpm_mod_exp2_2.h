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


#ifndef _MPM_MOD_EXP2_2_H_
#define _MPM_MOD_EXP2_2_H_		1


#include <spu_intrinsics.h>
#include "mpm_defines.h"
#include "mpm_mul.h"
#include "mpm_square.h"
#include "mpm_fixed_mod_reduction.h"

/* mpm_mod_exp2
 * ------------
 * Generic routine that computes modular exponentiation. This routine computes:
 *
 *	c = (b ^ e) % m
 *
 * where b, e, and m are large multi-precision numbers of msize, esize, 
 * and msize quadwords, respectively. The result, c, is of msize quadwords.
 *
 * This implementation uses a window-based modular exponentiation algorithm.
 * The maximum window size is specified by the input parameter k. 
 *
 * The basic algorithn is:
 *
 * 1.  s = (b*b) % m
 * 2.  T[1] = b
 * 3.  for i=1 to 2**(k-1) - 1 do
 *     3.1  T[2*i+1] = s * T[2*i-1] % m
 *     endfor
 * 4.  c = 1
 * 5.  h = most significant non-zero bit of e
 * 6.  while h >= 0
 *     6.1  if bit h of e is 0 then
 *          6.1.1  c = (c * c) % m  
 *          6.1.2  h -= 1
 *          else
 *          6.1.3  find longest bitstring z up to k bits whose lsb is 1
 *          6.1.4  for i = 1 to z do
 *                 6.1.4.1  c = (c*c) % m
 *                 endfor
 *          6.1.5  c = (c * T[bitstring]) % m
 *          endif
 * 7.  return c
 *
 * Note: The T array has been compressed to contain only the odd values since the window 
 *       must have both the msb and lsb equal to 1.
 *
 *	 The parameter k specifies the window size to be applied. This number must be 
 *       1 to MPM_MOD_EXP_MAX_K.
 */

static __inline void _mpm_mod_exp2_2(vector unsigned int *c, const vector unsigned int *e, int esize, const vector unsigned int *m, int msize, int k, vector unsigned int *u)
{
  int i;
  int h, T_idx, idx, window, bits_left;
  unsigned int shift;

  vector unsigned int window_size0, window_size1, selector, window_lsb;
  vector unsigned int qw0, qw1, qw;
//  vector unsigned int *ptr;
  vector unsigned int p[2*MPM_MAX_SIZE];		/* generic products */
//  vector unsigned int s[MPM_MAX_SIZE];
//  vector unsigned int T[1 << (MPM_MOD_EXP_MAX_K-1)][MPM_MAX_SIZE]; 

  window_lsb = spu_rlmask(spu_splats((unsigned int)0x80000000), 1-k);

  /* 1.  s = b*b % m
   */
//  _mpm_square(p, b, msize);
//  _mpm_fixed_mod_reduction(s, p, m, u, msize);

//  p[0] = ((vector unsigned int) { 0,0,0,0});
//  p[1] = ((vector unsigned int) { 0,0,0,0});
//  p[2] = ((vector unsigned int) { 0,0,0,0});
//  p[3] = ((vector unsigned int) { 0,0,0,0});
//  p[4] = ((vector unsigned int) { 0,0,0,0});
//  p[5] = ((vector unsigned int) { 0,0,0,4});

//  s[0] = ((vector unsigned int) { 0,0,0,0});
//  s[1] = ((vector unsigned int) { 0,0,0,0});
//  s[2] = ((vector unsigned int) { 0,0,0,4});
  
  /* 2. T[1] = b
   * 4. c = 1
   */
//  for (i=0, ptr=&T[0][0]; i<msize-1; i++) {
//    *ptr++ = b[i];
//    c[i] = spu_splats((unsigned int)0);
//  }
//  *ptr = b[i];
  c[0] = ((vector unsigned int) { 0, 0, 0, 0});
  c[1] = ((vector unsigned int) { 0, 0, 0, 0});
  c[2] = ((vector unsigned int) { 0, 0, 0, 1});
  
  /* 3.  for i=1 to 2**(k-1) - 1 do
   *     3.1  T[2*i+1] = s * T[2*i-1] % m
   *     endfor
   */
//  for (i=1; i<(1<<(k-1)); i++) {
//    _mpm_mul(p, s, msize, &T[i-1][0], msize);
//    T[i][0] = p[3];
//    T[i][1] = p[4];
//    T[i][2] = p[5];
//    _mpm_fixed_mod_reduction(&T[i][0], p, m, u, msize);           Niet nodig, want het past gewoon.
//
//    MPM_SHL_BITS_SMALL(T[i], T[i-1], msize, 2); // *4
//  }

  /* Locate the most significant exponent bit
   */
  MPM_COUNT_LEADING_ZEROS(h, e, esize);

  /* 6.  while h >= 0
   *
   * Repeat until all the exponent bits are consumed
   */
  idx = h >> 7;
  h &= 127;
  bits_left = ((esize-idx) << 7) - h;

  qw0 = e[idx];
  qw1 = spu_and(e[idx+1], spu_maskw(spu_extract(spu_cmpgt(spu_promote(bits_left, 0), 128), 0)));


  while (bits_left) {
    /* Move the MPM_MOD_EXP_MAX_K (k) bits starting at bit h into the least 
     * significant bits of the preferred word slot.
     */
    shift = (unsigned int)(h) >> 3;

    qw = spu_slqw(spu_or(spu_slqwbyte(qw0, shift), 
		       spu_rlmaskqwbyte(qw1, (signed int)(shift-16))),
		(h & 7));

    /* Determine the window size. window_size0 is the window size if 
     * the bit at position h is a 0. window_size1 is the window size
     * if the bit a position h is a 1.
     */
    
    window_size0 = spu_cntlz(qw);
    window_size1 = spu_add(spu_cntlz(spu_xor(spu_sub(qw, window_lsb), qw)), 1);

    selector = spu_rlmaska(qw, -31);

    window = spu_extract(spu_sel(window_size0, window_size1, selector), 0);

    h += window;

    /* Don't let the window extend beyond the end of the exponent.
     */
    if (window > bits_left) window = bits_left;
    bits_left -= window;
    
    /* Handle all the squaring terms for the window.
     */
    for (i=0; i<window; i++) {
      _mpm_square(p, c, msize);
      _mpm_fixed_mod_reduction(c, p, m, u, msize);
    }

    /* The most significant bit of the window is a one. Multiply by
     * the pre-computed T array value.
     */
    if (spu_extract(selector, 0)) {
      T_idx = spu_extract(spu_rlmask(qw, window-33), 0);

      T_idx = T_idx * 2 + 1;
      int ti = 2;

      if (T_idx >= 348)
      {
          printf("Error %d\n", T_idx);
      }
      else if (T_idx >= 256)
      {
          p[0] = ((vector unsigned int) { 0,0,0,0});
          p[1] = c[0];
          p[2] = c[1];
          p[3] = c[2];
          p[4] = ((vector unsigned int) { 0,0,0,0});
          p[5] = ((vector unsigned int) { 0,0,0,0});

          T_idx -= 256;
          ti = 0;
      }
      else if (T_idx >= 128)
      {
          p[0] = ((vector unsigned int) { 0,0,0,0});
          p[1] = ((vector unsigned int) { 0,0,0,0});
          p[2] = c[0];
          p[3] = c[1];
          p[4] = c[2];
          p[5] = ((vector unsigned int) { 0,0,0,0});

          T_idx -= 128;
          ti = 1;
      }
      else
      {
          p[0] = ((vector unsigned int) { 0,0,0,0});
          p[1] = ((vector unsigned int) { 0,0,0,0});
          p[2] = ((vector unsigned int) { 0,0,0,0});
          p[3] = c[0];
          p[4] = c[1];
          p[5] = c[2];

      }
      MPM_SHL_BITS_LARGE((&(p[ti])), (&(p[ti])), 4, T_idx, p[0]);
//      }
//      else
//      {
//          _mpm_mul(p, c, msize, &T[T_idx][0], msize);   // TODO dit is ook gewoon SHL natuurlijk..
//      }
      _mpm_fixed_mod_reduction(c, p, m, u, msize);
    }

    /* Advance to next quadword if it has been fully consumed.
     */
    if (h >> 7) {
      h &= 127;
      idx++;
      qw0 = qw1;
      qw1 = spu_and(e[idx+1], spu_maskw(spu_extract(spu_cmpgt(spu_promote(bits_left, 0), 128), 0)));
    }
  }
}

#endif /* _MPM_MOD_EXP2_H_ */
