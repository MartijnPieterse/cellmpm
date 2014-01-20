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

#include <spu_intrinsics.h>
#include "mpm_defines.h"
#include "mpm_cmpgt.h"
#include "mpm_sub.h"

#ifndef _FIXED_MOD_REDUCTION_H_
#define _FIXED_MOD_REDUCTION_H_		1


/* mpm_fixed_mod_reduction
 * -------------------
 * A basic building block of encryption is computing r = a mod m. 
 * For public-key algorithms (for example: RSA, Diffie-Hellman),
 * encryption/decryption requires reduction modulo the user's 
 * public key modulus (which is fixed for all exponentiations). 
 *
 * Fixed-modular reduction algorithm:
 *
 * Inputs: a (2*n words)
 *         m (n words, m!=0)
 *         u (n+2 words)
 *         n
 * Output: r (n words, 0<=r<m)
 *
 * 1.  if a < m then
 *     1.1  r = a
 *     1.2  return r
 * 2.  q = a
 * 3.  shift q to the right by n-1 words
 * 4.  q = q * u
 * 5.  shift q to the right by n+1 words
 * 6.  q = q * m
 * 7.  set q to the n+1 least significant words of q.
 * 8.  set a to the n+1 least signifncant words of a.
 * 9.  q = a - q
 * 10. if q is negative then
 *     10.1  q = q + 2^(w*(n+1))
 *     endif
 * 11. while q >= m do		"should be executed at most twice"
 *     11.1  q = q - m
 *     endwhile
 * 12. r = q
 * 13. return r
 * 
 * For this implementation, a word is 128-bits (ie, quadword).
 *
 * WARNING: This function only works if r is not a.
 */

static __inline void _mpm_fixed_mod_reduction(vector unsigned int *r, const vector unsigned int *a, const vector unsigned int *m, const vector unsigned int *u, int n)
{
  int i, j, idx;
  int qsize;
  vector unsigned int q[2*MPM_MAX_SIZE+3];
  vector unsigned int p[2*MPM_MAX_SIZE+3];
  vector unsigned int c[2*MPM_MAX_SIZE+3];
  vector unsigned int *ptr;
  vector unsigned short a0, a1, b0, b10, b32, b54, b76;
  vector unsigned int p0, q0;
  vector unsigned int c0_0;
  vector unsigned int p0_0, p0_1, p0_2, p0_3, p0_4, p0_5, p0_6, p0_7;
  vector unsigned int p1_0, p1_1, p1_2, p1_3, p1_4, p1_5, p1_6, p1_7;
  vector unsigned int s1, s2, s3, s4, s5, s6, s7;
  vector unsigned int c0, c1, c2, c3, c4, c5, c6, c7, c8;
  vector unsigned int prev_s1, prev_s2, prev_s3, prev_s4, prev_s5, prev_s6, prev_s7;
  vector unsigned int prev_c1, prev_c2, prev_c3, prev_c4, prev_c5, prev_c6, prev_c7;
  vector unsigned int ms1, ms2, ms3, ms4, ms5, ms6, ms7;
  vector unsigned int mc1, mc2, mc3, mc4, mc5, mc6, mc7;
  vector unsigned int mc01, mc23, mc45, mc67;
  vector unsigned int mc0123, mc4567, mc01234567;
  vector unsigned int carry, addend, sum, borrow;
  vector unsigned char splat_short10 = (vector unsigned char)(spu_splats((unsigned int)0x0C0D0E0F));
  vector unsigned char splat_short32 = (vector unsigned char)(spu_splats((unsigned int)0x08090A0B));
  vector unsigned char splat_short54 = (vector unsigned char)(spu_splats((unsigned int)0x04050607));
  vector unsigned char splat_short76 = (vector unsigned char)(spu_splats((unsigned int)0x00010203));


  vector unsigned char dbl_qw_shl_2  = ((vector unsigned char) {
						   0x02, 0x03, 0x04, 0x05, 
						   0x06, 0x07, 0x08, 0x09,
						   0x0A, 0x0B, 0x0C, 0x0D,
						   0x0E, 0x0F, 0x10, 0x11});
  vector unsigned char dbl_qw_shl_4  = ((vector unsigned char) {
						   0x04, 0x05, 0x06, 0x07, 
						   0x08, 0x09, 0x0A, 0x0B,
						   0x0C, 0x0D, 0x0E, 0x0F,
						   0x10, 0x11, 0x12, 0x13});
  vector unsigned char dbl_qw_shl_6  = ((vector unsigned char) {
						   0x06, 0x07, 0x08, 0x09, 
						   0x0A, 0x0B, 0x0C, 0x0D,
						   0x0E, 0x0F, 0x10, 0x11,
						   0x12, 0x13, 0x14, 0x15});
  vector unsigned char dbl_qw_shl_8  = ((vector unsigned char) {
						   0x08, 0x09, 0x0A, 0x0B, 
						   0x0C, 0x0D, 0x0E, 0x0F,
						   0x10, 0x11, 0x12, 0x13,
						   0x14, 0x15, 0x16, 0x17});
  vector unsigned char dbl_qw_shl_10 = ((vector unsigned char) {
						   0x0A, 0x0B, 0x0C, 0x0D,
						   0x0E, 0x0F, 0x10, 0x11,
						   0x12, 0x13, 0x14, 0x15, 
						   0x16, 0x17, 0x18, 0x19});
  vector unsigned char dbl_qw_shl_12 = ((vector unsigned char) {
						   0x0C, 0x0D, 0x0E, 0x0F,
						   0x10, 0x11, 0x12, 0x13,
						   0x14, 0x15, 0x16, 0x17,
						   0x18, 0x19, 0x1A, 0x1B});
  vector unsigned char dbl_qw_shl_14 = ((vector unsigned char) {
						   0x0E, 0x0F, 0x10, 0x11,
						   0x12, 0x13, 0x14, 0x15, 
						   0x16, 0x17, 0x18, 0x19,
						   0x1A, 0x1B, 0x1C, 0x1D});

  /* 1.  if a < m then
   *     1.1  r = a
   *     1.2  return r
   *
   * Don't bother testing for a less than m since this is infrequent and the
   * cost of this test out-weighs the benefit we will receive.
   */
  
  /* 2.  q = a
   * 3.  shift q to the right by n-1 words
   * 4.  q = q * u
   * 5.  shift q to the right by n+1 words
   *
   * These steps perform a basic multi-precision multiply of the n+1 
   * most significant words of a and u (n+2) words.
   *
   *    _mpm_mul(q, u, n+2, a, n+1)
   *
   * This code is further optimized by knowing that the least sigificant
   * n+1 words of the 2*n+3 product words are discarded.
   */
  {
    qsize = 2*n + 3;

    /* Zero the initial product and carry accumulate arrays
     */
    for (i=0; i<qsize; i++) {
      p[i] = c[i] = spu_splats((unsigned int)0);
    }

    for (i=n; i>=0; i--) {		/* for each quadword of b */
      b0 = (vector unsigned short)(a[i]);
      /* b10 = B1 B0 B1 B0 B1 B0 B1 B0 
       * b32 = B3 B2 B3 B2 B3 B2 B3 B2
       * b54 = B5 B4 B5 B4 B5 B4 B5 B4
       * b76 = B7 B6 B7 B6 B7 B6 B7 B6
       */
      b10 = spu_shuffle(b0, b0, splat_short10);
      b32 = spu_shuffle(b0, b0, splat_short32);
      b54 = spu_shuffle(b0, b0, splat_short54);
      b76 = spu_shuffle(b0, b0, splat_short76);

      prev_c1 = prev_c2 = prev_c3 = prev_c4 = prev_c5 = prev_c6 = prev_c7 = spu_splats((unsigned int)0);
      prev_s1 = prev_s2 = prev_s3 = prev_s4 = prev_s5 = prev_s6 = prev_s7 = spu_splats((unsigned int)0);
      p1_7    = spu_splats((unsigned int)0);
      
      idx = n+2+i;
      
      for (j=n+1; j>=0; j--, idx--) {	/* for each quadword of a */
	/* Fetch the product and carry quadwords affect by this
	 * quadword inner product.
	 */
	p0 = p[idx];
	c0 = c[idx];
	
	/* a0 = A7 A6 A5 A4 A3 A2 A1 A0
	 * a1 = A6 A7 A4 A4 A2 A3 A0 A1
	 */
	a0 = (vector unsigned short)(u[j]);
	a1 = (vector unsigned short)(spu_rl((vector unsigned int)(a0), 16));
	
	/* Compute the partial products, one most significant product
	 * term (p1_7) is computed later since it affects the next quadword
	 * (not this one).
	 */
	p0_0 = spu_mulo(a0, b10);	/* A6*B0 A4*B0 A2*B0 A0*B0 */
	p1_0 = spu_mulo(a1, b10);	/* A7*B0 A5*B0 A3*B0 A1*B0 */
	p0_1 = spu_mule(a1, b10);	/* A6*B1 A4*B1 A2*B1 A0*B1 */
	p1_1 = spu_mule(a0, b10);	/* A7*B1 A5*B1 A3*B1 A1*B1 */
	
	p0_2 = spu_mulo(a0, b32);	/* A6*B2 A4*B2 A2*B2 A0*B2 */
	p1_2 = spu_mulo(a1, b32);	/* A7*B2 A5*B2 A3*B2 A1*B2 */
	p0_3 = spu_mule(a1, b32);	/* A6*B3 A4*B3 A2*B3 A0*B3 */
	p1_3 = spu_mule(a0, b32);	/* A7*B3 A5*B3 A3*B3 A1*B3 */
	
	p0_4 = spu_mulo(a0, b54);	/* A6*B4 A4*B4 A2*B4 A0*B4 */
	p1_4 = spu_mulo(a1, b54);	/* A7*B4 A5*B4 A3*B4 A1*B4 */
	p0_5 = spu_mule(a1, b54);	/* A6*B5 A4*B5 A2*B5 A0*B5 */
	p1_5 = spu_mule(a0, b54);	/* A7*B5 A5*B5 A3*B5 A1*B5 */
	
	p0_6 = spu_mulo(a0, b76);	/* A6*B6 A4*B6 A2*B6 A0*B6 */
	p1_6 = spu_mulo(a1, b76);	/* A7*B6 A5*B6 A3*B6 A1*B6 */
	p0_7 = spu_mule(a1, b76);	/* A6*B7 A4*B7 A2*B7 A0*B7 */
	
	/* Combine product terms that have the same alignment. The
	 * combined product terms consist of sums (s#) and carrys (c#).
	 */
	c1 = spu_genc(p1_0, p0_1);
	s1 = spu_add(p1_0, p0_1);
	
	c2 = spu_genc(p1_1, p0_2);
	s2 = spu_add(p1_1, p0_2);
	
	c3 = spu_genc(p1_2, p0_3);
	s3 = spu_add(p1_2, p0_3);
	
	c4 = spu_genc(p1_3, p0_4);
	s4 = spu_add(p1_3, p0_4);
	
	c5 = spu_genc(p1_4, p0_5);
	s5 = spu_add(p1_4, p0_5);
	
	c6 = spu_genc(p1_5, p0_6);
	s6 = spu_add(p1_5, p0_6);
	
	c7 = spu_genc(p1_6, p0_7);
	s7 = spu_add(p1_6, p0_7);
	
	/* Merge combined partial products with the previously iterations 
	 * combined partial products.
	 */
	ms1 = spu_shuffle(s1, prev_s1, dbl_qw_shl_2);
	mc1 = spu_shuffle(c1, prev_c1, dbl_qw_shl_2);
	ms2 = spu_shuffle(s2, prev_s2, dbl_qw_shl_4);
	mc2 = spu_shuffle(c2, prev_c2, dbl_qw_shl_4);
	ms3 = spu_shuffle(s3, prev_s3, dbl_qw_shl_6);
	mc3 = spu_shuffle(c3, prev_c3, dbl_qw_shl_6);
	ms4 = spu_shuffle(s4, prev_s4, dbl_qw_shl_8);
	mc4 = spu_shuffle(c4, prev_c4, dbl_qw_shl_8);
	ms5 = spu_shuffle(s5, prev_s5, dbl_qw_shl_10);
	mc5 = spu_shuffle(c5, prev_c5, dbl_qw_shl_10);
	ms6 = spu_shuffle(s6, prev_s6, dbl_qw_shl_12);
	mc6 = spu_shuffle(c6, prev_c6, dbl_qw_shl_12);
	ms7 = spu_shuffle(s7, prev_s7, dbl_qw_shl_14);
	mc7 = spu_shuffle(c7, prev_c7, dbl_qw_shl_14);
	
	/* Accumulate all the terms into the final products and carry 
	 * arrays, p and c.
	 */
	c8    = spu_genc(p0, p1_7);	/* from previous loop */
	p0    = spu_add(p0, p1_7);
	c0    = spu_add(c0, c8);
	
	c0_0  = spu_genc(p0, p0_0);
	p0    = spu_add(p0, p0_0);
	
	MPM_ADD_PARTIAL(p0, c0, p0, ms1,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms2,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms3,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms4,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms5,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms6,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms7,  c0);
	
	mc01 = spu_add(c0_0, mc1);
	mc23 = spu_add(mc2,  mc3);
	mc45 = spu_add(mc4,  mc5);
	mc67 = spu_add(mc6,  mc7);
	
	mc0123 = spu_add(mc01, mc23);
	mc4567 = spu_add(mc45, mc67);
	
	mc01234567 = spu_add(mc0123, mc4567);
	
	c0 = spu_add(c0, mc01234567);
	
	/* Copy the product terms computed in this loop 
	 * for use in the next loop. 
	 */
	prev_c1 = c1;
	prev_s1 = s1;
	prev_c2 = c2;
	prev_s2 = s2;
	prev_c3 = c3;
	prev_s3 = s3;
	prev_c4 = c4;
	prev_s4 = s4;
	prev_c5 = c5;
	prev_s5 = s5;
	prev_c6 = c6;
	prev_s6 = s6;
	prev_c7 = c7;
	prev_s7 = s7;
	
	p1_7 = spu_mule(a0, b76);	/* A7*B7 A5*B7 A3*B7 A1*B7 */
	
	/* Store the result */
	p[idx] = p0;
	c[idx] = c0;
      }
      
      p0 = p[idx];
      c0 = c[idx];
      
      /* Align final partial products.
       */
      ms1 = spu_rlmaskqwbyte(prev_s1, 2-16);
      mc1 = spu_rlmaskqwbyte(prev_c1, 2-16);
      ms2 = spu_rlmaskqwbyte(prev_s2, 4-16);
      mc2 = spu_rlmaskqwbyte(prev_c2, 4-16);
      ms3 = spu_rlmaskqwbyte(prev_s3, 6-16);
      mc3 = spu_rlmaskqwbyte(prev_c3, 6-16);
      ms4 = spu_rlmaskqwbyte(prev_s4, 8-16);
      mc4 = spu_rlmaskqwbyte(prev_c4, 8-16);
      ms5 = spu_rlmaskqwbyte(prev_s5, 10-16);
      mc5 = spu_rlmaskqwbyte(prev_c5, 10-16);
      ms6 = spu_rlmaskqwbyte(prev_s6, 12-16);
      mc6 = spu_rlmaskqwbyte(prev_c6, 12-16);
      ms7 = spu_rlmaskqwbyte(prev_s7, 14-16);
      mc7 = spu_rlmaskqwbyte(prev_c7, 14-16);
      
      /* Accumulate all the terms into the final products and carry 
       * arrays, p and c.
       */
      c8    = spu_genc(p0, p1_7);	/* from previous loop */
      p0    = spu_add(p0, p1_7);
      
      MPM_ADD_PARTIAL(p0, c0, p0, ms1,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms2,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms3,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms4,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms5,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms6,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms7,  c0);
      
      mc01 = spu_add(c8, mc1);
      mc23 = spu_add(mc2,  mc3);
      mc45 = spu_add(mc4,  mc5);
      mc67 = spu_add(mc6,  mc7);
      
      mc0123 = spu_add(mc01, mc23);
      mc4567 = spu_add(mc45, mc67);
      
      mc01234567 = spu_add(mc0123, mc4567);
      
      c0 = spu_add(c0, mc01234567);
      
      /* Store the result */
      p[idx] = p0;
      c[idx] = c0;
    }
    
    /* Perform a fully carry propogate of the accumulated carries 
     * with the accumulated products. Since we are going to discard
     * the least significan n-1 quadwords, we only need to propogate
     * the n+4 most significant quadwords.
     */
    i = n+2;
    c0 = c[i];
    p0 = p[i];
    carry = spu_rlqwbyte(c0, 4);
    sum   = spu_add(p0, carry);
    carry = spu_rlmaskqwbyte(spu_genc(p0, carry), -12);
    
    if (__builtin_expect((spu_extract(sum, 0) == (unsigned int)(-1)), 0)) {
      /* Unknown carry condition, perform complete carry propogtion.
       */
      c0 = carry = spu_splats((unsigned int)0);
      i = qsize;
    }
    
    while (i-- > 0) {
      c1 = c[i];
      addend  = spu_shuffle(c1, c0, dbl_qw_shl_4);
      MPM_ADD_FULL(q[i], carry, p[i], addend, carry);
      carry = spu_rlmaskqwbyte(carry, -12);

      /* Clear product and carry arrays in preporation
       * of mext multiply.
       */
      p[i] = c[i] = spu_splats((unsigned int)0);
      
      c0 = c1;
    }
  }

  /* 
   * 6. q = q * m
   * 7. set q to the n+1 least signficant words of q.
   *
   * These steps perform a basic multi-precision multiply of q (of size
   * n+4 words) and m (of size n words). 
   *
   *    _mpm_mul(q, m, n, q, n+4)
   *
   * Since only the least significant n+1 words of the product are needed
   * a specialized implementation of mpm_mul is generated.
   */
  {
    /* Zero the initial product and carry accumulate arrays
     *
     * The initialization was moved into the add loop above.
     */
    for (i=n-1; i>=0; i--) {		/* for each quadword of b */
      b0 = (vector unsigned short)(m[i]);
      /* b10 = B1 B0 B1 B0 B1 B0 B1 B0 
       * b32 = B3 B2 B3 B2 B3 B2 B3 B2
       * b54 = B5 B4 B5 B4 B5 B4 B5 B4
       * b76 = B7 B6 B7 B6 B7 B6 B7 B6
       */
      b10 = spu_shuffle(b0, b0, splat_short10);
      b32 = spu_shuffle(b0, b0, splat_short32);
      b54 = spu_shuffle(b0, b0, splat_short54);
      b76 = spu_shuffle(b0, b0, splat_short76);

      prev_c1 = prev_c2 = prev_c3 = prev_c4 = prev_c5 = prev_c6 = prev_c7 = spu_splats((unsigned int)0);
      prev_s1 = prev_s2 = prev_s3 = prev_s4 = prev_s5 = prev_s6 = prev_s7 = spu_splats((unsigned int)0);
      p1_7    = spu_splats((unsigned int)0);
      
      for (j=n+1, idx=i+1; idx>=0; j--, idx--) {	/* for each quadword of a */
	/* Fetch the product and carry quadwords affect by this
	 * quadword inner product.
	 */
	p0 = p[idx];
	c0 = c[idx];
	
	/* a0 = A7 A6 A5 A4 A3 A2 A1 A0
	 * a1 = A6 A7 A4 A4 A2 A3 A0 A1
	 */
	a0 = (vector unsigned short)(q[j]);
	a1 = (vector unsigned short)(spu_rl((vector unsigned int)(a0), 16));
	
	/* Compute the partial products, one most significant product
	 * term (p1_7) is computed later since it affects the next quadword
	 * (not this one).
	 */
	p0_0 = spu_mulo(a0, b10);	/* A6*B0 A4*B0 A2*B0 A0*B0 */
	p1_0 = spu_mulo(a1, b10);	/* A7*B0 A5*B0 A3*B0 A1*B0 */
	p0_1 = spu_mule(a1, b10);	/* A6*B1 A4*B1 A2*B1 A0*B1 */
	p1_1 = spu_mule(a0, b10);	/* A7*B1 A5*B1 A3*B1 A1*B1 */
	
	p0_2 = spu_mulo(a0, b32);	/* A6*B2 A4*B2 A2*B2 A0*B2 */
	p1_2 = spu_mulo(a1, b32);	/* A7*B2 A5*B2 A3*B2 A1*B2 */
	p0_3 = spu_mule(a1, b32);	/* A6*B3 A4*B3 A2*B3 A0*B3 */
	p1_3 = spu_mule(a0, b32);	/* A7*B3 A5*B3 A3*B3 A1*B3 */
	
	p0_4 = spu_mulo(a0, b54);	/* A6*B4 A4*B4 A2*B4 A0*B4 */
	p1_4 = spu_mulo(a1, b54);	/* A7*B4 A5*B4 A3*B4 A1*B4 */
	p0_5 = spu_mule(a1, b54);	/* A6*B5 A4*B5 A2*B5 A0*B5 */
	p1_5 = spu_mule(a0, b54);	/* A7*B5 A5*B5 A3*B5 A1*B5 */
	
	p0_6 = spu_mulo(a0, b76);	/* A6*B6 A4*B6 A2*B6 A0*B6 */
	p1_6 = spu_mulo(a1, b76);	/* A7*B6 A5*B6 A3*B6 A1*B6 */
	p0_7 = spu_mule(a1, b76);	/* A6*B7 A4*B7 A2*B7 A0*B7 */
	
	/* Combine product terms that have the same alignment. The
	 * combined product terms consist of sums (s#) and carrys (c#).
	 */
	c1 = spu_genc(p1_0, p0_1);
	s1 = spu_add(p1_0, p0_1);
	
	c2 = spu_genc(p1_1, p0_2);
	s2 = spu_add(p1_1, p0_2);
	
	c3 = spu_genc(p1_2, p0_3);
	s3 = spu_add(p1_2, p0_3);
	
	c4 = spu_genc(p1_3, p0_4);
	s4 = spu_add(p1_3, p0_4);
	
	c5 = spu_genc(p1_4, p0_5);
	s5 = spu_add(p1_4, p0_5);
	
	c6 = spu_genc(p1_5, p0_6);
	s6 = spu_add(p1_5, p0_6);
	
	c7 = spu_genc(p1_6, p0_7);
	s7 = spu_add(p1_6, p0_7);
	
	/* Merge combined partial products with the previously iterations 
	 * combined partial products.
	 */
	ms1 = spu_shuffle(s1, prev_s1, dbl_qw_shl_2);
	mc1 = spu_shuffle(c1, prev_c1, dbl_qw_shl_2);
	ms2 = spu_shuffle(s2, prev_s2, dbl_qw_shl_4);
	mc2 = spu_shuffle(c2, prev_c2, dbl_qw_shl_4);
	ms3 = spu_shuffle(s3, prev_s3, dbl_qw_shl_6);
	mc3 = spu_shuffle(c3, prev_c3, dbl_qw_shl_6);
	ms4 = spu_shuffle(s4, prev_s4, dbl_qw_shl_8);
	mc4 = spu_shuffle(c4, prev_c4, dbl_qw_shl_8);
	ms5 = spu_shuffle(s5, prev_s5, dbl_qw_shl_10);
	mc5 = spu_shuffle(c5, prev_c5, dbl_qw_shl_10);
	ms6 = spu_shuffle(s6, prev_s6, dbl_qw_shl_12);
	mc6 = spu_shuffle(c6, prev_c6, dbl_qw_shl_12);
	ms7 = spu_shuffle(s7, prev_s7, dbl_qw_shl_14);
	mc7 = spu_shuffle(c7, prev_c7, dbl_qw_shl_14);
	
	/* Accumulate all the terms into the final products and carry 
	 * arrays, p and c.
	 */
	c8    = spu_genc(p0, p1_7);	/* from previous loop */
	p0    = spu_add(p0, p1_7);
	c0    = spu_add(c0, c8);
	
	c0_0  = spu_genc(p0, p0_0);
	p0    = spu_add(p0, p0_0);
	
	MPM_ADD_PARTIAL(p0, c0, p0, ms1,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms2,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms3,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms4,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms5,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms6,  c0);
	MPM_ADD_PARTIAL(p0, c0, p0, ms7,  c0);
	
	mc01 = spu_add(c0_0, mc1);
	mc23 = spu_add(mc2,  mc3);
	mc45 = spu_add(mc4,  mc5);
	mc67 = spu_add(mc6,  mc7);
	
	mc0123 = spu_add(mc01, mc23);
	mc4567 = spu_add(mc45, mc67);
	
	mc01234567 = spu_add(mc0123, mc4567);
	
	c0 = spu_add(c0, mc01234567);
	
	/* Copy the product terms computed in this loop 
	 * for use in the next loop. 
	 */
	prev_c1 = c1;
	prev_s1 = s1;
	prev_c2 = c2;
	prev_s2 = s2;
	prev_c3 = c3;
	prev_s3 = s3;
	prev_c4 = c4;
	prev_s4 = s4;
	prev_c5 = c5;
	prev_s5 = s5;
	prev_c6 = c6;
	prev_s6 = s6;
	prev_c7 = c7;
	prev_s7 = s7;
	
	p1_7 = spu_mule(a0, b76);	/* A7*B7 A5*B7 A3*B7 A1*B7 */
	
	/* Store the result */
	p[idx] = p0;
	c[idx] = c0;
      }
      
      p0 = p[idx];
      c0 = c[idx];
      
      /* Align final partial products.
       */
      ms1 = spu_rlmaskqwbyte(prev_s1, 2-16);
      mc1 = spu_rlmaskqwbyte(prev_c1, 2-16);
      ms2 = spu_rlmaskqwbyte(prev_s2, 4-16);
      mc2 = spu_rlmaskqwbyte(prev_c2, 4-16);
      ms3 = spu_rlmaskqwbyte(prev_s3, 6-16);
      mc3 = spu_rlmaskqwbyte(prev_c3, 6-16);
      ms4 = spu_rlmaskqwbyte(prev_s4, 8-16);
      mc4 = spu_rlmaskqwbyte(prev_c4, 8-16);
      ms5 = spu_rlmaskqwbyte(prev_s5, 10-16);
      mc5 = spu_rlmaskqwbyte(prev_c5, 10-16);
      ms6 = spu_rlmaskqwbyte(prev_s6, 12-16);
      mc6 = spu_rlmaskqwbyte(prev_c6, 12-16);
      ms7 = spu_rlmaskqwbyte(prev_s7, 14-16);
      mc7 = spu_rlmaskqwbyte(prev_c7, 14-16);
      
      /* Accumulate all the terms into the final products and carry 
       * arrays, p and c.
       */
      c8    = spu_genc(p0, p1_7);	/* from previous loop */
      p0    = spu_add(p0, p1_7);
      
      MPM_ADD_PARTIAL(p0, c0, p0, ms1,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms2,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms3,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms4,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms5,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms6,  c0);
      MPM_ADD_PARTIAL(p0, c0, p0, ms7,  c0);
      
      mc01 = spu_add(c8, mc1);
      mc23 = spu_add(mc2,  mc3);
      mc45 = spu_add(mc4,  mc5);
      mc67 = spu_add(mc6,  mc7);
      
      mc0123 = spu_add(mc01, mc23);
      mc4567 = spu_add(mc45, mc67);
      
      mc01234567 = spu_add(mc0123, mc4567);
      
      c0 = spu_add(c0, mc01234567);
      
      /* Store the result */
      p[idx] = p0;
      c[idx] = c0;
    }
    
    /* Perform a fully carry propogate of the accumulated carries 
     * with the accumulated products. Since we are going to discard
     * the least significan n-1 quadwords, we only need to propogate
     * the n+4 most significant quadwords.
     *
     * Note, this addition has been moved to the subtraction loop
     *       below.
     */
  }

  /* 8.  set a to the n+1 least signifncant words of a.
   * 9.  q = a - q
   *
   * The n least significant words of the result are written to the
   * array r in case we are done. The most significant word is kept 
   * in q0 for future use.
   */
  carry = c0 = spu_splats((unsigned int)0);
  borrow = spu_splats((unsigned int)1);
  ptr = (vector unsigned int *)(&a[n-1]);
  for (i=n; i>0; i--) {
    c1 = c[i];
    addend = spu_shuffle(c1, c0, dbl_qw_shl_4);
    MPM_ADD_FULL(sum, carry, p[i], addend, carry);
    carry = spu_rlmaskqwbyte(carry, -12);
    MPM_SUB_FULL(r[i-1], borrow, ptr[i], sum, borrow);
    c0 = c1;
  }
  addend = spu_shuffle(c[0], c0, dbl_qw_shl_4);
  MPM_ADD_FULL(sum, carry, p[0], addend, carry);
  MPM_SUB_FULL(q0, borrow, ptr[0], sum, borrow);

  /* 10. if q is negative then
   *     10.1  q = q + 2^(w*(n+1))
   *     endif
   *
   * I don't believe anything needs to be done here. This
   * addition is equivalent to taking the signed q value and
   * interpreting it as unsigned.
   */


  /* 11. while q >= m do
   *     11.1  q = q - m
   *     endwhile
   */
  while (spu_extract(q0, 3) | ~(_mpm_cmpgt(m, r, n))) {
    q0 = spu_subx(q0, spu_splats((unsigned int)0), _mpm_sub(r, r, m, n));
  }

  /* 12. r = q
   * 13. return r
   *
   * Result already in array r. See stop 9 (above).
   */
}


#endif /* _MPM_FIXED_MOD_REDUCTION_H_ */










