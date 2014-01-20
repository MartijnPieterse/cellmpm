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

#ifndef _MPM_MOD_H_
#define _MPM_MOD_H_	1

#include <spu_intrinsics.h>
#include "mpm_defines.h"
#include "mpm_macros.h"

#include <stdio.h>


/* mpm_mod
 * -------
 * Generic routine that computes the large unsigned number of <a_size> quadwords
 * pointed to by <a> modulo the large unsigned number of <b_size> quadwords 
 * pointed to by <b>. The resulting value is returned in <r> of <b_size> quadwords.
 * 
 *      <r> = <a> - (<a> / <b>) * <b>
 *
 * Note:
 *	<b> must be non-zero. An inifinite loop may occur if <b> is zero.
 */
static __inline void _mpm_mod(vector unsigned int *r, const vector unsigned int *a, int asize, const vector unsigned int *b, int bsize)
{
  int i, j;
  int acnt, bcnt;
  int abits, bbits;
  int askip, bskip;
  int ashift, bshift;
  int a_size, b_size;
  int delta, insert_idx, aa_idx, idx;
  int shift, qwords, qbytes;
  unsigned int aa_gt_bb, imask;
  vector unsigned char splat_int0 = (vector unsigned char)(spu_splats((unsigned int)0x00010203));
  vector unsigned char dbl_qw_shl_2  = ((vector unsigned char) {
						   0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
						   0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11});
  vector unsigned char dbl_qw_shl_4  = ((vector unsigned char) {
						   0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
						   0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13});
  vector unsigned short q10;
  vector unsigned int pattern = ((vector unsigned int) { 0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F});
  vector unsigned int pattern_a0 = ((vector unsigned int) { 0x00010203, 0x04050607, 0x80808080, 0x80808080});
  vector unsigned int mask, cmp;
  vector unsigned int a0, a1, ai, aj, b0, b1, bi, bj;
  vector unsigned int p0_0, p0_1, p1_0, p1_1, prev_p1_1;
  vector unsigned int p0, c0, c1, prev_c0, prev_c1, s1, prev_s1;
  vector unsigned int ms1, ms2, mc1;
  vector unsigned int sum, prev_sum, aligned_sum, align_sum;
  vector unsigned int remainder, remainder_msb, carry, borrow;
  vector unsigned int aa[MPM_MAX_SIZE+2], *aa_ptr;
  vector unsigned int bb[MPM_MAX_SIZE];
  vector unsigned int *src, *dst, msq;
  vector unsigned int estimate, align_a0, addend;
  vector unsigned int rem_gt_b0;
  vector unsigned int multiplier, mul0, mul1;
  vector unsigned int first_byte_a0, first_byte_sum;
  vector unsigned int gt, lt, gt_bigger, lt_bigger;
  vector unsigned int estimate_plus1, estimate_plus1_clamped, eq, B0, B1;

  /* Count leading zeros in <a> and <b>
   */
  acnt = 0;
  for (i=0; i<asize; i++) {
    vector unsigned int zero, v_acnt;							  

    ai = a[i];
    v_acnt = spu_cntlz(ai);
    zero = spu_cmpeq(ai, 0);
    acnt += spu_extract(v_acnt, 0);
    imask = spu_extract(zero, 0);
    acnt += imask & spu_extract(v_acnt, 1);
    imask &= spu_extract(zero, 1);
    acnt += imask & spu_extract(v_acnt, 2);
    imask &= spu_extract(zero, 2);
    acnt += imask & spu_extract(v_acnt, 3);
    if (spu_extract(spu_gather(zero), 0) != 15) break;
  }

  bcnt = 0;
  for (i=0; i<bsize; i++) {
    vector unsigned int zero, v_bcnt;							  

    bi = b[i];
    v_bcnt = spu_cntlz(bi);
    zero = spu_cmpeq(bi, 0);
    bcnt += spu_extract(v_bcnt, 0);
    imask = spu_extract(zero, 0);
    bcnt += imask & spu_extract(v_bcnt, 1);
    imask &= spu_extract(zero, 1);
    bcnt += imask & spu_extract(v_bcnt, 2);
    imask &= spu_extract(zero, 2);
    bcnt += imask & spu_extract(v_bcnt, 3);
    if (spu_extract(spu_gather(zero), 0) != 15) break;
  }

  abits = (asize << 7) - acnt;
  askip = (acnt >> 7);
  ashift = acnt & 127;
  a_size = asize - askip;

  bbits = (bsize << 7) - bcnt;
  bskip = (bcnt >> 7);
  bshift = bcnt & 127;
  b_size = bsize - bskip;

  delta = abits - bbits;

  a += askip;
  b += bskip;

  if (delta >= 0) {
    /* -----------------
     * Non-zero quotient
     * -----------------
     */
    aa[0] = spu_splats((unsigned int)0);
    aa[b_size+1] = spu_splats((unsigned int)0);
    
    /* Create local copies of <a> and <b> such that
     * <b> is left justified (ie, normalized with no 
     * leading 0 bits) and <a> is at least 2 words 
     * and <b> is less then the most significant bsize 
     * words of <a>. The local copies are aa and bb.
     */
    
    /* Copy the input arrays removing leading zeros and normalizing
     * the divisor <b> so that the first (msb) bit of the first word
     * is non-zero.
     */
    qwords = (31 + abits - bbits) >> 5;
    MPM_SHL_BITS_LARGE(bb, b, b_size, bshift, b[0]);
    
    /* The dividend (a) is aligned (shifted) by the same amount as b with
     * the size of a growing to accomodate bits being shifted off the front.
     */
    cmp = spu_cmpgt(spu_promote(bshift, 0), spu_promote(ashift, 0));
    mask = spu_maskw(spu_extract(cmp, 0));
    a_size -= spu_extract(cmp, 0);
    aa_ptr = (vector unsigned int *)(&aa[1]);
    src = (vector unsigned int *)(a) + spu_extract(cmp, 0);
    a0 = spu_andc(a[0], mask);
    
    MPM_SHL_BITS_LARGE(aa_ptr, src, a_size, bshift, a0);

    /* If the most signficant bits of aa >= bb, add a qword to the 
     * front of aa and extend it by a word.
     */
    imask = spu_extract(spu_cmpeq(spu_and(spu_promote(qwords, 0), 3), 0), 0);
    
    aa_gt_bb = -1;
    qbytes = 4 * qwords;
    first_byte_a0 = spu_add((vector unsigned int)spu_splats((unsigned char)((-qbytes)  & 0xF)), 
			    ((vector unsigned int){0x00010203, 0x04050607, 0x08090A0B, 0x0C0D0E0F}));
      
    a0 = aa_ptr[0];
    for (j=0; j<b_size; j++) {

      a1 = aa_ptr[j+1];
      bj = bb[j];
      
      aj = spu_shuffle(a0, a1, (vector unsigned char)(first_byte_a0));

      gt = spu_gather(spu_cmpgt(aj, bj));
      lt = spu_gather(spu_cmpgt(bj, aj));
      
      gt_bigger = spu_cmpgt(gt, lt);
      lt_bigger = spu_cmpgt(lt, gt);
      
      if (spu_extract(spu_or(gt_bigger, lt_bigger), 0)) {
	aa_gt_bb = spu_extract(gt_bigger, 0);
	break;
      }
    }
    
    qwords -= aa_gt_bb;
    aa_gt_bb &= imask;
    aa_ptr += aa_gt_bb;
    a_size -= aa_gt_bb;
    
    /* For each computed quotient word, we must do a double word (two most significant
     * words of a) divided by a word (the most significant word of b) to compute 
     * an estimate of the quotient word. Instead of performing a divide, we will
     * compute a fixed point inverse of the most significant word of b, which is
     * constant for all quotient words, and multiply this by the most significant
     * double words of a to compute an estimate.
     */
    
    /* The multiplier is a 33 bit value consisting of a 1 followed by the 32 bits
     * of element 0 of the variable multiplier.
     */
    a0 = aa_ptr[0];
    a1 = aa_ptr[1];
    b0 = bb[0];
    
    B0 = spu_and(b0, ((vector unsigned int) { -1, 0, 0, 0}));
    B1 = spu_rl(B0, 16);	/* used when below that improving the estimation */

    multiplier = spu_splats((unsigned int)0);
    remainder = spu_sub(0, b0), 1;
    
    for (i=0; i<32; i++) {
      remainder_msb = spu_rlmaska(remainder, -31);
      remainder = spu_sl(remainder, 1);
      rem_gt_b0 = spu_or(spu_cmpgt(remainder, b0), remainder_msb);
      remainder = spu_sel(remainder, spu_sub(remainder, b0), rem_gt_b0);
      multiplier = spu_sub(spu_sl(multiplier, 1), rem_gt_b0);
    }
    
    /* Given that multiplier is a 32 (plus 1) bit value comprised of two shorts
     * m1 m0.
     *
     *   mul0 = m1 m0 m1 m0  0  0  0  0
     *   mul1 = m0 m1 m0 m1  0  0  0  0
     */
    mul0 = spu_shuffle(multiplier, multiplier, 
		       ((vector unsigned char){0,1,2,3, 0,1,2,3, 128,128,128,128, 128,128,128,128}));
    mul1 = spu_rl(mul0, 16);
    
    aa_idx = 0;
    insert_idx = 4*asize - qwords;
    
    qbytes = 4 * qwords;
    
    shift = (qbytes - 4) & 0xF;
    first_byte_a0 = (vector unsigned int)(spu_splats((unsigned char)((-qbytes)  & 0xF)));
    first_byte_sum = (vector unsigned int)(spu_splats((unsigned char)((qbytes - 4) & 0xF)));
    
    align_a0 = spu_add(first_byte_a0, pattern_a0);
    align_sum = spu_add(first_byte_sum, pattern);
    
    for (i=0; i<qwords; i++) {
      /* Move the quadword word being processed into the left (most significant)
       * part of ai.
       */
      a0 = spu_shuffle(a0, a1, (vector unsigned char)(align_a0));
      
      /* Compute the quotient estimate as the 32 most signficant bits of
       * 96 bit product of the 64 most signficant bits of a0 times the 33 
       * multiplier bits.
       */
      p0_0 = spu_rlmaskqwbyte(spu_mulo((vector unsigned short)(a0), (vector unsigned short)(mul0)), -4);
      p0_1 = spu_rlmaskqwbyte(spu_mulo((vector unsigned short)(a0), (vector unsigned short)(mul1)), -2);
      
      p1_0 = spu_rlmaskqwbyte(spu_mule((vector unsigned short)(a0), (vector unsigned short)(mul1)), -2);
      p1_1 =                  spu_mule((vector unsigned short)(a0), (vector unsigned short)(mul0));
      
      MPM_ADD_PARTIAL_NO_CIN(estimate, carry, a0, p1_1);
      MPM_ADD_PARTIAL(estimate, carry, estimate, p0_0, carry);
      MPM_ADD_PARTIAL(estimate, carry, estimate, p0_1, carry);
      MPM_ADD_PARTIAL(estimate, carry, estimate, p1_0, carry);
      
      addend = spu_slqwbyte(carry, 4);
      MPM_ADD_PARTIAL_NO_CIN(estimate, carry, estimate, addend);
      
      addend = spu_slqwbyte(carry, 4);
      MPM_ADD_PARTIAL_NO_CIN(estimate, carry, estimate, addend);
      
      /* The estimate computed using fixed point multiplication can be 
       * up to 1 value too small. If (1 + estimate) * b0 <= a0, then increment
       * the estimate since it was under-valued. We could always increment
       * the estimate, but this causes extra iterations of the correction loop
       * below.
       */
      estimate_plus1 = spu_add(estimate, 1);
      estimate_plus1_clamped = spu_or(estimate_plus1, spu_cmpeq(estimate, -1));

      p0_0 = spu_rlmaskqwbyte(spu_mulo((vector unsigned short)(estimate_plus1), 
				       (vector unsigned short)(B0)), -4);
      p0_1 = spu_rlmaskqwbyte(spu_mulo((vector unsigned short)(estimate_plus1), 
				       (vector unsigned short)(B1)), -2);
      p1_0 = spu_rlmaskqwbyte(spu_mule((vector unsigned short)(estimate_plus1),
				       (vector unsigned short)(B1)), -2);
      p1_1 = spu_mule((vector unsigned short)(estimate_plus1), (vector unsigned short)(B0));
			      
      MPM_ADD_PARTIAL_NO_CIN(sum, carry, p1_0, p1_1);
      MPM_ADD_PARTIAL(sum, carry, sum, p0_1, carry);
      MPM_ADD_PARTIAL(sum, carry, sum, p0_0, carry);
      
      sum = spu_add(sum, spu_slqwbyte(carry, 4));
      gt = spu_cmpgt(sum, a0);
      eq = spu_cmpeq(a0, sum);
      cmp = spu_or(gt, spu_and(spu_rlqwbyte(gt, 4), eq));

      estimate = spu_sel(estimate_plus1_clamped, estimate, cmp);

      /* Compute the product of the estimated quotient and the divisor b and subtract it
       * from a to produce the remainder. Note: by pipeline the product and subtract in
       * the same loop, we can eliminate most of the dependency stalls and increase dual
       * issue rates.
       *
       *	r = a - (estimate * b)
       */
      
      /* q10 = Q1 Q0 Q1 Q0 Q1 Q0 Q1 Q0
       */
      q10 = (vector unsigned short)spu_shuffle(estimate, estimate, splat_int0);
      
      j = b_size-1;
      
      b0 = bb[j];
      b1 = spu_rl(b0, 16);
      
      /* Compute the partial products
       */	
      p0_0 = spu_mulo ((vector unsigned short)(b0), q10);
      p1_0 = spu_mulo ((vector unsigned short)(b1), q10);
      p0_1 = spu_mule((vector unsigned short)(b1), q10);
      prev_p1_1 = spu_mule((vector unsigned short)(b0), q10);
      
      /* Combine product terms that have the same alignment.
       */
      prev_c1 = spu_genc(p1_0, p0_1);
      prev_s1 = spu_add(p1_0, p0_1);
      
      /* Merge combined partial products with the previously iterations
       * combined partial products.
       */
      ms1 = spu_slqwbyte(prev_s1,  2);
      mc1 = spu_slqwbyte(prev_c1,  2);
      ms2 = spu_slqwbyte(prev_p1_1, 4);
      
      /* Accumulate all the terms into the final products and carry
       * arrays, p and c.
       */
      c0  = spu_genc(p0_0, ms2);
      p0  = spu_add(p0_0, ms2);
      
      MPM_ADD_PARTIAL(p0, c0, p0, ms1,  c0);
      
      c0 = spu_add(c0, mc1);
      
      carry = prev_c0 = prev_sum = spu_splats((unsigned int)0);
      borrow = spu_splats((unsigned int)1);
      
      idx = aa_idx + b_size;
      
      while (j-- > 0) {
	b0 = bb[j];
	b1 = spu_rl(b0, 16);
	
	/* Perform a full add of the sum of product terms and accumulated
	 * carries.
	 */
	addend  = spu_shuffle(c0, prev_c0, dbl_qw_shl_4);
	prev_c0 = c0;
	MPM_ADD_FULL(sum, carry, p0, addend, carry);
	
	aligned_sum = spu_shuffle(sum, prev_sum, (vector unsigned char)(align_sum));
	
	carry = spu_rlmaskqwbyte(carry, -12);
	
	/* Subtract result from a. 
	 */
	a0 = aa_ptr[idx];
	MPM_SUB_FULL(aa_ptr[idx], borrow, a0, aligned_sum, borrow);
	
	idx--;
	prev_sum = sum;
	
	/* Compute the partial products
	 */	
	p0_0 = spu_mulo((vector unsigned short)(b0), q10);
	p1_0 = spu_mulo((vector unsigned short)(b1), q10);
	p0_1 = spu_mule((vector unsigned short)(b1), q10);
	p1_1 = spu_mule((vector unsigned short)(b0), q10);
	
	/* Combine product terms that have the same alignment.
	 */
	c1 = spu_genc(p1_0, p0_1);
	s1 = spu_add(p1_0, p0_1);
	
	/* Merge combined partial products with the previously iterations
	 * combined partial products.
	 */
	ms1 = spu_shuffle(s1,   prev_s1,   dbl_qw_shl_2);
	mc1 = spu_shuffle(c1,   prev_c1,   dbl_qw_shl_2);
	ms2 = spu_shuffle(p1_1, prev_p1_1, dbl_qw_shl_4);
	
	/* Accumulate all the terms into the final products and carry
	 * arrays, p and c.
	 */
	c0  = spu_genc(p0_0, ms2);
	p0  = spu_add(p0_0, ms2);
	
	MPM_ADD_PARTIAL(p0, c0, p0, ms1, c0);
	
	c0 = spu_add(c0, mc1);
	
	/* Copy the product terms computed in this loop
	 * for use in the next loop.
	 */
	prev_c1 = c1;
	prev_s1 = s1;
	prev_p1_1 = p1_1;
      }
      
      /* Perform a full add of the sum of product terms and accumulated
       * carries.
       */
      addend  = spu_shuffle(c0, prev_c0, dbl_qw_shl_4);
      MPM_ADD_FULL(sum, carry, p0, addend, carry);
      
      aligned_sum = spu_shuffle(sum, prev_sum, (vector unsigned char)(align_sum));
      
      carry = spu_rlmaskqwbyte(carry, -12);
      
      /* Subtract result from a and place in r (remainder array). 
       */
      a0 = aa_ptr[aa_idx+1];
      MPM_SUB_FULL(aa_ptr[aa_idx+1], borrow, a0, aligned_sum, borrow);
      
      prev_sum = sum;
      
      /* Align final partial products.
       */
      ms1 = spu_rlmaskqwbyte(prev_s1, 2-16);
      ms2 = spu_rlmaskqwbyte(prev_p1_1, 4-16);
      
      /* Accumulate all the terms into the final products and carry
       * arrays, p and c. Note, we don't need to include the carries since
       * no values will carry out of the least significant word.
       */
      carry = spu_add(carry, spu_rlmaskqwbyte(c0, -12));
      p0 = spu_add(ms1, ms2);
      
      /* Store the result */
      sum = spu_add(p0, carry);
      
      aligned_sum = spu_shuffle(sum, prev_sum, (vector unsigned char)(align_sum));
      
      a0 = aa_ptr[aa_idx];
      
      MPM_SUB_FULL(a0, borrow, a0, aligned_sum, borrow);
      aa_ptr[aa_idx] = a0;
      
      /* If the quotient still was too large, repeatedly adjust by 
       * 1 until the remainder is positive.
       */
      while ((signed int)(spu_extract(a0, 0)) < 0) {
	estimate = spu_add(estimate, -1);
	mask  = spu_slqwbyte(spu_splats((unsigned int)-1), shift);
	b1 = spu_splats((unsigned int)0);
	carry = spu_splats((unsigned int)0);
	for (j=b_size-1; j>=0; j--) {
	  b0 = spu_rlqwbyte(bb[j], shift);
	  bj  = spu_sel(b1, b0, mask);
	  MPM_ADD_FULL(aa_ptr[aa_idx+j+1], carry, aa_ptr[aa_idx+j+1], bj, carry)
	    carry = spu_rlmaskqwbyte(carry, -12);
	  b1 = b0;
	}
	a0 = aa_ptr[aa_idx];
	b1 = spu_andc(b1, mask);
	
	MPM_ADD_FULL(a0, carry, a0, b1, carry);
	aa_ptr[aa_idx] = a0;
      }

      aa_idx -= spu_extract(spu_cmpeq(spu_promote(insert_idx & 3, 0), 3), 0);
      insert_idx++;

      first_byte_a0 = (vector unsigned int)spu_and((vector unsigned char)spu_add((vector unsigned short)(first_byte_a0), 0x0404), 15);
      first_byte_sum = (vector unsigned int)spu_and((vector unsigned char)spu_add((vector unsigned short)(first_byte_sum), 0x0C0C), 15);
      
      align_a0 = spu_add(first_byte_a0, pattern_a0);
      align_sum = spu_add(first_byte_sum, pattern);
      
      /* Fetch next words of aa
       */
      shift = (shift - 4) & 0xF;
      
      a0 = aa_ptr[aa_idx];
      a1 = aa_ptr[aa_idx+1];
    }
    
    /* Right shift whats left of aa into the remainder.
     */
    for (i=0; i<bsize-a_size; i++) r[i] = spu_splats((unsigned int)0);
    dst = &r[i];
    aa_idx = i+a_size-bsize;
    src = &aa_ptr[aa_idx];
    msq = spu_andc(aa_ptr[aa_idx-1], spu_maskw(spu_extract(spu_cmpeq(spu_promote(aa_idx, 0), 0), 0)));
      
    MPM_SHR_BITS_LARGE(dst, src, bsize-i, msq, bshift)
  } else {
    /* Copy a into the remainder (r).
     */
    for (i=0; i<bsize-a_size; i++) r[i] = spu_splats((unsigned int)0);
    for (j=0; j<a_size;  j++, i++) r[i] = a[j];
  }
}

#endif /* _MPM_MOD_H_ */
