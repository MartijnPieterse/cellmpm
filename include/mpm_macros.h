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

#ifndef _MPM_MACROS_H_
#define _MPM_MACROS_H_	1

#include <spu_intrinsics.h>

/* Multi-precision math number printing function used to debug 
 * problems.
 */
#define MPM_DEBUG(_label, _array, _size) {				\
  int _i;								\
									\
  printf("%s = ", _label);						\
  for (_i=0; _i<_size; _i++) {						\
    printf("%08x:%08x:%08x:%08x ",					\
	   spu_extract((_array)[_i], 0), spu_extract((_array)[_i], 1),	\
	   spu_extract((_array)[_i], 2), spu_extract((_array)[_i], 3));	\
  }									\
  printf("\n");								\
}

#define MPM_SWAP(_x, _y, _tmp)	_tmp = _x; _x = _y; _y = _tmp;

#define MPM_SELECT(_a, _b, _selector, _type)	(_type)spu_extract(spu_sel(spu_promote((unsigned int)(_a), 0), spu_promote((unsigned int)(_b), 0), _selector), 0)

/* MPM_COUNT_LEADING_ZEROS
 * =======================
 * Count the number of leading zeros (bits) in the large unsigned 
 * integer number specified by an array of unsigned int quadwords 
 * pointed to by _a. The number of quadwords in _a is specified by
 * _asize. The count is returned in the integer parameter _count.
 */
#define MPM_COUNT_LEADING_ZEROS(_count, _a, _asize) {	\
  int _i;						\
  unsigned int _mask;					\
  vector unsigned int _vcount, _zero, _ai;		\
							\
  _count = 0;						\
  for (_i=0; _i<_asize; _i++) {				\
							\
    _ai = _a[_i];					\
    _vcount = spu_cntlz(_ai);				\
    _zero = spu_cmpeq(_ai, 0);				\
    _count += spu_extract(_vcount, 0);			\
    _mask = spu_extract(_zero, 0);			\
    _count += _mask & spu_extract(_vcount, 1);		\
    _mask &= spu_extract(_zero, 1);			\
    _count += _mask & spu_extract(_vcount, 2);		\
    _mask &= spu_extract(_zero, 2);			\
    _count += _mask & spu_extract(_vcount, 3);		\
							\
    if (spu_extract(spu_gather(_zero), 0) != 15) break; \
  }							\
}



/* MPM_ADD_PARTIAL
 * ===============
 * Perform a partial add of two unsigned 128-bit values. This is considered
 * to be a partial add because addition is performed using 4 independent
 * unsigned integer (32-bit) values. Carry propogation is maintained by
 * accumulating the carry outs in a seperate unsigned integer vector.
 *
 * For each 32-bit unsigned word:
 *	out_s = _in_a1 + in_a2
 *      out_c = _in_c + CARRY_OUT(_in_a1 + in_a2)
 */
#define MPM_ADD_PARTIAL(_out_s, _out_c, _in_a1, _in_a2, _in_c)	\
  _out_c = spu_add(_in_c, spu_genc(_in_a1, _in_a2));		\
  _out_s = spu_add(_in_a1, _in_a2);


/* MPM_ADD_PARTIAL_NO_CIN
 * ======================
 * Add partial witout a carry in.
 */
#define MPM_ADD_PARTIAL_NO_CIN(_out_s, _out_c, _in_a1, _in_a2)	\
  _out_c = spu_genc(_in_a1, _in_a2);				\
  _out_s = spu_add(_in_a1, _in_a2);


/* MPM_ADD_FULL
 * ============
 * Perform a full add of two unsigned 128-bit values, _in_a1 and _in_a2, and 
 * a carry in (_in_c). The sum is returned in _out_s and any carry out 
 * produced is returned in _out_c (in element 0, must be right shifted 
 * into element 3 for true propogation to higher order quadwords).
 *
 * _out_s = _in_a1 + _in_a2 + _in_c
 * _out_c = CARRY_OUT(_in_a1 + _in_a2 + _in_c)
 */
#define MPM_ADD_FULL(_out_s, _out_c, _in_a1, _in_a2, _in_c) {		      \
  vector unsigned int _sum, _c0, _c1, _c2, _c3, _t0, _t1, _t2;		      \
									      \
  _c0    = spu_gencx(_in_a1, _in_a2, _in_c);				      \
  _sum   = spu_addx(_in_a1, _in_a2, _in_c);				      \
									      \
  _t0     = spu_slqwbyte(_c0, 4);					      \
  _c1    = spu_genc(_sum, _t0);						      \
  _sum   = spu_add(_sum, _t0);						      \
									      \
  _t1    = spu_slqwbyte(_c1, 4);					      \
  _c2    = spu_genc(_sum, _t1);						      \
  _sum   = spu_add(_sum, _t1);						      \
									      \
  _t2    = spu_slqwbyte(_c2, 4);					      \
  _c3    = spu_genc(_sum, _t2);						      \
  _out_s = spu_add(_sum, _t2);						      \
									      \
  _out_c = spu_or(spu_or(spu_or(_c0, _c1), _c2), _c3); 			      \
}


/* MPM_ADD_FULL_NO_CARRY
 * =====================
 * Perform a full add of unsigned two 128-bit values, _in_a1 and _in_a2, 
 * without a carry in or a carry out. The sum is returned in _out_s.
 *
 * _out_s = _in_a1 + _in_a2
 */
#define MPM_ADD_FULL_NO_CARRY(_out_s, _in_a1, _in_a2) {		\
  vector unsigned int _sum, _c0, _c1, _c2, _t0, _t1, _t2;	\
								\
  _c0    = spu_genc(_in_a1, _in_a2);				\
  _sum   = spu_add(_in_a1, _in_a2);				\
								\
  _t0     = spu_slqwbyte(_c0, 4);				\
  _c1    = spu_genc(_sum, _t0);					\
  _sum   = spu_add(_sum, _t0);					\
								\
  _t1    = spu_slqwbyte(_c1, 4);				\
  _c2    = spu_genc(_sum, _t1);					\
  _sum   = spu_add(_sum, _t1);					\
								\
  _t2    = spu_slqwbyte(_c2, 4);				\
  _out_s = spu_add(_sum, _t2);					\
}


/* MPM_ADD_FULL_NO_CIN
 * ===================
 * Perform a full add of unsigned two 128-bit values, _in_a1 and _in_a2, 
 * without a carry in. The sum is returned in _out_s and the carry out
 * _out_c (in element 0, must be right shifted into element 3 for true
 * propogation to higher order quadwords).
 *
 * _out_s = _in_a1 + _in_a2
 */
#define MPM_ADD_FULL_NO_CIN(_out_s, _out_c,_in_a1, _in_a2) {		\
  vector unsigned int _sum, _c0, _c1, _c2, _c3, _t0, _t1, _t2;		\
									\
  _c0    = spu_genc(_in_a1, _in_a2);					\
  _sum   = spu_add(_in_a1, _in_a2);					\
									\
  _t0     = spu_slqwbyte(_c0, 4);					\
  _c1    = spu_genc(_sum, _t0);						\
  _sum   = spu_add(_sum, _t0);						\
									\
  _t1    = spu_slqwbyte(_c1, 4);					\
  _c2    = spu_genc(_sum, _t1);						\
  _sum   = spu_add(_sum, _t1);						\
									\
  _t2    = spu_slqwbyte(_c2, 4);					\
  _c3    = spu_genc(_sum, _t2);						\
  _out_s = spu_add(_sum, _t2);						\
									\
  _out_c = spu_or(spu_or(spu_or(_c0, _c1), _c2), _c3);			\
}


/* MPM_ADD_CARRY_PROPOGATE
 * =======================
 * Perform a 32 bit add (_in_a2) to a 128-bit value (_in_a2), 
 * The sum is returned in _out_s and the carry out _out_c.
 *
 *   _out_s = _in_a1 + _in_a2
 *
 * Note: elements 0-2 of _in_a2 are assumed to be zero.
 *       If a carry out is produced, element 0 of _out_c is
 *       set to 1.
 */
#define MPM_ADD_CARRY_PROPOGATE(_out_s, _out_c, _in_a1, _in_a2) {	\
  vector unsigned int _cin, _cout, _sum;				\
									\
  _cout  = spu_rlqwbyte(spu_genc(_in_a1, _in_a2), 4);			\
  _sum   = spu_add(_in_a1, _in_a2);					\
									\
  _cin   = spu_genc(_sum, _cout);					\
  _sum   = spu_add(_sum, _cout);					\
  _cout  = spu_rlqwbyte(_cin, 4);					\
									\
  _cin   = spu_genc(_sum, _cout);					\
  _sum   = spu_add(_sum, _cout);					\
  _cout  = spu_rlqwbyte(_cin, 4);					\
									\
  _out_c = spu_genc(_sum, _cout);					\
  _out_s = spu_add(_sum, _cout);					\
}



/* MPM_SUB_FULL
 * ============
 * Perform a full subtraction of two 128-bit values, _in_a1 and _in_a2, and 
 * a borrow in (_in_b). The sum is returned in _out_s and any borrow out 
 * produced is returned in _out_b. 
 *
 * _out_s = _in_a1 - _in_a2 + _in_b
 * _out_b = BORROW_IN(_in_a1 - _in_a2 + _in_b)
 *
 * Note: Borrows (_in_b and _out_b) must be either: 
 *      (1, 1, 1, 1) indicating no borrow in/out, or
 *      (1, 1, 1, 0) indicating a borrow in/out.
 */
#define MPM_SUB_FULL(_out_s, _out_b, _in_a1, _in_a2, _in_b) {		\
  vector unsigned int _dif, _b0, _b1, _b2, _b3, _t0, _t1, _t2;		\
  vector unsigned int  _zero = spu_splats((unsigned int)0);		\
  vector unsigned int  _one = spu_splats((unsigned int)1);		\
  vector unsigned char _shl4 = ((vector unsigned char) {		\
					   4,  5,  6,  7,		\
					   8,  9, 10, 11,		\
					   12, 13, 14, 15,		\
					   16, 17, 18, 19});		\
									\
  _b0    = spu_genbx(_in_a1, _in_a2, _in_b);				\
  _dif   = spu_subx(_in_a1, _in_a2, _in_b);				\
									\
  _t0     = spu_shuffle(_b0, _one, _shl4);				\
  _b1    = spu_genbx(_dif, _zero, _t0);					\
  _dif   = spu_subx(_dif, _zero, _t0);					\
									\
  _t1    = spu_shuffle(_b1, _one, _shl4);				\
  _b2    = spu_genbx(_dif, _zero, _t1);					\
  _dif   = spu_subx(_dif, _zero, _t1);					\
									\
  _t2    = spu_shuffle(_b2, _one, _shl4);				\
  _b3    = spu_genbx(_dif, _zero, _t2);					\
  _out_s = spu_subx(_dif, _zero, _t2);					\
									\
  _out_b = spu_shuffle(_one, 						\
		       spu_and(spu_and(spu_and(_b0, _b1), _b2), _b3),	\
		       _shl4);						\
}


/* MPM_COMPUTE_CARRY_OUT_NO_CIN
 * ============================
 * Given a quadword (_in) and a carry quadword (_in_c), calculate the 
 * carry out (_cout) resulting from the addition of the carry quadword
 * to the quadword. The carry out is contained in element 3 of _cout.
 */

#define MPM_COMPUTE_CARRY_OUT_NO_CIN(_cout, _in, _in_c) {	\
  vector unsigned int _sum;					\
  vector unsigned int _c1, _c2, _c3;				\
  vector unsigned int _t1, _t2, _t3;				\
								\
  _t1 = spu_slqwbyte(_in_c, 4);					\
  _c1 = spu_genc(_in, _t1);					\
  _sum = spu_add(_in, _t1);					\
								\
  _t2 = spu_slqwbyte(_c1, 4);					\
  _c2 = spu_genc(_sum, _t2);					\
  _sum = spu_add(_sum, _t2);					\
								\
  _t3 = spu_slqwbyte(_c2, 4);					\
  _c3 = spu_genc(_sum, _t3);					\
  _sum = spu_add(_sum, _t3);					\
  								\
  _cout = spu_add(spu_add(spu_add(_in_c, _c1), _c2), _c3);	\
}


/* MPM_ADD_CARRY_TO_SUM
 * ====================
 * Given a quadword (_in) and a carry quadword (_in_c), compute the
 * sum resulting from the addition of the carry quadword to the quadword. 
 */

#define MPM_ADD_CARRY_TO_SUM(_sum, _in, _in_c) {		\
  vector unsigned int _c1, _c2, _c3;				\
  vector unsigned int _t1, _t2, _t3;				\
								\
  _t1 = spu_slqwbyte(_in_c, 4);					\
  _c1 = spu_genc(_in, _t1);					\
  _sum = spu_add(_in, _t1);					\
								\
  _t2 = spu_slqwbyte(_c1, 4);					\
  _c2 = spu_genc(_sum, _t2);					\
  _sum = spu_add(_sum, _t2);					\
								\
  _t3 = spu_slqwbyte(_c2, 4);					\
  _c3 = spu_genc(_sum, _t3);					\
  _sum = spu_add(_sum, _t3);					\
}


/* MPM_SHL_BITS_SMALL
 * ==================
 * Shift the large number _in of _size quadwords left by the _cnt number
 * of bits. The large number _in is an array of _size quadwords. _cnt is 
 * a integer (literal or non-literal) in the range 0 to 7. The shifted 
 * number is returned to the array specified by _out. 
 *
 * NOTE: _in and _out may be the same array, but can not be overlapping.
 */
#define MPM_SHL_BITS_SMALL(_out, _in, _size, _cnt) {			\
  int _i;								\
  vector unsigned int _in0, _in1;					\
  vector unsigned int _mask;						\
									\
  _mask = (vector unsigned int){0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff};					\
  _mask = spu_slqw(_mask, (unsigned int)(_cnt));			\
  _in0  = spu_rlqw(_in[0], (int)(_cnt));				\
  for (_i=0; _i<_size-1; _i++) {					\
    _in1 = spu_rlqw(_in[_i+1], (int)(_cnt));				\
    _out[_i] = spu_sel(_in1, _in0, _mask);				\
    _in0 = _in1;							\
  }									\
  _out[_i] = spu_and(_in0, _mask);					\
}


/* MPM_SHL_BITS_LARGE
 * ==================
 * Shift the large number _in of _size quadwords left by the _cnt number
 * of bits. The large number _in is an array of _size quadwords. _cnt is 
 * an integer (either literal or non-literal) in the range 0 to 127. The 
 * shifted number is returned to the array specified by _out. 
 *
 * NOTE: _in and _out may be the same array, but can not be overlapping.
 */
#define MPM_SHL_BITS_LARGE(_out, _in, _size, _cnt, _in_0) {		\
  int _i;								\
  vector unsigned int _in0, _in1;					\
  vector unsigned int _mask;						\
									\
  _mask = spu_splats((unsigned int)-1);					\
  _mask = spu_slqwbytebc(spu_slqw(_mask,				\
				  (unsigned int)(_cnt)), 		\
			          (unsigned int)(_cnt));		\
  _in0  = spu_rlqwbytebc(spu_rlqw(_in_0, (int)(_cnt)), (int)(_cnt));	\
  for (_i=0; _i<_size-1; _i++) {					\
    _in1  = spu_rlqwbytebc(spu_rlqw(_in[_i+1], (int)(_cnt)), (int)(_cnt));\
    _out[_i] = spu_sel(_in1, _in0, _mask);				\
    _in0 = _in1;							\
  }									\
  _out[_i] = spu_and(_in0, _mask);					\
}


/* MPM_SHR_BITS_LARGE
 * ==================
 * Shift the large number _in of _size quadwords right by the_cnt number 
 * of bits. The large number _in is an array of _size quadwords. _cnt 
 * is an integer (either literal or non-literal) in the range 0 to 127. 
 * The shifted number is returned to the array specified by _out. 
 *
 * _msq is the unsigned int vector that is right shifted into the most
 * significant quadword of the resulting large number. Typically, this equal
 * to "(vector unsigend int)(0)."
 *
 * NOTE: _in and _out may be the same array, but can not be overlapping.
 */
#define MPM_SHR_BITS_LARGE(_out, _in, _size, _msq, _cnt) {			\
  int _i;									\
  int _shift;									\
  vector unsigned int _in0, _in1;						\
  vector unsigned int _mask;							\
										\
  _shift = -(_cnt);							  	\
  _mask = spu_splats((unsigned int)-1);						\
  _mask = spu_rlmaskqwbytebc(spu_rlmaskqw(_mask, _shift), _shift+7);		\
  _in0  = spu_rlqwbytebc(spu_rlqw(_msq, _shift), _shift);			\
  for (_i=0; _i<_size; _i++) {							\
    _in1  = spu_rlqwbytebc(spu_rlqw(_in[_i], _shift), _shift);			\
    _out[_i] = spu_sel(_in0, _in1, _mask);					\
    _in0 = _in1;								\
  }										\
}


/* MPM_SHR_BYTES
 * =============
 * Shift the large number _in of _size quadwords right by the _cnt number
 * of bytes. The large number _in is an array of _size quadwords. _cnt is 
 * expected to be in the range 0 to 15. The shifted number is returned to
 * the array specified by _out. 
 *
 * _msq is the unsigned int vector that is right shifted into the most
 * significant quadword of the resulting large number. Typically, this equal
 * to "(vector unsigend int)(0)."
 *
 * NOTE: _in and _out may be the same array, but can not be overlapping.
 */
#define MPM_SHR_BYTES(_out, _in, _size, _msq, _cnt) {						\
  int _i;											\
  vector unsigned int _in0, _in1;								\
  vector unsigned int _pat;									\
												\
  _pat = ((vector unsigned int) { 0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F});		\
  _pat = spu_sub(_pat, (vector unsigned int)spu_splats((unsigned char)(_cnt)));			\
												\
  _in0  = _msq;											\
  for (_i=0; _i<_size; _i++) {									\
    _in1 = _in[_i];										\
    _out[_i] = spu_shuffle(_in0, _in1, (vector unsigned char)(_pat));				\
    _in0 = _in1;										\
  }												\
}




/* MPM_DIV_ESTIMATE
 * ================
 * Compute the quotient of the double word specified by word elements
 * 0 and 1 of <_va> divided by word element 0 of <_vb>. The result
 * is returned in word element of <_vq>. 
 *
 * 	q = a / b
 * 
 * 	va = (a1, a0, 0, 0)
 * 	vb = (b0, -,  -, -)
 * 	vq = (q,  -,  -, -)
 *
 *
 * NOTES: 
 * - All parameters are unsigned int vectors
 * - _va is corrupted by this macro
 * - _vq, _va, and _vb must be unique variables
 * - b0 must be non zero and normalized (ie, msb is non-zero).
 * - a1 must be less than b0
 * - 0/0 produces 1
 * - 0/x produces 0 for all x != 0 
 */

#define MPM_DIV_ESTIMATE(_vq, _va, _vb) {				\
  int _i;								\
  unsigned int _d;							\
  vector unsigned int _vr, _of;						\
  vector unsigned int _vmask, _vcmp;					\
  vector unsigned int _vcnta;						\
									\
  _vq = spu_splats((unsigned int)0);					\
  _vmask = spu_splats((unsigned int)1);					\
									\
  /* Compute _d equal to the number of leading 0 bits in _va		\
   */									\
  _vcnta = spu_cntlz(_va);						\
  _d = spu_extract(spu_add(_vcnta, spu_and(spu_slqwbyte(_vcnta, 4),	\
					 spu_cmpeq(_va, 0))), 0);	\
									\
  /* Shift _va left by _d bits;						\
   */									\
  _va = spu_slqwbytebc(spu_slqw(_va, _d), _d);				\
									\
  /* Left shift the most significant word of _va into _vr		\
   */									\
  _vr = _va;								\
  _va = spu_slqwbyte(_va, 4);						\
									\
  /* For each of the remaining bits in _va				\
   *  - if _vr >= _vb							\
   *    then  left shift "1" into _vq					\
   *          _vr -= _vb						\
   *    else  left shift "0" into _vq					\
   *  - left shift the msb bit of _va into _vr				\
   */									\
  _of = spu_splats((unsigned int)0);					\
  for (_i=32-_d; _i>0; _i--) {						\
    _vcmp = spu_cmpgt(_vb, spu_or(_vr, _of));				\
    _vr = spu_sel(spu_sub(_vr, _vb), _vr, _vcmp);			\
    _vq = spu_or(spu_sl(_vq, 1), spu_add(_vcmp, 1));			\
    _va = spu_rl(_va, 1);						\
    _of = spu_rlmaska(_vr, -31);					\
    _vr = spu_sel(spu_sl(_vr, 1), _va, _vmask);				\
  }									\
  _vcmp = spu_cmpgt(_vb, spu_or(_vr, _of));				\
  _vq = spu_or(spu_sl(_vq, 1), spu_add(_vcmp, 1));			\
}



/* MPM_MUL_1_2
 * ===========
 * Compute the product of a single word number <_va> and a
 * double word number <_vb> to produce a triple word number
 * <_vp>. The numbers are all unsigned integer vector quawords
 * and are left justified within the quadword.
 *
 * For the purposes of documentation, the numbers are 
 * expressed as a series of unsigned shorts.
 *
 * _va = a0 a1  -  -  -  -  -  -
 * _vb = b0 b1 b2 b3  -  -  -  -
 * _vp = p0 p1 p2 p3 p4 p5  -  -
 *
 *               a1*b3
 *            a0*b3
 *            a1*b2
 *         a0*b2
 *         a1*b1
 *      a0*b1
 *      a1*b0
 * +  a0*b0
 * ---------------------------
 *       p0   p1   p2   -
 */
#define MPM_MUL_1_2(_vp, _va, _vb) {						\
  vector unsigned int _a, _b;							\
  vector unsigned int _p0, _p1, _p2, _p3;					\
  vector unsigned int _c0, _c1;							\
										\
  /* _a = a0a1 a0a1 a1a0 a1a0 */						\
  _a = spu_shuffle(_va, _va, ((vector unsigned char) {				\
					 0,1,2,3, 0,1,2,3, 2,3,0,1, 2,3,0,1}));	\
										\
  /* _b = b0b0 b2b2 b1b3 b3b1 */						\
  _b = spu_shuffle(_vb, _vb, ((vector unsigned char) {				\
					 0,1,0,1, 4,5,4,5, 6,7,2,3, 2,3,6,7}));	\
										\
  /* _p0 = a0*b0 a0*b2 a1*b3 a1*b1 						\
   * _p1 = a1*b0 a1*b2 a0*b1 a0*b3						\
   * _p2 =   0   a1*b1   0     0						\
   * _p3 = a0*b1 a0*b3   0     0						\
   */										\
  _p0 = spu_mule((vector unsigned short)(_a), 					\
		 (vector unsigned short)(_b));					\
  _p1 = spu_mulo((vector unsigned short)(_a), 					\
		 (vector unsigned short)(_b));					\
  _p2 = spu_shuffle(_p0, _p0, ((vector unsigned char) {				\
					  128,128,128,128, 0xC,0xD,0xE,0xF,	\
					  128,128,128,128, 128,128,128,128}));	\
  _p3 = spu_slqwbyte(_p1, 8);							\
										\
										\
  _c0 = spu_genc(_p0, _p2);							\
  _c1 = spu_genc(_p1, _p3);							\
										\
  /* _p0 =       a0*b0 a0*b2+a1*b1 a1*b3 a1*b1					\
   * _p1 = a1*b0+a0*b1 a1*b2+a0*b3 a0*b1 a0*b3					\
   */										\
										\
  _p0 = spu_add(_p0, _p2);							\
  _p1 = spu_add(_p1, _p3);							\
    										\
  _c0 = spu_shuffle(_c0, _c1, ((vector unsigned char) {				\
					  0x80,0x80,0x80,0x80,			\
					  0x12,0x13,0x06,0x07,			\
					  0x16,0x17,0x80,0x80,			\
					  0x80,0x80,0x80,0x80}));		\
  _p1 = spu_shuffle(_p1, _p1, ((vector unsigned char) {				\
					  0x80,0x80,0x00,0x01,			\
					  0x02,0x03,0x04,0x05,			\
					  0x06,0x07,0x80,0x80,			\
					  0x80,0x80,0x80,0x80}));		\
										\
  _c0 = spu_add(_c0, spu_genc(_p0, _p1));					\
  _p0 = spu_add(_p0, _p1);							\
										\
  /* Account for carry propogates */						\
  _c0 = spu_slqwbyte(_c0, 4);							\
  _c1 = spu_genc(_p0, _c0);							\
  										\
  _p0 = spu_add(_p0, _c0);							\
  										\
  _vp = spu_add(_p0, spu_slqwbyte(_c1, 4));					\
}




/* MPM_MUL_N_1
 * ===========
 * Compute the product of a single word number <_vb> and a
 * large number <_va> of <_asize> quardwords to produce a 
 * produce a the large number <_vp> of size <_asize>+1.
 * The result is further left shifted by <_shift> bytes where
 * <shift> is expected to be 0, 4, 8, or 12.
 * 
 * The single word number is located in the most significant
 * byte of the vector <_vb>.
 */
#define MPM_MUL_N_1(_vp, _va, _asize, _vb, _shift)								\
{														\
  int _i;													\
  vector unsigned short _a0, _a1, _b10;										\
  vector unsigned char _splat_short10 = (vector unsigned char)(spu_splats((unsigned int)0x00010203));		\
														\
  vector unsigned char _dbl_qw_shl    = ((vector unsigned char) {						\
						    0x00, 0x01, 0x02, 0x03,					\
						    0x04, 0x05, 0x06, 0x07,					\
						    0x08, 0x09, 0x0A, 0x0B,					\
						    0x0C, 0x0D, 0x0E, 0x0F});					\
  vector unsigned char _dbl_qw_shl_2  = ((vector unsigned char) {						\
						    0x02, 0x03, 0x04, 0x05,					\
						    0x06, 0x07, 0x08, 0x09,					\
						    0x0A, 0x0B, 0x0C, 0x0D,					\
						    0x0E, 0x0F, 0x10, 0x11});					\
  vector unsigned char _dbl_qw_shl_4  = ((vector unsigned char) {						\
						    0x04, 0x05, 0x06, 0x07,					\
						    0x08, 0x09, 0x0A, 0x0B,					\
						    0x0C, 0x0D, 0x0E, 0x0F,					\
						    0x10, 0x11, 0x12, 0x13});					\
  vector unsigned int _prev_s0, _prev_s1, _prev_c0, _prev_c1, _prev_p1_1;					\
  vector unsigned int _p0_0, _p0_1, _p1_0, _p1_1;								\
  vector unsigned int _p0, _c0, _c1, _s0, _s1, _ms1, _ms2, _mc1;						\
  vector unsigned int _addend, _carry;										\
														\
  _dbl_qw_shl = (vector unsigned char)(spu_add((vector unsigned int)(_dbl_qw_shl),				\
					       (vector unsigned int)spu_splats((unsigned char)(_shift))));	\
														\
  /* b10 = B1 B0 B1 B0 B1 B0 B1 B0										\
   */														\
  _b10 = (vector unsigned short)spu_shuffle(_vb, _vb, _splat_short10);						\
														\
  _carry = spu_splats((unsigned int)0);										\
  _prev_c1 = _prev_c0 = _prev_s0 = _prev_s1 = _prev_p1_1 = spu_splats((unsigned int)0);				\
														\
  for (_i=_asize-1; _i>=0; _i--) {	/* for each quadword of a */						\
    /* a0 = A7 A6 A5 A4 A3 A2 A1 A0										\
     * a1 = A6 A7 A4 A4 A2 A3 A0 A1										\
     */														\
    _a0 = (vector unsigned short)(_va[_i]);									\
    _a1 = (vector unsigned short)(spu_rl((vector unsigned int)(_a0), 16));					\
														\
    /* Compute the partial products, one most significant product						\
     * term (p1_7) is computed later since it affects the next quadword						\
     * (not this one).												\
     */														\
    _p0_0 = spu_mulo(_a0, _b10);	/* A6*B0 A4*B0 A2*B0 A0*B0 */						\
    _p1_0 = spu_mulo(_a1, _b10);	/* A7*B0 A5*B0 A3*B0 A1*B0 */						\
    _p0_1 = spu_mule(_a1, _b10);	/* A6*B1 A4*B1 A2*B1 A0*B1 */						\
    _p1_1 = spu_mule(_a0, _b10);	/* A7*B1 A5*B1 A3*B1 A1*B1 */						\
														\
    /* Combine product terms that have the same alignment. The							\
     * combined product terms consist of sums (s#) and carrys (c#).						\
     */														\
    _c1 = spu_genc(_p1_0, _p0_1);										\
    _s1 = spu_add(_p1_0, _p0_1);										\
														\
    /* Merge combined partial products with the previously iterations						\
     * combined partial products.										\
     */														\
    _ms1 = spu_shuffle(_s1,   _prev_s1,   _dbl_qw_shl_2);							\
    _mc1 = spu_shuffle(_c1,   _prev_c1,   _dbl_qw_shl_2);							\
    _ms2 = spu_shuffle(_p1_1, _prev_p1_1, _dbl_qw_shl_4);							\
														\
    /* Accumulate all the terms into the final products and carry						\
     * arrays, p and c.												\
     */														\
    _c0  = spu_genc(_p0_0, _ms2);										\
    _p0  = spu_add(_p0_0, _ms2);										\
														\
    MPM_ADD_PARTIAL(_p0, _c0, _p0, _ms1,  _c0);									\
														\
    _c0 = spu_add(_c0, _mc1);											\
														\
    /* Perform a full add of the sum of product terms and accumulated						\
     * carries. Also, perform requested shift.									\
     */														\
    _addend  = spu_shuffle(_c0, _prev_c0, _dbl_qw_shl_4);							\
    MPM_ADD_FULL(_s0, _carry, _p0, _addend, _carry);								\
    _carry = spu_rlmaskqwbyte(_carry, -12);									\
														\
    _vp[_i+1] = spu_shuffle(_s0, _prev_s0, _dbl_qw_shl);							\
														\
    /* Copy the product terms computed in this loop								\
     * for use in the next loop.										\
     */														\
    _prev_c0 = _c0;												\
    _prev_c1 = _c1;												\
    _prev_s0 = _s0;												\
    _prev_s1 = _s1;												\
    _prev_p1_1 = _p1_1;												\
  }														\
														\
  /* Align final partial products.										\
   */														\
  _ms1 = spu_rlmaskqwbyte(_prev_s1, 2-16);									\
  _ms2 = spu_rlmaskqwbyte(_prev_p1_1, 4-16);									\
														\
  /* Accumulate all the terms into the final products and carry							\
   * arrays, p and c. Note, we don't need to include the carries since						\
   * no values will carry out of the least significant word.							\
   */														\
  _p0 = spu_add(_ms1, _ms2);											\
														\
  /* Store the result */											\
  _addend  = spu_rlmaskqwbyte(_c0, -12);									\
  _s0 = spu_add(_p0, _addend);											\
														\
  _vp[0] = spu_shuffle(_s0, _prev_s0, _dbl_qw_shl);								\
}


/* MPM_MUL_N_1_NO_SHIFT
 * ====================
 * Compute the product of a single word number <_vb> and a
 * large number <_va> of <_asize> quardwords to produce a 
 * produce a the large number <_vp> of size <_asize>+1.
 * The least significant _asize quadwords are stored in
 * _vp[0] through _vp[_asize-1]. The most significant word 
 * is returned in element 3 (right-most) of _vp0.
 * 
 * The single word number is located in the most significant
 * byte of the vector <_vb>.
 */
#define MPM_MUL_N_1_NO_SHIFT(_vp0, _vp, _va, _asize, _vb)					\
{												\
  int _i;											\
  vector unsigned short _a0, _a1, _b10;								\
  vector unsigned char _splat_short10 = (vector unsigned char)(spu_splats((unsigned int)0x00010203));	\
												\
  vector unsigned char _dbl_qw_shl_2  = ((vector unsigned char) {				\
						    0x02, 0x03, 0x04, 0x05,			\
						    0x06, 0x07, 0x08, 0x09,			\
						    0x0A, 0x0B, 0x0C, 0x0D,			\
						    0x0E, 0x0F, 0x10, 0x11});			\
  vector unsigned char _dbl_qw_shl_4  = ((vector unsigned char) {				\
						    0x04, 0x05, 0x06, 0x07,			\
						    0x08, 0x09, 0x0A, 0x0B,			\
						    0x0C, 0x0D, 0x0E, 0x0F,			\
						    0x10, 0x11, 0x12, 0x13});			\
  vector unsigned int _prev_s1, _prev_c1, _prev_p1_1;						\
  vector unsigned int _p0_0, _p0_1, _p1_0, _p1_1;						\
  vector unsigned int _p0, _c0, _c1, _s0, _s1, _ms1, _ms2, _mc1;				\
  vector unsigned int _addend, _carry;								\
												\
  /* b10 = B1 B0 B1 B0 B1 B0 B1 B0								\
   */												\
  _b10 = (vector unsigned short)spu_shuffle(_vb, _vb, _splat_short10);				\
												\
  _carry = spu_splats((unsigned int)0);								\
  _prev_c1 = _prev_s1 = _prev_p1_1 = spu_splats((unsigned int)0);				\
												\
  for (_i=_asize-1; _i>=0; _i--) {	/* for each quadword of a */				\
    /* a0 = A7 A6 A5 A4 A3 A2 A1 A0								\
     * a1 = A6 A7 A4 A4 A2 A3 A0 A1								\
     */												\
    _a0 = (vector unsigned short)(_va[_i]);							\
    _a1 = (vector unsigned short)(spu_rl((vector unsigned int)(_a0), 16));			\
												\
    /* Compute the partial products, one most significant product				\
     * term (p1_7) is computed later since it affects the next quadword				\
     * (not this one).										\
     */												\
    _p0_0 = spu_mulo(_a0, _b10);	/* A6*B0 A4*B0 A2*B0 A0*B0 */				\
    _p1_0 = spu_mulo(_a1, _b10);	/* A7*B0 A5*B0 A3*B0 A1*B0 */				\
    _p0_1 = spu_mule(_a1, _b10);	/* A6*B1 A4*B1 A2*B1 A0*B1 */				\
    _p1_1 = spu_mule(_a0, _b10);	/* A7*B1 A5*B1 A3*B1 A1*B1 */				\
												\
    /* Combine product terms that have the same alignment. The					\
     * combined product terms consist of sums (s#) and carrys (c#).				\
     */												\
    _c1 = spu_genc(_p1_0, _p0_1);								\
    _s1 = spu_add(_p1_0, _p0_1);								\
												\
    /* Merge combined partial products with the previously iterations				\
     * combined partial products.								\
     */												\
    _ms1 = spu_shuffle(_s1,   _prev_s1,   _dbl_qw_shl_2);					\
    _mc1 = spu_shuffle(_c1,   _prev_c1,   _dbl_qw_shl_2);					\
    _ms2 = spu_shuffle(_p1_1, _prev_p1_1, _dbl_qw_shl_4);					\
												\
    /* Accumulate all the terms into the final products and carry				\
     * arrays, p and c.										\
     */												\
    _c0  = spu_genc(_p0_0, _ms2);								\
    _p0  = spu_add(_p0_0, _ms2);								\
												\
    MPM_ADD_PARTIAL(_p0, _c0, _p0, _ms1,  _c0);							\
												\
    _c0 = spu_add(_c0, _mc1);									\
												\
    /* Perform a full add of the sum of product terms and accumulated				\
     * carries. Also, perform requested shift.							\
     */												\
    _addend  = spu_shuffle(_c0, _prev_c0, _dbl_qw_shl_4);					\
    MPM_ADD_FULL(_s0, _carry, _p0, _addend, _carry);						\
    _carry = spu_rlmaskqwbyte(_carry, -12);							\
												\
    _vp[_i] = _s0;										\
												\
    /* Copy the product terms computed in this loop						\
     * for use in the next loop.								\
     */												\
    _prev_c1 = _c1;										\
    _prev_s1 = _s1;										\
    _prev_p1_1 = _p1_1;										\
  }												\
												\
  /* Align final partial products.								\
   */												\
  _ms1 = spu_rlmaskqwbyte(_prev_s1, 2-16);							\
  _ms2 = spu_rlmaskqwbyte(_prev_p1_1, 4-16);							\
												\
  /* Accumulate all the terms into the final products and carry					\
   * arrays, p and c. Note, we don't need to include the carries since				\
   * no values will carry out of the least significant word.					\
   */												\
  _p0 = spu_add(_ms1, _ms2);									\
												\
  /* Store the result */									\
  _addend  = spu_rlmaskqw(_carry, -12);								\
  _vp0 = spu_add(_p0, _addend);									\
}

/* MPM_MUL_N_1H
 * ============
 * Compute the product of a halfword number <_vb> and a
 * large number <_va> of <_asize> quardwords to produce a 
 * produce a the large number <_vp> if size <_asize>+1.
 * 
 * The half word number is located in the least significant
 * bytes of the vector <_vb>.
 */
#define MPM_MUL_N_1H(_vp, _va, _asize, _vb)						\
{											\
  int _i;										\
  int _dsize;										\
  vector unsigned short _a0, _b0;							\
  vector unsigned short _splat_h = spu_splats((unsigned short)0x0E0F);			\
  vector unsigned char _dbl_qw_shl_2 = ((vector unsigned char) {			\
						   0x02, 0x03, 0x04, 0x05,		\
						   0x06, 0x07, 0x08, 0x09,		\
						   0x0A, 0x0B, 0x0C, 0x0D,		\
						   0x0E, 0x0F, 0x10, 0x11});		\
  vector unsigned int _prev_p0_1, _prev_c0;						\
  vector unsigned int _p0;								\
  vector unsigned int _p0_0, _p0_1, _m0_1;						\
											\
  _dsize = _asize+1;									\
											\
  /* b0 = B0 B0 B0 B0 B0 B0 B0 B0							\
   */											\
  _b0 = (vector unsigned short)(_vb);							\
  _b0 = spu_shuffle(_b0, _b0, (vector unsigned char)(_splat_h));			\
											\
  _prev_p0_1 = spu_splats((unsigned int)0);						\
  _prev_c0   = spu_splats((unsigned int)0);						\
											\
  for (_i=_asize-1; _i>=0; _i--) {							\
    /* _a0 = A7 A6 A5 A4 A3 A2 A1 A0							\
     */											\
    _a0 = (vector unsigned short)(_va[_i]);						\
											\
    /* Compute the partial products							\
     */											\
    _p0_0 = spu_mulo(_a0, _b0);		/* A6*B0 A4*B0 A2*B0 A0*B0 */			\
    _p0_1 = spu_mule(_a0, _b0);		/* A7*B0 A5*B0 A3*B0 A1*B0 */			\
											\
    /* Align partial products and accumulate them into each of				\
     * accumulators.									\
     */											\
    _m0_1 = spu_shuffle(_p0_1, _prev_p0_1, _dbl_qw_shl_2);				\
											\
    MPM_ADD_FULL(_p0, _prev_c0, _p0_0, _m0_1, _prev_c0);				\
    _prev_c0 = spu_rlmaskqwbyte(_prev_c0, -12);						\
											\
    _vp[_i+1] = _p0;									\
    _prev_p0_1 = _p0_1;									\
  }											\
											\
  _vp[0] = spu_add(spu_rlmaskqwbyte(_prev_p0_1, -14),  _prev_c0);			\
}



/* MPM_MUL_4_4_LOW
 * ===============
 * Compute a*b for the quadword inputs a, b and return the 
 * least significant quadword of the result in r. The quad word
 * input b is pre-expanded series of half words. Assuming that 
 *   _a   = A7 A6 A5 A4 A3 A2 A1 A0
 *   _b   = B7 B6 B5 B4 B3 B2 B1 B0	
 *
 * Then the inputs are quadowrds of the form:
 *   _a0  = A7 A6 A5 A4 A3 A2 A1 A0
 *   _a1  = A6 A7 A4 A5 A2 A3 A0 A1

 *   _b10 = B1 B0 B1 B0 B1 B0 B1 B0 
 *   _b32 = B3 B2 B3 B2 B3 B2 B3 B2
 *   _b54 = B5 B4 B5 B4 B5 B4 B5 B4
 *   _b76 = B7 B6 B7 B6 B7 B6 B7 B6
 */
#define MPM_MUL_4_4_LOW(_r, _a0, _a1, _b10, _b32, _b54, _b76) {			\
  vector unsigned int _p0_0, _p1_0, _p0_1, _p1_1, _p0_2, _p1_2, _p0_3, _p1_3;	\
  vector unsigned int _p0_4, _p1_4, _p0_5, _p1_5, _p0_6, _p1_6, _p0_7;		\
  vector unsigned int _mc1, _mc2, _mc3, _mc4, _mc5, _mc6, _mc7;	       		\
  vector unsigned int _ms1, _ms2, _ms3, _ms4, _ms5, _ms6, _ms7;	       		\
  vector unsigned int _c01, _c23, _c45;						\
  vector unsigned int _s01, _s23, _s45, _s67;					\
										\
										\
  /* Compute the partial products.of a * b					\
   */										\
  _p0_0 = spu_mulo(_a0, _b10);	/* A6*B0 A4*B0 A2*B0 A0*B0 */			\
  _p1_0 = spu_mulo(_a1, _b10);	/* A7*B0 A5*B0 A3*B0 A1*B0 */			\
  _p0_1 = spu_mule(_a1, _b10);	/* A6*B1 A4*B1 A2*B1 A0*B1 */			\
  _p1_1 = spu_mule(_a0, _b10);	/* A7*B1 A5*B1 A3*B1 A1*B1 */			\
										\
  _p0_2 = spu_mulo(_a0, _b32);	/* A6*B2 A4*B2 A2*B2 A0*B2 */			\
  _p1_2 = spu_mulo(_a1, _b32);	/* A7*B2 A5*B2 A3*B2 A1*B2 */			\
  _p0_3 = spu_mule(_a1, _b32);	/* A6*B3 A4*B3 A2*B3 A0*B3 */			\
  _p1_3 = spu_mule(_a0, _b32);	/* A7*B3 A5*B3 A3*B3 A1*B3 */			\
										\
  _p0_4 = spu_mulo(_a0, _b54);	/* A6*B4 A4*B4 A2*B4 A0*B4 */			\
  _p1_4 = spu_mulo(_a1, _b54);	/* A7*B4 A5*B4 A3*B4 A1*B4 */			\
  _p0_5 = spu_mule(_a1, _b54);	/* A6*B5 A4*B5 A2*B5 A0*B5 */			\
  _p1_5 = spu_mule(_a0, _b54);	/* A7*B5 A5*B5 A3*B5 A1*B5 */			\
										\
  _p0_6 = spu_mulo(_a0, _b76);	/* A6*B6 A4*B6 A2*B6 A0*B6 */			\
  _p1_6 = spu_mulo(_a1, _b76);	/* A7*B6 A5*B6 A3*B6 A1*B6 */			\
  _p0_7 = spu_mule(_a1, _b76);	/* A6*B7 A4*B7 A2*B7 A0*B7 */			\
										\
  /* Merge the like aligned product terms					\
   */										\
   _mc1 = spu_genc(_p1_0, _p0_1);						\
   _ms1 = spu_add(_p1_0, _p0_1);						\
										\
   _mc2 = spu_genc(_p1_1, _p0_2);						\
   _ms2 = spu_add(_p1_1, _p0_2);						\
										\
   _mc3 = spu_genc(_p1_2, _p0_3);						\
   _ms3 = spu_add(_p1_2, _p0_3);						\
										\
   _mc4 = spu_genc(_p1_3, _p0_4);						\
   _ms4 = spu_add(_p1_3, _p0_4);						\
										\
   _mc5 = spu_genc(_p1_4, _p0_5);						\
   _ms5 = spu_add(_p1_4, _p0_5);						\
										\
   _mc6 = spu_genc(_p1_5, _p0_6);						\
   _ms6 = spu_add(_p1_5, _p0_6);						\
										\
   _mc7 = spu_genc(_p1_6, _p0_7);						\
   _ms7 = spu_add(_p1_6, _p0_7);						\
										\
  /* Align all the product and carry terms					\
   */										\
  _mc1 = spu_slqwbyte(_mc1, 2);							\
  _ms1 = spu_slqwbyte(_ms1, 2);							\
										\
  _mc2 = spu_slqwbyte(_mc2, 4);							\
  _ms2 = spu_slqwbyte(_ms2, 4);							\
										\
  _mc3 = spu_slqwbyte(_mc3, 6);							\
  _ms3 = spu_slqwbyte(_ms3, 6);							\
										\
  _mc4 = spu_slqwbyte(_mc4, 8);							\
  _ms4 = spu_slqwbyte(_ms4, 8);							\
										\
  _mc5 = spu_slqwbyte(_mc5, 10);						\
  _ms5 = spu_slqwbyte(_ms5, 10);						\
										\
  _mc6 = spu_slqwbyte(_mc6, 12);						\
  _ms6 = spu_slqwbyte(_ms6, 12);						\
										\
  _mc7 = spu_slqwbyte(_mc7, 14);						\
  _ms7 = spu_slqwbyte(_ms7, 14);						\
										\
										\
  /* Accumulate the product terms while at the same time			\
   * perform carry propogation.							\
   */										\
  MPM_ADD_PARTIAL(_s01, _c01, _p0_0, _ms1, _mc1);				\
										\
  _c01 = spu_slqwbyte(_c01, 4);							\
  _c23 = spu_add(_mc2, _mc3);							\
  MPM_ADD_PARTIAL(_s23, _c23, _ms2, _ms3, _c23);				\
  MPM_ADD_PARTIAL(_s23, _c23, _s23, _s01, _c23);				\
  MPM_ADD_PARTIAL(_s23, _c23, _s23, _c01, _c23);				\
										\
  _c23 = spu_slqwbyte(_c23, 4);							\
  _c45 = spu_add(_mc4, _mc5);							\
  MPM_ADD_PARTIAL(_s45, _c45, _ms4, _ms5, _c45);				\
  MPM_ADD_PARTIAL(_s45, _c45, _s45, _s23, _c45);				\
  MPM_ADD_PARTIAL(_s45, _c45, _s45, _c23, _c45);				\
										\
  _c45 = spu_slqwbyte(_c45, 4);							\
  _s67 = spu_add(_ms6, _ms7);							\
  _s67 = spu_add(_s67, _s45);							\
  _r = spu_add(_s67, _c45);							\
}




#endif /* _MPM_MACROS_H_ */



