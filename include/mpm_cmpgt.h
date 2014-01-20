/* --------------------------------------------------------------  */
/* (C)Copyright 2001,2006,                                         */
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

#ifndef _MPM_CMPGT_H_
#define _MPM_CMPGT_H_	1

#include <spu_intrinsics.h>

/* mpm_cmpgt
 * ----------
 * Generic routine that compares two unsigned large numbers pointed to
 * by <a> and <b> of <size> quadwords. If <a> is greater than <b>, then
 * all 1's is returned, else 0 is returned.
 */
static __inline unsigned int _mpm_cmpgt(const vector unsigned int *a, const vector unsigned int *b, int size)
{
  int i;
  vector unsigned int ai, bi;
  vector unsigned int gt, lt, gt_bigger, lt_bigger;

  for (i=0; i<size; i++) {
    ai = a[i];
    bi = b[i];

    gt = spu_gather(spu_cmpgt(ai, bi));
    lt = spu_gather(spu_cmpgt(bi, ai));

    gt_bigger = spu_cmpgt(gt, lt);
    lt_bigger = spu_cmpgt(lt, gt);

    if (spu_extract(spu_or(gt_bigger, lt_bigger), 0)) {
      return (spu_extract(gt_bigger, 0));
    }
  }
  return (0);
}

#endif /* _MPM_CMPGT_H_ */
