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
#include "mpm_macros.h"

#ifndef _MPM_SUB_H_
#define _MPM_SUB_H_	1


/* mpm_sub
 * -------
 * Generic routine that subtract two large numbers pointed to by <a> and <b> 
 * of <size> quadwords. The result is stored in memory pointed to by <s> (of 
 * the same size), and a borrow out quadword is returned. A borrow out of 
 * (0,0,0,1) indicates that no borrow out occurred. A borrow out of (0,0,0,0) 
 * indicates a borrow resulted.
 *
 *	<s> = <a> - <b>
 */
static __inline vector unsigned int _mpm_sub(vector unsigned int *s, const vector unsigned int *a, const vector unsigned int *b, int size)
{
  vector unsigned int borrow;

  borrow = spu_splats((unsigned int)1);
  while (size--) {
    MPM_SUB_FULL(s[size], borrow, a[size], b[size], borrow);
  }
  return (borrow);
}

#endif /* _MPM_SUB_H_ */
