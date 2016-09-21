/**************************************************
 *
 * Intrinsic functions for the MSP430 IAR Systems C/C++ Compiler
 * provided for backward compatibility with version 1 of the compiler.
 *
 * Most intrincis functions have been replaced with equivalent
 * intrinsic functions that follow the current naming convention.
 * However, there are two exceptions:
 *
 * 1) The intrinsic functions _BIx_SR and _BIx_SR_IRQ are replaced
 *    with __bix_SR_register and __bix_SR_register_on_exit. The new
 *    intrinsic functions no longer returns the previous value of the
 *    status register. You can access that value by using the
 *    intrinsic function __get_SR_register.
 *
 * 2) The intrinsic function _BIS_NMI_IE1 has been replaced with the
 *    #pragma directive bis_nmi_ie1. The reason why this no longer is
 *    an intrinsic function is that the effect was always performed,
 *    even if the intrinsic was guarded with, say, an "if"-statement.
 *
 * Copyright 2002-2003, 2006 IAR Systems. All rights reserved.
 *
 * $Revision: 1.1.2.1 $
 *
 **************************************************/

#ifndef __IN430_H
#define __IN430_H

#ifndef _SYSTEM_BUILD
  #pragma system_include
#endif

#include "intrinsics.h"

#pragma language=save
#pragma language=extended


/*
 * Deprecated intrinsic functions.
 */

#ifdef __cplusplus
extern "C"
{
#endif

  /* Deprecated, please use "__bis_SR_register" instead. (If the
   * return value of this intrinsic is used, you can also use
   * "__get_SR_register".) */
  __intrinsic unsigned short _BIS_SR(unsigned short);

  /* Deprecated, please use "__bic_SR_register" instead. (If the
   * return value of this intrinsic is used, you can also use
   * "__get_SR_register".) */
  __intrinsic unsigned short _BIC_SR(unsigned short);

  /* Deprecated, please use "__bis_SR_register_on_exit" instead. (If
   * the return value of this intrinsic is used, you can also use
   * "__get_SR_register".) */
  __intrinsic unsigned short _BIS_SR_IRQ(unsigned short);

  /* Deprecated, please use "__bic_SR_register_on_exit" instead. (If
   * the return value of this intrinsic is used, you can also use
   * "__get_SR_register".) */
  __intrinsic unsigned short _BIC_SR_IRQ(unsigned short);

  /* Deprecated, please use the #pragma directive "bis_nmi_ie1"
   * instead. */
  __intrinsic unsigned short _BIS_NMI_IE1(unsigned short);

#ifdef __cplusplus
}
#endif


/*
 * Convenience macros mapping old names to new.
 */

/* Deprecated, please use "__disable_interrupt" instead. */
#define _DINT()        __disable_interrupt()

/* Deprecated, please use "__enable_interrupt" instead. */
#define _EINT()        __enable_interrupt()

/* Deprecated, please use "__no_operation" instead. */
#define _NOP()         __no_operation()

/* Deprecated, please use "__op_code" instead. */
#define _OPC(x)        __op_code(x)

/* Deprecated, please use "__swap_bytes" instead. */
#define _SWAP_BYTES(x) __swap_bytes(x)


/* Deprecated, please use the keyword "__monitor" instead. */
#define monitor        __monitor

/* Deprecated, please use the keyword "__no_init" instead. */
#define no_init        __no_init

#pragma language=restore

#endif /* __IN430_H */
