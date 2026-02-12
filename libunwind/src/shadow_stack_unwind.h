//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//
//===----------------------------------------------------------------------===//

#ifndef LIBUNWIND_SHADOW_STACK_UNWIND_H
#define LIBUNWIND_SHADOW_STACK_UNWIND_H

#include "libunwind.h"

// Currently, CET is implemented on some ELF x86 platforms.
#if defined(__CET__) && defined(__SHSTK__)
#define _LIBUNWIND_USE_CET 1
#endif

#if defined(_LIBUNWIND_USE_CET)
#include <cet.h>
#include <immintrin.h>

#define _LIBUNWIND_POP_SHSTK_SSP(x)                                            \
  do {                                                                         \
    unsigned long ssp = _get_ssp();                                            \
    if (ssp != 0) {                                                            \
      unsigned int tmp = (x);                                                  \
      while (tmp > 255) {                                                      \
        _inc_ssp(255);                                                         \
        tmp -= 255;                                                            \
      }                                                                        \
      _inc_ssp(tmp);                                                           \
    }                                                                          \
  } while (0)
#endif

// On AArch64 we use _LIBUNWIND_USE_GCS to indicate that GCS is supported. We
// need to guard any use of GCS instructions with __chkfeat though, as GCS may
// not be enabled.
#if defined(_LIBUNWIND_TARGET_AARCH64) && defined(__ARM_FEATURE_GCS_DEFAULT)
#include <arm_acle.h>

// We can only use GCS if arm_acle.h defines the GCS intrinsics.
#ifdef _CHKFEAT_GCS
#define _LIBUNWIND_USE_GCS 1
#endif

#define _LIBUNWIND_POP_SHSTK_SSP(x)                                            \
  do {                                                                         \
    if (__chkfeat(_CHKFEAT_GCS)) {                                             \
      unsigned tmp = (x);                                                      \
      while (tmp--)                                                            \
        __gcspopm();                                                           \
    }                                                                          \
  } while (0)

#endif

// LFI software shadow call stack. The SCS pointer is stored at 16(%r15).
// During unwinding, we need to pop entries from the SCS to keep it in sync.
// This is done via a runtime call that adjusts the SCS pointer.
#if defined(__LFI__) && defined(_LIBUNWIND_TARGET_X86_64)
#define _LIBUNWIND_USE_LFI_SCS 1

// LFI runtime call offsets from base register (%r14):
//   32: SCS save    — saves current SCS pointer, returns index in %rax
//   40: SCS restore — restores SCS pointer from index in %edi
//   48: SCS unwind  — pops n entries from SCS (n in %edi)

// Pops `n` entries from the LFI shadow call stack by calling the runtime's
// SCS unwind entry point at offset 48 from the LFI base register (%r14).
static void _lfi_scs_unwind(unsigned int n) {
  __asm__ volatile(".lfi_rewrite_disable\n\t"
                   "leaq 1f(%%rip), %%r11\n\t"
                   "jmpq *48(%%r14)\n\t"
                   "1:\n\t"
                   ".lfi_rewrite_enable\n\t"
                   :
                   : "D"(n)
                   : "r11", "memory");
}

#define _LIBUNWIND_POP_SHSTK_SSP(x)                                           \
  do {                                                                         \
    unsigned int _n = (x);                                                     \
    if (_n > 0)                                                                \
      _lfi_scs_unwind(_n);                                                     \
  } while (0)
#endif

#if defined(_LIBUNWIND_USE_CET) || defined(_LIBUNWIND_USE_GCS) || defined(_LIBUNWIND_USE_LFI_SCS)
extern void *__libunwind_shstk_get_registers(unw_cursor_t *);
extern void *__libunwind_shstk_get_jump_target(void);
#endif

#endif
