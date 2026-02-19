// RUN: not llvm-mc -filetype asm -triple aarch64_lfi %s 2>&1 | FileCheck %s

// Tests for PAC-related instructions that are not supported by LFI.

.arch_extension pauth

// Authenticated exception returns cannot be sandboxed.
eretaa
// CHECK: error: authenticated exception returns (ERETAA/ERETAB) are not supported by LFI

eretab
// CHECK: error: authenticated exception returns (ERETAA/ERETAB) are not supported by LFI

