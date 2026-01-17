// RUN: not llvm-mc -filetype asm -triple aarch64_lfi %s 2>&1 | FileCheck %s

// Tests for PAC-related instructions that are not supported by LFI.

.arch_extension pauth

// Authenticated returns combine return with PAC authentication and cannot be sandboxed.
retaa
// CHECK: error: authenticated returns (RETAA/RETAB/ERETAA/ERETAB) are not supported by LFI

retab
// CHECK: error: authenticated returns (RETAA/RETAB/ERETAA/ERETAB) are not supported by LFI

eretaa
// CHECK: error: authenticated returns (RETAA/RETAB/ERETAA/ERETAB) are not supported by LFI

eretab
// CHECK: error: authenticated returns (RETAA/RETAB/ERETAA/ERETAB) are not supported by LFI

// Authenticated branches combine branch with PAC authentication.
braa x0, x1
// CHECK: error: authenticated branches (BRAA/BRAB/BLRAA/BLRAB) are not supported by LFI

braaz x0
// CHECK: error: authenticated branches (BRAA/BRAB/BLRAA/BLRAB) are not supported by LFI

brab x0, x1
// CHECK: error: authenticated branches (BRAA/BRAB/BLRAA/BLRAB) are not supported by LFI

brabz x0
// CHECK: error: authenticated branches (BRAA/BRAB/BLRAA/BLRAB) are not supported by LFI

blraa x0, x1
// CHECK: error: authenticated branches (BRAA/BRAB/BLRAA/BLRAB) are not supported by LFI

blraaz x0
// CHECK: error: authenticated branches (BRAA/BRAB/BLRAA/BLRAB) are not supported by LFI

blrab x0, x1
// CHECK: error: authenticated branches (BRAA/BRAB/BLRAA/BLRAB) are not supported by LFI

blrabz x0
// CHECK: error: authenticated branches (BRAA/BRAB/BLRAA/BLRAB) are not supported by LFI

