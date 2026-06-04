// RUN: not llvm-mc -filetype asm -triple x86_64_lfi -mattr=+lfi-gs-context %s 2>&1 | FileCheck %s

// +lfi-gs-context repurposes the %gs segment base as the context register, so
// it cannot also be used as the sandbox base for Segue. Enabling it without
// +no-lfi-segue is a configuration error.

movq %fs:0, %rax
// CHECK: error: lfi-gs-context requires Segue to be disabled (add +no-lfi-segue)
