// RUN: llvm-mc -triple x86_64_lfi -filetype obj %s -o - | llvm-readobj -S --symbols - | FileCheck %s

// The forward-edge CFI check branches to the weak symbol _lfi_trap, which is
// emitted lazily into a COMDAT .text_lfi_trap section so all translation
// units share one copy.

callq *%rax

// CHECK:      Name: .text_lfi_trap
// CHECK:      SHF_ALLOC
// CHECK:      SHF_EXECINSTR
// CHECK:      SHF_GROUP

// CHECK:      Name: _lfi_trap
// CHECK:      Binding: Weak
