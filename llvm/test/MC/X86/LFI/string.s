// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// stosq - sandbox RDI (destination)
stosq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosq
// CHECK-NEXT: .bundle_unlock

rep stosq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep stosq
// CHECK-NEXT: .bundle_unlock

// Different sizes
stosl
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosl
// CHECK-NEXT: .bundle_unlock

stosw
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosw
// CHECK-NEXT: .bundle_unlock

stosb
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosb
// CHECK-NEXT: .bundle_unlock

// movsq - sandbox both RSI (source) and RDI (destination)
movsq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsq
// CHECK-NEXT: .bundle_unlock

rep movsq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep movsq
// CHECK-NEXT: .bundle_unlock

// Different sizes
movsl
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsl
// CHECK-NEXT: .bundle_unlock

movsw
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsw
// CHECK-NEXT: .bundle_unlock

movsb
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsb
// CHECK-NEXT: .bundle_unlock

// cmpsq - sandbox both RSI (source) and RDI (destination)
cmpsq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: cmpsq
// CHECK-NEXT: .bundle_unlock

rep cmpsq
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep cmpsq
// CHECK-NEXT: .bundle_unlock

repne cmpsb
// CHECK:      .bundle_lock
// CHECK-NEXT: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: repne cmpsb
// CHECK-NEXT: .bundle_unlock
