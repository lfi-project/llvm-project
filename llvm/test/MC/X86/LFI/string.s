// RUN: llvm-mc -filetype asm -triple x86_64_lfi %s | FileCheck %s

// stosq - sandbox RDI (destination)
stosq
// CHECK: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosq

rep stosq
// CHECK: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep stosq

// Different sizes
stosl
// CHECK: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosl

stosw
// CHECK: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosw

stosb
// CHECK: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: stosb

// movsq - sandbox both RSI (source) and RDI (destination)
movsq
// CHECK: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsq

rep movsq
// CHECK: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep movsq

// Different sizes
movsl
// CHECK: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsl

movsw
// CHECK: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsw

movsb
// CHECK: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: movsb

// cmpsq - sandbox both RSI (source) and RDI (destination)
cmpsq
// CHECK: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: cmpsq

rep cmpsq
// CHECK: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: rep cmpsq

repne cmpsb
// CHECK: movl %esi, %esi
// CHECK-NEXT: leaq (%r14,%rsi), %rsi
// CHECK-NEXT: movl %edi, %edi
// CHECK-NEXT: leaq (%r14,%rdi), %rdi
// CHECK-NEXT: repne cmpsb
