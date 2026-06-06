; RUN: llc -mtriple=x86_64_lfi -filetype=obj < %s | llvm-objdump --no-show-raw-insn -d - | FileCheck %s

target datalayout = "e-m:e-p270:32:32-p271:32:32-p272:64:64-i64:64-i128:128-f80:128-n8:16:32:64-S128"

define void @memcpy_q(ptr %d, ptr %s) optsize {
; CHECK-LABEL: <memcpy_q>:
; CHECK:         pextq %r15, %rdi, %rdi
; CHECK-NEXT:    leaq {{.*}}(%r14,%rdi), %rdi
; CHECK-NEXT:    rep movsq (%rsi), %es:(%rdi)
  call void @llvm.memcpy.p0.p0.i64(ptr align 8 %d, ptr align 8 %s, i64 128, i1 false)
  ret void
}

define void @memset_q(ptr %d, i8 %v) optsize {
; CHECK-LABEL: <memset_q>:
; CHECK:         pextq %r15, %rdi, %rdi
; CHECK-NEXT:    leaq {{.*}}(%r14,%rdi), %rdi
; CHECK-NEXT:    rep stosb %al, %es:(%rdi)
  call void @llvm.memset.p0.i64(ptr align 8 %d, i8 %v, i64 128, i1 false)
  ret void
}

declare void @llvm.memcpy.p0.p0.i64(ptr, ptr, i64, i1)
declare void @llvm.memset.p0.i64(ptr, i8, i64, i1)
