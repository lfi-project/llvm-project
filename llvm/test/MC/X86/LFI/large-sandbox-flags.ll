; RUN: llc -mtriple=x86_64_lfi-linux-musl -mattr=+lfi-large-sandbox -filetype=obj %s -o %t.o
; RUN: llvm-objdump -d --no-show-raw-insn %t.o | FileCheck %s

; End-to-end check of the large-sandbox flag-preservation optimization: an
; access whose EFLAGS is dead is masked with andq instead of pext.

; A lone store has no live flags across it, so it is masked with andq, not pext.
; CHECK-LABEL: <store1>:
; CHECK:      movq %rdi, %r11
; CHECK-NEXT: andq %r15, %r11
; CHECK-NEXT: movq %rsi, (%r14,%r11)
; CHECK-NOT:  pext
define void @store1(ptr %p, i64 %v) {
  store i64 %v, ptr %p
  ret void
}
