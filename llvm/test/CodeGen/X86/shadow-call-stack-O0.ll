; RUN: llc -O0 -verify-machineinstrs -mtriple=x86_64-unknown-linux-gnu -mattr=+reserve-r15 -o - %s | FileCheck %s

; At -O0 the return is emitted as the real RET64 (FastISel) rather than the
; X86::RET pseudo used at higher opt levels. The shadow call stack epilogue must
; still enforce the return through the SCS -- load r11 and jmp, never a plain
; retq that would trust the (possibly corrupt) on-stack return address.

; v1 (external): stack-sourced prologue, enforced return.
define i32 @v1(i32 %x) shadowcallstack {
  ; CHECK-LABEL: v1:
  ; CHECK:      movq (%rsp), %r11
  ; CHECK:      movq -8(%r15), %r11
  ; CHECK:      jmpq *%r11
  ; CHECK-NOT:  retq
  %r = add i32 %x, 1
  ret i32 %r
}

; v2 (internal, not address-taken): register-sourced prologue, enforced return.
define internal i32 @v2(i32 %x) shadowcallstack {
  ; CHECK-LABEL: v2:
  ; CHECK-NOT:  movq (%rsp), %r11
  ; CHECK:      movq -8(%r15), %r11
  ; CHECK:      jmpq *%r11
  ; CHECK-NOT:  retq
  %r = add i32 %x, 2
  ret i32 %r
}

define i32 @keepv2() shadowcallstack {
  %r = call i32 @v2(i32 3)
  ret i32 %r
}
