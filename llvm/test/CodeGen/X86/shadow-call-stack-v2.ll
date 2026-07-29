; RUN: llc -verify-machineinstrs -mtriple=x86_64-unknown-linux-gnu -mattr=+reserve-r15 -o - %s | FileCheck %s

; A v2 (register-sourced) function: internal, not address-taken. Its prologue
; stores %r11 -- passed by the caller -- and never reads (%rsp).
define internal i32 @v2_callee(i32 %x) shadowcallstack {
  ; CHECK-LABEL: v2_callee:
  ; CHECK-NOT:  movq (%rsp), %r11
  ; CHECK:      addq $8, %r15
  ; CHECK-NEXT: .cfi_escape
  ; CHECK-NEXT: movq %r11, -8(%r15)
  ; CHECK:      movq -8(%r15), %r11
  ; CHECK-NEXT: addq $-8, %r15
  ; CHECK:      jmpq *%r11
  %r = add i32 %x, 1
  ret i32 %r
}

declare i32 @ext(i32)

; A v1 (stack-sourced) caller. Each direct call to the v2 callee is preceded by
; a lea that loads the return address (the label after the call) into %r11.
define i32 @v1_caller(i32 %x) shadowcallstack {
  ; CHECK-LABEL: v1_caller:
  ; CHECK:      movq (%rsp), %r11
  ; CHECK:      leaq [[L1:\.L.*]](%rip), %r11
  ; CHECK-NEXT: callq v2_callee
  ; CHECK-NEXT: [[L1]]:
  %a = call i32 @v2_callee(i32 %x)
  ; Calls to external functions are unchanged: no lea.
  ; CHECK-NOT:  leaq {{.*}}, %r11
  ; CHECK:      callq ext
  %b = call i32 @ext(i32 %a)
  ret i32 %b
}

; A direct tail call to a v2 callee: the epilogue loads %r11 from the SCS before
; the (direct) jump, since the callee reads its return address from there.
define i32 @v1_tailcaller(i32 %x) shadowcallstack {
  ; CHECK-LABEL: v1_tailcaller:
  ; CHECK:      movq -8(%r15), %r11
  ; CHECK-NEXT: addq $-8, %r15
  ; CHECK:      jmp v2_callee
  %r = tail call i32 @v2_callee(i32 %x)
  ret i32 %r
}

; Internal but address-taken -> must fall back to v1 (stack-sourced), because it
; may be entered indirectly.
@fp = global ptr @addr_taken
define internal i32 @addr_taken(i32 %x) shadowcallstack {
  ; CHECK-LABEL: addr_taken:
  ; CHECK:      movq (%rsp), %r11
  ; CHECK:      jmpq *%r11
  %r = add i32 %x, 2
  ret i32 %r
}

; An indirect call never gets a lea (its target is address-taken, hence v1).
define i32 @indirect_caller(ptr %f, i32 %x) shadowcallstack {
  ; CHECK-LABEL: indirect_caller:
  ; CHECK-NOT:  leaq {{.*}}, %r11
  ; CHECK:      callq *%r{{.*}}
  %r = call i32 %f(i32 %x)
  ret i32 %r
}

; A v2 function calling another v2 function: v2 prologue in the callee, lea at
; the call site in the caller.
define internal i32 @v2_to_v2(i32 %x) shadowcallstack {
  ; CHECK-LABEL: v2_to_v2:
  ; CHECK-NOT:  movq (%rsp), %r11
  ; CHECK:      leaq {{.*}}(%rip), %r11
  ; CHECK-NEXT: callq v2_callee
  %r = call i32 @v2_callee(i32 %x)
  ret i32 %r
}
define i32 @keep_v2_to_v2_alive() shadowcallstack {
  %r = call i32 @v2_to_v2(i32 1)
  ret i32 %r
}
