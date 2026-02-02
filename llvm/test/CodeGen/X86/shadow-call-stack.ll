; RUN: llc -verify-machineinstrs -o - %s -mtriple=x86_64-linux-gnu | FileCheck %s

; Leaf function - no SCS prologue/epilogue
define void @leaf() shadowcallstack {
; CHECK-LABEL: leaf:
; CHECK-NOT:     %gs:0
; CHECK:         retq
  ret void
}

declare void @foo()

; Tail call only - no SCS (function has no regular calls)
define void @tail_call_only() shadowcallstack {
; CHECK-LABEL: tail_call_only:
; CHECK-NOT:     %gs:0
; CHECK:         jmp foo
  tail call void @foo()
  ret void
}

declare i32 @bar()

; Non-leaf function - SCS prologue and epilogue
define i32 @non_leaf() shadowcallstack {
; CHECK-LABEL: non_leaf:
; CHECK:         movq %gs:0, %r10
; CHECK-NEXT:    movq (%rsp), %r11
; CHECK-NEXT:    movq %r11, (%r10)
; CHECK-NEXT:    addq $8, %gs:0
; CHECK-NEXT:    .cfi_escape 0x16, 0x21, 0x03, 0x92, 0x21, 0x01
; CHECK:         callq bar
; CHECK:         movq %gs:0, %r10
; CHECK-NEXT:    movq -8(%r10), %r11
; CHECK-NEXT:    subq $8, %gs:0
; CHECK-NEXT:    movq %r11, (%rsp)
; CHECK-NEXT:    retq
  %res = call i32 @bar()
  %res1 = add i32 %res, 1
  ret i32 %res1
}

; Non-leaf with tail call - SCS prologue + pop before tail jump
define void @non_leaf_tail_call() shadowcallstack {
; CHECK-LABEL: non_leaf_tail_call:
; CHECK:         movq %gs:0, %r10
; CHECK-NEXT:    movq (%rsp), %r11
; CHECK-NEXT:    movq %r11, (%r10)
; CHECK-NEXT:    addq $8, %gs:0
; CHECK:         callq bar
; CHECK:         subq $8, %gs:0
; CHECK-NEXT:    jmp foo
  %res = call i32 @bar()
  tail call void @foo()
  ret void
}

; Multiple calls - still only one SCS prologue/epilogue pair
define i32 @multiple_calls() shadowcallstack {
; CHECK-LABEL: multiple_calls:
; CHECK:         movq %gs:0, %r10
; CHECK-NEXT:    movq (%rsp), %r11
; CHECK-NEXT:    movq %r11, (%r10)
; CHECK-NEXT:    addq $8, %gs:0
; CHECK:         callq bar
; CHECK:         callq bar
; CHECK:         movq %gs:0, %r10
; CHECK-NEXT:    movq -8(%r10), %r11
; CHECK-NEXT:    subq $8, %gs:0
; CHECK-NEXT:    movq %r11, (%rsp)
; CHECK-NEXT:    retq
  %res1 = call i32 @bar()
  %res2 = call i32 @bar()
  %res = add i32 %res1, %res2
  ret i32 %res
}

; nounwind - SCS prologue/epilogue but no CFI escape
define i32 @nounwind_scs() shadowcallstack nounwind {
; CHECK-LABEL: nounwind_scs:
; CHECK:         movq %gs:0, %r10
; CHECK-NOT:     .cfi_escape
; CHECK:         callq bar
; CHECK:         movq %gs:0, %r10
; CHECK:         retq
  %res = call i32 @bar()
  %res1 = add i32 %res, 1
  ret i32 %res1
}

; No SCS attribute - no instrumentation
define i32 @no_scs() {
; CHECK-LABEL: no_scs:
; CHECK-NOT:     %gs:0
; CHECK:         callq bar
; CHECK-NOT:     %gs:0
; CHECK:         retq
  %res = call i32 @bar()
  ret i32 %res
}
