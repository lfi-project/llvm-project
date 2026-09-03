; RUN: llc -mtriple=aarch64_lfi-linux-musl -filetype=obj %s -o - \
; RUN:   | llvm-objdump -d - | FileCheck %s

; The "lfi-config" module flag configures the MC LFI rewriter when compiling
; IR, so the configuration survives LTO. Subtarget features on the function
; drive code generation; the module flag drives the rewriter.

define i64 @get(ptr %p) "target-features"="+lfi-small-sandbox" {
; CHECK-LABEL: <get>:
; CHECK:      and x24, x0, #0xffffff
; CHECK-NEXT: ldr x0, [x27, x24]
; CHECK:      ret
  %v = load i64, ptr %p
  ret i64 %v
}

declare void @ext()

; A non-leaf function reloads LR in its epilogue, so its return carries the
; masked control-flow guard (small-sandbox form, not the fixed 4 GiB form).
define void @caller() "target-features"="+lfi-small-sandbox" {
; CHECK-LABEL: <caller>:
; CHECK:      and x24, x30, #0xffffff
; CHECK-NEXT: add x30, x27, x24
; CHECK-NEXT: ret
  call void @ext()
  ret void
}

!llvm.module.flags = !{!0}
!0 = !{i32 1, !"lfi-config", !"small-sandbox,sandbox-bits=24"}
