; RUN: llc -mtriple=x86_64_lfi-linux-musl -mattr=+lfi-small-sandbox -filetype=obj %s -o %t.o
; RUN: llvm-objdump -d --no-show-raw-insn %t.o | FileCheck %s

; End-to-end check of the small-sandbox displacement bound. A 512KiB frame puts
; a local beyond the 128KiB guard region from %rsp, so its access folds the
; displacement before the mask, while a nearby local keeps its plain
; %rsp-relative form. Prefix padding may add a %cs: override.

; CHECK-LABEL: <bigframe>:
; CHECK:      subq $0x80008, %rsp
; CHECK-NEXT: pextq %r15, %rsp, %rsp
; CHECK-NEXT: leaq {{(%cs:)?}}(%rsp,%r14), %rsp
; CHECK-NEXT: leaq {{(%cs:)?}}0x4e200(%rsp), %r11
; CHECK-NEXT: andq %r15, %r11
; CHECK-NEXT: movq %rdi, {{(%cs:)?}}(%r14,%r11)
; CHECK:      movq %rdi, {{(%cs:)?}}0x38(%rsp)
define void @bigframe(i64 %v) {
  %buf = alloca [65536 x i64], align 16
  %far = getelementptr [65536 x i64], ptr %buf, i64 0, i64 40000
  store volatile i64 %v, ptr %far
  %near = getelementptr [65536 x i64], ptr %buf, i64 0, i64 7
  store volatile i64 %v, ptr %near
  call void @use(ptr %buf)
  ret void
}

declare void @use(ptr)
