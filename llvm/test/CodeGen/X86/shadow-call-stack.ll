; RUN: llc -verify-machineinstrs -mtriple=x86_64-unknown-linux-gnu -mattr=+reserve-r15 -o - %s | FileCheck %s
; RUN: not --crash llc -verify-machineinstrs -mtriple=x86_64-unknown-linux-gnu -o - %s 2>&1 | FileCheck %s --check-prefix=RESERVE

; RESERVE: Must reserve r15 to use shadow call stack

; Unlike AArch64, there is no leaf-function exemption: a call always spills
; the return address to the stack on x86, so even leaves are instrumented.
define void @f1() shadowcallstack {
  ; CHECK-LABEL: f1:
  ; CHECK:      movq (%rsp), %r11
  ; CHECK-NEXT: addq $8, %r15
  ; CHECK-NEXT: .cfi_escape 0x16, 0x0f, 0x02, 0x7f, 0x78
  ; CHECK-NEXT: movq %r11, -8(%r15)
  ; CHECK:      movq -8(%r15), %r11
  ; CHECK-NEXT: addq $-8, %r15
  ; CHECK-NEXT: addq $8, %rsp
  ; CHECK-NEXT: .cfi_def_cfa_offset 0
  ; CHECK-NEXT: jmpq *%r11
  ; CHECK-NOT:  retq
  ret void
}

declare void @foo()

; A tail call pops the shadow call stack entry but leaves the on-stack return
; address in place for the callee.
define void @f2() shadowcallstack {
  ; CHECK-LABEL: f2:
  ; CHECK:      movq (%rsp), %r11
  ; CHECK-NEXT: addq $8, %r15
  ; CHECK-NEXT: .cfi_escape 0x16, 0x0f, 0x02, 0x7f, 0x78
  ; CHECK-NEXT: movq %r11, -8(%r15)
  ; CHECK:      addq $-8, %r15
  ; CHECK-NEXT: jmp foo
  tail call void @foo()
  ret void
}

declare i32 @bar()

define i32 @f3() shadowcallstack {
  ; CHECK-LABEL: f3:
  ; CHECK:      movq (%rsp), %r11
  ; CHECK-NEXT: addq $8, %r15
  ; CHECK-NEXT: .cfi_escape 0x16, 0x0f, 0x02, 0x7f, 0x78
  ; CHECK-NEXT: movq %r11, -8(%r15)
  ; CHECK: callq bar
  %res = call i32 @bar()
  %res1 = add i32 %res, 1
  ; CHECK:      movq -8(%r15), %r11
  ; CHECK-NEXT: addq $-8, %r15
  ; CHECK-NEXT: addq $8, %rsp
  ; CHECK-NEXT: .cfi_def_cfa_offset 0
  ; CHECK-NEXT: jmpq *%r11
  ret i32 %res1
}

define i32 @f4() shadowcallstack {
  ; CHECK-LABEL: f4:
  %res1 = call i32 @bar()
  %res2 = call i32 @bar()
  %res3 = call i32 @bar()
  %res4 = call i32 @bar()
  %res12 = add i32 %res1, %res2
  %res34 = add i32 %res3, %res4
  %res1234 = add i32 %res12, %res34
  ; CHECK:      movq -8(%r15), %r11
  ; CHECK-NEXT: addq $-8, %r15
  ; CHECK:      jmpq *%r11
  ret i32 %res1234
}

define i32 @f5() shadowcallstack nounwind {
  ; CHECK-LABEL: f5:
  ; CHECK-NOT: .cfi_escape
  ; CHECK: jmpq *%r11
  %res = call i32 @bar()
  %res1 = add i32 %res, 1
  ret i32 %res1
}

define i32 @f6() shadowcallstack nounwind uwtable {
  ; CHECK-LABEL: f6:
  ; CHECK: .cfi_escape 0x16, 0x0f, 0x02, 0x7f, 0x78
  ; CHECK: jmpq *%r11
  %res = call i32 @bar()
  %res1 = add i32 %res, 1
  ret i32 %res1
}

; Interrupt handlers are not entered through a call and return with iret:
; they are not instrumented, but functions they call still are.
define x86_intrcc void @f_intr(ptr byval(i8) %frame) shadowcallstack {
  ; CHECK-LABEL: f_intr:
  ; CHECK-NOT: %r15
  ; CHECK: iretq
  ret void
}

declare x86_fp80 @bar_fp80()

; x87 return values ride on the FP stack; make sure the enforced return
; carries them.
define x86_fp80 @f_fp80() shadowcallstack {
  ; CHECK-LABEL: f_fp80:
  ; CHECK:      movq (%rsp), %r11
  ; CHECK-NEXT: addq $8, %r15
  ; CHECK:      movq -8(%r15), %r11
  ; CHECK-NEXT: addq $-8, %r15
  ; CHECK:      jmpq *%r11
  %res = call x86_fp80 @bar_fp80()
  ret x86_fp80 %res
}

declare i64 @bar_chain()

; The static chain (nest) is passed in r10 per the SysV ABI. The prologue
; stages the return address through r11, not r10, so the chain survives into
; the body -- no calling-convention change is needed to keep nest working.
define i64 @f_nest(ptr nest %chain) shadowcallstack {
  ; CHECK-LABEL: f_nest:
  ; CHECK:      movq (%rsp), %r11
  ; CHECK-NEXT: addq $8, %r15
  ; CHECK:      movq %r10, %{{.*}}
  ; CHECK-NOT:  movq {{.*}}, %r10
  ; CHECK:      jmpq *%r11
  %p = ptrtoint ptr %chain to i64
  %r = call i64 @bar_chain()
  %s = add i64 %p, %r
  ret i64 %s
}
