; RUN: opt -passes=hlfi -S < %s | FileCheck %s

; Test forward-edge CFI transformation with x25 context

; CHECK: @__hlfi_sandbox_base = external global i64
; CHECK: @__hlfi_context = external thread_local global ptr
; CHECK: @.hlfi_cfi_table.foo = private constant ptr @foo, section ".hlfi_cfi_table"
; CHECK: @.hlfi_cfi_index.foo = private global i32 0, section ".hlfi_cfi_indices"

define void @foo() {
  ret void
}

define void @bar() {
  ret void
}

; CHECK-LABEL: define void @indirect_call
define void @indirect_call(ptr %fptr) {
entry:
  ; CHECK: %hlfi.call.idx = ptrtoint ptr %fptr to i64
  ; CHECK: %hlfi.call.masked = and i64 %hlfi.call.idx, 4095
  ; Load context pointer (x25)
  ; CHECK: %hlfi.ctx = load ptr, ptr @__hlfi_context
  ; GEP to offset 16 for CFI table
  ; CHECK: %hlfi.cfi.table.ptr = getelementptr inbounds i8, ptr %hlfi.ctx, i64 16
  ; CHECK: %hlfi.cfi.table = load ptr, ptr %hlfi.cfi.table.ptr
  ; CHECK: %hlfi.cfi.slot = getelementptr inbounds ptr, ptr %hlfi.cfi.table, i64 %hlfi.call.masked
  ; CHECK: %hlfi.cfi.target = load ptr, ptr %hlfi.cfi.slot
  ; CHECK: call void %hlfi.cfi.target()
  call void %fptr()
  ret void
}

; CHECK-LABEL: define ptr @take_address
define ptr @take_address() {
entry:
  ; CHECK: %hlfi.idx = load i32, ptr @.hlfi_cfi_index.foo
  ; CHECK: %hlfi.idx.ext = zext i32 %hlfi.idx to i64
  ; CHECK: inttoptr i64 %hlfi.idx.ext to ptr
  ret ptr @foo
}
