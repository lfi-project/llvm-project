; RUN: opt -passes=hlfi -S < %s | FileCheck %s

; Test heap masking transformation

; CHECK: @__hlfi_sandbox_base = external global i64

; CHECK-LABEL: define i32 @test_load
define i32 @test_load(ptr %p) {
entry:
  ; CHECK: %hlfi.ptr.int = ptrtoint ptr %p to i64
  ; CHECK: %hlfi.offset = and i64 %hlfi.ptr.int, 4294967295
  ; CHECK: %hlfi.base = load i64, ptr @__hlfi_sandbox_base
  ; CHECK: %hlfi.masked.int = add i64 %hlfi.base, %hlfi.offset
  ; CHECK: %hlfi.masked.ptr = inttoptr i64 %hlfi.masked.int to ptr
  ; CHECK: %val = load i32, ptr %hlfi.masked.ptr
  %val = load i32, ptr %p
  ret i32 %val
}

; CHECK-LABEL: define void @test_store
define void @test_store(ptr %p, i32 %val) {
entry:
  ; CHECK: %hlfi.masked.ptr = inttoptr
  ; CHECK: store i32 %val, ptr %hlfi.masked.ptr
  store i32 %val, ptr %p
  ret void
}

; CHECK-LABEL: define i32 @test_atomicrmw
define i32 @test_atomicrmw(ptr %p, i32 %val) {
entry:
  ; CHECK: %hlfi.masked.ptr = inttoptr
  ; CHECK: %old = atomicrmw add ptr %hlfi.masked.ptr, i32 %val seq_cst
  %old = atomicrmw add ptr %p, i32 %val seq_cst
  ret i32 %old
}

; CHECK-LABEL: define void @test_skip_runtime
define void @test_skip_runtime() {
entry:
  ; Runtime variable accesses should NOT be masked
  ; CHECK-NOT: %hlfi.masked
  ; CHECK: load i64, ptr @__hlfi_sandbox_base
  %base = load i64, ptr @__hlfi_sandbox_base
  ret void
}

@__hlfi_sandbox_base = external global i64
