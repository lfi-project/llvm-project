; RUN: opt -passes=hlfi -S < %s | FileCheck %s

; Test safe-stack transformation with x25 context

; CHECK: @__hlfi_context = external thread_local global ptr

; CHECK-LABEL: define void @test_array_alloca
define void @test_array_alloca() {
entry:
  ; Array allocas should be moved to unsafe stack via [x25+8]
  ; CHECK: %hlfi.ctx = load ptr, ptr @__hlfi_context
  ; CHECK: %hlfi.unsafe.sp.ptr = getelementptr inbounds i8, ptr %hlfi.ctx, i64 8
  ; CHECK: %hlfi.unsafe.sp = load ptr, ptr %hlfi.unsafe.sp.ptr
  ; CHECK: %hlfi.unsafe.sub = sub i64
  ; CHECK: %hlfi.unsafe.aligned = and i64
  ; CHECK: %hlfi.unsafe.ptr = inttoptr
  ; Store new unsafe SP back to [x25+8]
  ; CHECK: %hlfi.ctx{{[0-9]*}} = load ptr, ptr @__hlfi_context
  ; CHECK: %hlfi.ctx.field.ptr = getelementptr inbounds i8, ptr %hlfi.ctx{{[0-9]*}}, i64 8
  ; CHECK: store ptr %hlfi.unsafe.ptr, ptr %hlfi.ctx.field.ptr
  ; CHECK-NOT: alloca [100 x i8]
  %buf = alloca [100 x i8]
  %ptr = getelementptr [100 x i8], ptr %buf, i64 0, i64 0
  store i8 0, ptr %ptr
  ret void
}

; CHECK-LABEL: define i32 @test_safe_alloca
define i32 @test_safe_alloca(i32 %x) {
entry:
  ; Simple scalar allocas that are not address-taken should stay on safe stack
  ; CHECK: %a = alloca i32
  %a = alloca i32
  store i32 %x, ptr %a
  %val = load i32, ptr %a
  ret i32 %val
}

; CHECK-LABEL: define void @test_vla
define void @test_vla(i64 %n) {
entry:
  ; Variable-length allocas are unsafe
  ; CHECK: %hlfi.ctx = load ptr, ptr @__hlfi_context
  ; CHECK: %hlfi.unsafe.sp.ptr = getelementptr inbounds i8, ptr %hlfi.ctx, i64 8
  ; CHECK-NOT: alloca i8, i64 %n
  %vla = alloca i8, i64 %n
  store i8 0, ptr %vla
  ret void
}
