; RUN: split-file %s %t
; RUN: not --crash llc -mtriple=x86_64-unknown-linux-gnu -mattr=+reserve-r15 %t/ghc.ll -o - 2>&1 | FileCheck %s --check-prefix=CC
; RUN: not --crash llc -mtriple=x86_64-unknown-linux-gnu -mattr=+reserve-r15 %t/preserve-none.ll -o - 2>&1 | FileCheck %s --check-prefix=CC
; RUN: not --crash llc -mtriple=i686-unknown-linux-gnu %t/plain.ll -o - 2>&1 | FileCheck %s --check-prefix=BITS32
; RUN: not --crash llc -mtriple=x86_64-unknown-linux-gnu -mattr=+reserve-r15,+retpoline %t/plain.ll -o - 2>&1 | FileCheck %s --check-prefix=RETPOLINE
; RUN: not --crash llc -mtriple=x86_64-apple-darwin -mattr=+reserve-r15 %t/plain.ll -o - 2>&1 | FileCheck %s --check-prefix=TARGET
; RUN: not --crash llc -mtriple=x86_64-pc-windows-msvc -mattr=+reserve-r15 %t/plain.ll -o - 2>&1 | FileCheck %s --check-prefix=TARGET

; CC: shadow call stack is not supported with a calling convention that uses r15
; BITS32: shadow call stack is only supported on x86-64
; RETPOLINE: shadow call stack is not supported with indirect branch thunks
; TARGET: shadow call stack is not supported on this target

;--- ghc.ll
define ghccc void @f() shadowcallstack {
  ret void
}

;--- preserve-none.ll
define preserve_nonecc void @f() shadowcallstack {
  ret void
}

;--- plain.ll
define void @f() shadowcallstack {
  ret void
}
