; REQUIRES: x86
;; Test --lto-external-assembler: LTO code generation emits textual assembly
;; and invokes the given tool as <tool> <args...> -o <object> <assembly> to
;; produce each native object file. as.py records each invocation in a marker
;; file and assembles with llvm-mc.

; RUN: rm -rf %t && split-file %s %t && cd %t
; RUN: llvm-as a.ll -o a.bc

;; Regular LTO. --save-temps=prelink retains the returned native object as
;; out.lto.o so that it can be inspected.
; RUN: ld.lld a.bc -o out --save-temps=prelink \
; RUN:   --lto-external-assembler=%python \
; RUN:   --lto-external-assembler-arg=as.py \
; RUN:   --lto-external-assembler-arg=marker.txt
; RUN: llvm-readelf -s out | FileCheck %s --check-prefix=SYM
; RUN: FileCheck %s --check-prefix=MARKER < marker.txt

; SYM: _start

; MARKER: invoked

;; lld normally enables address-significance tables for LTO output, but they
;; have no representation GNU as accepts, so they must be disabled with an
;; external assembler. This test's assembler (llvm-mc) would accept .addrsig,
;; so the section's absence shows codegen did not emit the directives.
; RUN: llvm-readelf -S out.lto.o | FileCheck %s --check-prefix=SEC

; SEC-NOT: .llvm_addrsig
; SEC:     .text
; SEC-NOT: .llvm_addrsig

;; ThinLTO: one assembler invocation per backend task. --thinlto-jobs=1 keeps
;; the concurrent marker file appends deterministic.
; RUN: sed 's/@t1/@t2/g' t1.ll > t2.ll
; RUN: opt -thinlto-bc t1.ll -o t1.bc
; RUN: opt -thinlto-bc t2.ll -o t2.bc
; RUN: rm -f marker.txt
; RUN: ld.lld t1.bc t2.bc -e t1 -o out.thin --thinlto-jobs=1 \
; RUN:   --lto-external-assembler=%python \
; RUN:   --lto-external-assembler-arg=as.py \
; RUN:   --lto-external-assembler-arg=marker.txt
; RUN: llvm-readelf -s out.thin | FileCheck %s --check-prefix=THINSYM
; RUN: FileCheck %s --check-prefix=TWICE < marker.txt

; THINSYM: t2

; TWICE-COUNT-2: invoked

;; ThinLTO caching works and caches the externally assembled object.
; RUN: ld.lld t1.bc t2.bc -e t1 -o out.thin1 --thinlto-cache-dir=cache \
; RUN:   --lto-external-assembler=%python \
; RUN:   --lto-external-assembler-arg=as.py \
; RUN:   --lto-external-assembler-arg=marker.txt
; RUN: ld.lld t1.bc t2.bc -e t1 -o out.thin2 --thinlto-cache-dir=cache \
; RUN:   --lto-external-assembler=%python \
; RUN:   --lto-external-assembler-arg=as.py \
; RUN:   --lto-external-assembler-arg=marker.txt
; RUN: cmp out.thin1 out.thin2

;; A failing assembler produces an error and retains the assembly file.
; RUN: not ld.lld a.bc -o /dev/null \
; RUN:   --lto-external-assembler=%python \
; RUN:   --lto-external-assembler-arg=fail.py 2>&1 | \
; RUN:   FileCheck %s --check-prefix=FAILAS

; FAILAS: external assembler '{{.*}}' failed with exit code 3; assembly retained at

;; Incompatible option combinations are rejected.
; RUN: not ld.lld a.bc -o /dev/null --lto-external-assembler=x \
; RUN:   --lto-emit-asm 2>&1 | FileCheck %s --check-prefix=ERR-EMIT
; RUN: not ld.lld a.bc -o /dev/null --lto-external-assembler=x \
; RUN:   --plugin-opt=dwo_dir=dwo 2>&1 | FileCheck %s --check-prefix=ERR-DWO
; RUN: not ld.lld a.bc -o /dev/null --lto-external-assembler=x \
; RUN:   --thinlto-distributor=y 2>&1 | FileCheck %s --check-prefix=ERR-DTLTO

; ERR-EMIT:  error: --lto-external-assembler may not be used with --lto-emit-asm
; ERR-DWO:   error: --lto-external-assembler may not be used with --plugin-opt=dwo_dir=
; ERR-DTLTO: error: --lto-external-assembler may not be used with --thinlto-distributor=

;--- a.ll
target datalayout = "e-m:e-p270:32:32-p271:32:32-p272:64:64-i64:64-i128:128-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

define void @_start() {
  ret void
}

;--- t1.ll
target datalayout = "e-m:e-p270:32:32-p271:32:32-p272:64:64-i64:64-i128:128-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

define void @t1() {
  ret void
}

;--- as.py
import subprocess
import sys

args = sys.argv[1:]
marker = args.pop(0)
with open(marker, "a") as f:
    f.write("invoked\n")
sys.exit(
    subprocess.call(
        ["llvm-mc", "--filetype=obj", "--triple=x86_64-unknown-linux-gnu"] + args
    )
)

;--- fail.py
import sys

sys.exit(3)
