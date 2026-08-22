// Check that -fno-integrated-as with LTO makes the driver pass an external
// assembler callback (this clang) to ld.lld, so that LTO code generation in
// the linker routes its output through the external assembler.

// RUN: %clang -### --target=x86_64-unknown-linux-gnu -flto -fno-integrated-as \
// RUN:   -fuse-ld=lld %s 2>&1 | FileCheck %s
// RUN: %clang -### --target=x86_64-unknown-linux-gnu -flto=thin \
// RUN:   -fno-integrated-as -fuse-ld=lld %s 2>&1 | FileCheck %s
// CHECK:      "--lto-external-assembler={{[^"]*}}clang
// CHECK-SAME: "--lto-external-assembler-arg=--target=x86_64-unknown-linux-gnu"
// CHECK-SAME: "--lto-external-assembler-arg=-fno-integrated-as"
// CHECK-SAME: "--lto-external-assembler-arg=-c"

// The integrated assembler (the default) does not use a callback.
// RUN: %clang -### --target=x86_64-unknown-linux-gnu -flto -fuse-ld=lld %s \
// RUN:   2>&1 | FileCheck %s --check-prefix=NONE
// RUN: %clang -### --target=x86_64-unknown-linux-gnu -flto -fintegrated-as \
// RUN:   -fuse-ld=lld %s 2>&1 | FileCheck %s --check-prefix=NONE
// NONE-NOT: --lto-external-assembler

// -B prefixes, target flags the assembler job derives arguments from, and
// assembler options from the link command line are forwarded so that the
// callback finds the same assembler and behaves the same way.
// RUN: %clang -### --target=x86_64-unknown-linux-gnu -flto -fno-integrated-as \
// RUN:   -fuse-ld=lld -B/some/dir -march=x86-64-v3 -Wa,--noexecstack %s \
// RUN:   2>&1 | FileCheck %s --check-prefix=FWD
// FWD:      "--lto-external-assembler-arg=-B/some/dir"
// FWD-SAME: "--lto-external-assembler-arg=-march=x86-64-v3"
// FWD-SAME: "--lto-external-assembler-arg=-Wa,--noexecstack"

// -Wa options that configure the integrated assembler and are translated to
// LTO plugin options are not forwarded to the external assembler.
// RUN: %clang -### --target=x86_64-unknown-linux-gnu -flto -fno-integrated-as \
// RUN:   -fuse-ld=lld -Wa,--crel %s 2>&1 | FileCheck %s --check-prefix=CREL
// CREL-NOT: "--lto-external-assembler-arg=-Wa,--crel"
// CREL:     "-plugin-opt=-crel"
// CREL-NOT: "--lto-external-assembler-arg=-Wa,--crel"

// Linkers using the LTO plugin interface do not support an external
// assembler; warn that the flag is ignored.
// RUN: %clang -### --target=x86_64-unknown-linux-gnu -flto -fno-integrated-as \
// RUN:   -fuse-ld=bfd %s 2>&1 | FileCheck %s --check-prefix=WARN
// WARN: warning: ignoring '-fno-integrated-as' for LTO code generation; it is only supported when linking with ld.lld
// WARN-NOT: --lto-external-assembler

// With DTLTO the backend compilations run out of process, so the callback is
// not passed (see the TODO in addLTOOptions).
// RUN: %clang -### --target=x86_64-unknown-linux-gnu -flto=thin \
// RUN:   -fno-integrated-as -fuse-ld=lld -fthinlto-distributor=fake.py %s \
// RUN:   2>&1 | FileCheck %s --check-prefix=DTLTO
// DTLTO-NOT: --lto-external-assembler

// GPU links have no external assembler to call back into, even though their
// toolchains may not report using the integrated assembler.
// RUN: %clang -### --target=amdgcn-amd-amdhsa -flto -fno-integrated-as \
// RUN:   -fuse-ld=lld -nogpulib %s 2>&1 | FileCheck %s --check-prefix=NONE
