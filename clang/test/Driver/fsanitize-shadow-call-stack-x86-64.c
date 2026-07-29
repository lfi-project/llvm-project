// Test that -fsanitize=shadow-call-stack on x86-64 requires -ffixed-r15.

// RUN: not %clang --target=x86_64-unknown-linux-gnu \
// RUN:   -fsanitize=shadow-call-stack %s -### 2>&1 \
// RUN:   | FileCheck %s --check-prefix=X86-64-SCS-NO-R15

// RUN: %clang --target=x86_64-unknown-linux-gnu \
// RUN:   -fsanitize=shadow-call-stack -ffixed-r15 %s -### 2>&1 \
// RUN:   | FileCheck %s --check-prefix=X86-64-SCS-WITH-R15

// X86-64-SCS-NO-R15: error: invalid argument '-fsanitize=shadow-call-stack' only allowed with '-ffixed-r15'
// X86-64-SCS-WITH-R15-DAG: "-fsanitize=shadow-call-stack"
// X86-64-SCS-WITH-R15-DAG: "-target-feature" "+reserve-r15"
