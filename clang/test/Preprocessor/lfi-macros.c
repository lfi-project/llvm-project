// RUN: %clang_cc1 -triple x86_64_lfi-linux-musl -target-feature +lfi-small-sandbox \
// RUN:   -dM -E %s | FileCheck --check-prefix=SMALL %s
// RUN: %clang_cc1 -triple aarch64_lfi-linux-musl -target-feature +lfi-small-sandbox \
// RUN:   -dM -E %s | FileCheck --check-prefix=SMALL %s
// SMALL-DAG: #define __LFI_LARGE_SANDBOX__ 1
// SMALL-DAG: #define __LFI_SMALL_SANDBOX__ 1

// RUN: %clang_cc1 -triple x86_64_lfi-linux-musl -target-feature +lfi-large-sandbox \
// RUN:   -dM -E %s | FileCheck --check-prefix=LARGE %s
// RUN: %clang_cc1 -triple aarch64_lfi-linux-musl -target-feature +lfi-large-sandbox \
// RUN:   -dM -E %s | FileCheck --check-prefix=LARGE %s
// LARGE: #define __LFI_LARGE_SANDBOX__ 1
// LARGE-NOT: __LFI_SMALL_SANDBOX__

// RUN: %clang_cc1 -triple x86_64_lfi-linux-musl -dM -E %s \
// RUN:   | FileCheck --check-prefix=NONE %s
// RUN: %clang_cc1 -triple aarch64_lfi-linux-musl -dM -E %s \
// RUN:   | FileCheck --check-prefix=NONE %s
// NONE: #define __LFI__ 1
// NONE-NOT: __LFI_LARGE_SANDBOX__
// NONE-NOT: __LFI_SMALL_SANDBOX__
