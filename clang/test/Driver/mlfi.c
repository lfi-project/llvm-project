// RUN: %clang -### --target=x86_64_lfi-linux-musl -mlfi=no-loads,no-stores %s 2>&1 \
// RUN:   | FileCheck --check-prefix=X86-MODES %s
// X86-MODES: "-target-feature" "+no-lfi-loads" "-target-feature" "+no-lfi-stores"

// RUN: %clang -### --target=x86_64_lfi-linux-musl -mlfi=large-sandbox %s 2>&1 \
// RUN:   | FileCheck --check-prefix=X86-LARGE %s
// X86-LARGE: "-target-feature" "+lfi-large-sandbox" "-target-feature" "+lfi-gs-context" "-target-feature" "+no-lfi-segue"

// RUN: %clang -### --target=x86_64_lfi-linux-musl -mlfi=gs-context,use-ret %s 2>&1 \
// RUN:   | FileCheck --check-prefix=X86-GS %s
// X86-GS: "-target-feature" "+lfi-gs-context" "-target-feature" "+lfi-use-ret" "-target-feature" "+no-lfi-segue"

// RUN: %clang -### --target=x86_64_lfi-linux-musl -mlfi=small-sandbox %s 2>&1 \
// RUN:   | FileCheck --check-prefix=X86-SMALL %s
// X86-SMALL: "-target-feature" "+lfi-small-sandbox" "-target-feature" "+lfi-gs-context" "-target-feature" "+no-lfi-segue"

// RUN: %clang -### --target=aarch64_lfi-linux-musl -mlfi=small-sandbox %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64-SMALL %s
// AARCH64-SMALL: "-target-feature" "+lfi-small-sandbox"

// RUN: %clang -### --target=aarch64_lfi-linux-musl -mlfi=no-loads,large-sandbox %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64 %s
// AARCH64: "-target-feature" "+no-lfi-loads" "-target-feature" "+lfi-large-sandbox"

// gs-context is X86-only.
// RUN: not %clang -### --target=aarch64_lfi-linux-musl -mlfi=gs-context %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64-BAD %s
// AARCH64-BAD: error: unsupported argument 'gs-context' to option '-mlfi='

// -mlfi= requires an LFI target.
// RUN: not %clang -### --target=x86_64-linux-gnu -mlfi=large-sandbox %s 2>&1 \
// RUN:   | FileCheck --check-prefix=NOT-LFI %s
// NOT-LFI: error: unsupported option '-mlfi=' for target 'x86_64-unknown-linux-gnu'

// RUN: not %clang -### --target=x86_64_lfi-linux-musl -mlfi=bogus %s 2>&1 \
// RUN:   | FileCheck --check-prefix=BAD-TOKEN %s
// BAD-TOKEN: error: unsupported argument 'bogus' to option '-mlfi='
