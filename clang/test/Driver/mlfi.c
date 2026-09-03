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

// On AArch64 the driver also renders the canonical configuration string.
// RUN: %clang -### --target=aarch64_lfi-linux-musl -mlfi=small-sandbox %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64-SMALL %s
// AARCH64-SMALL: "-target-feature" "+lfi-small-sandbox"
// AARCH64-SMALL: "-mlfi-config" "small-sandbox,sandbox-bits=32"

// RUN: %clang -### --target=aarch64_lfi-linux-musl -mlfi=no-loads,large-sandbox %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64 %s
// AARCH64: "-target-feature" "+no-lfi-loads" "-target-feature" "+lfi-large-sandbox"
// AARCH64: "-mlfi-config" "no-loads,large-sandbox,sandbox-bits=32"

// sandbox-bits is carried only in the configuration string.
// RUN: %clang -### --target=aarch64_lfi-linux-musl -mlfi=small-sandbox,sandbox-bits=24 %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64-BITS %s
// AARCH64-BITS: "-target-feature" "+lfi-small-sandbox"
// AARCH64-BITS: "-mlfi-config" "small-sandbox,sandbox-bits=24"

// The default configuration is rendered too.
// RUN: %clang -### --target=aarch64_lfi-linux-musl %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64-DEFAULT %s
// AARCH64-DEFAULT: "-mlfi-config" "sandbox-bits=32"

// The assembler job gets the configuration too.
// RUN: %clang -### --target=aarch64_lfi-linux-musl -mlfi=small-sandbox,sandbox-bits=24 -x assembler %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64-AS %s
// AARCH64-AS: -cc1as
// AARCH64-AS: "-mlfi-config" "small-sandbox,sandbox-bits=24"

// A sandbox smaller than 4 GiB requires masked control-flow guards.
// RUN: not %clang -### --target=aarch64_lfi-linux-musl -mlfi=large-sandbox,sandbox-bits=24 %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64-BAD-BITS %s
// AARCH64-BAD-BITS: error: unsupported argument 'sandbox-bits=24 requires small-sandbox' to option '-mlfi='

// gs-context is X86-only.
// RUN: not %clang -### --target=aarch64_lfi-linux-musl -mlfi=gs-context %s 2>&1 \
// RUN:   | FileCheck --check-prefix=AARCH64-BAD %s
// AARCH64-BAD: error: unsupported argument 'unknown LFI configuration token 'gs-context'' to option '-mlfi='

// -mlfi= requires an LFI target.
// RUN: not %clang -### --target=x86_64-linux-gnu -mlfi=large-sandbox %s 2>&1 \
// RUN:   | FileCheck --check-prefix=NOT-LFI %s
// NOT-LFI: error: unsupported option '-mlfi=' for target 'x86_64-unknown-linux-gnu'

// RUN: not %clang -### --target=x86_64_lfi-linux-musl -mlfi=bogus %s 2>&1 \
// RUN:   | FileCheck --check-prefix=BAD-TOKEN %s
// BAD-TOKEN: error: unsupported argument 'bogus' to option '-mlfi='
