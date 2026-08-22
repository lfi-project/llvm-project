<!-- If you want to modify sections/contents permanently, you should modify both
ReleaseNotes.md and ReleaseNotesTemplate.txt. -->

(lld-release-release-notes)=

# lld {{ release | default("") }} Release Notes

```{contents}
:local: true
```

::::{only} PreRelease

:::{warning}
These are in-progress notes for the upcoming LLVM {{ release | default("") }} release.
Release notes for previous releases can be found on
[the Download Page](https://releases.llvm.org/download.html).
:::
::::

## Introduction

This document contains the release notes for the lld linker, release {{ release | default("") }}.
Here we describe the status of lld, including major improvements
from the previous release. All lld releases may be downloaded
from the [LLVM releases web site](https://llvm.org/releases/).

## Non-comprehensive list of changes in this release

### ELF Improvements

* `--lto-external-assembler=<tool>` and `--lto-external-assembler-arg=<arg>`
  make LTO code generation emit textual assembly and run the given tool to
  assemble each native object file, instead of using the integrated assembler.
  Clang passes these automatically when `-fno-integrated-as` is used together
  with LTO.

### Breaking changes

### COFF Improvements

### MinGW Improvements

### MachO Improvements

### WebAssembly Improvements

#### Fixes
