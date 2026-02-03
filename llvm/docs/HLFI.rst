=============================================
HLFI: High-Level Lightweight Fault Isolation
=============================================

.. contents::
   :local:

Introduction
============

HLFI (High-Level Lightweight Fault Isolation) is a software-based sandboxing
technique that provides memory isolation and control-flow integrity through
LLVM IR-level instrumentation. Unlike traditional LFI which operates at the
MC (machine code) level, HLFI instruments code at the IR level, enabling
more sophisticated transformations while maintaining low runtime overhead.

HLFI provides three main security mechanisms:

1. **Heap Masking**: Constrains memory accesses to a 4GB sandbox region
2. **Forward-Edge CFI**: Protects indirect calls/branches via an index-based table
3. **Safe Stack**: Separates safe and unsafe stack allocations for backward-edge protection

Target Support
==============

HLFI currently supports the AArch64 architecture with the ``aarch64_hlfi``
target triple (e.g., ``aarch64_hlfi-unknown-linux-gnu``).

Reserved Registers
==================

HLFI reserves two registers for its runtime:

- **x27**: Sandbox base address (fixed, same for all threads)
- **x25**: HLFI context pointer (per-thread context structure)

These registers are reserved by the compiler and cannot be used for general
register allocation when targeting ``aarch64_hlfi``.

HLFI Context Structure
======================

The HLFI context is a per-thread structure pointed to by x25:

.. code-block:: text

    Offset    Contents
    ------    --------
    [x25+0]   Reserved
    [x25+8]   Unsafe stack pointer
    [x25+16]  CFI table pointer
    [x25+24]  Thread pointer (TLS)

The runtime is responsible for initializing this structure and setting x25
before executing HLFI-instrumented code.

Security Mechanisms
===================

Heap Masking
------------

Heap masking constrains all memory accesses to a 4GB sandbox region. Each
pointer is masked before use:

.. code-block:: text

    masked_addr = sandbox_base + (addr & 0xFFFFFFFF)

This ensures that even if an attacker controls a pointer value, they cannot
access memory outside the sandbox region.

The HLFI pass instruments all load and store instructions (except those
accessing known-safe locations like the HLFI context or runtime globals).

Forward-Edge CFI
----------------

Forward-edge CFI protects indirect calls and branches using an index-based
table approach:

1. Each address-taken function is assigned an index
2. Function pointers are replaced with indices at compile time
3. Indirect calls look up the actual function pointer from a table:

.. code-block:: text

    // Original code:
    void (*fp)() = some_function;
    fp();

    // After HLFI transformation:
    uint32_t index = <index of some_function>;
    void (*real_fp)() = cfi_table[index];
    real_fp();

The CFI table is stored at ``[x25+16]`` and contains function pointers for
all valid indirect call targets.

**Index Assignment**: Indices are assigned by the ``llvm-hlfi patch`` post-linker
tool after linking, using position-based matching between the ``.hlfi_cfi_table``
and ``.hlfi_cfi_indices`` sections.

Safe Stack
----------

Safe stack separates stack allocations into two categories:

- **Safe stack**: Contains scalar local variables that are not address-taken
- **Unsafe stack**: Contains arrays, variable-length allocations, and address-taken variables

The unsafe stack pointer is stored at ``[x25+8]``. This separation prevents
stack buffer overflows from corrupting return addresses and other critical
stack data.

MC-Level Rewriting
==================

In addition to IR-level instrumentation, HLFI performs limited MC-level
rewriting for operations that cannot be handled at the IR level:

System Calls
------------

System calls are rewritten to go through a handler function:

.. code-block:: text

    // Original:
    svc #0

    // Rewritten:
    str x30, [sp, #-16]!     // Save LR
    ldr x17, [x27, #-8]      // Load syscall handler from [x27-8]
    blr x17                  // Call handler
    ldr x30, [sp], #16       // Restore LR

The syscall handler table is stored at a negative offset from the sandbox
base (x27).

TLS Accesses
------------

Thread-local storage accesses are redirected to the HLFI context:

.. code-block:: text

    // Original:
    mrs xN, tpidr_el0

    // Rewritten:
    ldr xN, [x25, #24]       // Load from HLFI context

This ensures TLS accesses go through the HLFI-managed thread pointer.

Usage
=====

Compilation
-----------

To compile code with HLFI instrumentation:

1. Use the ``aarch64_hlfi`` target triple
2. Run the HLFI pass on LLVM IR

.. code-block:: bash

    # Compile to LLVM IR
    clang -target aarch64_hlfi-linux-gnu -S -emit-llvm -O2 source.c -o source.ll

    # Run HLFI pass
    opt -passes=hlfi source.ll -o source_hlfi.ll

    # Compile to object file
    llc -filetype=obj source_hlfi.ll -o source.o

    # Link
    ld.lld source.o -o program

    # Patch CFI indices
    llvm-hlfi patch program

Post-Linker Tool
----------------

The ``llvm-hlfi`` tool performs post-link operations on HLFI binaries:

**dump** - Display CFI table information:

.. code-block:: bash

    llvm-hlfi dump program

**verify** - Verify CFI table correctness:

.. code-block:: bash

    llvm-hlfi verify program

**patch** - Patch CFI indices in a linked binary:

.. code-block:: bash

    llvm-hlfi patch program              # Patch in-place
    llvm-hlfi patch program -o output    # Write to new file

**build** - Build/merge CFI tables from object files:

.. code-block:: bash

    llvm-hlfi build obj1.o obj2.o -o cfi_table.bin

ELF Sections
============

HLFI uses the following ELF sections:

- ``.hlfi_cfi_table``: Contains function pointers for valid indirect call targets (8 bytes each)
- ``.hlfi_cfi_indices``: Contains index slots loaded by instrumented code (4 bytes each)

The post-linker patches the ``.hlfi_cfi_indices`` section with position-based
indices (slot N gets value N).

Runtime Requirements
====================

The HLFI runtime must:

1. Allocate and initialize the sandbox region (4GB)
2. Set x27 to the sandbox base address
3. Allocate per-thread HLFI context structures
4. Set x25 to point to the current thread's context
5. Initialize the unsafe stack pointer at ``[x25+8]``
6. Set the CFI table pointer at ``[x25+16]``
7. Set the thread pointer at ``[x25+24]``
8. Install the syscall handler at ``[x27-8]``

Limitations
===========

- Currently only supports AArch64
- Requires all code in the sandbox to be compiled with HLFI
- External library calls must go through a trampoline
- Position-based CFI index matching requires consistent ordering across compilation units

Comparison with LFI
===================

+------------------+---------------------------+---------------------------+
| Feature          | LFI                       | HLFI                      |
+==================+===========================+===========================+
| Instrumentation  | MC level                  | IR level                  |
+------------------+---------------------------+---------------------------+
| Memory safety    | Guard zones               | Heap masking (4GB sandbox)|
+------------------+---------------------------+---------------------------+
| Forward-edge CFI | Landing pads              | Index-based table         |
+------------------+---------------------------+---------------------------+
| Backward-edge    | Return address protection | Safe stack                |
+------------------+---------------------------+---------------------------+
| Reserved regs    | x27                       | x27, x25                  |
+------------------+---------------------------+---------------------------+
| Post-linking     | Not required              | Required (llvm-hlfi patch)|
+------------------+---------------------------+---------------------------+

See Also
========

- :doc:`LFI` - Traditional MC-level Lightweight Fault Isolation
