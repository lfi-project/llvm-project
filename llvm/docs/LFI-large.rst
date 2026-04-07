=============================================
LFI Large Sandbox Variant for AArch64
=============================================

.. contents::
   :local:

Introduction
++++++++++++

This document describes the "large sandbox" variant of
:doc:`Lightweight Fault Isolation (LFI) <LFI>` for AArch64. The standard LFI
scheme restricts sandboxes to exactly 4 GiB by using the ``uxtw`` (unsigned
extend word) operation to truncate addresses to 32 bits. The large sandbox
variant generalizes this to support any power-of-2 sandbox size.

The core change is replacing the single-instruction ``add`` guard (which relies
on 32-bit truncation) with a two-instruction sequence: an ``and`` to apply a
bitmask, followed by an ``add`` to combine with the sandbox base. An additional
reserved register (the "offset register") maintains an invariant that makes
both instructions independently safe, even if a program jumps directly to the
second instruction.

Sandbox Size Mask
+++++++++++++++++

The sandbox size is determined by a bitmask equal to ``sandbox_size - 1``. For
a sandbox of size 2\ :sup:`N`, the mask has N consecutive set bits starting
from bit 0. This mask is encoded directly as a logical immediate operand in
``and`` instructions — no register is needed to hold it.

Examples:

+------------------+-----------------------+
| Sandbox Size     | Mask                  |
+------------------+-----------------------+
| 1 GiB (2^30)    | ``0x3FFFFFFF``        |
+------------------+-----------------------+
| 4 GiB (2^32)    | ``0xFFFFFFFF``        |
+------------------+-----------------------+
| 8 GiB (2^33)    | ``0x1FFFFFFFF``       |
+------------------+-----------------------+
| 64 GiB (2^36)   | ``0xFFFFFFFFF``       |
+------------------+-----------------------+
| 1 TiB (2^40)    | ``0xFFFFFFFFFF``      |
+------------------+-----------------------+

All power-of-2 masks (consecutive runs of set bits from bit 0) are valid
AArch64 logical immediates for widths 1 through 63, covering sandbox sizes
from 2 bytes to 2\ :sup:`63` bytes.

Reserved Registers
++++++++++++++++++

The large sandbox variant reserves the following registers:

* ``x24``: offset register. Always holds a value in ``[0, sandbox_size - 1]``.
* ``x25``: context register (same as standard LFI).
* ``x26``: scratch register (same as standard LFI).
* ``x27``: always holds the sandbox base address.
* ``x28``: address register. Always holds an address within the sandbox.
* ``sp``: always holds an address within the sandbox.
* ``x30``: always holds an address within the sandbox.

The offset register (``x24``) is new compared to standard LFI. Its invariant
— that it always contains a properly masked offset — is what makes the guard
sequence safe against jump-to-second-instruction attacks (see `Security
Properties`_).

Guard Instruction
+++++++++++++++++

The fundamental guard changes from one instruction to two:

+-------------------------------------------+----------------------------------------------+
| Standard LFI (4 GiB only)                | Large Sandbox                                |
+-------------------------------------------+----------------------------------------------+
| .. code-block::                           | .. code-block::                              |
|                                           |                                              |
|    add x28, x27, wN, uxtw                |    and x24, xN, #mask                        |
|                                           |    add x28, x27, x24                         |
+-------------------------------------------+----------------------------------------------+

In the standard scheme, ``add x28, x27, wN, uxtw`` truncates the 32-bit W
register (equivalent to AND with ``0xFFFFFFFF``) and adds the base in a single
instruction. In the large scheme, these two operations are split: ``and``
applies the configurable mask, and ``add`` combines with the base.

When ``#mask`` is ``0xFFFFFFFF`` (4 GiB), the large scheme produces identical
results to the standard scheme.

Security Properties
+++++++++++++++++++

Both instructions in the guard sequence are **independently safe**, even when
reached via an arbitrary jump:

* ``and x24, xN, #mask``: Regardless of ``xN``'s value, ``x24`` is always set
  to a value in ``[0, sandbox_size - 1]``.
* ``add x28, x27, x24``: Since ``x24`` is invariantly a valid offset (the
  offset register invariant), ``x28`` is always set to
  ``base + valid_offset``, which is always within the sandbox.

This avoids the need for instruction bundling. A naive two-instruction guard
like ``and x28, xN, #mask; add x28, x27, x28`` would be unsafe: if a program
jumped directly to the ``add``, ``x28`` might contain a previously computed
sandbox address rather than a bare offset, yielding
``base + (base + offset) = 2 * base + offset`` — outside the sandbox.

The offset register approach avoids this because ``x24`` can only be written by
``and x24, xN, #mask``, which always produces a valid offset. No matter where
control flow enters, ``x24`` holds a safe value.

Assembly Rewrites
+++++++++++++++++

Terminology
~~~~~~~~~~~

* ``#mask``: the sandbox size mask (``sandbox_size - 1``), encoded as a
  logical immediate.

See :doc:`LFI` for definitions of ``xN``, ``wN``, ``LDSTr``, and ``LDSTx``.

Control flow
~~~~~~~~~~~~

Indirect branches are guarded through the offset register and address register.

+--------------------+-------------------------------+
|      Original      |         Rewritten             |
+--------------------+-------------------------------+
| .. code-block::    | .. code-block::               |
|                    |                               |
|    {br,blr,ret} xN |    and x24, xN, #mask        |
|                    |    add x28, x27, x24          |
|                    |    {br,blr,ret} x28           |
+--------------------+-------------------------------+
| .. code-block::    | .. code-block::               |
|                    |                               |
|    ret             |    ret                        |
+--------------------+-------------------------------+

Memory accesses
~~~~~~~~~~~~~~~

Memory accesses use the ``[x27, x24]`` addressing mode (register-register X
form) when no immediate offset is needed. This avoids an explicit ``add``
since the addressing mode performs the base addition. When an immediate offset
is present, the guard produces a full address in ``x28``.

+---------------------------------+----------------------------------------+
|            Original             |             Rewritten                  |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTr xN, [xM]              |    and x24, xM, #mask                  |
|                                 |    LDSTr xN, [x27, x24]               |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTr xN, [xM, #I]          |    and x24, xM, #mask                  |
|                                 |    add x28, x27, x24                   |
|                                 |    LDSTr xN, [x28, #I]                |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTr xN, [xM, #I]!         |    add xM, xM, #I                      |
|                                 |    and x24, xM, #mask                  |
|                                 |    LDSTr xN, [x27, x24]               |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTr xN, [xM], #I          |    and x24, xM, #mask                  |
|                                 |    LDSTr xN, [x27, x24]               |
|                                 |    add xM, xM, #I                      |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTr xN, [xM1, xM2]        |    add x26, xM1, xM2                   |
|                                 |    and x24, x26, #mask                 |
|                                 |    LDSTr xN, [x27, x24]               |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTr xN, [xM1, xM2, MOD #I]|    add x26, xM1, xM2, MOD #I          |
|                                 |    and x24, x26, #mask                 |
|                                 |    LDSTr xN, [x27, x24]               |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTx ..., [xM]             |    and x24, xM, #mask                  |
|                                 |    add x28, x27, x24                   |
|                                 |    LDSTx ..., [x28]                   |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTx ..., [xM, #I]         |    and x24, xM, #mask                  |
|                                 |    add x28, x27, x24                   |
|                                 |    LDSTx ..., [x28, #I]               |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTx ..., [xM, #I]!        |    and x24, xM, #mask                  |
|                                 |    add x28, x27, x24                   |
|                                 |    LDSTx ..., [x28, #I]               |
|                                 |    add xM, xM, #I                      |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTx ..., [xM], #I         |    and x24, xM, #mask                  |
|                                 |    add x28, x27, x24                   |
|                                 |    LDSTx ..., [x28]                   |
|                                 |    add xM, xM, #I                      |
+---------------------------------+----------------------------------------+
| .. code-block::                 | .. code-block::                       |
|                                 |                                        |
|    LDSTx ..., [xM1], xM2       |    and x24, xM1, #mask                 |
|                                 |    add x28, x27, x24                   |
|                                 |    LDSTx ..., [x28]                   |
|                                 |    add xM1, xM1, xM2                   |
+---------------------------------+----------------------------------------+

Stack pointer modification
~~~~~~~~~~~~~~~~~~~~~~~~~~

+------------------------------+---------------------------------------+
|           Original           |             Rewritten                 |
+------------------------------+---------------------------------------+
| .. code-block::              | .. code-block::                       |
|                              |                                        |
|    mov sp, xN                |    and x24, xN, #mask                  |
|                              |    add sp, x27, x24                    |
+------------------------------+---------------------------------------+
| .. code-block::              | .. code-block::                       |
|                              |                                        |
|    {add,sub} sp, sp, {#I,xN} |    {add,sub} x26, sp, {#I,xN}         |
|                              |    and x24, x26, #mask                 |
|                              |    add sp, x27, x24                    |
+------------------------------+---------------------------------------+

Link register modification
~~~~~~~~~~~~~~~~~~~~~~~~~~

Same deferred approach as standard LFI: the guard is deferred until the next
control flow instruction for PAC compatibility. The guard uses ``x24`` as the
intermediate.

+---------------------------+-------------------------------+
|         Original          |           Rewritten           |
+---------------------------+-------------------------------+
| .. code-block::           | .. code-block::               |
|                           |                               |
|    ldr x30, [...]         |    ldr x30, [...]             |
|    ret                    |    and x24, x30, #mask        |
|                           |    add x30, x27, x24          |
|                           |    ret                        |
+---------------------------+-------------------------------+
| .. code-block::           | .. code-block::               |
|                           |                               |
|    ldp xN, x30, [...]    |    ldp xN, x30, [...]         |
|    ret                    |    and x24, x30, #mask        |
|                           |    add x30, x27, x24          |
|                           |    ret                        |
+---------------------------+-------------------------------+

Pointer Authentication Code (PAC) support
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

PAC instructions (``paciasp``, ``autiasp``) pass through unchanged, same as
standard LFI.

Authenticated returns are expanded with the large sandbox guard:

+-------------------+-------------------------------+
|     Original      |           Rewritten           |
+-------------------+-------------------------------+
| .. code-block::   | .. code-block::               |
|                   |                               |
|    retaa          |    autiasp                    |
|                   |    and x24, x30, #mask        |
|                   |    add x30, x27, x24          |
|                   |    ret                        |
+-------------------+-------------------------------+
| .. code-block::   | .. code-block::               |
|                   |                               |
|    retab          |    autibsp                    |
|                   |    and x24, x30, #mask        |
|                   |    add x30, x27, x24          |
|                   |    ret                        |
+-------------------+-------------------------------+

Authenticated branches and calls are expanded similarly:

+-------------------+-------------------------------+
|     Original      |           Rewritten           |
+-------------------+-------------------------------+
| .. code-block::   | .. code-block::               |
|                   |                               |
|    braa xN, xM    |    autia xN, xM              |
|                   |    and x24, xN, #mask         |
|                   |    add x28, x27, x24          |
|                   |    br x28                     |
+-------------------+-------------------------------+
| .. code-block::   | .. code-block::               |
|                   |                               |
|    braaz xN       |    autiza xN                  |
|                   |    and x24, xN, #mask         |
|                   |    add x28, x27, x24          |
|                   |    br x28                     |
+-------------------+-------------------------------+
| .. code-block::   | .. code-block::               |
|                   |                               |
|    blraa xN, xM   |    autia xN, xM              |
|                   |    and x24, xN, #mask         |
|                   |    add x28, x27, x24          |
|                   |    blr x28                    |
+-------------------+-------------------------------+
| .. code-block::   | .. code-block::               |
|                   |                               |
|    blraaz xN      |    autiza xN                  |
|                   |    and x24, xN, #mask         |
|                   |    add x28, x27, x24          |
|                   |    blr x28                    |
+-------------------+-------------------------------+

The ``brab``/``blrab``/``brabz``/``blrabz`` variants are handled identically,
using ``autib``/``autizb`` instead of ``autia``/``autiza``.

Authenticated exception returns (``eretaa``/``eretab``) are not supported and
will produce an error.

System instructions
~~~~~~~~~~~~~~~~~~~

+-----------------+------------------------------+
|    Original     |          Rewritten           |
+-----------------+------------------------------+
| .. code-block:: | .. code-block::              |
|                 |                              |
|    svc #0       |    mov x26, x30              |
|                 |    ldur x30, [x27, #-8]      |
|                 |    blr x30                   |
|                 |    and x24, x26, #mask       |
|                 |    add x30, x27, x24         |
+-----------------+------------------------------+

Thread-local storage
~~~~~~~~~~~~~~~~~~~~

TLS rewrites are unchanged from standard LFI.

+----------------------+-------------------------+
|       Original       |        Rewritten        |
+----------------------+-------------------------+
| .. code-block::      | .. code-block::         |
|                      |                         |
|    mrs xN, tpidr_el0 |    ldr xN, [x25, #32]  |
+----------------------+-------------------------+
| .. code-block::      | .. code-block::         |
|                      |                         |
|    msr tpidr_el0, xN |    str xN, [x25, #32]  |
+----------------------+-------------------------+

Verification
++++++++++++

A verifier for large sandbox LFI binaries checks the following rules:

1. ``x24`` is only written by ``and x24, xN, #mask`` with the correct mask.
2. ``x28`` is only written by ``add x28, x27, x24``.
3. ``x27`` is never modified (immutable base register).
4. ``x25`` is never modified (immutable context register).
5. ``sp`` is only written by ``add sp, x27, x24``.
6. ``x30`` is only written by ``add x30, x27, x24``, or by ``blr``
   (which sets ``x30`` to the return address within the sandbox).
7. Every memory access uses ``x28``, ``sp``, or ``[x27, x24]`` as the base.
8. Every ``and x24, xN, #imm`` uses the correct sandbox mask immediate.

Optimizations
+++++++++++++

Guard elimination
~~~~~~~~~~~~~~~~~

If a register is guarded multiple times in the same basic block without
modification, subsequent guards can be removed. This applies to both the
``and`` and ``add`` instructions in the guard sequence.

+-------------------------------------+-------------------------------+
|              Original               |           Rewritten           |
+-------------------------------------+-------------------------------+
| .. code-block::                     | .. code-block::               |
|                                     |                               |
|    and x24, xN, #mask              |    and x24, xN, #mask         |
|    add x28, x27, x24               |    add x28, x27, x24          |
|    ldur xM, [x28]                  |    ldur xM, [x28]             |
|    and x24, xN, #mask              |    ldur xM, [x28, #8]         |
|    add x28, x27, x24               |    ldur xM, [x28, #16]        |
|    ldur xM, [x28, #8]              |                               |
|    and x24, xN, #mask              |                               |
|    add x28, x27, x24               |                               |
|    ldur xM, [x28, #16]             |                               |
+-------------------------------------+-------------------------------+

Performance Comparison
++++++++++++++++++++++

The large sandbox variant has a consistent +1 instruction overhead compared to
standard LFI across all rewrite categories.

+--------------------------+-------------------+-------------------+-------+
| Case                     | Standard (4 GiB)  | Large Sandbox     | Delta |
+--------------------------+-------------------+-------------------+-------+
| Zero-offset memory       | 1 instr           | 2 instr           | +1    |
+--------------------------+-------------------+-------------------+-------+
| Offset memory            | 2 instr           | 3 instr           | +1    |
+--------------------------+-------------------+-------------------+-------+
| Indirect branch/call     | 2 instr           | 3 instr           | +1    |
+--------------------------+-------------------+-------------------+-------+
| LR guard                 | 1 instr           | 2 instr           | +1    |
+--------------------------+-------------------+-------------------+-------+
| SP modification          | 2 instr           | 3 instr           | +1    |
+--------------------------+-------------------+-------------------+-------+
