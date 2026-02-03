//===-- llvm-hlfi.cpp - HLFI Post-Linker Tool -----------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This tool performs post-link operations for HLFI (High-Level Lightweight
// Fault Isolation) binaries, including:
//
// 1. dump   - Dump CFI table information from an ELF binary
// 2. verify - Verify that all indirect call targets are in the CFI table
// 3. build  - Build/merge CFI tables from object files
//
//===----------------------------------------------------------------------===//

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Object/ELF.h"
#include "llvm/Object/ELFObjectFile.h"
#include "llvm/Object/ObjectFile.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/FileOutputBuffer.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/InitLLVM.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/WithColor.h"
#include "llvm/Support/raw_ostream.h"

#include <algorithm>
#include <map>
#include <set>
#include <vector>

using namespace llvm;
using namespace llvm::object;

// Global options
static cl::OptionCategory HLFICategory("HLFI Options");

static cl::SubCommand DumpCmd("dump", "Dump CFI table information");
static cl::SubCommand VerifyCmd("verify", "Verify CFI table correctness");
static cl::SubCommand BuildCmd("build", "Build CFI table from object files");
static cl::SubCommand PatchCmd("patch", "Patch CFI indices in linked binary");

// Dump subcommand options
static cl::opt<std::string> DumpInput(cl::Positional, cl::desc("<input file>"),
                                      cl::Required, cl::sub(DumpCmd));

// Verify subcommand options
static cl::opt<std::string> VerifyInput(cl::Positional,
                                        cl::desc("<input file>"), cl::Required,
                                        cl::sub(VerifyCmd));

// Build subcommand options
static cl::list<std::string> BuildInputs(cl::Positional,
                                         cl::desc("<input files>"),
                                         cl::OneOrMore, cl::sub(BuildCmd));
static cl::opt<std::string> BuildOutput("o", cl::desc("Output file"),
                                        cl::value_desc("filename"),
                                        cl::init("hlfi_cfi_table.bin"),
                                        cl::sub(BuildCmd));

// Patch subcommand options
static cl::opt<std::string> PatchInput(cl::Positional, cl::desc("<input file>"),
                                       cl::Required, cl::sub(PatchCmd));
static cl::opt<std::string> PatchOutput("o", cl::desc("Output file (default: in-place)"),
                                        cl::value_desc("filename"),
                                        cl::init(""), cl::sub(PatchCmd));

static ExitOnError ExitOnErr;

// CFI table entry structure
struct CFIEntry {
  uint64_t Index;
  uint64_t Address;
  std::string SymbolName;
};

// Find a symbol by name in an ELF file
template <class ELFT>
static Expected<uint64_t> findSymbolAddress(const ELFFile<ELFT> &ELF,
                                            StringRef Name) {
  auto SectionsOrErr = ELF.sections();
  if (!SectionsOrErr)
    return SectionsOrErr.takeError();

  for (const auto &Sec : *SectionsOrErr) {
    if (Sec.sh_type != ELF::SHT_SYMTAB && Sec.sh_type != ELF::SHT_DYNSYM)
      continue;

    auto SymbolsOrErr = ELF.symbols(&Sec);
    if (!SymbolsOrErr)
      continue;

    auto StrTabOrErr = ELF.getStringTableForSymtab(Sec);
    if (!StrTabOrErr)
      continue;

    for (const auto &Sym : *SymbolsOrErr) {
      auto NameOrErr = Sym.getName(*StrTabOrErr);
      if (!NameOrErr)
        continue;
      if (*NameOrErr == Name)
        return Sym.st_value;
    }
  }

  return createStringError(inconvertibleErrorCode(),
                           "Symbol not found: " + Name.str());
}

// Get symbol name for an address
template <class ELFT>
static std::string getSymbolNameForAddress(const ELFFile<ELFT> &ELF,
                                           uint64_t Address) {
  auto SectionsOrErr = ELF.sections();
  if (!SectionsOrErr)
    return "";

  for (const auto &Sec : *SectionsOrErr) {
    if (Sec.sh_type != ELF::SHT_SYMTAB && Sec.sh_type != ELF::SHT_DYNSYM)
      continue;

    auto SymbolsOrErr = ELF.symbols(&Sec);
    if (!SymbolsOrErr)
      continue;

    auto StrTabOrErr = ELF.getStringTableForSymtab(Sec);
    if (!StrTabOrErr)
      continue;

    for (const auto &Sym : *SymbolsOrErr) {
      if (Sym.st_value == Address && Sym.getType() == ELF::STT_FUNC) {
        auto NameOrErr = Sym.getName(*StrTabOrErr);
        if (NameOrErr)
          return std::string(*NameOrErr);
      }
    }
  }

  return "";
}

// Find section by name
template <class ELFT>
static Expected<const typename ELFT::Shdr *>
findSection(const ELFFile<ELFT> &ELF, StringRef Name) {
  auto SectionsOrErr = ELF.sections();
  if (!SectionsOrErr)
    return SectionsOrErr.takeError();

  for (const auto &Sec : *SectionsOrErr) {
    auto NameOrErr = ELF.getSectionName(Sec);
    if (!NameOrErr)
      continue;
    if (*NameOrErr == Name)
      return &Sec;
  }

  return createStringError(inconvertibleErrorCode(),
                           "Section not found: " + Name.str());
}

// Read CFI table entries from an ELF file
template <class ELFT>
static Expected<std::vector<CFIEntry>>
readCFITable(const ELFFile<ELFT> &ELF) {
  std::vector<CFIEntry> Entries;
  constexpr size_t PtrSize = sizeof(typename ELFT::Addr);

  // First, try to find the .hlfi_cfi_table section directly
  // This works with the HLFI pass which creates individual entries
  // in this section
  auto SectionsOrErr = ELF.sections();
  if (!SectionsOrErr)
    return SectionsOrErr.takeError();

  for (const auto &Sec : *SectionsOrErr) {
    auto NameOrErr = ELF.getSectionName(Sec);
    if (!NameOrErr)
      continue;

    if (*NameOrErr == ".hlfi_cfi_table") {
      auto DataOrErr = ELF.getSectionContents(Sec);
      if (!DataOrErr)
        return DataOrErr.takeError();

      ArrayRef<uint8_t> Data = *DataOrErr;
      uint64_t Offset = 0;
      uint64_t Index = 0;

      // Read all function pointers from the section
      while (Offset + PtrSize <= Data.size()) {
        uint64_t FuncAddr;
        if constexpr (ELFT::Endianness == llvm::endianness::little) {
          FuncAddr = support::endian::read<typename ELFT::Addr,
                                           llvm::endianness::little>(
              Data.data() + Offset);
        } else {
          FuncAddr = support::endian::read<typename ELFT::Addr,
                                           llvm::endianness::big>(
              Data.data() + Offset);
        }

        // Skip zero entries (padding or uninitialized)
        if (FuncAddr != 0) {
          CFIEntry Entry;
          Entry.Index = Index;
          Entry.Address = FuncAddr;
          Entry.SymbolName = getSymbolNameForAddress(ELF, FuncAddr);
          Entries.push_back(Entry);
        }

        Offset += PtrSize;
        Index++;
      }
      return Entries;
    }
  }

  // Fallback: try to find __hlfi_cfi_table symbol (for alternate table format)
  auto TableAddrOrErr = findSymbolAddress(ELF, "__hlfi_cfi_table");
  if (!TableAddrOrErr) {
    // No CFI table found - not necessarily an error
    consumeError(TableAddrOrErr.takeError());
    return Entries;
  }
  uint64_t TableAddr = *TableAddrOrErr;

  // Find the section containing the table
  for (const auto &Sec : *SectionsOrErr) {
    if (TableAddr >= Sec.sh_addr &&
        TableAddr < Sec.sh_addr + Sec.sh_size) {
      auto DataOrErr = ELF.getSectionContents(Sec);
      if (!DataOrErr)
        return DataOrErr.takeError();

      ArrayRef<uint8_t> Data = *DataOrErr;
      uint64_t Offset = TableAddr - Sec.sh_addr;

      // Read function pointers until we hit zero or end of section
      uint64_t Index = 0;
      while (Offset + PtrSize <= Data.size()) {
        uint64_t FuncAddr;
        if constexpr (ELFT::Endianness == llvm::endianness::little) {
          FuncAddr = support::endian::read<typename ELFT::Addr,
                                           llvm::endianness::little>(
              Data.data() + Offset);
        } else {
          FuncAddr = support::endian::read<typename ELFT::Addr,
                                           llvm::endianness::big>(
              Data.data() + Offset);
        }

        if (FuncAddr == 0)
          break;

        CFIEntry Entry;
        Entry.Index = Index;
        Entry.Address = FuncAddr;
        Entry.SymbolName = getSymbolNameForAddress(ELF, FuncAddr);
        Entries.push_back(Entry);

        Offset += PtrSize;
        Index++;
      }
      break;
    }
  }

  return Entries;
}

// Collect all function symbols from an ELF file
template <class ELFT>
static Expected<std::vector<std::pair<std::string, uint64_t>>>
collectFunctionSymbols(const ELFFile<ELFT> &ELF) {
  std::vector<std::pair<std::string, uint64_t>> Functions;

  auto SectionsOrErr = ELF.sections();
  if (!SectionsOrErr)
    return SectionsOrErr.takeError();

  for (const auto &Sec : *SectionsOrErr) {
    if (Sec.sh_type != ELF::SHT_SYMTAB && Sec.sh_type != ELF::SHT_DYNSYM)
      continue;

    auto SymbolsOrErr = ELF.symbols(&Sec);
    if (!SymbolsOrErr)
      continue;

    auto StrTabOrErr = ELF.getStringTableForSymtab(Sec);
    if (!StrTabOrErr)
      continue;

    for (const auto &Sym : *SymbolsOrErr) {
      if (Sym.getType() == ELF::STT_FUNC && Sym.st_value != 0) {
        auto NameOrErr = Sym.getName(*StrTabOrErr);
        if (NameOrErr && !NameOrErr->empty())
          Functions.emplace_back(std::string(*NameOrErr), Sym.st_value);
      }
    }
  }

  return Functions;
}

// Check if a symbol is address-taken (appears in .hlfi_cfi_entries section)
template <class ELFT>
static Expected<std::set<std::string>>
getAddressTakenFunctions(const ELFFile<ELFT> &ELF) {
  std::set<std::string> AddressTaken;

  // Look for .hlfi_cfi_entries section which contains metadata about
  // which functions should be in the CFI table
  auto SecOrErr = findSection(ELF, ".hlfi_cfi_entries");
  if (!SecOrErr) {
    // Section not found - fall back to using the CFI table itself
    consumeError(SecOrErr.takeError());

    auto EntriesOrErr = readCFITable(ELF);
    if (!EntriesOrErr)
      return EntriesOrErr.takeError();

    for (const auto &Entry : *EntriesOrErr) {
      if (!Entry.SymbolName.empty())
        AddressTaken.insert(Entry.SymbolName);
    }
    return AddressTaken;
  }

  // Read the section contents
  auto DataOrErr = ELF.getSectionContents(**SecOrErr);
  if (!DataOrErr)
    return DataOrErr.takeError();

  // Parse entries - format: [index:u32, symbol_name:string\0]
  // Actually, this depends on how we emit the section from the compiler
  // For now, use the CFI table approach
  auto EntriesOrErr = readCFITable(ELF);
  if (!EntriesOrErr)
    return EntriesOrErr.takeError();

  for (const auto &Entry : *EntriesOrErr) {
    if (!Entry.SymbolName.empty())
      AddressTaken.insert(Entry.SymbolName);
  }

  return AddressTaken;
}

// Dump command implementation
static int dumpCFITable(StringRef Filename) {
  auto BufOrErr = MemoryBuffer::getFile(Filename);
  if (!BufOrErr) {
    WithColor::error() << "Could not open file: " << Filename << "\n";
    return 1;
  }

  auto ObjOrErr = ObjectFile::createELFObjectFile(**BufOrErr);
  if (!ObjOrErr) {
    WithColor::error() << "Not a valid ELF file: " << Filename << "\n";
    consumeError(ObjOrErr.takeError());
    return 1;
  }

  outs() << "HLFI CFI Table Analysis for: " << Filename << "\n";
  outs() << "========================================\n\n";

  if (auto *ELF64LE = dyn_cast<ELF64LEObjectFile>(ObjOrErr->get())) {
    auto EntriesOrErr = readCFITable(ELF64LE->getELFFile());
    if (!EntriesOrErr) {
      WithColor::error() << toString(EntriesOrErr.takeError()) << "\n";
      return 1;
    }

    auto &Entries = *EntriesOrErr;
    if (Entries.empty()) {
      outs() << "No CFI table found.\n";
      outs() << "(Neither .hlfi_cfi_table section nor __hlfi_cfi_table symbol present)\n";
      return 0;
    }

    outs() << "CFI Table Entries: " << Entries.size() << "\n\n";
    outs() << format("%-8s %-18s %s\n", "Index", "Address", "Symbol");
    outs() << "----------------------------------------------\n";

    for (const auto &Entry : Entries) {
      outs() << format("%-8" PRIu64 " 0x%016" PRIx64 " %s\n", Entry.Index,
                       Entry.Address,
                       Entry.SymbolName.empty() ? "<unknown>"
                                                : Entry.SymbolName.c_str());
    }
  } else {
    WithColor::error() << "Unsupported ELF format (only ELF64LE supported)\n";
    return 1;
  }

  return 0;
}

// Verify command implementation
static int verifyCFITable(StringRef Filename) {
  auto BufOrErr = MemoryBuffer::getFile(Filename);
  if (!BufOrErr) {
    WithColor::error() << "Could not open file: " << Filename << "\n";
    return 1;
  }

  auto ObjOrErr = ObjectFile::createELFObjectFile(**BufOrErr);
  if (!ObjOrErr) {
    WithColor::error() << "Not a valid ELF file: " << Filename << "\n";
    consumeError(ObjOrErr.takeError());
    return 1;
  }

  outs() << "HLFI CFI Table Verification for: " << Filename << "\n";
  outs() << "==============================================\n\n";

  if (auto *ELF64LE = dyn_cast<ELF64LEObjectFile>(ObjOrErr->get())) {
    const auto &ELF = ELF64LE->getELFFile();

    // Get CFI table entries
    auto EntriesOrErr = readCFITable(ELF);
    if (!EntriesOrErr) {
      WithColor::error() << toString(EntriesOrErr.takeError()) << "\n";
      return 1;
    }

    auto &Entries = *EntriesOrErr;
    if (Entries.empty()) {
      WithColor::warning()
          << "No CFI table found (__hlfi_cfi_table symbol not present)\n";
      return 0;
    }

    // Build set of addresses in the CFI table
    std::set<uint64_t> CFIAddresses;
    for (const auto &Entry : Entries)
      CFIAddresses.insert(Entry.Address);

    outs() << "CFI table contains " << Entries.size() << " entries\n\n";

    // Verify: check for duplicate indices (shouldn't happen with proper table)
    std::map<uint64_t, uint64_t> IndexToAddr;
    bool HasDuplicates = false;
    for (const auto &Entry : Entries) {
      if (IndexToAddr.count(Entry.Index)) {
        WithColor::warning()
            << "Duplicate index " << Entry.Index << ": 0x"
            << format_hex_no_prefix(IndexToAddr[Entry.Index], 16) << " and 0x"
            << format_hex_no_prefix(Entry.Address, 16) << "\n";
        HasDuplicates = true;
      }
      IndexToAddr[Entry.Index] = Entry.Address;
    }

    // Verify: check that all addresses point to valid functions
    auto FuncsOrErr = collectFunctionSymbols(ELF);
    if (!FuncsOrErr) {
      WithColor::error() << toString(FuncsOrErr.takeError()) << "\n";
      return 1;
    }

    std::set<uint64_t> FunctionAddresses;
    for (const auto &[Name, Addr] : *FuncsOrErr)
      FunctionAddresses.insert(Addr);

    int InvalidEntries = 0;
    for (const auto &Entry : Entries) {
      if (!FunctionAddresses.count(Entry.Address)) {
        WithColor::warning()
            << "CFI entry " << Entry.Index << " (0x"
            << format_hex_no_prefix(Entry.Address, 16)
            << ") does not point to a known function\n";
        InvalidEntries++;
      }
    }

    outs() << "\nVerification Summary:\n";
    outs() << "  Total entries: " << Entries.size() << "\n";
    outs() << "  Duplicate indices: " << (HasDuplicates ? "YES" : "No") << "\n";
    outs() << "  Invalid entries: " << InvalidEntries << "\n";

    if (HasDuplicates || InvalidEntries > 0) {
      outs() << "\nResult: FAILED\n";
      return 1;
    }

    outs() << "\nResult: PASSED\n";
    return 0;

  } else {
    WithColor::error() << "Unsupported ELF format (only ELF64LE supported)\n";
    return 1;
  }
}

// Build command implementation - creates a CFI table binary from object files
static int buildCFITable(ArrayRef<std::string> InputFiles,
                         StringRef OutputFile) {
  outs() << "Building HLFI CFI table from " << InputFiles.size()
         << " input files\n\n";

  // Collect all function entries across all input files
  std::map<std::string, uint64_t> AllFunctions;
  std::vector<CFIEntry> AllEntries;
  uint64_t NextIndex = 0;

  for (const auto &Filename : InputFiles) {
    auto BufOrErr = MemoryBuffer::getFile(Filename);
    if (!BufOrErr) {
      WithColor::error() << "Could not open file: " << Filename << "\n";
      return 1;
    }

    auto ObjOrErr = ObjectFile::createELFObjectFile(**BufOrErr);
    if (!ObjOrErr) {
      WithColor::error() << "Not a valid ELF file: " << Filename << "\n";
      consumeError(ObjOrErr.takeError());
      return 1;
    }

    if (auto *ELF64LE = dyn_cast<ELF64LEObjectFile>(ObjOrErr->get())) {
      const auto &ELF = ELF64LE->getELFFile();

      // Read existing CFI entries from this file
      auto EntriesOrErr = readCFITable(ELF);
      if (!EntriesOrErr) {
        WithColor::warning()
            << Filename << ": " << toString(EntriesOrErr.takeError()) << "\n";
        continue;
      }

      for (const auto &Entry : *EntriesOrErr) {
        if (Entry.SymbolName.empty())
          continue;

        // Check if we've already seen this function
        if (AllFunctions.count(Entry.SymbolName)) {
          // Update address if needed (could differ between .o files)
          continue;
        }

        // Assign new index
        CFIEntry NewEntry;
        NewEntry.Index = NextIndex++;
        NewEntry.Address = Entry.Address;
        NewEntry.SymbolName = Entry.SymbolName;
        AllEntries.push_back(NewEntry);
        AllFunctions[Entry.SymbolName] = NewEntry.Index;
      }

      outs() << "  " << Filename << ": " << EntriesOrErr->size()
             << " entries\n";
    } else {
      WithColor::warning()
          << Filename << ": Unsupported ELF format, skipping\n";
    }
  }

  outs() << "\nTotal unique entries: " << AllEntries.size() << "\n";

  // Write output table
  // Format: Binary file with 8-byte entries (function addresses)
  // The runtime will load this and set [x25+16] to point to it
  //
  // We also emit a text file with symbol mappings for debugging

  // Write binary table
  {
    std::error_code EC;
    raw_fd_ostream OS(OutputFile, EC, sys::fs::OF_None);
    if (EC) {
      WithColor::error() << "Could not open output file: " << OutputFile
                         << ": " << EC.message() << "\n";
      return 1;
    }

    // Sort by index
    std::sort(AllEntries.begin(), AllEntries.end(),
              [](const CFIEntry &A, const CFIEntry &B) {
                return A.Index < B.Index;
              });

    // Write entries
    for (const auto &Entry : AllEntries) {
      // Write 8-byte address in little-endian
      uint64_t Addr = Entry.Address;
      OS.write(reinterpret_cast<const char *>(&Addr), sizeof(Addr));
    }

    outs() << "Wrote binary table to: " << OutputFile << "\n";
  }

  // Write symbol map for debugging
  {
    SmallString<256> MapFile(OutputFile);
    sys::path::replace_extension(MapFile, ".map");

    std::error_code EC;
    raw_fd_ostream OS(MapFile, EC, sys::fs::OF_Text);
    if (EC) {
      WithColor::warning() << "Could not write map file: " << MapFile << "\n";
    } else {
      OS << "# HLFI CFI Table Symbol Map\n";
      OS << "# Index Address Symbol\n";
      for (const auto &Entry : AllEntries) {
        OS << format("%lu 0x%016lx %s\n", Entry.Index, Entry.Address,
                     Entry.SymbolName.c_str());
      }
      outs() << "Wrote symbol map to: " << MapFile << "\n";
    }
  }

  return 0;
}

// Patch command implementation - patches CFI indices in a linked binary
// Uses position-based matching: index slot N gets value N
static int patchCFIIndices(StringRef InputFile, StringRef OutputFile) {
  // Read the input file
  auto BufOrErr = MemoryBuffer::getFile(InputFile);
  if (!BufOrErr) {
    WithColor::error() << "Could not open file: " << InputFile << "\n";
    return 1;
  }

  // Make a mutable copy of the file contents
  std::string FileContents = (*BufOrErr)->getBuffer().str();

  auto ObjOrErr = ObjectFile::createELFObjectFile(
      MemoryBufferRef(FileContents, InputFile));
  if (!ObjOrErr) {
    WithColor::error() << "Not a valid ELF file: " << InputFile << "\n";
    consumeError(ObjOrErr.takeError());
    return 1;
  }

  outs() << "Patching HLFI CFI indices in: " << InputFile << "\n\n";

  if (auto *ELF64LE = dyn_cast<ELF64LEObjectFile>(ObjOrErr->get())) {
    const auto &ELF = ELF64LE->getELFFile();

    auto SectionsOrErr = ELF.sections();
    if (!SectionsOrErr) {
      WithColor::error() << toString(SectionsOrErr.takeError()) << "\n";
      return 1;
    }

    // Find the .hlfi_cfi_table and .hlfi_cfi_indices sections
    uint64_t CFITableOffset = 0, CFITableSize = 0;
    uint64_t IndicesOffset = 0, IndicesSize = 0;
    bool FoundTable = false, FoundIndices = false;

    for (const auto &Sec : *SectionsOrErr) {
      auto NameOrErr = ELF.getSectionName(Sec);
      if (!NameOrErr)
        continue;
      if (*NameOrErr == ".hlfi_cfi_table") {
        CFITableOffset = Sec.sh_offset;
        CFITableSize = Sec.sh_size;
        FoundTable = true;
      } else if (*NameOrErr == ".hlfi_cfi_indices") {
        IndicesOffset = Sec.sh_offset;
        IndicesSize = Sec.sh_size;
        FoundIndices = true;
      }
    }

    if (!FoundTable) {
      WithColor::error() << "No .hlfi_cfi_table section found\n";
      return 1;
    }

    if (!FoundIndices) {
      WithColor::error() << "No .hlfi_cfi_indices section found\n";
      return 1;
    }

    // Calculate number of entries
    // Table entries are 8 bytes each (function pointers)
    // Index slots are 4 bytes each (uint32_t)
    uint64_t NumTableEntries = CFITableSize / 8;
    uint64_t NumIndexSlots = IndicesSize / 4;

    outs() << "CFI table entries: " << NumTableEntries << "\n";
    outs() << "Index slots: " << NumIndexSlots << "\n";

    if (NumTableEntries != NumIndexSlots) {
      WithColor::warning() << "Mismatch: " << NumTableEntries
                           << " table entries but " << NumIndexSlots
                           << " index slots\n";
    }

    // Patch each index slot with its position (0, 1, 2, ...)
    // Position-based matching: slot N corresponds to table entry N
    uint64_t FileOffset = IndicesOffset;
    uint32_t PatchCount = 0;

    for (uint32_t i = 0; i < NumIndexSlots; i++) {
      if (FileOffset + 4 > FileContents.size()) {
        WithColor::error() << "Invalid file offset at index " << i << "\n";
        return 1;
      }

      // Patch the 32-bit value with the index
      uint32_t *Ptr = reinterpret_cast<uint32_t *>(&FileContents[FileOffset]);
      *Ptr = i;
      PatchCount++;

      FileOffset += 4;
    }

    outs() << "Patched " << PatchCount << " index slots\n";

    // Write output file
    StringRef OutFile = OutputFile.empty() ? InputFile : OutputFile;
    std::error_code EC;
    raw_fd_ostream OS(OutFile, EC, sys::fs::OF_None);
    if (EC) {
      WithColor::error() << "Could not write output file: " << OutFile << ": "
                         << EC.message() << "\n";
      return 1;
    }
    OS.write(FileContents.data(), FileContents.size());

    outs() << "Wrote patched binary to: " << OutFile << "\n";
    return 0;

  } else {
    WithColor::error() << "Unsupported ELF format (only ELF64LE supported)\n";
    return 1;
  }
}

int main(int argc, char **argv) {
  InitLLVM X(argc, argv);

  cl::HideUnrelatedOptions(HLFICategory);
  cl::ParseCommandLineOptions(
      argc, argv,
      "HLFI Post-Linker Tool\n\n"
      "This tool performs post-link operations for HLFI binaries.\n\n"
      "Subcommands:\n"
      "  dump   - Dump CFI table information from an ELF binary\n"
      "  verify - Verify CFI table correctness\n"
      "  build  - Build/merge CFI tables from object files\n"
      "  patch  - Patch CFI indices in a linked binary\n");

  if (DumpCmd) {
    return dumpCFITable(DumpInput);
  } else if (VerifyCmd) {
    return verifyCFITable(VerifyInput);
  } else if (BuildCmd) {
    return buildCFITable(BuildInputs, BuildOutput);
  } else if (PatchCmd) {
    return patchCFIIndices(PatchInput, PatchOutput);
  }

  cl::PrintHelpMessage();
  return 0;
}
