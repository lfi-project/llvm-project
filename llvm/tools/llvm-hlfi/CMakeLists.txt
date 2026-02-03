set(LLVM_LINK_COMPONENTS
  BinaryFormat
  MC
  MCDisassembler
  Object
  Support
  TargetParser
  )

add_llvm_tool(llvm-hlfi
  llvm-hlfi.cpp
  )
