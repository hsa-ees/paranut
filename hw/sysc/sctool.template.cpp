#define __SYNTHESIS__
#include <sc_tool/SCTool.h>  

const char* __sctool_args_str = R"( "{SC_TOOL_MAIN_CPP}" "-sv_out" "{SC_TOOL_SV_OUT}" "-top" "{SC_TOOL_TOP_MODULE}" "-init_local_vars" "--" "-D__SC_TOOL__" "-D__SC_TOOL_ANALYZE__" "-DNDEBUG" "-Wno-logical-op-parentheses" "-std=c++14" "-I{SC_TOOL_ICSC_INCLUDE}" "-I{SC_TOOL_ICSC_INCLUDE}/sctcommon" "-I{SC_TOOL_ICSC_INCLUDE}/sctmemory" "-I{SC_TOOL_AUTO_INCLUDE}" "-I{SC_TOOL_ICSC_INCLUDE}/sctmemory/utils" "-I{SC_TOOL_ICSC_HOME}/lib/clang/12.0.1/include" "-I{SC_TOOL_WORKDIR}/sc_elab2/lib" {SC_TOOL_HEADER_INCLUDE})"; 

{SC_TOOL_SRC_INCLUDE}
