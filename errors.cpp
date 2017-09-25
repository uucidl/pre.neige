#define UU_ERRORS
// die if bool is false
#define uu_fatal_ifnot(x)                                                      \
  if (!(x)) {                                                                  \
    uu_debugger_break();                                                       \
    uu::os_fatal();                                                            \
  }
#define uu_fatal_if(x) uu_fatal_ifnot(!(x))
#ifndef UU_OS
#include "os.cpp"
#endif
