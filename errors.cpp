#define UU_ERRORS
#ifndef UU_OS
#include "os.cpp"
#endif
// die if bool is false
#define fatal_ifnot(x)                                                         \
  if (!(x)) {                                                                  \
    debugger_break();                                                       \
    os_fatal();                                                                \
  }
#define fatal_if(x) fatal_ifnot(!(x))
