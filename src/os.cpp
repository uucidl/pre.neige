// (Os)
#define UU_OS
#ifndef UU_OS_API
#define UU_OS_API static
#endif
#ifndef UU_PLATFORM_CONFIG
#include "platform_config.hpp"
#endif
#ifndef DOC
#define DOC(...)
#endif
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
namespace uu
{
UU_OS_API void os_fatal() CLANG_ATTRIBUTE(noreturn)
  DOC("kill the current process and return to the OS");
UU_OS_API memory_address os_vm_alloc(memory_size size)
  DOC("allocate a memory block from the OS' virtual memory");
UU_OS_API void os_vm_free(memory_size size, memory_address data)
  DOC("release a memory block from the OS");
} // namespace uu
// (Os)
#if OS == OS_OSX
#include <mach/mach.h> // for vm_allocate
#include <unistd.h>    // for _exit
// TODO(nicolas): proof that this should be part of os
#ifndef uu_fatal_ifnot
#define uu_fatal_ifnot(x)                                                      \
  if (!(x)) {                                                                  \
    uu_debugger_break();                                                       \
    os_fatal();                                                                \
  }
#endif
namespace uu
{
UU_OS_API void os_fatal() { _exit(-3); }
UU_OS_API memory_address os_vm_alloc(memory_size size)
{
  uu_fatal_ifnot(size > 0);
  vm_address_t address = 0;
  auto vm_allocate_result = vm_allocate(mach_task_self(), &address, size, true);
  uu_fatal_ifnot(KERN_SUCCESS == vm_allocate_result);
  return memory_address(address);
}
UU_OS_API void os_vm_free(memory_size size, memory_address address)
{
  auto vm_deallocate_result =
    vm_deallocate(mach_task_self(), vm_address_t(address), size);
  uu_fatal_ifnot(KERN_SUCCESS == vm_deallocate_result);
}
} // namespace uu
#elif OS == OS_WINDOWS
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
namespace uu
{
UU_OS_API void os_fatal() { ExitProcess(3); }
UU_OS_API memory_address os_vm_alloc(memory_size size)
{
  LPVOID address = 0;
  DWORD flAllocationType = MEM_COMMIT | MEM_RESERVE;
  DWORD flProtect = PAGE_READWRITE;
  return memory_address(
    VirtualAlloc(address, size, flAllocationType, flProtect));
}
UU_OS_API void os_vm_free(memory_size size, memory_address data)
{
  VirtualFree(data, size, MEM_RELEASE);
}
} // namespace uu
#else
#error "Unimplemented OS module"
#endif
