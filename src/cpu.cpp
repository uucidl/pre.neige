// (Cpu)
#ifndef UU_PLATFORM_CONFIG
#include "platform_config.hpp"
#endif
#define UU_CPU
#ifndef UU_CPU_API
#define UU_CPU_API static
#endif
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
namespace uu
{
UU_CPU_API inline float64 cpu_abs(float64 x);
UU_CPU_API inline float64 cpu_sin(float64 x);
UU_CPU_API inline float64 cpu_cos(float64 x);
UU_CPU_API inline float64 cpu_sqrt(float64 x);
UU_CPU_API inline bool cpu_finite(float64 x);
} // namespace uu
#if COMPILER == COMPILER_CLANG && (CPU == CPU_IA32 || CPU == CPU_X86_64)
namespace uu
{
UU_CPU_API u8 bit_scan_reverse32(u32 x)
{
  u32 y;
  asm("bsrl %1,%0" : "=r"(y) : "r"(x));
  return y;
}
UU_CPU_API float64 cpu_abs(float64 x)
{
  float64 y;
  asm("fabs" : "=t"(y) : "0"(x));
  return y;
}
UU_CPU_API float64 cpu_sin(float64 x)
{
  float64 y;
  asm("fsin" : "=t"(y) : "0"(x));
  return y;
}
UU_CPU_API float64 cpu_cos(float64 x)
{
  float64 y;
  asm("fcos" : "=t"(y) : "0"(x));
  return y;
}
UU_CPU_API bool cpu_finite(float64 x)
{
  register u16 y asm("eax") = 0x88;
  asm("fxam\n"
      "fstsw %0"
      : "=r"(y)
      : "f"(x));
  union x87_status_register {
    u16 value;
    struct {
      u16 ignored : 8;
      u16 C0 : 1;
      u16 C1 : 1;
      u16 C2 : 1;
      u16 TOP : 3;
      u16 C3 : 1;
    };
  };
  x87_status_register yy;
  yy.value = y;
  u16 triple = (yy.C3 << 2) | (yy.C2 << 1) | (yy.C0);
  return triple == 2 || triple == 4 || triple == 6;
}
UU_CPU_API float64 cpu_sqrt(float64 x)
{
  float64 y;
  asm("fsqrt" : "=t"(y) : "0"(x));
  return y;
}
} // namespace uu
#elif COMPILER == COMPILER_MSC && (CPU == CPU_IA32 || CPU == CPU_X86_64)
#include <float.h>
#include <intrin.h>
#include <math.h>
namespace uu
{
/* see documentation:
 * URL(https://msdn.microsoft.com/en-us/library/hh977023.aspx) */
UU_CPU_API u8 bit_scan_reverse32(u32 x)
{
  unsigned long index;
  unsigned long mask = x;
  _BitScanReverse(&index, mask);
  return index;
}
UU_CPU_API inline float64 cpu_abs(float64 x) { return fabs(x); }
UU_CPU_API inline float64 cpu_sin(float64 x) { return sin(x); }
UU_CPU_API inline float64 cpu_cos(float64 x) { return cos(x); }
UU_CPU_API bool cpu_finite(float64 x) { return _finite(x); }
UU_CPU_API float64 cpu_sqrt(float64 x) { return sqrtf(x); }
} // namespace uu
#endif
