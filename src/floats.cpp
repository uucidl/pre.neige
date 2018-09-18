// (Floats)
#define UU_FLOATS
#ifndef UU_FLOATS_API
#define UU_FLOATS_API static
#endif
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
#ifndef UU_CPU
#include "cpu.cpp"
#endif
namespace uu
{
UU_FLOATS_API inline bool finite(float32 x) { return cpu_finite(x); }
UU_FLOATS_API inline bool finite(float64 x) { return cpu_finite(x); }
UU_FLOATS_API inline float32 absolute_value(float32 x) { return cpu_abs(x); }
UU_FLOATS_API inline float64 absolute_value(float64 x) { return cpu_abs(x); }
UU_FLOATS_API inline bool valid(float32 x) { return finite(x); }
UU_FLOATS_API inline float32 product(float32 a, float32 b) { return a * b; }
UU_FLOATS_API inline float64 product(float64 a, float64 b) { return a * b; }
UU_FLOATS_API inline float32 product(float32 a, float64 b) { return a * b; }
} // namespace uu
