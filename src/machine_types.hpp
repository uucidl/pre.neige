#ifndef UU_MACHINE_TYPES
// (Machine Type Aliases)
#define UU_MACHINE_TYPES
#ifndef UU_PLATFORM_CONFIG
#include "platform_config.hpp"
#endif
#ifndef UU_CONCEPTS
#include "concepts.hpp"
#endif
namespace uu
{
// (PointerOf)
template <typename T> using PointerOf = T *;
using u8 = unsigned char;
using u16 = unsigned short;
using u32 = unsigned int;
using s8 = signed char;
using s16 = signed short;
using s32 = signed int;
using bool32 = u32;
#if OS == OS_OSX && CPU == CPU_X86_64
using u64 = unsigned long;
using s64 = signed long;
#elif OS == OS_WINDOWS && CPU == CPU_X86_64
using u64 = unsigned long long;
using s64 = signed long long;
#else
#error "define 64bit types"
#endif
using float32 = float;
using float64 = double;

#define SizeOf(type_x) (sizeof(type_x))
static_assert(SizeOf(u8) == 1, "u8");
static_assert(SizeOf(s8) == 1, "s8");
static_assert(SizeOf(u16) == 2, "u16");
static_assert(SizeOf(s16) == 2, "s16");
static_assert(SizeOf(u32) == 4, "u32");
static_assert(SizeOf(s32) == 4, "s32");
static_assert(SizeOf(u64) == 8, "u64");
static_assert(SizeOf(s64) == 8, "s64");
static_assert(SizeOf(float32) == 4, "float32");
static_assert(SizeOf(float64) == 8, "float64");

// (Memory)
template <typename T> struct distance_type_impl<PointerOf<T>> {
  using type = s64;
};

using memory_address = PointerOf<u8>;
using memory_size = DistanceType<memory_address>;
static_assert(SizeOf(memory_address) == SizeOf(memory_size),
              "memory_size incorrect for architecture");
} // namespace uu
#endif
