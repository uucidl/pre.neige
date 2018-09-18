// (Perlin Noise2)
#define UU_PERLIN_NOISE2
#ifndef UU_PERLIN_NOISE2_API
#define UU_PERLIN_NOISE2_API static
#endif
#ifndef UU_PLATFORM_CONFIG
#include "platform_config.hpp"
#endif
#ifndef UU_FLOATS
#include "floats.cpp"
#endif
namespace uu
{
// returns a float in interval [-1..+1]
UU_PERLIN_NOISE2_API float32 perlin_noise2(float32 period, float32 x,
                                           float32 y);
} // namespace uu
// (stb_perlin)
#if COMPILER == COMPILER_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-conversion"
#endif
#define STB_PERLIN_IMPLEMENTATION
#include "uu.ticks/modules/stb/stb_perlin.h"
#if COMPILER == COMPILER_CLANG
#pragma clang diagnostic pop
#endif
namespace uu
{
UU_PERLIN_NOISE2_API float32 perlin_noise2(float32 period, float32 x, float32 y)
{
  auto result = stb_perlin_noise3(x / period, y / period, 0.0f, 256, 256, 256);
  return result;
}
} // namespace uu
