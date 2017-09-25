#define UU_NUMBERS
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
// (Numbers>
#define NumberConcept typename
namespace uu
{
template <NumberConcept N> struct number_concept {
  /* static constexpr void min; */
  /* static constexpr void max; */
};
template <typename T> using Number = number_concept<T>;
template <> struct number_concept<float32> {
  static constexpr float32 min = -2e38f;
  static constexpr float32 max = +2e38f;
};
} // namespace uu
