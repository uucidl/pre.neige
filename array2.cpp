// (Multi-Dimensional Arrays)
#define UU_ARRAY2
// PERSON(Pramod Gupta)
// URL(https://www.youtube.com/watch?v=CPPX4kwqh80)
// URL(https://github.com/astrobiology/orca_array)
#ifndef UU_CONCEPTS
#include "concepts.hpp"
#endif
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
#ifndef MODELS
#define MODELS(...)
#endif
template <typename T, typename I> struct array2_header MODELS(container) {
  T *memory;
  I memory_size;
  I d0_count;
  I d1_count;
};
template <typename T, IntegralConcept I>
struct container_concept<array2_header<T, I>> {
  using read_write_ordinate = PointerOf<T>;
};
template <typename T, typename I>
array2_header<T, I> make_array2(T *memory, I memory_size, I d0, I d1)
{
  fatal_ifnot(d0 * d1 <= memory_size);
  return {memory, memory_size, d0, d1};
}
template <typename T, typename I>
PointerOf<T> at(array2_header<T, I> const &array, I x0, I x1)
{
#if defined(NEIGE_SLOW)
  fatal_ifnot(x0 < array.d0_count);
  fatal_ifnot(x1 < array.d1_count);
#endif
  I offset = x1 * array.d0_count + x0;
  fatal_ifnot(offset < array.memory_size);
  return &array.memory[offset];
}
template <typename T, typename I>
PointerOf<T> begin(array2_header<T, I> const &x)
{
  return x.memory;
}
template <typename T, typename I>
memory_size container_size(array2_header<T, I> const &x)
{
  return x.d0_count * x.d1_count;
}
