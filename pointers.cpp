// (Pointers)
#define UU_POINTERS
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
namespace uu
{
template <typename T> struct readable_concept<PointerOf<T>> {
  using readable_type = T &; // at least readable, also writable
};
template <typename T> struct writable_concept<PointerOf<T>> {
  using writable_type = T &;
};
template <typename T> Readable<PointerOf<T>> source(PointerOf<T> x)
{
  return *x;
}
template <typename T> Writable<PointerOf<T>> sink(PointerOf<T> x) { return *x; }
template <typename T> struct ordinate_concept<PointerOf<T>> {
  using value_type = T;
};
template <typename T> PointerOf<T> successor(PointerOf<T> x) { return x + 1; }
template <typename T> PointerOf<T> predecessor(PointerOf<T> x) { return x - 1; }
template <typename T> memory_address AddressOf(T &xref)
{
  return memory_address(&xref);
}
} // namespace uu
