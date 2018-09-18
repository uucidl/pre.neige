#define UU_INTEGERS
#ifndef UU_INTEGERS_API
#define UU_INTEGERS_API static
#endif
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
#define Integer typename
namespace uu
{
// (Modular Integers)
struct modular_integer_tag {
};
template <typename T> struct integer_concept {
};
template <typename T> constexpr T valid_max();
#define DefineModularInteger(T, __upper_bound)                                 \
  template <> struct integer_concept<T> {                                      \
    using concept = modular_integer_tag;                                       \
  };                                                                           \
  template <> constexpr T valid_max<T>() { return __upper_bound; }
DefineModularInteger(u8, u8(~0));
DefineModularInteger(u16, u16(~0));
DefineModularInteger(u32, u32(~0));
DefineModularInteger(u64, u64(~0));
DefineModularInteger(s8, s8(127));
DefineModularInteger(s16, s16(32767));
DefineModularInteger(s32, s32(2147483647));
DefineModularInteger(s64, s64(9223372036854775807LL));
template <typename T>
using IntegerConcept = typename integer_concept<T>::concept;
template <Integer I>
bool addition_less(I x, I addand, I limit, modular_integer_tag)
{
  if (x >= limit) {
    return false;
  }
  if (addand > 0) {
    return limit - x > addand;
  } else {
    return true;
  }
}
template <Integer I> bool addition_valid(I x, I addand, modular_integer_tag tag)
{
  if (x >= valid_max<I>()) {
    return false;
  }
  return addition_less(x, addand, valid_max<I>(), tag);
}
// TODO(nicolas): name, addition_saturated
template <Integer I> bool addition_less(I x, I addand, I limit)
{
  return addition_less(x, addand, limit, IntegerConcept<I>());
}
template <Integer I> bool addition_less_or_equal(I x, I addand, I limit)
{
  return addition_less(x, addand, limit) || ((x + addand) == limit);
}
template <Integer I> bool addition_valid(I x, I addand)
{
  return addition_valid(x, addand, IntegerConcept<I>());
}
template <Integer I> I product(I a, I b, modular_integer_tag) { return a * b; }
template <Integer I> I product(I a, I b)
{
  return product(a, b, IntegerConcept<I>());
}
} // namespace uu
