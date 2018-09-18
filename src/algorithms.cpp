// (Algorithms)
#define UU_ALGORITHMS
#ifndef UU_CONCEPTS
#include "concepts.hpp"
#endif
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
#ifndef UU_INTEGERS
#include "integers.cpp"
#endif
#ifndef UU_POINTERS
#include "pointers.cpp"
#endif
#ifndef REQUIRES
#define REQUIRES(...)
#endif
#ifndef DOC
#define DOC(...)
#endif
namespace uu
{
template <IntegralConcept I> bool zero(I x) { return x == I(0); }
template <IntegralConcept I> I successor(I x) { return x + 1; }
template <IntegralConcept I> I predecessor(I x) { return x - 1; }
template <typename T> void swap(PointerOf<T> a, PointerOf<T> b)
{
  T temp = source(a);
  sink(a) = source(b);
  sink(b) = temp;
}
template <OrdinateConcept I, typename Check = Writable<I>>
I set_step(I *pos_ptr, Writable<I> const &value)
{
  auto &pos = *pos_ptr;
  auto const old_pos = pos;
  sink(pos) = value;
  pos = successor(pos);
  return old_pos;
}
template <OrdinateConcept I, UnaryPredicateConcept P>
REQUIRES(Domain(P) == ValueType(InputOrdinateConcept))
I find_if(I first, I last, P pred)
  DOC("in [`first`, `last`) find the first position where `pred` is true")
{
  while (first != last && !pred(source(first))) {
    first = successor(first);
  }
  return first;
}
template <OrdinateConcept O, IntegralConcept I,
          typename Check = IntegerConcept<I>>
O fill_n(O first, I n, ValueType<O> x)
{
  while (!zero(n)) {
    sink(first) = x;
    first = successor(first);
    n = predecessor(n);
  }
  return first;
}
template <OrdinateConcept InputOrdinateConcept, IntegralConcept I,
          UnaryFunctionConcept Op, typename Check = IntegerConcept<I>>
REQUIRES(Domain(Op) == ValueType(InputOrdinateConcept))
InputOrdinateConcept for_each_n(InputOrdinateConcept first, I n, Op operation)
  DOC("for `n` times, advance iterator `first` and apply `operation` on its "
      "source")
{
  while (!zero(n)) {
    operation(source(first));
    first = successor(first);
    n = predecessor(n);
  }
  return first;
}
template <OrdinateConcept InputOrdinateConcept, UnaryFunctionConcept Op>
REQUIRES(Domain(Op) == ValueType(InputOrdinateConcept))
InputOrdinateConcept
  for_each(InputOrdinateConcept first, InputOrdinateConcept last, Op operation)
    DOC("for `n` times, advance iterator `first` and apply `operation` on its "
        "source")
{
  while (first != last) {
    operation(source(first));
    first = successor(first);
  }
  return first;
}
template <OrdinateConcept InputOrdinateConcept, BinaryFunctionConcept P,
          UnaryFunctionConcept Op>
REQUIRES(Domain(Op) == ValueType(InputOrdinateConcept) &&
         HomogeneousFunction(P, ValueType(InputOrdinateConcept)) &&
         Domain(P) == ValueType(InputOrdinateConcept))
InputOrdinateConcept
  for_each_adjacent(InputOrdinateConcept first, InputOrdinateConcept last,
                    P equal, Op operation)
    DOC("in [`first`,`last`) advance iterator `first` and apply `operation` on "
        "its "
        "source as long as neighbours are equal according to `equal`")
{
  if (first != last) {
    operation(source(first));
    auto prev = first;
    first = successor(first);
    while (first != last && equal(source(prev), source(first))) {
      operation(source(first));
      prev = first;
      first = successor(first);
    }
  }
  return first;
}
template <OrdinateConcept I0, UnaryPredicateConcept OpPred>
REQUIRES(Domain(Op) == ValueType(I0))
I0 sequential_partition_nonstable(I0 first, I0 last, OpPred op_pred)
  DOC("non stable partition of the elements in [first, last), "
      "where `op_pred` returns true for elements that must be "
      "moved forward. returns the partition point.")
{
  while (first != last) {
    auto &element = source(first);
    bool good_one = op_pred(element);
    if (!good_one) {
      auto &back_element = source(predecessor(last));
      if (AddressOf(back_element) != AddressOf(element)) {
        swap(&back_element, &element);
      }
      last = predecessor(last);
    } else {
      first = successor(first);
    }
  }
  return last;
}
template <OrdinateConcept I0, IntegralConcept C0, OrdinateConcept I1,
          IntegralConcept C1>
REQUIRES(ValueType(I0) == ValueType(I1))
void copy_n_m(I0 from, C0 from_size, I1 to, C1 to_size)
{
  while (from_size > 0 && to_size > 0) {
    sink(to) = source(from);
    from = successor(from);
    to = successor(to);
    --from_size;
    --to_size;
  }
}
template <OrdinateConcept I0, IntegralConcept C0, OrdinateConcept I1>
REQUIRES(ValueType(I0) == ValueType(I1))
// TODO(nicolas): should actually return a pair
I1 copy_n_bounded(I0 from, C0 from_size, I1 to, I1 to_last)
{
  while (from_size > 0 && to != to_last) {
    sink(to) = source(from);
    from = successor(from);
    to = successor(to);
    --from_size;
  }
  return to;
}
} // namespace uu
