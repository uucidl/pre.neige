// (CircularOrdinate)
#define UU_CIRCULAR_ORDINATE
#ifndef UU_CONCEPTS
#include "concepts.hpp"
#endif
#ifndef REQUIRES
#define REQUIRES(...)
#endif
namespace uu
{
template <OrdinateConcept Ordinate> struct circular_ordinate {
  Ordinate current;
  Ordinate first;
  Ordinate last;
};
template <OrdinateConcept Ordinate>
circular_ordinate<Ordinate> predecessor(circular_ordinate<Ordinate> x)
{
  auto result = x;
  if (result.current == x.first) {
    result.current = x.last;
  }
  result.current = predecessor(result.current);
  return result;
}
template <OrdinateConcept Ordinate>
circular_ordinate<Ordinate> successor(circular_ordinate<Ordinate> x)
{
  auto result = x;
  result.current = successor(x.current);
  if (result.current == x.last) {
    result.current = x.first;
  }
  return result;
}
template <OrdinateConcept Ordinate>
struct writable_concept<circular_ordinate<Ordinate>> {
  using writable_type = Writable<Ordinate>;
};
template <OrdinateConcept Ordinate>
struct readable_concept<circular_ordinate<Ordinate>> {
  using readable_type = Readable<Ordinate>;
};
template <OrdinateConcept Ordinate>
REQUIRES(WritableConcept<Ordinate>)
Writable<Ordinate> &sink(struct circular_ordinate<Ordinate> x)
{
  return sink(x.current);
}
template <OrdinateConcept Ordinate>
REQUIRES(ReadableConcept<Ordinate>)
Readable<Ordinate> const &source(struct circular_ordinate<Ordinate> x)
{
  return source(x.current);
}
template <OrdinateConcept Ordinate>
circular_ordinate<Ordinate> make_circular_ordinate(Ordinate first,
                                                   Ordinate last)
{
  return {first, first, last};
}
} // namespace uu
