// Concepts
#define UU_CONCEPTS
#define IntegralConcept typename
#define UnaryFunctionConcept typename
#define BinaryFunctionConcept typename
#define UnaryPredicateConcept typename
namespace uu
{
// (Distance)
template <typename T> struct distance_type_impl {
};
template <typename T> using DistanceType = typename distance_type_impl<T>::type;
// (OrdinateConcept)
#define OrdinateConcept typename
template <typename T> struct ordinate_concept {
  using value_type = void;
};
template <typename T>
using ValueType = typename ordinate_concept<T>::value_type;
// (ReadableConcept)
#define ReadableConcept typename
template <typename T> struct readable_concept {
  using readable_type = void;
};
template <typename T>
using Readable = typename readable_concept<T>::readable_type;
template <ReadableConcept RC, typename Check = Readable<RC>>
Readable<RC> source(RC x);
// (WritableConcept)
#define WritableConcept typename
template <typename T> struct writable_concept {
  using writable_type = void;
};
template <typename T>
using Writable = typename writable_concept<T>::writable_type;
template <WritableConcept WC, typename Check = Writable<WC>>
Writable<WC> sink(WC x);
// (ContainerConcept)
#define ContainerConcept typename
template <typename T> struct container_concept {
  using read_write_ordinate = void;
};
template <ContainerConcept C>
using ReadWriteOrdinate = typename container_concept<C>::read_write_ordinate;
} // namespace uu
