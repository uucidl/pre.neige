// Memory Allocators
#define UU_ALLOCATORS
#ifndef UU_ALLOCATORS_API
#define UU_ALLOCATORS_API static
#endif
#ifndef UU_CONCEPTS
#include "concepts.hpp"
#endif
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
#ifndef UU_INTEGERS
#include "integers.cpp"
#endif
#ifndef UU_ERRORS
#include "errors.cpp"
#endif
#ifndef UU_ALGORITHMS
#include "algorithms.cpp"
#endif
#ifndef DOC
#define DOC(...)
#endif
namespace uu
{
template <typename T, IntegralConcept I> struct counted_range {
  T *first;
  I count;
};
template <typename T, IntegralConcept I>
struct container_concept<counted_range<T, I>> {
  using read_write_ordinate = PointerOf<T>;
};
template <typename T, IntegralConcept I> I container_size(counted_range<T, I> x)
{
  return x.count;
}
template <typename T, IntegralConcept I>
ReadWriteOrdinate<counted_range<T, I>> begin(counted_range<T, I> x)
{
  return x.first;
}
template <typename T, IntegralConcept I>
ReadWriteOrdinate<counted_range<T, I>> end(counted_range<T, I> x)
{
  return begin(x) + x.count;
}
template <typename T, IntegralConcept I>
ReadWriteOrdinate<counted_range<T, I>> at(counted_range<T, I> x, memory_size i)
{
  return &x.first[i];
}
template <typename T, IntegralConcept I>
ReadWriteOrdinate<counted_range<T, I>>
step_within(counted_range<T, I> x,
            ReadWriteOrdinate<counted_range<T, I>> *position)
{
  auto &p = *position;
  auto const previous = p;
  if (I(p - x.first) < x.count) {
    p = successor(p);
  }
  return previous;
}
template <typename T, IntegralConcept I>
ReadWriteOrdinate<counted_range<T, I>>
set_step_within(counted_range<T, I> x,
                PointerOf<ReadWriteOrdinate<counted_range<T, I>>> position,
                T value)
{
  auto &p = *position;
  auto const previous = p;
  uu_fatal_if(p < x.first);
  if (p - x.first < x.count) {
    sink(p) = value;
    p = successor(p);
  }
  return previous;
}
template <typename Allocator, typename T>
void alloc_array(PointerOf<Allocator> allocator, memory_size count,
                 PointerOf<PointerOf<T>> dest_pointer_output)
  DOC("allocate enough room for a contiguous array and copy it to "
      "`dest_pointer_output`")
{
  sink(dest_pointer_output) =
    reinterpret_cast<PointerOf<T>>(alloc(allocator, SizeOf(T) * count));
}
template <typename Allocator, typename T, IntegralConcept I>
void alloc_array(PointerOf<Allocator> allocator, I count,
                 PointerOf<counted_range<T, I>> dest)
  DOC("allocate enough room for a contiguous array and copy it to "
      "`dest`")
{
  alloc_array(allocator, count, &dest->first);
  dest->count = count;
}
template <typename T, typename Allocator>
PointerOf<T> object_alloc(PointerOf<Allocator> allocator)
{
  auto address = alloc(allocator, SizeOf(T));
  return reinterpret_cast<PointerOf<T>>(address);
}
template <typename T>
counted_range<T, memory_size> slice_memory(memory_address start,
                                           memory_size size)
{
  // TODO(nicolas): alignment assertion
  counted_range<T, memory_size> result;
  result.first = reinterpret_cast<PointerOf<T>>(start);
  result.count = size / SizeOf(T);
  return result;
}
struct slab_allocator {
  memory_address start;
  memory_address unallocated_start;
  memory_size size;
};
UU_ALLOCATORS_API slab_allocator make_slab_allocator(memory_address start,
                                                     memory_size size)
{
  slab_allocator result;
  result.start = start;
  result.unallocated_start = start;
  result.size = size;
  return result;
}
UU_ALLOCATORS_API memory_address alloc(slab_allocator *slab_allocator,
                                       memory_size size)
{
  // TODO(nicolas): alignment and zeroing
  memory_size current_size =
    memory_size(slab_allocator->unallocated_start - slab_allocator->start);
  uu_fatal_ifnot(
    addition_less_or_equal(current_size, size, slab_allocator->size));
  memory_address block_start = slab_allocator->unallocated_start;
  slab_allocator->unallocated_start += size;
  return block_start;
}
UU_ALLOCATORS_API memory_size allocatable_size(slab_allocator *slab_allocator)
{
  return memory_size(slab_allocator->start + slab_allocator->size -
                     slab_allocator->unallocated_start);
}
UU_ALLOCATORS_API void free(slab_allocator *slab_allocator,
                            memory_address start, memory_size size)
{
  uu_fatal_ifnot(start == slab_allocator->unallocated_start - size);
  slab_allocator->unallocated_start = start;
}
} // namespace uu
