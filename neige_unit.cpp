/*
  Build instructions:
*/
#define BUILD(__os, ...)
BUILD(OSX, "clang++ -DOS=OS_OSX -DSTATIC_GLEW -DNEIGE_SLOW -g -std=c++11 \
      -framework System -Wall \
  -Wextra \
  -Wno-tautological-compare -Wsign-conversion \
  neige_unit.cpp -o neige                                                    \
  -Iuu.micros/include/ -framework OpenGL -Iuu.micros/libs/glew/include/ \
  -Iuu.micros/libs -Luu.micros/libs/Darwin_x86_64/ -lglfw3 -framework Cocoa \
  -framework IOKit -framework CoreAudio hidapi/mac/hid.o")
/*
  Table Of Contents:
  - Jump to (Main) to find the program.
  - Jump to TAG("visuals") to find the main visual effect
  TAG(project)
   - TODO(nicolas): load sample
  TAG(style)
  - types named `_header` are non-owning. They refer to remote parts
    but do not manage these resources.
 */
// (Platform Configuration)
#define CPU_IA32 (1) DOC("32bit intel platform")
#define CPU_X86_64 (2) DOC("x64 introduced by AMD and adopted by intel")
#define OS_OSX (1)
#define OS_WINDOWS (2)
#if OS == OS_OSX
#if !defined(CPU)
#define CPU CPU_X86_64
#endif
#endif
#if !defined(COMPILER_CLANG) && defined(__clang__)
#define COMPILER_CLANG
#endif
#if !defined(COMPILER_MSC) && defined(_MSC_VER)
#define COMPILER_MSC
#endif
#if !defined(OS)
#error "OS must be defined"
#endif
#if !defined(CPU)
#error "CPU must be defined"
#endif
// (Meta)
#define DOC(...)          // to document a symbol
#define DOC_COUPLING(...) // to document coupling w/ other symbols
#define URL(...)          // reference to a resource
#define TAG(...)          // a tag, for documentation
// (Compiler)
#if defined(COMPILER_CLANG)
#define CLANG_ATTRIBUTE(x) __attribute__((x))
#define debugger_break() DOC("invoke debugger") asm("int3")
#else
#define CLANG_ATTRIBUTE(x)
#define debugger_break() DOC("invoke debugger") __debugbreak()
#endif
#define global_variable DOC("mark global variables") static
#define local_state DOC("mark locally persistent variables") static
#define internal_symbol DOC("mark internal symbols") static
#define UNUSED_PARAMETER(x) ((void)x)
// NOTE(uucidl): TAG(unsafe), use fixed size array type instead whenever
#define FixedArrayCount(array) (sizeof(array) / sizeof(*array))
#define assert(__predicate_expr)                                               \
  if (!(__predicate_expr)) debugger_break()
// (Word Types)
using u8 = unsigned char;
using u16 = unsigned short;
using u32 = unsigned int;
using s8 = signed char;
using s16 = signed short;
using s32 = signed int;
using bool32 = u32;
using float32 = float;
using float64 = double;
#if OS == OS_OSX && CPU == CPU_X86_64
using u64 = unsigned long;
using s64 = signed long;
#elif OS == OS_WINDOWS && CPU == CPU_X86_64
using u64 = unsigned long long;
using s64 = signed long long;
#else
#error "define 64bit types"
#endif
static_assert(sizeof(u8) == 1, "u8");
static_assert(sizeof(s8) == 1, "s8");
static_assert(sizeof(u16) == 2, "u16");
static_assert(sizeof(s16) == 2, "s16");
static_assert(sizeof(u32) == 4, "u32");
static_assert(sizeof(s32) == 4, "s32");
static_assert(sizeof(u64) == 8, "u64");
static_assert(sizeof(s64) == 8, "s64");
static_assert(sizeof(float32) == 4, "float32");
static_assert(sizeof(float64) == 8, "float64");
// (SizeOf)
#define SizeOf(x) sizeof(x)
// (PointerOf)
template <typename T> using PointerOf = T *;
// (DistanceType)
template <typename T> struct distance_type_impl {
};
template <typename T> struct distance_type_impl<PointerOf<T>> {
  using type = u64;
};
template <typename T> using DistanceType = typename distance_type_impl<T>::type;
// (Memory)
using memory_address = PointerOf<u8>;
using memory_size = DistanceType<memory_address>;
static_assert(SizeOf(memory_address) == SizeOf(memory_size),
              "memory_size incorrect for architecture");
// (Os)
internal_symbol void fatal() CLANG_ATTRIBUTE(noreturn)
  DOC("kill the current process and return to the OS");
internal_symbol memory_address vm_alloc(memory_size size)
  DOC("allocate a memory block from the OS' virtual memory");
internal_symbol void vm_free(memory_size size, memory_address data)
  DOC("release a memory block from the OS");
// die if bool is false
#define fatal_ifnot(x)                                                         \
  if (!(x)) {                                                                  \
    debugger_break();                                                          \
    fatal();                                                                   \
  }
#define fatal_if(x) fatal_ifnot(!(x))
// (Concepts)
#define MODELS(...)
#define REQUIRES(...)
#define IntegralConcept typename
#define UnaryFunctionConcept typename
#define BinaryFunctionConcept typename
#define UnaryPredicateConcept typename
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
// (Fixed Array Type)
template <typename T, memory_size N> constexpr u64 container_size(T(&)[N])
{
  return N;
}
template <typename T, memory_size N> struct fixed_array_header {
  T(&array)[N];
  fixed_array_header(T(&init_array)[N]) : array(init_array) {}
  T &operator[](memory_size i) { return array[i]; }
};
template <typename T, memory_size N>
fixed_array_header<T, N> make_fixed_array_header(T(&array)[N])
{
  fixed_array_header<T, N> header = {array};
  return header;
}
template <typename T, memory_size N>
PointerOf<T> begin(fixed_array_header<T, N> x)
{
  return &x[0];
}
template <typename T, memory_size N>
PointerOf<T> end(fixed_array_header<T, N> x)
{
  return begin(x) + container_size(x);
}
template <typename T, memory_size N>
constexpr u64 container_size(fixed_array_header<T, N> const &)
{
  return N;
}
// (ContainerConcept)
#define ContainerConcept typename
template <typename T> struct container_concept {
  using read_write_ordinate = void;
};
template <ContainerConcept C>
using ReadWriteOrdinate = typename container_concept<C>::read_write_ordinate;
// (CircularOrdinate)
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
// (Modular Integers)
#define Integer typename
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
// (Pointers)
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
// (Algorithms)
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
REQUIRES(ValueType(I0) == ValueType(I1)) void copy_n_m(I0 from, C0 from_size,
                                                       I1 to, C1 to_size)
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
// (Numbers>
#define NumberConcept typename
template <NumberConcept N> struct number_concept {
  /* static constexpr void min; */
  /* static constexpr void max; */
};
template <typename T> using Number = number_concept<T>;
template <> struct number_concept<float32> {
  static constexpr float32 min = -2e38f;
  static constexpr float32 max = +2e38f;
};
// (Arithmetics)
template <typename T> T absolute_value(T x) { return x < T(0) ? -x : x; }
template <typename T> bool finite(T x);
template <typename T> bool operator<(T a, T b) { return natural_order(a, b); }
template <typename T> bool operator==(T a, T b) { return equality(a, b); }
template <typename T> bool operator!=(T a, T b) { return !(a == b); }
template <typename T> T operator+(T a, T b) { return addition(a, b); }
template <typename T> T min(T a, T b) { return a < b ? a : b; }
template <typename T> T max(T a, T b) { return !(a < b) ? a : b; }
template <typename N> N squared(N n) { return n * n; }
template <typename T> T interpolate_linear(T a, T b, float64 a_b)
{
  return (1.0 - a_b) * a + a_b * b;
}
template <typename T> T lower_division(T x, T divisor)
{
  auto division = x / divisor;
  T result;
  if (division < 0) {
    result = divisor * (-int(-division));
  } else {
    result = divisor * int(x / divisor);
  }
  return result;
}
template <typename T> T upper_division(T x, T divisor)
{
  return 1 + -lower_division(-x, divisor);
}
// (Memory Allocation)
template <typename T, IntegralConcept I> struct counted_range {
  T *first;
  I count;
};
template <typename T, IntegralConcept I>
struct container_concept<counted_range<T, I>> {
  using read_write_ordinate = PointerOf<T>;
};
template <typename T, IntegralConcept I>
u64 container_size(counted_range<T, I> x)
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
  fatal_if(p < 0);
  if (I(p - x.first) < x.count) {
    p = successor(p);
  }
  return previous;
}
template <typename T, IntegralConcept I>
ReadWriteOrdinate<counted_range<T, I>>
set_step_within(counted_range<T, I> x,
                ReadWriteOrdinate<counted_range<T, I>> *position, T value)
{
  auto &p = *position;
  auto const previous = p;
  fatal_if(p < x.first);
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
                 counted_range<T, I> *dest)
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
internal_symbol slab_allocator make_slab_allocator(memory_address start,
                                                   memory_size size)
{
  slab_allocator result;
  result.start = start;
  result.unallocated_start = start;
  result.size = size;
  return result;
}
internal_symbol memory_address alloc(slab_allocator *slab_allocator,
                                     memory_size size)
{
  // TODO(nicolas): alignment and zeroing
  memory_size current_size =
    memory_size(slab_allocator->unallocated_start - slab_allocator->start);
  fatal_ifnot(addition_less_or_equal(current_size, size, slab_allocator->size));
  memory_address block_start = slab_allocator->unallocated_start;
  slab_allocator->unallocated_start += size;
  return block_start;
}
internal_symbol memory_size allocatable_size(slab_allocator *slab_allocator)
{
  return memory_size(slab_allocator->start + slab_allocator->size -
                     slab_allocator->unallocated_start);
}
internal_symbol void free(slab_allocator *slab_allocator, memory_address start,
                          memory_size size)
{
  fatal_ifnot(start == slab_allocator->unallocated_start - size);
  slab_allocator->unallocated_start = start;
}
// (DualShock4)
enum DS4Constants {
  DS4Constants_MAGIC = 0x0004ff05,
  DS4Constants_VendorId = 0x54c,
  DS4Constants_ProductId = 0x5c4,
};
#pragma pack(push, 1)
struct DS4Touch {
  u32 id : 7;
  u32 inactive : 1;
  u32 x : 12;
  u32 y : 12;
};
#pragma pack(pop)
#pragma pack(push, 1)
struct DS4 {
  u8 prepad;
  u8 leftx, lefty, rightx, righty;
  u16 buttons;
  u8 trackpad_ps : 2;
  u8 timestamp : 6;
  u8 l2, r2;
  u8 pad[2];
  u8 battery;
  short accel[3], gyro[3];
  u8 pad0[35 - 25];
  DS4Touch touch[2];
  u8 pad2[64 - 43];
};
#pragma pack(pop)
#pragma pack(push, 1)
struct DS4Out DOC("Output message for wired connection")
  URL("https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py",
      "wireless example") {
  u32 magic;
  u8 rumbler, rumblel, r, g, b, flashon, flashoff;
  u8 pad[32 - 11];
};
#pragma pack(pop)
#include "hidapi/hidapi/hidapi.h"
// (Cpu)
internal_symbol inline float64 cpu_abs(float64 x);
internal_symbol inline float64 cpu_sin(float64 x);
internal_symbol inline float64 cpu_cos(float64 x);
internal_symbol inline float64 cpu_sqrt(float64 x);
internal_symbol inline bool cpu_finite(float64 x);
template <> inline bool finite(float32 x) { return cpu_finite(x); }
template <> inline bool finite(float64 x) { return cpu_finite(x); }
template <> inline float32 absolute_value(float32 x)
{
  auto result = cpu_abs(x);
  return result;
}
template <> inline float64 absolute_value(float64 x) { return cpu_abs(x); }
// (Floats)
bool valid(float32 x) { return finite(x); }
// (Vector Math)
struct vec3 MODELS(Regular) {
  float32 x, y, z;
};
internal_symbol vec3 make_vec3(float32 v) { return {v, v, v}; }
internal_symbol vec3 make_vec3(float32 x, float32 y) { return {x, y, 0.0}; }
internal_symbol vec3 make_vec3(float32 x, float32 y, float32 z)
{
  return {x, y, z};
}
internal_symbol bool valid(vec3 x)
{
  return valid(x.x) && valid(x.y) && valid(x.z);
}
internal_symbol bool equality(vec3 a, vec3 b)
{
  return a.x == b.x && a.y == b.y && a.z == b.z;
};
internal_symbol bool default_order(vec3 a, vec3 b)
{
  return a.x < b.x ? true : a.y < b.y ? true : a.z < b.z;
}
internal_symbol vec3 addition(vec3 a, vec3 b)
{
  vec3 result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}
internal_symbol vec3 product(vec3 vector, float32 scalar)
{
  return vec3{vector.x * scalar, vector.y * scalar, vector.z * scalar};
}
internal_symbol vec3 hadamard_product(vec3 a, vec3 b)
{
  return vec3{a.x * b.x, a.y * b.y, a.z * b.z};
}
internal_symbol vec3 cross_product(vec3 a, vec3 b)
{
  return vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.z};
}
internal_symbol float32 squared_euclidean_norm(vec3 v)
{
  float32 xx = v.x * v.x;
  float32 yy = v.y * v.y;
  float32 zz = v.z * v.z;
  return xx + yy + zz;
}
internal_symbol float32 euclidean_norm(vec3 v)
{
  return cpu_sqrt(squared_euclidean_norm(v));
}
internal_symbol float32 squared_euclidean_distance(vec3 a, vec3 b)
{
  return squared_euclidean_norm(addition(b, product(a, -1)));
}
internal_symbol vec3 operator*(vec3 vector, float32 scalar)
{
  return product(vector, scalar);
}
internal_symbol vec3 operator*(float32 scalar, vec3 vector)
{
  return product(vector, scalar);
}
internal_symbol vec3 operator-(vec3 a, vec3 b)
{
  return addition(a, vec3{-b.x, -b.y, -b.z});
}
// (Bounding Box)
struct aabb2 MODELS(SemiRegular) {
  float32 min_x;
  float32 max_x;
  float32 min_y;
  float32 max_y;
};
aabb2 zero_aabb2()
{
  aabb2 result;
  result.min_x = result.min_y = Number<float32>::max;
  result.max_x = result.max_y = Number<float32>::min;
  return result;
}
aabb2 cover(aabb2 bounding_box, float32 point_x, float32 point_y)
{
  // TODO(nicolas): extract min and max in one step
  bounding_box.min_x = min(bounding_box.min_x, point_x);
  bounding_box.max_x = max(bounding_box.max_x, point_x);
  bounding_box.min_y = min(bounding_box.min_y, point_y);
  bounding_box.max_y = max(bounding_box.max_y, point_y);
  return bounding_box;
}
bool intersects(aabb2 a, aabb2 b)
{
  if (a.max_x < b.min_x || a.min_x > b.max_x) return false;
  if (a.max_y < b.min_y || a.min_y > b.max_y) return false;
  return true;
}
aabb2 translate(aabb2 bounding_box, float32 delta_x, float32 delta_y)
{
  bounding_box.min_x += delta_x;
  bounding_box.max_x += delta_x;
  bounding_box.min_y += delta_y;
  bounding_box.max_y += delta_y;
  return bounding_box;
}
aabb2 scale_around_center(aabb2 bounding_box, float32 scaling)
{
  float32 center_x = 0.5 * (bounding_box.min_x + bounding_box.max_x);
  float32 center_y = 0.5 * (bounding_box.min_y + bounding_box.max_y);
  float32 new_width = scaling * (bounding_box.max_x - bounding_box.min_x);
  float32 new_height = scaling * (bounding_box.max_y - bounding_box.min_y);
  aabb2 result;
  result.min_x = center_x - 0.5 * new_width;
  result.max_x = result.min_x + new_width;
  result.min_y = center_y - 0.5 * new_height;
  result.max_y = result.min_y + new_height;
  return result;
}
aabb2 make_symmetric(aabb2 bb)
{
  aabb2 result;
  auto x = max(absolute_value(bb.min_x), absolute_value(bb.max_x));
  auto y = max(absolute_value(bb.min_y), absolute_value(bb.max_y));
  result.min_x = -x;
  result.max_x = x;
  result.min_y = -y;
  result.max_y = y;
  return result;
}
bool contains(aabb2 bb, float32 x, float32 y)
{
  if (x < bb.min_x || x > bb.max_x) return false;
  if (y < bb.min_y || y > bb.max_y) return false;
  return true;
}
// (Multi-Dimensional Arrays)
// PERSON(Pramod Gupta)
// URL(https://www.youtube.com/watch?v=CPPX4kwqh80)
// URL(https://github.com/astrobiology/orca_array)
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
// (Main)
#include "nanovg/src/nanovg.h"
#include "neige_random.hpp"
#include "uu.micros/include/micros/api.h"
#include "uu.micros/include/micros/gl3.h"
#include "uu.ticks/src/render-debug-string/render-debug-string.hpp"
struct TransientMemory;
internal_symbol void vine_effect(TransientMemory& transient_memory, Display const &display, bool paused)
  TAG("visuals");
global_variable hid_device *global_optional_ds4;
internal_symbol NVGcontext *nvg_create_context();
internal_symbol void forget_ds4() { global_optional_ds4 = nullptr; }
internal_symbol hid_device *query_ds4(u64 micros)
{
  // TODO(uucidl): real hid support would question the device for its
  // capabilities. Would we gain knowledge about the DS4 gyro min/max?
  local_state u64 last_check_micros = 0;
  if (!global_optional_ds4 && micros - last_check_micros > 3000000) {
    last_check_micros = micros;
    global_optional_ds4 =
      hid_open(DS4Constants_VendorId, DS4Constants_ProductId, nullptr);
    if (global_optional_ds4) {
      DS4Out ds4_init = {DS4Constants_MAGIC, 0, 0, 25, 175, 15, 0, 0, {}};
      auto hidapi_result =
        hid_write(global_optional_ds4, AddressOf(ds4_init), SizeOf(ds4_init));
      fatal_ifnot(hidapi_result == SizeOf(ds4_init));
      DS4 ds4;
      hidapi_result =
        hid_read(global_optional_ds4, AddressOf(ds4), SizeOf(ds4));
      fatal_ifnot(hidapi_result == SizeOf(ds4));
    }
  }
  return global_optional_ds4;
}
enum GlobalEvents : u64 {
  GlobalEvents_PressedDown = 1 << 0,
};
global_variable u64 global_events = 0;
struct audio_sample_header DOC("a clip of audio data") {
  float64 start_time;
  float64 end_time;
  float64 data_rate_hz;
  float64 root_hz; // tuning frequency in hz
  counted_range<float32, u32> data;
};
struct polyphonic_voice_header {
  float64 phase;
  float64 phase_speed;
  audio_sample_header sample;
};
internal_symbol void make_voice(polyphonic_voice_header *voice,
                                audio_sample_header sample)
{
  voice->phase = 0.0;
  voice->phase_speed = 1.0;
  voice->sample = sample;
}
struct polyphony_header {
  u8 free_voices DOC("bitfield marking each free voice");
  polyphonic_voice_header voices[8];
};
global_variable polyphony_header global_polyphony = {0x08, {}};
struct MainMemory {
  slab_allocator allocator;
};
struct TransientMemory {
  slab_allocator allocator;
  slab_allocator frame_allocator;
};
global_variable struct TransientMemory transient_memory;
global_variable struct MainMemory main_memory;
#include <stdio.h> // for snprintf
void render_next_gl3(unsigned long long micros, Display display)
{
  auto saved_frame_allocator = transient_memory.frame_allocator;
  // NOTE(nicolas): we implement the main program logic as well as its rendering
  // here.
  local_state vec3 background_color = {};
  local_state bool paused = false;
  auto ds4_device = query_ds4(micros);
  if (ds4_device) {
    DS4 ds4;
    auto hidapi_result =
      hid_read(global_optional_ds4, AddressOf(ds4), SizeOf(ds4));
    if (hidapi_result == -1) {
      forget_ds4();
    } else if (hidapi_result == SizeOf(ds4)) {
      if (0) DOC("use the gyro to affect color")
        {
          local_state s16 xmin[3] = {
            (1 << 15) - 1, (1 << 15) - 1, (1 << 15) - 1,
          };
          local_state s16 xmax[3] = {
            -(1 << 15), -(1 << 15), -(1 << 15),
          };
          xmin[0] = min(xmin[0], ds4.gyro[0]);
          xmin[1] = min(xmin[1], ds4.gyro[1]);
          xmin[2] = min(xmin[2], ds4.gyro[2]);
          xmax[0] = max(xmax[0], ds4.gyro[0]);
          xmax[1] = max(xmax[1], ds4.gyro[1]);
          xmax[2] = max(xmax[2], ds4.gyro[2]);
          background_color.x =
            (ds4.gyro[0] - xmin[0]) / float32(xmax[0] - xmin[0]);
          background_color.y =
            (ds4.gyro[1] - xmin[1]) / float32(xmax[1] - xmin[1]);
          background_color.z =
            (ds4.gyro[2] - xmin[2]) / float32(xmax[2] - xmin[2]);
        }
      vec3 green = {0.05f, 1.00f, 0.05f};
      vec3 pink = {1.00f, 0.50f, 0.7f};
      vec3 yellow = {1.0f, 1.0f, 0.0f};
      vec3 grey = {0.7f, 0.7f, 0.7f};
      vec3 bright_blue = {0.3f, 0.7f, 1.0f};
      vec3 white = {1.0f, 1.0f, 1.0f};
      vec3 colors[] = {green, pink, yellow, grey, bright_blue, white};
      u16 colors_size = container_size(colors);
      local_state bool32 was_down = false;
      bool32 is_down = ds4.buttons >> 4;
      auto other_down = ds4.trackpad_ps;
      local_state bool ps_button_state = false;
      auto ps_button_state_now = bool(other_down & 0x01);
      if (ps_button_state_now && ps_button_state_now != ps_button_state)
        DOC("ps/playstation button pressed") { paused = !paused; }
      ps_button_state = ps_button_state_now;
      if (is_down) {
        local_state u16 color_index = 0;
        background_color = colors[color_index];
        if (!was_down && is_down) {
          global_events |= GlobalEvents_PressedDown;
          color_index = successor(color_index) % colors_size;
        }
      }
      was_down = is_down;
    }
  }
  glClearColor(background_color.x, background_color.y, background_color.z, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  background_color = 0.99f * background_color;
  DOC("ui")
  {
    if (ds4_device) {
      draw_debug_string(0, 0, "DS4 plugged in", 2, display.framebuffer_width_px,
                        display.framebuffer_height_px);
    } else {
      draw_debug_string(0, 0, "DS4 not plugged in", 2,
                        display.framebuffer_width_px,
                        display.framebuffer_height_px);
    }
    char free_voices[] = "XXXXXXXX";
    for (memory_size index = 0; index < SizeOf(free_voices) - 1; ++index) {
      bool is_free = global_polyphony.free_voices & (1 << index);
      free_voices[index] = is_free ? 'X' : 'O';
    }
    draw_debug_string(0, 36, free_voices, 2, display.framebuffer_width_px,
                      display.framebuffer_height_px);
    local_state u64 last_micros = micros;
    DOC("display framerate")
    {
      counted_range<char, memory_size> framerate_str;
      alloc_array(&transient_memory.frame_allocator, memory_size(1024),
                  &framerate_str);
      auto last = framerate_str.first;
      last = copy_n_bounded("Period: ", 8, last, end(framerate_str));
      auto n = snprintf(last, memory_size(end(framerate_str) - last), "%f",
                        (micros - last_micros) / 1000.0);
      if (n > 0) {
        last += n;
      }
      sink(last) = 0;
      draw_debug_string(0, 72, begin(framerate_str), 1,
                        display.framebuffer_width_px,
                        display.framebuffer_height_px);
    }
    last_micros = micros;
  }
  DOC("main effect") { vine_effect(transient_memory, display, paused); }
  transient_memory.frame_allocator = saved_frame_allocator;
}
internal_symbol void draw_bounding_box(NVGcontext *vg, aabb2 const bounding_box)
{
  nvgBeginPath(vg);
  nvgMoveTo(vg, bounding_box.min_x, bounding_box.min_y);
  nvgLineTo(vg, bounding_box.max_x, bounding_box.min_y);
  nvgLineTo(vg, bounding_box.max_x, bounding_box.max_y);
  nvgLineTo(vg, bounding_box.min_x, bounding_box.max_y);
  nvgLineTo(vg, bounding_box.min_x, bounding_box.min_y);
  nvgStroke(vg);
}
internal_symbol float32 perlin_noise2(float32 period, float32 x, float32 y);
struct grid2 {
  aabb2 bounding_box;
  float32 step;
};
internal_symbol grid2 make_grid2(aabb2 approximate_bounding_box,
                                 float32 target_step, u32 max_step_count)
{
  auto const &ibb = approximate_bounding_box;
  // ensure grids don't get too dense
  float32 step = target_step;
  while ((ibb.max_y - ibb.min_y) > step * max_step_count) {
    step *= 2;
  }
  grid2 result;
  result.bounding_box.min_x = lower_division(ibb.min_x, step);
  result.bounding_box.max_x = upper_division(ibb.max_x, step);
  result.bounding_box.min_y = lower_division(ibb.min_y, step);
  result.bounding_box.max_y = upper_division(ibb.max_y, step);
  result.step = step;
  return result;
}
#include "vine_effect.cpp"
struct music_event {
  u32 step;
  s8 note_semitones;
};
struct music_header {
  counted_range<music_event, memory_size> events_storage;
  counted_range<music_event, memory_size> events;
  u32 last_step;
};
internal_symbol music_header alloc_music_header(slab_allocator *allocator,
                                                memory_size max_event_count)
{
  music_header result;
  alloc_array(allocator, max_event_count, &result.events_storage);
  result.events = result.events_storage;
  result.events.count = 0;
  result.last_step = 1;
  return result;
}
internal_symbol void push_music_event(music_header *music_header, u32 step,
                                      s8 semitones)
{
  fatal_ifnot(addition_less(container_size(music_header->events),
                            memory_size(1),
                            container_size(music_header->events_storage)));
  auto event = at(music_header->events, container_size(music_header->events));
  ++music_header->events.count;
  event->step = step;
  event->note_semitones = semitones;
}
global_variable audio_sample_header global_audio_samples[1];
global_variable u8 global_audio_sample_index = 0;
global_variable music_header global_music;
global_variable u8 global_music_index = 0;
internal_symbol audio_sample_header get_audio_sample()
{
  return global_audio_samples[global_audio_sample_index];
}
internal_symbol double mix_audio_sample(audio_sample_header audio_sample,
                                        double time, double time_increment,
                                        float64 *destination, u32 mix_count)
{
  double attack_time = 48.0;
  double decay_time = 2 * 48.0;
  double const st = audio_sample.start_time;
  double const et = audio_sample.end_time;
  for_each_n(destination, mix_count, [&](float64 &destination) {
    double const t = time;
    if (t >= st && t < et) {
      double attack_slope = (t - st) / attack_time;
      double decay_slope = (et - t) / decay_time;
      double a = ((t - st) <= attack_time)
                   ? attack_slope
                   : (((et - t) <= decay_time) ? decay_slope : 1.0);
      u32 audio_sample_index = u32(time);
      if (audio_sample_index >= 0 &&
          audio_sample_index < container_size(audio_sample.data)) {
        float64 frac = time - float64(audio_sample_index);
        auto next_sample_index =
          audio_sample_index >= et ? u32(st) : 1 + audio_sample_index;
        float32 x =
          interpolate_linear(audio_sample.data.first[audio_sample_index],
                             audio_sample.data.first[next_sample_index], frac);
        destination = destination + a * x;
      }
      time += time_increment;
    }
  });
  return time;
}
internal_symbol u8 bit_scan_reverse32(u32 x);
static_assert(FixedArrayCount(polyphony_header::voices) >=
                8 * SizeOf(polyphony_header::free_voices),
              "too small bitflag");
extern "C" double pow(double m, double e); // TODO(uucidl): replace libc pow
internal_symbol s8 major_scale_semitones(s8 major_scale_offset)
{
  u8 scale[] = {
    0, 2, 4, 5, 7, 9, 11,
  };
  u8 scale_count = container_size(scale);
  s8 octave_offset = 0;
  while (major_scale_offset < 0) {
    major_scale_offset += scale_count;
    --octave_offset;
  }
  while (major_scale_offset >= scale_count) {
    major_scale_offset -= scale_count;
    ++octave_offset;
  }
  return 12 * octave_offset + scale[major_scale_offset];
}
internal_symbol float64 semitones_freq_hz(float64 root_freq_hz, s8 semitones)
{
  return root_freq_hz * pow(2.0, semitones / 12.0);
}
internal_symbol float64 major_scale_freq_hz(float64 root_freq_hz,
                                            s8 major_scale_offset)
{
  s8 note_semitones = major_scale_semitones(major_scale_offset);
  float64 freq_hz = semitones_freq_hz(root_freq_hz, note_semitones);
  return freq_hz;
}
void render_next_2chn_48khz_audio(unsigned long long now_micros,
                                  int sample_count_init, double *left,
                                  double *right)
{
  u32 sample_count = u32(sample_count_init);
  for_each_n(left, sample_count, [](double &sample) { sample = 0.0; });
  for_each_n(right, sample_count, [](double &sample) { sample = 0.0; });
  if (global_events & GlobalEvents_PressedDown) {
    global_events = global_events & (~GlobalEvents_PressedDown); // consume
    auto first_note = at(global_music.events, global_music_index);
    auto last_note = end(global_music.events);
    for_each_adjacent(
      first_note, last_note,
      [](music_event a, music_event b) { return a.step == b.step; },
      [&](music_event event) {
        if (global_polyphony.free_voices == 0) {
          // NOTE(uucidl): steals oldest voice
          global_polyphony.free_voices |=
            1 << (container_size(global_polyphony.voices) - 1);
        }
        u8 free_voice_index = bit_scan_reverse32(global_polyphony.free_voices);
        auto semitones = event.note_semitones;
        auto voice = global_polyphony.voices + free_voice_index;
        make_voice(voice, get_audio_sample());
        auto freq_hz = semitones_freq_hz(440.0, semitones);
        voice->phase_speed = freq_hz / voice->sample.root_hz *
                             voice->sample.data_rate_hz / 48000.0;
        global_polyphony.free_voices &= ~(1 << free_voice_index);
        global_music_index =
          (global_music_index + 1) % container_size(global_music.events);
      });
  }
  for (u32 voice_index = 0;
       voice_index < container_size(global_polyphony.voices); ++voice_index) {
    auto voice = global_polyphony.voices + voice_index;
    if ((global_polyphony.free_voices & (1 << voice_index)) == 0) {
      auto sample = voice->sample;
      auto phase = voice->phase;
      auto phase_speed = voice->phase_speed;
      if (phase >= sample.start_time && phase < sample.end_time) {
        mix_audio_sample(sample, phase + 0.01, phase_speed, left, sample_count);
        voice->phase =
          mix_audio_sample(sample, phase, phase_speed, right, sample_count);
      } else if (phase >= sample.end_time) {
        global_polyphony.free_voices |= 1 << voice_index; // free the voice
      }
    }
  }
}
internal_symbol audio_sample_header make_audio_sample(
  slab_allocator *slab_allocator, u32 sample_count, float64 data_rate_hz)
{
  audio_sample_header header;
  header.start_time = 0.0;
  header.end_time = sample_count;
  header.data_rate_hz = data_rate_hz;
  alloc_array(slab_allocator, sample_count, &header.data);
  return header;
}
URL("http://www.intel.com/content/www/us/en/processors/"
    "architectures-software-developer-manuals.html")
internal_symbol void fill_sample_with_sin(audio_sample_header sample,
                                          double frequency_hz)
{
  float64 const PI = 3.141592653589793238463;
  float64 phase_increment = 2.0 * PI * frequency_hz / sample.data_rate_hz;
  float64 phase = 0.0;
  for_each_n(begin(sample.data), container_size(sample.data), [&](float32 &y) {
    y = cpu_sin(phase);
    phase = phase + phase_increment;
  });
}
int main(int argc, char **argv) DOC("application entry point")
{
  UNUSED_PARAMETER(argc);
  UNUSED_PARAMETER(argv);
  auto all_memory_size = memory_size(1024 * 1024 * 1024);
  auto all_memory = vm_alloc(all_memory_size);
  auto all_allocator = make_slab_allocator(all_memory, all_memory_size);
  auto transient_memory_size = memory_size(64 * 1024 * 1024);
  transient_memory.allocator = make_slab_allocator(
    alloc(&all_allocator, transient_memory_size), transient_memory_size);
  counted_range<u8, memory_size> frame_memory_block;
  alloc_array(&transient_memory.allocator, memory_size(8 * 1024 * 1024),
              &frame_memory_block);
  transient_memory.frame_allocator =
    make_slab_allocator(frame_memory_block.first, frame_memory_block.count);
  auto main_memory_size = allocatable_size(&all_allocator);
  main_memory.allocator = make_slab_allocator(
    alloc(&all_allocator, main_memory_size), main_memory_size);
  auto music = alloc_music_header(&main_memory.allocator, 1024);
  {
    enum major_scale_note : s8 {
      C_1 = -7,
      D_1 = -6,
      E_1 = -5,
      F_1 = -4,
      G_1 = -3,
      A_1 = -2,
      B_1 = -1,
      C = 0,
      D = 1,
      E = 2,
      F = 3,
      G = 4,
      A = 5,
      B = 6,
    };
#define push_note(__STEP, __NOTE)                                              \
  push_music_event(&music, __STEP, major_scale_semitones(__NOTE))
#if 0
// Frere Jacques
    u32 step = 0;
    for (int count = 2; count--;) {
      push_note(step++, C);
      push_note(step++, D);
      push_note(step++, E);
      push_note(step++, C);
    }
    for (int count = 2; count--;) {
      push_note(step, E);
      push_note(step, C_1);
      ++step;
      push_note(step, F);
      push_note(step, D_1);
      ++step;
      push_note(step, G);
      push_note(step, C_1);
      ++step;
    }
    for (int count = 2; count--;) {
      push_note(step, G);
      push_note(step, E_1);
      ++step;
      push_note(step, A);
      push_note(step, F_1);
      ++step;
      push_note(step, F);
      push_note(step, G_1);
      ++step;
      push_note(step, E);
      push_note(step, C);
      ++step;
    }
    for (int count = 2; count--;) {
      push_note(step, E_1);
      push_note(step, C);
      ++step;
      push_note(step, G_1);
      push_note(step, F_1);
      ++step;
      push_note(step, E_1);
      push_note(step, C);
      ++step;
    }
#else
// Vive Le Vent
#define push_note(__STEP, __NOTE)                                              \
  push_music_event(&music, __STEP, major_scale_semitones(__NOTE))
    u32 step = 0;
    auto prefix = [&]() {
      for (int count = 2; count--;) {
        for (int count_j = 3; count_j--;) {
          push_note(step++, E);
        }
      }
      // m03
      push_note(step++, E);
      push_note(step++, G);
      push_note(step++, C);
      push_note(step++, D);
      // m04
      push_note(step++, E);
      // m05
      for (int count = 3; count--;) {
        push_note(step++, F);
      }
      // m06
      for (int count = 3; count--;) {
        push_note(step++, E);
      }
    };
    prefix();
    // m07
    push_note(step++, E);
    push_note(step++, D);
    push_note(step++, D);
    push_note(step++, E);
    // m08
    push_note(step++, D);
    push_note(step++, G);

    prefix();
    // m15
    push_note(step++, G);
    push_note(step++, G);
    push_note(step++, F);
    push_note(step++, D);
    push_note(step++, C);
#endif
#undef push_note
  }
  global_music = music;
  for_each_n(global_audio_samples, container_size(global_audio_samples),
             [&](audio_sample_header &header) {
               header =
                 make_audio_sample(&main_memory.allocator, 48000 / 2.0, 48000);
               float64 freq_hz = 440.0;
               header.root_hz = freq_hz;
               fill_sample_with_sin(header, freq_hz);
             });
  query_ds4(0);
  runtime_init();
  hid_exit();
  vm_free(all_memory_size, all_memory);
}
// (CPU)
#if defined(COMPILER_CLANG) && (CPU == CPU_IA32 || CPU == CPU_X86_64)
internal_symbol u8 bit_scan_reverse32(u32 x)
{
  u32 y;
  asm("bsrl %1,%0" : "=r"(y) : "r"(x));
  return y;
}
internal_symbol float64 cpu_abs(float64 x)
{
  float64 y;
  asm("fabs" : "=t"(y) : "0"(x));
  return y;
}
internal_symbol float64 cpu_sin(float64 x)
{
  float64 y;
  asm("fsin" : "=t"(y) : "0"(x));
  return y;
}
internal_symbol float64 cpu_cos(float64 x)
{
  float64 y;
  asm("fcos" : "=t"(y) : "0"(x));
  return y;
}
internal_symbol bool cpu_finite(float64 x)
{
  register u16 y asm("eax") = 0x88;
  asm("fxam\n"
      "fstsw %0"
      : "=r"(y)
      : "f"(x));
  union x87_status_register {
    u16 value;
    struct {
      u16 ignored : 8;
      u16 C0 : 1;
      u16 C1 : 1;
      u16 C2 : 1;
      u16 TOP : 3;
      u16 C3 : 1;
    };
  };
  x87_status_register yy;
  yy.value = y;
  u16 triple = (yy.C3 << 2) | (yy.C2 << 1) | (yy.C0);
  return triple == 2 || triple == 4 || triple == 6;
}
internal_symbol float64 cpu_sqrt(float64 x)
{
  float64 y;
  asm("fsqrt" : "=t"(y) : "0"(x));
  return y;
}
#elif defined(COMPILER_MSC) && (CPU == CPU_IA32 || CPU == CPU_X86_64)
#include <intrin.h>
#include <float.h>
#include <math.h>
/* see documentation:
 * URL(https://msdn.microsoft.com/en-us/library/hh977023.aspx) */
internal_symbol u8 bit_scan_reverse32(u32 x)
{
  unsigned long index;
  unsigned long mask = x;
  _BitScanReverse(&index, mask);
  return index;
}
internal_symbol inline float64 cpu_abs(float64 x) { return fabs(x); }
internal_symbol inline float64 cpu_sin(float64 x) { return sin(x); }
internal_symbol inline float64 cpu_cos(float64 x) { return cos(x); }
internal_symbol bool cpu_finite(float64 x) { return _finite(x); }
internal_symbol float64 cpu_sqrt(float64 x) { return sqrtf(x); }
#endif
// (Os)
#if OS == OS_OSX
#include <mach/mach.h> // for vm_allocate
#include <unistd.h>    // for _exit
internal_symbol void fatal() { _exit(-3); }
internal_symbol memory_address vm_alloc(memory_size size)
{
  fatal_ifnot(size > 0);
  vm_address_t address = 0;
  auto vm_allocate_result = vm_allocate(mach_task_self(), &address, size, true);
  fatal_ifnot(KERN_SUCCESS == vm_allocate_result);
  return memory_address(address);
}
internal_symbol void vm_free(memory_size size, memory_address address)
{
  auto vm_deallocate_result =
    vm_deallocate(mach_task_self(), vm_address_t(address), size);
  fatal_ifnot(KERN_SUCCESS == vm_deallocate_result);
}
#elif OS == OS_WINDOWS
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
internal_symbol void fatal() { ExitProcess(3); }
internal_symbol memory_address vm_alloc(memory_size size)
{
  LPVOID address = 0;
  DWORD flAllocationType = MEM_COMMIT | MEM_RESERVE;
  DWORD flProtect = PAGE_READWRITE;
  return memory_address(
    VirtualAlloc(address, size, flAllocationType, flProtect));
}
internal_symbol void vm_free(memory_size size, memory_address data)
{
  VirtualFree(data, size, MEM_RELEASE);
}
#else
#error "Unimplemented OS module"
#endif
// (Micros)
#if OS == OS_OSX
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#pragma clang diagnostic ignored "-Wsign-conversion"
#include "uu.micros/runtime/darwin_runtime.cpp"
#pragma clang diagnostic pop
#elif OS == OS_WINDOWS
#include "uu.micros/runtime/nt_runtime.cpp"
#endif
// (uu.ticks)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-conversion"
#include "uu.ticks/src/render-debug-string/render-debug-string.cpp"
#pragma clang diagnostic pop
// (nanovg)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-conversion"
#pragma clang diagnostic ignored "-Wmissing-field-initializers"
#if OS == OS_WINDOWS
#define _CRT_SECURE_NO_WARNINGS
#endif
#define NANOVG_GL3_IMPLEMENTATION
#include "nanovg/src/nanovg.c"
#include "nanovg/src/nanovg.h"
#include "nanovg/src/nanovg_gl.h"
internal_symbol NVGcontext *nvg_create_context()
{
  u32 flags = NVG_ANTIALIAS | NVG_STENCIL_STROKES;
  flags |= NVG_DEBUG;
  return nvgCreateGL3(flags);
}
#pragma clang diagnostic pop
#if defined(STATIC_GLEW)
#include "uu.micros/libs/glew/src/glew.c"
#endif
// (stb_perlin)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-conversion"
#define STB_PERLIN_IMPLEMENTATION
#include "uu.ticks/modules/stb/stb_perlin.h"
#pragma clang diagnostic pop
internal_symbol float32 perlin_noise2(float32 period, float32 x, float32 y)
{
  auto result = stb_perlin_noise3(x / period, y / period, 0.0f, 256, 256, 256);
  return result;
}
