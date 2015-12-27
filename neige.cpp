/*
  OSX: "clang++ -DOS_OSX -g -std=c++11 -framework System -Wall -Wextra \
  -Wno-tautological-compare neige.cpp \
  -o neige -Iuu.micros/include/ -framework OpenGL -Iuu.micros/libs/glew/include/
  \
  -Iuu.micros/libs -Luu.micros/libs/Darwin_x86_64/ -lglfw3 -framework Cocoa \
  -framework IOKit -framework CoreAudio hidapi/mac/hid.o"
*/
/* TAG(project)
   - TODO(nicolas): chords
   - TODO(nicolas): load sample
 */
// (Platform Configuration)
#define CPU_IA32 (1)
#define CPU_IA64 (2)
#if defined(OS_OSX)
#if !defined(CPU)
#define CPU CPU_IA64
#endif
#endif
#if !defined(COMPILER_CLANG) && defined(__clang__)
#define COMPILER_CLANG
#endif
// (Meta)
#define DOC(...) // to document a symbol
#define URL(...) // reference to a resource
// (Compiler)
#define CLANG_ATTRIBUTE(x) __attribute__((x))
#define debugger_break() DOC("invoke debugger") asm("int3")
#define global_variable DOC("mark global variables") static
#define local_state DOC("mark locally persistent variables") static
#define UNUSED_PARAMETER(x) ((void)x)
// NOTE(uucidl): TAG(unsafe), use fixed size array type instead whenever
#define FixedArrayCount(array) (sizeof(array) / sizeof(*array))
// (Concepts)
#define MODELS(...)
#define REQUIRES(...)
#define Iterator typename
#define Integral typename
#define UnaryFunction typename
#define BinaryFunction typename
// (Word Types)
using u8 = unsigned char;
using u16 = unsigned short;
using u32 = unsigned int;
using u64 = unsigned long;
using s8 = signed char;
using s16 = signed short;
using s32 = signed int;
using s64 = signed long;
using float32 = float;
using float64 = double;
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
#define PointerOf(T) T *
// (DistanceType)
template <typename T> struct distance_type_impl {
};
template <typename T> struct distance_type_impl<PointerOf(T)> {
  using type = u64;
};
#define DistanceType(T) typename distance_type_impl<T>::type
// (Memory)
using memory_address = PointerOf(u8);
using memory_size = DistanceType(memory_address);
static_assert(SizeOf(memory_address) == SizeOf(memory_size),
              "memory_size incorrect for architecture");
// (Fixed Array Type)
template <typename T, memory_size N> constexpr u64 container_size(T(&)[N])
{
  return N;
}
template <typename T, memory_size N> struct fixed_array_header {
  T(&array)[N];
  fixed_array_header(T(&init_array)[N]) : array(init_array) {}
  T &operator[](memory_size i) { return array[i]; };
};
template <typename T, memory_size N>
fixed_array_header<T, N> make_fixed_array_header(T(&array)[N])
{
  fixed_array_header<T, N> header = {array};
  return header;
}
template <typename T, memory_size N>
PointerOf(T) begin(fixed_array_header<T, N> &x)
{
  return &x[0];
}
template <typename T, memory_size N>
PointerOf(T) end(fixed_array_header<T, N> &x)
{
  return begin(x) + container_size(x);
}
template <typename T, memory_size N>
constexpr u64 container_size(fixed_array_header<T, N> const &)
{
  return N;
}
// (Modular Integers)
#define Integer typename
struct modular_integer_tag {
};
template <typename T> struct integer_concept {
};
#define DefineModularInteger(T)                                                \
  template <> struct integer_concept<T> {                                      \
    using concept = modular_integer_tag;                                       \
  };
DefineModularInteger(u8);
DefineModularInteger(u16);
DefineModularInteger(u32);
DefineModularInteger(u64);
DefineModularInteger(s8);
DefineModularInteger(s16);
DefineModularInteger(s32);
DefineModularInteger(s64);
#define IntegerConcept(T) typename integer_concept<T>::concept
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
template <Integer I> bool addition_less(I x, I addand, I limit)
{
  return addition_less(x, addand, limit, IntegerConcept(I)());
}
template <Integer I> bool addition_less_or_equal(I x, I addand, I limit)
{
  return addition_less(x, addand, limit) || ((x + addand) == limit);
}
// (Algorithms)
template <Integral I> bool zero(I x) { return x == I(0); }
template <Integral I> I successor(I x) { return x + 1; }
template <Integral I> I predecessor(I x) { return x - 1; }
template <typename T> T &source(PointerOf(T) x) { return *x; }
template <typename T> T &sink(PointerOf(T) x) { return *x; }
template <typename T> PointerOf(T) successor(PointerOf(T) x) { return x + 1; }
template <typename T> PointerOf(T) predecessor(PointerOf(T) x) { return x - 1; }
template <typename T> memory_address AddressOf(T &x)
{
  return memory_address(&x);
}
template <Iterator InputIterator, Integral I, UnaryFunction Op>
REQUIRES(Domain(Op) == ValueType(InputIterator)) InputIterator
  for_each_n(InputIterator first, I n, Op operation)
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
template <Iterator InputIterator, BinaryFunction P, UnaryFunction Op>
REQUIRES(
  Domain(Op) == ValueType(InputIterator) &&
  HomogeneousFunction(P, ValueType(InputIterator)) &&
  Domain(P) ==
    ValueType(
      InputIterator)) InputIterator for_each_adjacent(InputIterator first,
                                                      InputIterator last,
                                                      P equal, Op operation)
  DOC(
    "in [`first`,`last`) advance iterator `first` and apply `operation` on its "
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
template <Iterator I0, Integral C0, Iterator I1, Integral C1>
REQUIRES(ValueType(I0) == ValueType(I1)) void copy_bounded(I0 from,
                                                           C0 from_size, I1 to,
                                                           C1 to_size)
{
  while (from_size > 0 && to_size > 0) {
    sink(to) = source(from);
    from = successor(from);
    to = successor(to);
    --from_size;
    --to_size;
  }
}
// (Os)
// die if bool is false
#define fatal_ifnot(x)                                                         \
  if (!(x)) {                                                                  \
    debugger_break();                                                          \
    fatal();                                                                   \
  }
void fatal() CLANG_ATTRIBUTE(noreturn)
  DOC("kill the current process and return to the OS");
/// allocate block from the OS' virtual memory
memory_address vm_alloc(memory_size size) DOC("allocate from virtual memory");
/// release a block from the OS' virtual memory
void vm_free(memory_size size, memory_address data)
  DOC("free from virtual memory");
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
      "wireless example")
{
  u32 magic;
  u8 rumbler, rumblel, r, g, b, flashon, flashoff;
  u8 pad[32 - 11];
};
#pragma pack(pop)
#include "hidapi/hidapi/hidapi.h"
// (Main)
#include "uu.micros/include/micros/api.h"
#include "uu.micros/include/micros/gl3.h"
#include "uu.ticks/src/render-debug-string/render-debug-string.hpp"
global_variable hid_device *global_optional_ds4;
void forget_ds4() { global_optional_ds4 = nullptr; }
hid_device *query_ds4(u64 micros)
{
  // TODO(uucidl): real hid support would question the device for its
  // capabilities. Would we gain knowledge about the DS4 gyro min/max?
  local_state u64 last_check_micros = 0;
  if (!global_optional_ds4 && micros - last_check_micros > 3000000) {
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
struct vec3 MODELS(Regular) { float32 x, y, z; };
bool equality(vec3 a, vec3 b)
{
  return a.x == b.y && a.y == b.y && a.z == b.z;
};
bool default_order(vec3 a, vec3 b)
{
  return a.x < b.x ? true : a.y < b.y ? true : a.z < b.z;
}
template <typename T> bool operator<(T a, T b) { return default_order(a, b); }
template <typename T> bool operator==(T a, T b) { return equality(a, b); }
template <typename T> bool operator!=(T a, T b) { return !(a == b); }
template <typename T> T min(T a, T b) { return a < b ? a : b; }
template <typename T> T max(T a, T b) { return !(a < b) ? a : b; }
enum GlobalEvents {
  GlobalEvents_PressedDown = 1 << 0,
};
global_variable u64 global_events = 0;
struct audio_sample_header DOC("a clip of audio data")
{
  float64 start_time;
  float64 end_time;
  float64 data_rate_hz;
  float64 root_hz; // tuning frequency in hz
  float32 *data;
  u32 data_size;
};
struct polyphonic_voice_header {
  float64 phase;
  float64 phase_speed;
  audio_sample_header sample;
};
void make_voice(polyphonic_voice_header *voice, audio_sample_header sample)
{
  voice->phase = 0.0;
  voice->phase_speed = 1.0;
  voice->sample = sample;
}
struct polyphony_header {
  u8 free_voices;
  polyphonic_voice_header voices[8];
};
global_variable polyphony_header global_polyphony = {0x08, {}};
void render_next_gl3(unsigned long long micros, Display display)
{
  local_state vec3 rgb = {};
  auto ds4_device = query_ds4(micros);
  if (ds4_device) {
    DS4 ds4;
    auto hidapi_result =
      hid_read(global_optional_ds4, AddressOf(ds4), SizeOf(ds4));
    if (hidapi_result == -1) {
      forget_ds4();
    } else if (hidapi_result == SizeOf(ds4)) {
#if 0
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
      rgb.x = (ds4.gyro[0] - xmin[0]) / float32(xmax[0] - xmin[0]);
      rgb.y = (ds4.gyro[1] - xmin[1]) / float32(xmax[1] - xmin[1]);
      rgb.z = (ds4.gyro[2] - xmin[2]) / float32(xmax[2] - xmin[2]);
#endif
      vec3 green = {0.05f, 1.00f, 0.05f};
      vec3 pink = {1.00f, 0.50f, 0.7f};
      vec3 yellow = {1.0f, 1.0f, 0.0f};
      vec3 grey = {0.7f, 0.7f, 0.7f};
      vec3 bright_blue = {0.3f, 0.7f, 1.0f};
      vec3 white = {1.0f, 1.0f, 1.0f};
      vec3 colors[] = {green, pink, yellow, grey, bright_blue, white};
      u16 colors_size = container_size(colors);
      local_state bool was_down = false;
      bool is_down = ds4.buttons >> 4;
      if (is_down) {
        local_state u16 color_index = 0;
        rgb = colors[color_index];
        if (!was_down && is_down) {
          global_events |= GlobalEvents_PressedDown;
          color_index = successor(color_index) % colors_size;
        }
      }
      was_down = is_down;
    }
  }
  glClearColor(rgb.x, rgb.y, rgb.z, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  rgb.x *= 0.99f;
  rgb.y *= 0.99f;
  rgb.z *= 0.99f;
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
    for (u64 free_voices_index = 0; free_voices_index < SizeOf(free_voices) - 1;
         ++free_voices_index) {
      free_voices[free_voices_index] =
        (global_polyphony.free_voices & (1 << free_voices_index)) ? 'X' : 'O';
    }
    draw_debug_string(0, 36, free_voices, 2, display.framebuffer_width_px,
                      display.framebuffer_height_px);
  }
}
struct note_in_event {
  u32 event_id;
  s8 note_in_scale;
};
global_variable audio_sample_header global_audio_samples[1];
global_variable u8 global_audio_sample_index = 0;
global_variable u32 global_music_events_count = 0;
global_variable note_in_event *global_music;
global_variable u8 global_music_size;
global_variable u8 global_music_index = 0;
audio_sample_header get_audio_sample()
{
  return global_audio_samples[global_audio_sample_index];
}
template <typename T> T interpolate_linear(T a, T b, float64 a_b)
{
  return (1.0 - a_b) * a + a_b * b;
}
double mix_audio_sample(audio_sample_header audio_sample, double time,
                        double time_increment, double *destination,
                        u32 mix_count)
{
  double attack_time = 48.0;
  double decay_time = 2 * 48.0;
  double const st = audio_sample.start_time;
  double const et = audio_sample.end_time;
  for_each_n(destination, mix_count, [&](double &destination) {
    double const t = time;
    if (t >= st && t < et) {
      double attack_slope = (t - st) / attack_time;
      double decay_slope = (et - t) / decay_time;
      double a = ((t - st) <= attack_time)
                   ? attack_slope
                   : (((et - t) <= decay_time) ? decay_slope : 1.0);
      u32 audio_sample_index = u32(time);
      if (audio_sample_index >= 0 &&
          audio_sample_index < audio_sample.data_size) {
        float64 frac = time - float64(audio_sample_index);
        auto next_sample_index =
          audio_sample_index >= et ? u32(st) : 1 + audio_sample_index;
        float32 x =
          interpolate_linear(audio_sample.data[audio_sample_index],
                             audio_sample.data[next_sample_index], frac);
        destination = destination + a * x;
      }
      time += time_increment;
    }
  });
  return time;
}
u8 bit_scan_reverse32(u32 x);
static_assert(FixedArrayCount(polyphony_header::voices) >=
                8 * SizeOf(polyphony_header::free_voices),
              "too small bitflag");
extern "C" double pow(double m, double e); // TODO(uucidl): replace libc pow
float64 major_scale_freq_hz(float64 root_freq_hz, s8 major_scale_offset)
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
  float64 note =
    float64(octave_offset) + float64(scale[major_scale_offset]) / 12.0;
  float64 freq_hz = root_freq_hz * pow(2.0, note);
  return freq_hz;
}
void render_next_2chn_48khz_audio(unsigned long long, int sample_count,
                                  double *left, double *right)
{
  for_each_n(left, sample_count, [](double &sample) { sample = 0.0; });
  for_each_n(right, sample_count, [](double &sample) { sample = 0.0; });
  if (global_events & GlobalEvents_PressedDown) {
    global_events = global_events & (~GlobalEvents_PressedDown); // consume
    auto first_note = global_music + global_music_index;
    auto last_note = global_music + global_music_size;
    for_each_adjacent(
      first_note, last_note,
      [](note_in_event a, note_in_event b) { return a.event_id == b.event_id; },
      [&](note_in_event note) {
        if (global_polyphony.free_voices == 0) {
          // NOTE(uucidl): steals oldest voice
          global_polyphony.free_voices |=
            1 << (container_size(global_polyphony.voices) - 1);
        }
        u8 free_voice_index = bit_scan_reverse32(global_polyphony.free_voices);
        auto note_in_scale = note.note_in_scale;
        auto voice = global_polyphony.voices + free_voice_index;
        make_voice(voice, get_audio_sample());
        auto freq_hz = major_scale_freq_hz(440.0, note_in_scale);
        voice->phase_speed = freq_hz / voice->sample.root_hz *
                             voice->sample.data_rate_hz / 48000.0;
        global_polyphony.free_voices &= ~(1 << free_voice_index);
        global_music_index = (global_music_index + 1) % global_music_size;
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
struct slab_allocator {
  memory_address start;
  memory_address unallocated_start;
  memory_size size;
};
slab_allocator make_slab_allocator(memory_address start, memory_size size)
{
  slab_allocator result;
  result.start = start;
  result.unallocated_start = start;
  result.size = size;
  return result;
}
memory_address alloc(slab_allocator *slab_allocator, memory_size size)
{
  memory_size current_size =
    slab_allocator->unallocated_start - slab_allocator->start;
  fatal_ifnot(addition_less_or_equal(current_size, size, slab_allocator->size));
  slab_allocator->unallocated_start += size;
  return slab_allocator->unallocated_start;
}
void free(slab_allocator *slab_allocator, memory_address start,
          memory_size size)
{
  fatal_ifnot(start == slab_allocator->unallocated_start - size);
  slab_allocator->unallocated_start = start;
}
template <typename T>
void alloc_array(PointerOf(slab_allocator) slab_allocator, memory_size count,
                 PointerOf(PointerOf(T)) dest_pointer_output)
  DOC("allocate enough room for a contiguous array and copy it to "
      "`dest_pointer_output`")
{
  sink(dest_pointer_output) =
    reinterpret_cast<PointerOf(T)>(alloc(slab_allocator, SizeOf(T) * count));
}
audio_sample_header make_audio_sample(slab_allocator *slab_allocator,
                                      u32 sample_count, float64 data_rate_hz)
{
  audio_sample_header header;
  header.start_time = 0.0;
  header.end_time = sample_count;
  header.data_rate_hz = data_rate_hz;
  header.data_size = sample_count;
  alloc_array(slab_allocator, header.data_size, &header.data);
  return header;
}
URL("http://www.intel.com/content/www/us/en/processors/"
    "architectures-software-developer-manuals.html")
double cpu_sin(double x);
void fill_sample_with_sin(audio_sample_header sample, double frequency_hz)
{
  float64 const PI = 3.141592653589793238463;
  float64 phase_increment = 2.0 * PI * frequency_hz / sample.data_rate_hz;
  float64 phase = 0.0;
  for (u32 index = 0; index < sample.data_size; ++index) {
    sample.data[index] = cpu_sin(phase);
    phase = phase + phase_increment;
  }
}
int main(int argc, char **argv) DOC("application entry point")
{
  UNUSED_PARAMETER(argc);
  UNUSED_PARAMETER(argv);
  auto memory_size = 1024 * 1024 * 1024;
  auto memory = vm_alloc(memory_size);
  auto slab_allocator = make_slab_allocator(memory, memory_size);
  u32 vive_le_vent_next_event_id = 0;
  s8 vive_le_vent_G[] = {
    2, 2, 2,    // m01
    2, 2, 2,    // m02
    2, 4, 0, 1, // m03
    2,          // m4
    3, 3, 3,    // m05
    2, 2, 2,    // m06
    2, 1, 1, 2, // m07
    1, 4,       // m08
    2, 2, 2,    // m09
    2, 2, 2,    // m10
    2, 4, 0, 1, // m11
    2,          // m12
    3, 3, 3,    // m13
    2, 2, 2,    // m14
    4, 4, 3, 1, // m15
    0,          // m16
  };
  u32 frere_jacques_next_event_id = 0;
#define STEP (frere_jacques_next_event_id++)
#define SAME (frere_jacques_next_event_id)
#define C_1 (-7)
#define D_1 (-6)
#define E_1 (-5)
#define F_1 (-4)
#define G_1 (-3)
#define A_1 (-2)
#define B_1 (-1)
#define C (0)
#define D (1)
#define E (2)
#define F (3)
#define G (4)
#define A (5)
#define B (6)
  note_in_event frere_jacques_G[] = {
    // m00
    {STEP, C},
    {STEP, D},
    {STEP, E},
    {STEP, C},
    // m01
    {STEP, C},
    {STEP, D},
    {STEP, E},
    {STEP, C},
    // m02
    {SAME, C_1},
    {STEP, E},
    {SAME, D_1},
    {STEP, F},
    {SAME, E_1},
    {STEP, G},
    {STEP, C_1},
    // m03
    {SAME, C_1},
    {STEP, E},
    {SAME, D_1},
    {STEP, F},
    {SAME, E_1},
    {STEP, G},
    {STEP, C_1},
    // m04
    {SAME, E_1},
    {STEP, G},
    {SAME, F_1},
    {STEP, G},
    {SAME, F_1},
    {STEP, G},
    {STEP, F},
    {SAME, G_1},
    {STEP, E},
    {STEP, C},
    // m05
    {SAME, E_1},
    {STEP, G},
    {SAME, F_1},
    {STEP, A},
    {SAME, F_1},
    {STEP, G},
    {STEP, F},
    {SAME, G_1},
    {STEP, E},
    {STEP, C},
    // m06
    {SAME, E_1},
    {STEP, C},
    {SAME, G_1},
    {STEP, F_1},
    {SAME, E_1},
    {STEP, C},
    // m07
    {SAME, E_1},
    {STEP, C},
    {SAME, G_1},
    {STEP, F_1},
    {SAME, E_1},
    {STEP, C},
  };
#undef STEP
#undef SAME
  auto sequence = make_fixed_array_header(frere_jacques_G);
  auto sequence_next_event_id = frere_jacques_next_event_id;
  global_music_size = container_size(sequence);
  global_music_events_count = sequence_next_event_id;
  alloc_array(&slab_allocator, container_size(sequence), &global_music);
  copy_bounded(begin(sequence), container_size(sequence), global_music,
               global_music_size);
  {
    auto first_note = begin(sequence);
    auto const last_note = end(sequence);
    while (first_note != last_note) {
      first_note = for_each_adjacent(
        first_note, last_note,
        [](note_in_event a, note_in_event b) {
          return a.event_id == b.event_id;
        },
        [](note_in_event note) { printf("\tnote: %d\n", note.note_in_scale); });
      printf("----\n");
    }
  }
  for_each_n(global_audio_samples, container_size(global_audio_samples),
             [&](audio_sample_header &header) {
               header = make_audio_sample(&slab_allocator, 48000 / 2.0, 48000);
               float64 freq_hz = 440.0;
               header.root_hz = freq_hz;
               fill_sample_with_sin(header, freq_hz);
             });
  query_ds4(0);
  runtime_init();
  hid_exit();
  vm_free(1024, memory);
}
// CPU
#if defined(COMPILER_CLANG) && (CPU == CPU_IA32 || CPU == CPU_IA64)
double cpu_sin(double x)
{
  double y;
  asm("fld %0\nfsin" : "=t"(y) : "f"(x));
  return y;
}
u8 bit_scan_reverse32(u32 x)
{
  u32 y;
  asm("bsrl %1,%0" : "=r"(y) : "r"(x));
  return y;
}
#endif
// (Os)
#if defined(OS_OSX)
#include <unistd.h>    // for _exit
#include <mach/mach.h> // for vm_allocate
void fatal() { _exit(-3); }
memory_address vm_alloc(memory_size size)
{
  fatal_ifnot(size > 0);
  vm_address_t address = 0;
  auto vm_allocate_result = vm_allocate(mach_task_self(), &address, size, true);
  fatal_ifnot(KERN_SUCCESS == vm_allocate_result);
  return memory_address(address);
}
void vm_free(memory_size size, memory_address address)
{
  auto vm_deallocate_result =
    vm_deallocate(mach_task_self(), vm_address_t(address), size);
  fatal_ifnot(KERN_SUCCESS == vm_deallocate_result);
}
#else
#error "Unimplemented OS module"
#endif
// (Micros)
#if defined(OS_OSX)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#include "uu.micros/runtime/darwin_runtime.cpp"
#include "uu.micros/libs/glew/src/glew.c"
#pragma clang diagnostic pop
#endif
// (uu.ticks)
#include "uu.ticks/src/render-debug-string/render-debug-string.cpp"
