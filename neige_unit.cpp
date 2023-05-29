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
// (Meta)
#define DOC(...)          // to document a symbol
#define DOC_COUPLING(...) // to document coupling w/ other symbols
#define URL(...)          // reference to a resource
#define TAG(...)          // a tag, for documentation
// (Compiler)
#define global_variable DOC("mark global variables") static
#define local_state DOC("mark locally persistent variables") static
#define internal_symbol DOC("mark internal symbols") static
#define UNUSED_PARAMETER(x) ((void)x)
// NOTE(uucidl): TAG(unsafe), use fixed size array type instead whenever
#define FixedArrayCount(array) (sizeof(array) / sizeof(*array))
#define assert(__predicate_expr)                                               \
  if (!(__predicate_expr)) uu_debugger_break()
#define MODELS(...)
#define REQUIRES(...)
#include "src/integers.cpp"
#include "src/floats.cpp"
#include "src/arithmetics.cpp"
#include "src/machine_types.hpp"
#include "src/pointers.cpp"
#include "src/algorithms.cpp"
#include "src/allocators.cpp"
#include "src/errors.cpp"
namespace uu
{
// (Fixed Array Type)
template <typename T, memory_size N>
constexpr memory_size container_size(T (&)[N])
{
  return N;
}
template <typename T, memory_size N> struct fixed_array_header {
  T (&array)[N];
  fixed_array_header(T (&init_array)[N]) : array(init_array) {}
  T &operator[](memory_size i) { return array[i]; }
};
template <typename T, memory_size N>
fixed_array_header<T, N> make_fixed_array_header(T (&array)[N])
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
} // namespace uu
// (DualShock4)
namespace uu
{
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
} // namespace uu
#include "hidapi/hidapi/hidapi.h"
#ifndef UU_VEC3
#include "src/vec3.cpp"
#endif
namespace uu
{
UU_VEC3_API float euclidean_norm(vec3 v)
{
  return cpu_sqrt(squared_euclidean_norm(v));
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
} // namespace uu
// (Main)
#include "nanovg/src/nanovg.h"
#include "src/neige_random.hpp"
#include "uu.micros/include/micros/api.h"
#include "uu.micros/include/micros/gl3.h"
#include "uu.ticks/src/render-debug-string/render-debug-string.hpp"
namespace uu
{
struct slab_allocator;
} // namespace uu
namespace uu_vine_growing
{
static void vine_effect(uu::slab_allocator *persistent_memory_,
                        uu::slab_allocator *frame_allocator_,
                        uu::float32 framebuffer_width_px,
                        uu::float32 framebuffer_height_px, NVGcontext *nvg);
}
namespace uu
{
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
      uu_fatal_ifnot(hidapi_result == SizeOf(ds4_init));
      DS4 ds4;
      hidapi_result =
        hid_read(global_optional_ds4, AddressOf(ds4), SizeOf(ds4));
      uu_fatal_ifnot(hidapi_result == SizeOf(ds4));
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
} // namespace uu
namespace uu
{
struct MainMemory {
  slab_allocator allocator;
};
struct TransientMemory {
  slab_allocator allocator;
  slab_allocator frame_allocator;
};
}
global_variable struct uu::TransientMemory transient_memory;
global_variable struct uu::MainMemory main_memory;
#include <stdio.h> // for snprintf
void render_next_gl3(unsigned long long micros, Display display)
{
  using namespace uu;
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
  background_color = 0.92f * background_color; // TODO(uucidl): make this frame rate independent
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
    for (memory_size index = 0; index < container_size(free_voices) - 1;
         ++index) {
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
  local_state auto nvg = nvg_create_context();
  glClear(GL_STENCIL_BUFFER_BIT);
  DOC("main effect")
  {
    uu_vine_growing::vine_effect(
      &transient_memory.frame_allocator, &main_memory.allocator,
      display.framebuffer_width_px, display.framebuffer_height_px, nvg);
  }
  transient_memory.frame_allocator = saved_frame_allocator;
}
#include "src/vine_effect.cpp"
namespace uu
{
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
  uu_fatal_ifnot(addition_less(container_size(music_header->events),
                               memory_size(1),
                               container_size(music_header->events_storage)));
  auto event = at(music_header->events, container_size(music_header->events));
  ++music_header->events.count;
  event->step = step;
  event->note_semitones = semitones;
}
} // namespace uu
global_variable uu::audio_sample_header global_audio_samples[1];
global_variable uu::u8 global_audio_sample_index = 0;
global_variable uu::music_header global_music;
global_variable uu::u8 global_music_index = 0;
extern "C" double pow(double m, double e); // TODO(uucidl): replace libc pow
namespace uu
{
internal_symbol uu::audio_sample_header get_audio_sample()
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
} // namespace uu
void render_next_2chn_48khz_audio(unsigned long long now_micros,
                                  int sample_count_init, double *left,
                                  double *right)
{
  using namespace uu;
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
namespace uu
{
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
}
namespace uu
{
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
}
int main(int argc, char **argv) DOC("application entry point")
{
  using namespace uu;
  UNUSED_PARAMETER(argc);
  UNUSED_PARAMETER(argv);
  auto all_memory_size = memory_size(1024 * 1024 * 1024);
  auto all_memory = os_vm_alloc(all_memory_size);
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
  os_vm_free(all_memory_size, all_memory);
}
// (CPU)
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
namespace uu
{
internal_symbol NVGcontext *nvg_create_context()
{
  u32 flags = NVG_ANTIALIAS | NVG_STENCIL_STROKES;
  flags |= NVG_DEBUG;
  return nvgCreateGL3(flags);
}
} // namespace uu
#pragma clang diagnostic pop
#if defined(STATIC_GLEW)
#include "uu.micros/libs/glew/src/glew.c"
#endif
