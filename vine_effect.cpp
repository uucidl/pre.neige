// Vine growing effect
#define UU_VINE_GROWING
#ifndef UU_VINE_GROWING_API
#define UU_VINE_GROWING_API static
#endif
#ifndef UU_VEC3
#include "vec3.cpp"
#endif
#ifndef UU_ALLOCATORS
#include "allocators.cpp"
#endif
#ifndef UU_AABB2
#include "aabb2.cpp"
#endif
#ifndef UU_CIRCULAR_ORDINATE
#include "circular_ordinate.cpp"
#endif
#ifndef UU_POINTERS
#include "pointers.cpp"
#endif
#ifndef UU_ALGORITHMS
#include "algorithms.cpp"
#endif
#ifndef UU_ARITHMETICS
#include "arithmetics.cpp"
#endif
#ifndef UU_ARRAY2
#include "array2.cpp"
#endif
#ifndef UU_CPU
#include "cpu.cpp"
#endif
#ifndef UU_PERLIN_NOISE2
#include "perlin_noise2.cpp"
#endif
#ifndef UU_INTEGERS
#include "integers.cpp"
#endif
#ifndef DOC
#define DOC(...)
#endif
#ifndef TAG
#define TAG(...)
#endif
#ifndef local_storage
#define local_storage static
#endif
#include "nanovg/src/nanovg.h"
#include <cstdio> // snprintf
namespace uu_vine_growing
{
using namespace uu;
enum { NULL_STEM_ID = 0 };
enum { TICKS_PER_SECOND = 60 };
struct stem {
  u32 stem_id;
  vec3 start;
  vec3 end;
  vec3 d_end_dt;    // first derivative
  vec3 dd_end_dtdt; // second derivative
  vec3 density_force;
  u32 generation_count;
  float32 twirl;
  float32 energy_spent;
  float32 life;
  float32 bifurcation_energy_spent;
  bool32 bifurcation_bit;
  float32 bifurcation_threshold;
};
struct vine_stem_history_header {
  u32 stem_id;
  aabb2 bounding_box;
  counted_range<vec3, memory_size> path_storage;
  circular_ordinate<ReadWriteOrdinate<decltype(path_storage)>> last_point_pos;
  memory_size point_count;
  vec3 last_observed_point;
  bool32 recyclable;
};
struct vine_header {
  counted_range<stem, memory_size> stem_storage;
  stem *last_stem_pos;
  counted_range<vine_stem_history_header, memory_size> history_storage;
  vine_stem_history_header *last_history_pos;
  u32 next_stem_id;
};
struct grid2 {
  aabb2 bounding_box;
  float32 step;
};
struct vine_effect_drawables_header {
  aabb2 camera_bb;
  vec3 camera_halfsize;
  vec3 camera_center;
  aabb2 active_points_bb;
  vec3 active_point_average;
  aabb2 eviction_bb;
  grid2 density_grid;
  int framebuffer_width_px;
  int framebuffer_height_px;
  array2_header<float32, u32> density;
  array2_header<bool, u32> density_degenerate_tag;
  vine_header vine;
};
bool valid_stem(stem stem)
{
  return valid(stem.start) && valid(stem.end) && valid(stem.d_end_dt) &&
         valid(stem.dd_end_dtdt) && valid(stem.twirl) &&
         valid(stem.energy_spent) && valid(stem.life) &&
         valid(stem.bifurcation_energy_spent) &&
         valid(stem.bifurcation_threshold);
}
static grid2 make_grid2(aabb2 approximate_bounding_box, float32 target_step,
                        u32 max_step_count)
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
static void draw_bounding_box(NVGcontext *vg, aabb2 const bounding_box)
{
  nvgBeginPath(vg);
  nvgMoveTo(vg, bounding_box.min_x, bounding_box.min_y);
  nvgLineTo(vg, bounding_box.max_x, bounding_box.min_y);
  nvgLineTo(vg, bounding_box.max_x, bounding_box.max_y);
  nvgLineTo(vg, bounding_box.min_x, bounding_box.max_y);
  nvgLineTo(vg, bounding_box.min_x, bounding_box.min_y);
  nvgStroke(vg);
}
void trace_float32(char const *id, float32 x);
void vine_effect_debug_draw(vine_effect_drawables_header const &vine_drawables,
                            NVGcontext *nvg);
UU_VINE_GROWING_API void
vine_effect(slab_allocator *persistent_memory_,
            slab_allocator *frame_allocator_, float32 framebuffer_width_px,
            float32 framebuffer_height_px, NVGcontext *nvg) TAG("visuals")
  DOC("growing a plant like organism")
{
  auto &persistent_memory = *persistent_memory_;
  auto &frame_allocator = *frame_allocator_;
  auto allocate_vine_stem_history = [](slab_allocator *allocator,
                                       memory_size max_path_size) {
    vine_stem_history_header result = {};
    alloc_array(allocator, max_path_size, &result.path_storage);
    return result;
  };
  auto init_vine_stem_history = [](vine_stem_history_header *dest,
                                   u32 stem_id) {
    auto &y = sink(dest);
    y.stem_id = stem_id;
    y.bounding_box = zero_aabb2();
    y.last_point_pos =
      make_circular_ordinate(begin(y.path_storage), end(y.path_storage));
    y.point_count = 0;
    y.last_observed_point = {};
    y.recyclable = false;
  };
  auto init_vine_stem = [](stem *dest, vec3 start, vec3 growth_cm_per_tick,
                           float32 bifurcation_threshold,
                           u32 generation_count) {
    auto &y = sink(dest);
    uu_fatal_if(y.stem_id == NULL_STEM_ID);
    y.start = start;
    y.end = start;
    y.d_end_dt = growth_cm_per_tick;
    y.dd_end_dtdt = make_vec3(0);
    y.energy_spent = 0.0;
    y.life = 1.0;
    y.bifurcation_energy_spent = 0.0;
    y.bifurcation_bit = 0;
    y.twirl = 0.0;
    y.generation_count = generation_count;
    y.bifurcation_threshold = bifurcation_threshold;
  };
  auto push_vine_stem = [=](vine_header *dest) -> PointerOf<stem> {
    auto &y = sink(dest);
    auto stem_pos = step_within(y.stem_storage, &y.last_stem_pos);
    if (stem_pos < end(y.stem_storage)) {
      auto &new_stem = sink(stem_pos);
      new_stem = {};
      new_stem.stem_id = dest->next_stem_id++;
      return &sink(stem_pos);
    } else {
      return nullptr;
    }
  };
  counted_range<stem, u32> created_stems;
  enum { MAX_STEMS_CREATED_PER_FRAME = 16 };
  alloc_array(&frame_allocator, u32(MAX_STEMS_CREATED_PER_FRAME),
              &created_stems);
  stem *last_created_stem = begin(created_stems);
  auto allocate_vine =
    [&](slab_allocator *allocator, memory_size max_stem_count, vec3 start,
        vec3 growth, float32 bifurcation_threshold, memory_size max_path_size) {
      vine_header result;
      alloc_array(allocator, max_stem_count, &result.stem_storage);
      result.last_stem_pos = begin(result.stem_storage);
      alloc_array(allocator, max_stem_count / 10, &result.history_storage);
      result.last_history_pos = begin(result.history_storage);
      for_each_n(begin(result.history_storage),
                 container_size(result.history_storage),
                 [&](vine_stem_history_header &y) {
                   y = allocate_vine_stem_history(allocator, max_path_size);
                 });
      result.next_stem_id = 1;
      auto stem_ptr = push_vine_stem(&result);
      if (stem_ptr) {
        init_vine_stem(stem_ptr, start, growth, bifurcation_threshold, 0);
        set_step_within(created_stems, &last_created_stem, *stem_ptr);
      }
      return result;
    };
  local_storage vine_header vine = allocate_vine(
    &persistent_memory, 600, make_vec3(float32(-7.6), float32(-5.8)),
    make_vec3(float32(4.0 / TICKS_PER_SECOND), float32(7.0 / TICKS_PER_SECOND)),
    float32(0.3), memory_size(30 * TICKS_PER_SECOND));
  local_storage vec3 camera_center = {};
  local_storage vec3 camera_halfsize = product(
    vec3{float32(framebuffer_width_px), float32(framebuffer_height_px), 0.0f},
    20.0 / framebuffer_width_px);
  aabb2 camera_bb = zero_aabb2();
  aabb2 eviction_bb = zero_aabb2();
  aabb2 active_points_bb = zero_aabb2();
  vec3 active_point_average = make_vec3(0);
  grid2 density_grid;
  array2_header<float32, u32> density;
  array2_header<bool, u32> density_degenerate_tag;
  float32 stem_radius = 0.07;
  vec3 active_points_running_sum = {};
  u8 active_points_count = 0;
  auto push_active_point = [&](vec3 point) {
    if (addition_valid(active_points_count, u8(1))) {
      ++active_points_count;
      active_points_running_sum = active_points_running_sum + point;
      active_points_bb = cover(active_points_bb, point.x, point.y);
    }
  };
  if (0)
    DOC("always consider start of effect as part of the camera focus of "
        "attention")
    {
      auto tip = vine.stem_storage.first->start;
      active_points_bb = cover(active_points_bb, tip.x, tip.y);
    }
  auto integrate = [&](stem &stem) {
    // TODO(nicolas): rk4 integration?
    stem.d_end_dt = stem.d_end_dt + stem.dd_end_dtdt;
    stem.end = stem.end + stem.d_end_dt;
  };
  auto twirl_force = [&](stem &stem) {
    auto growth = stem.d_end_dt;
    auto const initial_growth_magnitude_squared =
      squared_euclidean_norm(growth);
    if (initial_growth_magnitude_squared > 0.0) {
      float32 const mag = 0.00071 * (stem.bifurcation_bit ? 1 : -1);
      auto const twirl_threshold = 0.62 * stem.bifurcation_threshold;
      auto const twirl_distance = stem.bifurcation_threshold - twirl_threshold;
      if (stem.generation_count >= 0 && twirl_distance > 0.01 &&
          stem.energy_spent >= twirl_threshold) {
        stem.twirl = 10 * mag *
                     min(1.0, squared(stem.energy_spent - twirl_threshold) /
                                squared(twirl_distance));
      } else {
        stem.twirl = 0.0;
      }
      {
        // TODO(nicolas): dimensional analysis results in something weird:
        vec3 instant_velocity = product(stem.d_end_dt, TICKS_PER_SECOND);
        float32 norm = cpu_sqrt(1.0 / squared_euclidean_norm(instant_velocity));
        vec3 magnetic_pole = product(make_vec3(0.0, 0.0, -1.0), mag);
        vec3 magnetic_force =
          product(cross_product(instant_velocity, magnetic_pole), norm);
        vec3 magnetic_pole1 = product(make_vec3(0.0, 0.0, 1.0), stem.twirl);
        vec3 magnetic_force1 =
          product(cross_product(instant_velocity, magnetic_pole1), norm);
        growth = addition(
          growth, product(addition(magnetic_force, magnetic_force1), 0.5));
      }
      stem.d_end_dt =
        product(growth, cpu_sqrt(initial_growth_magnitude_squared /
                                 squared_euclidean_norm(growth)));
    }
  };
  auto tally_energy_spent = [&](stem &stem) {
    float32 growth_cost_j_per_cm = 0.09;
    auto growth_cost_j =
      growth_cost_j_per_cm * cpu_sqrt(squared_euclidean_norm(stem.d_end_dt));
    stem.energy_spent = stem.energy_spent + growth_cost_j;
    stem.life -= growth_cost_j;
  };
  auto collect_active_point = [&](stem stem) {
    if (stem.generation_count == 0) {
      push_active_point(stem.end);
    }
  };
  auto bifurcate = [&](stem &stem) {
    auto bifurcate_decision_cost_j = 0.071;
    auto new_energy_spent = stem.bifurcation_energy_spent +
                            bifurcate_decision_cost_j *
                              cpu_sqrt(squared_euclidean_norm(stem.d_end_dt));
    auto should_sprout =
      stem.generation_count < 2 &&
      stem.bifurcation_energy_spent < stem.bifurcation_threshold &&
      !(new_energy_spent < stem.bifurcation_threshold);
    if (should_sprout) {
      // bifurcate!
      new_energy_spent = 0; // reset to delay next bifurcation
      auto g = stem.d_end_dt;
      auto v = make_vec3(0, 0, stem.bifurcation_bit ? 1 : -1);
      g = product(g, 0.6) + product(cross_product(g, v), 0.4);
      stem.bifurcation_bit = ~stem.bifurcation_bit;
      auto stem_ptr = push_vine_stem(&vine);
      if (stem_ptr) {
        init_vine_stem(stem_ptr, stem.end, g, 0.9 * stem.bifurcation_threshold,
                       successor(stem.generation_count));
        set_step_within(created_stems, &last_created_stem, *stem_ptr);
      }
    }
    stem.bifurcation_energy_spent = new_energy_spent;
  };
  auto limit_lifespan = [](stem &s) {
    float32 growth_cost_j_per_cm = 0.001;
    s.life -= 0.25 * growth_cost_j_per_cm *
              cpu_sqrt(squared_euclidean_norm(s.d_end_dt));
    if (s.generation_count != 0) {
      if (s.life <= 0.0) {
        s.d_end_dt = make_vec3(0);
      } else if (s.life <= 0.2) {
        s.d_end_dt = product(s.d_end_dt, 0.98);
      }
    }
  };
  auto growth_noise = [](stem &s) {
    // TODO(nicolas): how to visualize effect of growth noise on path?
    // TODO(nicolas): doesn't it introduce some energy into the system?
    auto mag_period_cm = 7.0;
    auto mag = perlin_noise2(mag_period_cm, s.end.x, s.end.y);
    auto fluctuation_dir_period_cm = 12.0;
    auto fluctuation_direction = interpolate_linear(
      s.d_end_dt, make_vec3(0, 1),
      perlin_noise2(fluctuation_dir_period_cm, s.end.x - s.start.x,
                    s.end.y - s.start.y));
    s.end = addition(s.end, product(fluctuation_direction, mag));
  };
  auto strategy_three = [&](stem &stem) {
    integrate(stem);
    collect_active_point(stem);
    tally_energy_spent(stem);
    limit_lifespan(stem);
    stem.dd_end_dtdt = make_vec3(0); /* reset forces */
    bifurcate(stem);
    twirl_force(stem);
#if 0
    growth_noise(stem);
#endif
    uu_fatal_ifnot(valid_stem(stem));
  };
  DOC("grow vine")
  {
    // it can be helpful to sequence various strategies
    for_each(begin(vine.stem_storage), vine.last_stem_pos, strategy_three);
  }
  DOC("initialize state propagation")
  for_each(begin(created_stems), last_created_stem, [&](stem stem) {
    auto pos = step_within(vine.history_storage, &vine.last_history_pos);
    if (pos < end(vine.history_storage)) {
      init_vine_stem_history(&sink(pos), stem.stem_id);
    }
  });
  {
    camera_bb.min_x = -camera_halfsize.x;
    camera_bb.max_x = camera_halfsize.x;
    camera_bb.min_y = -camera_halfsize.y;
    camera_bb.max_y = camera_halfsize.y;
    camera_bb = translate(camera_bb, camera_center.x, camera_center.y);
  }
  eviction_bb = scale_around_center(camera_bb, 2.0);
  density_grid = make_grid2(camera_bb, 1.0, 100);
  if (1) DOC("collect density")
    {
      u32 density_NY =
        (density_grid.bounding_box.max_y - density_grid.bounding_box.min_y) /
        density_grid.step;
      u32 density_NX =
        (density_grid.bounding_box.max_x - density_grid.bounding_box.min_x) /
        density_grid.step;
      u32 NX = density_NX;
      u32 NY = density_NY;
      counted_range<float32, u32> density_memory;
      counted_range<bool, u32> density_degenerate_tag_memory;
      alloc_array(&frame_allocator, density_NX * density_NY, &density_memory);
      alloc_array(&frame_allocator, density_NX * density_NY,
                  &density_degenerate_tag_memory);
      density = make_array2(density_memory.first, density_memory.count,
                            density_NX, density_NY);
      density_degenerate_tag = make_array2(density_degenerate_tag_memory.first,
                                           density_degenerate_tag_memory.count,
                                           density_NX, density_NY);
      fill_n(begin(density), container_size(density), float32(0.0));
      fill_n(begin(density_degenerate_tag),
             container_size(density_degenerate_tag), false);
      {
        float32 const y0 = density_grid.bounding_box.min_y;
        float32 const x0 = density_grid.bounding_box.min_x;
        float32 const block_area_reciprocal = 1.0f / squared(density_grid.step);
        auto const record_stem_density = [&](vec3 pos) {
          float32 xs = (pos.x - x0) / density_grid.step;
          float32 ys = (pos.y - y0) / density_grid.step;
          if (xs < 0.0 || xs >= NX) return;
          if (ys < 0.0 || ys >= NY) return;
          auto xi = u32(xs);
          auto yi = u32(ys);
          *at(density, xi, yi) +=
            (stem_radius * stem_radius) * block_area_reciprocal;
        };
        for_each(begin(vine.stem_storage), vine.last_stem_pos,
                 [&](stem &stem) { record_stem_density(stem.end); });
        for_each(begin(vine.history_storage), vine.last_history_pos,
                 [&](vine_stem_history_header const history) {
                   if (history.stem_id == 0) return;
                   s64 path_n = min(container_size(history.path_storage),
                                    history.point_count);
                   for_each_n(begin(history.path_storage), path_n,
                              [&](vec3 pos) { record_stem_density(pos); });
                 });
      }
    }
  if (1) DOC("janky physics (minimize density)")
    {
      auto NX = density.d0_count;
      auto NY = density.d1_count;
      float32 y0 = density_grid.bounding_box.min_y;
      float32 x0 = density_grid.bounding_box.min_x;
      for (u32 yi = 1; yi < NY - 1; ++yi) {
        for (u32 xi = 1; xi < NX - 1; ++xi) {
          auto v00 = *at(density, xi, yi);
          if (v00 == 0.0) continue;
          auto vl0 = *at(density, xi - 1, yi);
          auto vr0 = *at(density, xi + 1, yi);
          auto v0d = *at(density, xi, yi - 1);
          auto v0u = *at(density, xi, yi + 1);
          auto dvx = (vr0 - vl0) / 2.0;
          auto dvy = (v0u - v0d) / 2.0;
          auto dvec = make_vec3(dvx, dvy);
          if (squared_euclidean_norm(dvec) == 0.0) TAG(degenerate)
            {
              *at(density_degenerate_tag, xi, yi) = true;
              if (v0d == 0.0) {
                dvec = make_vec3(0.0, 1.0);
              } else if (v0u == 0.0) {
                dvec = make_vec3(0.0, -1.0);
              } else if (vl0 == 0.0) {
                dvec = make_vec3(1.0, 0.0);
              }
              dvec = make_vec3(-1.0, 0.0);
            }
          auto du =
            product(dvec, -1.0 / cpu_sqrt(squared_euclidean_norm(dvec)));
          for_each(begin(vine.stem_storage), vine.last_stem_pos,
                   [&](stem &stem) {
                     float32 xs = (stem.end.x - x0) / density_grid.step;
                     float32 ys = (stem.end.y - y0) / density_grid.step;
                     if (xs < 0.0 || xs >= NX) return;
                     if (ys < 0.0 || ys >= NY) return;
                     auto sxi = u32(xs);
                     auto syi = u32(ys);
                     if (sxi != xi || syi != yi) return;
                     auto force = product(du, 0.3 * squared(v00));
                     stem.density_force = force;
                     stem.dd_end_dtdt = stem.dd_end_dtdt + force;
                   });
        }
      }
    }
  if (1) DOC("stream in/out stems that are in simulation region")
    {
      vine.last_stem_pos = sequential_partition_nonstable(
        begin(vine.stem_storage), vine.last_stem_pos, [&](stem &stem) {
          auto must_be_simulated =
            contains(eviction_bb, stem.start.x, stem.start.y) ||
            contains(eviction_bb, stem.end.x, stem.end.y);
          // TODO(nicolas): we should be testing intersection with the
          // segment.
          //
          // TODO(nicolas): actually store the stem in backing store,
          // with its path history (so that we can reconstruct the
          // entire vine if necessary
          return must_be_simulated;
        });
    }
  DOC("propagate stem state to histories")
  {
    vine.last_history_pos = sequential_partition_nonstable(
      begin(vine.history_storage), vine.last_history_pos,
      [&](vine_stem_history_header &history) {
        if (history.stem_id == NULL_STEM_ID) {
          return false; // must delete
        }
        // NOTE(nicolas): TAG(perf) TAG(O(n^2) this linear search
        // makes the overall algo O(n^2). It's ok because the number
        // of stem is small.
        auto stem_pos =
          find_if(begin(vine.stem_storage), vine.last_stem_pos,
                  [=](stem const s) { return s.stem_id == history.stem_id; });
        if (stem_pos >= vine.last_stem_pos) {
          return false;
        }
        auto stem = source(stem_pos);
        auto point = stem.end;
        auto previous_point = source(predecessor(history.last_point_pos));
        auto d_to_previous = squared_euclidean_distance(previous_point, point);
        auto last_observed_point = history.last_observed_point;
        history.last_observed_point = point;
        if (d_to_previous >= squared(0.3)) {
          history.bounding_box = cover(history.bounding_box, point.x, point.y);
          set_step(&history.last_point_pos, point);
          ++history.point_count;
        } else if (point == last_observed_point && history.point_count > 0) {
          history.recyclable = true;
        }
        if (history.recyclable &&
            !intersects(history.bounding_box, camera_bb)) {
          history.stem_id = NULL_STEM_ID;
          return false; // must delete
        }
        return true; // must keep
      });
  }
#if NEIGE_SLOW
  DOC("test partitioning is correct")
  {
    auto first_unallocated =
      find_if(begin(vine.history_storage), vine.last_history_pos,
              [](vine_stem_history_header history) {
                return history.stem_id == NULL_STEM_ID;
              });
    uu_fatal_ifnot(vine.last_history_pos == first_unallocated);
  }
#endif
  if (active_points_count > 0) {
    active_point_average =
      product(active_points_running_sum, 1.0 / active_points_count);
  } else {
    active_point_average = make_vec3(0);
  }
  if (active_points_count > 0)
    DOC("janky camera physics focusing on average point")
    {
      local_storage vec3 point_of_interest = active_point_average;
      point_of_interest =
        interpolate_linear(point_of_interest, active_point_average, 0.025);
      camera_center = point_of_interest;
      active_points_bb = make_symmetric(
        translate(active_points_bb, -camera_center.x, -camera_center.y));
      float32 scale = 1.0;
      float32 tolerance = 1.2;
      if (active_points_bb.max_x > camera_halfsize.x) {
        scale =
          max(scale, tolerance * active_points_bb.max_x / camera_halfsize.x);
      }
      if (active_points_bb.max_y > camera_halfsize.y) {
        scale =
          max(scale, tolerance * active_points_bb.max_y / camera_halfsize.y);
      }
      auto desired_camera_halfsize = product(camera_halfsize, scale);
      if (desired_camera_halfsize.x > 1000.0) {
        desired_camera_halfsize =
          product(desired_camera_halfsize, 1000.0 / desired_camera_halfsize.x);
      }
      camera_halfsize = interpolate_linear(
        camera_halfsize, desired_camera_halfsize, squared(0.05));
      uu_fatal_ifnot(valid(camera_halfsize));
      uu_fatal_ifnot(valid(camera_halfsize));
    }
  vine_effect_debug_draw(
    vine_effect_drawables_header{
      camera_bb, camera_halfsize, camera_center, active_points_bb,
      active_point_average, eviction_bb, density_grid,
      int(framebuffer_width_px), int(framebuffer_height_px), density,
      density_degenerate_tag, vine},
    nvg);
}
void vine_effect_debug_draw(vine_effect_drawables_header const &vine_drawables,
                            NVGcontext *nvg)
{
  auto vg = nvg;
  auto const &camera_bb = vine_drawables.camera_bb;
  auto const &framebuffer_width_px = vine_drawables.framebuffer_width_px;
  auto const &framebuffer_height_px = vine_drawables.framebuffer_height_px;
  auto const &camera_halfsize = vine_drawables.camera_halfsize;
  auto const &camera_center = vine_drawables.camera_center;
  auto const &density = vine_drawables.density;
  auto const &density_grid = vine_drawables.density_grid;
  auto const &density_degenerate_tag = vine_drawables.density_degenerate_tag;
  auto const &vine = vine_drawables.vine;
  auto const &active_points_bb = vine_drawables.active_points_bb;
  auto const &active_point_average = vine_drawables.active_point_average;
  auto const &eviction_bb = vine_drawables.eviction_bb;
  nvgBeginFrame(vg, framebuffer_width_px, framebuffer_height_px,
                1.0); // should be 2.0 on retina
  nvgReset(vg);
  auto const halfwidth = camera_halfsize.x;
  auto const cm_to_display = framebuffer_width_px / (2.0 * halfwidth);
  vec3 center_in_screen_coordinates =
    make_vec3(framebuffer_width_px / 2.0, framebuffer_height_px / 2.0) +
    product(make_vec3(-camera_center.x, -camera_center.y), cm_to_display);
  nvgTranslate(vg, center_in_screen_coordinates.x,
               center_in_screen_coordinates.y);
  nvgScale(vg, cm_to_display, cm_to_display);
  if (1) DOC("grid to get a sense of the space")
    {
      nvgBeginPath(vg);
      nvgFillColor(vg, nvgRGBA(80, 85, 80, 128));
      auto grid = make_grid2(camera_bb, 1.0, 100);
      for (float32 x = grid.bounding_box.min_x; x < grid.bounding_box.max_x;
           x += grid.step) {
        nvgMoveTo(vg, x, grid.bounding_box.max_y);
        nvgLineTo(vg, x, grid.bounding_box.min_y);
      }
      for (float32 y = grid.bounding_box.min_y; y < grid.bounding_box.max_y;
           y += grid.step) {
        nvgMoveTo(vg, grid.bounding_box.max_x, y);
        nvgLineTo(vg, grid.bounding_box.min_x, y);
      }
      nvgFill(vg);
      DOC("display grid position/resolution")
      {
        float savedTransform[6];
        nvgCurrentTransform(vg, savedTransform);
        nvgResetTransform(vg);
        static auto const nvg_font_id = nvgCreateFont(
          vg, "DefaultFont", "assets/Inter UI/Inter-UI-Regular.ttf");
        nvgFontSize(vg, 32);
        nvgTextAlign(vg, NVG_ALIGN_LEFT | NVG_ALIGN_BOTTOM);
        nvgFontFaceId(vg, nvg_font_id);
        char resolution_text[32];
        auto resolution_text_n = std::snprintf(
          resolution_text, sizeof resolution_text, "%0.3f", grid.step);
        char position_text[32];
        auto position_text_n =
          std::snprintf(position_text, sizeof position_text, "%0.3fx%0.3f",
                        camera_bb.min_x, camera_bb.min_y);
        nvgFillColor(vg, nvgRGBA(255, 255, 255, 255));
        if (resolution_text_n < (int)(sizeof resolution_text)) {
          nvgText(vg, 0, framebuffer_height_px, resolution_text, nullptr);
        }
        if (position_text_n < (int)(sizeof position_text)) {
          nvgText(vg, 0, framebuffer_height_px - 32, position_text, nullptr);
        }
        nvgTransform(vg, savedTransform[0], savedTransform[1],
                     savedTransform[2], savedTransform[3], savedTransform[4],
                     savedTransform[5]);
      }
    }
  if (1) DOC("display density grid")
    {
      u32 NX = density.d0_count;
      u32 NY = density.d1_count;
      float32 step = density_grid.step;
      float32 y0 = density_grid.bounding_box.min_y;
      for (u32 yi = 0; yi < NY; ++yi) {
        float32 x0 = density_grid.bounding_box.min_x;
        for (u32 xi = 0; xi < NX; ++xi) {
          float32 d = *at(density, xi, yi);
          nvgBeginPath(vg);
          float alpha = 255.0 * min(1.0, 10.0 * d);
          nvgFillColor(vg, nvgRGBA(80, 125, 80, alpha));
          if (*at(density_degenerate_tag, xi, yi)) {
            nvgFillColor(vg, nvgRGBA(255, 125, 80, alpha));
          }
          nvgRect(vg, x0, y0, step, step);
          nvgFill(vg);
          x0 += step;
        }
        y0 += step;
      }
    }
  nvgBeginPath(vg);
  auto dot_radius = 0.07;
  DOC("draw a vine as a series of dots")
  {
    for_each(begin(vine.history_storage), vine.last_history_pos,
             [&](vine_stem_history_header history) {
               for_each_n(begin(history.path_storage),
                          (s64)min(container_size(history.path_storage),
                                   history.point_count),
                          [&](vec3 p) { nvgCircle(vg, p.x, p.y, dot_radius); });
             });
    nvgFillColor(vg, nvgRGBA(78, 192, 117, 255));
    nvgFill(vg);
  }
  DOC("draw tips")
  {
    nvgBeginPath(vg);
    nvgFillColor(vg, nvgRGBA(138, 138, 108, 255));
    for_each(begin(vine.stem_storage), vine.last_stem_pos, [&](stem stem) {
      nvgCircle(vg, stem.start.x, stem.start.y, dot_radius);
      nvgCircle(vg, stem.end.x, stem.end.y, 2.0 * dot_radius);
    });
    nvgFill(vg);
  }
  if (1) DOC("draw density force")
    {
      float32 scale = 30.0;
      nvgStrokeWidth(vg, 0.1);
      nvgBeginPath(vg);
      nvgStrokeColor(vg, nvgRGBA(188, 188, 108, 255));
      for_each(begin(vine.stem_storage), vine.last_stem_pos, [&](stem stem) {
        nvgMoveTo(vg, stem.end.x, stem.end.y);
        nvgLineTo(vg, stem.end.x + scale * stem.density_force.x,
                  stem.end.y + scale * stem.density_force.y);
      });
      nvgStroke(vg);
    }
  if (0) DOC("draw line from start to end")
    {
      nvgBeginPath(vg);
      for_each(begin(vine.stem_storage), vine.last_stem_pos, [&](stem stem) {
        nvgMoveTo(vg, stem.start.x, stem.start.y);
        nvgLineTo(vg, stem.end.x, stem.end.y);
      });
      nvgFill(vg);
    }
  if (1) DOC("draw average active point / camera / bounding boxes")
    {
      nvgBeginPath(vg);
      nvgFillColor(vg, nvgRGBA(255, 0, 10, 255));
      nvgCircle(vg, active_point_average.x, active_point_average.y,
                0.75 * dot_radius);
      nvgCircle(vg, camera_center.x, camera_center.y, 0.5 * dot_radius);
      nvgFill(vg);
      nvgStrokeWidth(vg, 0.1);
      nvgStrokeColor(vg, nvgRGBA(20, 140, 130, 80));
      draw_bounding_box(vg, active_points_bb);
      nvgStrokeWidth(vg, 0.5);
      nvgStrokeColor(vg, nvgRGBA(150, 140, 30, 80));
      draw_bounding_box(vg, camera_bb);
      draw_bounding_box(vg, eviction_bb);
      nvgStrokeWidth(vg, 0.1);
      for_each(begin(vine.history_storage), vine.last_history_pos,
               [vg](vine_stem_history_header history) {
                 if (history.recyclable) {
                   nvgStrokeColor(vg, nvgRGBA(120, 40, 30, 180));
                 } else {
                   nvgStrokeColor(vg, nvgRGBA(20, 140, 130, 80));
                 }
                 draw_bounding_box(vg, history.bounding_box);
               });
    }
  nvgEndFrame(vg);
}
} // namespace uu_vine_growing
#include <cstdio>
namespace uu_vine_growing
{
void trace_float32(char const *id, float32 x)
{
  if (auto trace = stdout) {
    std::fprintf(trace, "%s\t%f\n", id, x);
  }
}
} // namespace uu_vine_growing
