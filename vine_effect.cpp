/* Vine growing effect */
enum { NULL_STEM_ID = 0 };
enum { TICKS_PER_SECOND = 60 };
struct stem {
  u32 stem_id;
  vec3 start;
  vec3 end;
  vec3 d_end_dt;    // first derivative
  vec3 dd_end_dtdt; // second derivative
  vec3 density_force;
  bool apply_density_force;
  u32 generation_count;
  float32 twirl;
  float32 energy_spent;
  float32 life;
  float32 bifurcation_energy_spent;
  bool32 bifurcation_bit;
  float32 bifurcation_threshold;
};
bool valid_stem(stem stem)
{
  return valid(stem.start) && valid(stem.end) && valid(stem.d_end_dt) &&
         valid(stem.dd_end_dtdt) && valid(stem.twirl) &&
         valid(stem.energy_spent) && valid(stem.life) &&
         valid(stem.bifurcation_energy_spent) &&
         valid(stem.bifurcation_threshold);
}
internal_symbol void vine_effect(Display const &display, bool paused)
  TAG("visuals") DOC("growing a plant like organism")
{
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
    fatal_if(y.stem_id == NULL_STEM_ID);
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
  alloc_array(&transient_memory.frame_allocator,
              u32(MAX_STEMS_CREATED_PER_FRAME), &created_stems);
  stem *last_created_stem = begin(created_stems);
  auto allocate_vine = [&](
    slab_allocator *allocator, memory_size max_stem_count, vec3 start,
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
  local_state vine_header vine = allocate_vine(
    &main_memory.allocator, 600, make_vec3(float32(-7.6), float32(-5.8)),
    make_vec3(float32(4.0 / TICKS_PER_SECOND), float32(7.0 / TICKS_PER_SECOND)),
    float32(0.3), memory_size(30 * TICKS_PER_SECOND));
  local_state vec3 camera_center = {};
  local_state vec3 camera_halfsize =
    20.0 / display.framebuffer_width_px *
    vec3{float32(display.framebuffer_width_px),
         float32(display.framebuffer_height_px), 0.0f};
  aabb2 camera_bb = zero_aabb2();
  aabb2 eviction_bb = zero_aabb2();
  aabb2 active_points_bounding_box = zero_aabb2();
  vec3 active_point_average = make_vec3(0);
  grid2 simulation_grid;
  array2_header<float32, u32> density;
  if (paused) goto rendering;
  {
    vec3 active_points_running_sum = {};
    u8 active_points_count = 0;
    auto push_active_point = [&](vec3 point) {
      if (addition_valid(active_points_count, u8(1))) {
        ++active_points_count;
        active_points_running_sum = active_points_running_sum + point;
        active_points_bounding_box =
          cover(active_points_bounding_box, point.x, point.y);
      }
    };
    if (0)
      DOC("always consider start of effect as part of the camera focus of "
          "attention")
      {
        auto tip = vine.stem_storage.first->start;
        active_points_bounding_box =
          cover(active_points_bounding_box, tip.x, tip.y);
      }
    auto integrate = [&](stem &stem) {
      // TODO(nicolas): rk4 integration?
      stem.d_end_dt = stem.d_end_dt + stem.dd_end_dtdt;
      stem.end = stem.end + stem.d_end_dt;
    };
    auto twirl_force = [&](stem &stem) {
      auto growth = stem.d_end_dt;
      auto const initial_growth_magnitude = euclidean_norm(growth);
      if (initial_growth_magnitude > 0.0) {
        float32 const mag = 0.00071 * (stem.bifurcation_bit ? 1 : -1);
        auto const twirl_threshold = 0.62 * stem.bifurcation_threshold;
        auto const twirl_distance =
          stem.bifurcation_threshold - twirl_threshold;
        if (stem.generation_count >= 0 && twirl_distance > 0.01 &&
            stem.energy_spent >= twirl_threshold) {
          stem.twirl =
            10 * mag * min(1.0, square(stem.energy_spent - twirl_threshold) /
                                  square(twirl_distance));
        } else {
          stem.twirl = 0.0;
        }
        {
          // TODO(nicolas): dimensional analysis results in something weird:
          vec3 instant_velocity = stem.d_end_dt * TICKS_PER_SECOND;
          float32 norm = 1.0 / euclidean_norm(instant_velocity);
          vec3 magnetic_pole = mag * make_vec3(0.0, 0.0, -1.0);
          vec3 magnetic_force =
            norm * cross_product(instant_velocity, magnetic_pole);
          vec3 magnetic_pole1 = stem.twirl * make_vec3(0.0, 0.0, 1.0);
          vec3 magnetic_force1 =
            norm * cross_product(instant_velocity, magnetic_pole1);
          growth = growth + 0.5 * (magnetic_force + magnetic_force1);
        }
        stem.d_end_dt =
          (initial_growth_magnitude / euclidean_norm(growth)) * growth;
      }
    };
    auto tally_energy_spent = [&](stem &stem) {
      float32 growth_cost_j_per_cm = 0.09;
      auto growth_cost_j = growth_cost_j_per_cm * euclidean_norm(stem.d_end_dt);
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
      auto new_energy_spent =
        stem.bifurcation_energy_spent +
        bifurcate_decision_cost_j * euclidean_norm(stem.d_end_dt);
      auto should_sprout =
        stem.generation_count < 2 &&
        stem.bifurcation_energy_spent < stem.bifurcation_threshold &&
        !(new_energy_spent < stem.bifurcation_threshold);
      if (should_sprout) {
        // bifurcate!
        new_energy_spent = 0; // reset to delay next bifurcation
        auto g = stem.d_end_dt;
        auto v = make_vec3(0, 0, stem.bifurcation_bit ? 1 : -1);
        g = 0.6 * g + 0.4 * cross_product(g, v);
        stem.bifurcation_bit = ~stem.bifurcation_bit;
        auto stem_ptr = push_vine_stem(&vine);
        if (stem_ptr) {
          init_vine_stem(stem_ptr, stem.end, g,
                         0.9 * stem.bifurcation_threshold,
                         successor(stem.generation_count));
          set_step_within(created_stems, &last_created_stem, *stem_ptr);
        }
      }
      stem.bifurcation_energy_spent = new_energy_spent;
    };
    auto limit_lifespan = [](stem &s) {
      float32 growth_cost_j_per_cm = 0.001;
      s.life -= 0.25 * growth_cost_j_per_cm * euclidean_norm(s.d_end_dt);
      if (s.generation_count != 0) {
        if (s.life <= 0.0) {
          s.d_end_dt = make_vec3(0);
        } else if (s.life <= 0.2) {
          s.d_end_dt = 0.98 * s.d_end_dt;
        }
      }
    };
    auto growth_noise = [](stem &s) {
      auto fluctuation_period_cm = 6.0;
      auto fluctuation_dir_period_cm = 12.0;
      auto fluctuation = perlin_noise2(fluctuation_period_cm, s.end.x, s.end.y);
      auto fluctuation_direction = interpolate_linear(
        s.d_end_dt, make_vec3(0, 1),
        perlin_noise2(fluctuation_dir_period_cm, s.end.x - s.start.x,
                      s.end.y - s.start.y));
      auto magnitude = 0.8;
      s.end = s.end + magnitude * fluctuation_direction * fluctuation;
    };
    auto strategy_three = [&](stem &stem) {
      integrate(stem);
      assert(valid_stem(stem));
      collect_active_point(stem);
      tally_energy_spent(stem);
      limit_lifespan(stem);
      stem.dd_end_dtdt = make_vec3(0); /* reset forces */
      bifurcate(stem);
      assert(valid_stem(stem));
#if NEIGE_DISABLED
      twirl_force(stem);
      assert(valid_stem(stem));
      assert(valid_stem(stem));
      growth_noise(stem);
      assert(valid_stem(stem));
#endif
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
    aabb2 eviction_bb = scale_around_center(camera_bb, 2.0);
    simulation_grid = make_grid2(camera_bb, 1.5, 100);
    if (1) DOC("collect density")
      {
        u32 density_NY = (simulation_grid.bounding_box.max_y -
                          simulation_grid.bounding_box.min_y) /
                         simulation_grid.step;
        u32 density_NX = (simulation_grid.bounding_box.max_x -
                          simulation_grid.bounding_box.min_x) /
                         simulation_grid.step;
        u32 NX = density_NX;
        u32 NY = density_NY;
        counted_range<float32, u32> density_memory;
        alloc_array(&transient_memory.frame_allocator, density_NX * density_NY,
                    &density_memory);
        density = make_array2(density_memory.first, density_memory.count,
                              density_NX, density_NY);
        fill_n(begin(density), container_size(density), float32(0.0));

        {
          float32 y0 = simulation_grid.bounding_box.min_y;
          float32 x0 = simulation_grid.bounding_box.min_x;
          for_each(begin(vine.stem_storage), vine.last_stem_pos,
                   [&](stem &stem) {
                     float32 xs = (stem.end.x - x0) / simulation_grid.step;
                     float32 ys = (stem.end.y - y0) / simulation_grid.step;
                     if (xs < 0.0 || xs >= NX) return;
                     if (ys < 0.0 || ys >= NY) return;
                     auto xi = u32(xs);
                     auto yi = u32(ys);
                     *at(density, xi, yi) += 1.0;
                   });
          for_each(
            begin(vine.history_storage), vine.last_history_pos,
            [&](vine_stem_history_header const history) {
              if (history.stem_id == 0) return;
              for_each_n(
                begin(history.path_storage),
                min(container_size(history.path_storage), history.point_count),
                [&](vec3 pos) {
                  float32 xs = (pos.x - x0) / simulation_grid.step;
                  float32 ys = (pos.y - y0) / simulation_grid.step;
                  if (xs < 0.0 || xs >= NX) return;
                  if (ys < 0.0 || ys >= NY) return;
                  auto xi = u32(xs);
                  auto yi = u32(ys);
                  *at(density, xi, yi) += 1.0;
                });
            });
        }
      }
    if (1) DOC("janky physics (minimize density)")
      {
        auto NX = density.d0_count;
        auto NY = density.d1_count;
        float32 y0 = simulation_grid.bounding_box.min_y;
        float32 x0 = simulation_grid.bounding_box.min_x;
        float32 step = simulation_grid.step;
        float32 y1 = y0 + step;
        float32 x1 = x0 + step;
        for (u32 yi = 1; yi < NY - 1; ++yi) {
          for (u32 xi = 1; xi < NX - 1; ++xi) {
            auto v00 = *at(density, xi, yi) / step;
            if (v00 == 0.0) continue;
            auto vl0 = *at(density, xi + 1, yi) / step;
            auto vr0 = *at(density, xi - 1, yi) / step;
            auto v0d = *at(density, xi, yi - 1) / step;
            auto v0u = *at(density, xi, yi + 1) / step;
            auto dvx = (vr0 - vl0) / 2.0 / step;
            auto dvy = (v0u - v0d) / 2.0 / step;
            auto dvec = make_vec3(dvx, dvy);
            auto dvec_norm = euclidean_norm(dvec);
            if (dvec_norm == 0.0) TAG(degenerate)
              {
                if (v0d == 0.0) {
                  dvec = make_vec3(0.0, 1.0);
                } else if (v0u == 0.0) {
                  dvec = make_vec3(0.0, -1.0);
                } else if (vl0 == 0.0) {
                  dvec = make_vec3(1.0, 0.0);
                }
                dvec = make_vec3(-1.0, 0.0);
              }
            auto du = (-1.0 / euclidean_norm(dvec)) * dvec;
            for_each(begin(vine.stem_storage), vine.last_stem_pos,
                     [&](stem &stem) {
                       float32 xs = (stem.end.x - x0) / simulation_grid.step;
                       float32 ys = (stem.end.y - y0) / simulation_grid.step;
                       if (xs < 0.0 || xs >= NX) return;
                       if (ys < 0.0 || ys >= NY) return;
                       auto sxi = u32(xs);
                       auto syi = u32(ys);
                       if (sxi != xi || syi != yi) return;
                       auto force = 0.003 * square(v00 - 1.0) * du;
                       stem.density_force = force;
                       stem.dd_end_dtdt = stem.dd_end_dtdt + force;
                     });

            x0 = x1;
            x1 += step;
          }
          y0 = y1;
          y1 += step;
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
          auto d_to_previous =
            squared_euclidean_distance(previous_point, point);
          auto last_observed_point = history.last_observed_point;
          history.last_observed_point = point;
          if (d_to_previous >= square(0.3)) {
            history.bounding_box =
              cover(history.bounding_box, point.x, point.y);
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
      assert(vine.last_history_pos == first_unallocated);
    }
#endif
    if (active_points_count > 0) {
      active_point_average =
        (1.0 / active_points_count) * active_points_running_sum;
    } else {
      active_point_average = make_vec3(0);
    }
    if (active_points_count > 0)
      DOC("janky camera physics focusing on average point")
      {
        local_state vec3 point_of_interest = active_point_average;
        point_of_interest =
          interpolate_linear(point_of_interest, active_point_average, 0.025);
        camera_center = point_of_interest;
        auto active_points_bb = make_symmetric(translate(
          active_points_bounding_box, -camera_center.x, -camera_center.y));
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
        auto desired_camera_halfsize = camera_halfsize * scale;
        if (desired_camera_halfsize.x > 1000.0) {
          desired_camera_halfsize =
            1000.0 / desired_camera_halfsize.x * desired_camera_halfsize;
        }
        camera_halfsize = interpolate_linear(
          camera_halfsize, desired_camera_halfsize, square(0.05));
        assert(valid(camera_halfsize));
        fatal_ifnot(valid(camera_halfsize));
      }
  }
rendering:
  local_state auto vg = nvg_create_context();
  glClear(GL_STENCIL_BUFFER_BIT);
  {
    nvgBeginFrame(vg, int(display.framebuffer_width_px),
                  int(display.framebuffer_height_px),
                  1.0); // should be 2.0 on retina
    nvgReset(vg);
    auto halfwidth = camera_halfsize.x;
    if (0) DOC("turn debug camera on")
      {
        halfwidth = 0.5 * 1.5 * (eviction_bb.max_x - eviction_bb.min_x);
      }
    auto cm_to_display = display.framebuffer_width_px / (2.0 * halfwidth);
    vec3 center_in_screen_coordinates =
      make_vec3(display.framebuffer_width_px / 2.0,
                display.framebuffer_height_px / 2.0) +
      cm_to_display * make_vec3(-camera_center.x, camera_center.y);
    nvgTranslate(vg, center_in_screen_coordinates.x,
                 center_in_screen_coordinates.y);
    nvgScale(vg, cm_to_display, -cm_to_display);
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
      }
    if (1) DOC("display density grid")
      {
        u32 NX = density.d0_count;
        u32 NY = density.d1_count;
        float32 step = simulation_grid.step;
        float32 y0 = simulation_grid.bounding_box.min_y;
        for (u32 yi = 0; yi < NY; ++yi) {
          float32 x0 = simulation_grid.bounding_box.min_x;
          for (u32 xi = 0; xi < NX; ++xi) {
            float32 d = *at(density, xi, yi) / 4.0;
            nvgBeginPath(vg);
            nvgFillColor(vg, nvgRGBA(80, 125, 80, 128.0 * d));
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
      for_each(
        begin(vine.history_storage), vine.last_history_pos,
        [&](vine_stem_history_header history) {
          for_each_n(
            begin(history.path_storage),
            min(container_size(history.path_storage), history.point_count),
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
        nvgStrokeColor(vg, nvgRGBA(88, 88, 108, 255));
        for_each(begin(vine.stem_storage), vine.last_stem_pos, [&](stem stem) {
          if (stem.apply_density_force) return;
          nvgMoveTo(vg, stem.end.x, stem.end.y);
          nvgLineTo(vg, stem.end.x + scale * stem.density_force.x,
                    stem.end.y + scale * stem.density_force.y);
        });
        nvgStroke(vg);
        nvgBeginPath(vg);
        nvgStrokeColor(vg, nvgRGBA(188, 188, 108, 255));
        for_each(begin(vine.stem_storage), vine.last_stem_pos, [&](stem stem) {
          if (!stem.apply_density_force) return;
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
    if (0) DOC("draw average active point / camera / bounding boxes")
      {
        nvgBeginPath(vg);
        nvgFillColor(vg, nvgRGBA(255, 0, 10, 255));
        nvgCircle(vg, active_point_average.x, active_point_average.y,
                  0.75 * dot_radius);
        nvgCircle(vg, camera_center.x, camera_center.y, 0.5 * dot_radius);
        nvgFill(vg);
        nvgStrokeWidth(vg, 0.1);
        nvgStrokeColor(vg, nvgRGBA(20, 140, 130, 80));
        draw_bounding_box(vg, active_points_bounding_box);
        nvgStrokeWidth(vg, 0.5);
        nvgStrokeColor(vg, nvgRGBA(150, 140, 30, 80));
        draw_bounding_box(vg, camera_bb);
        draw_bounding_box(vg, eviction_bb);
        nvgStrokeWidth(vg, 0.1);
        for_each(begin(vine.history_storage), vine.last_history_pos,
                 [](vine_stem_history_header history) {
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
}
