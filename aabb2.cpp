// (Axis Aligned Bounding Box)
#define UU_AABB2
#ifndef UU_AABB2_API
#define UU_AABB2_API static
#endif
#ifndef UU_CONCEPTS
#include "concepts.hpp"
#endif
#ifndef UU_MACHINE_TYPES
#include "machine_types.hpp"
#endif
#ifndef UU_NUMBERS
#include "numbers.hpp"
#endif
#ifndef UU_FLOATS
#include "floats.cpp"
#endif
#ifndef MODELS
#define MODELS(...)
#endif
namespace uu
{
// (Bounding Box)
struct aabb2 MODELS(SemiRegular) {
  float32 min_x;
  float32 max_x;
  float32 min_y;
  float32 max_y;
};
UU_AABB2_API aabb2 zero_aabb2()
{
  aabb2 result;
  result.min_x = result.min_y = Number<float32>::max;
  result.max_x = result.max_y = Number<float32>::min;
  return result;
}
UU_AABB2_API aabb2 cover(aabb2 bounding_box, float32 point_x, float32 point_y)
{
  bounding_box.min_x =
    point_x < bounding_box.min_x ? point_x : bounding_box.min_x;
  bounding_box.max_x =
    bounding_box.max_x < point_x ? point_x : bounding_box.max_x;
  bounding_box.min_y =
    point_y < bounding_box.min_y ? point_y : bounding_box.min_y;
  bounding_box.max_y =
    bounding_box.max_y < point_y ? point_y : bounding_box.max_y;
  return bounding_box;
}
UU_AABB2_API bool intersects(aabb2 a, aabb2 b)
{
  if (a.max_x < b.min_x || a.min_x > b.max_x) return false;
  if (a.max_y < b.min_y || a.min_y > b.max_y) return false;
  return true;
}
UU_AABB2_API aabb2 translate(aabb2 bounding_box, float32 delta_x,
                             float32 delta_y)
{
  bounding_box.min_x += delta_x;
  bounding_box.max_x += delta_x;
  bounding_box.min_y += delta_y;
  bounding_box.max_y += delta_y;
  return bounding_box;
}
UU_AABB2_API aabb2 scale_around_center(aabb2 bounding_box, float32 scaling)
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
UU_AABB2_API aabb2 make_symmetric(aabb2 bb)
{
  aabb2 result;
  auto const amx = absolute_value(bb.min_x);
  auto const aMx = absolute_value(bb.max_x);
  auto const amy = absolute_value(bb.min_y);
  auto const aMy = absolute_value(bb.max_y);
  auto x = amx > aMx ? amx : aMx;
  auto y = amy > aMy ? amy : aMy;
  result.min_x = -x;
  result.max_x = x;
  result.min_y = -y;
  result.max_y = y;
  return result;
}
UU_AABB2_API bool contains(aabb2 bb, float32 x, float32 y)
{
  if (x < bb.min_x || x > bb.max_x) return false;
  if (y < bb.min_y || y > bb.max_y) return false;
  return true;
}
} // namespace uu
