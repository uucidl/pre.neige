// (Vector Math)
#define UU_VEC3
#ifndef UU_VEC3_API
#define UU_VEC3_API static
#endif
#ifndef UU_FLOATS
#include "floats.cpp"
#endif
namespace uu
{
struct vec3 /* @models{Regular} */ {
  float x, y, z;
};
UU_VEC3_API vec3 make_vec3(float v) { return {v, v, v}; }
UU_VEC3_API vec3 make_vec3(float x, float y) { return {x, y, 0.0}; }
UU_VEC3_API vec3 make_vec3(float x, float y, float z) { return {x, y, z}; }
UU_VEC3_API void assign(vec3 *dest_, vec3 const x)
{
  auto &dest = *dest_;
  dest = x;
}
UU_VEC3_API bool valid(vec3 x)
{
  return valid(x.x) && valid(x.y) && valid(x.z);
}
UU_VEC3_API bool equality(vec3 a, vec3 b)
{
  return a.x == b.x && a.y == b.y && a.z == b.z;
};
UU_VEC3_API bool default_order(vec3 a, vec3 b)
{
  return a.x < b.x ? true : a.y < b.y ? true : a.z < b.z;
}
UU_VEC3_API vec3 addition(vec3 a, vec3 b)
{
  vec3 result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}
UU_VEC3_API vec3 product(vec3 vector, float scalar)
{
  return vec3{vector.x * scalar, vector.y * scalar, vector.z * scalar};
}
UU_VEC3_API vec3 hadamard_product(vec3 a, vec3 b)
{
  return vec3{a.x * b.x, a.y * b.y, a.z * b.z};
}
UU_VEC3_API vec3 cross_product(vec3 a, vec3 b)
{
  return vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.z};
}
UU_VEC3_API float squared_euclidean_norm(vec3 v)
{
  float xx = v.x * v.x;
  float yy = v.y * v.y;
  float zz = v.z * v.z;
  return xx + yy + zz;
}
UU_VEC3_API float squared_euclidean_distance(vec3 a, vec3 b)
{
  return squared_euclidean_norm(addition(b, product(a, -1)));
}
} // namespace uu
