#define UU_ARITHMETICS
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
template <typename T> T interpolate_linear(T a, T b, double a_b)
{
  return product(a, 1.0 - a_b) + product(b, a_b);
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
