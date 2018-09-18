#define UU_HEXAGON_TILING
#if defined(UU_HEXAGON_TILING_MAIN)
#include <complex>
#include <stdio.h>
static void hexagon_print(float radius, float center_x, float center_y)
{
  float const cx = center_x;
  float const cy = center_y;
  float const u = radius;
  auto const pi = std::acos(-1);
  auto const rota = pi/3.0;
  std::complex<float> const sv (cos(rota), sin(rota));
  printf("sv: %f %f\n", real(sv), imag(sv));
  std::complex<float> v = u;
  printf("v: %f %f\n", cx, cy);
  for (int n = 6; n--; ) {
    printf("v: %f %f\n", cx + real(v), cy + imag(v));
    v *= sv;
  }
}  
int main(int argc, char **argv)
{
  float y = 0.0;
  for(auto hexagon_col_n=2; hexagon_col_n--; ) {
    float x=0;
    for(auto hexagon_row_n=10; hexagon_row_n--; ) {
      hexagon_print(1, x, y);
      x+=1.50;
      hexagon_print(1, x, y + std::sin(std::acos(-1)/3.0));
      x+=1.50;
    }
    y+=3;
  }
  return 0;
}
#endif
