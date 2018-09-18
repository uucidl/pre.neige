
// quick hack job by @mmalex
// thanks to libhid author and @johndrinkwater and
// https://github.com/chrippa/ds4drv
#include "hidapi/hidapi/hidapi.h" // using http://www.signal11.us/oss/hidapi/
#include <stdio.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

#pragma pack(push, 1)
struct DS4Touch {
  u32 id : 7;
  u32 inactive : 1;
  u32 x : 12;
  u32 y : 12;
};
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
struct
  DS4Out { // this is only correct for wired connection. see
           // https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py for
           // example for wireless
  u32 magic; // 0x0004ff05
  u8 rumbler, rumblel, r, g, b, flashon, flashoff;
  u8 pad[32 - 11];
};
#pragma pack(pop)

int main()
{
  int res;
  hid_device *handle;
  if (!(handle = hid_open(0x54c, 0x5c4, NULL))) {
    fprintf(stderr, "unable to open device\n");
    return 1;
  }
  DS4 ds4;
  DS4Out ds4_out = {0x0004ff05, 0, 0, 255, 0, 185, 0, 0, {}}; // pink LED please
  res = hid_write(handle, (u8 *)&ds4_out, sizeof(ds4_out));
  bool must_run = true;
  while (must_run) {
    res = hid_read(handle, (u8 *)&ds4, 64);
    if (res == 64) {
      printf("%04x %01x %3d %3d %5d %d-%d %d,%d %d,%d\n", ds4.buttons,
             ds4.trackpad_ps, ds4.l2, ds4.r2, ds4.accel[0],
             ds4.touch[0].inactive, ds4.touch[0].id, ds4.touch[0].x,
             ds4.touch[0].y, ds4.leftx, ds4.lefty);
      printf("gyro: %d %d %d\n", ds4.gyro[0], ds4.gyro[1], ds4.gyro[2]);
      ds4_out.r = ds4.rightx;
      ds4_out.g = ds4.righty;
      hid_write(handle, (u8 *)&ds4_out, sizeof(ds4_out));
      if (ds4.buttons & 1 << 13) {
        printf("pressed Options\n");
        must_run = false;
      }
    }
  }
  hid_close(handle);
  hid_exit();
  return 0;
}
