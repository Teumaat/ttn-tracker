#include "stdio.h"
#include "stdint.h"

int32_t lat = 255;
int8_t output[4];

int main () {
  int8_t *cur_lat = &lat;
  output[0] = cur_lat[0];
  output[1] = cur_lat[1];
  output[2] = cur_lat[2];
  output[3] = cur_lat[3];

  printf( "%02x%02x%02x%02x", output[0], output[1], output[2], output[3] );
  return 0;
}
