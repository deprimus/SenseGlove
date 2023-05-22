// UW's trivial type definitions
#ifndef UWTYPES_H_GUARD
#define UWTYPES_H_GUARD

#include <cstdint>

typedef int8_t   int8;
typedef int16_t  int16;
typedef int32_t  int32;
typedef int64_t  int64;

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

#define UW_PI 3.14159265358979323846
#define DEG2RAD (UW_PI / 180)

inline float map(float x, float start1, float end1, float start2, float end2) {
  return ((x - start1) / (end1 - start1)) * (end2 - start2) + start2;
}

#endif //  UWTYPES_H_GUARD
