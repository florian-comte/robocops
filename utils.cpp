#include "utils.h"
/**
 * @brief Maps a floating-point value from one range to another.
 *
 * Similar to Arduino's `map()` but supports float precision.
 *
 * @param x The input value to map.
 * @param in_min The lower bound of the input range.
 * @param in_max The upper bound of the input range.
 * @param out_min The lower bound of the output range.
 * @param out_max The upper bound of the output range.
 * @return Mapped float value.
 */
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
