#pragma once

#include <algorithm>
#include <random>

#include "vec3.h"

namespace rtweekend::detail {
  double random_double(double a = 0, double b = 1.0);

  int random_int(int a = 0 , int b = 1);

  color random_vec3(double min = 0, double max = 1.0);

  vec3 random_in_unit_sphere();

  vec3 random_in_unit_disk();

  vec3 random_unit_vector();
}  // namespace rtweekend::detail

namespace rtweekend {
  using detail::random_double;
  using detail::random_int;
  using detail::random_vec3;
}

// Local Variables:
// mode: c++
// End:
