#include <glm/gtx/norm.hpp>
#include "random_utils.h"


namespace rtweekend::detail {

  std::mt19937& gen(){
    static std::mt19937 the_generator;
    return the_generator;
  }

  double random_double(double a, double b) {
    return std::uniform_real_distribution{a, b}(gen());
  }

  int random_int(int a, int b) {
    return std::uniform_int_distribution{a, b}(gen());
  }

  color random_vec3(double min, double max) {
    return vec3{random_double(min, max), random_double(min, max),
                random_double(min, max)};
  }
  vec3 random_in_unit_sphere() {
    while (true) {
      auto v = random_vec3();
      if (glm::length2(v) >= 1)
        continue;
      return v;
    }
  }
  vec3 random_unit_vector() {
    return random_in_unit_sphere();
  }
  vec3 random_in_unit_disk() {
    while (true) {
      auto p = vec3(random_double(-1, 1), random_double(-1, 1), 0);
      if (glm::length2(p) >= 1)
        continue;
      return p;
    }
  }
} // namespace rtweekend::detail
