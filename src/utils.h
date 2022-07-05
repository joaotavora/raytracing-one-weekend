#pragma once

#include <algorithm>
#include <iostream>
#include <random>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>


namespace rtweekend::detail {
  using vec3 = glm::dvec3;
  using color = glm::dvec3;
  using point = glm::dvec3;

  inline double random_double(double a=0, double b=1.0) {
    static std::uniform_real_distribution<double> distribution(a, b);
    static std::mt19937 generator;
    return distribution(generator);
  }

  color random_vec3(double min=0, double max=1.0) {
    return vec3{random_double(min, max),
                random_double(min, max),
                random_double(min, max)};
  }

  vec3 random_in_unit_sphere() {
    while (true) {
      auto v = random_vec3();
      if (glm::length2(v) >= 1) continue;
      return v;
    }
  }

  vec3 random_in_unit_disk() {
    while (true) {
      auto p = vec3(random_double(-1,1), random_double(-1,1), 0);
      if (glm::length2(p) >= 1) continue;
      return p;
    }
  }

  vec3 random_unit_vector() {
    return random_in_unit_sphere();
  }

  void write_color(std::ostream &out, color c, int samples_per_pixel) {
    // Divide the color by the number of samples and correct with gamma=2

    c = glm::sqrt(c / static_cast<double>(samples_per_pixel));

    // Write the translated [0,255] value of each color component.
    out << static_cast<int>(256 * std::clamp(c.r, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * std::clamp(c.g, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * std::clamp(c.b, 0.0, 0.999)) << '\n';
  }

  std::ostream& operator<<(std::ostream& o, const vec3& x) {
    return o << "{" << x.x << "," << x.y << "," << x.z << "}";
  }
}  // namespace rtweekend::detail

// Local Variables:
// mode: c++
// End:
