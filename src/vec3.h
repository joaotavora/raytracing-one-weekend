#pragma once

#include <glm/glm.hpp>

namespace rtweekend::detail {
  using vec3 = glm::dvec3;
  using color = glm::dvec3;
  using point = glm::dvec3;
}

namespace rtweekend {
  using detail::point;
  using detail::vec3;
  using detail::color;
}

// Local Variables:
// mode: c++
// End:
