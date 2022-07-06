#pragma once

#include "model.h"

namespace rtweekend {
  struct Config {
    double aspect_ratio = 3.0/2.0;
    int image_width = 1200;
    int samples_per_pixel = 100;
    int max_child_rays = 50;
    int nthreads = 4;
  };
  void render(const World &world, const Camera &cam, const Config &cfg);
} // namespace rtweekend

// Local Variables:
// mode: c++
// End:
