#pragma once

#include "oomodel.h"
#include <iosfwd>

namespace rtweekend::detail {

  class BVHNode;

  struct Config {
    int number_of_balls_sqrt = 11;
    double aspect_ratio = 3.0/2.0;
    int image_width = 1200;
    int samples_per_pixel = 100;
    bool moving_spheres = true;
    int max_child_rays = 50;
    int nthreads = 4;
  };

  class World  {
    mutable PrimitiveStore_t primitives_;
    MaterialStore_t boutique_;
  public:
    BVHNode get_root_bvh() const;
    auto& primitives() {return primitives_;}
    auto& boutique() {return boutique_;}
  };
  using PView_t = std::span<std::unique_ptr<Primitive>>;

  void render(const World& world, const Camera &cam, const Config &cfg);

  std::ostream& operator<<(std::ostream& o, const Config& c);
} // namespace rtweekend::detail

namespace rtweekend {
  using detail::World;
  using detail::render;
  using detail::Config;
}
// Local Variables:
// mode: c++
// End:
