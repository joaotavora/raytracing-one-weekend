#pragma once
#include <optional>
#include <string>

#include "primitive-model.h"

namespace rtweekend::detail {

  class BVHNode;

  struct Config {
    int number_of_balls_sqrt = 11;
    double aspect_ratio = 3.0/2.0;
    int image_width = 200;
    int samples_per_pixel = 20;
    bool moving_spheres = true;
    int max_child_rays = 20;
    int nthreads = 4;
    std::optional<std::string> model = {};
  };

  class Scene  {
    mutable PrimitiveStore_t primitives_;
    MaterialStore_t boutique_;
    Camera cam_;
  public:
    template <typename T>
    explicit Scene(T&& camera) : cam_{std::forward<T>(camera)} {}
    auto& camera() const {return cam_;}
    BVHNode get_root_bvh() const;
    auto& primitives() {return primitives_;}
    auto& boutique() {return boutique_;}
  };

  void render(const Scene& world, const Config &cfg);

  std::ostream& operator<<(std::ostream& o, const Config& c);
} // namespace rtweekend::detail

namespace rtweekend {
  using detail::Scene;
  using detail::render;
  using detail::Config;
}
// Local Variables:
// mode: c++
// End:
