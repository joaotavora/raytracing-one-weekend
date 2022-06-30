#include <iostream>
#include <glm/glm.hpp>

namespace rtweekend {
  // could be "color.h"
  using color = glm::vec3;
  using point = glm::dvec3;

  void write_color(std::ostream &out, color pixel_color) {
    // Write the translated [0,255] value of each color component.
    out << static_cast<int>(255.999 * pixel_color.x) << ' '
        << static_cast<int>(255.999 * pixel_color.y) << ' '
        << static_cast<int>(255.999 * pixel_color.z) << '\n';
  }

  // could be "ray.h"
  struct ray {
    glm::dvec3 origin{};
    glm::dvec3 direction{};
    glm::dvec3 at(double t) const {return origin + direction * t;}
  };
}  // namespace rtweekend


int main() {

  namespace rt=rtweekend;
  // Image

  const int image_width = 256;
  const int image_height = 256;

  rt::ray r{
    .origin    = {1,2,3},
    .direction = {12,3,4},
  };

  // Render

  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

  for (int j = image_height-1; j >= 0; --j) {
    std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
    for (int i = 0; i < image_width; ++i) {
      rt::color pixel_color{double(i)/(image_width-1),
                            double(j)/(image_height-1),
                            0.25};
      rt::write_color(std::cout, pixel_color);
    }
  }

  std::cerr << "\nDone.\n";
}
