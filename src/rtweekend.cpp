#include <cmath>
#include <iostream>
#include <glm/glm.hpp>
#include <functional>



namespace rtweekend {
  // could be "color.h"
  using vec3 = glm::dvec3;
  using color = vec3;
  using point = vec3;

  void write_color(std::ostream &out, color pixel_color) {


    // Write the translated [0,255] value of each color component.
    out << static_cast<int>(255.999 * pixel_color.x) << ' '
        << static_cast<int>(255.999 * pixel_color.y) << ' '
        << static_cast<int>(255.999 * pixel_color.z) << '\n';
  }

  // could be "ray.h"
  struct ray {
    vec3 origin{};
    vec3 direction{};
    vec3 at(double t) const {return origin + direction * t;}
  };

  /** Tell if ray R hits sphere at CENTER with radius RADIUS */
  bool hit_sphere(const point& center, double radius, const ray& r) {
    //  x = -b +- sqrt(b^2 - 4ac)/2a
    vec3 oc = r.origin - center;                      // (A-C)
    auto a = glm::dot(r.direction, r.direction);      // (b.b)
    auto b = 2.0 * glm::dot(oc, r.direction);         // 2b(A-C)
    auto c = glm::dot(oc, oc) - radius*radius;        // (A-C)(A-C) - r^2
    auto discriminant = b*b - 4*a*c;
    return (discriminant > 0);
  }

  /** Say color of ray R */
  color ray_color(const ray& r) {
    if (hit_sphere(vec3{0,0,-1}, 0.3, r)) return color{1.0, 0, 0};
    // I then did a standard graphics trick of scaling that to
    // 0.0≤t≤1.0. When t=1.0 I want blue. When t=0.0 I want
    // white. In between, I want a blend. This forms a “linear
    // blend”, or “linear interpolation”, or “lerp” for short,
    // between two things. A lerp is always of the form
    //
    // blendedValue=(1−t)⋅startValue+t⋅endValue,
    //
    // with t going from zero to one.
      vec3 unit_direction = glm::normalize(r.direction);
    auto ycomp = unit_direction.y;
    // t is between 0 and 1, proportional to y
    auto t = 0.5*(ycomp + 1.0);
    return
      // lower t, whiter image, higher t, bluer image centered 
      (1.0-t)*color{1.0, 1.0, 1.0} + t*color{0.5, 0.7, 1.0};
  }
}  // namespace rtweekend


int main() {

  namespace rt=rtweekend;
  // Image

  constexpr auto aspect_ratio = 16.0/9.0;
  constexpr int image_width = 400;
  constexpr int image_height = static_cast<int>(image_width/aspect_ratio);


  auto viewport_height = 2.0;
  auto viewport_width = aspect_ratio * viewport_height;
  auto focal_length = 1.0;

  auto origin = rt::vec3{0,0,0};
  auto horizontal = rt::vec3{viewport_width, 0, 0};
  auto vertical = rt::vec3{0, viewport_height, 0};
  auto lower_left_corner =
    origin - horizontal/2.0 - vertical/2.0 - rt::vec3{0, 0, focal_length};

  // Render

  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

  for (int j = image_height-1; j >= 0; --j) {
    std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
    for (int i = 0; i < image_width; ++i) {
      auto u = double(i) / (image_width-1);   // sweep [0 -> 1]
      auto v = double(j) / (image_height-1);  // sweep [1 -> 0]
      rt::ray r{
        .origin   = origin,
        .direction= (lower_left_corner + u*horizontal + v*vertical) - origin
      };
      rt::color pixel_color = rt::ray_color(r);

      rt::write_color(std::cout, pixel_color);
    }
  }

  std::cerr << "\nDone.\n";
}
