#include <cmath>
#include <iostream>
#include <glm/glm.hpp>
#include <functional>
#include <optional>

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


  struct hittable {
    struct hit_record {point p; vec3 normal; double t;};
    [[nodiscard]] virtual std::optional<hit_record>
    hit(double tmin, double tmax, const ray& r) const = 0;
  };

  struct sphere : public hittable {
    point center;
    double radius;
    sphere(point c, double r) : center{c}, radius{r} {};

    [[nodiscard]] std::optional<hit_record>
    hit(double tmin, double tmax, const ray& r) const override {
      vec3 oc = r.origin - center;                      // (A-C)
      auto a = glm::dot(r.direction, r.direction);      // (b.b)
      auto half_b = glm::dot(oc, r.direction);
      auto c = glm::dot(oc, oc) - radius*radius;        // (A-C)(A-C) - r^2
      auto discriminant = half_b*half_b - a*c;
      if (discriminant < 0) return {};

      auto root = (-half_b - std::sqrt(discriminant));
      if (root < tmin || root > tmax) {
        root = (-half_b + std::sqrt(discriminant));
        if (root < tmin || root > tmax) return {};
      }

      auto hitpoint = r.at(root);
      return hit_record{hitpoint, glm::normalize(hitpoint - center), root};

    }
  };

  /** Compute T of Ray R where it hits infinite slanted wall Z units away */
  double hit_background(double slant, const ray& r) {
    vec3 unit_direction = glm::normalize(r.direction);
    auto ycomp = unit_direction.y;
    // t is between 0 and 1, proportional to y
    return 0.5*(ycomp + slant);
  }

  /** Say color of ray R */
  color ray_color(const ray& r) {
    sphere s{point{0,0,-1}, 0.5};

    auto probe = s.hit(0, 100, r);
    if (probe) {
      return 0.5 * (probe->normal + vec3{1, 1, 1});
    }

    auto t = hit_background(+1.0, r);
    return
      // lower t, whiter image, higher t, bluer image centered
      (1.0-t)*color{1.0, 1.0, 1.0} + t*color{0.5, 0.7, 1.0};
  }
}  // namespace rtweekend


int main() {

  namespace rt=rtweekend;
  // Image

  constexpr auto aspect_ratio = 16.0/9.0;
  constexpr int image_width = 600;
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
