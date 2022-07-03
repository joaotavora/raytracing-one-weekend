#include <cmath>
#include <iostream>
#include <glm/glm.hpp>
#include <functional>
#include <limits>
#include <optional>

namespace rtweekend {
  using vec3 = glm::dvec3;
  using color = vec3;
  using point = glm::dvec3;

  void write_color(std::ostream &out, color pixel_color) {
    // Write the translated [0,255] value of each color component.
    out << static_cast<int>(255.999 * pixel_color.x) << ' '
        << static_cast<int>(255.999 * pixel_color.y) << ' '
        << static_cast<int>(255.999 * pixel_color.z) << '\n';
  }

  std::ostream& operator<<(std::ostream& o, const vec3& x) {
    return o << "{" << x.x << "," << x.y << "," << x.z << "}";
  }

  struct ray {
    vec3 origin{};
    vec3 direction{};
    vec3 at(double t) const {return origin + direction * t;}
  };

  struct hittable {
    struct hit_record {
      point p;
      vec3 normal;
      double t;
    };
    [[nodiscard]] virtual std::optional<hit_record>
    hit(const ray& r, double tmin, double tmax) const = 0;
  };

  struct sphere : public hittable {
    point center;
    double radius;
    sphere(point c, double r) : center{c}, radius{r} {};

    [[nodiscard]] std::optional<hit_record>
    hit(const ray& r, double tmin, double tmax) const override {
      vec3 oc = r.origin - center;                      // (A-C)
      double a = glm::dot(r.direction, r.direction);    // (b.b)
      auto h = glm::dot(oc, r.direction);               // (A-C).b
      auto c = glm::dot(oc, oc) - radius*radius;        // (A-C)(A-C) - r^2
      auto discriminant = h*h - a*c;
      if (discriminant < 0.0) return {};

      auto root = (-h - std::sqrt(discriminant))/a;
      if (root < tmin || root > tmax) {
        
        root = (-h + std::sqrt(discriminant))/a;
        if (root < tmin || root > tmax) return {};
      }

      auto hitpoint = r.at(root);
      auto normal = glm::normalize(hitpoint - center);

      // normal = glm::faceforward(normal, r.direction, normal);
      normal = glm::dot(r.direction, normal) < 0?normal:-normal;
      return hit_record{hitpoint,
                        normal,
                        root};

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

    auto probe = s.hit(r, 0, std::numeric_limits<double>::infinity());
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
  constexpr int image_width = 800;
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
