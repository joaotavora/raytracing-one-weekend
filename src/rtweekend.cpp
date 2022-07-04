#include "glm/exponential.hpp"
#include "glm/geometric.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <functional>
#include <limits>
#include <optional>
#include <memory>
#include <random>

namespace rtweekend::detail {
  using vec3 = glm::dvec3;
  using color = glm::dvec3;
  using point = glm::dvec3;

  inline double random_double(double a=0, double b=1.0) {
    static std::uniform_real_distribution<double> distribution(a, b);
    static std::mt19937 generator;
    return distribution(generator);
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

  struct Ray {
    vec3 origin{};
    vec3 direction{};
    vec3 at(double t) const {return origin + direction * t;}
  };

  struct Hittable {
    struct hit_record {
      point p;
      vec3 normal;
      double t;
    };
    [[nodiscard]] virtual std::optional<hit_record>
    hit(const Ray& r, double tmin, double tmax) const = 0;

    Hittable() = default;
    Hittable(const Hittable&) = default;
    Hittable(Hittable&&) = default;
    Hittable& operator=(const Hittable&) = default;
    Hittable& operator=(Hittable&&) = default;
    virtual ~Hittable() = default;
  };

  struct Sphere : public Hittable {
    point center;
    double radius;
    Sphere(point c, double r) : center{c}, radius{r} {};

    [[nodiscard]] std::optional<hit_record>
    hit(const Ray& r, double tmin, double tmax) const override {
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

  class Camera {
  public:
    static constexpr auto aspect_ratio = 16.0 / 9.0;
    static constexpr auto viewport_height = 2.0;
    static constexpr auto viewport_width = aspect_ratio * viewport_height;
    static constexpr auto focal_length = 1.0;
    Camera() : origin_{point{0,0,0}},
               horizontal_{vec3{viewport_width, 0, 0}},
               vertical_{vec3{0, viewport_height, 0}},
               lower_left_corner_{origin_
                                  - horizontal_/2.0
                                  - vertical_/2.0
                                  - vec3(0,0,focal_length)}
               
    {}

    Ray get_ray(double u, double v) const {
      return Ray{origin_, lower_left_corner_ + u*horizontal_ + v*vertical_ - origin_};
    }

  private:
    point origin_;
    vec3 horizontal_;
    vec3 vertical_;
    point lower_left_corner_;
  };

  using world_t = std::vector<std::unique_ptr<Hittable>>;

  vec3 random_in_unit_sphere() {
    while (true) {
      vec3 v{random_double(), random_double(), random_double()};
      if (glm::length2(v) >= 1) continue;
      return v;
    }
  }
  
  color ray_color(const Ray& r, const world_t& world, size_t max_depth=20) {
    std::optional<Hittable::hit_record> closest{};
    double tmax = std::numeric_limits<double>::infinity();
    for (const auto& h : world) {
      auto probe = h->hit(r, 0.001, tmax);
      if (probe) {
        closest = probe;
        tmax = probe->t;
      }
    }
    if (closest) {
      if (max_depth <= 0) return {0,0,0};
      point target = closest->p + closest->normal + random_in_unit_sphere();
      return 0.5 * ray_color(Ray{closest->p, target - closest->p},
                             world, 
                             max_depth-1);
    }

    // Fallback to background
    // t is between 0 and 1, proportional to y
    // lower t, whiter image, higher t, bluer image centered
    vec3 unit_direction = glm::normalize(r.direction);
    auto ycomp = unit_direction.y;
    auto t =  0.5*(ycomp + +1.0);
    return (1.0-t)*color{1.0, 1.0, 1.0} + t*color{0.5, 0.7, 1.0};
  }
}  // namespace rtweekend::detail

namespace rtweekend {
  using detail::world_t;
  using detail::Sphere;
  using detail::point;
  using detail::color;
  using detail::random_double;
  using detail::ray_color;
  using detail::write_color;
  using detail::Camera;
}

int main() {
  namespace rt=rtweekend;

  // Camera
  rt::Camera cam{};
  
  // Image
  constexpr int image_width = 300;
  constexpr int image_height = static_cast<int>(image_width/cam.aspect_ratio); // NOLINT
  constexpr int samples_per_pixel = 30;
  constexpr int max_child_rays = 10;

  // World of spheres
  rt::world_t world{};
  world.reserve(10);
  world.push_back(std::make_unique<rt::Sphere>(rt::point{0,0,-1}, 0.5));
  world.push_back(std::make_unique<rt::Sphere>(rt::point{0,-100.5,-1}, 100));

  // Render

  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

  for (int j = image_height-1; j >= 0; --j) {
    std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
    for (int i = 0; i < image_width; ++i) {
      rt::color pixel_color{0,0,0};
      for (int s = 0; s < samples_per_pixel; ++s) {
        auto u = (i + rt::random_double()) / (image_width-1);   // sweep [0 -> 1]
        auto v = (j + rt::random_double()) / (image_height-1);  // sweep [1 -> 0]
        auto r = cam.get_ray(u, v);
        pixel_color += rt::ray_color(r, world, max_child_rays);
      }
      rt::write_color(std::cout, pixel_color, samples_per_pixel);

      
    }
  }

  std::cerr << "\nDone.\n";
}
