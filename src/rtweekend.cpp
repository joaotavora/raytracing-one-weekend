#include "glm/gtx/quaternion.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/exponential.hpp>
#include <glm/geometric.hpp>
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

  struct Ray {
    vec3 origin{};
    vec3 direction{};
    vec3 at(double t) const {return origin + direction * t;}
  };

  struct HitRecord;

  struct ScatterRecord {
    Ray r;
    color attenuation;
  };

  struct Material { // NOLINT
    [[nodiscard]] virtual std::optional<ScatterRecord>
    scatter(const Ray& r_in, const HitRecord& rec) const = 0;

    virtual ~Material() = default;
  };

  struct Hittable { // NOLINT
    [[nodiscard]] virtual std::optional<HitRecord>
    hit(const Ray& r, double tmin, double tmax) const = 0;

    explicit Hittable(const Material& m) : material_{&m} {};
    virtual ~Hittable() = default;
    const Material& material() const {return *material_;}
  private:
    const Material* material_;
  };

  struct HitRecord {
    point p;
    vec3 normal;
    double t;
    const Hittable* hittable;
    bool front_facing;
  };

  struct Lambertian : public Material {
    explicit Lambertian(const color& albedo) : albedo{albedo} {}

    std::optional<ScatterRecord>
    scatter(const Ray&, const HitRecord& rec) const override {
      auto rand = random_unit_vector();
      if (glm::all(glm::epsilonEqual(rec.normal, rand, 1e-8))) {
        return {};
      }
      auto scatter_direction = rec.normal + rand;
      return ScatterRecord{.r = Ray{rec.p, scatter_direction},
                           .attenuation = albedo};
    }
    color albedo;
  };

  struct Metal : public Material {
    explicit Metal(const color& albedo, double fuzz=0)
      : albedo{albedo}, fuzz{std::clamp(fuzz, 0.0, 1.0)} {}

    std::optional<ScatterRecord>
    scatter(const Ray& r, const HitRecord& rec) const override {
      auto reflected = glm::reflect(r.direction, rec.normal);

      return ScatterRecord{.r = Ray{rec.p, reflected + fuzz*random_unit_vector()},
                           .attenuation = albedo};
    }

    color albedo;
    double fuzz;
  };

  struct Dielectric : public Material {
    explicit Dielectric(double index_of_refraction, double fuzz=0)
      : ir{index_of_refraction}, fuzz{std::clamp(fuzz, 0.0, 1.0)} {}

    std::optional<ScatterRecord>
    scatter(const Ray& r, const HitRecord& rec) const override {

      auto unit_direction = glm::normalize(r.direction);

      double cos_theta = dot(-unit_direction, rec.normal);
      double sin_theta = sqrt(1.0 - cos_theta*cos_theta);

      double refraction_ratio = rec.front_facing ? (1.0/ir) : ir;

      bool cannot_refract = refraction_ratio * sin_theta > 1.0;
      vec3 direction;

      if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_double())
        direction = glm::reflect(unit_direction, rec.normal);
      else
        direction = glm::refract(unit_direction, rec.normal, refraction_ratio);


      return ScatterRecord{.r = Ray{rec.p,
                                    direction
                                    + fuzz*random_unit_vector()},
                           .attenuation = {1.0, 1.0, 1.0}};
    }
    double ir;
    double fuzz;
  private:
    static double reflectance(double cosine, double ref_idx) {
      // Use Schlick's approximation for reflectance.
      auto r0 = (1-ref_idx) / (1+ref_idx);
      r0 = r0*r0;
      return r0 + (1-r0)*pow((1 - cosine),5);
    }
  };

  struct Sphere : public Hittable {
    point center;
    double radius;
    Sphere(point c, double r, const Material& m) : Hittable{m}, center{c}, radius{r} {};

    [[nodiscard]] std::optional<HitRecord>
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

      bool front_facing = (glm::dot(r.direction, normal) < 0) ^ (radius < 0);
      normal = front_facing?normal:-normal;
      return HitRecord{hitpoint, normal, root, this, front_facing};
    }
  };

  class Camera {
  public:
    static constexpr auto focal_length = 1.0;
    Camera(point  lookfrom,
           point  lookat,
           vec3   vup,
           double fov,
           double aspect_ratio,
           double aperture,
           std::optional<double> focus_dist = std::nullopt) :
      origin_{lookfrom},
      w_{glm::normalize(lookfrom - lookat)},
      u_{glm::normalize(glm::cross(vup, w_))},
      v_{glm::normalize(glm::cross(w_, u_))},
      lens_radius_{aperture/2}

    {

      auto viewport_height = 2.0 * std::tan(fov * std::numbers::pi / 180 / 2);
      auto viewport_width = aspect_ratio * viewport_height;

      auto fd = focus_dist?focus_dist.value():glm::length(lookfrom - lookat);

      horizontal_ = fd * viewport_width * u_;
      vertical_   = fd * viewport_height * v_;
      lower_left_corner_ = origin_
        - horizontal_/2.0
        - vertical_/2.0
        - fd * w_;
    }

    Ray get_ray(double s, double t) const {
      vec3 rd = lens_radius_ * random_in_unit_disk();
      vec3 offset = u_ * rd.x + v_ * rd.y;
      return Ray{origin_ + offset, lower_left_corner_ + s*horizontal_ + t*vertical_ - origin_ - offset};
    }

  private:

    point origin_;

    vec3 w_, u_, v_;

    vec3 horizontal_{};
    vec3 vertical_{};
    point lower_left_corner_{};
    double lens_radius_;
  };

  using world_t = std::vector<std::unique_ptr<Hittable>>;

  color ray_color(const Ray& r, const world_t& world, size_t max_depth=20) {
    std::optional<HitRecord> closest{};
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

      auto probe = closest->hittable->material().scatter(r, closest.value());
      if (probe)
        return probe->attenuation * ray_color(probe->r, world, max_depth-1);
      return {0,0,0};
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
  using detail::Material;
  using detail::Lambertian;
  using detail::Metal;
  using detail::Dielectric;
  using detail::point;
  using detail::vec3;
  using detail::color;
  using detail::random_double;
  using detail::ray_color;
  using detail::write_color;
  using detail::random_vec3;
  using detail::Camera;
}  // namespace rtweekend

int main() {
  namespace rt=rtweekend;
  constexpr double aspect_ratio = 3.0/2.0;

  // Camera
  rt::Camera cam{rt::point(13,2,3),
                 rt::point(0,0,0),
                 rt::vec3(0,1,0),
                 20.0,
                 aspect_ratio,
                 0.1,
                 10.0
                 };

  // Image
  constexpr int image_width = 1200;
  constexpr int image_height = static_cast<int>(image_width/aspect_ratio); // NOLINT
  constexpr int samples_per_pixel = 100;
  constexpr int max_child_rays = 50;

  // World of spheres
  rt::world_t world{};

    // Materials
  rt::Lambertian ground_material{rt::color{0.5, 0.5, 0.5}};
  world.push_back(std::make_unique<rt::Sphere>(rt::point{0,-1000,0}, 1000, ground_material));

  std::vector<std::unique_ptr<rt::Material>> materials{};


  for (int a = -11; a < 11; a++) {
    for (int b = -11; b < 11; b++) {
      auto choose_mat = rt::random_double();
      rt::point center(a+ 0.9*rt::random_double(), 0.2, b + 0.9*rt::random_double());

      if (glm::length(center - rt::point{4,0.2,0}) > 0.9) {
        if (choose_mat < 0.8) {
          // diffuse
          auto albedo = rt::random_vec3() * rt::random_vec3();
          materials.push_back(std::make_unique<rt::Lambertian>(albedo));
          world.push_back(std::make_unique<rt::Sphere>(center, 0.2, *materials.back()));
        } else if (choose_mat < 0.95) {
          // metal
          auto albedo = rt::random_vec3(0.5, 1);
          auto fuzz = rt::random_double(0, 0.5);
          materials.push_back(std::make_unique<rt::Metal>(albedo, fuzz));
          world.push_back(std::make_unique<rt::Sphere>(center, 0.2, *materials.back()));
        } else {
          // glass
          materials.push_back(std::make_unique<rt::Dielectric>(1.5));
          world.push_back(std::make_unique<rt::Sphere>(center, 0.2, *materials.back()));
        }
      }
    }
  }

  rt::Dielectric glass{1.5};
  rt::Lambertian reddish{rt::color{0.4, 0.2, 0.1}};
  rt::Metal reddish_metal{rt::color{0.7, 0.6, 0.5}};

  world.push_back(std::make_unique<rt::Sphere>(rt::point(0, 1, 0), 1.0, glass));
  world.push_back(std::make_unique<rt::Sphere>(rt::point(-4, 1, 0), 1.0, reddish));
  world.push_back(std::make_unique<rt::Sphere>(rt::point(4, 1, 0), 1.0, reddish_metal));


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

  std::cerr << "\nDone (also " << sizeof(rt::detail::HitRecord) << ").\n";
}
