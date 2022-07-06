#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/exponential.hpp>
#include <glm/geometric.hpp>
#include <optional>
#include <memory>

#include "utils.h"

namespace rtweekend::detail {
  class Ray {
  public:
    Ray(point origin, vec3 direction) : origin_{origin}, direction_{direction} {}
    vec3 at(double t) const {return origin_ + direction_ * t;}
    [[nodiscard]] const point& origin() const {return origin_;}
    [[nodiscard]] const vec3& direction() const {return direction_;}
  private:
    vec3 origin_;
    vec3 direction_;
  };

  class Hit;

  struct ScatterRecord {
    Ray r;
    color attenuation;
  };

  class Material { // NOLINT(cppcoreguidelines-special-member-functions)
  public:
    [[nodiscard]] virtual std::optional<ScatterRecord>
    scatter(const Ray& r_in, const Hit& rec) const = 0;

    virtual ~Material() = default;
  };

  class Primitive { // NOLINT(cppcoreguidelines-special-member-functions)
  public:
    [[nodiscard]] virtual std::optional<Hit>
    hit(const Ray& r, double tmin, double tmax) const = 0;
    explicit Primitive(const Material& m) : material_{&m} {};

    virtual ~Primitive() = default;
    [[nodiscard]] const Material& material() const {return *material_;}
  private:
    const Material* material_;
  };

  class Hit {
  private:
    const Primitive* what_;
    point where_;
    double at_;
    vec3 normal_;
    bool front_facing_;
  public:
    Hit(const Primitive& what, point where, double at, vec3 normal,
        bool front_facing) :
      what_{&what}, where_{where}, at_{at}, normal_{normal}, front_facing_{front_facing} {}

    [[nodiscard]] const Primitive& what() const {return *what_;}
    [[nodiscard]] const point& where() const noexcept {return where_;}
    [[nodiscard]] const vec3& normal() const noexcept {return normal_;}
    [[nodiscard]] double at() const noexcept {return at_;}
    [[nodiscard]] bool front_facing() const noexcept {return front_facing_;}
  };

  struct Lambertian : public Material {
    explicit Lambertian(const color& albedo) : albedo{albedo} {}

    std::optional<ScatterRecord>
    scatter(const Ray&, const Hit& hit) const override {
      auto rand = random_unit_vector();
      if (glm::all(glm::epsilonEqual(hit.normal(), rand, 1e-8))) {
        return {};
      }
      auto scatter_direction = hit.normal() + rand;
      return ScatterRecord{.r = Ray{hit.where(), scatter_direction},
                           .attenuation = albedo};
    }
    color albedo;
  };

  struct Metal : public Material {
    explicit Metal(const color& albedo, double fuzz=0)
      : albedo{albedo}, fuzz{std::clamp(fuzz, 0.0, 1.0)} {}

    std::optional<ScatterRecord>
    scatter(const Ray& r, const Hit& hit) const override {
      auto reflected = glm::reflect(r.direction(), hit.normal());

      return ScatterRecord{.r = Ray{hit.where(), reflected + fuzz*random_unit_vector()},
                           .attenuation = albedo};
    }

    color albedo;
    double fuzz;
  };

  struct Dielectric : public Material {
    explicit Dielectric(double index_of_refraction, double fuzz=0)
      : ir{index_of_refraction}, fuzz{std::clamp(fuzz, 0.0, 1.0)} {}

    std::optional<ScatterRecord>
    scatter(const Ray& ray, const Hit& rec) const override {

      auto unit_direction = glm::normalize(ray.direction());

      double cos_theta = dot(-unit_direction, rec.normal());
      double sin_theta = sqrt(1.0 - cos_theta*cos_theta);

      double refraction_ratio = rec.front_facing() ? (1.0/ir) : ir;

      bool cannot_refract = refraction_ratio * sin_theta > 1.0;
      vec3 direction;

      if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_double())
        direction = glm::reflect(unit_direction, rec.normal());
      else
        direction = glm::refract(unit_direction, rec.normal(), refraction_ratio);


      return ScatterRecord{.r = Ray{rec.where(),
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

  struct Sphere : public Primitive {
    point center;
    double radius;
    Sphere(point c, double r, const Material& m) : Primitive{m}, center{c}, radius{r} {};

    [[nodiscard]] std::optional<Hit>
    hit(const Ray& r, double tmin, double tmax) const override {
      vec3 oc = r.origin() - center;                      // (A-C)
      double a = glm::dot(r.direction(), r.direction());    // (b.b)
      auto h = glm::dot(oc, r.direction());               // (A-C).b
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

      bool front_facing = (glm::dot(r.direction(), normal) < 0) ^ (radius < 0);
      normal = front_facing?normal:-normal;
      return Hit{*this, hitpoint, root, normal, front_facing};
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

  template <typename Base>
  class Store {
    std::vector<std::unique_ptr<Base>> items_;
  public:
    template <typename Derived, typename ...Args>
    Derived& add(Args&& ...args) {
      items_.push_back(std::make_unique<Derived>(std::forward<Args>(args)...));
      return static_cast<Derived&>(*items_.back());
    }

    [[nodiscard]] auto begin() const {return items_.begin();}
    [[nodiscard]] auto end() const {return items_.end();}
  };

  using World = Store<Primitive>;
  using Boutique = Store<Material>;

  color ray_color(const Ray& ray, const World& world, size_t max_depth=20) {
    std::optional<Hit> hit{};
    double upper_bound = std::numeric_limits<double>::infinity();
    for (const auto& h : world) {
      auto probe = h->hit(ray, 0.001, upper_bound);
      if (probe) {
        hit = probe;
        upper_bound = probe->at();
      }
    }
    if (hit) {
      if (max_depth <= 0) return {0,0,0};

      auto scatter = hit->what().material().scatter(ray, hit.value());
      if (scatter)
        return scatter->attenuation * ray_color(scatter->r, world, max_depth-1);
      return {0,0,0};
    }

    // Fallback to background
    // t is between 0 and 1, proportional to y
    // lower t, whiter image, higher t, bluer image centered
    vec3 unit_direction = glm::normalize(ray.direction());
    auto ycomp = unit_direction.y;
    auto t =  0.5*(ycomp + +1.0);
    return (1.0-t)*color{1.0, 1.0, 1.0} + t*color{0.5, 0.7, 1.0};
  }

}  // namespace rtweekend::detail

namespace rtweekend {
  using detail::World;
  using detail::Boutique;
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

// Local Variables:
// mode: c++
// End:
