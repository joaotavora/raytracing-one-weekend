#include <glm/gtx/norm.hpp>

#include "vec3.h"
#include "random_utils.h"
#include "model.h"

namespace rtweekend::detail {
  color ray_color(const Ray& ray, const World& world, size_t max_depth) {
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

  std::optional<ScatterRecord> Lambertian::scatter(const Ray &,
                                                   const Hit &hit) const {
    auto rand = random_unit_vector();
    if (glm::all(glm::epsilonEqual(hit.normal(), rand, 1e-8))) {
      return {};
    }
    auto scatter_direction = hit.normal() + rand;
    return ScatterRecord{.r = Ray{hit.where(), scatter_direction},
                         .attenuation = albedo};
  }

  std::optional<ScatterRecord> Metal::scatter(const Ray &r,
                                              const Hit &hit) const {
    auto reflected = glm::reflect(r.direction(), hit.normal());

    return ScatterRecord{
      .r = Ray{hit.where(), reflected + fuzz * random_unit_vector()},
      .attenuation = albedo};
  }

  std::optional<ScatterRecord> Dielectric::scatter(const Ray &ray,
                                                   const Hit &rec) const {

    auto unit_direction = glm::normalize(ray.direction());

    double cos_theta = dot(-unit_direction, rec.normal());
    double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

    double refraction_ratio = rec.front_facing() ? (1.0 / ir) : ir;

    bool cannot_refract = refraction_ratio * sin_theta > 1.0;
    vec3 direction;

    if (cannot_refract ||
        reflectance(cos_theta, refraction_ratio) > random_double())
      direction = glm::reflect(unit_direction, rec.normal());
    else
      direction = glm::refract(unit_direction, rec.normal(), refraction_ratio);

    return ScatterRecord{
      .r = Ray{rec.where(), direction + fuzz * random_unit_vector()},
      .attenuation = {1.0, 1.0, 1.0}};
  }

  std::optional<Hit> Sphere::hit(const Ray &r, double tmin, double tmax) const {
    vec3 oc = r.origin() - center;                     // (A-C)
    double a = glm::dot(r.direction(), r.direction()); // (b.b)
    auto h = glm::dot(oc, r.direction());              // (A-C).b
    auto c = glm::dot(oc, oc) - radius * radius;       // (A-C)(A-C) - r^2
    auto discriminant = h * h - a * c;
    if (discriminant < 0.0)
      return {};

    auto root = (-h - std::sqrt(discriminant)) / a;
    if (root < tmin || root > tmax) {

      root = (-h + std::sqrt(discriminant)) / a;
      if (root < tmin || root > tmax)
        return {};
    }

    auto hitpoint = r.at(root);
    auto normal = glm::normalize(hitpoint - center);

    bool front_facing = (glm::dot(r.direction(), normal) < 0) ^ (radius < 0);
    normal = front_facing ? normal : -normal;
    return Hit{*this, hitpoint, root, normal, front_facing};
  }

  Camera::Camera(point lookfrom, point lookat, vec3 vup, double fov,
                 double aspect_ratio, double aperture,
                 std::optional<double> focus_dist, time_t t0, time_t t1)
    : origin_{lookfrom}, w_{glm::normalize(lookfrom - lookat)},
      u_{glm::normalize(glm::cross(vup, w_))},
      v_{glm::normalize(glm::cross(w_, u_))},
      lens_radius_{aperture / 2},
      t0_{t0}, t1_{t1} {

    auto viewport_height = 2.0 * std::tan(fov * std::numbers::pi / 180 / 2);
    auto viewport_width = aspect_ratio * viewport_height;

    auto fd = focus_dist ? focus_dist.value() : glm::length(lookfrom - lookat);

    horizontal_ = fd * viewport_width * u_;
    vertical_ = fd * viewport_height * v_;
    lower_left_corner_ =
      origin_ - horizontal_ / 2.0 - vertical_ / 2.0 - fd * w_;
  }

  Ray Camera::get_ray(double s, double t) const {
    vec3 rd = lens_radius_ * random_in_unit_disk();
    vec3 offset = u_ * rd.x + v_ * rd.y;

    auto from = origin_ + offset;
    auto direction = lower_left_corner_ +
      s * horizontal_ +
      t * vertical_
      - from;
    auto when = random_double(t0_, t1_);
    return Ray{from, direction, when};
  }
} // namespace rtweekend::detail
