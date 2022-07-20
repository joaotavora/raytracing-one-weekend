#include <glm/gtx/norm.hpp>
#include <numeric>
#include <span>
#include <ranges>
#include <iostream>

#include "vec3.h"
#include "random_utils.h"
#include "model.h"

namespace rtweekend::detail {
  color ray_color(const Ray& ray, const BVHNode& root, size_t max_depth) {
    auto hit = root.hit(ray);
    if (hit) {
      if (max_depth <= 0) return {0,0,0};

      auto scatter = hit->what().material().scatter(ray, hit.value());
      if (scatter)
        return scatter->attenuation * ray_color(scatter->r, root, max_depth-1);
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

  std::optional<ScatterRecord> Lambertian::scatter(const Ray &r,
                                                   const Hit &hit) const {
    auto rand = random_unit_vector();
    if (glm::all(glm::epsilonEqual(hit.normal(), rand, 1e-8))) {
      return {};
    }
    auto scatter_direction = hit.normal() + rand;
    return ScatterRecord{.r = Ray{hit.where(), scatter_direction, r.time()},
                         .attenuation = albedo};
  }

  std::optional<ScatterRecord> Metal::scatter(const Ray &r,
                                              const Hit &hit) const {
    auto reflected = glm::reflect(r.direction(), hit.normal());

    return ScatterRecord{
      .r = Ray{hit.where(), reflected + fuzz * random_unit_vector(), r.time()},
      .attenuation = albedo};
  }

  std::optional<ScatterRecord> Dielectric::scatter(const Ray &r,
                                                   const Hit &rec) const {

    auto unit_direction = glm::normalize(r.direction());

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
      .r = Ray{rec.where(), direction + fuzz * random_unit_vector(), r.time()},
      .attenuation = {1.0, 1.0, 1.0}};
  }

  inline std::optional<Hit> sphere_hit_helper(const Ray &r,
                                       double tmin,
                                       double tmax,
                                       const point& center,
                                       double radius,
                                       const Primitive& hit) {
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
    return Hit{hit, hitpoint, root, normal, front_facing};
  }

  std::optional<Hit>
  Sphere::hit(const Ray &r, double tmin, double tmax) const {
    return sphere_hit_helper(r, tmin, tmax, center(), radius(), *this);
  }

  std::optional<Hit>
  MovingSphere::hit(const Ray &r, double tmin, double tmax) const {
    return sphere_hit_helper(r, tmin, tmax, center(r.time()), radius(), *this);
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
  Aabb Sphere::bounding_box() const {
    return Aabb{center_ - vec3(radius_, radius_, radius_),
                center_ + vec3(radius_, radius_, radius_)};
  }

  Aabb surrounding_box(Aabb box0, Aabb box1) {
    point small(fmin(box0.min().x, box1.min().x),
                fmin(box0.min().y, box1.min().y),
                fmin(box0.min().z, box1.min().z));

    point big(fmax(box0.max().x, box1.max().x),
              fmax(box0.max().y, box1.max().y),
              fmax(box0.max().z, box1.max().z));

    return Aabb(small,big);
  }

  Aabb intersection_box(Aabb box0, Aabb box1) {
    point small(fmax(box0.min().x, box1.min().x),
                fmax(box0.min().y, box1.min().y),
                fmax(box0.min().z, box1.min().z));

    point big(fmin(box0.max().x, box1.max().x),
              fmin(box0.max().y, box1.max().y),
              fmin(box0.max().z, box1.max().z));

    return Aabb(small,big);
  }

  Aabb MovingSphere::bounding_box() const {
    double time0 = 0.0;
    double time1 = 1.0;
    Aabb box0(
              center(time0) - vec3(radius_, radius_, radius_),
              center(time0) + vec3(radius_, radius_, radius_));
    Aabb box1(
              center(time1) - vec3(radius_, radius_, radius_),
              center(time1) + vec3(radius_, radius_, radius_));
    return surrounding_box(box0, box1);
  }
  std::optional<Hit> BVHNode::hit(const Ray &r, double tmin, double tmax) const {
    if (!bounding_box_.hit(r, tmin, tmax)) return {};

    if (!wv_.empty()) {
      std::optional<Hit> hit{};
      auto upper_bound = tmax;
      for (const auto &h : wv_) {
        auto probe = h->hit(r, tmin, upper_bound);
        if (probe) {
          hit = probe;
          upper_bound = probe->at();
        }
      }
      return hit;
    }
    auto left_probe = left_->hit(r, tmin, tmax);
    auto right_probe = right_->hit(r, tmin, left_probe?left_probe->at():tmax);
    if (right_probe) return right_probe;
    return left_probe;
  }

  BVHNode::BVHNode(WorldView_t wv) {
    namespace rv = std::ranges::views;
    if (!wv.empty() && wv.size() <= 6) {
      wv_ = wv;
      for (const auto& p : wv) {
        bounding_box_ = surrounding_box(bounding_box_, p->bounding_box());
      }
    } else {
      auto axis = [&]{
        auto pbegbb = (*wv.begin())->bounding_box();
        auto pendbb = (*wv.rbegin())->bounding_box();
        int retval{};
        auto delta = pendbb.min() - pbegbb.min();
        if (::fabs(delta.x) > ::fabs(delta.y)) {
          if (::fabs(delta.x) > ::fabs(delta.z)) retval = 0;
          else retval = 2;
        } else {
          if (::fabs(delta.y) > ::fabs(delta.z)) retval = 1;
          else retval = 2;
        }
        return retval;
      }();
      std::sort(wv.begin(), wv.end(),
                [&axis](auto& p1, auto& p2) {
                auto p1b = p1->bounding_box();
                auto p2b = p2->bounding_box();

                return p1b.min()[axis] < p2b.min()[axis];
                return true;
                });

      auto total = wv.size();
      auto leftn = total / 2;
      left_ = std::make_unique<BVHNode>(wv.first(leftn));
      right_ = std::make_unique<BVHNode>(wv.last(total - leftn));
      bounding_box_ = surrounding_box(left_->bounding_box_,
                                      right_->bounding_box_);
    }
  }

  std::ostream& operator<<(std::ostream& o, const point& x) {
    return o << "[" << x.x << "," << x.y << "," << x.z << "]";
  }

  std::ostream& operator<<(std::ostream& o, const Aabb& x) {
    return o << "[Aabb min: " << x.min() << " max: " << x.max() << "]";
  }

  // double BVHNode::volume() const {
  //   return bounding_box_.volume() +
  //     (left_?left_->volume():0) +
  //     (right_?right_->volume():0);
  // };

  double BVHNode::stupid_volume() const {
    double myown = bounding_box_.volume();
    double childrens = 0;
    if (left_ && right_) {
      myown-= left_->bounding_box_.volume();
      myown-= right_->bounding_box_.volume();
      childrens+= left_->stupid_volume();
      childrens+= right_->stupid_volume();
    } else {
      myown = 0;
    }
    if (myown < 0) myown = -myown; else myown = 0;
    return myown + childrens;
  }
} // namespace rtweekend::detail
