#pragma once

#include <optional>
#include <vector>
#include <memory>

#include "vec3.h"

namespace rtweekend::detail {

  using time_t = double;
 
  class Hit;
  class Aabb;

  class Ray {
  public:
    Ray(point origin, vec3 direction, time_t time = 0.0) :
      origin_{origin},
      direction_{direction},
      time_{time}{}
    vec3 at(double t) const {return origin_ + direction_ * t;}
    [[nodiscard]] const point&  origin()    const {return origin_;}
    [[nodiscard]] const vec3&   direction() const {return direction_;}
    [[nodiscard]] const time_t& time()      const {return time_;}
  private:
    vec3 origin_;
    vec3 direction_;
    time_t time_;
  };

  struct ScatterRecord {
    Ray r;
    color attenuation;
  };

  class Primitive;

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

  class Aabb {
  public:
    Aabb() {}
    Aabb(const point& a, const point& b)
      : min_{a}, max_{b} {}

    const point& min() const {return min_; }
    const point& max() const {return max_; }
    double volume() const {
      return (max_.x - min_.x) * (max_.y - min_.y) * (max_.z - min_.z);
    }

    bool hit(const Ray& r, double t_min, double t_max) const {
      for (int a = 0; a < 3; a++) {
        auto inv_direction = 1.0F / r.direction()[a];
        auto t0 = (min()[a] - r.origin()[a]) * inv_direction;
        auto t1 = (max()[a] - r.origin()[a]) * inv_direction;
        if (inv_direction < 0.0F)
          std::swap(t0, t1);
        t_min = t0 > t_min ? t0 : t_min;
        t_max = t1 < t_max ? t1 : t_max;
        if (t_max <= t_min)
          return false;
      }
      return true;
    }
  private:
    point min_{};
    point max_{};
  };

  Aabb surrounding_box(Aabb box0, Aabb box1);

  class Camera {
  public:
    static constexpr auto focal_length = 1.0;
    Camera(point lookfrom, point lookat, vec3 vup, double fov,
           double aspect_ratio, double aperture,
           std::optional<double> focus_dist = std::nullopt,
           time_t t0 = 0, time_t t1 = 0);

    Ray get_ray(double s, double t) const;

  private:

    point origin_;

    vec3 w_, u_, v_;

    vec3 horizontal_{};
    vec3 vertical_{};
    point lower_left_corner_{};
    double lens_radius_;
    double t0_, t1_;
  };

    class Material { // NOLINT(cppcoreguidelines-special-member-functions)
  public:
    [[nodiscard]] virtual std::optional<ScatterRecord>
    scatter(const Ray& r_in, const Hit& rec) const = 0;

    virtual ~Material() = default;
  };

  struct Lambertian : public Material {
    explicit Lambertian(const color& albedo) : albedo{albedo} {}

    std::optional<ScatterRecord> scatter(const Ray &,
                                         const Hit &hit) const override;
    color albedo;
  };

  struct Metal : public Material {
    explicit Metal(const color& albedo, double fuzz=0)
      : albedo{albedo}, fuzz{std::clamp(fuzz, 0.0, 1.0)} {}

    std::optional<ScatterRecord> scatter(const Ray &r,
                                         const Hit &hit) const override;

    color albedo;
    double fuzz;
  };

  struct Dielectric : public Material {
    explicit Dielectric(double index_of_refraction, double fuzz=0)
      : ir{index_of_refraction}, fuzz{std::clamp(fuzz, 0.0, 1.0)} {}

    std::optional<ScatterRecord> scatter(const Ray &ray,
                                         const Hit &rec) const override;
    double ir;
    double fuzz;
  };


  template <typename Base>
  class OOStore : private std::vector<std::unique_ptr<Base>> {
    using IBase = std::vector<std::unique_ptr<Base>>;

  public:
    template <typename Derived, typename ...Args>
    Derived& add(Args&& ...args) {
      IBase::push_back(std::make_unique<Derived>(std::forward<Args>(args)...));
      return static_cast<Derived&>(*IBase::back());
    }

    using IBase::cbegin, IBase::cend, IBase::size, IBase::data;
    using IBase::begin, IBase::end;

  };


}  // namespace rtweekend::detail

namespace rtweekend {
  using detail::OOStore;
  using detail::Camera;
  using detail::Material;
  using detail::Lambertian;
  using detail::Metal;
  using detail::Dielectric;
}  // namespace rtweekend

// Local Variables:
// mode: c++
// End:
