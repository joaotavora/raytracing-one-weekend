#pragma once

#include <optional>
#include <span>
#include <memory>
#include <vector>
#include <algorithm>

#include "vec3.h"
#include "common-model.h"

namespace rtweekend::detail {
  extern int thrashing_allocator_pathos; // NOLINT

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

    [[nodiscard]] virtual Aabb bounding_box() const = 0;

    explicit Primitive(const Material& m) : material_{&m} {};

    virtual ~Primitive() = default;
    [[nodiscard]] const Material& material() const {return *material_;}
  private:
    const Material* material_;
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

  class Sphere : public Primitive {
  public:
    Sphere(point center, double radius, const Material& material)
      : Primitive{material}, center_{center}, radius_{radius} {}

    [[nodiscard]] std::optional<Hit>
    hit(const Ray &r, double tmin, double tmax) const override;

    [[nodiscard]] Aabb
    bounding_box() const override;

    const point& center() const {
      return center_;
    }

    [[nodiscard]] const double& radius() const {return radius_;}
  private:
    point center_;
    double radius_;
  };

  class MovingSphere : public Primitive {
  public:
    MovingSphere(point c0, point c1, double radius,
                 const Material& material)
      : Primitive{material}, center0_{c0}, center1_{c1},
        t0_{0.0}, t1_{1.0}, radius_{radius} {};

    [[nodiscard]] std::optional<Hit>
    hit(const Ray &r, double tmin, double tmax) const override;

    [[nodiscard]] Aabb bounding_box() const override;

    const point& center() const {
      return center0_;
    }

    [[nodiscard]]
    point center(time_t time) const {
      return center0_ + ((time - t0_) / (t1_ - t0_))*(center1_ - center0_);
    }
    [[nodiscard]] const double& radius() const {return radius_;}
  private:
    point center0_, center1_;
    time_t t0_, t1_;
    double radius_;
  };

  inline auto hit(const std::unique_ptr<Primitive>& h, const Ray& r, double tmin, double tmax) {
    return h->hit(r, tmin, tmax);
  }

  inline auto bounding_box(const std::unique_ptr<Primitive>& h) {
    return h->bounding_box();
  }
}  // namespace rtweekend::detail

namespace rtweekend {
  using PrimitiveStore_t = detail::OOStore<detail::Primitive>;
  using MaterialStore_t = detail::OOStore<detail::Material>;
  using detail::Sphere;
  using detail::MovingSphere;
  using detail::Material;
  using detail::Lambertian;
  using detail::Metal;
  using detail::Dielectric;
}  // namespace rtweekend

// Local Variables:
// mode: c++
// End:
