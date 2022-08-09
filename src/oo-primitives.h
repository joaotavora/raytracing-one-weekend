#pragma once

#include <optional>
#include <span>
#include <memory>
#include <vector>
#include <algorithm>

#include "vec3.h"
#include "common-model.h"

namespace rtweekend::detail {
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

  using PView_t = std::span<std::unique_ptr<Primitive>>;

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
}  // namespace rtweekend

// Local Variables:
// mode: c++
// End:
