#pragma once

#include <optional>
#include <span>
#include <memory>
#include <variant>
#include <vector>
#include <algorithm>

#include "vec3.h"
#include "common-model.h"

namespace rtweekend::detail {
  extern int thrashing_allocator_pathos; // NOLINT

  class Primitive {
  public:
    explicit Primitive(const Material& m) : material_{&m} {};
    [[nodiscard]] const Material& material() const {return *material_;}
  private:
    const Material* material_;
  };

  class Sphere : public Primitive {
  public:
    Sphere(point center, double radius, const Material& material)
      : Primitive{material}, center_{center}, radius_{radius} {}

    [[nodiscard]] std::optional<Hit>
    hit(const Ray &r, double tmin, double tmax) const;

    [[nodiscard]] Aabb
    bounding_box() const;

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
    hit(const Ray &r, double tmin, double tmax) const;

    [[nodiscard]] Aabb bounding_box() const;

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

  template <typename ...T>
  class VariantStore : private std::vector<std::variant<T...>> {
  public:
    using value_type = std::variant<T...>;
  private:
    using IBase = std::vector<value_type>;

  public:
    template <typename U, typename ...Args>
    U& add(Args&& ...args) {
      auto u = U{std::forward<Args>(args)...};
      IBase::push_back(u);
      return std::get<U>(IBase::back());
    }

    using IBase::cbegin, IBase::cend, IBase::size, IBase::data;
    using IBase::begin, IBase::end;

  };

  using PrimitiveStore_t = detail::VariantStore<detail::Sphere, detail::MovingSphere>;
  using PView_t = std::span<PrimitiveStore_t::value_type>;

  inline auto hit(const PrimitiveStore_t::value_type& h, const Ray& r, double tmin, double tmax) {
    return std::visit([&](const auto& h){return h.hit(r, tmin, tmax);}, h);
  }

  inline auto bounding_box(const PrimitiveStore_t::value_type& h) {
    return std::visit([](const auto& h){return h.bounding_box();}, h);
  }
}  // namespace rtweekend::detail

namespace rtweekend {
  using detail::PrimitiveStore_t;
  using MaterialStore_t = detail::OOStore<detail::Material>;
  using detail::Sphere;
  using detail::MovingSphere;
}  // namespace rtweekend

// Local Variables:
// mode: c++
// End:
