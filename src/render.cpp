#include <chrono>
#include <future>
#include <iostream>
#include <random>

#include "random_utils.h"
#include "render.h"
#include "oomodel.h"

#include <fmt/core.h>

namespace rtweekend::detail {
  static void write_color(std::ostream &out, color c, int samples_per_pixel) {
    // Divide the color by the number of samples and correct with gamma=2

    c = glm::sqrt(c / static_cast<double>(samples_per_pixel));

    // Write the translated [0,255] value of each color component.
    out << static_cast<int>(256 * std::clamp(c.r, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * std::clamp(c.g, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * std::clamp(c.b, 0.0, 0.999)) << '\n';
  }

  class BVHNode {
    PView_t wv_;
    Aabb bounding_box_;
    std::unique_ptr<BVHNode> left_{};
    std::unique_ptr<BVHNode> right_{};

  public:
    double stupid_volume() const;
    explicit BVHNode(PView_t wv);

    [[nodiscard]] std::optional<Hit>
    hit(const Ray &r, double tmin = 0.001,
        double tmax = std::numeric_limits<double>::infinity()) const;
  };

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

  BVHNode::BVHNode(PView_t wv) {
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

  BVHNode World::get_root_bvh() const {
      return detail::BVHNode{primitives_};
  }

  void render(const World& world, const Camera &cam, const Config &cfg) {
    int image_height = static_cast<int>(cfg.image_width / cfg.aspect_ratio);

    namespace khr = std::chrono;
    std::cerr << "Started rendering with " << cfg.nthreads << " threads\n";
    khr::time_point start{khr::high_resolution_clock::now()};

    using Image = std::vector<color>;
    Image global_image(cfg.image_width *image_height, color{0, 0, 0});

    auto root = world.get_root_bvh();

    std::cerr << "Total BVH stupid volume: " << root.stupid_volume() << "\n";

    auto do_work = [&](int samples) {
      Image local_image(cfg.image_width * image_height, color{0, 0, 0});
      for (int i = 0; i < image_height; ++i) {
        auto from_top_i = image_height - i - 1;
        std::cerr << "\rScanlines remaining: " << from_top_i << ' ' << std::flush;
        for (int j = 0; j < cfg.image_width; ++j) {
          color pixel_color{0, 0, 0};
          for (int s = 0; s < samples; ++s) {
            auto u = (j + random_double()) / (cfg.image_width - 1);
            auto v = (from_top_i + random_double()) / (image_height - 1);
            auto r = cam.get_ray(u, v);
            pixel_color += ray_color(r, root, cfg.max_child_rays);
          }
          local_image[i * cfg.image_width + j] = pixel_color;
        }
      }
      return local_image;
    };

    std::vector<std::future<Image>> work{};
    work.reserve(cfg.nthreads);

    for (int i = 0; i < cfg.nthreads; ++i) {
      work.push_back(std::async(std::launch::async, do_work,
                                cfg.samples_per_pixel / cfg.nthreads));
    }
    for (auto &f : work) {
      const auto &done = f.get();
      std::transform(done.cbegin(), done.cend(), global_image.cbegin(),
                     global_image.begin(), std::plus<color>());
    }

    std::cout << "P3\n" << cfg.image_width << ' ' << image_height << "\n255\n";
    for (const auto &c : global_image) {
      write_color(std::cout, c,
                  cfg.samples_per_pixel / cfg.nthreads * cfg.nthreads);
    }

    auto took = khr::high_resolution_clock::now() - start;
    std::cerr << "\nDone in " << khr::duration_cast<khr::milliseconds>(took)
              << "\n";
  }

  std::ostream& operator<<(std::ostream& o, const Config& c) {
    return o << "Config {\n"
             << "aspect_ratio: "      << c.aspect_ratio << "\n"
             << "number_of_balls_sqrt: " << c.number_of_balls_sqrt << "\n"
             << "moving_spheres: "    << c.moving_spheres << "\n"
             << "image_width: "       << c.image_width << "\n"
             << "samples_per_pixel: " << c.samples_per_pixel << "\n"
             << "max_child_rays: "    << c.max_child_rays << "\n"
             << "nthreads: "          << c.nthreads << "\n"
             << "}\n";
  }

}  // namespace rtweekend::detail
