#include <chrono>
#include <future>
#include <iostream>
#include <random>

#include "random_utils.h"
#include "render.h"
#include "model.h"

namespace rtweekend {
  static void write_color(std::ostream &out, color c, int samples_per_pixel) {
    // Divide the color by the number of samples and correct with gamma=2

    c = glm::sqrt(c / static_cast<double>(samples_per_pixel));

    // Write the translated [0,255] value of each color component.
    out << static_cast<int>(256 * std::clamp(c.r, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * std::clamp(c.g, 0.0, 0.999)) << ' '
        << static_cast<int>(256 * std::clamp(c.b, 0.0, 0.999)) << '\n';
  }

  void render(const World &world, const Camera &cam, const Config &cfg) {
    int image_height = static_cast<int>(cfg.image_width / cfg.aspect_ratio);

    namespace khr = std::chrono;
    std::cerr << "Started rendering with " << cfg.nthreads << " threads\n";
    khr::time_point start{khr::high_resolution_clock::now()};

    using Image = std::vector<color>;
    Image global_image(cfg.image_width *image_height, color{0, 0, 0});

    auto root = detail::BVHNode{world};

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

}  // namespace rtweekend
