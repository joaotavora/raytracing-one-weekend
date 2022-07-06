#include <chrono>
#include <iostream>
#include <glm/glm.hpp>
#include <functional>
#include <random>
#include <future>

#include "utils.h"
#include "model.h"

struct Config {
  double aspect_ratio = 3.0/2.0;
  int image_width = 1200;
  int samples_per_pixel = 100;
  int max_child_rays = 50;
  int nthreads = 4;
}; // NOLINT

namespace rt=rtweekend;

struct Scene {
  rt::World world{};
  rt::Boutique boutique{};
};

Scene scene() {
  Scene scn{};
  auto& world = scn.world;
  auto& boutique = scn.boutique;

  auto& ground_material = boutique.add<rt::Lambertian>(rt::color{0.5, 0.5, 0.5});
  world.add<rt::Sphere>(rt::point{0,-1000,0}, 1000, ground_material);

  for (int a = -11; a < 11; a++) {
    for (int b = -11; b < 11; b++) {
      auto choose_mat = rt::random_double();
      rt::point center(a+ 0.9*rt::random_double(), 0.2, b + 0.9*rt::random_double());

      rt::Material* mat{};
      if (glm::length(center - rt::point{4,0.2,0}) > 0.9) {
        if (choose_mat < 0.8) {
          // diffuse
          auto albedo = rt::random_vec3() * rt::random_vec3();
          mat = &boutique.add<rt::Lambertian>(albedo);
        } else if (choose_mat < 0.95) {
          // metal
          auto albedo = rt::random_vec3(0.5, 1);
          auto fuzz = rt::random_double(0, 0.5);
          mat = &boutique.add<rt::Metal>(albedo, fuzz);
        } else {
          // glass
          mat = &boutique.add<rt::Dielectric>(1.5);
        }
        world.add<rt::Sphere>(center, 0.2, *mat);
      }
    }
  }

  auto& glass = boutique.add<rt::Dielectric>(1.5);
  auto& reddish = boutique.add<rt::Lambertian>(rt::color{0.4, 0.2, 0.1});
  auto& reddish_metal = boutique.add<rt::Metal>(rt::color{0.7, 0.6, 0.5});

  world.add<rt::Sphere>(rt::point(0, 1, 0), 1.0, glass);
  world.add<rt::Sphere>(rt::point(-4, 1, 0), 1.0, reddish);
  world.add<rt::Sphere>(rt::point(4, 1, 0), 1.0, reddish_metal);
  return scn;
}

void render(const Scene& scn, const rt::Camera& cam, const Config& cfg) {
  const auto& world = scn.world;
  int image_height = static_cast<int>(cfg.image_width/cfg.aspect_ratio); // NOLINT

  namespace khr = std::chrono;
  std::cerr << "Started rendering with " << cfg.nthreads << " threads\n";
  khr::time_point start{khr::high_resolution_clock::now()};

  using Image = std::vector<rt::color>;
  Image global_image(cfg.image_width * image_height, rt::color{0,0,0});

  auto do_work = [&](int samples) {
    Image local_image(cfg.image_width * image_height, rt::color{0,0,0});
    for (int i = 0; i < image_height; ++i) {
      auto from_top_i = image_height - i -1;
      std::cerr << "\rScanlines remaining: "
                << from_top_i << ' ' << std::flush;
      for (int j = 0; j < cfg.image_width; ++j) {
        rt::color pixel_color{0,0,0};
        for (int s = 0; s < samples; ++s) {
          auto u = (j + rt::random_double()) / (cfg.image_width-1);
          auto v = (from_top_i + rt::random_double()) / (image_height-1);
          auto r = cam.get_ray(u, v);
          pixel_color += rt::ray_color(r, world, cfg.max_child_rays);
        }
        local_image[i*cfg.image_width + j] = pixel_color;
      }
    }
    return local_image;
  };

  std::vector<std::future<Image>> work{};
  work.reserve(cfg.nthreads);

  for (int i = 0; i < cfg.nthreads; ++i) {
    work.push_back(std::async(std::launch::async,
                              do_work,
                              cfg.samples_per_pixel/cfg.nthreads));
  }
  for (auto& f : work) {
    const auto& done = f.get();
    std::transform(done.cbegin(), done.cend(),
                   global_image.cbegin(), global_image.begin(),
                   std::plus<rt::color>());
  }

  std::cout << "P3\n" << cfg.image_width << ' ' << image_height << "\n255\n";
  for (const auto& c : global_image) {
    rt::write_color(std::cout, c,
                    cfg.samples_per_pixel/cfg.nthreads * cfg.nthreads);
  }

  auto took = khr::high_resolution_clock::now() - start;
  std::cerr << "\nDone in "
            << khr::duration_cast<khr::milliseconds>(took) << "\n";
}

int main(int argc, char* argv[]) {
  Config cfg{};
  bool quick = argc>=2 && std::string{argv[1]} == "-q"; // NOLINT
  if (quick) {
    cfg.image_width = 300;
    cfg.samples_per_pixel = 48;
    cfg.max_child_rays = 25;
  }
  // Camera
  rt::Camera cam{rt::point(13,2,3),
                 rt::point(0,0,0),
                 rt::vec3(0,1,0),
                 20.0,
                 cfg.aspect_ratio,
                 0.1,
                 10.0};

  render(scene(), cam, cfg);
}
