#include <random>

#include <CLI/App.hpp>
#include <CLI/Formatter.hpp>
#include <CLI/Config.hpp>

#include "random-utils.h"
#include "primitive-model.h"
#include "render.h"

namespace rt=rtweekend;

int rt::detail::thrashing_allocator_pathos = 0; // NOLINT

rt::World lots_of_balls(const rt::Config& cfg) {
  rt::World world{};
  auto& boutique = world.boutique();

  auto& ground_material = boutique.add<rt::Lambertian>(rt::color{0.5, 0.5, 0.5});
  world.primitives().add<rt::Sphere>(rt::point{0,-1000,0}, 1000.0, ground_material);

  int nsqrt = cfg.number_of_balls_sqrt;

  for (int a = -nsqrt; a < nsqrt; a++) {
    for (int b = -nsqrt; b < nsqrt; b++) {
      auto choose_mat = rt::random_double();
      rt::point center(a+ 0.9*rt::random_double(), 0.2, b + 0.9*rt::random_double());

      rt::Material* mat{};
      if (glm::length(center - rt::point{4,0.2,0}) > 0.9) {
        if (choose_mat < 0.8) {
          // diffuse
          auto albedo = rt::random_vec3() * rt::random_vec3();
          mat = &boutique.add<rt::Lambertian>(albedo);
          if (cfg.moving_spheres) {
            auto center2 = center + rt::point(0, rt::random_double(0,.5), 0);
            world.primitives().add<rt::MovingSphere>(center, center2, 0.2, *mat);
          } else {
            world.primitives().add<rt::Sphere>(center, 0.2, *mat);
          }
        } else if (choose_mat < 0.95) {
          // metal
          auto albedo = rt::random_vec3(0.5, 1);
          auto fuzz = rt::random_double(0, 0.5);
          mat = &boutique.add<rt::Metal>(albedo, fuzz);
          world.primitives().add<rt::Sphere>(center, 0.2, *mat);
        } else {
          // glass
          mat = &boutique.add<rt::Dielectric>(1.5);
          world.primitives().add<rt::Sphere>(center, 0.2, *mat);
        }
      }
    }
  }

  auto& glass = boutique.add<rt::Dielectric>(1.5);
  auto& reddish = boutique.add<rt::Lambertian>(rt::color{0.4, 0.2, 0.1});
  auto& reddish_metal = boutique.add<rt::Metal>(rt::color{0.7, 0.6, 0.5});

  world.primitives().add<rt::Sphere>(rt::point(0, 1, 0), 1.0, glass);
  world.primitives().add<rt::Sphere>(rt::point(-4, 1, 0), 1.0, reddish);
  world.primitives().add<rt::Sphere>(rt::point(4, 1, 0), 1.0, reddish_metal);
  return world;
}

int main(int argc, char* argv[]) {
  CLI::App app{"Raytracing one weekend/week/restoflife"};
  rt::Config cfg{};
  bool quick = false;
  bool dry_run = false;

  app.option_defaults()->always_capture_default();

  app.add_option("-t,--threads", cfg.nthreads, "Number of threads to use");
  app.add_option("-w,--image-width", cfg.image_width, "Image width");
  app.add_option("-s,--samples-per-pixel", cfg.samples_per_pixel,
                 "Samples per pixel");
  app.add_option("-c,--max-child-rays", cfg.max_child_rays, "Max child rays");
  app.add_option("-a,--aspect-ratio", cfg.aspect_ratio, "Aspect ratio");
  app.add_option("-n,--balls_sqrt", cfg.number_of_balls_sqrt,
                 "Number of balls sqrt");
  app.add_option("--thrash,--thrashing-allocator-pathos",
                 rtweekend::detail::thrashing_allocator_pathos,
                 "Thrashing_allocator_pathos");
  app.add_flag("-m,--moving-spheres", cfg.moving_spheres,
                 "Moving spheres");
  app.add_flag("-q,--quick", quick, "Quickie");
  app.add_flag("--dry-run", dry_run, "Dry run");

  CLI11_PARSE(app, argc, argv);

  if (quick) {
    cfg.image_width = 200;
    cfg.number_of_balls_sqrt = 11;
    cfg.moving_spheres = true;
    cfg.samples_per_pixel = 20;
    cfg.max_child_rays = 20;
  }

  if (dry_run) {
    std::cout << cfg;
    return 0;
  }

  // Camera
  rt::Camera cam{rt::point(13,2,3),
                 rt::point(0,0,0),
                 rt::vec3(0,1,0),
                 20.0,
                 cfg.aspect_ratio,
                 0.1,
                 10.0,
                 0,
                 1};

  rt::render(lots_of_balls(cfg), cam, cfg);
}
