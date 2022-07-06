#include <random>

#include "utils.h"
#include "model.h"
#include "render.h"

namespace rt=rtweekend;

rt::World lots_of_balls() {
  rt::World world{};
  auto& boutique = world.boutique;

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
  return world;
}

int main(int argc, char* argv[]) {
  rt::Config cfg{};
  bool quick = argc>=2 && std::string{argv[1]} == "-q"; // NOLINT
  if (quick) {
    cfg.image_width = 300;
    cfg.samples_per_pixel = 40;
    cfg.max_child_rays = 20;
  }
  // Camera
  rt::Camera cam{rt::point(13,2,3),
                 rt::point(0,0,0),
                 rt::vec3(0,1,0),
                 20.0,
                 cfg.aspect_ratio,
                 0.1,
                 10.0};

  rt::render(lots_of_balls(), cam, cfg);
}
