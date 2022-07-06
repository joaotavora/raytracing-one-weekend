#include <cmath>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/exponential.hpp>
#include <glm/geometric.hpp>
#include <functional>
#include <limits>
#include <random>

#include "utils.h"
#include "model.h"


int main(int argc, char* argv[]) {
  namespace rt=rtweekend;
  constexpr double aspect_ratio = 3.0/2.0;

  // Camera
  rt::Camera cam{rt::point(13,2,3),
                 rt::point(0,0,0),
                 rt::vec3(0,1,0),
                 20.0,
                 aspect_ratio,
                 0.1,
                 10.0
                 };

  bool quick = argc>=2 && std::string{argv[1]} == "-q"; // NOLINT
  // Image
  int image_width = quick?200:1200;
  int image_height = static_cast<int>(image_width/aspect_ratio); // NOLINT
  int samples_per_pixel = quick?15:100;
  int max_child_rays = quick?5:50;

  // World of spheres
  rt::World world{};

  // A boutique of materials
  rt::Boutique boutique{};

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

  rt::Dielectric glass{1.5};
  rt::Lambertian reddish{rt::color{0.4, 0.2, 0.1}};
  rt::Metal reddish_metal{rt::color{0.7, 0.6, 0.5}};

  world.add<rt::Sphere>(rt::point(0, 1, 0), 1.0, glass);
  world.add<rt::Sphere>(rt::point(-4, 1, 0), 1.0, reddish);
  world.add<rt::Sphere>(rt::point(4, 1, 0), 1.0, reddish_metal);


  // Render
  std::vector<std::vector<rt::color>> lines(image_height,
                                            std::vector(image_width, rt::color{0,0,0}));
  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

  for (int i = 0; i < image_height; ++i) {
    auto reflect_i = image_height - i -1;
    std::cerr << "\rScanlines remaining: "
              << reflect_i << ' ' << std::flush;
    for (int j = 0; j < image_width; ++j) {
      rt::color pixel_color{0,0,0};
      for (int s = 0; s < samples_per_pixel; ++s) {
        auto u = (j + rt::random_double()) / (image_width-1);
        auto v = (reflect_i + rt::random_double()) / (image_height-1);
        auto r = cam.get_ray(u, v);
        pixel_color += rt::ray_color(r, world, max_child_rays);
      }
      lines[i][j] = pixel_color;
    }
  }
  for (const auto& line : lines) {
    for (const auto& c : line) {
      rt::write_color(std::cout, c, samples_per_pixel);
    }
  }


  std::cerr << "\nDone (also " << sizeof(rt::detail::Hit) << ").\n";
}
