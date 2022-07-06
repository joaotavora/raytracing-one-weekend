#include <chrono>
#include <iostream>
#include <glm/glm.hpp>
#include <functional>
#include <random>
#include <future>

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
  int image_width = quick?300:1200;
  int image_height = static_cast<int>(image_width/aspect_ratio); // NOLINT
  int samples_per_pixel = quick?40:100;
  int max_child_rays = quick?15:50;

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
  int nthreads = 4;
  namespace khr = std::chrono;
  std::cerr << "Started rendering with " << nthreads << " threads\n";
  khr::time_point start{khr::high_resolution_clock::now()};

  using Image = std::vector<rt::color>;
  Image global_image(image_width * image_height, rt::color{0,0,0});

  auto do_work = [&](int samples) {
    Image local_image(image_width * image_height, rt::color{0,0,0});
    for (int i = 0; i < image_height; ++i) {
      auto from_top_i = image_height - i -1;
      std::cerr << "\rScanlines remaining: "
                << from_top_i << ' ' << std::flush;
      for (int j = 0; j < image_width; ++j) {
        rt::color pixel_color{0,0,0};
        for (int s = 0; s < samples; ++s) {
          auto u = (j + rt::random_double()) / (image_width-1);
          auto v = (from_top_i + rt::random_double()) / (image_height-1);
          auto r = cam.get_ray(u, v);
          pixel_color += rt::ray_color(r, world, max_child_rays);
        }
        local_image[i*image_width + j] = pixel_color;
      }
    }
    return local_image;
  };

  std::vector<std::future<Image>> work{};
  work.reserve(nthreads);

  for (int i = 0; i < nthreads; ++i) {
    work.push_back(std::async(std::launch::async,
                              do_work,
                              samples_per_pixel/nthreads));
  }
  for (auto& f : work) {
    const auto& done = f.get();
    std::transform(done.cbegin(), done.cend(),
                   global_image.cbegin(), global_image.begin(),
                   std::plus<rt::color>());
  }

  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";
  for (const auto& c : global_image) {
    rt::write_color(std::cout, c, samples_per_pixel/nthreads * nthreads);
  }

  auto took = khr::high_resolution_clock::now() - start;
  std::cerr << "\nDone in "
            << khr::duration_cast<khr::milliseconds>(took) << "\n";

}
