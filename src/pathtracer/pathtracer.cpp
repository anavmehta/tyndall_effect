#include "pathtracer.h"

#include "pathtracer/intersection.h"
#include "pathtracer/ray.h"
#include "pathtracer/sampler.h"
#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"
#include "util/random_util.h"
#include "vector2D.h"
#include "vector3D.h"
#include <cassert>


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
#ifndef JITTERED
  gridSampler = new UniformGridSampler2D();
#else
  gridSampler = new JitteredGridSampler2D();
#endif
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
  P_absorb = 15;
  P_scatter = 0.3;
  P_transmit = P_scatter+ P_absorb; 
  P_density = 10;

}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 

  assert(r.depth > 0);

  for (int i = 0; i < num_samples; i++) {
    // generate the incident direction in object space
    Vector3D wi = hemisphereSampler->get_sample();
    Intersection new_isect;

    // the ray is in world space
    Ray new_ray = Ray(hit_p, o2w * wi, int(r.depth - 1));
    new_ray.min_t = EPS_F;

    // the bsdf is the property of the object and is in object space
    Vector3D bsdf = isect.bsdf->f(w_out, wi);

    Vector3D L_i = (bvh->intersect(new_ray, &new_isect))
                    ? new_isect.bsdf->get_emission()
                    : envLight->sample_dir(new_ray);

    L_out += bsdf * L_i * dot(isect.n, new_ray.d) * 2; // we multiply the pdf outside of the for loop for efficiency since it is a constant
  }
  L_out *= (2 * PI) / num_samples; // 1 / (2 * PI) is the pdf

  return L_out;


}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  assert(r.depth > 0);

  Vector3D wi;
  double dist_to_light;
  double pdf;
  for (const auto &light: scene->lights) {
    if (light->is_delta_light()) {
      Vector3D light_radiance = light->sample_L(hit_p, &wi, &dist_to_light, &pdf);
      Vector3D bsdf = isect.bsdf->f(w_out, wi);
      Ray new_ray(hit_p, wi, dist_to_light - EPS_F, r.depth - 1);
      new_ray.min_t = EPS_F;
      Intersection new_isect;

      Vector3D L_i = (bvh->intersect(new_ray, &new_isect))
                                    ? new_isect.bsdf->get_emission()
                                    : light_radiance;
      L_out += bsdf * L_i * dot(isect.n, new_ray.d) / pdf * 2;
    } else {
      Vector3D cur_L_out;
      for (int i = 0; i < ns_area_light; i++) {
        Vector3D light_radiance = light->sample_L(hit_p, &wi, &dist_to_light, &pdf);
        Vector3D bsdf = isect.bsdf->f(w_out, wi);
        Ray new_ray(hit_p, wi, dist_to_light - EPS_F, r.depth - 1);
        new_ray.min_t = EPS_F;
        Intersection new_isect;

        Vector3D L_i = (bvh->intersect(new_ray, &new_isect))
                                      ? new_isect.bsdf->get_emission()
                                      : light_radiance;
        cur_L_out += bsdf * L_i * dot(isect.n, new_ray.d) / pdf * 2;
      }
      cur_L_out /= ns_area_light;
      L_out += cur_L_out;
    }
  }


  return L_out;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  return hit_fog(r, isect);
  if (direct_hemisphere_sample)
    return estimate_direct_lighting_hemisphere(r, isect);
  else
    return estimate_direct_lighting_importance(r, isect);
}
Vector3D PathTracer::hit_fog(const Ray &r, const Intersection &isect){
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
   Vector3D hit_p = r.o + r.d * isect.t;
  auto epsilon = ((double) rand() / (RAND_MAX));
  auto distance_btwn_origin_fog = -log(1-epsilon)/P_density;
  auto fog_time = distance_btwn_origin_fog / r.d.norm2();
  const Vector3D fog_pos = r.o + r.d * fog_time;
  const Vector3D w_out = w2o * (-r.d);
  bool hit_fog = false;
  if (fog_time < isect.t) {
    hit_fog = true;
    hit_p = fog_pos;
  }

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 

  assert(r.depth > 0);

  for (int i = 0; i < num_samples; i++) {
    // generate the incident direction in object space
    Vector3D wi = hemisphereSampler->get_sample();
    Intersection new_isect;

    // the ray is in world space
    Ray new_ray = Ray(hit_p, o2w * wi, int(r.depth - 1));
    new_ray.min_t = EPS_F;

    // the bsdf is the property of the object and is in object space
    Vector3D bsdf = isect.bsdf->f(w_out, wi);
    Vector3D L_individual;
    if (hit_fog == false) {
    L_individual = (bvh->intersect(new_ray, &new_isect))
                    ? new_isect.bsdf->get_emission()
                    : envLight->sample_dir(new_ray);
    } else {
      L_individual = 0; 
    }

    L_out += bsdf * L_individual * dot(isect.n, new_ray.d) * 2; // we multiply the pdf outside of the for loop for efficiency since it is a constant
  }
  L_out *= (2 * PI) / num_samples; // 1 / (2 * PI) is the pdf

  return L_out;


}
Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  

  if (r.depth == 1)
    return one_bounce_radiance(r, isect);

  // K^{depth + 1}(L_e)
  if (isAccumBounces)
    L_out = one_bounce_radiance(r, isect);

  // generate a ray and calculate the radiance
  Vector3D wi;
  double pdf;
  Vector3D bsdf = isect.bsdf->sample_f(w_out, &wi, &pdf);
  Ray new_ray(hit_p, o2w * wi, int(r.depth - 1));
  new_ray.min_t = EPS_F;

  Intersection new_isect;
  // Task 2
  // if (bvh->intersect(new_ray, &new_isect)) {
  //   Vector3D L_i = at_least_one_bounce_radiance(new_ray, new_isect);
  //   L_out += bsdf * L_i * dot(isect.n, new_ray.d) / pdf;
  // }

  // Task 3
  static const double p_rr = 0.3; // probability of russian roulette
  if (bvh->intersect(new_ray, &new_isect)) {
    if (r.depth == max_ray_depth || r.depth == max_ray_depth - 1) {
      Vector3D L_i = at_least_one_bounce_radiance(new_ray, new_isect) * exp(-P_absorb *new_isect.t);
      // std::cout << new_isect.t << endl;
      L_out += bsdf * L_i * dot(isect.n, new_ray.d) / pdf * 2;
    } else if (coin_flip(p_rr)) {
      Vector3D L_i = at_least_one_bounce_radiance(new_ray, new_isect);
      L_out += bsdf * L_i * dot(isect.n, new_ray.d) / pdf / p_rr * 2;
    }
  }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  if (PART <= 2)
    L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
  else if (PART == 3)
    L_out = zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
  else {
    if (isAccumBounces || max_ray_depth == 0)
      L_out = zero_bounce_radiance(r, isect);

    if (max_ray_depth > 0)
      L_out += at_least_one_bounce_radiance(r, isect);
  }

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Vector3D overall_radiance;
  auto w = sampleBuffer.w, h = sampleBuffer.h; // the width and height of the camera space

  // Part 1.2
  if (PART <= 4) { 
    auto &&samples = gridSampler->get_samples_batch(num_samples);
    for (int i = 0; i < num_samples; i++) {
      // sample uniformly from the pixel
      // auto sample = origin + gridSampler->get_sample();
      auto sample = origin + samples[i];
      Ray ray = camera->generate_ray(sample.x / w, sample.y / h); // transform into image space and call generate_ray
      ray.depth = max_ray_depth;
      auto est_radiance = est_radiance_global_illumination(ray);
      overall_radiance += est_radiance;
    }

    // Refer to Lecture12:29 for the n-dimensional MC estimator
    // the width and height of the pixel are both 1
    Vector3D MC_est_radiance = overall_radiance / num_samples;

    sampleBuffer.update_pixel(MC_est_radiance, x, y);
    sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
  } 
  // Part 5
  else {
    int cur_samples = 0;
    double s1 = 0, s2 = 0;
    while (cur_samples < num_samples) {
      cur_samples += samplesPerBatch;
      auto &&samples = gridSampler->get_samples_batch(samplesPerBatch);
      for (int i = 0; i < samplesPerBatch; i++) {
        // auto sample = origin + gridSampler->get_sample();
        auto sample = origin + samples[i];
        Ray ray = camera->generate_ray(sample.x / w, sample.y / h); // transform into image space and call generate_ray
        ray.depth = max_ray_depth;
        auto est_radiance = est_radiance_global_illumination(ray);
        overall_radiance += est_radiance;
        auto illum = est_radiance.illum();
        s1 += illum;
        s2 += illum * illum;
      }
      auto mean = s1 / cur_samples;
      auto variance = (s2 - s1 * s1 / cur_samples) / (cur_samples - 1);
      if (1.96 * sqrt(variance) / sqrt(cur_samples) <= maxTolerance * mean)
        break;
    }
    Vector3D MC_est_radiance = overall_radiance / cur_samples;

    sampleBuffer.update_pixel(MC_est_radiance, x, y);
    sampleCountBuffer[x + y * sampleBuffer.w] = cur_samples;
  }



}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
