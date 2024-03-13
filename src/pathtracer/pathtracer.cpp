#include "pathtracer.h"

#include "pathtracer/intersection.h"
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
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
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

  for (int i = 0; i < num_samples; i++) {
    // generate the incident direction in object space
    Vector3D wi = hemisphereSampler->get_sample();
    Intersection new_isect;

    // the ray is in world space
    Ray new_ray = Ray(hit_p, o2w * wi);
    new_ray.min_t = EPS_F;

    // the bsdf is the property of the object and is in object space
    Vector3D bsdf = isect.bsdf->f(w_out, wi);

    Vector3D L_i = (bvh->intersect(new_ray, &new_isect))
                    ? new_isect.bsdf->get_emission()
                    : envLight->sample_dir(new_ray);

    L_out += bsdf * L_i * dot(isect.n, new_ray.d); // we multiply the pdf outside of the for loop for efficiency since it is a constant
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

  Vector3D wi;
  double dist_to_light;
  double pdf;
  for (const auto &light: scene->lights) {
    if (light->is_delta_light()) {
      Vector3D light_radiance = light->sample_L(hit_p, &wi, &dist_to_light, &pdf);
      Vector3D bsdf = isect.bsdf->f(w_out, wi);
      Ray new_ray(hit_p, wi, dist_to_light - EPS_F);
      new_ray.min_t = EPS_F;
      Intersection new_isect;

      Vector3D L_i = (bvh->intersect(new_ray, &new_isect))
                                    ? new_isect.bsdf->get_emission()
                                    : light_radiance;
      L_out += bsdf * L_i * dot(isect.n, new_ray.d) / pdf;
    } else {
      Vector3D cur_L_out;
      for (int i = 0; i < ns_area_light; i++) {
        Vector3D light_radiance = light->sample_L(hit_p, &wi, &dist_to_light, &pdf);
        Vector3D bsdf = isect.bsdf->f(w_out, wi);
        Ray new_ray(hit_p, wi, dist_to_light - EPS_F);
        new_ray.min_t = EPS_F;
        Intersection new_isect;

        Vector3D L_i = (bvh->intersect(new_ray, &new_isect))
                                      ? new_isect.bsdf->get_emission()
                                      : light_radiance;
        cur_L_out += bsdf * L_i * dot(isect.n, new_ray.d) / pdf;
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
  if (direct_hemisphere_sample)
    return estimate_direct_lighting_hemisphere(r, isect);
  else
    return estimate_direct_lighting_importance(r, isect);
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


  // L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
  L_out = zero_bounce_radiance(r, isect);
  L_out += one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

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

  for (int i = 0; i < num_samples; i++) {
    // sample uniformly from the pixel
    auto sample = origin + gridSampler->get_sample();
    Ray ray = camera->generate_ray(sample.x / w, sample.y / h); // transform into image space and call generate_ray
    auto est_radiance = est_radiance_global_illumination(ray);
    overall_radiance += est_radiance;
  }

  // Refer to Lecture12:29 for the n-dimensional MC estimator
  // the width and height of the pixel are both 1
  Vector3D MC_est_radiance = overall_radiance / ns_aa;

  sampleBuffer.update_pixel(MC_est_radiance, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;


}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
