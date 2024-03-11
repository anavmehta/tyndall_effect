#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  auto a = dot(r.d, r.d);
  auto b = 2 * dot(r.o - this->o, r.d);
  auto c = dot(r.o - this->o, r.o - this->o) - this->r2;
  auto delta = b * b - 4 * a * c;
  if (delta == 0) { // tangent, only one intersection
    double t = -b / (2 * a);
    if (t >= t1 && t <= t2) {
      t1 = t;
      return true;
    }
    return false;
  } else if (delta > 0) { // two intersections
    double tmp = sqrt(delta);
    double low = (-b - tmp) / (2 * a);
    double high = (-b + tmp) / (2 * a);
    if (low >= t1 && low <= t2) {
      t1 = low;
      return true;
    }
    if (high >= t1 && high <= t2) {
      t1 = high;
      return true;
    }
    return false;
  } else { // no intersection
    return false;
  }
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1 = r.min_t, t2 = r.max_t;
  return test(r, t1, t2);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1 = r.min_t, t2 = i->t;
  if (test(r, t1, t2)) {
    r.max_t = t1;
    i->t = t1;
    i->n = (r.o + t1 * r.d - this->o).unit();
    i->primitive = this;
    i->bsdf = get_bsdf();
    return true;
  }


  return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
