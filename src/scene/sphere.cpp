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
    t1 = t2 = -b / (2 * a);
    return t1 >= r.min_t && t1 <= r.max_t;
  } else if (delta > 0) { // two intersections
    auto tmp = sqrt(delta);
    t1 = (-b - tmp) / (2 * a);
    t2 = (-b + tmp) / (2 * a);
    return (t1 >= r.min_t && t1 <= r.max_t) || (t2 >= r.min_t && t2 <= r.max_t);
  } else { // no intersection
    return false;
  }
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double dummy1, dummy2;
  return test(r, dummy1, dummy2);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2;
  if (test(r, t1, t2)) {
    double t = (t1 >= r.min_t && t1 <= r.max_t) ? t1 : t2;
    r.max_t = t;
    i->t = t;
    i->n = (r.o + t * r.d - this->o).unit();
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
