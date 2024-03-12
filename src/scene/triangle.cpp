#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"
#include "vector3D.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <tuple>

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

static std::array<double, 4> Moller_Trumbore(const Ray &r, const Triangle &tri) {
  auto E1 = tri.p2 - tri.p1;
  auto E2 = tri.p3 - tri.p1;
  auto S = r.o - tri.p1;
  auto S1 = cross(r.d, E2);
  auto S2 = cross(S, E1);
  auto tmp = dot(S1, E1);
  auto t = dot(S2, E2) / tmp;
  auto b1 = dot(S1, S) / tmp;
  auto b2 = dot(S2, r.d) / tmp;
  auto b0 = 1 - b1 - b2;
  return {t, b0, b1, b2};
}

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
  auto [t, b0, b1, b2] = Moller_Trumbore(r, *this);
  assert(r.min_t >= 0);
  if (t <= r.min_t || t >= r.max_t)
    return false;
  return (min({b0, b1, b2}) >= 0) && (max({b0, b1, b2}) <= 1);
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
  auto [t, b0, b1, b2] = Moller_Trumbore(r, *this);
  assert(r.min_t >= 0);
  if (t <= r.min_t || t >= min(isect->t, r.max_t))
    return false;
  if (min({b0, b1, b2}) < 0 || max({b0, b1, b2}) > 1)
    return false;
  isect->t = t;
  isect->n = (b0 * n1 + b1 * n2 + b2 * n3).unit();
  isect->primitive = this;
  isect->bsdf = get_bsdf();

  return true;


}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
