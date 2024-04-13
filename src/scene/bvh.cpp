#include "bvh.h"

#include "CGL/CGL.h"
#include "pathtracer/intersection.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  BBox bbox;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);
  size_t size = std::distance(start, end);

  if (size <= max_leaf_size) {
    node->start = start;
    node->end = end;
    return node;
  } else {
    // determine the longest axis
    int longest_axis = 0;

    if (bbox.extent[1] > bbox.extent[longest_axis]) {
      longest_axis = 1;
    }

    if (bbox.extent[2] > bbox.extent[longest_axis]) {
      longest_axis = 2;
    }
    // sort primitives based on centroid.longestaxis
    std::sort(start, end,
              [longest_axis](const Primitive *a, const Primitive *b) {
                return a->get_bbox().centroid()[longest_axis] <
                       b->get_bbox().centroid()[longest_axis];
              });
    std::vector<Primitive *>::iterator mid =
        start + std::distance(start, end) / 2;
    if (start != mid) {
      node->l = construct_bvh(start, next(mid), max_leaf_size);
    } else {
      node->l = NULL;
    }
    if (next(mid) != end) {
      node->r = construct_bvh(next(mid), end, max_leaf_size);
    } else {
      node->r = NULL;
    }
    return node;
  }
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)) // If the ray doesn't intersect
                                               // the bounding box, return false
  {
    return false;
  }
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray)) {

        return true;
      }
    }
    return false;
  }
  bool answer1 = has_intersection(ray, node->l);

  bool answer2 = has_intersection(ray, node->r);

  return answer1 || answer2;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!node->bb.intersect(
          ray, t0,
          t1)) // If the ray doesn't intersect the bounding box, return false
  {
    return false;
  }

  bool hit = false;
  if (node->isLeaf()) {
    Intersection *betterI = i;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      hit = (*p)->intersect(ray, i) || hit;
      if (betterI->t > i->t) {
        betterI = i;
      }
    }
    i = betterI;
    return hit;
  } else {
    bool answer1 = intersect(ray, i, node->l);
    Intersection *betterI = i;
    bool answer2 = intersect(ray, i, node->r);
    if (betterI->t < i->t) {
      i = betterI;
    }
    return answer1 || answer2;
  }
}

} // namespace SceneObjects
} // namespace CGL