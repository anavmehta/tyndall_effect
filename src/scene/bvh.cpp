#include "bvh.h"

#include "CGL/CGL.h"
#include "scene/primitive.h"

#include <algorithm>

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

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox bbox;

  // compute the bounding box of a list of primitives
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  // initialize a new BVHNode with that bounding box
  BVHNode *node = new BVHNode(bbox);
  // node->start = start;
  // node->end = end;

  // If there are no more than max_leaf_size primitives in the list, the node we just created is a leaf node and we should update its start and end iterators appropriately.
  if (end - start <= max_leaf_size) {
    node->start = start;
    node->end = end;
  }
  // Otherwise, we need to divide the primitives into a "left" and "right" collection.
  else {
    Vector3D centroid = bbox.centroid();
    std::vector<int> left(3), right(3);

    // split along all axises
    for (auto p = start; p != end; p++) {
      BBox bb = (*p)->get_bbox();
      Vector3D bbc = bb.centroid();
      for (int i = 0; i < 3; i++) {
        if (bbc[i] < centroid[i])
          left[i]++;
        else
          right[i]++;
      }
    }

    // create a heuristic that selects the axis giving us the most benefit
    auto h = [&](int i, int j){ return left[i] * right[i] < left[j] * right[j]; };
    int axis = max({0, 1, 2}, h);

    // partition the array so that elements of the left child are placed on the left
    // elements of the right child are placed on the right
    auto partition_point = std::partition(start, end, [&](Primitive* p){
      return p->get_bbox().centroid()[axis] < centroid[axis];
    });

    node->l = construct_bvh(start, partition_point, max_leaf_size);
    node->r = construct_bvh(partition_point, end, max_leaf_size);
  }

  return node;


}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  BBox bb = node->bb;
  double min_t = ray.min_t, max_t = ray.max_t;
  if (bb.intersect(ray, min_t, max_t)) {
    if (node->isLeaf()) {
      for (auto p = node->start; p != node->end; p++) {
        total_isects++;
        if ((*p)->has_intersection(ray))
          return true;
      }
      return false;
    } else {
      return has_intersection(ray, node->l) || has_intersection(ray, node->r);
    }
  }

  return false;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

  BBox bb = node->bb;
  if (bb.intersect(ray, ray.min_t, ray.max_t)) {
    if (node->isLeaf()) {
      bool hit = false;
      for (auto p = node->start; p != node->end; p++) {
        total_isects++;
        hit |= (*p)->intersect(ray, i);
      }
      return hit;
    } else {
      bool hit1 = intersect(ray, i, node->l);
      bool hit2 = intersect(ray, i, node->r);
      return hit1 || hit2;
    }
  }
  return false;
}

} // namespace SceneObjects
} // namespace CGL
