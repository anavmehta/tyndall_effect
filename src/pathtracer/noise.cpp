#include "noise.h"
#include "util/random_util.h"
#include "vector3D.h"
#include <algorithm>
#include <cmath>
#include <random>

using namespace std;
namespace CGL {

PerlinNoise::PerlinNoise() {
  std::random_device rd;
  std::mt19937 generator(rd());

  std::uniform_real_distribution<float> distribution;
  auto gen = std::bind(distribution, generator);

  gradients.resize(tableSize);
  permutationTable.resize(2 * tableSize);
  for (unsigned i = 0; i < tableSize; ++i) {
    float theta = acos(2 * gen() - 1);
    float phi = 2 * gen() * M_PI;
    float x = cos(phi) * sin(theta), y = sin(phi) * sin(theta), z = cos(theta);
    gradients[i] = Vector3D(x, y, z);
    permutationTable[i] = i;
  }
  std::shuffle(permutationTable.begin(), permutationTable.begin() + tableSize,
               generator);
  for (int i = 0; i < tableSize; ++i) {
    permutationTable[tableSize + i] = permutationTable[i];
  }
}

/* inline */
int PerlinNoise::hash(const int &x, const int &y, const int &z) const {
  return permutationTable[permutationTable[permutationTable[x] + y] + z];
}

float PerlinNoise::lerp(const float &a, const float &b, const float &t) const {
  return a * (1 - t) + b * t;
}

float PerlinNoise::smoothstep(const float &t) const {
  return 6 * pow(t, 2) - 15 * pow(t, 4) + 10 * pow(t, 3);
}

float PerlinNoise::eval(Vector3D p) const {
  int x0 = ((int)floor(p.x)) & (tableSize - 1);
  int y0 = ((int)floor(p.y)) & (tableSize - 1);
  int z0 = ((int)floor(p.z)) & (tableSize - 1);

  int x1 = (x0 + 1) & (tableSize - 1);
  int y1 = (y0 + 1) & (tableSize - 1);
  int z1 = (z0 + 1) & (tableSize - 1);

  float tx = p.x - ((int)floor(p.x));
  float ty = p.y - ((int)floor(p.y));
  float tz = p.z - ((int)floor(p.z));

  float u = smoothstep(tx);
  float v = smoothstep(ty);
  float w = smoothstep(tz);

  // gradients at the corner of the cell
  const Vector3D &c000 = gradients[hash(x0, y0, z0)];
  const Vector3D &c100 = gradients[hash(x1, y0, z0)];
  const Vector3D &c010 = gradients[hash(x0, y1, z0)];
  const Vector3D &c110 = gradients[hash(x1, y1, z0)];

  const Vector3D &c001 = gradients[hash(x0, y0, z1)];
  const Vector3D &c101 = gradients[hash(x1, y0, z1)];
  const Vector3D &c011 = gradients[hash(x0, y1, z1)];
  const Vector3D &c111 = gradients[hash(x1, y1, z1)];

  // distances from the left/bottom/back grid point to p and right/top/front
  // grid points to p
  float left_to_p = tx;
  float bottom_to_p = ty;
  float back_to_p = tz;

  float right_to_p = tx - 1;
  float top_to_p = ty - 1;
  float front_to_p = tz - 1;

  // Distance Vectors from each grid point pointing to p
  Vector3D p000 = Vector3D(left_to_p, bottom_to_p, back_to_p);

  Vector3D p100 = Vector3D(right_to_p, bottom_to_p, back_to_p);
  Vector3D p010 = Vector3D(left_to_p, top_to_p, back_to_p);
  Vector3D p110 = Vector3D(right_to_p, top_to_p, back_to_p);

  Vector3D p001 = Vector3D(left_to_p, bottom_to_p, front_to_p);
  Vector3D p101 = Vector3D(right_to_p, bottom_to_p, front_to_p);
  Vector3D p011 = Vector3D(left_to_p, top_to_p, front_to_p);
  Vector3D p111 = Vector3D(right_to_p, top_to_p, front_to_p);

  // linear interpolation
  float a = lerp(dot(c000, p000), dot(c100, p100), u);
  float b = lerp(dot(c010, p010), dot(c110, p110), u);
  float c = lerp(dot(c001, p001), dot(c101, p101), u);
  float d = lerp(dot(c011, p011), dot(c111, p111), u);

  float e = lerp(a, b, v);
  float f = lerp(c, d, v);

  return lerp(e, f, w); // g
}
} // namespace CGL