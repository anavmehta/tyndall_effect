// noise.h
#ifndef PERLIN_NOISE_H
#define PERLIN_NOISE_H

#include "CGL/CGL.h"
#include "CGL/matrix3x3.h"
#include "CGL/vector3D.h"
#include <vector>
namespace CGL {
class PerlinNoise {
public:
  PerlinNoise(); // Constructor
                 //   virtual ~PerlinNoise(); // Destructor
  float eval(Vector3D p) const;

private:
  static const int tableSize = 256;
  std::vector<Vector3D> gradients;
  std::vector<unsigned> permutationTable;
  int hash(const int &x, const int &y, const int &z) const;
  float lerp(const float &a, const float &b, const float &t) const;
  float smoothstep(const float &t) const;
};
} // namespace CGL
#endif // PERLIN_NOISE_H