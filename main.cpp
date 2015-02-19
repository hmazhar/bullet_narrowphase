#include <iostream>
#include <fstream>
#include <string>
using namespace std;
#include "narrowphase/data_structures.h"
#include "narrowphase/gjk_epa.h"

int main(int argc, char* argv[]) {
  real envelope = 0;

  {
    ContactManifold manifold;
    real3 p, n(0, 0, 0);
    real d = 0;
    ConvexShape shapeA, shapeB;
    shapeA.type = SPHERE;
    shapeA.A = real3(0, 1.5, 0);
    shapeA.B = real3(1, 1, 1);
    shapeA.C = real3(0);
    shapeA.R = real4(1, 0, 0, 0);

    shapeB.type = SPHERE;
    shapeB.A = real3(0, 0, 0);
    shapeB.B = real3(1, 1, 1);
    shapeB.C = real3(0);
    shapeB.R = real4(1, 0, 0, 0);
    real3 sep_axis;

    if (Collide(shapeA, shapeB, manifold, sep_axis, envelope)) {
      cout << sep_axis << endl;
      PerturbedCollide(shapeA, shapeB, manifold, envelope, sep_axis);
    }
  }

  return 0;
}
